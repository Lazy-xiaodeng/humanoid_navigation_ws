#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_waypoint_pose_validation.py

文件作用：
  1. 这是点位 JSON 专项离线验证脚本，不属于线上功能源码。
  2. 读取用户点位 JSON，例如 data/dynamic_waypoints.json 或 data/waypoints/hall.json。
  3. 在真实 bag 的 /robot_realpose 轨迹中，为每个点位寻找最近经过的点云帧。
  4. 生成 body/registered_world 两路 evaluator YAML，验证点位附近冷启动和定位丢失恢复是否能找回。
  5. 输出每个点位的匹配距离、单帧重定位结果、temporal decision 和资源统计，便于定位薄弱区域。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_waypoint_pose_validation.py \
    --waypoints data/dynamic_waypoints.json --run
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml


@dataclass(frozen=True)
class Waypoint:
    """从点位 JSON 中解析出的单个导航点。"""

    waypoint_id: str
    name: str
    category: str
    x: float
    y: float
    yaw_deg: float


@dataclass(frozen=True)
class ReferenceSample:
    """bag 中一条 /robot_realpose 样本。"""

    bag_time_sec: float
    x: float
    y: float
    yaw_deg: float


@dataclass(frozen=True)
class CloudSample:
    """bag 中一条点云帧索引和时间。"""

    index: int
    bag_time_sec: float
    stamp_sec: float


@dataclass(frozen=True)
class MatchedWaypoint:
    """点位与 bag 中最近真实经过帧的匹配结果。"""

    waypoint: Waypoint
    cloud_index: int
    cloud_stamp_sec: float
    reference_x: float
    reference_y: float
    reference_yaw_deg: float
    distance_m: float
    yaw_delta_deg: float


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def yaw_from_quaternion(q: list[float]) -> float:
    """把 JSON 四元数 [x,y,z,w] 转成 yaw，单位 rad。"""
    if len(q) != 4:
        return 0.0
    x, y, z, w = (float(v) for v in q)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle_deg(angle: float) -> float:
    """把角度差规整到 [-180, 180]，便于比较点位朝向和真实朝向。"""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def pose_yaw_deg(pose: Any) -> float:
    """从 ROS Pose orientation 中读取 yaw，单位 deg。"""
    q = pose.orientation
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def stamp_to_sec(stamp: Any) -> float:
    """ROS builtin_interfaces/Time 转秒。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    """打开 rosbag2 reader；storage_id 留空让 rosbag2 根据 metadata 自动判断 mcap/sqlite。"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def load_waypoints(path: Path) -> list[Waypoint]:
    """解析点位 JSON，支持当前 data/waypoints 和 data/dynamic_waypoints 两种结构。"""
    data = json.loads(path.read_text(encoding="utf-8"))
    waypoints: list[Waypoint] = []
    root = data.get("waypoints", {})
    if not isinstance(root, dict):
        return waypoints
    for category, entries in root.items():
        if not isinstance(entries, dict):
            continue
        for waypoint_id, item in entries.items():
            if not isinstance(item, dict) or not isinstance(item.get("position"), list):
                continue
            position = item["position"]
            if len(position) < 2:
                continue
            orientation = item.get("orientation", [0.0, 0.0, 0.0, 1.0])
            waypoints.append(
                Waypoint(
                    waypoint_id=str(item.get("id", waypoint_id)),
                    name=str(item.get("name", waypoint_id)),
                    category=str(item.get("type", category)),
                    x=float(position[0]),
                    y=float(position[1]),
                    yaw_deg=math.degrees(yaw_from_quaternion(orientation)),
                )
            )
    return waypoints


def nearest_reference(references: list[ReferenceSample], bag_time_sec: float, tolerance_sec: float) -> ReferenceSample | None:
    """按 bag receive time 给点云帧找最近 /robot_realpose。"""
    best: ReferenceSample | None = None
    best_dt = float("inf")
    for ref in references:
        dt = abs(ref.bag_time_sec - bag_time_sec)
        if dt < best_dt:
            best = ref
            best_dt = dt
    if best is None or best_dt > tolerance_sec:
        return None
    return best


def read_bag_reference_clouds(
    bag_path: Path,
    cloud_topic: str,
    reference_topic: str,
    reference_tolerance: float,
) -> tuple[list[ReferenceSample], list[tuple[CloudSample, ReferenceSample]]]:
    """读取 bag 中参考轨迹和指定点云话题，并给每个点云帧附上最近真值。"""
    cloud_type = get_message("sensor_msgs/msg/PointCloud2")
    reference_type = get_message("geometry_msgs/msg/PoseWithCovarianceStamped")

    references: list[ReferenceSample] = []
    raw_clouds: list[CloudSample] = []
    reader = open_reader(bag_path)
    cloud_index = -1
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        bag_time_sec = float(bag_time_ns) * 1e-9
        if topic == reference_topic:
            msg = deserialize_message(data, reference_type)
            pose = msg.pose.pose
            references.append(
                ReferenceSample(
                    bag_time_sec=bag_time_sec,
                    x=float(pose.position.x),
                    y=float(pose.position.y),
                    yaw_deg=pose_yaw_deg(pose),
                )
            )
            continue
        if topic != cloud_topic:
            continue
        cloud_index += 1
        msg = deserialize_message(data, cloud_type)
        raw_clouds.append(CloudSample(cloud_index, bag_time_sec, stamp_to_sec(msg.header.stamp)))

    paired: list[tuple[CloudSample, ReferenceSample]] = []
    for cloud in raw_clouds:
        ref = nearest_reference(references, cloud.bag_time_sec, reference_tolerance)
        if ref is not None:
            paired.append((cloud, ref))
    return references, paired


def match_waypoints_to_clouds(
    waypoints: list[Waypoint],
    clouds: list[tuple[CloudSample, ReferenceSample]],
    max_distance_m: float,
    allow_duplicate_cloud_indices: bool,
) -> list[MatchedWaypoint]:
    """为每个点位选择距离最近的真实经过帧，超过 max_distance_m 的点位会被跳过。"""
    matches: list[MatchedWaypoint] = []
    used_indices: set[int] = set()
    for waypoint in waypoints:
        best: tuple[CloudSample, ReferenceSample] | None = None
        best_dist = float("inf")
        for cloud, ref in clouds:
            dist = math.hypot(ref.x - waypoint.x, ref.y - waypoint.y)
            if dist < best_dist:
                best = (cloud, ref)
                best_dist = dist
        if best is None or best_dist > max_distance_m:
            continue
        cloud, ref = best
        if not allow_duplicate_cloud_indices and cloud.index in used_indices:
            # 多个很近点位可能落在同一帧。保留第一个点位，避免 evaluator 重复同一帧。
            continue
        used_indices.add(cloud.index)
        matches.append(
            MatchedWaypoint(
                waypoint=waypoint,
                cloud_index=cloud.index,
                cloud_stamp_sec=cloud.stamp_sec,
                reference_x=ref.x,
                reference_y=ref.y,
                reference_yaw_deg=ref.yaw_deg,
                distance_m=best_dist,
                yaw_delta_deg=abs(normalize_angle_deg(ref.yaw_deg - waypoint.yaw_deg)),
            )
        )
    return matches


def load_template(path: Path) -> dict[str, Any]:
    """读取主 YAML 模板。"""
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def params(config: dict[str, Any]) -> dict[str, Any]:
    """取得 ROS 参数字典。"""
    return config["global_relocalization_eval"]["ros__parameters"]


def write_config(path: Path, config: dict[str, Any], note: str) -> None:
    """写出点位验证自动生成 YAML。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "# waypoint pose validation 自动生成配置。",
        "#",
        "# 作用：",
        "#   - 由 test/run_waypoint_pose_validation.py 根据点位 JSON 和 bag 真值轨迹生成。",
        "#   - 用点位附近真实点云帧验证冷启动/定位丢失恢复能力。",
        "#   - 该文件位于 .codex_tmp，不作为线上运行配置。",
        f"#   - {note}",
        "",
    ]
    path.write_text("\n".join(header) + yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")


def build_eval_config(
    template: dict[str, Any],
    mode: str,
    bag_path: Path,
    output_dir: Path,
    sample_indices: list[int],
) -> dict[str, Any]:
    """基于主 YAML 构造点位验证 evaluator 配置。"""
    config = yaml.safe_load(yaml.safe_dump(template, allow_unicode=True, sort_keys=False))
    p = params(config)
    p["input_mode"] = mode
    p["bag_paths"] = [str(bag_path)]
    p["bag_sample_frame_indices"] = sample_indices
    p["max_bag_frames"] = len(sample_indices)
    p["bag_start_frame_skip"] = 0
    p["bag_frame_stride"] = 1
    p["odom_time_tolerance_sec"] = 0.10
    p["use_bag_reference_pose"] = True
    p["reference_time_tolerance_sec"] = 0.05
    p["save_aligned_cloud"] = False
    p["output_dir"] = str(output_dir)
    p["scan_leaf_size"] = 0.30
    p["bbs_num_threads"] = 2
    p["max_refine_candidates"] = 8
    p["refine_method"] = "gicp"
    p["refine_methods_for_sweep"] = ["gicp"]
    p["enable_temporal_consistency"] = True
    p["temporal_consistency_window_before"] = 4
    p["temporal_consistency_window_after"] = 0
    p["temporal_consistency_online_min_support_frames"] = 2
    p["temporal_consistency_online_max_refine_fitness"] = 0.12
    return config


def run_offline_eval(workspace: Path, config_path: Path) -> int:
    """调用 C++ offline evaluator。"""
    command = [
        "ros2",
        "run",
        "humanoid_global_relocalization_runtime",
        "global_relocalization_offline_eval",
        "--config",
        str(config_path),
    ]
    process = subprocess.run(
        command,
        cwd=str(workspace),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(process.stdout, end="")
    if process.stderr:
        print(process.stderr, end="", file=sys.stderr)
    return process.returncode


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV 文件。"""
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def to_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """安全读取浮点字段。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def key_from_stamp(stamp_sec: float) -> str:
    """统一 stamp key 精度。"""
    return f"{stamp_sec:.6f}"


def write_matches_csv(path: Path, matches: list[MatchedWaypoint]) -> None:
    """写出点位到 bag 帧的匹配表。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "waypoint_id",
                "name",
                "category",
                "waypoint_x",
                "waypoint_y",
                "waypoint_yaw_deg",
                "cloud_index",
                "cloud_stamp_sec",
                "reference_x",
                "reference_y",
                "reference_yaw_deg",
                "waypoint_reference_distance_m",
                "waypoint_reference_yaw_delta_deg",
            ],
        )
        writer.writeheader()
        for match in matches:
            writer.writerow(
                {
                    "waypoint_id": match.waypoint.waypoint_id,
                    "name": match.waypoint.name,
                    "category": match.waypoint.category,
                    "waypoint_x": f"{match.waypoint.x:.6f}",
                    "waypoint_y": f"{match.waypoint.y:.6f}",
                    "waypoint_yaw_deg": f"{match.waypoint.yaw_deg:.6f}",
                    "cloud_index": str(match.cloud_index),
                    "cloud_stamp_sec": f"{match.cloud_stamp_sec:.6f}",
                    "reference_x": f"{match.reference_x:.6f}",
                    "reference_y": f"{match.reference_y:.6f}",
                    "reference_yaw_deg": f"{match.reference_yaw_deg:.6f}",
                    "waypoint_reference_distance_m": f"{match.distance_m:.6f}",
                    "waypoint_reference_yaw_delta_deg": f"{match.yaw_delta_deg:.6f}",
                }
            )


def summarize_mode(label: str, output_dir: Path, matches: list[MatchedWaypoint]) -> list[dict[str, str]]:
    """把 evaluator 输出映射回每个点位，生成冷启动/丢失恢复明细。"""
    metrics = [
        row for row in read_csv(output_dir / "global_relocalization_metrics.csv")
        if row.get("scenario_name") == "arbitrary_start_no_prior"
    ]
    decisions = [
        row for row in read_csv(output_dir / "global_relocalization_temporal_decisions.csv")
        if row.get("scenario_name") == "arbitrary_start_no_prior"
    ]
    metric_by_stamp = {key_from_stamp(to_float(row, "stamp_sec", 0.0)): row for row in metrics}
    decision_by_stamp = {key_from_stamp(to_float(row, "stamp_sec", 0.0)): row for row in decisions}

    rows: list[dict[str, str]] = []
    for match in matches:
        stamp_key = key_from_stamp(match.cloud_stamp_sec)
        metric = metric_by_stamp.get(stamp_key, {})
        decision = decision_by_stamp.get(stamp_key, {})
        rows.append(
            {
                "label": label,
                "waypoint_id": match.waypoint.waypoint_id,
                "name": match.waypoint.name,
                "category": match.waypoint.category,
                "cloud_index": str(match.cloud_index),
                "stamp_sec": f"{match.cloud_stamp_sec:.6f}",
                "waypoint_x": f"{match.waypoint.x:.3f}",
                "waypoint_y": f"{match.waypoint.y:.3f}",
                "reference_x": f"{match.reference_x:.3f}",
                "reference_y": f"{match.reference_y:.3f}",
                "distance_to_waypoint_m": f"{match.distance_m:.3f}",
                "cold_start_localized": metric.get("localized", ""),
                "cold_start_success": metric.get("success", ""),
                "cold_start_trans_err_m": metric.get("translation_error_m", ""),
                "cold_start_yaw_err_deg": metric.get("yaw_error_deg", ""),
                "recovery_decision": decision.get("decision", ""),
                "recovery_reason": decision.get("decision_reason", ""),
                "selected_support_frames": decision.get("selected_support_frames", ""),
                "refined_fitness": decision.get("refined_fitness", ""),
                "recovery_success": decision.get("refined_success", ""),
                "recovery_trans_err_m": decision.get("refined_translation_error_m", ""),
                "recovery_yaw_err_deg": decision.get("refined_yaw_error_deg", ""),
            }
        )
    return rows


def write_results_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """写出点位验证结果 CSV。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "label",
        "waypoint_id",
        "name",
        "category",
        "cloud_index",
        "stamp_sec",
        "waypoint_x",
        "waypoint_y",
        "reference_x",
        "reference_y",
        "distance_to_waypoint_m",
        "cold_start_localized",
        "cold_start_success",
        "cold_start_trans_err_m",
        "cold_start_yaw_err_deg",
        "recovery_decision",
        "recovery_reason",
        "selected_support_frames",
        "refined_fitness",
        "recovery_success",
        "recovery_trans_err_m",
        "recovery_yaw_err_deg",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows: list[dict[str, str]]) -> None:
    """终端打印每路点位结果摘要。"""
    labels = sorted({row["label"] for row in rows})
    for label in labels:
        subset = [row for row in rows if row["label"] == label]
        cold_success = sum(row["cold_start_success"] == "1" for row in subset)
        accepted = [row for row in subset if row["recovery_decision"] == "accept"]
        accepted_success = sum(row["recovery_success"] == "1" for row in accepted)
        false_accept = sum(row["recovery_decision"] == "accept" and row["recovery_success"] != "1" for row in subset)
        print(
            "[waypoint_validation] "
            f"{label} points={len(subset)} "
            f"cold_start_success={cold_success}/{len(subset)} "
            f"recovery_accept={len(accepted)} recovery_success={accepted_success}/{len(accepted)} "
            f"false_accept={false_accept}"
        )
        for row in subset:
            if row["cold_start_success"] != "1" or row["recovery_decision"] != "accept" or row["recovery_success"] != "1":
                print(
                    "  issue "
                    f"id={row['waypoint_id']} name={row['name']} "
                    f"wp=({row['waypoint_x']},{row['waypoint_y']}) "
                    f"ref=({row['reference_x']},{row['reference_y']}) "
                    f"cold={row['cold_start_success']} err={row['cold_start_trans_err_m']}/{row['cold_start_yaw_err_deg']} "
                    f"decision={row['recovery_decision']} reason={row['recovery_reason']} "
                    f"support={row['selected_support_frames']} fitness={row['refined_fitness']}"
                )


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate global relocalization at waypoint JSON positions.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--waypoints", type=Path, default=Path("data/dynamic_waypoints.json"), help="点位 JSON 文件")
    parser.add_argument("--bag", type=Path, default=Path("/home/ubuntu/nav_drift_test/nav_drift_test46"), help="验证 bag")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(".codex_tmp/waypoint_pose_validation"),
        help="输出目录；批量验证多个 bag 时建议为每个 bag 单独指定",
    )
    parser.add_argument("--template", type=Path, default=None, help="YAML 模板")
    parser.add_argument("--max-distance", type=float, default=1.0, help="点位与 bag 真值最近距离超过该值则不纳入验证")
    parser.add_argument("--reference-tolerance", type=float, default=0.05, help="点云和 /robot_realpose bag time 同步容差")
    parser.add_argument(
        "--match-cloud-topic",
        default="/cloud_registered_body",
        help="用于点位匹配和 frame index 计数的点云话题；老 bag 没有 body 话题时可设为 /fast_lio/cloud_registered",
    )
    parser.add_argument(
        "--allow-duplicate-cloud-indices",
        action="store_true",
        help="允许多个距离很近的点位共用同一个最近点云帧；用于逐点位出表时保留全部点位。",
    )
    parser.add_argument("--mode", choices=["both", "body", "registered_world"], default="both", help="输入模式")
    parser.add_argument("--run", action="store_true", help="生成 YAML 后立即运行 evaluator")
    parser.add_argument("--summarize-existing", action="store_true", help="只汇总已有产物")
    parser.add_argument("--keep-existing", action="store_true", help="运行前不删除已有 output_dir")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    waypoint_path = (workspace / args.waypoints).resolve() if not args.waypoints.is_absolute() else args.waypoints
    bag_path = args.bag.resolve()
    template_path = args.template or workspace / "src/humanoid_global_relocalization_runtime/config/global_relocalization_eval.yaml"
    template = load_template(template_path)
    modes = ["body", "registered_world"] if args.mode == "both" else [args.mode]

    waypoints = load_waypoints(waypoint_path)
    if not waypoints:
        print(f"[waypoint_validation] no waypoints parsed from {waypoint_path}", file=sys.stderr)
        return 2

    rclpy.init()
    _, body_clouds = read_bag_reference_clouds(
        bag_path,
        args.match_cloud_topic,
        "/robot_realpose",
        args.reference_tolerance,
    )
    rclpy.shutdown()

    matches = match_waypoints_to_clouds(
        waypoints,
        body_clouds,
        args.max_distance,
        args.allow_duplicate_cloud_indices,
    )
    if not matches:
        print(
            f"[waypoint_validation] no waypoint has bag coverage within {args.max_distance:.2f}m",
            file=sys.stderr,
        )
        return 2

    output_root = (workspace / args.output_root).resolve() if not args.output_root.is_absolute() else args.output_root
    waypoint_stem = waypoint_path.stem
    matches_path = output_root / f"{waypoint_stem}_matches.csv"
    write_matches_csv(matches_path, matches)
    print(
        f"[waypoint_validation] waypoints={len(waypoints)} matched={len(matches)} "
        f"matches={matches_path}"
    )

    result_rows: list[dict[str, str]] = []
    sample_indices = [match.cloud_index for match in matches]
    config_root = output_root / "configs"
    for mode in modes:
        label = f"{waypoint_stem}_{mode}"
        output_dir = output_root / label
        if args.run and not args.keep_existing and output_dir.exists():
            shutil.rmtree(output_dir)
        config = build_eval_config(template, mode, bag_path, output_dir, sample_indices)
        config_path = config_root / f"{label}.yaml"
        write_config(config_path, config, f"waypoints={waypoint_path}, mode={mode}, samples={len(sample_indices)}")
        print(f"[waypoint_validation] generated {config_path}")
        if args.run:
            rc = run_offline_eval(workspace, config_path)
            if rc != 0:
                return rc
        if args.run or args.summarize_existing:
            result_rows.extend(summarize_mode(label, output_dir, matches))

    if args.run or args.summarize_existing:
        results_path = output_root / f"{waypoint_stem}_results.csv"
        write_results_csv(results_path, result_rows)
        print(f"[waypoint_validation] wrote {results_path}")
        print_summary(result_rows)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
