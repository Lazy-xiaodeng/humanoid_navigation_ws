#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_accumulated_waypoint_sweep.py

文件作用：
  1. 这是多帧累积 scan 专项验证脚本，不属于线上功能源码。
  2. 针对点位验证中失败的目标帧，一次性从 bag 中读取所需点云、odom 和参考位姿。
  3. 分别生成 3/5/7 帧累积 scan PCD，并调用 C++ offline evaluator 验证是否能救回。
  4. 汇总每个点位、每个累积窗口的成功率、误差、耗时、CPU 和内存，用于决定是否值得接入恢复模式。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_accumulated_waypoint_sweep.py --run
"""

from __future__ import annotations

import argparse
import csv
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import sensor_msgs_py.point_cloud2 as pc2


@dataclass(frozen=True)
class Target:
    """一个需要验证多帧累积的点位目标。"""

    waypoint_id: str
    name: str
    cloud_index: int


@dataclass(frozen=True)
class OdomSample:
    """bag 中一条 /odom 样本。"""

    stamp_sec: float
    msg: Any


@dataclass(frozen=True)
class ReferenceSample:
    """bag 中一条 /robot_realpose 样本。"""

    bag_time_sec: float
    msg: Any


@dataclass(frozen=True)
class CloudSample:
    """bag 中一条点云样本。"""

    index: int
    stamp_sec: float
    bag_time_sec: float
    msg: Any


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def stamp_to_sec(stamp: Any) -> float:
    """ROS builtin_interfaces/Time 转秒。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_to_rotation_matrix(q_msg: Any) -> np.ndarray:
    """把 ROS 四元数转成 3x3 旋转矩阵。"""
    x = float(q_msg.x)
    y = float(q_msg.y)
    z = float(q_msg.z)
    w = float(q_msg.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_from_quaternion(q_msg: Any) -> float:
    """从 ROS 四元数读取 yaw，单位 rad。"""
    rotation = quaternion_to_rotation_matrix(q_msg)
    return math.atan2(rotation[1, 0], rotation[0, 0])


def raw_body_to_base(points_raw: np.ndarray) -> np.ndarray:
    """Fast-LIO raw body 轴转换到 ROS base_footprint 标准轴。"""
    points_base = np.empty_like(points_raw)
    points_base[:, 0] = -points_raw[:, 2] + 0.072
    points_base[:, 1] = points_raw[:, 0] - 0.004
    points_base[:, 2] = -points_raw[:, 1] + 1.215
    return points_base


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    """打开 rosbag2 reader。"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def nearest_odom(odoms: list[OdomSample], stamp_sec: float, tolerance_sec: float) -> OdomSample:
    """按 header stamp 找最近 odom。"""
    best = min(odoms, key=lambda odom: abs(odom.stamp_sec - stamp_sec), default=None)
    if best is None or abs(best.stamp_sec - stamp_sec) > tolerance_sec:
        raise RuntimeError(f"no odom within {tolerance_sec}s for stamp {stamp_sec}")
    return best


def nearest_reference(references: list[ReferenceSample], bag_time_sec: float, tolerance_sec: float) -> ReferenceSample:
    """按 bag receive time 找最近 /robot_realpose。"""
    best = min(references, key=lambda ref: abs(ref.bag_time_sec - bag_time_sec), default=None)
    if best is None or abs(best.bag_time_sec - bag_time_sec) > tolerance_sec:
        raise RuntimeError(f"no reference within {tolerance_sec}s for bag time {bag_time_sec}")
    return best


def write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    """写 ASCII PCD，便于 C++ evaluator 直接读取。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    points = points.astype(np.float32)
    with path.open("w", encoding="ascii") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def write_eval_yaml(path: Path, pcd_path: Path, output_dir: Path, reference_pose: Any) -> None:
    """写单个累积 scan 的 evaluator YAML。"""
    pose = reference_pose.pose.pose
    yaw = yaw_from_quaternion(pose.orientation)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"""# 自动生成的多帧累积 scan 验证配置。
# 由 run_accumulated_waypoint_sweep.py 生成，仅用于离线实验。

global_relocalization_eval:
  ros__parameters:
    input_mode: "registered_world"
    registered_world_topic: "/fast_lio/cloud_registered"
    body_topic: "/cloud_registered_body"
    odom_topic: "/odom"
    single_scan_pcd_path: "{pcd_path}"
    bag_paths: []
    map_path: ""
    map_dir: "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/pcd"
    map_file_name: "hall_open3d_grounded.pcd"
    map_candidates: ["hall_open3d_grounded.pcd"]
    output_dir: "{output_dir}"

    convert_raw_body_to_base: true
    raw_body_to_base_xyz: [0.072, -0.004, 1.215]
    force_2d_output: true

    map_leaf_size: 0.25
    scan_leaf_size: 0.30
    min_scan_range: 0.80
    max_scan_range: 20.0
    enable_map_z_crop: true
    map_min_z: 0.2
    map_max_z: 2.5
    enable_scan_z_crop: true
    scan_min_z: 0.2
    scan_max_z: 2.5

    bbs_min_level_res: 0.75
    bbs_max_level: 5
    bbs_voxel_expansion_rate: 2.0
    bbs_num_threads: 2
    bbs_score_threshold_percentage: 0.12
    bbs_timeout_msec: 0
    search_min_xyz: [-35.0, -35.0, -1.0]
    search_max_xyz: [ 35.0,  35.0,  2.0]
    search_min_rpy: [-0.05, -0.05, 0.0]
    search_max_rpy: [0.05, 0.05, 6.283185307]
    top_k: 30

    refine_method: "gicp"
    refine_methods_for_sweep: ["gicp"]
    refine_top_k: true
    max_refine_candidates: 8
    refine_max_iterations: 25
    refine_max_correspondence_distance: 1.5
    refine_transformation_epsilon: 0.001
    refine_euclidean_fitness_epsilon: 0.001
    refine_with_gicp: true
    gicp_max_iterations: 25
    gicp_max_correspondence_distance: 1.5
    gicp_transformation_epsilon: 0.001

    reference_pose_xyzrpy: [{float(pose.position.x):.9f}, {float(pose.position.y):.9f}, {float(pose.position.z):.9f}, 0.0, 0.0, {yaw:.9f}]
    use_bag_reference_pose: false
    reference_pose_topic: "/robot_realpose"
    reference_time_tolerance_sec: 0.05
    enable_reference_sanity_check: true
    reference_max_abs_xy_m: 100.0
    reference_max_abs_z_m: 10.0
    success_translation_thresh: 0.80
    success_yaw_thresh_deg: 15.0

    save_aligned_cloud: false
    metrics_csv_name: "global_relocalization_metrics.csv"
    candidates_csv_name: "global_relocalization_candidates.csv"
    summary_csv_name: "global_relocalization_summary.csv"
    enable_temporal_consistency: false

    simulated_relocalization_cases:
      - name: "arbitrary_start_no_prior"
        offset_xyzrpy: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
""",
        encoding="utf-8",
    )


def load_targets_from_matches(path: Path, ids: set[str]) -> list[Target]:
    """从点位匹配 CSV 读取目标 cloud index。"""
    targets: list[Target] = []
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["waypoint_id"] in ids:
                targets.append(Target(row["waypoint_id"], row["name"], int(row["cloud_index"])))
    return targets


def read_required_bag_data(
    bag_path: Path,
    targets: list[Target],
    max_radius: int,
) -> tuple[list[OdomSample], list[ReferenceSample], dict[int, CloudSample]]:
    """一次扫 bag，读取所有目标窗口需要的点云，以及全量 odom/reference。"""
    point_cloud_type = get_message("sensor_msgs/msg/PointCloud2")
    odom_type = get_message("nav_msgs/msg/Odometry")
    reference_type = get_message("geometry_msgs/msg/PoseWithCovarianceStamped")
    wanted_indices: set[int] = set()
    for target in targets:
        wanted_indices.update(range(target.cloud_index - max_radius, target.cloud_index + max_radius + 1))

    odoms: list[OdomSample] = []
    references: list[ReferenceSample] = []
    clouds: dict[int, CloudSample] = {}
    reader = open_reader(bag_path)
    cloud_index = -1
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        bag_time_sec = float(bag_time_ns) * 1e-9
        if topic == "/odom":
            msg = deserialize_message(data, odom_type)
            odoms.append(OdomSample(stamp_to_sec(msg.header.stamp), msg))
            continue
        if topic == "/robot_realpose":
            msg = deserialize_message(data, reference_type)
            references.append(ReferenceSample(bag_time_sec, msg))
            continue
        if topic != "/fast_lio/cloud_registered":
            continue
        cloud_index += 1
        if cloud_index in wanted_indices:
            msg = deserialize_message(data, point_cloud_type)
            clouds[cloud_index] = CloudSample(cloud_index, stamp_to_sec(msg.header.stamp), bag_time_sec, msg)
    return odoms, references, clouds


def make_accumulated_case(
    target: Target,
    radius: int,
    odoms: list[OdomSample],
    references: list[ReferenceSample],
    clouds: dict[int, CloudSample],
    output_root: Path,
    odom_tolerance: float,
    reference_tolerance: float,
) -> tuple[Path, Path, int]:
    """生成单个目标/窗口的累积 PCD 和 evaluator YAML。"""
    target_cloud = clouds[target.cloud_index]
    target_odom = nearest_odom(odoms, target_cloud.stamp_sec, odom_tolerance)
    target_reference = nearest_reference(references, target_cloud.bag_time_sec, reference_tolerance)
    rotation_world_target = quaternion_to_rotation_matrix(target_odom.msg.pose.pose.orientation)
    translation_world_target = np.array(
        [
            float(target_odom.msg.pose.pose.position.x),
            float(target_odom.msg.pose.pose.position.y),
            float(target_odom.msg.pose.pose.position.z),
        ],
        dtype=np.float64,
    )

    accumulated_raw_body = []
    for index in range(target.cloud_index - radius, target.cloud_index + radius + 1):
        cloud = clouds[index]
        points_world = pc2.read_points_numpy(cloud.msg, field_names=["x", "y", "z"], skip_nans=True).astype(np.float64)
        points_target_body = (points_world - translation_world_target) @ rotation_world_target
        accumulated_raw_body.append(points_target_body)
    points_base = raw_body_to_base(np.vstack(accumulated_raw_body))

    case_name = f"wp{target.waypoint_id}_{2 * radius + 1}f"
    pcd_path = output_root / "pcd" / f"{case_name}.pcd"
    yaml_path = output_root / "configs" / f"{case_name}.yaml"
    eval_output_dir = output_root / "eval" / case_name
    write_ascii_pcd(pcd_path, points_base)
    write_eval_yaml(yaml_path, pcd_path.resolve(), eval_output_dir.resolve(), target_reference.msg)
    return yaml_path, eval_output_dir, len(points_base)


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
    process = subprocess.run(command, cwd=str(workspace), text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    print(process.stdout, end="")
    if process.stderr:
        print(process.stderr, end="", file=sys.stderr)
    return process.returncode


def read_metric(output_dir: Path) -> dict[str, str]:
    """读取 single PCD evaluator 输出的 arbitrary_start_no_prior 行。"""
    path = output_dir / "global_relocalization_metrics.csv"
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("scenario_name") == "arbitrary_start_no_prior":
                return row
    raise RuntimeError(f"no metric row in {path}")


def write_summary(path: Path, rows: list[dict[str, str]]) -> None:
    """写 sweep 汇总 CSV。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "waypoint_id",
        "name",
        "target_index",
        "frames",
        "raw_points",
        "localized",
        "success",
        "translation_error_m",
        "yaw_error_deg",
        "refined_candidate_rank",
        "refine_fitness_score",
        "scan_points",
        "search_ms",
        "refine_ms",
        "total_ms",
        "delta_cpu_ms",
        "peak_rss_mb",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run accumulated scan sweep for failed waypoint frames.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--bag", type=Path, default=Path("/home/ubuntu/nav_drift_test/nav_drift_test46"), help="验证 bag")
    parser.add_argument(
        "--matches",
        type=Path,
        default=Path(".codex_tmp/waypoint_pose_validation/dynamic_waypoints_matches.csv"),
        help="run_waypoint_pose_validation.py 生成的点位匹配 CSV",
    )
    parser.add_argument("--ids", nargs="*", default=["2", "4", "8", "12"], help="要验证的 waypoint_id")
    parser.add_argument("--windows", nargs="*", type=int, default=[3, 5, 7], help="累积帧数，必须为奇数")
    parser.add_argument("--odom-tolerance", type=float, default=0.05, help="点云和 odom header 时间同步容差")
    parser.add_argument("--reference-tolerance", type=float, default=0.05, help="点云和 /robot_realpose bag time 同步容差")
    parser.add_argument("--run", action="store_true", help="生成 PCD/YAML 后立即运行 evaluator")
    parser.add_argument("--keep-existing", action="store_true", help="不删除旧输出目录")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    matches_path = (workspace / args.matches).resolve() if not args.matches.is_absolute() else args.matches
    output_root = workspace / ".codex_tmp/accumulated_waypoint_sweep"
    if args.run and output_root.exists() and not args.keep_existing:
        shutil.rmtree(output_root)

    targets = load_targets_from_matches(matches_path, set(args.ids))
    if not targets:
        print("[accumulated_sweep] no matched targets found", file=sys.stderr)
        return 2
    radii = [(frames - 1) // 2 for frames in args.windows if frames > 0 and frames % 2 == 1]
    if not radii:
        print("[accumulated_sweep] windows must be positive odd numbers", file=sys.stderr)
        return 2

    rclpy.init()
    odoms, references, clouds = read_required_bag_data(args.bag.resolve(), targets, max(radii))
    rclpy.shutdown()
    print(
        f"[accumulated_sweep] targets={len(targets)} odoms={len(odoms)} "
        f"references={len(references)} collected_clouds={len(clouds)}"
    )

    rows: list[dict[str, str]] = []
    for target in targets:
        for radius in radii:
            frames = 2 * radius + 1
            yaml_path, eval_output_dir, raw_points = make_accumulated_case(
                target,
                radius,
                odoms,
                references,
                clouds,
                output_root,
                args.odom_tolerance,
                args.reference_tolerance,
            )
            print(f"[accumulated_sweep] generated waypoint={target.waypoint_id} frames={frames} yaml={yaml_path}")
            if args.run:
                rc = run_offline_eval(workspace, yaml_path)
                if rc != 0:
                    return rc
                metric = read_metric(eval_output_dir)
                rows.append(
                    {
                        "waypoint_id": target.waypoint_id,
                        "name": target.name,
                        "target_index": str(target.cloud_index),
                        "frames": str(frames),
                        "raw_points": str(raw_points),
                        "localized": metric.get("localized", ""),
                        "success": metric.get("success", ""),
                        "translation_error_m": metric.get("translation_error_m", ""),
                        "yaw_error_deg": metric.get("yaw_error_deg", ""),
                        "refined_candidate_rank": metric.get("refined_candidate_rank", ""),
                        "refine_fitness_score": metric.get("refine_fitness_score", ""),
                        "scan_points": metric.get("scan_points", ""),
                        "search_ms": metric.get("search_ms", ""),
                        "refine_ms": metric.get("refine_ms", ""),
                        "total_ms": metric.get("total_ms", ""),
                        "delta_cpu_ms": (
                            f"{float(metric.get('delta_user_cpu_ms', '0')) + float(metric.get('delta_system_cpu_ms', '0')):.3f}"
                        ),
                        "peak_rss_mb": metric.get("peak_rss_mb", ""),
                    }
                )

    if rows:
        summary_path = output_root / "summary.csv"
        write_summary(summary_path, rows)
        print(f"[accumulated_sweep] wrote {summary_path}")
        for row in rows:
            print(
                "[accumulated_sweep] "
                f"wp={row['waypoint_id']} frames={row['frames']} "
                f"success={row['success']} err={row['translation_error_m']}m/{row['yaw_error_deg']}deg "
                f"total={row['total_ms']}ms cpu={row['delta_cpu_ms']}ms"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
