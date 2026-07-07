#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_opd_reloc_validation.py

文件作用：
  1. 用 LiDAR-Localization-100FPS / OPD 的完整“离线模板库 + 在线占据投票”流程验证 bag46 点位。
  2. 从全局地图生成候选站位，对每个候选站位和 yaw 角生成 OPD 占据模板数据库。
  3. 从 bag 的 /fast_lio/cloud_registered + /odom 还原当前 base 视角 scan，生成同样的 OPD 查询模板。
  4. 输出 OPD top-K 候选与真值误差，用来判断它能否替代或增强现有 Scan Context / 3D-BBS 召回层。

重要说明：
  - 这里没有改线上节点，只做离线实验；算法流程保持官方 100FPS 的核心模板/投票方法。
  - 官方离线 mapping 会从地面栅格生成 candidate_pts；本脚本用已建好的 grounded 地图低高度点云近似生成候选。
  - 输出的 yaw 是官方 relocate.cpp 中 pose_relocate = R(theta)^-1 后的 map->base yaw。
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d
import psutil
import yaml
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree

from run_scan_context_keyframe_recall import (
    OdomSample,
    ReferenceSample,
    nearest_odom,
    quaternion_to_rotation_matrix,
    registered_world_cloud_to_body,
    repo_root_from_script,
    stamp_to_sec,
    yaw_from_quaternion,
)


@dataclass(frozen=True)
class CloudSample:
    """一帧用于 OPD 查询的点云及其真值。"""

    cloud_index: int
    stamp_sec: float
    bag_time_sec: float
    msg: Any | None
    points: np.ndarray | None
    reference_x: float
    reference_y: float
    reference_yaw_deg: float


@dataclass(frozen=True)
class OpdDatabase:
    """OPD 离线模板库；inverted_index[cell] 存储占据该 cell 的候选模板编号。"""

    candidates_xy: np.ndarray
    theta_num: int
    rows: int
    cols: int
    length: float
    inverted_index: list[np.ndarray]


def normalize_angle_deg(angle: float) -> float:
    """把角度规整到 [-180, 180]，便于计算 yaw 误差。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    """打开 rosbag2 reader。"""

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def load_indices_from_yaml(path: Path) -> list[int]:
    """从现有 evaluator yaml 读取 bag_sample_frame_indices。"""

    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    params = data["global_relocalization_eval"]["ros__parameters"]
    return [int(v) for v in params["bag_sample_frame_indices"]]


def nearest_reference(references: list[ReferenceSample], bag_time_sec: float) -> ReferenceSample:
    """按 bag time 寻找最近 /robot_realpose。"""

    return min(references, key=lambda item: abs(item.bag_time_sec - bag_time_sec))


def read_bag_targets(bag_path: Path, target_indices: set[int]) -> tuple[list[OdomSample], list[CloudSample]]:
    """读取 odom、真值和指定 cloud index 的查询点云。"""

    point_cloud_type = get_message("sensor_msgs/msg/PointCloud2")
    odom_type = get_message("nav_msgs/msg/Odometry")
    reference_type = get_message("geometry_msgs/msg/PoseWithCovarianceStamped")

    odoms: list[OdomSample] = []
    references: list[ReferenceSample] = []
    raw_targets: dict[int, tuple[float, float, Any]] = {}
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
            pose = msg.pose.pose
            references.append(
                ReferenceSample(
                    bag_time_sec,
                    float(pose.position.x),
                    float(pose.position.y),
                    math.degrees(yaw_from_quaternion(pose.orientation)),
                )
            )
            continue
        if topic != "/fast_lio/cloud_registered":
            continue
        cloud_index += 1
        if cloud_index in target_indices:
            msg = deserialize_message(data, point_cloud_type)
            raw_targets[cloud_index] = (stamp_to_sec(msg.header.stamp), bag_time_sec, msg)

    targets: list[CloudSample] = []
    for cloud_index, (stamp_sec, bag_time_sec, msg) in sorted(raw_targets.items()):
        ref = nearest_reference(references, bag_time_sec)
        targets.append(
                CloudSample(
                    cloud_index,
                    stamp_sec,
                    bag_time_sec,
                    msg,
                    None,
                    ref.x,
                    ref.y,
                    ref.yaw_deg,
                )
        )
    return odoms, targets


def read_pcd_targets(targets_csv: Path) -> list[CloudSample]:
    """读取已导出的目标 PCD 数据集；PCD 点云已经是 base/body 查询坐标。"""

    targets: list[CloudSample] = []
    with targets_csv.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            pcd_path = Path(row["pcd"])
            cloud = o3d.io.read_point_cloud(str(pcd_path))
            points = np.asarray(cloud.points, dtype=np.float64)
            targets.append(
                CloudSample(
                    int(row["cloud_index"]),
                    0.0,
                    0.0,
                    None,
                    points,
                    float(row["x"]),
                    float(row["y"]),
                    float(row["yaw_deg"]),
                )
            )
    return targets


def load_map_points(map_path: Path, voxel_size: float, min_z: float, max_z: float) -> np.ndarray:
    """读取地图并降采样/裁剪到 OPD 模板需要的高度范围。"""

    cloud = o3d.io.read_point_cloud(str(map_path))
    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size)
    points = np.asarray(cloud.points, dtype=np.float64)
    valid = np.isfinite(points).all(axis=1) & (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    return points[valid, :3]


def generate_candidate_xy(points: np.ndarray, grid_size: float, min_points_per_cell: int) -> np.ndarray:
    """按官方 candidate_pts 思路，从低高度地图点所在栅格生成候选站位。"""

    grid = np.floor(points[:, :2] / grid_size).astype(np.int32)
    unique, counts = np.unique(grid, axis=0, return_counts=True)
    valid = unique[counts >= min_points_per_cell]
    centers = (valid.astype(np.float64) + 0.5) * grid_size
    order = np.lexsort((centers[:, 1], centers[:, 0]))
    return centers[order]


def make_opd_descriptor(points_xy: np.ndarray, center_xy: np.ndarray, rows: int, cols: int, length: float) -> np.ndarray:
    """生成官方 40x40 二值占据模板，返回被占据 cell 的一维索引。"""

    max_x = center_xy[0] + rows * 0.5 * length
    max_y = center_xy[1] + cols * 0.5 * length
    min_x = center_xy[0] - rows * 0.5 * length
    min_y = center_xy[1] - cols * 0.5 * length
    valid = (
        (points_xy[:, 0] >= min_x) & (points_xy[:, 0] <= max_x) &
        (points_xy[:, 1] >= min_y) & (points_xy[:, 1] <= max_y)
    )
    if not np.any(valid):
        return np.empty(0, dtype=np.int32)
    local = points_xy[valid]
    r = ((max_x - local[:, 0]) / length).astype(np.int32)
    c = ((max_y - local[:, 1]) / length).astype(np.int32)
    ok = (r >= 0) & (r < rows) & (c >= 0) & (c < cols)
    return np.unique((r[ok] * cols + c[ok]).astype(np.int32))


def build_opd_database(map_points: np.ndarray, args: argparse.Namespace) -> OpdDatabase:
    """构建 OPD 模板倒排索引数据库。"""

    candidates_xy = generate_candidate_xy(
        map_points,
        args.candidate_grid_size,
        args.min_points_per_candidate_cell,
    )
    tree = cKDTree(map_points[:, :2])
    cell_count = args.rows * args.cols
    theta_step = 2.0 * math.pi / float(args.theta_num)
    radius = math.sqrt(2.0) * args.rows * 0.5 * args.length
    inverted: list[list[int]] = [[] for _ in range(cell_count)]
    print(
        f"[opd] build database map_points={len(map_points)} candidates={len(candidates_xy)} "
        f"templates={len(candidates_xy) * args.theta_num} radius={radius:.2f}m",
        flush=True,
    )

    for candidate_index, center_xy in enumerate(candidates_xy):
        if candidate_index % max(1, len(candidates_xy) // 10) == 0:
            print(f"[opd] database progress {candidate_index}/{len(candidates_xy)}", flush=True)
        near_indices = tree.query_ball_point(center_xy, radius)
        near_xy = map_points[np.asarray(near_indices, dtype=np.int64), :2]
        relative = near_xy - center_xy
        for theta_index in range(args.theta_num):
            theta = theta_index * theta_step
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            rotated = np.empty_like(relative)
            rotated[:, 0] = cos_t * relative[:, 0] - sin_t * relative[:, 1] + center_xy[0]
            rotated[:, 1] = sin_t * relative[:, 0] + cos_t * relative[:, 1] + center_xy[1]
            cells = make_opd_descriptor(rotated, center_xy, args.rows, args.cols, args.length)
            template_index = candidate_index * args.theta_num + theta_index
            for cell in cells:
                inverted[int(cell)].append(template_index)

    compact = [np.asarray(values, dtype=np.int32) for values in inverted]
    return OpdDatabase(candidates_xy, args.theta_num, args.rows, args.cols, args.length, compact)


def save_database(path: Path, database: OpdDatabase, args: argparse.Namespace) -> None:
    """把 OPD 倒排索引缓存成 npz，避免每组验证重复离线建库。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    lengths = np.asarray([len(values) for values in database.inverted_index], dtype=np.int32)
    offsets = np.zeros(len(lengths) + 1, dtype=np.int64)
    offsets[1:] = np.cumsum(lengths, dtype=np.int64)
    flat = np.concatenate(database.inverted_index) if offsets[-1] > 0 else np.empty(0, dtype=np.int32)
    np.savez_compressed(
        path,
        candidates_xy=database.candidates_xy,
        theta_num=np.asarray([database.theta_num], dtype=np.int32),
        rows=np.asarray([database.rows], dtype=np.int32),
        cols=np.asarray([database.cols], dtype=np.int32),
        length=np.asarray([database.length], dtype=np.float64),
        offsets=offsets,
        flat=flat,
        map_voxel_size=np.asarray([args.map_voxel_size], dtype=np.float64),
        candidate_grid_size=np.asarray([args.candidate_grid_size], dtype=np.float64),
    )


def load_database(path: Path) -> OpdDatabase:
    """从 npz 读取 OPD 倒排索引缓存。"""

    data = np.load(path)
    offsets = data["offsets"]
    flat = data["flat"]
    inverted = [flat[offsets[i]:offsets[i + 1]].astype(np.int32, copy=False) for i in range(len(offsets) - 1)]
    return OpdDatabase(
        data["candidates_xy"],
        int(data["theta_num"][0]),
        int(data["rows"][0]),
        int(data["cols"][0]),
        float(data["length"][0]),
        inverted,
    )


def query_opd(scan_points: np.ndarray, database: OpdDatabase, args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray]:
    """按官方 findMaxVotes 思路对查询 scan 投票并返回 top-K 模板。"""

    radius = np.linalg.norm(scan_points[:, :3], axis=1)
    valid = (
        np.isfinite(scan_points).all(axis=1) &
        (radius >= args.min_range) &
        (radius <= args.max_radius) &
        (scan_points[:, 2] >= args.query_min_z) &
        (scan_points[:, 2] <= args.query_max_z)
    )
    scan_xy = scan_points[valid, :2]
    cells = make_opd_descriptor(scan_xy, np.array([0.0, 0.0]), database.rows, database.cols, database.length)
    votes = np.zeros(len(database.candidates_xy) * database.theta_num, dtype=np.int32)
    for cell in cells:
        indexes = database.inverted_index[int(cell)]
        if indexes.size:
            np.add.at(votes, indexes, 1)
    top_k = min(args.top_k, votes.size)
    top = np.argpartition(-votes, np.arange(top_k))[:top_k]
    top = top[np.argsort(-votes[top])]
    return top, votes[top]


def evaluate_target(target: CloudSample, odoms: list[OdomSample], database: OpdDatabase, args: argparse.Namespace) -> list[dict[str, str]]:
    """评估单个查询点，输出 top-K 候选误差。"""

    if target.points is not None:
        scan_points = target.points
    else:
        odom = nearest_odom(odoms, target.stamp_sec)
        scan_points = registered_world_cloud_to_body(target, odom)
    top_indexes, top_votes = query_opd(scan_points, database, args)
    theta_step_deg = 360.0 / float(database.theta_num)

    rows: list[dict[str, str]] = []
    best_success_rank = 0
    for rank, (template_index, vote) in enumerate(zip(top_indexes, top_votes), start=1):
        candidate_index = int(template_index) // database.theta_num
        theta_index = int(template_index) % database.theta_num
        x, y = database.candidates_xy[candidate_index]
        yaw_deg = normalize_angle_deg(-theta_index * theta_step_deg)
        trans_error = math.hypot(x - target.reference_x, y - target.reference_y)
        yaw_error = abs(normalize_angle_deg(yaw_deg - target.reference_yaw_deg))
        success = trans_error <= args.success_translation_thresh and yaw_error <= args.success_yaw_thresh_deg
        if success and best_success_rank == 0:
            best_success_rank = rank
        rows.append(
            {
                "cloud_index": str(target.cloud_index),
                "rank": str(rank),
                "vote": str(int(vote)),
                "candidate_index": str(candidate_index),
                "theta_index": str(theta_index),
                "final_x_m": f"{x:.6f}",
                "final_y_m": f"{y:.6f}",
                "final_yaw_deg": f"{yaw_deg:.6f}",
                "reference_x_m": f"{target.reference_x:.6f}",
                "reference_y_m": f"{target.reference_y:.6f}",
                "reference_yaw_deg": f"{target.reference_yaw_deg:.6f}",
                "translation_error_m": f"{trans_error:.6f}",
                "yaw_error_deg": f"{yaw_error:.6f}",
                "success": "1" if success else "0",
                "best_success_rank": str(best_success_rank),
            }
        )
    return rows


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """写 CSV 文件。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows: list[dict[str, str]], args: argparse.Namespace, elapsed_sec: float, peak_rss_mb: float) -> dict[str, str]:
    """按目标级别汇总 top1/topK 成功率和资源占用。"""

    by_cloud: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        by_cloud.setdefault(row["cloud_index"], []).append(row)
    top1_success = 0
    topk_success = 0
    best_errors: list[float] = []
    for cloud_rows in by_cloud.values():
        top1_success += int(cloud_rows[0]["success"] == "1")
        successes = [row for row in cloud_rows if row["success"] == "1"]
        topk_success += int(bool(successes))
        best_errors.append(min(float(row["translation_error_m"]) for row in cloud_rows))
    return {
        "targets": str(len(by_cloud)),
        "top_k": str(args.top_k),
        "top1_success": str(top1_success),
        "topk_success": str(topk_success),
        "success_translation_thresh_m": f"{args.success_translation_thresh:.3f}",
        "success_yaw_thresh_deg": f"{args.success_yaw_thresh_deg:.3f}",
        "candidate_grid_size_m": f"{args.candidate_grid_size:.3f}",
        "rows": str(args.rows),
        "cols": str(args.cols),
        "theta_num": str(args.theta_num),
        "elapsed_sec": f"{elapsed_sec:.3f}",
        "peak_rss_mb": f"{peak_rss_mb:.3f}",
        "best_translation_error_median": f"{float(np.median(best_errors)):.6f}" if best_errors else "nan",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate full OPD / LiDAR-Localization-100FPS descriptor voting.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--bag", type=Path, default=Path("/home/ubuntu/nav_drift_test/nav_drift_test46"), help="验证 bag")
    parser.add_argument("--indices-yaml", type=Path, default=None, help="包含 bag_sample_frame_indices 的现有 evaluator yaml；bag 模式必填")
    parser.add_argument("--targets-csv", type=Path, default=None, help="已导出的目标 PCD CSV；提供后不再顺序读取 bag 点云")
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"), help="全局地图 PCD")
    parser.add_argument("--output", type=Path, required=True, help="候选明细 CSV 输出路径")
    parser.add_argument("--summary-output", type=Path, required=True, help="汇总 CSV 输出路径")
    parser.add_argument("--map-voxel-size", type=float, default=0.35, help="地图降采样体素，越小越细但建库越慢")
    parser.add_argument("--map-min-z", type=float, default=0.02, help="地图模板最低高度")
    parser.add_argument("--map-max-z", type=float, default=2.5, help="地图模板最高高度")
    parser.add_argument("--candidate-grid-size", type=float, default=1.5, help="候选站位栅格尺寸")
    parser.add_argument("--min-points-per-candidate-cell", type=int, default=4, help="生成候选站位所需的最低低高度地图点数")
    parser.add_argument("--rows", type=int, default=40, help="OPD 模板行数")
    parser.add_argument("--cols", type=int, default=40, help="OPD 模板列数")
    parser.add_argument("--theta-num", type=int, default=60, help="yaw 离散数量")
    parser.add_argument("--length", type=float, default=1.0, help="OPD 模板每个 cell 的边长")
    parser.add_argument("--query-min-z", type=float, default=0.2, help="查询 scan 最低高度")
    parser.add_argument("--query-max-z", type=float, default=2.5, help="查询 scan 最高高度")
    parser.add_argument("--min-range", type=float, default=0.6, help="忽略机器人近场点半径")
    parser.add_argument("--max-radius", type=float, default=20.0, help="查询 scan 最大半径")
    parser.add_argument("--top-k", type=int, default=20, help="输出 OPD top-K 候选")
    parser.add_argument("--database-cache", type=Path, default=None, help="OPD 离线模板库 npz 缓存路径")
    parser.add_argument("--success-translation-thresh", type=float, default=0.3, help="成功平移阈值")
    parser.add_argument("--success-yaw-thresh-deg", type=float, default=5.0, help="成功 yaw 阈值")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    bag = args.bag if args.bag.is_absolute() else workspace / args.bag
    indices_yaml = None
    if args.indices_yaml is not None:
        indices_yaml = args.indices_yaml if args.indices_yaml.is_absolute() else workspace / args.indices_yaml
    targets_csv = None
    if args.targets_csv is not None:
        targets_csv = args.targets_csv if args.targets_csv.is_absolute() else workspace / args.targets_csv
    map_path = args.map if args.map.is_absolute() else workspace / args.map
    output = args.output if args.output.is_absolute() else workspace / args.output
    summary_output = args.summary_output if args.summary_output.is_absolute() else workspace / args.summary_output
    database_cache = None
    if args.database_cache is not None:
        database_cache = args.database_cache if args.database_cache.is_absolute() else workspace / args.database_cache

    start = time.perf_counter()
    process = psutil.Process()
    if targets_csv is not None:
        odoms = []
        targets = read_pcd_targets(targets_csv)
        print(f"[opd] loaded pcd targets={len(targets)} from {targets_csv}", flush=True)
    else:
        if indices_yaml is None:
            raise RuntimeError("--indices-yaml is required when --targets-csv is not provided")
        indices = load_indices_from_yaml(indices_yaml)
        print(f"[opd] read bag targets count={len(indices)}", flush=True)
        odoms, targets = read_bag_targets(bag, set(indices))
        print(f"[opd] loaded targets={len(targets)} odoms={len(odoms)}", flush=True)
    if database_cache is not None and database_cache.exists():
        database = load_database(database_cache)
        print(f"[opd] loaded database cache {database_cache}", flush=True)
    else:
        map_points = load_map_points(map_path, args.map_voxel_size, args.map_min_z, args.map_max_z)
        print(f"[opd] loaded map_points={len(map_points)}", flush=True)
        database = build_opd_database(map_points, args)
        if database_cache is not None:
            save_database(database_cache, database, args)
            print(f"[opd] saved database cache {database_cache}", flush=True)
    print(
        f"[opd] targets={len(targets)} "
        f"candidates={len(database.candidates_xy)} templates={len(database.candidates_xy) * database.theta_num}"
    )

    rows: list[dict[str, str]] = []
    for i, target in enumerate(targets, start=1):
        target_rows = evaluate_target(target, odoms, database, args)
        rows.extend(target_rows)
        best = min(float(row["translation_error_m"]) for row in target_rows)
        top1 = target_rows[0]
        print(
            f"[opd] {i}/{len(targets)} idx={target.cloud_index} "
            f"top1={top1['translation_error_m']}m/{top1['yaw_error_deg']}deg "
            f"best_xy={best:.3f}m"
        )

    elapsed = time.perf_counter() - start
    peak_rss_mb = process.memory_info().rss / 1024.0 / 1024.0
    summary = summarize(rows, args, elapsed, peak_rss_mb)
    write_csv(output, rows)
    write_csv(summary_output, [summary])
    print(f"[opd] wrote {output}")
    print(f"[opd] summary {summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
