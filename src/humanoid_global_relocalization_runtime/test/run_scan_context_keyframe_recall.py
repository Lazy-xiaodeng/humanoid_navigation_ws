#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_scan_context_keyframe_recall.py

文件作用：
  1. 离线验证“真实 keyframe scan 描述子库”是否能提升静止冷启动候选召回率。
  2. 从 bag 中抽样历史 /fast_lio/cloud_registered 点云，结合 /odom 转成当前 body/base 视角，构造 Scan Context 风格描述子库。
  3. 对点位 JSON 匹配出的目标帧计算同样的描述子，在排除目标附近帧后检索 top-K keyframe。
  4. 输出每个目标的 top1/topK 召回误差，用来判断后续是否值得把描述子候选接入 C++ 3D-BBS/GICP 验证链路。

重要说明：
  - 该脚本是验证工具，不是线上节点。它用真实 bag keyframe 模拟“建图/巡航阶段已经离线建好的描述子库”。
  - 默认会排除目标帧前后若干 cloud index，避免把当前帧或相邻几秒直接放进数据库造成泄漏。
  - 当前只处理 registered_world 输入链路；body 链路上线时可以直接对 /cloud_registered_body 计算同一描述子。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_scan_context_keyframe_recall.py \
    --matches .codex_tmp/waypoint_pose_validation_bag46_25pts_single_fallback/dynamic_waypoints_matches.csv \
    --ids 8 \
    --output .codex_tmp/scan_context_keyframe_recall_wp8.csv
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import sensor_msgs_py.point_cloud2 as pc2


@dataclass(frozen=True)
class Target:
    """一个需要验证描述子召回的目标点位。"""

    waypoint_id: str
    name: str
    cloud_index: int
    reference_x: float
    reference_y: float
    reference_yaw_deg: float


@dataclass(frozen=True)
class OdomSample:
    """一条 /odom 样本，用于把 registered_world 点云转回当前 body/base 视角。"""

    stamp_sec: float
    msg: Any


@dataclass(frozen=True)
class ReferenceSample:
    """一条 /robot_realpose 样本，用于评估召回 keyframe 和目标真值的距离。"""

    bag_time_sec: float
    x: float
    y: float
    yaw_deg: float


@dataclass(frozen=True)
class CloudSample:
    """一条被抽样进入描述子库或作为目标查询的点云。"""

    cloud_index: int
    stamp_sec: float
    bag_time_sec: float
    msg: Any


@dataclass(frozen=True)
class DescriptorEntry:
    """描述子数据库中的一个 keyframe 条目。"""

    cloud_index: int
    descriptor: np.ndarray
    reference_x: float
    reference_y: float
    reference_yaw_deg: float


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""

    return Path(__file__).resolve().parents[3]


def stamp_to_sec(stamp: Any) -> float:
    """ROS builtin_interfaces/Time 转成浮点秒。"""

    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(q: Any) -> float:
    """从 ROS 四元数读取 yaw，单位 rad。"""

    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_to_rotation_matrix(q: Any) -> np.ndarray:
    """把 ROS 四元数转为 3x3 旋转矩阵。"""

    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def normalize_angle_deg(angle: float) -> float:
    """把角度规整到 [-180, 180]，方便统计 yaw 误差。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    """打开 rosbag2 reader；storage_id 为空时由 rosbag2 根据 metadata 自动判断 mcap/sqlite。"""

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def read_targets(path: Path, ids: set[str]) -> list[Target]:
    """从点位匹配 CSV 读取目标帧。"""

    targets: list[Target] = []
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            waypoint_id = row["waypoint_id"]
            if ids and waypoint_id not in ids:
                continue
            targets.append(
                Target(
                    waypoint_id=waypoint_id,
                    name=row["name"],
                    cloud_index=int(row["cloud_index"]),
                    reference_x=float(row["reference_x"]),
                    reference_y=float(row["reference_y"]),
                    reference_yaw_deg=float(row["reference_yaw_deg"]),
                )
            )
    return targets


def read_bag_samples(
    bag_path: Path,
    targets: list[Target],
    keyframe_stride: int,
) -> tuple[list[OdomSample], list[ReferenceSample], dict[int, CloudSample], list[CloudSample]]:
    """读取 odom/reference、目标帧和抽样 keyframe 点云。"""

    point_cloud_type = get_message("sensor_msgs/msg/PointCloud2")
    odom_type = get_message("nav_msgs/msg/Odometry")
    reference_type = get_message("geometry_msgs/msg/PoseWithCovarianceStamped")
    target_indices = {target.cloud_index for target in targets}

    odoms: list[OdomSample] = []
    references: list[ReferenceSample] = []
    target_clouds: dict[int, CloudSample] = {}
    keyframe_clouds: list[CloudSample] = []
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
        if cloud_index in target_indices or cloud_index % keyframe_stride == 0:
            msg = deserialize_message(data, point_cloud_type)
            sample = CloudSample(cloud_index, stamp_to_sec(msg.header.stamp), bag_time_sec, msg)
            if cloud_index in target_indices:
                target_clouds[cloud_index] = sample
            if cloud_index % keyframe_stride == 0:
                keyframe_clouds.append(sample)
    return odoms, references, target_clouds, keyframe_clouds


def nearest_odom(odoms: list[OdomSample], stamp_sec: float) -> OdomSample:
    """按 header stamp 找最近 odom。"""

    return min(odoms, key=lambda item: abs(item.stamp_sec - stamp_sec))


def nearest_reference(references: list[ReferenceSample], bag_time_sec: float) -> ReferenceSample:
    """按 bag time 找最近真值位姿。"""

    return min(references, key=lambda item: abs(item.bag_time_sec - bag_time_sec))


def registered_world_cloud_to_body(sample: CloudSample, odom: OdomSample) -> np.ndarray:
    """把 /fast_lio/cloud_registered 世界点云变换到 C++ evaluator 使用的 base 坐标。"""

    points_world = pc2.read_points_numpy(
        sample.msg,
        field_names=["x", "y", "z"],
        skip_nans=True,
    ).astype(np.float64)
    pose = odom.msg.pose.pose
    translation = np.array(
        [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
        dtype=np.float64,
    )
    rotation = quaternion_to_rotation_matrix(pose.orientation)
    raw_body = (points_world - translation) @ rotation
    # 与 point_cloud_adapter.cpp 中 convert_raw_body_to_base_cloud 保持一致：
    # Fast-LIO raw/body 为 x 左、y 下、z 后；ROS base 为 x 前、y 左、z 上。
    base = np.empty_like(raw_body)
    base[:, 0] = -raw_body[:, 2] + 0.072
    base[:, 1] = raw_body[:, 0] - 0.004
    base[:, 2] = -raw_body[:, 1] + 1.215
    return base


def scan_context_descriptor(
    points: np.ndarray,
    rings: int,
    sectors: int,
    max_radius: float,
    min_range: float,
    min_z: float,
    max_z: float,
) -> np.ndarray:
    """计算一个简化 Scan Context 描述子：环扇区占据 + 弱高度编码。"""

    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    radius = np.hypot(points[:, 0], points[:, 1])
    valid = (
        (radius >= min_range) &
        (radius <= max_radius) &
        (points[:, 2] >= min_z) &
        (points[:, 2] <= max_z)
    )
    points = points[valid]
    radius = radius[valid]
    descriptor = np.zeros((rings, sectors), dtype=np.float32)
    if len(points) == 0:
        return descriptor

    theta = (np.arctan2(points[:, 1], points[:, 0]) + 2.0 * np.pi) % (2.0 * np.pi)
    ring_index = np.minimum(rings - 1, (radius / max_radius * rings).astype(np.int32))
    sector_index = np.minimum(sectors - 1, (theta / (2.0 * np.pi) * sectors).astype(np.int32))
    # 用 1.0 表示占据，再加少量高度信息。这样比纯 max-z 更不容易被天花板/高墙主导。
    np.maximum.at(descriptor, (ring_index, sector_index), 1.0 + 0.1 * points[:, 2].astype(np.float32))
    return descriptor


def descriptor_distance(query: np.ndarray, candidate: np.ndarray) -> tuple[float, int]:
    """计算两个描述子的最佳循环列移位距离。"""

    sectors = query.shape[1]
    best_distance = float("inf")
    best_shift = 0
    for shift in range(sectors):
        shifted = np.roll(candidate, shift, axis=1)
        occupied = (query > 0.0) | (shifted > 0.0)
        if not occupied.any():
            continue
        intersection = np.logical_and(query > 0.0, shifted > 0.0).sum()
        union = occupied.sum()
        occupancy_distance = 1.0 - float(intersection) / float(max(1, union))
        height_distance = float(np.mean(np.abs(query[occupied] - shifted[occupied])))
        distance = occupancy_distance + 0.05 * height_distance
        if distance < best_distance:
            best_distance = distance
            best_shift = shift
    return best_distance, best_shift


def build_descriptor_database(
    keyframe_clouds: list[CloudSample],
    odoms: list[OdomSample],
    references: list[ReferenceSample],
    args: argparse.Namespace,
) -> list[DescriptorEntry]:
    """把抽样 keyframe 点云转成描述子数据库。"""

    entries: list[DescriptorEntry] = []
    for sample in keyframe_clouds:
        odom = nearest_odom(odoms, sample.stamp_sec)
        reference = nearest_reference(references, sample.bag_time_sec)
        points = registered_world_cloud_to_body(sample, odom)
        descriptor = scan_context_descriptor(
            points,
            args.rings,
            args.sectors,
            args.max_radius,
            args.min_range,
            args.min_z,
            args.max_z,
        )
        entries.append(
            DescriptorEntry(
                sample.cloud_index,
                descriptor,
                reference.x,
                reference.y,
                reference.yaw_deg,
            )
        )
    return entries


def query_target(
    target: Target,
    sample: CloudSample,
    database: list[DescriptorEntry],
    odoms: list[OdomSample],
    args: argparse.Namespace,
) -> dict[str, str]:
    """查询单个目标，并返回 top1/topK 召回统计。"""

    odom = nearest_odom(odoms, sample.stamp_sec)
    points = registered_world_cloud_to_body(sample, odom)
    query = scan_context_descriptor(
        points,
        args.rings,
        args.sectors,
        args.max_radius,
        args.min_range,
        args.min_z,
        args.max_z,
    )

    scored: list[tuple[float, int, DescriptorEntry]] = []
    for entry in database:
        if abs(entry.cloud_index - target.cloud_index) <= args.exclude_index_radius:
            continue
        distance, shift = descriptor_distance(query, entry.descriptor)
        scored.append((distance, shift, entry))
    scored.sort(key=lambda item: item[0])
    top = scored[:max(1, args.top_k)]

    def error(entry: DescriptorEntry) -> tuple[float, float]:
        xy = math.hypot(entry.reference_x - target.reference_x, entry.reference_y - target.reference_y)
        yaw = abs(normalize_angle_deg(entry.reference_yaw_deg - target.reference_yaw_deg))
        return xy, yaw

    top1_distance, top1_shift, top1_entry = top[0]
    top1_xy, top1_yaw = error(top1_entry)

    best_rank = 0
    best_xy = float("inf")
    best_yaw = float("inf")
    best_distance = float("inf")
    for rank, (distance, _shift, entry) in enumerate(top, start=1):
        xy, yaw = error(entry)
        if xy < best_xy:
            best_rank = rank
            best_xy = xy
            best_yaw = yaw
            best_distance = distance

    return {
        "waypoint_id": target.waypoint_id,
        "name": target.name,
        "target_index": str(target.cloud_index),
        "db_entries": str(len(database)),
        "top_k": str(args.top_k),
        "exclude_index_radius": str(args.exclude_index_radius),
        "top1_index": str(top1_entry.cloud_index),
        "top1_distance": f"{top1_distance:.6f}",
        "top1_shift": str(top1_shift),
        "top1_xy_error_m": f"{top1_xy:.6f}",
        "top1_yaw_error_deg": f"{top1_yaw:.6f}",
        "best_topk_rank": str(best_rank),
        "best_topk_distance": f"{best_distance:.6f}",
        "best_topk_xy_error_m": f"{best_xy:.6f}",
        "best_topk_yaw_error_deg": f"{best_yaw:.6f}",
        "topk_success_1m_15deg": "1" if best_xy <= 1.0 and best_yaw <= 15.0 else "0",
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """写出召回统计 CSV。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate Scan Context keyframe recall for static cold start.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--bag", type=Path, default=Path("/home/ubuntu/nav_drift_test/nav_drift_test46"), help="描述子 DB 和查询来源 bag")
    parser.add_argument("--matches", type=Path, required=True, help="点位匹配 CSV")
    parser.add_argument("--ids", nargs="*", default=[], help="只验证指定 waypoint_id；不传则验证全部")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/scan_context_keyframe_recall.csv"), help="输出 CSV")
    parser.add_argument("--keyframe-stride", type=int, default=40, help="描述子库每隔多少个 cloud frame 抽一帧")
    parser.add_argument("--exclude-index-radius", type=int, default=240, help="查询时排除目标前后多少个 cloud index，避免当前附近帧泄漏")
    parser.add_argument("--top-k", type=int, default=20, help="统计前多少个召回结果")
    parser.add_argument("--rings", type=int, default=20, help="Scan Context 环数")
    parser.add_argument("--sectors", type=int, default=60, help="Scan Context 扇区数")
    parser.add_argument("--max-radius", type=float, default=20.0, help="描述子最大水平半径")
    parser.add_argument("--min-range", type=float, default=0.6, help="忽略机器人近场点的半径")
    parser.add_argument("--min-z", type=float, default=0.2, help="描述子最低高度")
    parser.add_argument("--max-z", type=float, default=2.5, help="描述子最高高度")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    matches = args.matches if args.matches.is_absolute() else workspace / args.matches
    bag = args.bag if args.bag.is_absolute() else workspace / args.bag
    output = args.output if args.output.is_absolute() else workspace / args.output

    targets = read_targets(matches, set(args.ids))
    if not targets:
        print("[scan_context_recall] no targets selected")
        return 2

    odoms, references, target_clouds, keyframe_clouds = read_bag_samples(
        bag,
        targets,
        max(1, args.keyframe_stride),
    )
    database = build_descriptor_database(keyframe_clouds, odoms, references, args)

    rows: list[dict[str, str]] = []
    for target in targets:
        sample = target_clouds.get(target.cloud_index)
        if sample is None:
            print(f"[scan_context_recall] missing target cloud index={target.cloud_index}")
            continue
        row = query_target(target, sample, database, odoms, args)
        rows.append(row)
        print(
            "[scan_context_recall] "
            f"wp={row['waypoint_id']} top1={row['top1_xy_error_m']}m/{row['top1_yaw_error_deg']}deg "
            f"best_top{row['top_k']}=rank{row['best_topk_rank']} "
            f"{row['best_topk_xy_error_m']}m/{row['best_topk_yaw_error_deg']}deg "
            f"success={row['topk_success_1m_15deg']}"
        )

    write_csv(output, rows)
    print(f"[scan_context_recall] wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
