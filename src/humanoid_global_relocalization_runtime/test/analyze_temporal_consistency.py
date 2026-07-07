#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_temporal_consistency.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 读取 global_relocalization_metrics.csv 和 global_relocalization_candidates.csv。
  3. 再从原始 rosbag2 中读取 /odom，把每个 3D-BBS 粗候选换算成它隐含的 map->odom。
  4. 在连续多帧窗口里统计某个 map->odom 假设能被多少帧候选共同支持。
  5. 用于验证“单帧最佳候选是否应该经过多帧一致性确认后才能重置定位”。

使用示例：
  source /opt/ros/jazzy/setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/analyze_temporal_consistency.py \
    .codex_tmp/global_relocalization_nav_drift_extended_v1/global_relocalization_metrics.csv \
    .codex_tmp/global_relocalization_nav_drift_extended_v1/global_relocalization_candidates.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


# Fast-LIO raw body 轴到 ROS base_footprint 标准轴的转换矩阵。
# C++ 点云预处理里使用：
#   base.x = -raw.z
#   base.y =  raw.x
#   base.z = -raw.y
# 这里对 /odom 姿态做同一坐标基变换，才能把候选 map->base 与 odom->base 放在同一个标准轴下比较。
RAW_TO_BASE = np.array(
    [
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


@dataclass
class OdomSample:
    """保存一条 /odom，用 header stamp 和点云帧匹配。"""

    stamp_sec: float
    pose_raw: np.ndarray


@dataclass
class CandidateRecord:
    """保存一个粗候选，以及它换算出的隐含 map->odom 假设。"""

    rank: int
    score: int
    map_base: np.ndarray
    map_odom: np.ndarray | None = None
    map_odom_x: float = math.nan
    map_odom_y: float = math.nan
    map_odom_yaw_deg: float = math.nan


@dataclass
class FrameRecord:
    """保存一帧评估记录和它的 top-K 候选。"""

    bag_path: str
    bag_name: str
    stamp: str
    stamp_sec: float
    success: int
    trans_error_m: float
    yaw_error_deg: float
    refined_rank: int
    candidates: list[CandidateRecord] = field(default_factory=list)
    odom_pose_base: np.ndarray | None = None


def to_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """从 CSV 行里取浮点数，空字段或异常字段统一变成 NaN，方便后续统计跳过。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def to_int(row: dict[str, str], key: str, default: int = 0) -> int:
    """从 CSV 行里取整数；部分字段可能写成 1.000，所以先走 float 再转 int。"""
    try:
        return int(float(row.get(key, "")))
    except ValueError:
        return default


def stamp_to_sec(stamp) -> float:
    """把 ROS builtin_interfaces/Time 转成秒。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_diff_deg(a: float, b: float) -> float:
    """计算两个 yaw 的最小夹角差，输出范围为 0~180 度。"""
    diff = math.radians(a - b)
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return abs(math.degrees(diff))


def yaw_from_matrix(pose: np.ndarray) -> float:
    """从 4x4 位姿矩阵中提取平面 yaw，忽略 roll/pitch。"""
    return math.degrees(math.atan2(float(pose[1, 0]), float(pose[0, 0])))


def quaternion_to_rotation_matrix(q_msg) -> np.ndarray:
    """把 ROS quaternion 转为 3x3 旋转矩阵，并对异常零长度四元数做单位阵保护。"""
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


def pose_from_xyzyaw(x: float, y: float, z: float, yaw_deg: float) -> np.ndarray:
    """用 CSV 中的 x/y/z/yaw 构造 map->base 粗候选矩阵。"""
    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    pose = np.eye(4, dtype=np.float64)
    pose[0, 0] = c
    pose[0, 1] = -s
    pose[1, 0] = s
    pose[1, 1] = c
    pose[0, 3] = x
    pose[1, 3] = y
    pose[2, 3] = z
    return pose


def odom_raw_to_base_pose(odom_msg) -> np.ndarray:
    """把 Fast-LIO raw world->raw body odom 转成标准轴 odom->base 矩阵。"""
    raw_pose = np.eye(4, dtype=np.float64)
    raw_pose[:3, :3] = quaternion_to_rotation_matrix(odom_msg.pose.pose.orientation)
    raw_pose[:3, 3] = np.array(
        [
            float(odom_msg.pose.pose.position.x),
            float(odom_msg.pose.pose.position.y),
            float(odom_msg.pose.pose.position.z),
        ],
        dtype=np.float64,
    )

    # 坐标基变换：p_odom_base_axis = R_raw_to_base * p_odom_raw。
    # 旋转需要左右各乘一次，平移只需要从 raw 轴表达转成 base 标准轴表达。
    base_pose = np.eye(4, dtype=np.float64)
    base_pose[:3, :3] = RAW_TO_BASE @ raw_pose[:3, :3] @ RAW_TO_BASE.T
    base_pose[:3, 3] = RAW_TO_BASE @ raw_pose[:3, 3]
    return base_pose


def key_for(row: dict[str, str]) -> tuple[str, str, str, str]:
    """metrics 和 candidates CSV 共用的帧级 key。"""
    return (
        row.get("map_path", ""),
        row.get("bag_path", ""),
        row.get("stamp_sec", ""),
        row.get("refine_method", ""),
    )


def load_csv(path: Path) -> list[dict[str, str]]:
    """读取 UTF-8 CSV，返回 DictReader 行列表。"""
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def load_frames(metrics_path: Path, candidates_path: Path, scenario_name: str) -> list[FrameRecord]:
    """合并主指标和候选明细，只保留指定模拟场景，避免同一帧重复统计。"""
    candidate_groups: dict[tuple[str, str, str, str], list[dict[str, str]]] = defaultdict(list)
    for row in load_csv(candidates_path):
        if row.get("scenario_name") == scenario_name:
            candidate_groups[key_for(row)].append(row)

    frames: list[FrameRecord] = []
    for row in load_csv(metrics_path):
        if row.get("scenario_name") != scenario_name:
            continue
        if to_int(row, "localized") != 1:
            continue

        bag_path = row.get("bag_path", "")
        frame = FrameRecord(
            bag_path=bag_path,
            bag_name=Path(bag_path).name,
            stamp=row.get("stamp_sec", ""),
            stamp_sec=to_float(row, "stamp_sec"),
            success=to_int(row, "success"),
            trans_error_m=to_float(row, "translation_error_m"),
            yaw_error_deg=to_float(row, "yaw_error_deg"),
            refined_rank=to_int(row, "refined_candidate_rank"),
        )

        for candidate_row in sorted(candidate_groups.get(key_for(row), []), key=lambda r: to_int(r, "rank")):
            frame.candidates.append(
                CandidateRecord(
                    rank=to_int(candidate_row, "rank"),
                    score=to_int(candidate_row, "score"),
                    map_base=pose_from_xyzyaw(
                        to_float(candidate_row, "candidate_x_m"),
                        to_float(candidate_row, "candidate_y_m"),
                        to_float(candidate_row, "candidate_z_m"),
                        to_float(candidate_row, "candidate_yaw_deg"),
                    ),
                )
            )
        frames.append(frame)
    return frames


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    """打开 rosbag2；storage_id 留空，让 ROS 按 metadata 自动选择 sqlite3/mcap。"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def load_odoms(bag_path: str, odom_topic: str, min_stamp: float, max_stamp: float, tolerance_sec: float) -> list[OdomSample]:
    """读取一个 bag 中目标时间段附近的 /odom，并保留标准轴下的 odom->base 位姿。"""
    odom_type = get_message("nav_msgs/msg/Odometry")
    odoms: list[OdomSample] = []
    reader = open_reader(bag_path)
    lower_bound = min_stamp - tolerance_sec
    upper_bound = max_stamp + tolerance_sec
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != odom_topic:
            continue
        msg = deserialize_message(data, odom_type)
        stamp_sec = stamp_to_sec(msg.header.stamp)

        # nav_drift bag 的 /odom header stamp 单调递增；超过目标窗口后立即早停。
        # 如果后续遇到非单调 bag，可以去掉这个 break，代价只是多扫一些无关消息。
        if stamp_sec > upper_bound and odoms:
            break
        if lower_bound <= stamp_sec <= upper_bound:
            odoms.append(OdomSample(stamp_sec, odom_raw_to_base_pose(msg)))
    odoms.sort(key=lambda sample: sample.stamp_sec)
    return odoms


def nearest_odom(odoms: list[OdomSample], stamp_sec: float, tolerance_sec: float) -> OdomSample | None:
    """按 header stamp 查找最近 odom；当前 bag 点云和 odom header 时间同源。"""
    best: OdomSample | None = None
    best_dt = float("inf")
    for sample in odoms:
        dt = abs(sample.stamp_sec - stamp_sec)
        if dt < best_dt:
            best = sample
            best_dt = dt
    if best is None or best_dt > tolerance_sec:
        return None
    return best


def attach_odom_and_map_odom(frames: list[FrameRecord], odom_topic: str, tolerance_sec: float) -> None:
    """给每帧候选计算隐含 map->odom，公式为 T_map_odom = T_map_base * inverse(T_odom_base)。"""
    odom_cache: dict[str, list[OdomSample]] = {}
    requested_stamps: dict[str, list[float]] = defaultdict(list)
    for frame in frames:
        requested_stamps[frame.bag_path].append(frame.stamp_sec)

    for frame in frames:
        if frame.bag_path not in odom_cache:
            stamps = requested_stamps[frame.bag_path]
            odom_cache[frame.bag_path] = load_odoms(
                frame.bag_path,
                odom_topic,
                min(stamps),
                max(stamps),
                tolerance_sec,
            )
        odom = nearest_odom(odom_cache[frame.bag_path], frame.stamp_sec, tolerance_sec)
        if odom is None:
            continue
        frame.odom_pose_base = odom.pose_raw
        odom_base_inv = np.linalg.inv(odom.pose_raw)
        for candidate in frame.candidates:
            candidate.map_odom = candidate.map_base @ odom_base_inv
            candidate.map_odom_x = float(candidate.map_odom[0, 3])
            candidate.map_odom_y = float(candidate.map_odom[1, 3])
            candidate.map_odom_yaw_deg = yaw_from_matrix(candidate.map_odom)


def candidate_distance(a: CandidateRecord, b: CandidateRecord) -> tuple[float, float]:
    """计算两个候选隐含 map->odom 的平面距离和 yaw 差。"""
    if a.map_odom is None or b.map_odom is None:
        return float("inf"), float("inf")
    xy = math.hypot(a.map_odom_x - b.map_odom_x, a.map_odom_y - b.map_odom_y)
    yaw = yaw_diff_deg(a.map_odom_yaw_deg, b.map_odom_yaw_deg)
    return xy, yaw


def support_for_seed(
    bag_frames: list[FrameRecord],
    frame_index: int,
    seed: CandidateRecord,
    window_before: int,
    window_after: int,
    xy_gate_m: float,
    yaw_gate_deg: float,
) -> tuple[int, int, float]:
    """统计一个 seed 候选在连续窗口内被多少帧支持，并返回候选总数和支持候选 median rank。"""
    start = max(0, frame_index - window_before)
    end = min(len(bag_frames), frame_index + window_after + 1)
    supporting_ranks: list[int] = []
    support_frames = 0

    for frame in bag_frames[start:end]:
        matched_ranks: list[int] = []
        for candidate in frame.candidates:
            xy, yaw = candidate_distance(seed, candidate)
            if xy <= xy_gate_m and yaw <= yaw_gate_deg:
                matched_ranks.append(candidate.rank)
        if matched_ranks:
            support_frames += 1
            supporting_ranks.append(min(matched_ranks))

    median_rank = statistics.median(supporting_ranks) if supporting_ranks else math.nan
    return support_frames, len(supporting_ranks), median_rank


def selected_candidate(frame: FrameRecord) -> CandidateRecord | None:
    """找到 C++ 精配准最终选择的粗候选 rank。"""
    for candidate in frame.candidates:
        if candidate.rank == frame.refined_rank:
            return candidate
    return None


def best_supported_seed(
    bag_frames: list[FrameRecord],
    frame_index: int,
    window_before: int,
    window_after: int,
    xy_gate_m: float,
    yaw_gate_deg: float,
) -> tuple[CandidateRecord | None, int, int, float]:
    """在当前帧 top-K 中找时序支持最强的 seed；并列时优先 rank 更靠前者。"""
    frame = bag_frames[frame_index]
    best_key: tuple[int, int, float, int] | None = None
    best_candidate: CandidateRecord | None = None
    best_median_rank = math.nan
    for candidate in frame.candidates:
        support_frames, support_count, median_rank = support_for_seed(
            bag_frames,
            frame_index,
            candidate,
            window_before,
            window_after,
            xy_gate_m,
            yaw_gate_deg,
        )
        key = (support_frames, support_count, -median_rank if math.isfinite(median_rank) else -9999.0, -candidate.rank)
        if best_key is None or key > best_key:
            best_key = key
            best_candidate = candidate
            best_median_rank = median_rank
    if best_key is None or best_candidate is None:
        return None, 0, 0, math.nan
    return best_candidate, best_key[0], best_key[1], best_median_rank


def median(values: list[float]) -> float:
    """只对有限数值取 median；空集合输出 NaN。"""
    finite = [v for v in values if math.isfinite(v)]
    return statistics.median(finite) if finite else math.nan


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze temporal map->odom consistency for 3D-BBS candidates.")
    parser.add_argument("metrics_csv", type=Path, help="global_relocalization_metrics.csv")
    parser.add_argument("candidates_csv", type=Path, help="global_relocalization_candidates.csv")
    parser.add_argument("--scenario", default="arbitrary_start_no_prior", help="只分析指定模拟场景")
    parser.add_argument("--odom-topic", default="/odom", help="bag 中的 odom 话题")
    parser.add_argument("--odom-tolerance", type=float, default=0.05, help="点云 header stamp 与 odom header stamp 同步容差，秒")
    parser.add_argument("--window-before", type=int, default=2, help="时序窗口向前看多少个抽样帧")
    parser.add_argument("--window-after", type=int, default=2, help="时序窗口向后看多少个抽样帧")
    parser.add_argument("--xy-gate", type=float, default=1.0, help="map->odom 平移一致性阈值，米")
    parser.add_argument("--yaw-gate", type=float, default=12.0, help="map->odom yaw 一致性阈值，度")
    args = parser.parse_args()

    rclpy.init()
    frames = load_frames(args.metrics_csv, args.candidates_csv, args.scenario)
    attach_odom_and_map_odom(frames, args.odom_topic, args.odom_tolerance)

    # 按 bag 内时间排序，避免跨 bag 把不同测试段强行聚到一个窗口里。
    frames_by_bag: dict[str, list[FrameRecord]] = defaultdict(list)
    for frame in frames:
        if frame.odom_pose_base is not None:
            frames_by_bag[frame.bag_path].append(frame)
    for bag_frames in frames_by_bag.values():
        bag_frames.sort(key=lambda f: f.stamp_sec)

    rows: list[dict[str, float | int | str]] = []
    for bag_frames in frames_by_bag.values():
        for index, frame in enumerate(bag_frames):
            selected = selected_candidate(frame)
            if selected is None:
                continue

            selected_support_frames, selected_support_count, selected_median_rank = support_for_seed(
                bag_frames,
                index,
                selected,
                args.window_before,
                args.window_after,
                args.xy_gate,
                args.yaw_gate,
            )
            best_seed, best_support_frames, best_support_count, best_median_rank = best_supported_seed(
                bag_frames,
                index,
                args.window_before,
                args.window_after,
                args.xy_gate,
                args.yaw_gate,
            )

            rows.append(
                {
                    "bag": frame.bag_name,
                    "stamp": frame.stamp,
                    "success": frame.success,
                    "trans_error_m": frame.trans_error_m,
                    "yaw_error_deg": frame.yaw_error_deg,
                    "selected_rank": selected.rank,
                    "selected_support_frames": selected_support_frames,
                    "selected_support_count": selected_support_count,
                    "selected_median_rank": selected_median_rank,
                    "best_seed_rank": best_seed.rank if best_seed is not None else 0,
                    "best_support_frames": best_support_frames,
                    "best_support_count": best_support_count,
                    "best_median_rank": best_median_rank,
                    "selected_map_odom_x": selected.map_odom_x,
                    "selected_map_odom_y": selected.map_odom_y,
                    "selected_map_odom_yaw_deg": selected.map_odom_yaw_deg,
                }
            )

    numeric_keys = [
        "selected_support_frames",
        "selected_support_count",
        "selected_median_rank",
        "best_seed_rank",
        "best_support_frames",
        "best_support_count",
        "best_median_rank",
    ]

    print("group,n," + ",".join(f"median_{key}" for key in numeric_keys))
    for label, success_value in [("success", 1), ("failure", 0)]:
        group_rows = [row for row in rows if row["success"] == success_value]
        values = [label, str(len(group_rows))]
        for key in numeric_keys:
            values.append(f"{median([float(row[key]) for row in group_rows]):.3f}")
        print(",".join(values))

    print("\nframes_sorted_by_error")
    headers = [
        "bag",
        "stamp",
        "success",
        "trans_error_m",
        "yaw_error_deg",
        "selected_rank",
        "selected_support_frames",
        "selected_support_count",
        "selected_median_rank",
        "best_seed_rank",
        "best_support_frames",
        "best_support_count",
        "best_median_rank",
        "selected_map_odom_x",
        "selected_map_odom_y",
        "selected_map_odom_yaw_deg",
    ]
    print(",".join(headers))
    for row in sorted(rows, key=lambda r: float(r["trans_error_m"]), reverse=True):
        print(",".join(str(row[h]) for h in headers))

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
