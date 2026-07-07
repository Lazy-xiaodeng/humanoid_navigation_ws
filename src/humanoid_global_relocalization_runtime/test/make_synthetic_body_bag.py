#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_synthetic_body_bag.py

文件作用：
  1. 这是验证辅助脚本，不属于功能源码。
  2. 从已有 /fast_lio/cloud_registered + /odom bag 中抽取少量点云帧。
  3. 使用同帧 /odom 把 camera_init raw world 点云反变换成 raw body 点云，并写成 /cloud_registered_body。
  4. 同步写入最近的 /robot_realpose，生成一个很小的派生 bag，用于验证 body 输入链路。

重要说明：
  - 该脚本生成的是“合成 body bag”，用于验证本包 body 分支和坐标处理逻辑。
  - 它不能替代真实 Fast-LIO 发布 /cloud_registered_body 的实测 bag。
  - 输出目录建议放在当前工作空间 .codex_tmp 下，避免污染原始数据目录。
"""

from __future__ import annotations

import argparse
import math
import shutil
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


@dataclass
class OdomSample:
    stamp_sec: float
    bag_time_ns: int
    msg: object


@dataclass
class ReferenceSample:
    bag_time_sec: float
    bag_time_ns: int
    msg: object


@dataclass
class CloudSample:
    stamp_sec: float
    bag_time_sec: float
    bag_time_ns: int
    msg: object


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_to_rotation_matrix(q_msg) -> np.ndarray:
    """把 ROS quaternion 转成 3x3 旋转矩阵，用于 odom world->body 反变换。"""
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


def nearest_by_stamp(samples: list[OdomSample], stamp_sec: float, tolerance_sec: float) -> OdomSample | None:
    """按 header stamp 找最近 odom；registered_world 点云和 odom 使用同一时间基准。"""
    best: OdomSample | None = None
    best_dt = float("inf")
    for sample in samples:
        dt = abs(sample.stamp_sec - stamp_sec)
        if dt < best_dt:
            best = sample
            best_dt = dt
    if best is None or best_dt > tolerance_sec:
        return None
    return best


def nearest_reference(samples: list[ReferenceSample], bag_time_sec: float, tolerance_sec: float) -> ReferenceSample | None:
    """按 bag receive time 找最近参考位姿，和 C++ evaluator 的同步策略保持一致。"""
    best: ReferenceSample | None = None
    best_dt = float("inf")
    for sample in samples:
        dt = abs(sample.bag_time_sec - bag_time_sec)
        if dt < best_dt:
            best = sample
            best_dt = dt
    if best is None or best_dt > tolerance_sec:
        return None
    return best


def world_cloud_to_body_cloud(cloud_msg, odom_msg):
    """执行 p_body = inverse(T_world_body) * p_world，并生成 /cloud_registered_body 消息。"""
    points_world = pc2.read_points_numpy(cloud_msg, field_names=["x", "y", "z"], skip_nans=True).astype(np.float64)
    rotation_world_body = quaternion_to_rotation_matrix(odom_msg.pose.pose.orientation)
    translation_world_body = np.array(
        [
            float(odom_msg.pose.pose.position.x),
            float(odom_msg.pose.pose.position.y),
            float(odom_msg.pose.pose.position.z),
        ],
        dtype=np.float64,
    )
    points_body = (points_world - translation_world_body) @ rotation_world_body

    header = Header()
    header.stamp = cloud_msg.header.stamp
    header.frame_id = "body"
    return pc2.create_cloud_xyz32(header, points_body.astype(np.float32))


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def open_writer(output_path: str, overwrite: bool) -> rosbag2_py.SequentialWriter:
    out = Path(output_path)
    if out.exists():
        if not overwrite:
            raise FileExistsError(f"output bag already exists: {output_path}")
        shutil.rmtree(out)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            0,
            "/cloud_registered_body",
            "sensor_msgs/msg/PointCloud2",
            "cdr",
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            1,
            "/odom",
            "nav_msgs/msg/Odometry",
            "cdr",
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            2,
            "/robot_realpose",
            "geometry_msgs/msg/PoseWithCovarianceStamped",
            "cdr",
        )
    )
    return writer


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a tiny synthetic /cloud_registered_body bag.")
    parser.add_argument("--input-bag", required=True, help="原始 rosbag2 目录")
    parser.add_argument("--output-bag", required=True, help="输出派生 rosbag2 目录")
    parser.add_argument("--skip", type=int, default=50, help="跳过开头多少帧 /fast_lio/cloud_registered")
    parser.add_argument("--stride", type=int, default=500, help="抽帧间隔")
    parser.add_argument("--max-frames", type=int, default=5, help="最多生成多少帧 body 点云")
    parser.add_argument("--odom-tolerance", type=float, default=0.05, help="点云和 odom header 时间同步容差，秒")
    parser.add_argument("--reference-tolerance", type=float, default=0.05, help="点云和 /robot_realpose bag time 同步容差，秒")
    parser.add_argument("--overwrite", action="store_true", help="允许覆盖已有输出 bag")
    args = parser.parse_args()

    rclpy.init()
    point_cloud_type = get_message("sensor_msgs/msg/PointCloud2")
    odom_type = get_message("nav_msgs/msg/Odometry")
    reference_type = get_message("geometry_msgs/msg/PoseWithCovarianceStamped")

    odoms: list[OdomSample] = []
    references: list[ReferenceSample] = []
    clouds: list[CloudSample] = []

    reader = open_reader(args.input_bag)
    cloud_seen = 0
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        bag_time_sec = float(bag_time_ns) * 1e-9
        if topic == "/odom":
            msg = deserialize_message(data, odom_type)
            odoms.append(OdomSample(stamp_to_sec(msg.header.stamp), bag_time_ns, msg))
            continue
        if topic == "/robot_realpose":
            msg = deserialize_message(data, reference_type)
            references.append(ReferenceSample(bag_time_sec, bag_time_ns, msg))
            continue
        if topic != "/fast_lio/cloud_registered":
            continue

        sample_index = cloud_seen - max(0, args.skip)
        if sample_index >= 0 and sample_index % max(1, args.stride) == 0 and len(clouds) < args.max_frames:
            msg = deserialize_message(data, point_cloud_type)
            clouds.append(CloudSample(stamp_to_sec(msg.header.stamp), bag_time_sec, bag_time_ns, msg))
        cloud_seen += 1

    if not clouds:
        raise RuntimeError("no sampled /fast_lio/cloud_registered frames; check skip/stride/max-frames")

    writer = open_writer(args.output_bag, args.overwrite)
    written = 0
    skipped_no_odom = 0
    skipped_no_reference = 0
    for cloud in clouds:
        odom = nearest_by_stamp(odoms, cloud.stamp_sec, args.odom_tolerance)
        if odom is None:
            skipped_no_odom += 1
            continue
        reference = nearest_reference(references, cloud.bag_time_sec, args.reference_tolerance)
        if reference is None:
            skipped_no_reference += 1
            continue

        body_cloud = world_cloud_to_body_cloud(cloud.msg, odom.msg)
        writer.write("/cloud_registered_body", serialize_message(body_cloud), cloud.bag_time_ns)
        writer.write("/odom", serialize_message(odom.msg), odom.bag_time_ns)
        writer.write("/robot_realpose", serialize_message(reference.msg), reference.bag_time_ns)
        written += 1

    rclpy.shutdown()
    print(
        "synthetic_body_bag "
        f"input={args.input_bag} output={args.output_bag} "
        f"sampled={len(clouds)} written={written} "
        f"skipped_no_odom={skipped_no_odom} skipped_no_reference={skipped_no_reference}"
    )
    if written == 0:
        raise RuntimeError("no frames written; odom/reference synchronization failed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
