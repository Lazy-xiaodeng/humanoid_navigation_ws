#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
inspect_bag_timestamps.py

文件作用：
  1. 这是验证辅助脚本，不属于功能源码。
  2. 用于检查 rosbag2 中点云、odom、robot_realpose 的 header stamp 与 bag receive time 的关系。
  3. 当离线评估无法匹配参考位姿时，先用它确认到底应该按哪一种时间同步。

使用示例：
  source /opt/ros/jazzy/setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/inspect_bag_timestamps.py \
    /home/ubuntu/nav_drift_test/nav_drift_test45
"""

from __future__ import annotations

import sys
from typing import Iterable

import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/fast_lio/cloud_registered": "sensor_msgs/msg/PointCloud2",
    "/cloud_registered_body": "sensor_msgs/msg/PointCloud2",
    "/odom": "nav_msgs/msg/Odometry",
    "/robot_realpose": "geometry_msgs/msg/PoseWithCovarianceStamped",
}


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def main(argv: Iterable[str]) -> int:
    args = list(argv)
    if len(args) != 2:
        print("usage: inspect_bag_timestamps.py <bag_dir>", file=sys.stderr)
        return 2

    rclpy.init()
    reader = open_reader(args[1])
    type_map = {topic: get_message(type_name) for topic, type_name in TOPICS.items()}
    seen = {topic: 0 for topic in TOPICS}

    while reader.has_next() and any(count < 5 for count in seen.values()):
      topic, data, bag_time_ns = reader.read_next()
      if topic not in TOPICS or seen[topic] >= 5:
          continue
      msg = deserialize_message(data, type_map[topic])
      header_stamp = stamp_to_sec(msg.header.stamp)
      bag_time = float(bag_time_ns) * 1e-9
      print(
          f"{topic} index={seen[topic]} header={header_stamp:.9f} "
          f"bag_time={bag_time:.9f} delta_bag_header={bag_time - header_stamp:.9f} "
          f"frame_id={msg.header.frame_id}"
      )
      seen[topic] += 1

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
