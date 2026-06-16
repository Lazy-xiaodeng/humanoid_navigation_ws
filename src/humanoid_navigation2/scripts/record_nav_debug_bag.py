#!/usr/bin/env python3
"""导航/避障调参专用 rosbag2 录制脚本。

默认 profile=iter_obstacle：
  只录本轮“误报 / 撞人不停 / 障碍等待恢复 / 左右乱转失败”最需要的话题，
  尽量减小录包体积，方便多次迭代。

可选 profile=full：
  保留现有全量导航调试口径，适合需要完整回放时使用。
"""

from __future__ import annotations

import argparse
import os
import shlex
import sys
from typing import Iterable, List


CORE_TOPICS = [
    "/tf",
    "/tf_static",
    "/rosout",
    "/cmd_vel",
    "/goal_pose",
    "/initialpose",
    "/plan",
    "/robot_realpose",
    "/robot_speed",
    "/odom",
]

NAV_STATE_TOPICS = [
    "/navigation/status",
    "/navigation/acknowledgments",
    "/navigation/waypoints_data",
    "/app/navigation_command",
    "/app/waypoint_command",
    "/map/status",
]

LOCALIZATION_TOPICS = [
    "/localization/prior_map_odom_bridge_status",
    "/localization/recovery_status",
    "/prior_localization/pose",
    "/prior_localization/pose_with_covariance",
    "/prior_localization/odom",
    "/prior_localization/confidence",
]

ACTION_TOPICS = [
    "/navigate_to_pose/_action/status",
    "/navigate_to_pose/_action/feedback",
    "/follow_path/_action/status",
    "/follow_path/_action/feedback",
    "/spin/_action/status",
    "/spin/_action/feedback",
    "/behavior_tree_log",
]

SENSOR_TOPICS = [
    "/fast_lio/cloud_registered",
    "/airy_points_filtered",
    "/airy_points",
    "/airy_imu",
]

COSTMAP_TOPICS = [
    "/local_costmap/costmap",
    "/global_costmap/costmap",
    "/local_costmap/costmap_updates",
    "/global_costmap/costmap_updates",
    "/local_costmap/costmap_raw",
    "/global_costmap/costmap_raw",
    "/local_costmap/published_footprint",
    "/global_costmap/published_footprint",
    "/local_costmap/voxel_grid",
    "/clearing_cloud_3d",
    "/clearing_scan",
]

ROI_TOPICS = [
    "/front_obstacle/has_obstacle",
    "/front_obstacle/debug",
    "/front_obstacle/roi_cloud",
]

OPEN3D_EXTRA_TOPICS = [
    "/prior_localization/open3d_input_odom",
    "/prior_localization/open3d_input_cloud",
    "/prior_localization/open3d_odom2map",
    "/prior_localization/open3d_odom2map_kalman",
    "/prior_localization/open3d_baselink2map_kalman",
    "/prior_localization/open3d_motionlink2map",
    "/prior_localization/open3d_scan",
    "/prior_localization/open3d_submap",
    "/prior_localization/open3d_scan2map",
]


def dedupe(items: Iterable[str]) -> List[str]:
    seen = set()
    ordered: List[str] = []
    for item in items:
        if item not in seen:
            seen.add(item)
            ordered.append(item)
    return ordered


def topics_for_profile(profile: str) -> List[str]:
    if profile == "iter_obstacle":
        return dedupe(
            CORE_TOPICS
            + NAV_STATE_TOPICS
            + LOCALIZATION_TOPICS
            + ACTION_TOPICS
            + SENSOR_TOPICS
            + COSTMAP_TOPICS
            + ROI_TOPICS
        )

    if profile == "iter_min":
        return dedupe(
            CORE_TOPICS
            + NAV_STATE_TOPICS
            + [
                "/localization/prior_map_odom_bridge_status",
                "/prior_localization/odom",
            ]
            + [
                "/follow_path/_action/status",
                "/follow_path/_action/feedback",
                "/spin/_action/status",
                "/spin/_action/feedback",
                "/behavior_tree_log",
            ]
            + SENSOR_TOPICS
            + [
                "/local_costmap/costmap_raw",
                "/local_costmap/voxel_grid",
                "/local_costmap/published_footprint",
                "/clearing_cloud_3d",
                "/clearing_scan",
            ]
            + ROI_TOPICS
        )

    if profile == "full":
        return dedupe(
            CORE_TOPICS
            + NAV_STATE_TOPICS
            + LOCALIZATION_TOPICS
            + OPEN3D_EXTRA_TOPICS
            + ACTION_TOPICS
            + SENSOR_TOPICS
            + COSTMAP_TOPICS
            + ROI_TOPICS
        )

    raise ValueError(f"unknown profile: {profile}")


def build_command(output: str, topics: List[str], compression_mode: str, compression_format: str) -> List[str]:
    command = ["ros2", "bag", "record", "-o", output]
    if compression_mode != "none":
        command.extend(["--compression-mode", compression_mode, "--compression-format", compression_format])
    command.extend(topics)
    return command


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="导航/避障调参专用 rosbag 录制脚本")
    parser.add_argument("-o", "--output", default="nav_debug_iter", help="输出 bag 名称")
    parser.add_argument(
        "--profile",
        choices=["iter_obstacle", "iter_min", "full"],
        default="iter_obstacle",
        help="录包话题档位：iter_obstacle=本轮避障迭代推荐；iter_min=更小；full=完整",
    )
    parser.add_argument(
        "--compression-mode",
        choices=["none", "file", "message"],
        default="file",
        help="默认按文件压缩，减小体积",
    )
    parser.add_argument(
        "--compression-format",
        choices=["zstd"],
        default="zstd",
        help="压缩格式",
    )
    parser.add_argument(
        "--extra-topic",
        action="append",
        default=[],
        help="额外补录的话题，可重复传入",
    )
    parser.add_argument(
        "--print-topics",
        action="store_true",
        help="只打印最终话题列表，不启动录制",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印最终命令，不启动录制",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    topics = dedupe(topics_for_profile(args.profile) + args.extra_topic)
    command = build_command(args.output, topics, args.compression_mode, args.compression_format)

    print(f"profile: {args.profile}")
    print(f"topic_count: {len(topics)}")

    if args.print_topics:
        for topic in topics:
            print(topic)
        return 0

    print("command:")
    print(shlex.join(command))

    if args.dry_run:
        return 0

    os.execvp(command[0], command)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
