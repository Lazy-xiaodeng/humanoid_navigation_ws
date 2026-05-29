#!/usr/bin/env python3
import argparse
import json
import math
from collections import Counter
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def yaw_from_pose(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/pcl_pose", "/localization/ndt_status"]))

    statuses = []
    poses = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp * 1e-9
        if topic == "/localization/ndt_status":
            msg = deserialize_message(data, String)
            try:
                item = json.loads(msg.data)
            except json.JSONDecodeError:
                item = {"raw": msg.data}
            item["_t"] = t
            statuses.append(item)
        elif topic == "/pcl_pose":
            msg = deserialize_message(data, PoseWithCovarianceStamped)
            p = msg.pose.pose.position
            poses.append((t, float(p.x), float(p.y), yaw_from_pose(msg)))
    return statuses, poses


def summarize(name, path):
    statuses, poses = read_bag(path)
    state_counts = Counter(s.get("state", "missing") for s in statuses)
    reason_counts = Counter(s.get("reason", "missing") for s in statuses)

    max_step = 0.0
    max_yaw_step = 0.0
    for prev, cur in zip(poses, poses[1:]):
        _, x0, y0, yaw0 = prev
        _, x1, y1, yaw1 = cur
        max_step = max(max_step, math.hypot(x1 - x0, y1 - y0))
        dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        max_yaw_step = max(max_yaw_step, abs(dyaw))

    def max_field(key):
        values = [float(s[key]) for s in statuses if key in s and isinstance(s[key], (int, float))]
        return max(values) if values else float("nan")

    def first_last_time_for_reason(prefixes):
        selected = [
            s["_t"] for s in statuses
            if any(str(s.get("reason", "")).startswith(prefix) for prefix in prefixes)
        ]
        if not selected:
            return None
        return min(selected), max(selected), len(selected)

    print(f"== {name} ==")
    print(f"bag: {path}")
    print(f"statuses={len(statuses)} poses={len(poses)}")
    print(f"states={dict(state_counts)}")
    print("top_reasons=" + ", ".join(f"{k}:{v}" for k, v in reason_counts.most_common(12)))
    print(f"max_pose_step_xy={max_step:.3f} max_pose_step_yaw={max_yaw_step:.3f}")
    print(
        "max_correction_translation="
        f"{max_field('correction_translation'):.3f} max_correction_yaw={max_field('correction_yaw'):.3f} "
        f"max_fitness={max_field('fitness_score'):.3f}"
    )
    print(
        "max_candidate_delta_last_good_xy="
        f"{max_field('ndt_candidate_delta_last_good_xy'):.3f} "
        f"max_candidate_delta_last_good_yaw={max_field('ndt_candidate_delta_last_good_yaw'):.3f}"
    )
    print(
        "max_freeze_duration="
        f"{max_field('freeze_duration_sec'):.3f}s "
        f"max_freeze_odom_delta_translation={max_field('freeze_odom_delta_translation'):.3f} "
        f"max_freeze_odom_delta_yaw={max_field('freeze_odom_delta_yaw'):.3f}"
    )
    print(
        "max_freeze_anchor_error_xy="
        f"{max_field('freeze_anchor_error_xy'):.3f} "
        f"max_freeze_anchor_error_yaw={max_field('freeze_anchor_error_yaw'):.3f}"
    )
    for label, prefixes in [
        ("rotation_freeze", ["rotation_guard_freeze", "rotation_guard_settle_freeze"]),
        ("recovery_gate", ["rotation_guard_recovery"]),
        ("high_fitness", ["high_fitness", "rotation_guard_high_fitness"]),
        ("pose_jump", ["pose_jump"]),
    ]:
        span = first_last_time_for_reason(prefixes)
        if span:
            first, last, count = span
            print(f"{label}_span: first={first:.3f} last={last:.3f} duration={last-first:.3f}s count={count}")
    if poses:
        print(f"first_pose=({poses[0][1]:.3f},{poses[0][2]:.3f}) last_pose=({poses[-1][1]:.3f},{poses[-1][2]:.3f})")
    print()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", type=Path)
    args = parser.parse_args()
    for bag in args.bags:
        summarize(bag.name, bag)


if __name__ == "__main__":
    main()
