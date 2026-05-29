#!/usr/bin/env python3
import argparse
from bisect import bisect_left
from math import atan2, cos, sin
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quat(q):
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_tracks(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/odom", "/tf"]))

    odom = []
    map_to_odom = []
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        if topic == "/odom":
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            t = stamp_to_sec(msg.header.stamp) or bag_stamp * 1e-9
            odom.append((t, float(p.x), float(p.y)))
            continue

        msg = deserialize_message(data, TFMessage)
        for tf in msg.transforms:
            if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
                tr = tf.transform.translation
                yaw = yaw_from_quat(tf.transform.rotation)
                t = stamp_to_sec(tf.header.stamp) or bag_stamp * 1e-9
                map_to_odom.append((t, float(tr.x), float(tr.y), yaw))

    return odom, map_to_odom


def nearest_tf(tf_track, t):
    times = [item[0] for item in tf_track]
    idx = bisect_left(times, t)
    if idx <= 0:
        return tf_track[0]
    if idx >= len(tf_track):
        return tf_track[-1]
    before = tf_track[idx - 1]
    after = tf_track[idx]
    return before if abs(t - before[0]) <= abs(after[0] - t) else after


def transform_odom_with_tf(odom, map_to_odom):
    corrected = []
    for t, x, y in odom:
        _, tx, ty, yaw = nearest_tf(map_to_odom, t)
        c = cos(yaw)
        s = sin(yaw)
        corrected.append((t, tx + c * x - s * y, ty + s * x + c * y))
    return corrected


def initial_align(reference_xy, target_xy):
    if not reference_xy or not target_xy:
        return target_xy
    dx = reference_xy[0][0] - target_xy[0][0]
    dy = reference_xy[0][1] - target_xy[0][1]
    return [(x + dx, y + dy) for x, y in target_xy]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, default=Path("odom_corrected_by_map_odom.png"))
    args = parser.parse_args()

    odom, map_to_odom = read_tracks(args.bag)
    if not odom:
        raise RuntimeError("No /odom messages found")
    if not map_to_odom:
        raise RuntimeError("No map -> odom TF messages found")

    corrected = transform_odom_with_tf(odom, map_to_odom)
    corrected_xy = [(x, y) for _, x, y in corrected]
    odom_xy = [(x, y) for _, x, y in odom]
    odom_aligned = initial_align(corrected_xy, odom_xy)

    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=150)
    ax.plot([p[0] for p in odom_aligned], [p[1] for p in odom_aligned], color="#1f77b4", linewidth=2.0, label="/odom initial aligned")
    ax.plot([p[0] for p in corrected_xy], [p[1] for p in corrected_xy], color="#d62728", linewidth=2.0, label="corrected by map->odom")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, color="#dfe6ee", linewidth=0.8)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(args.output)

    print(f"wrote {args.output}")
    print(f"/odom: {len(odom)} points")
    print(f"map->odom tf: {len(map_to_odom)} transforms")
    print(f"corrected: {len(corrected)} points")


if __name__ == "__main__":
    main()
