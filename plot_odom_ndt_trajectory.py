#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def message_type_map(reader):
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def xy_from_msg(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        p = msg.pose.pose.position
    elif hasattr(msg, "pose"):
        p = msg.pose.position
    else:
        raise TypeError(f"Unsupported message shape: {type(msg)!r}")
    return float(p.x), float(p.y)


def read_xy_tracks(bag_path, topics):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    type_by_topic = message_type_map(reader)
    missing = [topic for topic in topics if topic not in type_by_topic]
    if missing:
        raise RuntimeError(f"Missing topic(s): {', '.join(missing)}")

    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))

    wanted_types = {topic: get_message(type_by_topic[topic]) for topic in topics}
    tracks = {topic: {"t": [], "x": [], "y": []} for topic in topics}

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in wanted_types:
            continue
        msg = deserialize_message(data, wanted_types[topic])
        x, y = xy_from_msg(msg)
        tracks[topic]["t"].append(stamp * 1e-9)
        tracks[topic]["x"].append(x)
        tracks[topic]["y"].append(y)

    return tracks


def align_to_first_point(reference, target):
    if not reference["x"] or not target["x"]:
        return target["x"], target["y"]
    dx = reference["x"][0] - target["x"][0]
    dy = reference["y"][0] - target["y"][0]
    return [x + dx for x in target["x"]], [y + dy for y in target["y"]]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--ndt-topic", default="/pcl_pose")
    parser.add_argument("--output", type=Path, default=Path("odom_vs_ndt_trajectory.png"))
    parser.add_argument("--align-first", action="store_true", help="Translate odom so its first point matches NDT.")
    args = parser.parse_args()

    topics = [args.odom_topic, args.ndt_topic]
    tracks = read_xy_tracks(args.bag, topics)
    odom = tracks[args.odom_topic]
    ndt = tracks[args.ndt_topic]

    odom_x, odom_y = odom["x"], odom["y"]
    if args.align_first:
        odom_x, odom_y = align_to_first_point(ndt, odom)

    fig, ax = plt.subplots(figsize=(12.8, 7.2), dpi=150)
    ax.plot(odom_x, odom_y, color="#1f77b4", linewidth=2.0, label=f"{args.odom_topic} odom")
    ax.plot(ndt["x"], ndt["y"], color="#d62728", linewidth=2.0, label=f"{args.ndt_topic} NDT")

    ax.set_title("Odom vs NDT Trajectory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, color="#dfe6ee", linewidth=0.8)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(args.output)

    print(f"wrote {args.output}")
    print(f"{args.odom_topic}: {len(odom['x'])} points")
    print(f"{args.ndt_topic}: {len(ndt['x'])} points")
    if odom["x"] and ndt["x"]:
        print(f"first odom: ({odom['x'][0]:.3f}, {odom['y'][0]:.3f})")
        print(f"first ndt:  ({ndt['x'][0]:.3f}, {ndt['y'][0]:.3f})")


if __name__ == "__main__":
    main()
