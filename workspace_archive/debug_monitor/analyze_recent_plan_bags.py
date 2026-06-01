#!/usr/bin/env python3
import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/plan",
    "/robot_realpose",
    "/odom",
    "/navigation/status",
    "/app/navigation_command",
}


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_metrics(points):
    if len(points) < 2:
        return None
    length = sum(dist(a, b) for a, b in zip(points, points[1:]))
    start = points[0]
    end = points[-1]
    chord = dist(start, end)
    if chord < 1e-6:
        max_dev = 0.0
    else:
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        max_dev = max(abs(dy * x - dx * y + ex * sy - ey * sx) / chord for x, y in points)
    return {
        "points": len(points),
        "length": length,
        "chord": chord,
        "excess": length - chord,
        "ratio": length / chord if chord > 1e-6 else float("nan"),
        "max_dev": max_dev,
        "start": start,
        "end": end,
    }


def status_summary(text):
    try:
        data = json.loads(text)
    except Exception:
        return text[:120]
    keys = [
        "state",
        "status",
        "current_waypoint_index",
        "current_waypoint_name",
        "target_waypoint_index",
        "target_waypoint_name",
        "distance_to_goal",
    ]
    out = {k: data.get(k) for k in keys if k in data}
    goal = data.get("current_goal") or data.get("target_goal")
    if isinstance(goal, dict):
        for k in ("waypoint_id", "waypoint_name", "name", "x", "y"):
            if k in goal:
                out[f"goal.{k}"] = goal[k]
    return out or data


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, topic_types


def nearest_pose(poses, stamp_ns):
    if not poses:
        return None
    best = min(poses, key=lambda p: abs(p[0] - stamp_ns))
    return best if abs(best[0] - stamp_ns) < 5_000_000_000 else None


def analyze_bag(bag_dir):
    reader, topic_types = open_reader(bag_dir)
    msg_types = {topic: get_message(typ) for topic, typ in topic_types.items() if topic in TOPICS}

    plans = []
    poses = []
    odoms = []
    statuses = []
    commands = []

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        if topic == "/plan":
            points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            metrics = path_metrics(points)
            if metrics:
                metrics["stamp_ns"] = stamp_ns
                plans.append(metrics)
        elif topic == "/robot_realpose":
            poses.append((stamp_ns, msg.pose.pose.position.x, msg.pose.pose.position.y))
        elif topic == "/odom":
            odoms.append((stamp_ns, msg.pose.pose.position.x, msg.pose.pose.position.y))
        elif topic == "/navigation/status":
            statuses.append((stamp_ns, status_summary(msg.data)))
        elif topic == "/app/navigation_command":
            commands.append((stamp_ns, msg.data))

    first_stamp = min([p["stamp_ns"] for p in plans] + [x[0] for x in poses] + [x[0] for x in odoms], default=0)
    for item in plans:
        pose = nearest_pose(poses, item["stamp_ns"])
        odom = nearest_pose(odoms, item["stamp_ns"])
        item["t"] = (item["stamp_ns"] - first_stamp) / 1e9 if first_stamp else 0.0
        if pose:
            item["robot_xy"] = (pose[1], pose[2])
            item["robot_to_plan_start"] = dist(item["robot_xy"], item["start"])
        if odom:
            item["odom_xy"] = (odom[1], odom[2])

    return {
        "bag": str(bag_dir),
        "topic_counts": defaultdict(int),
        "plans": plans,
        "poses": poses,
        "odoms": odoms,
        "statuses": statuses,
        "commands": commands,
        "topic_types": {k: topic_types[k] for k in sorted(topic_types) if k in TOPICS},
    }


def print_report(result, top_n):
    print(f"\n=== {Path(result['bag']).name} ===")
    print("topics:", ", ".join(f"{k}:{v}" for k, v in result["topic_types"].items()))
    print(f"plans={len(result['plans'])} robot_realpose={len(result['poses'])} odom={len(result['odoms'])} "
          f"statuses={len(result['statuses'])} commands={len(result['commands'])}")

    if result["commands"]:
        print("commands:")
        for stamp, cmd in result["commands"][:12]:
            try:
                data = json.loads(cmd)
                ids = data.get("waypoint_ids") or ([data.get("waypoint_id")] if data.get("waypoint_id") else [])
                print(f"  {stamp}: {data.get('command_type')} ids={ids}")
            except Exception:
                print(f"  {stamp}: {cmd[:180]}")

    if result["statuses"]:
        print("status samples:")
        step = max(1, len(result["statuses"]) // 6)
        for stamp, status in result["statuses"][::step][:7]:
            print(f"  {stamp}: {status}")

    plans = result["plans"]
    if not plans:
        return
    if result["commands"]:
        print("plan after command:")
        for stamp, cmd in result["commands"]:
            try:
                data = json.loads(cmd)
            except Exception:
                continue
            if data.get("command_type") != "start_multi_point_navigation":
                continue
            ids = data.get("waypoint_ids") or ([data.get("waypoint_id")] if data.get("waypoint_id") else [])
            next_plans = [p for p in plans if 0 <= (p["stamp_ns"] - stamp) / 1e9 <= 15.0]
            if not next_plans:
                continue
            p = next_plans[0]
            print(
                f"  ids={ids} dt={(p['stamp_ns'] - stamp)/1e9:.2f}s len={p['length']:.2f} chord={p['chord']:.2f} "
                f"max_dev={p['max_dev']:.2f} start=({p['start'][0]:.2f},{p['start'][1]:.2f}) "
                f"end=({p['end'][0]:.2f},{p['end'][1]:.2f})"
            )
    by_curvy = sorted(plans, key=lambda p: (p["max_dev"], p["excess"]), reverse=True)[:top_n]
    print("curviest plans:")
    for p in by_curvy:
        rps = p.get("robot_to_plan_start", float("nan"))
        print(
            f"  t={p['t']:.1f}s pts={p['points']:3d} len={p['length']:.2f} chord={p['chord']:.2f} "
            f"excess={p['excess']:.2f} ratio={p['ratio']:.3f} max_dev={p['max_dev']:.2f} "
            f"start=({p['start'][0]:.2f},{p['start'][1]:.2f}) end=({p['end'][0]:.2f},{p['end'][1]:.2f}) "
            f"robot_to_start={rps:.2f}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--top", type=int, default=12)
    args = parser.parse_args()

    for bag in args.bags:
        print_report(analyze_bag(Path(bag)), args.top)


if __name__ == "__main__":
    main()
