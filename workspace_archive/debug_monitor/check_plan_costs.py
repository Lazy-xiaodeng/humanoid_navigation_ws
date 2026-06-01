#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {"/plan", "/app/navigation_command", "/global_costmap/costmap"}


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_metrics(points):
    length = sum(dist(a, b) for a, b in zip(points, points[1:]))
    chord = dist(points[0], points[-1])
    return length, chord


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, {topic: get_message(typ) for topic, typ in topic_types.items() if topic in TOPICS}


def grid_cost(grid, x, y):
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y
    ix = int((x - ox) / res)
    iy = int((y - oy) / res)
    if ix < 0 or iy < 0 or ix >= grid.info.width or iy >= grid.info.height:
        return None
    return grid.data[iy * grid.info.width + ix]


def sample_line(a, b, step=0.05):
    n = max(2, int(dist(a, b) / step) + 1)
    for i in range(n):
        t = i / (n - 1)
        yield (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def summarize_costs(grid, points):
    vals = [grid_cost(grid, x, y) for x, y in points]
    valid = [v for v in vals if v is not None]
    if not valid:
        return "out_of_grid"
    unknown = sum(1 for v in valid if v < 0)
    high = sum(1 for v in valid if v >= 80)
    med = sum(1 for v in valid if 40 <= v < 80)
    return {
        "samples": len(valid),
        "unknown": unknown,
        "high>=80": high,
        "med40-79": med,
        "max": max(valid),
        "mean_known": sum(v for v in valid if v >= 0) / max(1, sum(1 for v in valid if v >= 0)),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--ids", nargs="*", default=["1089", "1091"])
    args = parser.parse_args()

    reader, msg_types = open_reader(Path(args.bag))
    last_grid = None
    pending = []
    reports = []

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        msg = deserialize_message(data, msg_types[topic])
        if topic == "/global_costmap/costmap":
            last_grid = msg
        elif topic == "/app/navigation_command":
            try:
                cmd = json.loads(msg.data)
            except Exception:
                continue
            ids = cmd.get("waypoint_ids") or ([cmd.get("waypoint_id")] if cmd.get("waypoint_id") else [])
            if cmd.get("command_type") == "start_multi_point_navigation" and any(i in args.ids for i in ids):
                pending.append((stamp_ns, ids))
        elif topic == "/plan" and pending:
            for cmd_stamp, ids in list(pending):
                dt = (stamp_ns - cmd_stamp) / 1e9
                if dt < 0:
                    continue
                if dt > 15:
                    pending.remove((cmd_stamp, ids))
                    continue
                pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
                if len(pts) < 2:
                    continue
                length, chord = path_metrics(pts)
                reports.append((ids, dt, pts[0], pts[-1], length, chord, last_grid, pts))
                pending.remove((cmd_stamp, ids))
                break

    print(f"=== {Path(args.bag).name} ===")
    for ids, dt, start, end, length, chord, grid, pts in reports:
        print(f"ids={ids} dt={dt:.2f}s start=({start[0]:.2f},{start[1]:.2f}) end=({end[0]:.2f},{end[1]:.2f}) len={length:.2f} chord={chord:.2f}")
        if grid is None:
            print("  no global costmap before plan")
            continue
        print(f"  straight costs: {summarize_costs(grid, list(sample_line(start, end)))}")
        print(f"  plan costs    : {summarize_costs(grid, pts)}")


if __name__ == "__main__":
    main()
