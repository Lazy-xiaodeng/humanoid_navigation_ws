#!/usr/bin/env python3
import argparse
import html
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from simulate_straight_first_from_bags import Grid, max_deviation, path_len, straight_first


TOPICS = {"/app/navigation_command", "/global_costmap/costmap", "/plan", "/robot_realpose"}


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    return reader, types


def analyze_bag(bag_dir, ids_filter, max_cost, allow_unknown):
    reader, types = open_reader(bag_dir)
    commands = []
    plans = []
    poses = []
    grids = []
    last_grid = None

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == "/global_costmap/costmap":
            last_grid = Grid(msg)
            grids.append((stamp_ns, last_grid))
        elif topic == "/app/navigation_command":
            try:
                cmd = json.loads(msg.data)
            except Exception:
                continue
            if cmd.get("command_type") != "start_multi_point_navigation":
                continue
            ids = cmd.get("waypoint_ids") or ([cmd.get("waypoint_id")] if cmd.get("waypoint_id") else [])
            if ids_filter and not any(i in ids_filter for i in ids):
                continue
            commands.append((stamp_ns, ids))
        elif topic == "/plan":
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if len(pts) >= 2:
                plans.append((stamp_ns, pts, last_grid))
        elif topic == "/robot_realpose":
            p = msg.pose.pose.position
            poses.append((stamp_ns, p.x, p.y))

    rows = []
    for i, (cmd_stamp, ids) in enumerate(commands):
        next_stamp = commands[i + 1][0] if i + 1 < len(commands) else cmd_stamp + 30_000_000_000
        plan = next(((s, pts, grid) for s, pts, grid in plans if cmd_stamp <= s <= cmd_stamp + 15_000_000_000), None)
        if plan is None:
            continue
        plan_stamp, old_plan, grid = plan
        if grid is None and grids:
            grid = min(grids, key=lambda item: abs(item[0] - plan_stamp))[1]
        if grid is None:
            continue

        start = old_plan[0]
        goal = old_plan[-1]
        new_plan, mode = straight_first(grid, start, goal, max_cost=max_cost, allow_unknown=allow_unknown)

        actual = [(x, y) for s, x, y in poses if cmd_stamp <= s <= next_stamp]
        trimmed = []
        close_seen = 0
        for point in actual:
            trimmed.append(point)
            if dist(point, goal) < 0.35:
                close_seen += 1
                if close_seen > 20:
                    break
        actual = trimmed or actual

        rows.append({
            "bag": Path(bag_dir).name,
            "ids": ids,
            "mode": mode,
            "old_plan": old_plan,
            "new_plan": new_plan,
            "actual": actual,
            "start": start,
            "goal": goal,
            "plan_dt": (plan_stamp - cmd_stamp) / 1e9,
        })
    return rows


def polyline(points, sx, sy):
    return " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in points)


def render_html(rows, output):
    grouped = {}
    for row in rows:
        grouped.setdefault(row["bag"], []).append(row)

    parts = [
        "<!doctype html><html><head><meta charset='utf-8'><title>StraightFirst plan with actual pose</title>",
        "<style>body{font-family:Arial,sans-serif;margin:24px;color:#222}svg{border:1px solid #ccc;background:#fafafa;margin:12px 0 30px}table{border-collapse:collapse;margin:10px 0 18px}td,th{border:1px solid #ddd;padding:4px 8px;font-size:13px}.new{color:#2ca02c}.actual{color:#1f77b4}.old{color:#d62728}.straight{color:#777}</style>",
        "</head><body><h1>StraightFirst plan with recorded actual pose</h1>",
        "<p><span class='new'>green=new StraightFirst plan</span>, <span class='actual'>blue=recorded robot_realpose</span>, <span class='old'>red=bag original /plan</span>, <span class='straight'>gray=start-goal chord</span>.</p>",
    ]

    for bag, bag_rows in grouped.items():
        all_pts = []
        for row in bag_rows:
            all_pts.extend(row["old_plan"])
            all_pts.extend(row["new_plan"])
            all_pts.extend(row["actual"])
            all_pts.extend([row["start"], row["goal"]])
        min_x = min(p[0] for p in all_pts) - 1.0
        max_x = max(p[0] for p in all_pts) + 1.0
        min_y = min(p[1] for p in all_pts) - 1.0
        max_y = max(p[1] for p in all_pts) + 1.0
        width, height, margin = 1200, 820, 35
        scale = min(width / max(1e-6, max_x - min_x), height / max(1e-6, max_y - min_y))
        canvas_w = int((max_x - min_x) * scale + 2 * margin)
        canvas_h = int((max_y - min_y) * scale + 2 * margin)

        def sx(x):
            return margin + (x - min_x) * scale

        def sy(y):
            return canvas_h - margin - (y - min_y) * scale

        parts.append(f"<h2>{html.escape(bag)}</h2>")
        parts.append("<table><tr><th>ids</th><th>mode</th><th>old_plan_dev</th><th>new_plan_dev</th><th>actual_dev_vs_goal_line</th><th>old_len</th><th>new_len</th><th>actual_len</th></tr>")
        for row in bag_rows:
            parts.append(
                f"<tr><td>{html.escape(','.join(row['ids']))}</td><td>{html.escape(row['mode'])}</td>"
                f"<td>{max_deviation(row['old_plan']):.2f}</td><td>{max_deviation(row['new_plan']):.2f}</td>"
                f"<td>{max_deviation(row['actual']):.2f}</td><td>{path_len(row['old_plan']):.2f}</td>"
                f"<td>{path_len(row['new_plan']):.2f}</td><td>{path_len(row['actual']):.2f}</td></tr>"
            )
        parts.append("</table>")
        parts.append(f"<svg width='{canvas_w}' height='{canvas_h}' viewBox='0 0 {canvas_w} {canvas_h}'>")
        for row in bag_rows:
            label = ",".join(row["ids"])
            parts.append(f"<polyline points='{polyline([row['start'], row['goal']], sx, sy)}' fill='none' stroke='#777' stroke-width='1' stroke-dasharray='5,5'/>")
            parts.append(f"<polyline points='{polyline(row['old_plan'], sx, sy)}' fill='none' stroke='#d62728' stroke-width='1.5' opacity='0.45'/>")
            if row["new_plan"]:
                parts.append(f"<polyline points='{polyline(row['new_plan'], sx, sy)}' fill='none' stroke='#2ca02c' stroke-width='2.6' opacity='0.9'/>")
            if row["actual"]:
                parts.append(f"<polyline points='{polyline(row['actual'], sx, sy)}' fill='none' stroke='#1f77b4' stroke-width='2' opacity='0.78'/>")
            parts.append(f"<circle cx='{sx(row['start'][0]):.1f}' cy='{sy(row['start'][1]):.1f}' r='3' fill='#111'><title>{html.escape(label)} start</title></circle>")
            parts.append(f"<circle cx='{sx(row['goal'][0]):.1f}' cy='{sy(row['goal'][1]):.1f}' r='4' fill='#2ca02c'><title>{html.escape(label)} goal</title></circle>")
            parts.append(f"<text x='{sx(row['goal'][0]) + 5:.1f}' y='{sy(row['goal'][1]) - 5:.1f}' font-size='11'>{html.escape(label)}</text>")
        parts.append("</svg>")

    parts.append("</body></html>")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(parts), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--ids", nargs="*", default=[])
    parser.add_argument("--max-cost", type=int, default=20)
    parser.add_argument("--allow-unknown", action="store_true")
    parser.add_argument("--output", default="workspace_archive/debug_monitor/straight_first_with_actual.html")
    args = parser.parse_args()

    rows = []
    for bag in args.bags:
        rows.extend(analyze_bag(Path(bag), set(args.ids), args.max_cost, args.allow_unknown))

    render_html(rows, Path(args.output))
    print(f"wrote {args.output}")
    print("bag,ids,mode,old_plan_dev,new_plan_dev,actual_dev,old_len,new_len,actual_len")
    for row in rows:
        print(
            f"{row['bag']},{'+'.join(row['ids'])},{row['mode']},"
            f"{max_deviation(row['old_plan']):.3f},{max_deviation(row['new_plan']):.3f},"
            f"{max_deviation(row['actual']):.3f},{path_len(row['old_plan']):.3f},"
            f"{path_len(row['new_plan']):.3f},{path_len(row['actual']):.3f}"
        )


if __name__ == "__main__":
    main()
