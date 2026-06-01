#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {"/app/navigation_command", "/plan", "/robot_realpose", "/cmd_vel"}


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_norm(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def path_len(points):
    return sum(dist(a, b) for a, b in zip(points, points[1:])) if len(points) > 1 else 0.0


def max_dev(points, start=None, end=None):
    if len(points) < 2:
        return 0.0
    start = start or points[0]
    end = end or points[-1]
    chord = dist(start, end)
    if chord < 1e-9:
        return 0.0
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    return max(abs(dy * x - dx * y + ex * sy - ey * sx) / chord for x, y in points)


def nearest_pose(poses, stamp, max_dt_ns=1_000_000_000):
    if not poses:
        return None
    p = min(poses, key=lambda x: abs(x[0] - stamp))
    return p if abs(p[0] - stamp) <= max_dt_ns else None


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    return reader, types


def read_bag(bag_dir):
    reader, types = open_reader(bag_dir)
    commands, plans, poses, cmds = [], [], [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == "/app/navigation_command":
            try:
                obj = json.loads(msg.data)
            except Exception:
                continue
            ids = obj.get("waypoint_ids") or ([obj.get("waypoint_id")] if obj.get("waypoint_id") else [])
            commands.append((stamp, obj.get("command_type"), ids))
        elif topic == "/plan":
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if len(pts) >= 2:
                plans.append((stamp, pts))
        elif topic == "/robot_realpose":
            p = msg.pose.pose.position
            poses.append((stamp, p.x, p.y, yaw_from_quat(msg.pose.pose.orientation)))
        elif topic == "/cmd_vel":
            cmds.append((stamp, msg.linear.x, msg.angular.z))
    return commands, plans, poses, cmds


def analyze_segment(commands, plans, poses, cmds, target_ids):
    starts = [(s, ids) for s, typ, ids in commands if typ == "start_multi_point_navigation" and any(i in target_ids for i in ids)]
    rows = []
    for idx, (start_stamp, ids) in enumerate(starts):
        # Segment ends at the next start command in the whole command stream, not only target IDs.
        next_starts = [s for s, typ, _ in commands if typ == "start_multi_point_navigation" and s > start_stamp]
        end_stamp = min(next_starts) if next_starts else start_stamp + 60_000_000_000
        seg_plans = [(s, p) for s, p in plans if start_stamp <= s < end_stamp]
        seg_cmds = [(s, vx, wz) for s, vx, wz in cmds if start_stamp <= s < end_stamp]
        if not seg_plans:
            continue

        first_goal = seg_plans[0][1][-1]
        plan_rows = []
        prev_start = None
        prev_shape = None
        for s, pts in seg_plans:
            pose = nearest_pose(poses, s)
            robot_xy = (pose[1], pose[2]) if pose else None
            robot_yaw = pose[3] if pose else None
            start_xy = pts[0]
            goal_xy = pts[-1]
            heading = math.atan2(goal_xy[1] - start_xy[1], goal_xy[0] - start_xy[0])
            yaw_err = angle_norm(heading - robot_yaw) if robot_yaw is not None else float("nan")
            start_shift = dist(start_xy, prev_start) if prev_start else 0.0
            # Shape change: compare max deviation from this plan's start-goal line.
            dev = max_dev(pts, start_xy, goal_xy)
            shape_delta = abs(dev - prev_shape) if prev_shape is not None else 0.0
            plan_rows.append({
                "stamp": s,
                "t": (s - start_stamp) / 1e9,
                "points": len(pts),
                "start": start_xy,
                "goal": goal_xy,
                "len": path_len(pts),
                "chord": dist(start_xy, goal_xy),
                "dev": dev,
                "robot_xy": robot_xy,
                "robot_to_start": dist(robot_xy, start_xy) if robot_xy else float("nan"),
                "yaw_err_deg": math.degrees(yaw_err),
                "start_shift": start_shift,
                "shape_delta": shape_delta,
            })
            prev_start = start_xy
            prev_shape = dev

        intervals = [(b[0] - a[0]) / 1e9 for a, b in zip(seg_plans, seg_plans[1:])]
        rows.append({
            "ids": ids,
            "start_stamp": start_stamp,
            "duration": (end_stamp - start_stamp) / 1e9,
            "plans": plan_rows,
            "intervals": intervals,
            "cmd_count": len(seg_cmds),
        })
    return rows


def render_html(rows, output):
    parts = [
        "<!doctype html><html><head><meta charset='utf-8'><title>Replanning loop analysis</title>",
        "<style>body{font-family:Arial,sans-serif;margin:24px;color:#222}svg{border:1px solid #ccc;background:#fafafa;margin:10px 0 24px}table{border-collapse:collapse;margin:8px 0 18px}td,th{border:1px solid #ddd;padding:4px 8px;font-size:13px}.old{opacity:.25}</style>",
        "</head><body><h1>Replanning Loop Analysis</h1>",
        "<p>Each colored polyline is one /plan in the same navigation segment. Black dots are plan starts.</p>",
    ]
    palette = ["#d62728", "#1f77b4", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf", "#8c564b", "#e377c2"]
    for seg in rows:
        all_pts = []
        for p in seg["plans"]:
            all_pts.extend([p["start"], p["goal"]])
        min_x = min(p[0] for p in all_pts) - 1
        max_x = max(p[0] for p in all_pts) + 1
        min_y = min(p[1] for p in all_pts) - 1
        max_y = max(p[1] for p in all_pts) + 1
        scale = min(1000 / max(1e-6, max_x - min_x), 700 / max(1e-6, max_y - min_y))
        margin = 35
        cw = int((max_x - min_x) * scale + 2 * margin)
        ch = int((max_y - min_y) * scale + 2 * margin)

        def sx(x): return margin + (x - min_x) * scale
        def sy(y): return ch - margin - (y - min_y) * scale
        def poly(points): return " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in points)

        title = ",".join(seg["ids"])
        parts.append(f"<h2>ids={title}</h2>")
        parts.append("<table><tr><th>#</th><th>t</th><th>len</th><th>chord</th><th>dev</th><th>robot_to_start</th><th>start_shift</th><th>yaw_err_deg</th></tr>")
        for i, p in enumerate(seg["plans"]):
            parts.append(
                f"<tr><td>{i}</td><td>{p['t']:.2f}</td><td>{p['len']:.2f}</td><td>{p['chord']:.2f}</td>"
                f"<td>{p['dev']:.2f}</td><td>{p['robot_to_start']:.2f}</td><td>{p['start_shift']:.2f}</td><td>{p['yaw_err_deg']:.1f}</td></tr>"
            )
        parts.append("</table>")
        parts.append(f"<svg width='{cw}' height='{ch}' viewBox='0 0 {cw} {ch}'>")
        for i, p in enumerate(seg["plans"]):
            color = palette[i % len(palette)]
            opacity = 0.35 if i < len(seg["plans"]) - 1 else 0.95
            parts.append(f"<polyline points='{poly([p['start'], p['goal']])}' fill='none' stroke='#999' stroke-width='1' stroke-dasharray='4,4' opacity='{opacity}'/>")
            # Only start-goal line is enough to show start movement in this script; detailed path is not retained.
            parts.append(f"<circle cx='{sx(p['start'][0]):.1f}' cy='{sy(p['start'][1]):.1f}' r='3' fill='{color}' opacity='{opacity}'><title>plan {i} start</title></circle>")
            parts.append(f"<circle cx='{sx(p['goal'][0]):.1f}' cy='{sy(p['goal'][1]):.1f}' r='3' fill='#2ca02c' opacity='{opacity}'><title>plan {i} goal</title></circle>")
        parts.append("</svg>")
    parts.append("</body></html>")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\\n".join(parts), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--ids", nargs="+", required=True)
    parser.add_argument("--html", default="")
    args = parser.parse_args()
    commands, plans, poses, cmds = read_bag(Path(args.bag))
    rows = analyze_segment(commands, plans, poses, cmds, set(args.ids))
    print(f"=== {Path(args.bag).name} ===")
    for seg in rows:
        intervals = seg["intervals"]
        print(f"ids={seg['ids']} plans={len(seg['plans'])} duration={seg['duration']:.1f}s")
        if intervals:
            print(f"  intervals min/mean/max={min(intervals):.2f}/{sum(intervals)/len(intervals):.2f}/{max(intervals):.2f}s")
        for i, p in enumerate(seg["plans"]):
            print(
                f"  #{i:02d} t={p['t']:.2f}s pts={p['points']} len={p['len']:.2f} chord={p['chord']:.2f} dev={p['dev']:.2f} "
                f"start=({p['start'][0]:.2f},{p['start'][1]:.2f}) goal=({p['goal'][0]:.2f},{p['goal'][1]:.2f}) "
                f"robot_to_start={p['robot_to_start']:.2f} start_shift={p['start_shift']:.2f} yaw_err={p['yaw_err_deg']:.1f}deg"
            )
    if args.html:
        render_html(rows, Path(args.html))
        print(f"wrote {args.html}")


if __name__ == "__main__":
    main()
