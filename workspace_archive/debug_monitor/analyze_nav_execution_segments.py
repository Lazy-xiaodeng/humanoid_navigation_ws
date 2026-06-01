#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/app/navigation_command",
    "/plan",
    "/robot_realpose",
    "/cmd_vel",
    "/global_costmap/costmap",
    "/local_costmap/costmap",
}


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_norm(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


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


def grid_cost(grid, x, y):
    if grid is None:
        return None
    _, info, data = grid
    ix = int((x - info.origin.position.x) / info.resolution)
    iy = int((y - info.origin.position.y) / info.resolution)
    if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
        return None
    return data[iy * info.width + ix]


def cost_summary(grid, points):
    vals = [grid_cost(grid, x, y) for x, y in points]
    vals = [v for v in vals if v is not None]
    if not vals:
        return "n/a"
    known = [v for v in vals if v >= 0]
    return {
        "max": max(vals),
        "mean": sum(known) / len(known) if known else -1,
        "high": sum(1 for v in vals if v >= 80),
        "unknown": sum(1 for v in vals if v < 0),
    }


def sample_line(a, b, step=0.05):
    n = max(2, int(dist(a, b) / step) + 1)
    return [(a[0] + (b[0] - a[0]) * i / (n - 1), a[1] + (b[1] - a[1]) * i / (n - 1)) for i in range(n)]


def open_reader(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    return reader, types


def analyze(bag_dir, ids_filter):
    reader, types = open_reader(bag_dir)
    commands = []
    plans = []
    poses = []
    cmds = []
    global_maps = []
    local_maps = []

    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == "/app/navigation_command":
            try:
                obj = json.loads(msg.data)
            except Exception:
                continue
            typ = obj.get("command_type")
            ids = obj.get("waypoint_ids") or ([obj.get("waypoint_id")] if obj.get("waypoint_id") else [])
            if typ == "start_multi_point_navigation" and (not ids_filter or any(i in ids_filter for i in ids)):
                commands.append((stamp, ids))
        elif topic == "/plan":
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if len(pts) >= 2:
                plans.append((stamp, pts))
        elif topic == "/robot_realpose":
            p = msg.pose.pose.position
            yaw = yaw_from_quat(msg.pose.pose.orientation)
            poses.append((stamp, p.x, p.y, yaw))
        elif topic == "/cmd_vel":
            cmds.append((stamp, msg.linear.x, msg.angular.z))
        elif topic == "/global_costmap/costmap":
            global_maps.append((stamp, msg.info, list(msg.data)))
        elif topic == "/local_costmap/costmap":
            local_maps.append((stamp, msg.info, list(msg.data)))

    rows = []
    all_command_times = [c[0] for c in commands]
    for i, (cmd_stamp, ids) in enumerate(commands):
        next_stamp = commands[i + 1][0] if i + 1 < len(commands) else cmd_stamp + 30_000_000_000
        plan = next(((s, p) for s, p in plans if cmd_stamp <= s <= cmd_stamp + 15_000_000_000), None)
        if plan is None:
            continue
        plan_stamp, plan_pts = plan
        start, goal = plan_pts[0], plan_pts[-1]
        seg_poses = [(s, x, y, yaw) for s, x, y, yaw in poses if cmd_stamp <= s <= next_stamp]
        # Trim after robot is close to goal, with a small tail to include final approach.
        trimmed = []
        close_seen = 0
        for item in seg_poses:
            trimmed.append(item)
            if dist((item[1], item[2]), goal) < 0.35:
                close_seen += 1
                if close_seen > 20:
                    break
        seg_poses = trimmed or seg_poses
        actual_pts = [(x, y) for _, x, y, _ in seg_poses]
        seg_cmds = [(s, vx, wz) for s, vx, wz in cmds if cmd_stamp <= s <= next_stamp]
        first_cmds = [(s, vx, wz) for s, vx, wz in cmds if cmd_stamp <= s <= cmd_stamp + 4_000_000_000]
        post_plan_cmds = [(s, vx, wz) for s, vx, wz in cmds if plan_stamp <= s <= plan_stamp + 4_000_000_000]
        first_pose = seg_poses[0] if seg_poses else None
        desired_heading = math.atan2(goal[1] - start[1], goal[0] - start[0])
        yaw_err = angle_norm(desired_heading - first_pose[3]) if first_pose else float("nan")
        spin_samples = [1 for _, vx, wz in first_cmds if abs(vx) < 0.05 and abs(wz) > 0.12]
        move_samples = [1 for _, vx, wz in first_cmds if abs(vx) > 0.08]
        nearest_gmap = min(global_maps, key=lambda m: abs(m[0] - plan_stamp), default=None)
        nearest_lmap = min(local_maps, key=lambda m: abs(m[0] - plan_stamp), default=None)
        rows.append({
            "bag": Path(bag_dir).name,
            "ids": ids,
            "plan_dt": (plan_stamp - cmd_stamp) / 1e9,
            "start": start,
            "goal": goal,
            "plan_len": path_len(plan_pts),
            "plan_chord": dist(start, goal),
            "plan_dev": max_dev(plan_pts, start, goal),
            "actual_len": path_len(actual_pts),
            "actual_dev": max_dev(actual_pts, start, goal),
            "actual_pts": len(actual_pts),
            "plan_points": plan_pts,
            "actual_points": actual_pts,
            "yaw_err_deg": math.degrees(yaw_err),
            "first4_spin_ratio": len(spin_samples) / max(1, len(first_cmds)),
            "first4_move_ratio": len(move_samples) / max(1, len(first_cmds)),
            "mean_abs_wz": sum(abs(wz) for _, _, wz in first_cmds) / max(1, len(first_cmds)),
            "mean_vx": sum(vx for _, vx, _ in first_cmds) / max(1, len(first_cmds)),
            "post_plan_spin_ratio": len([1 for _, vx, wz in post_plan_cmds if abs(vx) < 0.05 and abs(wz) > 0.12]) / max(1, len(post_plan_cmds)),
            "post_plan_move_ratio": len([1 for _, vx, wz in post_plan_cmds if abs(vx) > 0.08]) / max(1, len(post_plan_cmds)),
            "post_plan_mean_abs_wz": sum(abs(wz) for _, _, wz in post_plan_cmds) / max(1, len(post_plan_cmds)),
            "post_plan_mean_vx": sum(vx for _, vx, _ in post_plan_cmds) / max(1, len(post_plan_cmds)),
            "global_straight_cost": cost_summary(nearest_gmap, sample_line(start, goal)),
            "global_plan_cost": cost_summary(nearest_gmap, plan_pts),
            "global_actual_cost": cost_summary(nearest_gmap, actual_pts[:: max(1, len(actual_pts) // 200)]),
            "local_straight_cost": cost_summary(nearest_lmap, sample_line(start, goal)),
            "local_plan_cost": cost_summary(nearest_lmap, plan_pts),
        })
    return rows


def render_html(rows, output):
    if not rows:
        return
    groups = {}
    for row in rows:
        groups.setdefault(row["bag"], []).append(row)

    parts = [
        "<!doctype html><html><head><meta charset='utf-8'><title>Navigation execution analysis</title>",
        "<style>body{font-family:Arial,sans-serif;margin:24px;color:#222}svg{border:1px solid #ccc;background:#fafafa;margin:10px 0 24px}table{border-collapse:collapse;margin:8px 0 18px}td,th{border:1px solid #ddd;padding:4px 8px;font-size:13px}.plan{color:#d62728}.actual{color:#1f77b4}.straight{color:#888}</style>",
        "</head><body><h1>Navigation Execution Analysis</h1>",
        "<p><span class='plan'>red=/plan</span>, <span class='actual'>blue=robot_realpose actual</span>, <span class='straight'>gray=start-goal straight</span>.</p>",
    ]
    for bag, bag_rows in groups.items():
        all_pts = []
        for row in bag_rows:
            all_pts.extend(row["plan_points"])
            all_pts.extend(row["actual_points"])
            all_pts.extend([row["start"], row["goal"]])
        min_x = min(p[0] for p in all_pts) - 1.0
        max_x = max(p[0] for p in all_pts) + 1.0
        min_y = min(p[1] for p in all_pts) - 1.0
        max_y = max(p[1] for p in all_pts) + 1.0
        width, height, margin = 1000, 700, 35
        scale = min(width / max(1e-6, max_x - min_x), height / max(1e-6, max_y - min_y))
        cw = int((max_x - min_x) * scale + 2 * margin)
        ch = int((max_y - min_y) * scale + 2 * margin)

        def sx(x):
            return margin + (x - min_x) * scale

        def sy(y):
            return ch - margin - (y - min_y) * scale

        def poly(points):
            return " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in points)

        parts.append(f"<h2>{bag}</h2>")
        parts.append("<table><tr><th>ids</th><th>plan_dev</th><th>actual_dev</th><th>yaw_err_deg</th><th>first4s_spin</th><th>first4s_move</th></tr>")
        for row in bag_rows:
            parts.append(
                f"<tr><td>{','.join(row['ids'])}</td><td>{row['plan_dev']:.2f}</td><td>{row['actual_dev']:.2f}</td>"
                f"<td>{row['yaw_err_deg']:.1f}</td><td>{row['first4_spin_ratio']:.2f}</td><td>{row['first4_move_ratio']:.2f}</td></tr>"
            )
        parts.append("</table>")
        parts.append(f"<svg width='{cw}' height='{ch}' viewBox='0 0 {cw} {ch}'>")
        for row in bag_rows:
            label = ",".join(row["ids"])
            parts.append(f"<polyline points='{poly([row['start'], row['goal']])}' fill='none' stroke='#888' stroke-width='1' stroke-dasharray='5,5'/>")
            parts.append(f"<polyline points='{poly(row['plan_points'])}' fill='none' stroke='#d62728' stroke-width='2' opacity='0.75'/>")
            parts.append(f"<polyline points='{poly(row['actual_points'])}' fill='none' stroke='#1f77b4' stroke-width='2' opacity='0.85'/>")
            parts.append(f"<circle cx='{sx(row['start'][0]):.1f}' cy='{sy(row['start'][1]):.1f}' r='3' fill='#111'><title>{label} start</title></circle>")
            parts.append(f"<circle cx='{sx(row['goal'][0]):.1f}' cy='{sy(row['goal'][1]):.1f}' r='4' fill='#2ca02c'><title>{label} goal</title></circle>")
            parts.append(f"<text x='{sx(row['goal'][0]) + 5:.1f}' y='{sy(row['goal'][1]) - 5:.1f}' font-size='12'>{label}</text>")
        parts.append("</svg>")
    parts.append("</body></html>")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(parts), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--ids", nargs="*", default=[])
    parser.add_argument("--html", default="")
    args = parser.parse_args()
    all_rows = []
    for bag in args.bags:
        print(f"\n=== {Path(bag).name} ===")
        rows = analyze(Path(bag), set(args.ids))
        all_rows.extend(rows)
        for r in rows:
            print(
                f"ids={','.join(r['ids'])} plan_dt={r['plan_dt']:.2f}s "
                f"plan_len/chord={r['plan_len']:.2f}/{r['plan_chord']:.2f} plan_dev={r['plan_dev']:.2f} "
                f"actual_len={r['actual_len']:.2f} actual_dev={r['actual_dev']:.2f} pts={r['actual_pts']} "
                f"start_yaw_err={r['yaw_err_deg']:.1f}deg first4s_spin={r['first4_spin_ratio']:.2f} "
                f"first4s_move={r['first4_move_ratio']:.2f} mean_vx={r['mean_vx']:.2f} mean|wz|={r['mean_abs_wz']:.2f} "
                f"post_plan_spin={r['post_plan_spin_ratio']:.2f} post_plan_move={r['post_plan_move_ratio']:.2f} "
                f"post_plan_vx={r['post_plan_mean_vx']:.2f} post_plan|wz|={r['post_plan_mean_abs_wz']:.2f}"
            )
            print(f"  global straight={r['global_straight_cost']} plan={r['global_plan_cost']} actual={r['global_actual_cost']}")
            print(f"  local  straight={r['local_straight_cost']} plan={r['local_plan_cost']}")
    if args.html:
        render_html(all_rows, Path(args.html))
        print(f"\nwrote {args.html}")


if __name__ == "__main__":
    main()
