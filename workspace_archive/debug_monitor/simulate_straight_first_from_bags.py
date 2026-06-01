#!/usr/bin/env python3
import argparse
import heapq
import html
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {"/app/navigation_command", "/global_costmap/costmap", "/plan"}


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_len(points):
    return sum(dist(a, b) for a, b in zip(points, points[1:])) if len(points) > 1 else 0.0


def max_deviation(points):
    if len(points) < 2:
        return 0.0
    start, end = points[0], points[-1]
    chord = dist(start, end)
    if chord < 1e-9:
        return 0.0
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    return max(abs(dy * x - dx * y + ex * sy - ey * sx) / chord for x, y in points)


class Grid:
    def __init__(self, msg):
        self.width = msg.info.width
        self.height = msg.info.height
        self.res = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.data = list(msg.data)

    def world_to_map(self, p):
        return int((p[0] - self.ox) / self.res), int((p[1] - self.oy) / self.res)

    def map_to_world(self, c):
        return self.ox + (c[0] + 0.5) * self.res, self.oy + (c[1] + 0.5) * self.res

    def in_bounds(self, c):
        return 0 <= c[0] < self.width and 0 <= c[1] < self.height

    def cost(self, c):
        return self.data[c[1] * self.width + c[0]]

    def safe(self, c, max_cost=20, allow_unknown=False):
        if not self.in_bounds(c):
            return False
        v = self.cost(c)
        if v < 0:
            return allow_unknown
        return v <= max_cost

    def theta_safe(self, c, allow_unknown=False):
        if not self.in_bounds(c):
            return False
        v = self.cost(c)
        if v < 0:
            return allow_unknown
        return v <= 252


def bresenham(a, b):
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


def dense_straight(start, goal, step):
    d = dist(start, goal)
    steps = max(1, int(math.ceil(d / step)))
    return [
        (start[0] + (goal[0] - start[0]) * i / steps, start[1] + (goal[1] - start[1]) * i / steps)
        for i in range(steps + 1)
    ]


def straight_is_safe(grid, start, goal, max_cost=20, allow_unknown=False):
    for c in bresenham(grid.world_to_map(start), grid.world_to_map(goal)):
        if not grid.safe(c, max_cost=max_cost, allow_unknown=allow_unknown):
            return False
    return True


def los_cost(grid, a, b, w_traversal, allow_unknown):
    total = 0.0
    for c in bresenham(a, b):
        if not grid.theta_safe(c, allow_unknown):
            return None
        raw = grid.cost(c)
        if raw < 0:
            raw = 100
        scaled = 26.0 + 0.9 * raw
        total += w_traversal * scaled * scaled / (252.0 * 252.0)
    return total


def theta_star(grid, start_w, goal_w, w_euc=1.0, w_traversal=2.0, allow_unknown=False):
    start = grid.world_to_map(start_w)
    goal = grid.world_to_map(goal_w)
    if not grid.theta_safe(start, allow_unknown) or not grid.theta_safe(goal, allow_unknown):
        return []

    moves = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, -1), (-1, 1), (1, 1), (-1, -1)]
    g = {start: 0.0}
    parent = {start: start}
    closed = set()
    open_heap = [(dist(start, goal), 0, start)]
    seq = 1

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            break
        closed.add(current)

        for dx, dy in moves:
            nb = (current[0] + dx, current[1] + dy)
            if not grid.theta_safe(nb, allow_unknown) or nb in closed:
                continue

            p = parent[current]
            sl_cost = los_cost(grid, p, nb, w_traversal, allow_unknown)
            if sl_cost is not None:
                cand_parent = p
                cand_g = g[p] + w_euc * dist(p, nb) + sl_cost
            else:
                step_cost = los_cost(grid, current, nb, w_traversal, allow_unknown)
                if step_cost is None:
                    continue
                cand_parent = current
                cand_g = g[current] + w_euc * dist(current, nb) + step_cost

            if cand_g < g.get(nb, float("inf")):
                g[nb] = cand_g
                parent[nb] = cand_parent
                heapq.heappush(open_heap, (cand_g + dist(nb, goal), seq, nb))
                seq += 1

    if goal not in parent:
        return []

    cells = []
    c = goal
    while True:
        cells.append(c)
        if c == start:
            break
        c = parent[c]
    cells.reverse()

    points = []
    for a, b in zip(cells, cells[1:]):
        aw = grid.map_to_world(a)
        bw = grid.map_to_world(b)
        steps = max(1, int(dist(aw, bw) / grid.res))
        for i in range(steps):
            t = i / steps
            points.append((aw[0] + (bw[0] - aw[0]) * t, aw[1] + (bw[1] - aw[1]) * t))
    points.append(grid.map_to_world(cells[-1]))
    if points:
        points[0] = start_w
        points[-1] = goal_w
    return points


def straight_first(grid, start, goal, max_cost=20, allow_unknown=False):
    if straight_is_safe(grid, start, goal, max_cost=max_cost, allow_unknown=allow_unknown):
        return dense_straight(start, goal, max(grid.res, 0.05)), "straight"
    return theta_star(grid, start, goal, allow_unknown=allow_unknown), "theta_fallback"


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
    pending = []
    last_grid = None
    rows = []

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == "/global_costmap/costmap":
            last_grid = Grid(msg)
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
            pending.append((stamp_ns, ids))
        elif topic == "/plan":
            if not pending:
                continue
            for item in list(pending):
                cmd_stamp, ids = item
                dt = (stamp_ns - cmd_stamp) / 1e9
                if dt < 0:
                    continue
                if dt > 15:
                    pending.remove(item)
                    continue
                original = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
                if len(original) < 2 or last_grid is None:
                    pending.remove(item)
                    continue
                simulated, mode = straight_first(last_grid, original[0], original[-1], max_cost, allow_unknown)
                rows.append({
                    "bag": Path(bag_dir).name,
                    "ids": ids,
                    "dt": dt,
                    "original": original,
                    "simulated": simulated,
                    "mode": mode,
                    "start": original[0],
                    "goal": original[-1],
                })
                pending.remove(item)
                break
    return rows


def polyline(points, sx, sy):
    return " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in points)


def render_html(rows, output):
    grouped = {}
    for r in rows:
        grouped.setdefault(r["bag"], []).append(r)

    parts = [
        "<!doctype html><html><head><meta charset='utf-8'><title>StraightFirst bag simulation</title>",
        "<style>body{font-family:Arial,sans-serif;margin:24px;color:#222}svg{border:1px solid #ccc;background:#fafafa;margin:12px 0 28px}table{border-collapse:collapse;margin:10px 0 28px}td,th{border:1px solid #ddd;padding:4px 8px;font-size:13px}.orig{color:#d62728}.new{color:#2ca02c}.straight{color:#777}</style>",
        "</head><body><h1>StraightFirst bag simulation</h1>",
        "<p><span class='orig'>red=bag /plan</span>, <span class='new'>green=StraightFirst simulated plan</span>, <span class='straight'>gray=start-goal chord</span>.</p>",
    ]

    for bag, bag_rows in grouped.items():
        all_pts = []
        for r in bag_rows:
            all_pts.extend(r["original"])
            all_pts.extend(r["simulated"])
            all_pts.extend([r["start"], r["goal"]])
        min_x = min(p[0] for p in all_pts) - 1.0
        max_x = max(p[0] for p in all_pts) + 1.0
        min_y = min(p[1] for p in all_pts) - 1.0
        max_y = max(p[1] for p in all_pts) + 1.0
        width, height = 1200, 820
        scale = min(width / max(1e-6, max_x - min_x), height / max(1e-6, max_y - min_y))
        margin = 35
        canvas_w = int((max_x - min_x) * scale + 2 * margin)
        canvas_h = int((max_y - min_y) * scale + 2 * margin)

        def sx(x):
            return margin + (x - min_x) * scale

        def sy(y):
            return canvas_h - margin - (y - min_y) * scale

        parts.append(f"<h2>{html.escape(bag)}</h2>")
        parts.append("<table><tr><th>ids</th><th>mode</th><th>bag_len</th><th>bag_dev</th><th>new_len</th><th>new_dev</th><th>dev_reduce</th></tr>")
        for r in bag_rows:
            olen = path_len(r["original"])
            odev = max_deviation(r["original"])
            nlen = path_len(r["simulated"])
            ndev = max_deviation(r["simulated"])
            parts.append(
                f"<tr><td>{html.escape(','.join(r['ids']))}</td><td>{html.escape(r['mode'])}</td>"
                f"<td>{olen:.2f}</td><td>{odev:.2f}</td><td>{nlen:.2f}</td><td>{ndev:.2f}</td><td>{odev - ndev:.2f}</td></tr>"
            )
        parts.append("</table>")
        parts.append(f"<svg width='{canvas_w}' height='{canvas_h}' viewBox='0 0 {canvas_w} {canvas_h}'>")
        for r in bag_rows:
            label = ",".join(r["ids"])
            parts.append(f"<polyline points='{polyline([r['start'], r['goal']], sx, sy)}' fill='none' stroke='#777' stroke-width='1' stroke-dasharray='5,5'/>")
            parts.append(f"<polyline points='{polyline(r['original'], sx, sy)}' fill='none' stroke='#d62728' stroke-width='2' opacity='0.65'/>")
            if r["simulated"]:
                parts.append(f"<polyline points='{polyline(r['simulated'], sx, sy)}' fill='none' stroke='#2ca02c' stroke-width='2.5' opacity='0.85'/>")
            parts.append(f"<circle cx='{sx(r['start'][0]):.1f}' cy='{sy(r['start'][1]):.1f}' r='3' fill='#222'><title>{html.escape(label)} start</title></circle>")
            parts.append(f"<circle cx='{sx(r['goal'][0]):.1f}' cy='{sy(r['goal'][1]):.1f}' r='4' fill='#2ca02c'><title>{html.escape(label)} goal</title></circle>")
            parts.append(f"<text x='{sx(r['goal'][0]) + 5:.1f}' y='{sy(r['goal'][1]) - 5:.1f}' font-size='11'>{html.escape(label)}</text>")
        parts.append("</svg>")

    parts.append("</body></html>")
    output.write_text("\n".join(parts), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+")
    parser.add_argument("--ids", nargs="*", default=[])
    parser.add_argument("--max-cost", type=int, default=20)
    parser.add_argument("--allow-unknown", action="store_true")
    parser.add_argument("--output", default="workspace_archive/debug_monitor/straight_first_bag_simulation.html")
    args = parser.parse_args()

    rows = []
    for bag in args.bags:
        rows.extend(analyze_bag(Path(bag), set(args.ids), args.max_cost, args.allow_unknown))

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    render_html(rows, output)

    print(f"wrote {output}")
    print("bag,ids,mode,bag_len,bag_dev,new_len,new_dev")
    for r in rows:
        print(
            f"{r['bag']},{'+'.join(r['ids'])},{r['mode']},"
            f"{path_len(r['original']):.3f},{max_deviation(r['original']):.3f},"
            f"{path_len(r['simulated']):.3f},{max_deviation(r['simulated']):.3f}"
        )


if __name__ == "__main__":
    main()
