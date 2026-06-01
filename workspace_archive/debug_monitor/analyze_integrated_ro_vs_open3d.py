#!/usr/bin/env python3
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def read_samples(path):
    rows = []
    if not path.exists():
        return rows
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            parsed = {}
            for key, value in row.items():
                if value in ("", "None", None):
                    parsed[key] = None
                elif key == "bridge_status":
                    parsed[key] = value
                else:
                    parsed[key] = float(value)
            rows.append(parsed)
    rows.sort(key=lambda r: r.get("sim_time") or -1.0)
    return rows


def path_stats(rows, prefix):
    points = []
    for row in rows:
        x = row.get(f"{prefix}_x")
        y = row.get(f"{prefix}_y")
        yaw = row.get(f"{prefix}_yaw")
        t = row.get("sim_time")
        if None not in (x, y, yaw, t):
            points.append({"t": t, "x": x, "y": y, "yaw": yaw, "status": row.get("bridge_status") or ""})
    if not points:
        return {
            "count": 0,
            "duration": 0.0,
            "path_length": 0.0,
            "max_step_xy": 0.0,
            "max_step_yaw_deg": 0.0,
            "max_step_time": None,
            "max_step_status": "",
            "max_yaw_time": None,
            "max_yaw_status": "",
        }
    length = 0.0
    max_step = 0.0
    max_step_time = None
    max_step_status = ""
    max_yaw = 0.0
    max_yaw_time = None
    max_yaw_status = ""
    prev = None
    for point in points:
        if prev is not None:
            dxy = math.hypot(point["x"] - prev["x"], point["y"] - prev["y"])
            dyaw = abs(wrap(point["yaw"] - prev["yaw"]))
            length += dxy
            if dxy > max_step:
                max_step = dxy
                max_step_time = point["t"]
                max_step_status = point["status"]
            if dyaw > max_yaw:
                max_yaw = dyaw
                max_yaw_time = point["t"]
                max_yaw_status = point["status"]
        prev = point
    return {
        "count": len(points),
        "duration": points[-1]["t"] - points[0]["t"],
        "path_length": length,
        "max_step_xy": max_step,
        "max_step_yaw_deg": math.degrees(max_yaw),
        "max_step_time": max_step_time,
        "max_step_status": max_step_status,
        "max_yaw_time": max_yaw_time,
        "max_yaw_status": max_yaw_status,
    }


def summarize(rows):
    status_counter = Counter()
    for row in rows:
        status = row.get("bridge_status") or ""
        key = status.split()[0] if status else ""
        if key:
            status_counter[key] += 1
    return {
        "sample_count": len(rows),
        "status_counter": dict(status_counter),
        "prior_path": path_stats(rows, "prior"),
        "map_odom_path": path_stats(rows, "map_odom"),
    }


def fmt(value, digits=3):
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def main():
    if len(sys.argv) < 4:
        raise SystemExit(
            "usage: analyze_integrated_ro_vs_open3d.py RUN_DIR OPEN3D_BASE_DIR BAG_NAME [BAG_NAME...]"
        )
    run_dir = Path(sys.argv[1])
    open3d_base = Path(sys.argv[2])
    bag_names = sys.argv[3:]
    summary = {}
    lines = [
        "# Integrated RoboSense(ro) vs Open3D(op) Bag Validation",
        "",
        f"RoboSense run dir: `{run_dir}`",
        f"Open3D baseline dir: `{open3d_base}`",
        "",
        "| bag | method | samples | map->odom max xy(m) | map->odom max yaw(deg) | prior max xy(m) | prior max yaw(deg) | status |",
        "|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    for bag in bag_names:
        ro_rows = read_samples(run_dir / "robosense_integrated" / bag / "samples.csv")
        op_rows = read_samples(open3d_base / "open3d" / bag / "samples.csv")
        ro = summarize(ro_rows)
        op = summarize(op_rows)
        summary[bag] = {"robosense_integrated": ro, "open3d": op}
        for method, data in (("ro integrated", ro), ("op baseline", op)):
            mp = data["map_odom_path"]
            pp = data["prior_path"]
            lines.append(
                f"| {bag} | {method} | {data['sample_count']} | "
                f"{fmt(mp['max_step_xy'], 4)} | {fmt(mp['max_step_yaw_deg'], 2)} | "
                f"{fmt(pp['max_step_xy'], 4)} | {fmt(pp['max_step_yaw_deg'], 2)} | "
                f"{json.dumps(data['status_counter'], ensure_ascii=False)} |"
            )
    lines.extend([
        "",
        "## Max Jump Detail",
        "",
        "| bag | method | map->odom max xy time | map->odom xy status | map->odom max yaw time | map->odom yaw status |",
        "|---|---:|---:|---|---:|---|",
    ])
    for bag, methods in summary.items():
        for method, data in methods.items():
            mp = data["map_odom_path"]
            lines.append(
                f"| {bag} | {method} | {fmt(mp['max_step_time'])} | "
                f"{mp['max_step_status']} | {fmt(mp['max_yaw_time'])} | {mp['max_yaw_status']} |"
            )
    lines.extend([
        "",
        "## Notes",
        "",
        "- ro integrated uses `robosense_lidar_localization` + `prior_map_odom_bridge` with `jump_protection_mode=monitor`.",
        "- SpinToPose guard is enabled and listens to `/navigation/status` from the bag.",
        "- Because monitor mode does not block jumps, `WOULD_*` statuses show what protect mode would have held/pended.",
        "",
    ])
    (run_dir / "integrated_ro_vs_open3d_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False)
    )
    report = run_dir / "integrated_ro_vs_open3d_report.md"
    report.write_text("\n".join(lines))
    print(report)


if __name__ == "__main__":
    main()
