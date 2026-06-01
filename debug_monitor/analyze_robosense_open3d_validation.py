#!/usr/bin/env python3
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path


def yaw_from_q(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def read_robosense_csv(path):
    rows = []
    if not path.exists():
        return rows
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            if row.get("source") != "lidar_localization":
                continue
            rows.append({
                "t": float(row["stamp"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"]),
                "yaw": yaw_from_q(
                    float(row["qx"]), float(row["qy"]),
                    float(row["qz"]), float(row["qw"])
                ),
                "status": row.get("status", ""),
            })
    rows.sort(key=lambda r: r["t"])
    return rows


def read_open3d_samples(path):
    rows = []
    if not path.exists():
        return rows
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            def maybe_float(name):
                value = row.get(name)
                if value in (None, ""):
                    return None
                return float(value)
            rows.append({
                "t": maybe_float("sim_time"),
                "confidence": maybe_float("confidence"),
                "status": row.get("bridge_status", ""),
                "prior_x": maybe_float("prior_x"),
                "prior_y": maybe_float("prior_y"),
                "prior_yaw": maybe_float("prior_yaw"),
                "map_odom_x": maybe_float("map_odom_x"),
                "map_odom_y": maybe_float("map_odom_y"),
                "map_odom_yaw": maybe_float("map_odom_yaw"),
            })
    rows.sort(key=lambda r: r["t"] if r["t"] is not None else -1.0)
    return rows


def path_stats(points, x_key="x", y_key="y", yaw_key="yaw"):
    if not points:
        return {
            "count": 0,
            "duration": 0.0,
            "path_length": 0.0,
            "max_step_xy": 0.0,
            "max_step_yaw_deg": 0.0,
            "start": None,
            "end": None,
        }
    length = 0.0
    max_step = 0.0
    max_yaw = 0.0
    prev = None
    for p in points:
        if prev is not None:
            dx = p[x_key] - prev[x_key]
            dy = p[y_key] - prev[y_key]
            dxy = math.hypot(dx, dy)
            dyaw = abs(wrap(p[yaw_key] - prev[yaw_key]))
            length += dxy
            max_step = max(max_step, dxy)
            max_yaw = max(max_yaw, dyaw)
        prev = p
    return {
        "count": len(points),
        "duration": points[-1]["t"] - points[0]["t"] if points[-1]["t"] and points[0]["t"] else 0.0,
        "path_length": length,
        "max_step_xy": max_step,
        "max_step_yaw_deg": math.degrees(max_yaw),
        "start": [points[0][x_key], points[0][y_key], points[0][yaw_key]],
        "end": [points[-1][x_key], points[-1][y_key], points[-1][yaw_key]],
    }


def open3d_stats(rows):
    prior = [
        {"t": r["t"], "x": r["prior_x"], "y": r["prior_y"], "yaw": r["prior_yaw"]}
        for r in rows
        if r["prior_x"] is not None and r["prior_y"] is not None and r["prior_yaw"] is not None
    ]
    map_odom = [
        {"t": r["t"], "x": r["map_odom_x"], "y": r["map_odom_y"], "yaw": r["map_odom_yaw"]}
        for r in rows
        if r["map_odom_x"] is not None and r["map_odom_y"] is not None and r["map_odom_yaw"] is not None
    ]
    confidences = [r["confidence"] for r in rows if r["confidence"] is not None]
    status_counter = Counter()
    for r in rows:
        status = (r["status"] or "").split()
        if status:
            status_counter[status[0]] += 1
    return {
        "sample_count": len(rows),
        "status_counter": dict(status_counter),
        "confidence_count": len(confidences),
        "confidence_min": min(confidences) if confidences else None,
        "confidence_mean": sum(confidences) / len(confidences) if confidences else None,
        "confidence_max": max(confidences) if confidences else None,
        "prior_path": path_stats(prior),
        "map_odom_path": path_stats(map_odom),
    }


def robosense_stats(rows):
    return {
        "status_counter": dict(Counter(r["status"] for r in rows if r["status"])),
        "pose_path": path_stats(rows),
    }


def fmt(v, digits=3):
    if v is None:
        return "n/a"
    if isinstance(v, float):
        return f"{v:.{digits}f}"
    return str(v)


def find_robosense_csv(directory):
    files = sorted(directory.glob("poses_*s.csv"))
    return files[0] if files else directory / "poses.csv"


def write_report(run_dir, bag_names):
    lines = [
        "# RoboSense vs Open3D Bag Validation Report",
        "",
        f"Run dir: `{run_dir}`",
        "",
        "| bag | method | samples | duration(s) | max step xy(m) | max step yaw(deg) | status/confidence |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    summary = {}
    for bag in bag_names:
        bag_summary = {}
        robo_dir = run_dir / "robosense" / bag
        robo_rows = read_robosense_csv(find_robosense_csv(robo_dir))
        robo = robosense_stats(robo_rows)
        bag_summary["robosense"] = robo
        rp = robo["pose_path"]
        lines.append(
            f"| {bag} | RoboSense | {rp['count']} | {fmt(rp['duration'])} | "
            f"{fmt(rp['max_step_xy'], 4)} | {fmt(rp['max_step_yaw_deg'], 2)} | "
            f"{json.dumps(robo['status_counter'], ensure_ascii=False)} |"
        )

        open_dir = run_dir / "open3d" / bag
        open_rows = read_open3d_samples(open_dir / "samples.csv")
        op = open3d_stats(open_rows)
        bag_summary["open3d"] = op
        mp = op["map_odom_path"]
        conf = (
            f"conf mean={fmt(op['confidence_mean'])}, "
            f"min={fmt(op['confidence_min'])}, max={fmt(op['confidence_max'])}; "
            f"{json.dumps(op['status_counter'], ensure_ascii=False)}"
        )
        lines.append(
            f"| {bag} | Open3D bridge | {op['sample_count']} | {fmt(mp['duration'])} | "
            f"{fmt(mp['max_step_xy'], 4)} | {fmt(mp['max_step_yaw_deg'], 2)} | {conf} |"
        )
        summary[bag] = bag_summary

    lines.extend([
        "",
        "## Notes",
        "",
        "- RoboSense max step is computed from `/lidar_pose_xyz` pose samples.",
        "- Open3D max step is computed from bridge-maintained `map->odom` samples recorded by `prior_map_bag_monitor`.",
        "- This report is a first-pass numerical summary; detailed failure timing still needs log inspection around large steps.",
        "",
    ])
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False))
    (run_dir / "report.md").write_text("\n".join(lines))


def main():
    if len(sys.argv) < 3:
        raise SystemExit("usage: analyze_robosense_open3d_validation.py RUN_DIR BAG_NAME [BAG_NAME...]")
    run_dir = Path(sys.argv[1])
    write_report(run_dir, sys.argv[2:])
    print(run_dir / "report.md")


if __name__ == "__main__":
    main()
