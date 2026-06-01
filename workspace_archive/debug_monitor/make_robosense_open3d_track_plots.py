#!/usr/bin/env python3
import bisect
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def yaw_from_q(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def read_pose_csv(path):
    poses = {}
    with Path(path).open(newline="") as f:
        for row in csv.DictReader(f):
            qx = float(row["qx"])
            qy = float(row["qy"])
            qz = float(row["qz"])
            qw = float(row["qw"])
            poses.setdefault(row["source"], []).append({
                "t": float(row["stamp"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "yaw": yaw_from_q(qx, qy, qz, qw),
                "status": row.get("status", ""),
            })
    return poses


def read_open3d_samples(path):
    rows = []
    with Path(path).open(newline="") as f:
        for row in csv.DictReader(f):
            def val(name):
                text = row.get(name)
                return None if text in (None, "") else float(text)
            rows.append({
                "robot_x": val("robot_x"),
                "robot_y": val("robot_y"),
                "prior_x": val("prior_x"),
                "prior_y": val("prior_y"),
                "map_odom_x": val("map_odom_x"),
                "map_odom_y": val("map_odom_y"),
                "status": row.get("bridge_status", ""),
            })
    return rows


def nearest(points, t):
    times = [p["t"] for p in points]
    idx = bisect.bisect_left(times, t)
    candidates = []
    if idx < len(points):
        candidates.append(points[idx])
    if idx > 0:
        candidates.append(points[idx - 1])
    return min(candidates, key=lambda p: abs(p["t"] - t)) if candidates else None


def align_odom_to_global(odom, global_pose):
    if not odom or not global_pose:
        return []
    ref_g = global_pose[0]
    ref_o = nearest(odom, ref_g["t"])
    if ref_o is None:
        return []
    dyaw = ref_g["yaw"] - ref_o["yaw"]
    c = math.cos(dyaw)
    s = math.sin(dyaw)
    tx = ref_g["x"] - (c * ref_o["x"] - s * ref_o["y"])
    ty = ref_g["y"] - (s * ref_o["x"] + c * ref_o["y"])
    out = []
    for p in odom:
        out.append({
            "x": c * p["x"] - s * p["y"] + tx,
            "y": s * p["x"] + c * p["y"] + ty,
        })
    return out


def plot_xy(series, title, output):
    fig, ax = plt.subplots(figsize=(13, 7))
    for label, points, color, width in series:
        xs = [p["x"] for p in points if p.get("x") is not None and p.get("y") is not None]
        ys = [p["y"] for p in points if p.get("x") is not None and p.get("y") is not None]
        if not xs:
            continue
        ax.plot(xs, ys, color=color, linewidth=width, label=label)
        ax.scatter([xs[0]], [ys[0]], color=color, s=30, marker="o")
        ax.scatter([xs[-1]], [ys[-1]], color=color, s=45, marker="x")
    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: make_robosense_open3d_track_plots.py RUN_DIR")
    run_dir = Path(sys.argv[1])
    out_dir = run_dir / "plots"
    out_dir.mkdir(exist_ok=True)

    durations = {
        "nav_drift_test23": "1406",
        "nav_drift_test24": "1258",
        "nav_drift_test25": "1510",
    }
    for bag, dur in durations.items():
        poses = read_pose_csv(run_dir / "robosense" / bag / f"poses_{dur}s.csv")
        lidar = poses.get("lidar_localization", [])
        odom_aligned = align_odom_to_global(poses.get("fastlio_base_odom", []), lidar)
        plot_xy(
            [
                ("Fast-LIO /odom initial aligned", odom_aligned, "#1f77b4", 1.6),
                ("RoboSense /lidar_pose_xyz global pose", lidar, "#d62728", 1.6),
            ],
            f"{bag}: RoboSense global pose vs odom",
            out_dir / f"{bag}_robosense_odom_vs_global.png",
        )

        samples = read_open3d_samples(run_dir / "open3d" / bag / "samples.csv")
        robot = [{"x": r["robot_x"], "y": r["robot_y"]} for r in samples if r["robot_x"] is not None]
        prior = [{"x": r["prior_x"], "y": r["prior_y"]} for r in samples if r["prior_x"] is not None]
        map_odom = [{"x": r["map_odom_x"], "y": r["map_odom_y"]} for r in samples if r["map_odom_x"] is not None]
        plot_xy(
            [
                ("bag /robot_realpose", robot, "#1f77b4", 1.8),
                ("current Open3D /prior_localization/odom", prior, "#d62728", 1.4),
            ],
            f"{bag}: Open3D prior pose vs bag robot_realpose",
            out_dir / f"{bag}_open3d_prior_vs_robot_realpose.png",
        )
        plot_xy(
            [("current Open3D bridge map->odom correction", map_odom, "#9467bd", 1.8)],
            f"{bag}: Open3D map->odom correction trajectory",
            out_dir / f"{bag}_open3d_map_odom_correction.png",
        )
    print(out_dir)


if __name__ == "__main__":
    main()
