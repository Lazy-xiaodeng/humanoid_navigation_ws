#!/usr/bin/env python3
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_samples(path):
    rows = []
    with Path(path).open(newline="") as f:
        for row in csv.DictReader(f):
            def val(name):
                text = row.get(name)
                return None if text in (None, "", "None") else float(text)
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


def points(rows, prefix):
    return [
        {"x": row[f"{prefix}_x"], "y": row[f"{prefix}_y"]}
        for row in rows
        if row.get(f"{prefix}_x") is not None and row.get(f"{prefix}_y") is not None
    ]


def plot_xy(series, title, output):
    fig, ax = plt.subplots(figsize=(13, 7))
    for label, pts, color, width in series:
        if not pts:
            continue
        xs = [p["x"] for p in pts]
        ys = [p["y"] for p in pts]
        ax.plot(xs, ys, color=color, linewidth=width, label=label)
        ax.scatter([xs[0]], [ys[0]], color=color, s=28, marker="o")
        ax.scatter([xs[-1]], [ys[-1]], color=color, s=42, marker="x")
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
    if len(sys.argv) != 3:
        raise SystemExit("usage: make_integrated_ro_open3d_plots.py RUN_DIR OPEN3D_BASE_DIR")
    run_dir = Path(sys.argv[1])
    open3d_base = Path(sys.argv[2])
    out_dir = run_dir / "plots"
    out_dir.mkdir(exist_ok=True)
    for bag in ("nav_drift_test23", "nav_drift_test24", "nav_drift_test25"):
        ro = read_samples(run_dir / "robosense_integrated" / bag / "samples.csv")
        op = read_samples(open3d_base / "open3d" / bag / "samples.csv")
        plot_xy(
            [
                ("bag /robot_realpose", points(ro, "robot"), "#1f77b4", 1.8),
                ("ro /prior_localization/robosense_odom", points(ro, "prior"), "#d62728", 1.3),
                ("op /prior_localization/odom", points(op, "prior"), "#2ca02c", 1.3),
            ],
            f"{bag}: prior pose comparison",
            out_dir / f"{bag}_prior_pose_ro_vs_op.png",
        )
        plot_xy(
            [
                ("ro bridge map->odom", points(ro, "map_odom"), "#d62728", 1.5),
                ("op bridge map->odom", points(op, "map_odom"), "#2ca02c", 1.5),
            ],
            f"{bag}: bridge map->odom correction comparison",
            out_dir / f"{bag}_map_odom_ro_vs_op.png",
        )
    print(out_dir)


if __name__ == "__main__":
    main()
