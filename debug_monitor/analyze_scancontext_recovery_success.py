#!/usr/bin/env python3
"""Estimate conservative/global ScanContext recovery success from offline samples."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def as_bool(value: str) -> bool:
    return str(value).lower() == "true"


def load_rows(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            item = dict(row)
            for key in (
                "sample_index",
                "stamp",
                "odom_x",
                "odom_y",
                "odom_route_dist",
                "odom_nearest_waypoint_dist",
                "sc_x",
                "sc_y",
                "sc_route_dist",
                "sc_nearest_waypoint_dist",
                "sc_odom_error",
                "sc_distance",
                "odom_gate_error",
            ):
                item[key] = int(item[key]) if key == "sample_index" else float(item[key])
            for key in ("odom_gate_pass", "ambiguity_gate_pass", "sc_accepted", "rescue_possible"):
                item[key] = as_bool(item[key])
            rows.append(item)
    return rows


def dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def conservative_ok(row: dict, rescue_error: float) -> bool:
    return row["sc_accepted"] and row["sc_odom_error"] <= rescue_error


def global_cluster(
    rows: list[dict],
    start: int,
    *,
    window: int,
    required: int,
    xy_tolerance: float,
    sc_threshold: float,
) -> dict | None:
    candidates = [
        row
        for row in rows[start : min(len(rows), start + window)]
        if row["sc_distance"] <= sc_threshold and row["ambiguity_gate_pass"]
    ]
    best: list[dict] = []
    for row in candidates:
        center = (row["sc_x"], row["sc_y"])
        cluster = [
            other
            for other in candidates
            if dist(center, (other["sc_x"], other["sc_y"])) <= xy_tolerance
        ]
        if len(cluster) > len(best):
            best = cluster
    if len(best) < required:
        return None
    x = sum(row["sc_x"] for row in best) / len(best)
    y = sum(row["sc_y"] for row in best) / len(best)
    anchor = rows[start]
    error = dist((x, y), (anchor["odom_x"], anchor["odom_y"]))
    return {
        "count": len(best),
        "x": x,
        "y": y,
        "error": error,
        "nearest_waypoint": best[0]["sc_nearest_waypoint"],
        "first_sample": best[0]["sample_index"],
        "last_sample": best[-1]["sample_index"],
    }


def summarize(rows: list[dict], args: argparse.Namespace) -> tuple[dict, list[dict], list[dict]]:
    starts: list[dict] = []
    for idx, row in enumerate(rows):
        conservative = conservative_ok(row, args.rescue_error)
        cluster = None if conservative else global_cluster(
            rows,
            idx,
            window=args.global_window,
            required=args.global_required,
            xy_tolerance=args.global_xy_tolerance,
            sc_threshold=args.global_sc_threshold,
        )
        global_correct = bool(cluster and cluster["error"] <= args.rescue_error)
        global_wrong = bool(cluster and cluster["error"] > args.rescue_error)
        starts.append(
            {
                **row,
                "conservative_success": conservative,
                "global_published": cluster is not None,
                "global_success": global_correct,
                "global_wrong": global_wrong,
                "global_error": cluster["error"] if cluster else math.inf,
                "global_count": cluster["count"] if cluster else 0,
                "global_nearest_waypoint": cluster["nearest_waypoint"] if cluster else "",
            }
        )

    total = len(starts)
    conservative_count = sum(row["conservative_success"] for row in starts)
    global_success_count = sum((not row["conservative_success"]) and row["global_success"] for row in starts)
    global_wrong_count = sum((not row["conservative_success"]) and row["global_wrong"] for row in starts)
    total_success = conservative_count + global_success_count
    no_publish = total - total_success - global_wrong_count
    summary = {
        "samples": total,
        "conservative_success": conservative_count,
        "conservative_success_rate": conservative_count / total if total else 0.0,
        "global_additional_success": global_success_count,
        "global_additional_success_rate": global_success_count / total if total else 0.0,
        "total_success": total_success,
        "total_success_rate": total_success / total if total else 0.0,
        "global_wrong_publish": global_wrong_count,
        "global_wrong_publish_rate": global_wrong_count / total if total else 0.0,
        "no_recovery_publish": no_publish,
        "no_recovery_publish_rate": no_publish / total if total else 0.0,
    }

    waypoint_rows = []
    names = sorted(
        {row["odom_nearest_waypoint"] for row in starts},
        key=lambda name: int(name.replace("点位", "")) if name.startswith("点位") else 9999,
    )
    for name in names:
        near = [
            row for row in starts
            if row["odom_nearest_waypoint"] == name and row["odom_nearest_waypoint_dist"] <= args.waypoint_radius
        ]
        if not near:
            continue
        conservative_near = sum(row["conservative_success"] for row in near)
        global_near = sum((not row["conservative_success"]) and row["global_success"] for row in near)
        wrong_near = sum((not row["conservative_success"]) and row["global_wrong"] for row in near)
        waypoint_rows.append(
            {
                "waypoint": name,
                "samples": len(near),
                "conservative_success": conservative_near,
                "global_additional_success": global_near,
                "total_success": conservative_near + global_near,
                "global_wrong_publish": wrong_near,
                "no_publish": len(near) - conservative_near - global_near - wrong_near,
            }
        )
    return summary, starts, waypoint_rows


def write_outputs(output_dir: Path, source: Path, summary: dict, rows: list[dict], waypoint_rows: list[dict]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    with (output_dir / "recovery_samples.csv").open("w", newline="", encoding="utf-8") as f:
        fields = [
            "sample_index", "stamp", "odom_nearest_waypoint", "odom_nearest_waypoint_dist",
            "sc_nearest_waypoint", "sc_odom_error", "sc_distance", "conservative_success",
            "global_published", "global_success", "global_wrong", "global_error",
            "global_count", "global_nearest_waypoint",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row[key] for key in fields})
    with (output_dir / "waypoint_recovery.csv").open("w", newline="", encoding="utf-8") as f:
        fields = [
            "waypoint", "samples", "conservative_success", "global_additional_success",
            "total_success", "global_wrong_publish", "no_publish",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(waypoint_rows)

    md = [
        "# ScanContext Recovery Success Estimate",
        "",
        f"- Source samples: `{source}`",
        f"- Samples: {summary['samples']}",
        f"- Conservative success: {summary['conservative_success']}/{summary['samples']} ({summary['conservative_success_rate'] * 100:.1f}%)",
        f"- Global additional success: {summary['global_additional_success']}/{summary['samples']} ({summary['global_additional_success_rate'] * 100:.1f}%)",
        f"- Total estimated success: {summary['total_success']}/{summary['samples']} ({summary['total_success_rate'] * 100:.1f}%)",
        f"- Estimated wrong global publish: {summary['global_wrong_publish']}/{summary['samples']} ({summary['global_wrong_publish_rate'] * 100:.1f}%)",
        f"- No recovery publish: {summary['no_recovery_publish']}/{summary['samples']} ({summary['no_recovery_publish_rate'] * 100:.1f}%)",
        "",
        "This estimates the new recovery state machine from offline SC samples. It does not include real GICP fitness, so wrong-publish risk is an upper-bound signal rather than a final safety proof.",
        "",
        "## Waypoints",
        "",
        "| waypoint | samples | conservative | global add | total | wrong global | no publish |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in waypoint_rows:
        md.append(
            f"| {row['waypoint']} | {row['samples']} | {row['conservative_success']} | "
            f"{row['global_additional_success']} | {row['total_success']} | "
            f"{row['global_wrong_publish']} | {row['no_publish']} |"
        )
    (output_dir / "report.md").write_text("\n".join(md) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--rescue-error", type=float, default=1.0)
    parser.add_argument("--global-window", type=int, default=6)
    parser.add_argument("--global-required", type=int, default=3)
    parser.add_argument("--global-xy-tolerance", type=float, default=0.8)
    parser.add_argument("--global-sc-threshold", type=float, default=0.25)
    parser.add_argument("--waypoint-radius", type=float, default=1.5)
    args = parser.parse_args()

    source = Path(args.samples)
    rows = load_rows(source)
    summary, starts, waypoint_rows = summarize(rows, args)
    write_outputs(Path(args.output_dir), source, summary, starts, waypoint_rows)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
