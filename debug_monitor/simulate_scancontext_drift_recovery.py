#!/usr/bin/env python3
"""Simulate drift injection and ScanContext recovery retries from offline samples."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
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


def xy_error(row: dict, x: float, y: float) -> float:
    return math.hypot(row["odom_x"] - x, row["odom_y"] - y)


def cluster_global(rows: list[dict], start: int, args: argparse.Namespace) -> dict | None:
    candidates = [
        row
        for row in rows[start : min(len(rows), start + args.global_window)]
        if row["sc_distance"] <= args.global_sc_threshold and row["ambiguity_gate_pass"]
    ]
    best_cluster: list[dict] = []
    for row in candidates:
        cluster = [
            other
            for other in candidates
            if math.hypot(row["sc_x"] - other["sc_x"], row["sc_y"] - other["sc_y"])
            <= args.global_xy_tolerance
        ]
        if len(cluster) > len(best_cluster):
            best_cluster = cluster
    if len(best_cluster) < args.global_required:
        return None
    x = sum(row["sc_x"] for row in best_cluster) / len(best_cluster)
    y = sum(row["sc_y"] for row in best_cluster) / len(best_cluster)
    anchor = rows[start]
    return {
        "mode": "global",
        "x": x,
        "y": y,
        "error": xy_error(anchor, x, y),
        "count": len(best_cluster),
        "nearest_waypoint": best_cluster[0]["sc_nearest_waypoint"],
        "first_sample": best_cluster[0]["sample_index"],
        "last_sample": best_cluster[-1]["sample_index"],
    }


def simulate_one(rows: list[dict], start: int, args: argparse.Namespace) -> dict:
    row = rows[start]
    attempts: list[dict] = []
    cursor = start
    global_mode = False

    for attempt in range(1, args.max_attempts + 1):
        if not global_mode:
            candidate = {
                "mode": "conservative",
                "x": row["sc_x"],
                "y": row["sc_y"],
                "error": row["sc_odom_error"],
                "count": 1,
                "nearest_waypoint": row["sc_nearest_waypoint"],
                "first_sample": row["sample_index"],
                "last_sample": row["sample_index"],
                "published": bool(row["sc_accepted"]),
            }
            if candidate["published"] and candidate["error"] <= args.ndt_accept_error:
                attempts.append({**candidate, "ndt_result": "accepted"})
                return result(row, attempts, True, "conservative accepted by simulated NDT")
            if candidate["published"]:
                attempts.append({**candidate, "ndt_result": "rejected"})
            else:
                attempts.append({**candidate, "ndt_result": "no_initialpose"})
            if attempt >= args.global_after_attempts:
                global_mode = True
            cursor += args.retry_stride
            continue

        if cursor >= len(rows):
            break
        candidate = cluster_global(rows, cursor, args)
        if candidate is None:
            attempts.append(
                {
                    "mode": "global",
                    "error": math.inf,
                    "count": 0,
                    "nearest_waypoint": "",
                    "first_sample": rows[cursor]["sample_index"],
                    "last_sample": rows[cursor]["sample_index"],
                    "published": False,
                    "ndt_result": "no_initialpose",
                }
            )
            cursor += args.retry_stride
            continue
        candidate["published"] = True
        if candidate["error"] <= args.ndt_accept_error:
            attempts.append({**candidate, "ndt_result": "accepted"})
            return result(row, attempts, True, "global accepted by simulated NDT")
        attempts.append({**candidate, "ndt_result": "rejected"})
        cursor += args.retry_stride

    return result(row, attempts, False, "max attempts reached")


def result(row: dict, attempts: list[dict], success: bool, reason: str) -> dict:
    final = attempts[-1] if attempts else {}
    return {
        "start_sample": row["sample_index"],
        "stamp": row["stamp"],
        "odom_x": row["odom_x"],
        "odom_y": row["odom_y"],
        "nearest_waypoint": row["odom_nearest_waypoint"],
        "waypoint_dist": row["odom_nearest_waypoint_dist"],
        "success": success,
        "reason": reason,
        "attempts": len(attempts),
        "final_mode": final.get("mode", ""),
        "final_error": final.get("error", math.inf),
        "final_nearest_waypoint": final.get("nearest_waypoint", ""),
        "attempt_trace": attempts,
    }


def choose_random_points(rows: list[dict], args: argparse.Namespace) -> list[int]:
    valid = [
        idx
        for idx, row in enumerate(rows[: max(0, len(rows) - args.global_window - args.max_attempts)])
        if row["odom_nearest_waypoint_dist"] <= args.max_waypoint_dist
    ]
    rng = random.Random(args.seed)
    if len(valid) <= args.count:
        return valid
    chosen = []
    used_windows: list[tuple[int, int]] = []
    for idx in rng.sample(valid, len(valid)):
        if any(abs(idx - other) < args.min_spacing for other in chosen):
            continue
        chosen.append(idx)
        if len(chosen) >= args.count:
            break
    return sorted(chosen)


def write_outputs(output_dir: Path, source: Path, results: list[dict], args: argparse.Namespace) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "source": str(source),
        "count": len(results),
        "success": sum(r["success"] for r in results),
        "success_rate": sum(r["success"] for r in results) / len(results) if results else 0.0,
        "seed": args.seed,
        "ndt_accept_error": args.ndt_accept_error,
        "max_attempts": args.max_attempts,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    with (output_dir / "drift_trials.csv").open("w", newline="", encoding="utf-8") as f:
        fields = [
            "start_sample", "stamp", "nearest_waypoint", "waypoint_dist", "success",
            "attempts", "final_mode", "final_error", "final_nearest_waypoint", "reason",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in results:
            writer.writerow({k: r[k] for k in fields})
    with (output_dir / "drift_trials_detail.json").open("w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    md = [
        "# Simulated Drift Recovery Trials",
        "",
        f"- Source samples: `{source}`",
        f"- Random seed: `{args.seed}`",
        f"- Trials: {summary['count']}",
        f"- Success: {summary['success']}/{summary['count']} ({summary['success_rate'] * 100:.1f}%)",
        f"- Simulated NDT accepts initialpose if XY error <= {args.ndt_accept_error:.2f} m",
        "",
        "This simulates the current feedback loop from offline ScanContext samples. It does not execute live NDT; use it as a repeatable bag-level risk test.",
        "",
        "| sample | nearest wp | wp dist | result | attempts | final mode | final error | final wp | reason |",
        "|---:|---|---:|---|---:|---|---:|---|---|",
    ]
    for r in results:
        error = r["final_error"]
        error_text = "inf" if not math.isfinite(error) else f"{error:.3f}"
        md.append(
            f"| {r['start_sample']} | {r['nearest_waypoint']} | {r['waypoint_dist']:.3f} | "
            f"{'success' if r['success'] else 'fail'} | {r['attempts']} | {r['final_mode']} | "
            f"{error_text} | {r['final_nearest_waypoint']} | {r['reason']} |"
        )
    (output_dir / "report.md").write_text("\n".join(md) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--seed", type=int, default=20260525)
    parser.add_argument("--count", type=int, default=12)
    parser.add_argument("--min-spacing", type=int, default=60)
    parser.add_argument("--max-waypoint-dist", type=float, default=1.5)
    parser.add_argument("--global-after-attempts", type=int, default=3)
    parser.add_argument("--max-attempts", type=int, default=5)
    parser.add_argument("--retry-stride", type=int, default=6)
    parser.add_argument("--ndt-accept-error", type=float, default=1.0)
    parser.add_argument("--global-window", type=int, default=6)
    parser.add_argument("--global-required", type=int, default=3)
    parser.add_argument("--global-xy-tolerance", type=float, default=0.8)
    parser.add_argument("--global-sc-threshold", type=float, default=0.25)
    args = parser.parse_args()

    source = Path(args.samples)
    rows = load_rows(source)
    starts = choose_random_points(rows, args)
    results = [simulate_one(rows, start, args) for start in starts]
    write_outputs(Path(args.output_dir), source, results, args)
    print(json.dumps({
        "trials": len(results),
        "success": sum(r["success"] for r in results),
        "success_rate": sum(r["success"] for r in results) / len(results) if results else 0.0,
        "output_dir": args.output_dir,
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
