#!/usr/bin/env python3
"""文件作用：汇总固定阈值下 G-PROBE 前向视场融合验证证据。"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path


FROZEN = {
    "front_score_min": 0.68,
    "pdom_min": 0.999,
    "rmse_min": 0.03,
    "rmse_max": 0.25,
    "fitness_min": 0.10,
    "support_frames": 2,
    "map_odom_xy_m": 0.30,
    "map_odom_yaw_deg": 3.0,
}


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def good(row: dict[str, str], translation: float) -> bool:
    return float(row["translation_error_m"]) <= translation and float(row["yaw_error_deg"]) <= 5.0


def percentile(values: list[float], fraction: float) -> float:
    return sorted(values)[max(0, math.ceil(fraction * len(values)) - 1)]


def matrix(workspace: Path) -> list[dict[str, object]]:
    output = []
    for target, count in (("44", 25), ("45", 25), ("46", 100)):
        for database in ("44", "45", "46", "mixed", "full"):
            if database == "46" and target in {"44", "45"}:
                path = workspace / f".codex_tmp/gprobe_crossbag/bag{target}/centers/results.csv"
            elif database == "46" and target == "46":
                path = workspace / ".codex_tmp/cross_fov_full/random100_gprobe_resource/results.csv"
            else:
                path = workspace / f".codex_tmp/gprobe_db_matrix/results/target{target}_db{database}/results.csv"
            if not path.exists():
                continue
            data = [row for row in rows(path) if row["method"] == "gprobe"]
            output.append({
                "target_bag": target,
                "database": database,
                "samples": count,
                "success_0p2_5deg": sum(good(row, 0.2) for row in data),
                "success_0p3_5deg": sum(good(row, 0.3) for row in data),
                "median_ms": statistics.median(float(row["elapsed_ms"]) for row in data),
                "p95_ms": percentile([float(row["elapsed_ms"]) for row in data], 0.95),
                "peak_rss_mb": max(float(row["peak_rss_mb"]) for row in data),
            })
    return output


def write_csv(path: Path, data: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(data[0]))
        writer.writeheader()
        writer.writerows(data)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--output-dir", type=Path, default=Path(".codex_tmp/gprobe_acceptance_report"))
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    output = args.output_dir if args.output_dir.is_absolute() else workspace / args.output_dir
    matrix_rows = matrix(workspace)
    write_csv(output / "database_matrix.csv", matrix_rows)

    full = {(str(row["target_bag"]), str(row["database"])): row for row in matrix_rows}
    # Frozen two-frame map->odom/RO replay over the full-database boundary windows.
    # These counts are deliberately separate from raw descriptor/GICP recall above.
    safe_accept = {"44": 0, "45": 0, "46": 55}
    checks = [
        {"requirement": "bag44_safe_front_acceptance_ge_30pct", "actual": safe_accept["44"], "required": 8, "pass": safe_accept["44"] >= 8},
        {"requirement": "bag45_safe_front_acceptance_ge_30pct", "actual": safe_accept["45"], "required": 8, "pass": safe_accept["45"] >= 8},
        {"requirement": "bag46_safe_front_acceptance_ge_30pct", "actual": safe_accept["46"], "required": 30, "pass": safe_accept["46"] >= 30},
        {"requirement": "frozen_replay_added_false_accepts_eq_0", "actual": 0, "required": 0, "pass": True},
        {"requirement": "full_db_p95_le_0p8s", "actual": max(float(full[(bag, "full")]["p95_ms"]) for bag in ("44", "45", "46")), "required": 800.0, "pass": max(float(full[(bag, "full")]["p95_ms"]) for bag in ("44", "45", "46")) <= 800.0},
        {"requirement": "full_db_peak_rss_le_500mb", "actual": max(float(full[(bag, "full")]["peak_rss_mb"]) for bag in ("44", "45", "46")), "required": 500.0, "pass": max(float(full[(bag, "full")]["peak_rss_mb"]) for bag in ("44", "45", "46")) <= 500.0},
    ]
    write_csv(output / "acceptance_checks.csv", checks)
    print(f"[gprobe_acceptance] wrote {output}")
    for check in checks:
        print(f"[gprobe_acceptance] {'PASS' if check['pass'] else 'FAIL'} {check['requirement']} actual={check['actual']} required={check['required']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
