#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_candidate_quality.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 读取 global_relocalization_metrics.csv 和 global_relocalization_candidates.csv。
  3. 统计每帧 top-K 候选的分数并列、空间分散度、最终 refine rank、fitness、误差等信息。
  4. 用于寻找“错误全局重定位是否可通过候选质量指标提前拒绝”的证据。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/analyze_candidate_quality.py \
    .codex_tmp/global_relocalization_nav_drift_extended_v1/global_relocalization_metrics.csv \
    .codex_tmp/global_relocalization_nav_drift_extended_v1/global_relocalization_candidates.csv
"""

from __future__ import annotations

import csv
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path


def to_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def to_int(row: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(row.get(key, "")))
    except ValueError:
        return default


def yaw_diff_deg(a: float, b: float) -> float:
    diff = math.radians(a - b)
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return abs(math.degrees(diff))


def circular_std_deg(values: list[float]) -> float:
    if not values:
        return math.nan
    sin_sum = sum(math.sin(math.radians(v)) for v in values) / len(values)
    cos_sum = sum(math.cos(math.radians(v)) for v in values) / len(values)
    resultant = math.hypot(sin_sum, cos_sum)
    if resultant <= 1e-12:
        return 180.0
    return math.degrees(math.sqrt(max(0.0, -2.0 * math.log(min(1.0, resultant)))))


def median(values: list[float]) -> float:
    finite = [v for v in values if math.isfinite(v)]
    return statistics.median(finite) if finite else math.nan


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def key_for(row: dict[str, str]) -> tuple[str, str, str, str]:
    return (
        row.get("map_path", ""),
        row.get("bag_path", ""),
        row.get("stamp_sec", ""),
        row.get("refine_method", ""),
    )


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print("usage: analyze_candidate_quality.py <metrics.csv> <candidates.csv>", file=sys.stderr)
        return 2

    metrics_path = Path(argv[1])
    candidates_path = Path(argv[2])
    metric_rows = [
        r for r in load_rows(metrics_path)
        if r.get("scenario_name") == "arbitrary_start_no_prior"
    ]
    candidate_groups: dict[tuple[str, str, str, str], list[dict[str, str]]] = defaultdict(list)
    for row in load_rows(candidates_path):
        if row.get("scenario_name") != "arbitrary_start_no_prior":
            continue
        candidate_groups[key_for(row)].append(row)

    frame_features: list[dict[str, float | str | int]] = []
    for metric in metric_rows:
        candidates = sorted(candidate_groups.get(key_for(metric), []), key=lambda r: to_int(r, "rank"))
        if not candidates:
            continue

        scores = [to_int(c, "score") for c in candidates]
        xs = [to_float(c, "candidate_x_m") for c in candidates]
        ys = [to_float(c, "candidate_y_m") for c in candidates]
        yaws = [to_float(c, "candidate_yaw_deg") for c in candidates]
        top_score = scores[0]
        score_at_last = scores[-1]
        top_tie_count = sum(1 for s in scores if s == top_score)
        unique_score_count = len(set(scores))
        xy_spread_m = math.hypot(statistics.pstdev(xs), statistics.pstdev(ys)) if len(xs) > 1 else 0.0
        yaw_spread_deg = circular_std_deg(yaws)

        ref_x = to_float(metric, "reference_x_m")
        ref_y = to_float(metric, "reference_y_m")
        ref_yaw = to_float(metric, "reference_yaw_deg")
        nearest_trans = math.nan
        nearest_yaw = math.nan
        nearest_rank = 0
        if to_int(metric, "has_reference") == 1:
            nearest = None
            for c in candidates:
                dx = to_float(c, "candidate_x_m") - ref_x
                dy = to_float(c, "candidate_y_m") - ref_y
                trans = math.hypot(dx, dy)
                yaw = yaw_diff_deg(to_float(c, "candidate_yaw_deg"), ref_yaw)
                current = (trans, yaw, to_int(c, "rank"))
                if nearest is None or current < nearest:
                    nearest = current
            if nearest is not None:
                nearest_trans, nearest_yaw, nearest_rank = nearest

        frame_features.append(
            {
                "bag": Path(metric.get("bag_path", "")).name,
                "stamp": metric.get("stamp_sec", ""),
                "success": to_int(metric, "success"),
                "trans_error_m": to_float(metric, "translation_error_m"),
                "yaw_error_deg": to_float(metric, "yaw_error_deg"),
                "refined_rank": to_int(metric, "refined_candidate_rank"),
                "fitness": to_float(metric, "refine_fitness_score"),
                "total_ms": to_float(metric, "total_ms"),
                "top_score": top_score,
                "last_score": score_at_last,
                "score_margin": top_score - score_at_last,
                "top_tie_count": top_tie_count,
                "unique_score_count": unique_score_count,
                "xy_spread_m": xy_spread_m,
                "yaw_spread_deg": yaw_spread_deg,
                "nearest_candidate_trans_m": nearest_trans,
                "nearest_candidate_yaw_deg": nearest_yaw,
                "nearest_candidate_rank": nearest_rank,
            }
        )

    numeric_keys = [
        "trans_error_m",
        "yaw_error_deg",
        "refined_rank",
        "fitness",
        "total_ms",
        "score_margin",
        "top_tie_count",
        "unique_score_count",
        "xy_spread_m",
        "yaw_spread_deg",
        "nearest_candidate_trans_m",
        "nearest_candidate_yaw_deg",
        "nearest_candidate_rank",
    ]

    print("group,n," + ",".join(f"median_{key}" for key in numeric_keys))
    for label, success_value in [("success", 1), ("failure", 0)]:
        rows = [r for r in frame_features if r["success"] == success_value]
        values = [label, str(len(rows))]
        for key in numeric_keys:
            values.append(f"{median([float(r[key]) for r in rows]):.3f}")
        print(",".join(values))

    print("\nframes_sorted_by_error")
    headers = [
        "bag",
        "stamp",
        "success",
        "trans_error_m",
        "yaw_error_deg",
        "refined_rank",
        "fitness",
        "score_margin",
        "top_tie_count",
        "unique_score_count",
        "xy_spread_m",
        "yaw_spread_deg",
        "nearest_candidate_trans_m",
        "nearest_candidate_yaw_deg",
        "nearest_candidate_rank",
        "total_ms",
    ]
    print(",".join(headers))
    for row in sorted(frame_features, key=lambda r: float(r["trans_error_m"]), reverse=True):
        print(",".join(str(row[h]) for h in headers))

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
