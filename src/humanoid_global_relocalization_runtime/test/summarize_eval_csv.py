#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
summarize_eval_csv.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 读取 global_relocalization_metrics.csv，按 map_path/refine_method 汇总定位率、成功率、误差和耗时。
  3. 用于快速判断一轮 bag sweep 的参数组合是否值得继续扩大验证。
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


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: summarize_eval_csv.py <global_relocalization_metrics.csv>", file=sys.stderr)
        return 2

    csv_path = Path(argv[1])
    groups: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    with csv_path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            # 多个模拟场景对应同一次真实评估。汇总算法效果时先只取 no_prior 场景，避免重复计数。
            if row.get("scenario_name") != "arbitrary_start_no_prior":
                continue
            groups[(row.get("map_path", ""), row.get("refine_method", ""))].append(row)

    print(
        "map,refine,n,localized_rate,success_rate,reference_rate,"
        "success_over_reference_rate,"
        "median_trans_err,median_yaw_err,median_search_ms,median_refine_ms,"
        "median_total_ms,median_delta_cpu_ms,peak_rss_mb"
    )
    for (map_path, refine_method), rows in sorted(groups.items()):
        n = len(rows)
        localized = sum(1 for r in rows if r.get("localized") == "1")
        success = sum(1 for r in rows if r.get("success") == "1")
        reference = sum(1 for r in rows if r.get("has_reference") == "1")

        trans = [to_float(r, "translation_error_m") for r in rows if to_float(r, "translation_error_m") >= 0.0]
        yaw = [to_float(r, "yaw_error_deg") for r in rows if to_float(r, "yaw_error_deg") >= 0.0]
        search = [to_float(r, "search_ms") for r in rows]
        refine = [to_float(r, "refine_ms") for r in rows]
        total = [to_float(r, "total_ms") for r in rows]
        delta_cpu = [
            to_float(r, "delta_user_cpu_ms") + to_float(r, "delta_system_cpu_ms")
            for r in rows
            if math.isfinite(to_float(r, "delta_user_cpu_ms"))
            and math.isfinite(to_float(r, "delta_system_cpu_ms"))
        ]
        peak_rss = [to_float(r, "peak_rss_mb") for r in rows]

        def med(values: list[float]) -> float:
            finite = [v for v in values if math.isfinite(v)]
            return statistics.median(finite) if finite else math.nan

        print(
            f"{Path(map_path).name},{refine_method},{n},"
            f"{localized / n if n else 0:.3f},"
            f"{success / n if n else 0:.3f},"
            f"{reference / n if n else 0:.3f},"
            f"{success / reference if reference else 0:.3f},"
            f"{med(trans):.3f},{med(yaw):.3f},"
            f"{med(search):.1f},{med(refine):.1f},{med(total):.1f},"
            f"{med(delta_cpu):.1f},{max([v for v in peak_rss if math.isfinite(v)], default=math.nan):.1f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
