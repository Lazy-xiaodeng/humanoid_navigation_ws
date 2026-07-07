#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_resource_metrics.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 检查关键 metrics CSV 是否持续记录耗时、CPU、内存和线程数等资源字段。
  3. 防止后续修改 evaluator 后，算法准确率还在但资源消耗统计被遗漏或写成无效值。
  4. 该脚本只验证字段存在和数值合理性，不用固定耗时上限卡死不同机器的性能波动。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/verify_resource_metrics.py
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path


REQUIRED_COLUMNS = [
    "build_index_ms",
    "search_ms",
    "refine_ms",
    "total_ms",
    "user_cpu_ms",
    "system_cpu_ms",
    "delta_user_cpu_ms",
    "delta_system_cpu_ms",
    "rss_mb",
    "peak_rss_mb",
    "virtual_mem_mb",
    "thread_count",
]


def repo_root_from_script() -> Path:
    """根据脚本路径反推出工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def read_no_prior_rows(path: Path) -> list[dict[str, str]]:
    """读取任意点启动场景，避免同一帧多个模拟先验重复计数。"""
    if not path.exists():
        raise FileNotFoundError(f"metrics csv does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return [row for row in csv.DictReader(f) if row.get("scenario_name") == "arbitrary_start_no_prior"]


def as_float(row: dict[str, str], key: str) -> float:
    """安全读取浮点字段。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return math.nan


def finite_values(rows: list[dict[str, str]], key: str) -> list[float]:
    """返回某字段的有限数值列表。"""
    values = [as_float(row, key) for row in rows]
    return [value for value in values if math.isfinite(value)]


def require(condition: bool, message: str, failures: list[str]) -> None:
    """统一收集失败项。"""
    if not condition:
        failures.append(message)


def check_metrics_file(label: str, path: Path, expected_rows: int, failures: list[str]) -> None:
    """检查单个 metrics CSV 的资源字段和基本数值合理性。"""
    rows = read_no_prior_rows(path)
    require(len(rows) == expected_rows, f"{label} row count {len(rows)} != {expected_rows}", failures)
    if not rows:
        return

    missing_columns = [column for column in REQUIRED_COLUMNS if column not in rows[0]]
    require(not missing_columns, f"{label} missing resource columns: {missing_columns}", failures)

    for column in REQUIRED_COLUMNS:
        values = finite_values(rows, column)
        require(len(values) == len(rows), f"{label} column {column} has non-finite values", failures)

    total = finite_values(rows, "total_ms")
    search = finite_values(rows, "search_ms")
    refine = finite_values(rows, "refine_ms")
    delta_user = finite_values(rows, "delta_user_cpu_ms")
    delta_system = finite_values(rows, "delta_system_cpu_ms")
    rss = finite_values(rows, "rss_mb")
    peak_rss = finite_values(rows, "peak_rss_mb")
    threads = finite_values(rows, "thread_count")

    require(min(total) > 0.0, f"{label} total_ms should be positive", failures)
    require(min(search) > 0.0, f"{label} search_ms should be positive", failures)
    require(min(refine) >= 0.0, f"{label} refine_ms should be non-negative", failures)
    require(min(delta_user) >= 0.0, f"{label} delta_user_cpu_ms should be non-negative", failures)
    require(min(delta_system) >= 0.0, f"{label} delta_system_cpu_ms should be non-negative", failures)
    require(min(rss) > 0.0, f"{label} rss_mb should be positive", failures)
    require(min(peak_rss) >= max(rss), f"{label} peak_rss_mb should be >= observed rss_mb", failures)
    require(min(threads) >= 1.0, f"{label} thread_count should be at least 1", failures)

    delta_cpu = [u + s for u, s in zip(delta_user, delta_system)]
    print(
        f"[resource_metrics:{label}] rows={len(rows)} "
        f"median_total_ms={statistics.median(total):.1f} "
        f"median_delta_cpu_ms={statistics.median(delta_cpu):.1f} "
        f"peak_rss_mb={max(peak_rss):.1f} "
        f"threads={min(threads):.0f}-{max(threads):.0f}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify resource metrics columns and finite values.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--artifacts-root", type=Path, default=None, help="验证产物根目录，默认 <workspace>/.codex_tmp")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    artifacts_root = (args.artifacts_root or workspace / ".codex_tmp").resolve()
    failures: list[str] = []

    try:
        check_metrics_file(
            "nav_drift_bidirectional",
            artifacts_root / "global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_metrics.csv",
            30,
            failures,
        )
        check_metrics_file(
            "nav_drift_causal",
            artifacts_root / "global_relocalization_nav_drift_extended_cpp_temporal_causal_v1/global_relocalization_metrics.csv",
            30,
            failures,
        )
        check_metrics_file(
            "synthetic_body",
            artifacts_root / "global_relocalization_synthetic_body_nav43_v1/global_relocalization_metrics.csv",
            5,
            failures,
        )
        check_metrics_file(
            "synthetic_body_three_bag",
            artifacts_root / "global_relocalization_synthetic_body_nav_drift_all_v1/global_relocalization_metrics.csv",
            15,
            failures,
        )
    except FileNotFoundError as exc:
        failures.append(str(exc))

    if failures:
        print("[verify_resource_metrics] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[verify_resource_metrics] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
