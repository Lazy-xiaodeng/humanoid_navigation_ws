#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_refine_sweep.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 检查精匹配方法和地图 sweep 的关键产物是否还存在、统计口径是否没有退化。
  3. focused sweep 用于验证 none/ICP/GICP 的效果差异；baseline sweep 用于验证 NDT 和多地图候选机制跑通过。
  4. 该脚本不重新跑耗时实验，只读取已有 CSV，作为“多方法验证证据没有丢失”的回归门禁。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/verify_refine_sweep.py
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path


def repo_root_from_script() -> Path:
    """根据脚本所在位置反推出 humanoid_ws 根目录。"""
    return Path(__file__).resolve().parents[3]


def read_no_prior_rows(path: Path) -> list[dict[str, str]]:
    """读取 metrics CSV 中任意点启动场景，避免多个模拟先验重复计数。"""
    if not path.exists():
        raise FileNotFoundError(f"metrics csv does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return [row for row in csv.DictReader(f) if row.get("scenario_name") == "arbitrary_start_no_prior"]


def as_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """安全读取浮点字段。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def median(values: list[float]) -> float:
    """只对有限数值求中位数。"""
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else math.nan


def require(condition: bool, message: str, failures: list[str]) -> None:
    """统一收集失败项，便于一次输出所有问题。"""
    if not condition:
        failures.append(message)


def group_by_method(rows: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[row.get("refine_method", "")].append(row)
    return grouped


def method_stats(rows: list[dict[str, str]]) -> dict[str, float]:
    """汇总单个 refine 方法的核心效果。"""
    trans = [as_float(row, "translation_error_m") for row in rows if as_float(row, "translation_error_m") >= 0.0]
    yaw = [as_float(row, "yaw_error_deg") for row in rows if as_float(row, "yaw_error_deg") >= 0.0]
    total = [as_float(row, "total_ms") for row in rows]
    return {
        "n": float(len(rows)),
        "localized": float(sum(1 for row in rows if row.get("localized") == "1")),
        "success": float(sum(1 for row in rows if row.get("success") == "1")),
        "median_trans": median(trans),
        "median_yaw": median(yaw),
        "median_total": median(total),
    }


def check_focused_sweep(path: Path, failures: list[str]) -> None:
    """检查 focused sweep 中 none/ICP/GICP 的效果差异是否仍与报告一致。"""
    rows = read_no_prior_rows(path)
    grouped = group_by_method(rows)
    required_methods = {"none", "icp", "gicp"}
    require(required_methods.issubset(grouped), f"focused sweep missing methods: {required_methods - set(grouped)}", failures)

    stats = {method: method_stats(grouped.get(method, [])) for method in required_methods}
    for method in sorted(required_methods):
        item = stats[method]
        print(
            f"[refine_sweep:focused] method={method} n={item['n']:.0f} "
            f"success={item['success']:.0f} median_trans={item['median_trans']:.3f} "
            f"median_yaw={item['median_yaw']:.3f} median_total_ms={item['median_total']:.1f}"
        )

    require(stats["none"]["n"] == 15.0, "focused none should have 15 sampled frames", failures)
    require(stats["icp"]["n"] == 15.0, "focused icp should have 15 sampled frames", failures)
    require(stats["gicp"]["n"] == 15.0, "focused gicp should have 15 sampled frames", failures)
    require(stats["none"]["success"] == 3.0, "focused none success should remain 3/15", failures)
    require(stats["icp"]["success"] == 13.0, "focused icp success should remain 13/15", failures)
    require(stats["gicp"]["success"] == 13.0, "focused gicp success should remain 13/15", failures)
    require(
        stats["gicp"]["median_trans"] < stats["icp"]["median_trans"] < stats["none"]["median_trans"],
        "focused median translation should show GICP < ICP < none",
        failures,
    )
    require(
        stats["gicp"]["median_yaw"] < stats["icp"]["median_yaw"] < stats["none"]["median_yaw"],
        "focused median yaw should show GICP < ICP < none",
        failures,
    )


def check_baseline_coverage(path: Path, failures: list[str]) -> None:
    """检查 baseline sweep 覆盖了多地图和 NDT，不用它评价最终推荐成功率。"""
    rows = read_no_prior_rows(path)
    methods = {row.get("refine_method", "") for row in rows}
    maps = {Path(row.get("map_path", "")).name for row in rows}
    print(f"[refine_sweep:baseline] methods={sorted(methods)} maps={sorted(maps)} rows={len(rows)}")
    require({"none", "icp", "gicp", "ndt"}.issubset(methods), "baseline sweep should cover none/icp/gicp/ndt", failures)
    require({"hall.pcd", "hall_open3d_grounded.pcd"}.issubset(maps), "baseline sweep should cover both map candidates", failures)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify refine/map sweep artifacts.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--artifacts-root", type=Path, default=None, help="验证产物根目录，默认 <workspace>/.codex_tmp")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    artifacts_root = (args.artifacts_root or workspace / ".codex_tmp").resolve()
    failures: list[str] = []

    try:
        check_focused_sweep(
            artifacts_root / "global_relocalization_nav_drift_focused_v1/global_relocalization_metrics.csv",
            failures,
        )
        check_baseline_coverage(
            artifacts_root / "global_relocalization_nav_drift_baseline_v1/global_relocalization_metrics.csv",
            failures,
        )
    except FileNotFoundError as exc:
        failures.append(str(exc))

    if failures:
        print("[verify_refine_sweep] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[verify_refine_sweep] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
