#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_trajectory_long_history_bags.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 检查 nav_drift_test44/45/46 三个真实 bag 的长历史 trajectory likelihood 产物。
  3. 重点约束“短历史/单帧不稳时，长历史恢复是否仍能全量成功且不误接受”。
  4. 该脚本只读取已有 summary.csv 和汇总 CSV，不播放 bag，也不重新运行重定位二进制。
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


EXPECTED = {
    "bag44": {"targets": 25, "single_success": 15, "trajectory_success": 25, "accept": 23},
    "bag45": {"targets": 22, "single_success": 12, "trajectory_success": 22, "accept": 21},
    "bag46": {"targets": 22, "single_success": 18, "trajectory_success": 22, "accept": 22},
}


def repo_root_from_script() -> Path:
    """根据脚本位置反推出工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV 文件；缺失文件直接抛出异常，让门禁明确失败。"""
    if not path.exists():
        raise FileNotFoundError(f"missing csv: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def to_int(row: dict[str, str], key: str) -> int:
    """读取整数字段。"""
    try:
        return int(row.get(key, ""))
    except ValueError as exc:
        raise ValueError(f"invalid integer field {key}={row.get(key)!r}") from exc


def to_float(row: dict[str, str], key: str) -> float:
    """读取浮点字段。"""
    try:
        value = float(row.get(key, ""))
    except ValueError as exc:
        raise ValueError(f"invalid float field {key}={row.get(key)!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"non-finite float field {key}={row.get(key)!r}")
    return value


def fail(message: str, failures: list[str]) -> None:
    """记录失败信息。"""
    failures.append(message)


def check_summary_csv(summary_path: Path, failures: list[str]) -> None:
    """检查三包长历史汇总 CSV 的整体统计。"""
    rows = {row.get("label", ""): row for row in read_csv(summary_path)}
    missing = sorted(set(EXPECTED) - set(rows))
    if missing:
        fail(f"long-history summary missing labels: {missing}", failures)
        return

    for label, expected in EXPECTED.items():
        row = rows[label]
        targets = to_int(row, "targets")
        single_success = to_int(row, "single_success")
        trajectory_success = to_int(row, "trajectory_success")
        trajectory_accept = to_int(row, "trajectory_accept")
        trajectory_accept_success = to_int(row, "trajectory_accept_success")
        trajectory_false_accept = to_int(row, "trajectory_false_accept")
        trajectory_reject_success = to_int(row, "trajectory_reject_success")
        max_error = to_float(row, "max_trajectory_error_m")
        min_margin = to_float(row, "min_margin")
        max_rss = to_float(row, "max_peak_rss_mb")

        print(
            "[trajectory_long_history_bags] "
            f"{label} targets={targets} single={single_success} trajectory={trajectory_success} "
            f"accept={trajectory_accept} accept_success={trajectory_accept_success} "
            f"false_accept={trajectory_false_accept} reject_success={trajectory_reject_success} "
            f"max_error={max_error:.3f}m min_margin={min_margin:.6f} max_rss={max_rss:.1f}MB"
        )

        if targets != expected["targets"]:
            fail(f"{label} targets changed: {targets}", failures)
        if single_success != expected["single_success"]:
            fail(f"{label} single_success changed: {single_success}", failures)
        if trajectory_success != expected["trajectory_success"]:
            fail(f"{label} trajectory_success changed: {trajectory_success}", failures)
        if trajectory_accept != expected["accept"]:
            fail(f"{label} trajectory_accept changed: {trajectory_accept}", failures)
        if trajectory_accept_success != trajectory_accept:
            fail(f"{label} accepted trajectory rows are not all successful", failures)
        if trajectory_false_accept != 0:
            fail(f"{label} has trajectory_false_accept={trajectory_false_accept}", failures)
        if trajectory_success != targets:
            fail(f"{label} long-history trajectory did not solve every target", failures)
        if max_error > 0.80:
            fail(f"{label} max trajectory error {max_error:.3f}m exceeds 0.80m", failures)
        if min_margin < 0.0015:
            fail(f"{label} min margin {min_margin:.6f} below configured gate", failures)
        if max_rss > 800.0:
            fail(f"{label} peak RSS {max_rss:.1f}MB unexpectedly high", failures)


def check_raw_runs(artifacts_root: Path, failures: list[str]) -> None:
    """检查每个原始 summary.csv，确认没有隐藏的 accept 失败行。"""
    for label in EXPECTED:
        path = artifacts_root / f"trajectory_long_history_full_{label}" / "summary.csv"
        rows = read_csv(path)
        accepted = [row for row in rows if row.get("trajectory_decision") == "accept"]
        false_accept = [row for row in accepted if row.get("trajectory_success") != "1"]
        solved = [row for row in rows if row.get("trajectory_success") == "1"]
        if len(rows) != EXPECTED[label]["targets"]:
            fail(f"{label} raw summary row count changed: {len(rows)}", failures)
        if len(solved) != len(rows):
            fail(f"{label} raw long-history success is {len(solved)}/{len(rows)}", failures)
        if false_accept:
            ids = [row.get("waypoint_id", "") for row in false_accept]
            fail(f"{label} raw summary has false accept waypoints: {ids}", failures)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify long-history trajectory likelihood across nav bags.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--artifacts-root", type=Path, default=None, help="验证产物目录，默认 workspace/.codex_tmp")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    artifacts_root = (args.artifacts_root or workspace / ".codex_tmp").resolve()
    summary_path = artifacts_root / "trajectory_long_history_summary.csv"
    failures: list[str] = []

    try:
        check_summary_csv(summary_path, failures)
        check_raw_runs(artifacts_root, failures)
    except (FileNotFoundError, ValueError) as exc:
        print(f"[verify_trajectory_long_history_bags] FAIL {exc}", file=sys.stderr)
        return 1

    if failures:
        print("[verify_trajectory_long_history_bags] FAIL", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("[verify_trajectory_long_history_bags] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
