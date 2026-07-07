#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_nav46_topk_strategy.py

文件作用：
  1. 这是 nav46 hard-frame top-K 分层恢复策略校验脚本，不属于线上功能源码。
  2. 检查长历史二阶段使用 top30 时，是否仍能安全救回 11/12 个短历史拒绝点。
  3. 检查主动新视角使用 top30 时，最后一个歧义点是否仍被保守拒绝，避免误接受。
  4. 检查主动新视角升级到 top60 时，最后一个歧义点是否能被救回。
  5. 该脚本用于防止后续调参把“先 top30、必要时 top60 深搜”的成本策略改坏。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/verify_nav46_topk_strategy.py
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


MODES = {"body", "registered_world"}


def repo_root_from_script() -> Path:
    """根据脚本路径反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV；缺文件时直接让调用方知道是哪份证据缺失。"""
    if not path.exists():
        raise FileNotFoundError(f"missing csv: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def as_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """安全读取浮点字段，用于误差和 overlap 门控判断。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def require(condition: bool, message: str, failures: list[str]) -> None:
    """收集失败项，最后一次性打印，方便看完整问题。"""
    if not condition:
        failures.append(message)


def rows_by_mode(path: Path) -> dict[str, list[dict[str, str]]]:
    """按输入模式分组 summary CSV。"""
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in read_csv(path):
        grouped[row.get("mode", "")].append(row)
    return grouped


def check_long_top30(path: Path, failures: list[str]) -> None:
    """验证 top30 长历史二阶段可救回 11/12，且最后只留下 rand80_reject_12。"""
    grouped = rows_by_mode(path)
    for mode in MODES:
        rows = grouped.get(mode, [])
        require(len(rows) == 12, f"{mode} top30 long history should contain 12 targets", failures)
        accepted = [
            row for row in rows
            if row.get("trajectory_decision") == "accept" and row.get("trajectory_success") == "1"
        ]
        false_accept = [
            row for row in rows
            if row.get("trajectory_decision") == "accept" and row.get("trajectory_success") != "1"
        ]
        left = sorted(
            row.get("waypoint_id", "")
            for row in rows
            if row.get("waypoint_id", "") not in {item.get("waypoint_id", "") for item in accepted}
        )
        require(len(accepted) == 11, f"{mode} top30 long history should accept 11 successes, got {len(accepted)}", failures)
        require(not false_accept, f"{mode} top30 long history false_accept should be 0", failures)
        require(left == ["rand80_reject_12"], f"{mode} top30 long history should leave only rand80_reject_12, got {left}", failures)
        dangerous_recovered = {
            row.get("waypoint_id", "")
            for row in accepted
            if as_float(row, "single_error_m") >= 8.0 or as_float(row, "single_yaw_deg") >= 80.0
        }
        require(
            {"rand80_reject_02", "rand80_reject_03", "rand80_reject_04", "rand80_reject_05", "rand80_reject_10"}.issubset(
                dangerous_recovered
            ),
            f"{mode} top30 long history did not recover all dangerous single-frame failures: {sorted(dangerous_recovered)}",
            failures,
        )
        print(
            "[nav46_topk_strategy] "
            f"mode={mode} top30_long accepted={len(accepted)}/12 false_accept={len(false_accept)} left={left}"
        )


def check_active_top30_reject(path: Path, failures: list[str]) -> None:
    """验证主动新视角 top30 仍保守拒绝最后歧义点，而不是误接受。"""
    grouped = rows_by_mode(path)
    for mode in MODES:
        rows = grouped.get(mode, [])
        require(len(rows) == 1, f"{mode} top30 active should contain one target", failures)
        if not rows:
            continue
        row = rows[0]
        require(row.get("waypoint_id") == "rand80_reject_12", f"{mode} top30 active target mismatch", failures)
        require(row.get("trajectory_decision") != "accept", f"{mode} top30 active should reject/candidate", failures)
        require(row.get("trajectory_success") != "1", f"{mode} top30 active should not be a successful recovery", failures)
        require(as_float(row, "margin") < 0.005, f"{mode} top30 active margin should stay below fallback threshold", failures)
        print(
            "[nav46_topk_strategy] "
            f"mode={mode} top30_active decision={row.get('trajectory_decision')} "
            f"error={row.get('trajectory_error_m')}m/{row.get('trajectory_yaw_deg')}deg margin={row.get('margin')}"
        )


def check_active_top60_accept(path: Path, failures: list[str]) -> None:
    """验证主动新视角 top60 可救回最后歧义点。"""
    grouped = rows_by_mode(path)
    for mode in MODES:
        rows = grouped.get(mode, [])
        require(len(rows) == 1, f"{mode} top60 active should contain one target", failures)
        if not rows:
            continue
        row = rows[0]
        require(row.get("waypoint_id") == "rand80_reject_12", f"{mode} top60 active target mismatch", failures)
        require(row.get("trajectory_decision") == "accept", f"{mode} top60 active should accept", failures)
        require(row.get("trajectory_success") == "1", f"{mode} top60 active should succeed", failures)
        require(as_float(row, "trajectory_error_m") <= 0.80, f"{mode} top60 active error should be <=0.80m", failures)
        print(
            "[nav46_topk_strategy] "
            f"mode={mode} top60_active decision={row.get('trajectory_decision')} "
            f"error={row.get('trajectory_error_m')}m/{row.get('trajectory_yaw_deg')}deg margin={row.get('margin')}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify nav46 top-K staged recovery strategy.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--artifacts-root", type=Path, default=None, help="验证产物根目录，默认 <workspace>/.codex_tmp")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    artifacts = (args.artifacts_root or workspace / ".codex_tmp").resolve()
    failures: list[str] = []

    try:
        check_long_top30(
            artifacts / "trajectory_rand80_rejects_long_history_top30_probe/summary.csv",
            failures,
        )
        check_active_top30_reject(
            artifacts / "trajectory_rand80_reject12_active_view_top30_probe/summary.csv",
            failures,
        )
        check_active_top60_accept(
            artifacts / "trajectory_rand80_reject12_active_view_v2/summary.csv",
            failures,
        )
    except FileNotFoundError as exc:
        print(f"[verify_nav46_topk_strategy] FAIL {exc}", file=sys.stderr)
        return 1

    if failures:
        print("[verify_nav46_topk_strategy] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[verify_nav46_topk_strategy] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
