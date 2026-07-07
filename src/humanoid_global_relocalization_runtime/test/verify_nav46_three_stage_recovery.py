#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_nav46_three_stage_recovery.py

文件作用：
  1. 这是 nav46 随机 80 点三层恢复证据校验脚本，不属于功能源码。
  2. 第一层检查短历史 selected support 门控是否 68/80 安全接受且 false_accept=0。
  3. 第二层按当前 average_overlap>=0.95 规则复核长历史 trajectory likelihood 是否再救 11/12 且 false_accept=0。
  4. 第三层检查主动恢复新视角是否救回最后 1 个长历史保守拒绝点。
  5. 该脚本把“能不能救回来”的结论变成可重复门禁，避免只依赖验证报告文字。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/verify_nav46_three_stage_recovery.py
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


MODES = {"body", "registered_world"}
STRICT_AVERAGE_OVERLAP = 0.95


def repo_root_from_script() -> Path:
    """根据脚本路径反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV；缺文件直接抛出清晰错误。"""
    if not path.exists():
        raise FileNotFoundError(f"missing csv: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def as_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """安全读取浮点字段。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def require(condition: bool, message: str, failures: list[str]) -> None:
    """收集失败项。"""
    if not condition:
        failures.append(message)


def check_short_history(summary_path: Path, failures: list[str]) -> dict[str, int]:
    """检查随机 80 点第一层短历史门控产物。"""
    rows = read_csv(summary_path)
    recovered: dict[str, int] = {}
    for mode in MODES:
        label = f"nav46_{mode}_rand80_seed20260702"
        row = next((item for item in rows if item.get("label") == label), None)
        require(row is not None, f"short history missing label {label}", failures)
        if row is None:
            continue
        require(row.get("frames") == "80", f"{label} frames should be 80", failures)
        require(row.get("localized") == "80/80", f"{label} localized should be 80/80", failures)
        require(row.get("temporal_accept") == "68", f"{label} temporal_accept should be 68", failures)
        require(row.get("temporal_reject") == "12", f"{label} temporal_reject should be 12", failures)
        require(
            row.get("temporal_accepted_ref_success") == "68/68",
            f"{label} accepted_ref_success should be 68/68",
            failures,
        )
        require(row.get("temporal_false_accept") == "0", f"{label} false_accept should be 0", failures)
        recovered[mode] = 68
    return recovered


def check_long_history(summary_path: Path, failures: list[str]) -> dict[str, set[str]]:
    """按当前 average overlap 门控复核长历史 trajectory likelihood。"""
    rows_by_mode: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in read_csv(summary_path):
        rows_by_mode[row.get("mode", "")].append(row)

    accepted_ids_by_mode: dict[str, set[str]] = {}
    for mode in MODES:
        rows = rows_by_mode.get(mode, [])
        require(len(rows) == 12, f"{mode} long history should contain 12 rejected targets", failures)
        accepted = [
            row for row in rows
            if row.get("trajectory_decision") == "accept"
            and row.get("trajectory_success") == "1"
            and as_float(row, "average_overlap") >= STRICT_AVERAGE_OVERLAP
        ]
        false_accept = [
            row for row in rows
            if row.get("trajectory_decision") == "accept"
            and as_float(row, "average_overlap") >= STRICT_AVERAGE_OVERLAP
            and row.get("trajectory_success") != "1"
        ]
        rejected_or_filtered = [
            row for row in rows
            if row.get("waypoint_id") not in {item.get("waypoint_id") for item in accepted}
        ]
        require(len(accepted) == 11, f"{mode} long history strict accept should be 11", failures)
        require(not false_accept, f"{mode} long history strict false_accept should be 0", failures)
        require(
            len(rejected_or_filtered) == 1 and rejected_or_filtered[0].get("waypoint_id") == "rand80_reject_12",
            f"{mode} long history should only leave rand80_reject_12 for active recovery",
            failures,
        )
        accepted_ids_by_mode[mode] = {row.get("waypoint_id", "") for row in accepted}
    return accepted_ids_by_mode


def check_active_recovery(summary_path: Path, failures: list[str]) -> dict[str, int]:
    """检查主动恢复新视角是否救回最后一个长历史拒绝点。"""
    rows_by_mode: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in read_csv(summary_path):
        rows_by_mode[row.get("mode", "")].append(row)

    recovered: dict[str, int] = {}
    for mode in MODES:
        rows = rows_by_mode.get(mode, [])
        require(len(rows) == 1, f"{mode} active recovery should contain one target", failures)
        if not rows:
            continue
        row = rows[0]
        require(row.get("waypoint_id") == "rand80_reject_12", f"{mode} active recovery target mismatch", failures)
        require(row.get("trajectory_decision") == "accept", f"{mode} active recovery should accept", failures)
        require(row.get("trajectory_success") == "1", f"{mode} active recovery should succeed", failures)
        require(as_float(row, "trajectory_error_m") <= 0.80, f"{mode} active recovery error should be <=0.80m", failures)
        recovered[mode] = 1
    return recovered


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify nav46 random-80 three-stage recovery evidence.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--artifacts-root", type=Path, default=None, help="验证产物根目录，默认 <workspace>/.codex_tmp")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    artifacts = (args.artifacts_root or workspace / ".codex_tmp").resolve()
    failures: list[str] = []

    try:
        short_recovered = check_short_history(
            artifacts / "nav46_stress_validation/summary_rand80_seed20260702.csv",
            failures,
        )
        long_recovered = check_long_history(
            artifacts / "trajectory_rand80_rejects_long_history/summary.csv",
            failures,
        )
        active_recovered = check_active_recovery(
            artifacts / "trajectory_rand80_reject12_active_view_v2/summary.csv",
            failures,
        )
    except FileNotFoundError as exc:
        print(f"[verify_nav46_three_stage_recovery] FAIL {exc}", file=sys.stderr)
        return 1

    for mode in sorted(MODES):
        total = short_recovered.get(mode, 0) + len(long_recovered.get(mode, set())) + active_recovered.get(mode, 0)
        require(total == 80, f"{mode} three-stage recovered total should be 80, got {total}", failures)
        print(
            "[nav46_three_stage_recovery] "
            f"mode={mode} short={short_recovered.get(mode, 0)} "
            f"long={len(long_recovered.get(mode, set()))} active={active_recovered.get(mode, 0)} total={total}/80"
        )

    if failures:
        print("[verify_nav46_three_stage_recovery] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[verify_nav46_three_stage_recovery] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
