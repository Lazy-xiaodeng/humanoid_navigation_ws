#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_online_active_view_status_smoke.py

文件作用：
  1. 这是在线主动恢复状态 smoke 的证据校验脚本，不属于功能源码。
  2. 检查 run_online_active_view_status_smoke.py 生成的 CSV 和临时 YAML 是否存在。
  3. 确认 recovery_status 中包含 need_active_view_* 状态、recovery_hint=active_view 和 trajectory_attempted=true。
  4. 该脚本用于防止后续改动把“需要主动采集新视角”的状态语义弄丢。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/verify_online_active_view_status_smoke.py
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def read_rows(path: Path) -> list[dict[str, str]]:
    """读取主动恢复状态 smoke CSV。"""
    if not path.exists():
        raise FileNotFoundError(f"missing active-view status smoke csv: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def as_float(row: dict[str, str], key: str) -> float:
    """读取有限浮点字段。"""
    try:
        value = float(row.get(key, ""))
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def require(condition: bool, message: str, failures: list[str]) -> None:
    """收集失败项。"""
    if not condition:
        failures.append(message)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify online active-view status smoke evidence.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--csv", type=Path, default=None, help="online_active_view_status_smoke.csv 路径")
    parser.add_argument("--yaml", type=Path, default=None, help="online_active_view_status_smoke.yaml 路径")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    evidence_csv = args.csv or workspace / ".codex_tmp/online_active_view_status_smoke.csv"
    generated_yaml = args.yaml or workspace / ".codex_tmp/online_active_view_status_smoke.yaml"
    failures: list[str] = []

    require(generated_yaml.exists(), f"generated yaml missing: {generated_yaml}", failures)
    try:
        rows = read_rows(evidence_csv)
    except FileNotFoundError as exc:
        print(f"[verify_online_active_view_status_smoke] FAIL {exc}", file=sys.stderr)
        return 1

    require(len(rows) >= 1, "active-view status smoke csv has no rows", failures)
    row = rows[-1] if rows else {}
    state = row.get("state", "")
    require(state.startswith("need_active_view_"), f"state should start with need_active_view_, got {state}", failures)
    require(row.get("recovery_hint") == "active_view", "recovery_hint should be active_view", failures)
    require(row.get("trajectory_attempted") == "true", "trajectory_attempted should be true", failures)
    require(as_float(row, "trajectory_support") >= 1.0, "trajectory_support should be finite and >= 1", failures)
    require(
        math.isfinite(as_float(row, "trajectory_average_overlap")),
        "trajectory_average_overlap should be finite",
        failures,
    )
    require(math.isfinite(as_float(row, "trajectory_margin")), "trajectory_margin should be finite", failures)

    print(
        "[online_active_view_status_smoke] "
        f"state={state} support={row.get('trajectory_support')} "
        f"overlap={row.get('trajectory_average_overlap')} margin={row.get('trajectory_margin')}"
    )
    if failures:
        print("[verify_online_active_view_status_smoke] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[verify_online_active_view_status_smoke] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
