#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_online_smoke_evidence.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 检查 online_smoke_evidence.csv 中记录的在线 debug 冒烟覆盖是否完整。
  3. 防止后续交接时只知道“跑过在线 smoke”，却不知道具体覆盖了哪些输入模式和 bag。
  4. 校验 recovery_status 里的 map_odom_x/y/yaw 与实际 /verified_map_to_odom PoseStamped 记录一致。
  5. 该脚本只校验证据记录和路径存在；真正运行在线 smoke 仍使用 run_online_debug_smoke.py。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/verify_online_smoke_evidence.py
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


EXPECTED_ROWS = {
    ("registered_world", "nav_drift_test44"),
    ("registered_world", "nav_drift_test45"),
    ("registered_world", "nav_drift_test46"),
    ("body", "real_body_nav_drift_test46"),
    ("body", "synthetic_body_nav_drift_test43"),
    ("body", "synthetic_body_nav_drift_test44"),
    ("body", "synthetic_body_nav_drift_test45"),
}


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def read_rows(path: Path) -> list[dict[str, str]]:
    """读取在线 smoke 证据 CSV。"""
    if not path.exists():
        raise FileNotFoundError(f"missing online smoke evidence: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def as_float(row: dict[str, str], key: str) -> float:
    """读取有限浮点数；证据里的 verified pose 不能是空值或 NaN。"""
    try:
        value = float(row.get(key, ""))
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def status_tokens(status: str) -> dict[str, str]:
    """解析 recovery_status 中的 key=value 字段。"""
    parsed: dict[str, str] = {}
    for token in status.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parsed[key] = value
    return parsed


def finite_token(tokens: dict[str, str], key: str) -> float:
    """从 status token 中读取有限浮点数。"""
    try:
        value = float(tokens.get(key, ""))
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def yaw_diff_deg(a: float, b: float) -> float:
    """计算两个 yaw 角的最小差值，单位度。"""
    diff = math.radians(a - b)
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return abs(math.degrees(diff))


def require(condition: bool, message: str, failures: list[str]) -> None:
    """收集失败项。"""
    if not condition:
        failures.append(message)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify online smoke evidence coverage.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="online_smoke_evidence.csv 路径，默认使用包 test 目录下的文件",
    )
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    package = workspace / "src/humanoid_global_relocalization_runtime"
    evidence_csv = args.csv or package / "test/online_smoke_evidence.csv"
    failures: list[str] = []

    try:
        rows = read_rows(evidence_csv)
    except FileNotFoundError as exc:
        print(f"[verify_online_smoke_evidence] FAIL {exc}", file=sys.stderr)
        return 1

    actual_keys = {(row.get("input_mode", ""), row.get("bag_label", "")) for row in rows}
    require(actual_keys == EXPECTED_ROWS, f"online smoke coverage mismatch: {sorted(actual_keys)}", failures)

    for row in rows:
        key = (row.get("input_mode", ""), row.get("bag_label", ""))
        require(row.get("result") == "PASS", f"{key} result is not PASS", failures)
        bag_path = Path(row.get("bag_path", ""))
        config_path = workspace / row.get("config_path", "")
        require(bag_path.exists(), f"{key} bag path missing: {bag_path}", failures)
        require(config_path.exists(), f"{key} config path missing: {config_path}", failures)
        for field in [
            "verified_x_m",
            "verified_y_m",
            "verified_z_m",
            "map_odom_x_m",
            "map_odom_y_m",
            "map_odom_yaw_deg",
        ]:
            require(math.isfinite(as_float(row, field)), f"{key} {field} is not finite", failures)
        tokens = status_tokens(row.get("status", ""))
        require(tokens.get("state") == "verified", f"{key} status state is not verified", failures)
        require(tokens.get("input_mode") == row.get("input_mode"), f"{key} status input_mode mismatch", failures)
        require(tokens.get("refined_converged") == "true", f"{key} refined_converged is not true", failures)
        require(math.isfinite(finite_token(tokens, "refined_fitness")), f"{key} refined_fitness is not finite", failures)
        status_map_x = finite_token(tokens, "map_odom_x")
        status_map_y = finite_token(tokens, "map_odom_y")
        status_map_yaw = finite_token(tokens, "map_odom_yaw_deg")
        require(math.isfinite(status_map_x), f"{key} status map_odom_x is not finite", failures)
        require(math.isfinite(status_map_y), f"{key} status map_odom_y is not finite", failures)
        require(math.isfinite(status_map_yaw), f"{key} status map_odom_yaw_deg is not finite", failures)
        require(
            abs(status_map_x - as_float(row, "map_odom_x_m")) <= 0.002,
            f"{key} status map_odom_x differs from PoseStamped CSV",
            failures,
        )
        require(
            abs(status_map_y - as_float(row, "map_odom_y_m")) <= 0.002,
            f"{key} status map_odom_y differs from PoseStamped CSV",
            failures,
        )
        require(
            yaw_diff_deg(status_map_yaw, as_float(row, "map_odom_yaw_deg")) <= 0.01,
            f"{key} status map_odom_yaw_deg differs from PoseStamped CSV",
            failures,
        )
        try:
            selected_support = int(tokens.get("selected_support", "0"))
            min_support = int(tokens.get("min_support", "0"))
        except ValueError:
            selected_support = 0
            min_support = 0
        require(selected_support >= min_support >= 1, f"{key} selected_support below min_support", failures)

    print(f"[online_smoke_evidence] rows={len(rows)} coverage={len(actual_keys)}")
    if failures:
        print("[verify_online_smoke_evidence] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[verify_online_smoke_evidence] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
