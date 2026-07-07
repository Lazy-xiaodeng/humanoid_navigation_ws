#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_online_trajectory_recovery_smoke.py

文件作用：
  1. 这是在线 trajectory recovery 专项 smoke 的证据校验脚本，不属于功能源码。
  2. 检查 run_online_trajectory_recovery_smoke.py 生成的临时 YAML 和 pose CSV 是否存在。
  3. 确认 CSV 中确实收到了 state=verified_trajectory_single_agreement，而不是普通 state=verified。
  4. 同时校验 map->base、map->odom、GICP fitness 等关键字段是有限值，防止空 CSV 或错误状态被误认为通过。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/verify_online_trajectory_recovery_smoke.py
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


EXPECTED_STATE = "verified_trajectory_single_agreement"
MAX_REFINED_FITNESS = 0.04


def repo_root_from_script() -> Path:
    """根据脚本路径反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def read_rows(path: Path) -> list[dict[str, str]]:
    """读取在线 trajectory recovery smoke 输出的 pose CSV。"""
    if not path.exists():
        raise FileNotFoundError(f"missing trajectory recovery smoke pose csv: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def as_float(row: dict[str, str], key: str) -> float:
    """读取 CSV 中必须存在的有限浮点字段。"""
    try:
        value = float(row.get(key, ""))
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def status_tokens(status: str) -> dict[str, str]:
    """把 recovery_status 中的 key=value 文本拆成字典，便于检查状态和 fitness。"""
    parsed: dict[str, str] = {}
    for token in status.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parsed[key.strip()] = value.strip()
    return parsed


def yaw_diff_deg(a: float, b: float) -> float:
    """计算两个 yaw 角的最小差值，单位度。"""
    diff = math.radians(a - b)
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return abs(math.degrees(diff))


def require(condition: bool, message: str, failures: list[str]) -> None:
    """收集失败项，保证一次运行能看到所有缺失字段。"""
    if not condition:
        failures.append(message)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify online trajectory recovery smoke evidence.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument(
        "--pose-csv",
        type=Path,
        default=None,
        help="online_trajectory_recovery_smoke_pose.csv 路径，默认使用 .codex_tmp 下的产物",
    )
    parser.add_argument(
        "--generated-yaml",
        type=Path,
        default=None,
        help="online_trajectory_recovery_smoke.yaml 路径，默认使用 .codex_tmp 下的产物",
    )
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    pose_csv = args.pose_csv or workspace / ".codex_tmp/online_trajectory_recovery_smoke_pose.csv"
    generated_yaml = args.generated_yaml or workspace / ".codex_tmp/online_trajectory_recovery_smoke.yaml"
    failures: list[str] = []

    require(generated_yaml.exists(), f"generated yaml missing: {generated_yaml}", failures)
    try:
        rows = read_rows(pose_csv)
    except FileNotFoundError as exc:
        print(f"[verify_online_trajectory_recovery_smoke] FAIL {exc}", file=sys.stderr)
        return 1

    require(len(rows) >= 1, "trajectory recovery smoke csv has no rows", failures)
    matched = 0
    best_fitness = math.inf
    for index, row in enumerate(rows, start=1):
        for field in [
            "verified_x_m",
            "verified_y_m",
            "verified_z_m",
            "map_odom_x_m",
            "map_odom_y_m",
            "map_odom_yaw_deg",
        ]:
            require(math.isfinite(as_float(row, field)), f"row {index} {field} is not finite", failures)

        tokens = status_tokens(row.get("status", ""))
        if tokens.get("state") == EXPECTED_STATE:
            matched += 1
            require(tokens.get("refined_converged") == "true", f"row {index} refined did not converge", failures)
            try:
                fitness = float(tokens.get("refined_fitness", "nan"))
            except ValueError:
                fitness = math.nan
            require(math.isfinite(fitness), f"row {index} refined_fitness is not finite", failures)
            require(
                fitness <= MAX_REFINED_FITNESS,
                f"row {index} refined_fitness={fitness:.6f} exceeds {MAX_REFINED_FITNESS:.3f}",
                failures,
            )
            try:
                status_yaw = float(tokens.get("map_odom_yaw_deg", "nan"))
            except ValueError:
                status_yaw = math.nan
            require(math.isfinite(status_yaw), f"row {index} map_odom_yaw_deg is not finite in status", failures)
            require(
                yaw_diff_deg(status_yaw, as_float(row, "map_odom_yaw_deg")) <= 0.01,
                f"row {index} status map_odom_yaw_deg differs from PoseStamped CSV",
                failures,
            )
            if math.isfinite(fitness):
                best_fitness = min(best_fitness, fitness)

    require(matched >= 1, f"no row contains state={EXPECTED_STATE}", failures)
    print(
        "[online_trajectory_recovery_smoke] "
        f"rows={len(rows)} matched={matched} best_refined_fitness={best_fitness:.6f}"
    )

    if failures:
        print("[verify_online_trajectory_recovery_smoke] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[verify_online_trajectory_recovery_smoke] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
