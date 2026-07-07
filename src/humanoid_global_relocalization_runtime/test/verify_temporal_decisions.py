#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_temporal_decisions.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 读取 global_relocalization_temporal_decisions.csv。
  3. 自动检查 temporal decision 策略是否满足当前 nav_drift 回归门禁。
  4. 防止后续改参数/改代码后，人工漏看“危险帧被误接受”或“可救回帧没有救回”。

默认门禁基于当前 30 帧 support>=2 回归：
  - 总帧数应为 30。
  - accept 应为 28。
  - reject 应为 2。
  - accepted refined success 应为 28/28。
  - nav43 两个危险 hard failure 必须 reject。
  - nav45 hard failure 必须 accept 且 refined success。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/verify_temporal_decisions.py \
    .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_temporal_decisions.csv

因果窗口验证示例：
  python3 src/humanoid_global_relocalization_runtime/test/verify_temporal_decisions.py \
    .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_causal_v1/global_relocalization_temporal_decisions.csv \
    --expected-accept 25 \
    --expected-reject 5 \
    --expect-causal-warmup-rejects
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def as_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def row_key(row: dict[str, str]) -> tuple[str, str]:
    """用 bag 名和 stamp 定位 hard case，避免绝对路径变化影响校验。"""
    return Path(row.get("bag_path", "")).name, row.get("stamp_sec", "")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify temporal decision CSV gates.")
    parser.add_argument("csv_path", type=Path, help="global_relocalization_temporal_decisions.csv")
    parser.add_argument("--expected-total", type=int, default=30, help="期望总行数")
    parser.add_argument("--expected-accept", type=int, default=28, help="期望 accept 行数")
    parser.add_argument("--expected-reject", type=int, default=2, help="期望 reject 行数")
    parser.add_argument("--max-accepted-trans", type=float, default=0.80, help="accepted refined 平移最大允许误差")
    parser.add_argument("--max-accepted-yaw", type=float, default=15.0, help="accepted refined yaw 最大允许误差，deg")
    parser.add_argument(
        "--expect-causal-warmup-rejects",
        action="store_true",
        help="要求因果窗口的 reject 精确等于 3 个每包首帧 warm-up + 2 个 nav43 危险帧",
    )
    args = parser.parse_args()

    rows = load_rows(args.csv_path)
    accepted = [row for row in rows if row.get("decision") == "accept"]
    rejected = [row for row in rows if row.get("decision") == "reject"]
    accepted_success = [row for row in accepted if row.get("refined_success") == "1"]
    failures: list[str] = []

    require(len(rows) == args.expected_total, f"total rows {len(rows)} != {args.expected_total}", failures)
    require(len(accepted) == args.expected_accept, f"accept rows {len(accepted)} != {args.expected_accept}", failures)
    require(len(rejected) == args.expected_reject, f"reject rows {len(rejected)} != {args.expected_reject}", failures)
    require(
        len(accepted_success) == len(accepted),
        f"accepted refined success {len(accepted_success)}/{len(accepted)} is not all success",
        failures,
    )

    for row in accepted:
        trans = as_float(row, "refined_translation_error_m")
        yaw = as_float(row, "refined_yaw_error_deg")
        require(
            trans <= args.max_accepted_trans,
            f"accepted row {row_key(row)} trans {trans:.3f} > {args.max_accepted_trans:.3f}",
            failures,
        )
        require(
            yaw <= args.max_accepted_yaw,
            f"accepted row {row_key(row)} yaw {yaw:.3f} > {args.max_accepted_yaw:.3f}",
            failures,
        )

    by_key = {row_key(row): row for row in rows}
    dangerous_rejects = [
        ("nav_drift_test43", "1782119604.896394"),
        ("nav_drift_test43", "1782119654.897294"),
    ]
    causal_warmup_rejects = [
        ("nav_drift_test43", "1782119429.893823"),
        ("nav_drift_test44", "1782178970.591691"),
        ("nav_drift_test45", "1782896041.599820"),
    ]
    for key in dangerous_rejects:
        row = by_key.get(key)
        require(row is not None, f"missing dangerous hard case {key}", failures)
        if row is not None:
            require(row.get("decision") == "reject", f"dangerous hard case {key} was not rejected", failures)

    if args.expect_causal_warmup_rejects:
        expected_reject_keys = set(dangerous_rejects + causal_warmup_rejects)
        actual_reject_keys = {row_key(row) for row in rejected}
        require(
            actual_reject_keys == expected_reject_keys,
            f"causal reject keys {sorted(actual_reject_keys)} != expected {sorted(expected_reject_keys)}",
            failures,
        )
        for key in causal_warmup_rejects:
            row = by_key.get(key)
            require(row is not None, f"missing causal warm-up reject {key}", failures)
            if row is not None:
                require(row.get("decision") == "reject", f"causal warm-up frame {key} was not rejected", failures)
                require(
                    row.get("decision_reason") == "support_below_threshold",
                    f"causal warm-up frame {key} reason should be support_below_threshold",
                    failures,
                )

    nav45_key = ("nav_drift_test45", "1782896166.601948")
    nav45 = by_key.get(nav45_key)
    require(nav45 is not None, f"missing nav45 hard case {nav45_key}", failures)
    if nav45 is not None:
        require(nav45.get("decision") == "accept", "nav45 hard case was not accepted", failures)
        require(nav45.get("refined_success") == "1", "nav45 hard case refined_success != 1", failures)
        require(
            as_float(nav45, "refined_translation_error_m") <= args.max_accepted_trans,
            "nav45 hard case refined translation exceeds threshold",
            failures,
        )
        require(
            as_float(nav45, "refined_yaw_error_deg") <= args.max_accepted_yaw,
            "nav45 hard case refined yaw exceeds threshold",
            failures,
        )

    trans_values = [as_float(row, "refined_translation_error_m") for row in accepted]
    yaw_values = [as_float(row, "refined_yaw_error_deg") for row in accepted]
    print(f"rows={len(rows)} accept={len(accepted)} reject={len(rejected)} accepted_success={len(accepted_success)}")
    if trans_values:
        print(
            "accepted_refined "
            f"median_trans={statistics.median(trans_values):.6f} "
            f"max_trans={max(trans_values):.6f} "
            f"median_yaw={statistics.median(yaw_values):.6f} "
            f"max_yaw={max(yaw_values):.6f}"
        )

    if failures:
        print("[verify_temporal_decisions] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[verify_temporal_decisions] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
