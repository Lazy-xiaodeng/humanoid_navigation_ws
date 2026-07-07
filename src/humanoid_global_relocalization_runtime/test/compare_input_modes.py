#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_input_modes.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 读取两份 global_relocalization_metrics.csv，按同一 stamp 对齐 registered_world 与 body 结果。
  3. 检查两条输入链路最终位姿、误差、候选 rank 是否一致，用来验证话题转换和坐标轴处理没有引入偏差。
  4. 当前主要用于“从 /fast_lio/cloud_registered + /odom 派生出的合成 /cloud_registered_body bag”自洽验证；
     它不能替代真实 Fast-LIO /cloud_registered_body bag 的准确率实测。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/compare_input_modes.py \
    --registered-csv .codex_tmp/global_relocalization_nav_drift_balanced_iter25_v5_validref/global_relocalization_metrics.csv \
    --body-csv .codex_tmp/global_relocalization_synthetic_body_nav43_v1/global_relocalization_metrics.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path


def read_no_prior_rows(path: Path) -> dict[str, dict[str, str]]:
    """读取任意点启动场景行，并按 stamp 建索引；其它模拟 jump 场景会重复同一搜索结果，因此跳过。"""
    if not path.exists():
        raise FileNotFoundError(f"metrics csv does not exist: {path}")
    rows: dict[str, dict[str, str]] = {}
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("scenario_name") != "arbitrary_start_no_prior":
                continue
            rows[row.get("stamp_sec", "")] = row
    return rows


def as_float(row: dict[str, str], key: str) -> float:
    """把 CSV 数值字段转换成浮点数；缺字段会变成 NaN 并在阈值检查中失败。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return math.nan


def yaw_diff_deg(a: float, b: float) -> float:
    """计算角度差，单位 deg，并处理 ±180/360 跳变。"""
    diff = math.radians(a - b)
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return abs(math.degrees(diff))


def finite_max(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return max(finite) if finite else math.nan


def finite_median(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else math.nan


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare registered_world and body metrics on matched stamps.")
    parser.add_argument("--registered-csv", type=Path, required=True, help="registered_world metrics CSV")
    parser.add_argument("--body-csv", type=Path, required=True, help="body metrics CSV")
    parser.add_argument("--min-matched", type=int, default=5, help="最少要求对齐到多少帧")
    parser.add_argument("--max-xy-diff", type=float, default=0.002, help="同帧最终 xy 位姿最大允许差，m")
    parser.add_argument("--max-yaw-diff", type=float, default=0.01, help="同帧最终 yaw 最大允许差，deg")
    parser.add_argument("--max-error-diff", type=float, default=0.002, help="同帧平移误差最大允许差，m")
    args = parser.parse_args()

    registered = read_no_prior_rows(args.registered_csv)
    body = read_no_prior_rows(args.body_csv)
    matched_stamps = sorted(set(registered) & set(body))

    failures: list[str] = []
    if len(matched_stamps) < args.min_matched:
        failures.append(f"matched stamps {len(matched_stamps)} < required {args.min_matched}")

    xy_diffs: list[float] = []
    yaw_diffs: list[float] = []
    error_diffs: list[float] = []
    rank_diffs: list[int] = []

    for stamp in matched_stamps:
        reg = registered[stamp]
        bod = body[stamp]
        dx = as_float(reg, "final_x_m") - as_float(bod, "final_x_m")
        dy = as_float(reg, "final_y_m") - as_float(bod, "final_y_m")
        xy = math.hypot(dx, dy)
        yaw = yaw_diff_deg(as_float(reg, "final_yaw_deg"), as_float(bod, "final_yaw_deg"))
        err = abs(as_float(reg, "translation_error_m") - as_float(bod, "translation_error_m"))
        try:
            rank_diff = abs(int(reg.get("refined_candidate_rank", "0")) - int(bod.get("refined_candidate_rank", "0")))
        except ValueError:
            rank_diff = 9999

        xy_diffs.append(xy)
        yaw_diffs.append(yaw)
        error_diffs.append(err)
        rank_diffs.append(rank_diff)

        if xy > args.max_xy_diff:
            failures.append(f"stamp {stamp} xy diff {xy:.6f}m > {args.max_xy_diff:.6f}m")
        if yaw > args.max_yaw_diff:
            failures.append(f"stamp {stamp} yaw diff {yaw:.6f}deg > {args.max_yaw_diff:.6f}deg")
        if err > args.max_error_diff:
            failures.append(f"stamp {stamp} translation error diff {err:.6f}m > {args.max_error_diff:.6f}m")
        if rank_diff != 0:
            failures.append(f"stamp {stamp} refined candidate rank differs: {rank_diff}")

    print(
        "matched="
        f"{len(matched_stamps)} "
        f"max_xy_diff={finite_max(xy_diffs):.9f} "
        f"median_xy_diff={finite_median(xy_diffs):.9f} "
        f"max_yaw_diff={finite_max(yaw_diffs):.9f} "
        f"median_yaw_diff={finite_median(yaw_diffs):.9f} "
        f"max_error_diff={finite_max(error_diffs):.9f} "
        f"max_rank_diff={max(rank_diffs) if rank_diffs else 0}"
    )

    if failures:
        print("[compare_input_modes] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[compare_input_modes] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
