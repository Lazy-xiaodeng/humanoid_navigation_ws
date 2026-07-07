#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
summarize_trajectory_long_history_bags.py

文件作用：
  1. 汇总多个 bag 的长历史轨迹 likelihood 验证产物。
  2. 读取 run_trajectory_likelihood_validation.py 生成的 summary.csv 和 metrics CSV。
  3. 输出每个 bag 的单帧成功率、轨迹成功率、accept/false accept、资源消耗和 margin sweep。
  4. 该脚本只用于离线验证报告，不参与功能节点运行。
"""

from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV 文件；文件不存在时返回空列表，方便批量汇总时跳过缺失产物。"""
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def to_float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    """安全读取浮点字段。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def median(values: list[float]) -> float:
    """空列表安全 median。"""
    return statistics.median(values) if values else float("nan")


def percentile(values: list[float], ratio: float) -> float:
    """简单分位数，用于资源统计。"""
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(len(ordered) * ratio) - 1))
    return ordered[index]


def summarize_one(label: str, root: Path, mode: str) -> dict[str, str]:
    """汇总单个 bag 输出目录。"""
    summary_rows = read_csv(root / "summary.csv")
    metrics_rows = [
        row for row in read_csv(root / mode / "global_relocalization_metrics.csv")
        if row.get("scenario_name") == "arbitrary_start_no_prior"
    ]
    accepted = [row for row in summary_rows if row.get("trajectory_decision") == "accept"]
    accepted_success = [row for row in accepted if row.get("trajectory_success") == "1"]
    rejected_success = [
        row for row in summary_rows
        if row.get("trajectory_decision") != "accept" and row.get("trajectory_success") == "1"
    ]
    trajectory_errors = [
        to_float(row, "trajectory_error_m")
        for row in summary_rows
        if row.get("trajectory_success") == "1"
    ]
    margins = [to_float(row, "margin") for row in summary_rows]
    cpu_ms = [
        to_float(row, "delta_user_cpu_ms") + to_float(row, "delta_system_cpu_ms")
        for row in metrics_rows
    ]
    total_ms = [to_float(row, "total_ms") for row in metrics_rows]
    rss_mb = [to_float(row, "peak_rss_mb") for row in metrics_rows]
    return {
        "label": label,
        "targets": str(len(summary_rows)),
        "single_success": str(sum(row.get("single_success") == "1" for row in summary_rows)),
        "trajectory_success": str(sum(row.get("trajectory_success") == "1" for row in summary_rows)),
        "trajectory_accept": str(len(accepted)),
        "trajectory_accept_success": str(len(accepted_success)),
        "trajectory_false_accept": str(len(accepted) - len(accepted_success)),
        "trajectory_reject_success": str(len(rejected_success)),
        "median_trajectory_error_m": f"{median(trajectory_errors):.6f}",
        "max_trajectory_error_m": f"{max(trajectory_errors) if trajectory_errors else float('nan'):.6f}",
        "median_margin": f"{median(margins):.6f}",
        "min_margin": f"{min(margins) if margins else float('nan'):.6f}",
        "median_total_ms": f"{median(total_ms):.3f}",
        "p90_total_ms": f"{percentile(total_ms, 0.90):.3f}",
        "max_total_ms": f"{max(total_ms) if total_ms else float('nan'):.3f}",
        "median_cpu_ms": f"{median(cpu_ms):.3f}",
        "p90_cpu_ms": f"{percentile(cpu_ms, 0.90):.3f}",
        "max_cpu_ms": f"{max(cpu_ms) if cpu_ms else float('nan'):.3f}",
        "max_peak_rss_mb": f"{max(rss_mb) if rss_mb else float('nan'):.3f}",
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """写 CSV 汇总。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def print_margin_sweep(rows_by_label: dict[str, list[dict[str, str]]], thresholds: list[float]) -> None:
    """按阈值打印接受统计，便于选择 min_margin。"""
    all_rows: list[dict[str, str]] = []
    for rows in rows_by_label.values():
        all_rows.extend(rows)
    print("[trajectory_long_history_summary] margin,accept,accept_success,false_accept,reject_success")
    for threshold in thresholds:
        accepted = [
            row for row in all_rows
            if to_float(row, "margin") >= threshold and
            to_float(row, "average_overlap") >= 0.18 and
            int(row.get("support_frames", "0") or "0") >= 2
        ]
        accepted_success = [row for row in accepted if row.get("trajectory_success") == "1"]
        rejected_success = [
            row for row in all_rows
            if row not in accepted and row.get("trajectory_success") == "1"
        ]
        print(
            "[trajectory_long_history_summary] "
            f"{threshold:.4f},{len(accepted)},{len(accepted_success)},"
            f"{len(accepted) - len(accepted_success)},{len(rejected_success)}"
        )


def print_average_overlap_sweep(rows_by_label: dict[str, list[dict[str, str]]], thresholds: list[float]) -> None:
    """按平均 overlap 阈值打印接受统计，检查新增整体贴合门控对召回的影响。"""
    all_rows: list[dict[str, str]] = []
    for rows in rows_by_label.values():
        all_rows.extend(rows)
    print("[trajectory_long_history_summary] avg_overlap,accept,accept_success,false_accept,reject_success")
    for threshold in thresholds:
        accepted = [
            row for row in all_rows
            if to_float(row, "margin") >= 0.0015 and
            to_float(row, "average_overlap") >= threshold and
            int(row.get("support_frames", "0") or "0") >= 2
        ]
        accepted_success = [row for row in accepted if row.get("trajectory_success") == "1"]
        rejected_success = [
            row for row in all_rows
            if row not in accepted and row.get("trajectory_success") == "1"
        ]
        print(
            "[trajectory_long_history_summary] "
            f"{threshold:.3f},{len(accepted)},{len(accepted_success)},"
            f"{len(accepted) - len(accepted_success)},{len(rejected_success)}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize long-history trajectory validation outputs.")
    parser.add_argument(
        "--run",
        action="append",
        nargs=2,
        metavar=("LABEL", "ROOT"),
        default=[],
        help="一个验证产物目录，例如 --run bag44 .codex_tmp/trajectory_long_history_full_bag44",
    )
    parser.add_argument("--mode", default="registered_world", help="metrics 子目录名")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(".codex_tmp/trajectory_long_history_summary.csv"),
        help="输出汇总 CSV",
    )
    args = parser.parse_args()

    default_runs = args.run or [
        ["bag44", ".codex_tmp/trajectory_long_history_full_bag44"],
        ["bag45", ".codex_tmp/trajectory_long_history_full_bag45"],
        ["bag46", ".codex_tmp/trajectory_long_history_full_bag46"],
    ]
    rows: list[dict[str, str]] = []
    rows_by_label: dict[str, list[dict[str, str]]] = {}
    for label, root_text in default_runs:
        root = Path(root_text)
        summary_rows = read_csv(root / "summary.csv")
        if not summary_rows:
            print(f"[trajectory_long_history_summary] skip missing run {label}: {root}")
            continue
        rows.append(summarize_one(label, root, args.mode))
        rows_by_label[label] = summary_rows

    write_csv(args.output, rows)
    for row in rows:
        print(
            "[trajectory_long_history_summary] "
            f"{row['label']} targets={row['targets']} single={row['single_success']} "
            f"trajectory={row['trajectory_success']} accept={row['trajectory_accept']} "
            f"false_accept={row['trajectory_false_accept']} "
            f"median_total_ms={row['median_total_ms']} median_cpu_ms={row['median_cpu_ms']} "
            f"max_rss={row['max_peak_rss_mb']}"
        )
    print_margin_sweep(rows_by_label, [0.0010, 0.0015, 0.0020, 0.0030, 0.0050, 0.0080, 0.0100])
    print_average_overlap_sweep(rows_by_label, [0.900, 0.930, 0.950, 0.970, 0.980])
    print(f"[trajectory_long_history_summary] wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
