#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simulate_recovery_consumer.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 模拟后续导航状态机或 bridge 在真正接收全局重定位恢复量前会做的安全验收。
  3. 读取 temporal decision CSV 和 online smoke evidence，检查哪些帧会被“模拟接收”、哪些会被拒绝。
  4. 在有 /robot_realpose 真值的 bag 上统计 false accept，确保错误恢复不会被安全规则放行。
  5. 对 need_active_view_* 在线状态做反向校验：它只能触发主动采集新视角，不能被当作可注入恢复量。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/simulate_recovery_consumer.py
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class DecisionDataset:
    """一个 temporal decision 验证数据集。"""

    label: str
    path: Path
    expected_rows: int
    expected_accept: int
    expected_reject: int
    expected_accepted_ref_success: str


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV 文件；缺失时抛出清晰错误。"""
    if not path.exists():
        raise FileNotFoundError(f"missing csv: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def as_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """安全读取浮点字段。"""
    try:
        value = float(row.get(key, ""))
    except ValueError:
        return default
    return value if math.isfinite(value) else default


def as_int(row: dict[str, str], key: str, default: int = 0) -> int:
    """安全读取整数字段。"""
    try:
        return int(float(row.get(key, "")))
    except ValueError:
        return default


def is_consumer_accept(row: dict[str, str], min_support: int, max_fitness: float) -> bool:
    """模拟状态机接收规则；只使用线上可获得字段，不使用真值。"""
    return (
        row.get("decision") == "accept"
        and as_int(row, "selected_support_frames") >= min_support
        and row.get("refined_converged") == "1"
        and as_float(row, "refined_fitness", math.inf) <= max_fitness
    )


def summarize_decision_dataset(
    dataset: DecisionDataset,
    min_support: int,
    max_fitness: float,
) -> tuple[dict[str, str], list[str]]:
    """汇总单个 temporal decision 数据集，并返回失败项。"""
    rows = [
        row for row in read_csv(dataset.path)
        if row.get("scenario_name") == "arbitrary_start_no_prior"
    ]
    accepted = [row for row in rows if row.get("decision") == "accept"]
    rejected = [row for row in rows if row.get("decision") == "reject"]
    reference_accepts = [row for row in accepted if row.get("has_reference") == "1"]
    accepted_ref_success = sum(row.get("refined_success") == "1" for row in reference_accepts)
    consumer_accepts = [row for row in rows if is_consumer_accept(row, min_support, max_fitness)]
    consumer_reference_accepts = [row for row in consumer_accepts if row.get("has_reference") == "1"]
    false_accepts = [
        row for row in consumer_reference_accepts
        if row.get("refined_success") != "1"
    ]
    consumer_rejects = len(rows) - len(consumer_accepts)

    failures: list[str] = []
    if len(rows) != dataset.expected_rows:
        failures.append(f"{dataset.label}: rows {len(rows)} != {dataset.expected_rows}")
    if len(accepted) != dataset.expected_accept:
        failures.append(f"{dataset.label}: accept {len(accepted)} != {dataset.expected_accept}")
    if len(rejected) != dataset.expected_reject:
        failures.append(f"{dataset.label}: reject {len(rejected)} != {dataset.expected_reject}")
    actual_success = f"{accepted_ref_success}/{len(reference_accepts)}"
    if actual_success != dataset.expected_accepted_ref_success:
        failures.append(
            f"{dataset.label}: accepted_ref_success {actual_success} != {dataset.expected_accepted_ref_success}"
        )
    if false_accepts:
        stamps = [row.get("stamp_sec", "") for row in false_accepts]
        failures.append(f"{dataset.label}: false_accept stamps={stamps}")

    summary = {
        "label": dataset.label,
        "kind": "temporal_decisions",
        "rows": str(len(rows)),
        "accept": str(len(accepted)),
        "reject": str(len(rejected)),
        "accepted_ref_success": actual_success,
        "consumer_accept": str(len(consumer_accepts)),
        "consumer_reject": str(consumer_rejects),
        "consumer_false_accept": str(len(false_accepts)),
        "min_support": str(min_support),
        "max_fitness": f"{max_fitness:.3f}",
        "evidence": str(dataset.path),
    }
    return summary, failures


def status_tokens(status: str) -> dict[str, str]:
    """解析 recovery_status 中的 key=value 字段。"""
    parsed: dict[str, str] = {}
    for token in status.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parsed[key] = value
    return parsed


def summarize_online_evidence(path: Path, min_support: int, max_fitness: float) -> tuple[dict[str, str], list[str]]:
    """检查在线 smoke 是否发布恢复接口，并按消费者规则筛选可接收的 verified。"""
    rows = read_csv(path)
    failures: list[str] = []
    verified = 0
    consumer_accepts = 0
    for row in rows:
        key = (row.get("input_mode", ""), row.get("bag_label", ""))
        finite_map_odom = all(
            math.isfinite(as_float(row, field))
            for field in ["map_odom_x_m", "map_odom_y_m", "map_odom_yaw_deg"]
        )
        tokens = status_tokens(row.get("status", ""))
        status_ok = tokens.get("state") == "verified"
        if row.get("result") == "PASS" and finite_map_odom and status_ok:
            verified += 1
            support = as_int(tokens, "selected_support")
            refined_ok = tokens.get("refined_converged") == "true"
            fitness = as_float(tokens, "refined_fitness", math.inf)
            if support >= min_support and refined_ok and fitness <= max_fitness:
                consumer_accepts += 1
        else:
            failures.append(f"online evidence invalid for {key}")

    summary = {
        "label": "online_smoke_recovery_interface",
        "kind": "online_smoke",
        "rows": str(len(rows)),
        "accept": str(verified),
        "reject": "0",
        "accepted_ref_success": "n/a",
        "consumer_accept": str(consumer_accepts),
        "consumer_reject": str(len(rows) - consumer_accepts),
        "consumer_false_accept": "n/a",
        "min_support": str(min_support),
        "max_fitness": f"{max_fitness:.3f}",
        "evidence": str(path),
    }
    return summary, failures


def summarize_active_view_status(path: Path) -> tuple[dict[str, str], list[str]]:
    """检查 need_active_view 状态会被模拟消费者拒绝，而不是当作恢复结果接收。"""
    rows = read_csv(path)
    failures: list[str] = []
    active_view_rows = [
        row for row in rows
        if row.get("state", "").startswith("need_active_view_")
        and row.get("recovery_hint") == "active_view"
        and row.get("trajectory_attempted") == "true"
    ]
    if len(rows) != 1:
        failures.append(f"online active-view status rows {len(rows)} != 1")
    if len(active_view_rows) != 1:
        failures.append("online active-view status did not contain exactly one need_active_view row")

    summary = {
        "label": "online_active_view_status_interface",
        "kind": "online_status",
        "rows": str(len(rows)),
        "accept": "0",
        "reject": str(len(active_view_rows)),
        "accepted_ref_success": "n/a",
        "consumer_accept": "0",
        "consumer_reject": str(len(active_view_rows)),
        "consumer_false_accept": "n/a",
        "min_support": "n/a",
        "max_fitness": "n/a",
        "evidence": str(path),
    }
    return summary, failures


def write_summary(path: Path, rows: list[dict[str, str]]) -> None:
    """写出恢复消费者仿真汇总 CSV。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "label",
        "kind",
        "rows",
        "accept",
        "reject",
        "accepted_ref_success",
        "consumer_accept",
        "consumer_reject",
        "consumer_false_accept",
        "min_support",
        "max_fitness",
        "evidence",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def build_datasets(workspace: Path) -> list[DecisionDataset]:
    """定义当前必须通过恢复消费者仿真的验证数据集。"""
    artifacts = workspace / ".codex_tmp"
    return [
        DecisionDataset(
            label="nav46_real_body_30frame",
            path=artifacts / "global_relocalization_real_body_validation_30frame_v1/global_relocalization_temporal_decisions.csv",
            expected_rows=30,
            expected_accept=27,
            expected_reject=3,
            expected_accepted_ref_success="27/27",
        ),
        DecisionDataset(
            label="nav46_registered_world_30frame",
            path=artifacts / "global_relocalization_nav46_registered_world_30frame_v1/global_relocalization_temporal_decisions.csv",
            expected_rows=30,
            expected_accept=27,
            expected_reject=3,
            expected_accepted_ref_success="27/27",
        ),
        DecisionDataset(
            label="nav_drift_registered_world_causal",
            path=artifacts / "global_relocalization_nav_drift_extended_cpp_temporal_causal_v1/global_relocalization_temporal_decisions.csv",
            expected_rows=30,
            expected_accept=25,
            expected_reject=5,
            expected_accepted_ref_success="25/25",
        ),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description="Simulate recovery consumer safety checks.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--min-support", type=int, default=2, help="模拟状态机要求的最小支持帧数")
    parser.add_argument("--max-fitness", type=float, default=0.12, help="模拟状态机允许的最大 refine fitness")
    parser.add_argument("--output", type=Path, default=None, help="输出 CSV，默认 .codex_tmp/recovery_consumer_simulation.csv")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    output = args.output or workspace / ".codex_tmp/recovery_consumer_simulation.csv"
    summaries: list[dict[str, str]] = []
    failures: list[str] = []

    try:
        for dataset in build_datasets(workspace):
            summary, dataset_failures = summarize_decision_dataset(dataset, args.min_support, args.max_fitness)
            summaries.append(summary)
            failures.extend(dataset_failures)
        online_summary, online_failures = summarize_online_evidence(
            workspace / "src/humanoid_global_relocalization_runtime/test/online_smoke_evidence.csv",
            args.min_support,
            args.max_fitness,
        )
        summaries.append(online_summary)
        failures.extend(online_failures)
        active_view_summary, active_view_failures = summarize_active_view_status(
            workspace / ".codex_tmp/online_active_view_status_smoke.csv"
        )
        summaries.append(active_view_summary)
        failures.extend(active_view_failures)
    except FileNotFoundError as exc:
        failures.append(str(exc))

    write_summary(output, summaries)
    print(f"[recovery_consumer_simulation] wrote {output}")
    for row in summaries:
        print(
            "[recovery_consumer_simulation] "
            f"{row['label']} accept={row['consumer_accept']} reject={row['consumer_reject']} "
            f"false_accept={row['consumer_false_accept']} evidence={row['evidence']}"
        )

    if failures:
        print("[recovery_consumer_simulation] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1
    print("[recovery_consumer_simulation] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
