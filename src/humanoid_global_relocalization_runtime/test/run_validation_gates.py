#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_validation_gates.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 汇总第一阶段全局重定位关键验证产物，并执行一组可重复的“防退化门禁”。
  3. 门禁只检查已经有强证据的数据：真实 nav_drift registered_world 30 帧、temporal decision、
     合成 body 输入链路、bag inventory。
  4. 运行时会先只读刷新 latest bag inventory，避免目标证据矩阵长期依赖旧扫描文件。
  5. 如果当前机器仍没有真实 /cloud_registered_body bag，本脚本会明确打印该限制，但不会把合成 body
     冒充成真实 body 准确率结论。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/run_validation_gates.py
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import statistics
import sys
from collections import defaultdict
from pathlib import Path


def repo_root_from_script() -> Path:
    """根据脚本所在位置反推出工作空间根目录，避免用户必须在固定目录执行。"""
    return Path(__file__).resolve().parents[3]


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV 并返回字典行；文件缺失时抛出清晰错误，方便定位是哪一步验证没有生成产物。"""
    if not path.exists():
        raise FileNotFoundError(f"required artifact does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def as_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """把 CSV 字段安全转成浮点数；空字段用于缺真值/未定位场景，不让脚本直接崩溃。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def as_int(row: dict[str, str], key: str, default: int = 0) -> int:
    """把 CSV 字段安全转成整数，主要用于 localized/success/has_reference 这些 0/1 字段。"""
    try:
        return int(row.get(key, ""))
    except ValueError:
        return default


def median(values: list[float]) -> float:
    """只对有限浮点数求中位数；没有有效值时返回 NaN，输出层再决定如何展示。"""
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else math.nan


def require(condition: bool, message: str, failures: list[str]) -> None:
    """统一收集失败项，让一次运行能暴露所有门禁问题，而不是遇到第一个错误就停止。"""
    if not condition:
        failures.append(message)


def no_prior_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    """每个 scan 会写多个模拟先验场景；算法效果统计只取任意点启动场景，避免重复计数。"""
    return [row for row in rows if row.get("scenario_name") == "arbitrary_start_no_prior"]


def summarize_metrics(rows: list[dict[str, str]]) -> dict[str, float]:
    """汇总主 metrics CSV 的核心指标，和 README/验证报告中的统计口径保持一致。"""
    rows = no_prior_rows(rows)
    localized = sum(as_int(row, "localized") for row in rows)
    success = sum(as_int(row, "success") for row in rows)
    references = sum(as_int(row, "has_reference") for row in rows)
    trans = [as_float(row, "translation_error_m") for row in rows if as_float(row, "translation_error_m") >= 0.0]
    yaw = [as_float(row, "yaw_error_deg") for row in rows if as_float(row, "yaw_error_deg") >= 0.0]
    total = [as_float(row, "total_ms") for row in rows]
    delta_cpu = [
        as_float(row, "delta_user_cpu_ms") + as_float(row, "delta_system_cpu_ms")
        for row in rows
        if math.isfinite(as_float(row, "delta_user_cpu_ms")) and math.isfinite(as_float(row, "delta_system_cpu_ms"))
    ]
    peak_rss_values = [as_float(row, "peak_rss_mb") for row in rows if math.isfinite(as_float(row, "peak_rss_mb"))]
    return {
        "n": float(len(rows)),
        "localized": float(localized),
        "success": float(success),
        "references": float(references),
        "success_over_reference": float(success) / float(references) if references else 0.0,
        "median_trans": median(trans),
        "median_yaw": median(yaw),
        "median_total": median(total),
        "median_delta_cpu": median(delta_cpu),
        "peak_rss": max(peak_rss_values) if peak_rss_values else math.nan,
    }


def check_nav_drift_metrics(path: Path, failures: list[str]) -> None:
    """检查真实 nav_drift registered_world 30 帧主指标，确认推荐配置的基础表现没有退化。"""
    stats = summarize_metrics(read_csv(path))
    print(
        "[nav_drift_metrics] "
        f"n={stats['n']:.0f} localized={stats['localized']:.0f} success={stats['success']:.0f} "
        f"success_over_reference={stats['success_over_reference']:.3f} "
        f"median_trans={stats['median_trans']:.3f} median_yaw={stats['median_yaw']:.3f} "
        f"median_total_ms={stats['median_total']:.1f} median_delta_cpu_ms={stats['median_delta_cpu']:.1f} "
        f"peak_rss_mb={stats['peak_rss']:.1f}"
    )
    require(stats["n"] == 30.0, "nav_drift 30-frame metrics row count changed", failures)
    require(stats["localized"] == 30.0, "nav_drift should localize all 30 sampled frames", failures)
    require(stats["references"] == 30.0, "nav_drift should have 30 valid reference poses", failures)
    require(stats["success"] == 27.0, "single-frame nav_drift success should remain 27/30 before temporal decision", failures)
    require(stats["median_trans"] <= 0.10, "nav_drift median translation error regressed above 0.10m", failures)
    require(stats["median_yaw"] <= 0.50, "nav_drift median yaw error regressed above 0.50deg", failures)


def row_key(row: dict[str, str]) -> tuple[str, str]:
    """用 bag 名和时间戳定位 hard case，避免绝对路径变化导致校验失效。"""
    return Path(row.get("bag_path", "")).name, row.get("stamp_sec", "")


def check_temporal_decisions(
    path: Path,
    failures: list[str],
    *,
    label: str,
    expected_accept: int,
    expected_reject: int,
    expect_causal_warmup_rejects: bool = False,
) -> None:
    """检查多帧 temporal decision：必须救回 nav45 hard case，同时拒绝两个危险 nav43 hard case。"""
    rows = read_csv(path)
    accepted = [row for row in rows if row.get("decision") == "accept"]
    rejected = [row for row in rows if row.get("decision") == "reject"]
    accepted_success = [row for row in accepted if row.get("refined_success") == "1"]
    accepted_trans = [as_float(row, "refined_translation_error_m") for row in accepted]
    accepted_yaw = [as_float(row, "refined_yaw_error_deg") for row in accepted]
    print(
        f"[temporal_decisions:{label}] "
        f"rows={len(rows)} accept={len(accepted)} reject={len(rejected)} "
        f"accepted_success={len(accepted_success)} "
        f"median_trans={median(accepted_trans):.3f} max_trans={max(accepted_trans):.3f} "
        f"median_yaw={median(accepted_yaw):.3f} max_yaw={max(accepted_yaw):.3f}"
    )

    require(len(rows) == 30, "temporal decision row count should be 30", failures)
    require(
        len(accepted) == expected_accept,
        f"{label} temporal decision accept count should be {expected_accept}",
        failures,
    )
    require(
        len(rejected) == expected_reject,
        f"{label} temporal decision reject count should be {expected_reject}",
        failures,
    )
    require(len(accepted_success) == len(accepted), "all accepted temporal decisions should refine successfully", failures)
    require(max(accepted_trans) <= 0.80, "accepted temporal decision has translation error above 0.80m", failures)
    require(max(accepted_yaw) <= 15.0, "accepted temporal decision has yaw error above 15deg", failures)

    by_key = {row_key(row): row for row in rows}
    dangerous_nav43_rejects = [
        ("nav_drift_test43", "1782119604.896394"),
        ("nav_drift_test43", "1782119654.897294"),
    ]
    causal_warmup_rejects = [
        ("nav_drift_test43", "1782119429.893823"),
        ("nav_drift_test44", "1782178970.591691"),
        ("nav_drift_test45", "1782896041.599820"),
    ]
    for key in dangerous_nav43_rejects:
        row = by_key.get(key)
        require(row is not None, f"missing dangerous nav43 hard case {key}", failures)
        if row is not None:
            require(row.get("decision") == "reject", f"dangerous nav43 hard case {key} must be rejected", failures)

    if expect_causal_warmup_rejects:
        expected_reject_keys = set(dangerous_nav43_rejects + causal_warmup_rejects)
        actual_reject_keys = {row_key(row) for row in rejected}
        require(
            actual_reject_keys == expected_reject_keys,
            f"{label} temporal reject profile changed: {sorted(actual_reject_keys)}",
            failures,
        )
        for key in causal_warmup_rejects:
            row = by_key.get(key)
            require(row is not None, f"missing causal warm-up reject {key}", failures)
            if row is not None:
                require(
                    row.get("decision_reason") == "support_below_threshold",
                    f"causal warm-up reject {key} should be caused by support_below_threshold",
                    failures,
                )

    nav45_key = ("nav_drift_test45", "1782896166.601948")
    nav45 = by_key.get(nav45_key)
    require(nav45 is not None, f"missing nav45 hard case {nav45_key}", failures)
    if nav45 is not None:
        require(nav45.get("decision") == "accept", "nav45 hard case should be accepted by temporal support", failures)
        require(nav45.get("refined_success") == "1", "nav45 hard case should refine successfully", failures)


def check_synthetic_body(
  path: Path,
  failures: list[str],
  label: str = "synthetic_body",
  expected_frames: float = 5.0,
) -> None:
    """检查合成 body bag 自洽验证，证明 body 输入分支和轴转换链路没有工程级退化。"""
    stats = summarize_metrics(read_csv(path))
    print(
        f"[{label}] "
        f"n={stats['n']:.0f} localized={stats['localized']:.0f} success={stats['success']:.0f} "
        f"median_trans={stats['median_trans']:.3f} median_yaw={stats['median_yaw']:.3f}"
    )
    require(stats["n"] == expected_frames, f"{label} metrics row count should be {expected_frames:.0f}", failures)
    require(stats["localized"] == expected_frames, f"{label} should localize all sampled frames", failures)
    require(stats["success"] == expected_frames, f"{label} should succeed on all sampled frames", failures)
    require(stats["median_trans"] <= 0.10, f"{label} median translation error regressed above 0.10m", failures)


def yaw_diff_deg(a: float, b: float) -> float:
    """计算 yaw 差值，处理 360 度绕回，用于比较两条输入链路同帧输出是否一致。"""
    diff = math.radians(a - b)
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return abs(math.degrees(diff))


def check_input_mode_equivalence(registered_csv: Path, body_csv: Path, failures: list[str]) -> None:
    """检查 registered_world 和合成 body 在同一批 stamp 上的最终位姿是否几乎一致。"""
    registered_rows = {
        row.get("stamp_sec", ""): row
        for row in no_prior_rows(read_csv(registered_csv))
    }
    body_rows = {
        row.get("stamp_sec", ""): row
        for row in no_prior_rows(read_csv(body_csv))
    }
    matched_stamps = sorted(set(registered_rows) & set(body_rows))
    xy_diffs: list[float] = []
    yaw_diffs: list[float] = []
    error_diffs: list[float] = []
    rank_diffs: list[int] = []

    for stamp in matched_stamps:
        registered = registered_rows[stamp]
        body = body_rows[stamp]
        xy_diffs.append(
            math.hypot(
                as_float(registered, "final_x_m") - as_float(body, "final_x_m"),
                as_float(registered, "final_y_m") - as_float(body, "final_y_m"),
            )
        )
        yaw_diffs.append(
            yaw_diff_deg(
                as_float(registered, "final_yaw_deg"),
                as_float(body, "final_yaw_deg"),
            )
        )
        error_diffs.append(
            abs(as_float(registered, "translation_error_m") - as_float(body, "translation_error_m"))
        )
        try:
            rank_diffs.append(
                abs(
                    int(registered.get("refined_candidate_rank", "0")) -
                    int(body.get("refined_candidate_rank", "0"))
                )
            )
        except ValueError:
            rank_diffs.append(9999)

    max_xy = max(xy_diffs, default=math.nan)
    max_yaw = max(yaw_diffs, default=math.nan)
    max_error = max(error_diffs, default=math.nan)
    max_rank = max(rank_diffs, default=0)
    print(
        "[input_mode_equivalence] "
        f"matched={len(matched_stamps)} max_xy_diff={max_xy:.9f}m "
        f"max_yaw_diff={max_yaw:.9f}deg max_error_diff={max_error:.9f}m "
        f"max_rank_diff={max_rank}"
    )
    require(len(matched_stamps) >= 5, "registered_world/body equivalence should match at least 5 stamps", failures)
    require(max_xy <= 0.002, "registered_world/body final xy differs by more than 2mm", failures)
    require(max_yaw <= 0.01, "registered_world/body final yaw differs by more than 0.01deg", failures)
    require(max_error <= 0.002, "registered_world/body translation error differs by more than 2mm", failures)
    require(max_rank == 0, "registered_world/body refined candidate rank differs", failures)


def check_bag_inventory(path: Path, failures: list[str]) -> None:
    """检查 bag inventory，确认当前验证覆盖哪些输入模式，并明确真实 body bag 是否存在。"""
    rows = read_csv(path)
    registered_with_ref = [row for row in rows if row.get("registered_world_with_reference") == "yes"]
    body_ready = [row for row in rows if row.get("body_ready") == "yes"]
    body_with_ref = [row for row in rows if row.get("body_with_reference") == "yes"]
    print(
        "[bag_inventory] "
        f"metadata={len(rows)} registered_world_with_reference={len(registered_with_ref)} "
        f"body_ready={len(body_ready)} body_with_reference={len(body_with_ref)}"
    )
    require(len(rows) >= 3, "bag inventory should contain at least the nav_drift bags", failures)
    require(len(registered_with_ref) >= 3, "bag inventory should find registered_world+reference nav_drift bags", failures)
    if not body_ready:
        print("[bag_inventory] 当前可读 bag 中仍未发现真实 /cloud_registered_body；body 准确率需等后续补录 bag。")
    else:
        print("[bag_inventory] 已发现真实 /cloud_registered_body bag，建议追加真实 body 同场景回归。")


def check_artifact_matrix(workspace: Path, artifacts_root: Path, failures: list[str]) -> None:
    """检查源码、配置、README、验证报告等核心文件存在，防止只复制了部分产物。"""
    required = [
        workspace / "src/humanoid_global_relocalization_runtime/CMakeLists.txt",
        workspace / "src/humanoid_global_relocalization_runtime/package.xml",
        workspace / "src/humanoid_global_relocalization_runtime/README.md",
        workspace / "src/humanoid_global_relocalization_runtime/config/global_relocalization_runtime.yaml",
        workspace / "src/humanoid_global_relocalization_runtime/launch/global_relocalization_runtime.launch.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/check_documentation_contract.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/check_goal_evidence.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/check_workspace_boundaries.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/compare_input_modes.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/verify_cpp_build_artifacts.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/extra_registered_world_no_reference_smoke.yaml",
        workspace / "src/humanoid_global_relocalization_runtime/test/online_smoke_evidence.csv",
        workspace / "src/humanoid_global_relocalization_runtime/test/synthetic_body_nav_drift_all.yaml",
        workspace
        / "src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_extended_cpp_temporal_causal.yaml",
        workspace / "src/humanoid_global_relocalization_runtime/test/nav_drift_test46_registered_world.yaml",
        workspace / "src/humanoid_global_relocalization_runtime/test/extra_registered_world_hall_mapping_20260605_152637.yaml",
        workspace / "src/humanoid_global_relocalization_runtime/test/nav_drift_validation_report.md",
        workspace / "src/humanoid_global_relocalization_runtime/test/run_online_smoke_matrix.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/run_online_active_view_status_smoke.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/run_online_trajectory_recovery_smoke.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/run_nav46_resource_sweep.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/run_nav46_hardframe_recovery_validation.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/run_real_body_validation.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/nav46_rand80_reject_matches.csv",
        workspace / "src/humanoid_global_relocalization_runtime/test/simulate_recovery_consumer.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/verify_online_smoke_evidence.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/verify_online_active_view_status_smoke.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/verify_online_trajectory_recovery_smoke.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/verify_nav46_three_stage_recovery.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/verify_nav46_topk_strategy.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/verify_trajectory_long_history_bags.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/verify_refine_sweep.py",
        workspace / "src/humanoid_global_relocalization_runtime/test/verify_resource_metrics.py",
        workspace / "src/humanoid_global_relocalization_runtime/third_party/3d_bbs/LICENSE",
        artifacts_root / "bag_inventory_v2.csv",
        artifacts_root / "bag_inventory_home_scan.csv",
        artifacts_root / "bag_inventory_home_scan_latest.csv",
        artifacts_root / "bag_inventory_nav_drift_latest.csv",
        artifacts_root / "global_relocalization_nav_drift_focused_v1/global_relocalization_metrics.csv",
        artifacts_root / "global_relocalization_nav_drift_baseline_v1/global_relocalization_metrics.csv",
        artifacts_root
        / "global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_metrics.csv",
        artifacts_root
        / "global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_temporal_decisions.csv",
        artifacts_root
        / "global_relocalization_nav_drift_extended_cpp_temporal_causal_v1/global_relocalization_temporal_decisions.csv",
        artifacts_root / "global_relocalization_extra_hall_mapping_20260605_152637_v4_validref/global_relocalization_metrics.csv",
        artifacts_root / "global_relocalization_extra_no_reference_smoke_v1/global_relocalization_metrics.csv",
        artifacts_root / "global_relocalization_synthetic_body_nav_drift_all_v1/global_relocalization_metrics.csv",
        artifacts_root / "global_relocalization_synthetic_body_nav43_v1/global_relocalization_metrics.csv",
        artifacts_root / "global_relocalization_real_body_validation_v1/global_relocalization_metrics.csv",
        artifacts_root / "global_relocalization_nav46_registered_world_v1/global_relocalization_metrics.csv",
        artifacts_root / "global_relocalization_real_body_validation_30frame_v1/global_relocalization_metrics.csv",
        artifacts_root / "global_relocalization_nav46_registered_world_30frame_v1/global_relocalization_metrics.csv",
        artifacts_root / "nav46_resource_sweep_summary.csv",
        artifacts_root / "nav46_stress_validation/summary_rand80_seed20260702.csv",
        artifacts_root / "online_active_view_status_smoke.csv",
        artifacts_root / "online_active_view_status_smoke.yaml",
        artifacts_root / "online_trajectory_recovery_smoke_pose.csv",
        artifacts_root / "online_trajectory_recovery_smoke.yaml",
        artifacts_root / "recovery_consumer_simulation.csv",
        artifacts_root / "trajectory_rand80_rejects_long_history/summary.csv",
        artifacts_root / "trajectory_rand80_reject12_active_view_v2/summary.csv",
        artifacts_root / "trajectory_rand80_rejects_long_history_top30_probe/summary.csv",
        artifacts_root / "trajectory_rand80_reject12_active_view_top30_probe/summary.csv",
        artifacts_root / "trajectory_long_history_summary.csv",
        artifacts_root / "trajectory_long_history_full_bag44/summary.csv",
        artifacts_root / "trajectory_long_history_full_bag45/summary.csv",
        artifacts_root / "trajectory_long_history_full_bag46/summary.csv",
        artifacts_root / "synthetic_body_nav_drift_test43/metadata.yaml",
        artifacts_root / "synthetic_body_nav_drift_test44/metadata.yaml",
        artifacts_root / "synthetic_body_nav_drift_test45/metadata.yaml",
    ]
    for path in required:
        require(path.exists(), f"missing required file/artifact: {path}", failures)


def check_documentation_contract(workspace: Path, failures: list[str]) -> None:
    """运行中文文档/注释契约检查，确保 README、报告、源码头和主 YAML 注释没有退化。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/check_documentation_contract.py"
    result = subprocess.run(
        [sys.executable, str(script), "--workspace", str(workspace)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("documentation contract check failed")


def check_cpp_build_artifacts_contract(workspace: Path, failures: list[str]) -> None:
    """检查当前工作空间最近一次 C++ 构建后的 install/build 关键产物是否存在。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/verify_cpp_build_artifacts.py"
    result = subprocess.run(
        [sys.executable, str(script), "--workspace", str(workspace)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("C++ build artifact verification failed")


def check_workspace_boundaries_contract(workspace: Path, artifacts_root: Path, failures: list[str]) -> None:
    """运行工作空间边界检查，防止误依赖 /home/ubuntu/humanoid_ws。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/check_workspace_boundaries.py"
    result = subprocess.run(
        [
            sys.executable,
            str(script),
            "--workspace",
            str(workspace),
            "--artifacts-root",
            str(artifacts_root),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("workspace boundary check failed")


def check_goal_evidence_contract(workspace: Path, failures: list[str]) -> None:
    """运行目标证据矩阵检查，确认显式要求都有对应证据或已知外部限制说明。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/check_goal_evidence.py"
    result = subprocess.run(
        [sys.executable, str(script), "--workspace", str(workspace)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("goal evidence check failed")


def refresh_latest_bag_inventory(workspace: Path, artifacts_root: Path, failures: list[str]) -> None:
    """刷新 latest bag inventory，确保门禁基于当前机器的 bag 状态，而不是只读取旧扫描文件。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/scan_bag_inventory.py"
    artifacts_root.mkdir(parents=True, exist_ok=True)
    scans = [
        [
            sys.executable,
            str(script),
            "--root",
            "/home/ubuntu",
            "--exclude",
            "/home/ubuntu/humanoid_ws",
            "--exclude",
            str(artifacts_root),
            "--csv",
            str(artifacts_root / "bag_inventory_home_scan_latest.csv"),
            "--md",
            str(artifacts_root / "bag_inventory_home_scan_latest.md"),
        ],
        [
            sys.executable,
            str(script),
            "--root",
            "/home/ubuntu/nav_drift_test",
            "--csv",
            str(artifacts_root / "bag_inventory_nav_drift_latest.csv"),
            "--md",
            str(artifacts_root / "bag_inventory_nav_drift_latest.md"),
        ],
    ]
    for command in scans:
        result = subprocess.run(
            command,
            cwd=str(workspace),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        print(result.stdout, end="")
        if result.returncode != 0:
            if result.stderr:
                print(result.stderr, end="", file=sys.stderr)
            failures.append(f"latest bag inventory refresh failed: {' '.join(command)}")


def check_refine_sweep_contract(workspace: Path, artifacts_root: Path, failures: list[str]) -> None:
    """运行多精匹配方法/多地图 sweep 产物检查，确保推荐 GICP 的对比证据没有丢失。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/verify_refine_sweep.py"
    result = subprocess.run(
        [
            sys.executable,
            str(script),
            "--workspace",
            str(workspace),
            "--artifacts-root",
            str(artifacts_root),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("refine sweep verification failed")


def check_resource_metrics_contract(workspace: Path, artifacts_root: Path, failures: list[str]) -> None:
    """运行资源字段检查，确保耗时、CPU、内存和线程数统计没有丢失。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/verify_resource_metrics.py"
    result = subprocess.run(
        [
            sys.executable,
            str(script),
            "--workspace",
            str(workspace),
            "--artifacts-root",
            str(artifacts_root),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("resource metrics verification failed")


def check_online_smoke_evidence_contract(workspace: Path, failures: list[str]) -> None:
    """运行在线 smoke 证据检查，确认在线 debug 发布链路覆盖了预期输入模式和 bag。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/verify_online_smoke_evidence.py"
    result = subprocess.run(
        [sys.executable, str(script), "--workspace", str(workspace)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("online smoke evidence verification failed")


def check_online_trajectory_recovery_contract(workspace: Path, failures: list[str]) -> None:
    """运行在线 trajectory recovery 专项证据检查，确认第二层恢复分支真实发布过 debug 恢复量。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/verify_online_trajectory_recovery_smoke.py"
    result = subprocess.run(
        [sys.executable, str(script), "--workspace", str(workspace)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("online trajectory recovery smoke verification failed")


def check_online_active_view_status_contract(workspace: Path, failures: list[str]) -> None:
    """运行在线主动恢复状态证据检查，确认拒绝发布时会给出 active_view 提示。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/verify_online_active_view_status_smoke.py"
    result = subprocess.run(
        [sys.executable, str(script), "--workspace", str(workspace)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("online active-view status smoke verification failed")


def check_nav46_three_stage_recovery_contract(workspace: Path, artifacts_root: Path, failures: list[str]) -> None:
    """运行 nav46 随机 80 点三层恢复校验，确认短历史、长历史和主动新视角能合计救回。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/verify_nav46_three_stage_recovery.py"
    result = subprocess.run(
        [
            sys.executable,
            str(script),
            "--workspace",
            str(workspace),
            "--artifacts-root",
            str(artifacts_root),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("nav46 three-stage recovery verification failed")


def check_trajectory_long_history_bags_contract(workspace: Path, artifacts_root: Path, failures: list[str]) -> None:
    """运行 44/45/46 三包长历史恢复校验，防止优化只对 nav46 单包成立。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/verify_trajectory_long_history_bags.py"
    result = subprocess.run(
        [
            sys.executable,
            str(script),
            "--workspace",
            str(workspace),
            "--artifacts-root",
            str(artifacts_root),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("trajectory long-history bags verification failed")


def check_nav46_hardframe_recovery_contract(workspace: Path, failures: list[str]) -> None:
    """运行 hard-frame 补救脚本的轻量复核模式，确认长历史/主动视角重跑产物仍可解释为安全救回。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/run_nav46_hardframe_recovery_validation.py"
    result = subprocess.run(
        [sys.executable, str(script), "--workspace", str(workspace)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("nav46 hard-frame recovery verification failed")


def check_nav46_topk_strategy_contract(workspace: Path, artifacts_root: Path, failures: list[str]) -> None:
    """运行 top-K 分层策略校验，确认 top30 快速恢复和 top60 深搜边界仍成立。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/verify_nav46_topk_strategy.py"
    result = subprocess.run(
        [
            sys.executable,
            str(script),
            "--workspace",
            str(workspace),
            "--artifacts-root",
            str(artifacts_root),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("nav46 top-K strategy verification failed")


def check_recovery_consumer_contract(workspace: Path, failures: list[str]) -> None:
    """运行恢复消费者仿真，确认安全接收规则没有放行错误恢复。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/simulate_recovery_consumer.py"
    result = subprocess.run(
        [sys.executable, str(script), "--workspace", str(workspace)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr)
        failures.append("recovery consumer simulation failed")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run first-stage global relocalization validation gates.")
    parser.add_argument(
        "--workspace",
        type=Path,
        default=repo_root_from_script(),
        help="humanoid_ws 工作空间路径，默认根据脚本位置自动推断",
    )
    parser.add_argument(
        "--artifacts-root",
        type=Path,
        default=None,
        help="验证产物根目录，默认使用 <workspace>/.codex_tmp",
    )
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    artifacts_root = (args.artifacts_root or workspace / ".codex_tmp").resolve()
    failures: list[str] = []

    print(f"[workspace] {workspace}")
    print(f"[artifacts_root] {artifacts_root}")

    try:
        refresh_latest_bag_inventory(workspace, artifacts_root, failures)
        check_artifact_matrix(workspace, artifacts_root, failures)
        check_documentation_contract(workspace, failures)
        check_cpp_build_artifacts_contract(workspace, failures)
        check_workspace_boundaries_contract(workspace, artifacts_root, failures)
        check_goal_evidence_contract(workspace, failures)
        check_refine_sweep_contract(workspace, artifacts_root, failures)
        check_resource_metrics_contract(workspace, artifacts_root, failures)
        check_online_smoke_evidence_contract(workspace, failures)
        check_online_active_view_status_contract(workspace, failures)
        check_online_trajectory_recovery_contract(workspace, failures)
        check_nav46_three_stage_recovery_contract(workspace, artifacts_root, failures)
        check_trajectory_long_history_bags_contract(workspace, artifacts_root, failures)
        check_nav46_hardframe_recovery_contract(workspace, failures)
        check_nav46_topk_strategy_contract(workspace, artifacts_root, failures)
        check_recovery_consumer_contract(workspace, failures)
        check_bag_inventory(artifacts_root / "bag_inventory_home_scan_latest.csv", failures)
        check_nav_drift_metrics(
            artifacts_root
            / "global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_metrics.csv",
            failures,
        )
        check_temporal_decisions(
            artifacts_root
            / "global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_temporal_decisions.csv",
            failures,
            label="offline_bidirectional",
            expected_accept=28,
            expected_reject=2,
        )
        check_temporal_decisions(
            artifacts_root
            / "global_relocalization_nav_drift_extended_cpp_temporal_causal_v1/global_relocalization_temporal_decisions.csv",
            failures,
            label="causal_history_only",
            expected_accept=25,
            expected_reject=5,
            expect_causal_warmup_rejects=True,
        )
        check_synthetic_body(
            artifacts_root / "global_relocalization_synthetic_body_nav43_v1/global_relocalization_metrics.csv",
            failures,
            label="synthetic_body_nav43",
            expected_frames=5.0,
        )
        check_synthetic_body(
            artifacts_root / "global_relocalization_synthetic_body_nav_drift_all_v1/global_relocalization_metrics.csv",
            failures,
            label="synthetic_body_nav43_44_45",
            expected_frames=15.0,
        )
        check_input_mode_equivalence(
            artifacts_root
            / "global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_metrics.csv",
            artifacts_root / "global_relocalization_synthetic_body_nav43_v1/global_relocalization_metrics.csv",
            failures,
        )
    except FileNotFoundError as exc:
        failures.append(str(exc))

    if failures:
        print("[run_validation_gates] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[run_validation_gates] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
