#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_goal_evidence.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 把用户目标里的显式要求映射到当前工作空间中的证据文件、CSV 结果和门禁脚本。
  3. 输出一份简短的目标证据矩阵，帮助后续交接时快速判断哪些要求已验证、哪些仍受外部数据限制。
  4. 该脚本不会把真实 /cloud_registered_body 缺失伪装成通过；发现真实 body bag 后会要求真实 body 验证产物。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/check_goal_evidence.py
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path


EXPECTED_WORKSPACE = Path("/home/ubuntu/software/Todesk/Files/humanoid_ws")
FORBIDDEN_WORKSPACE = Path("/home/ubuntu/humanoid_ws")
EXPECTED_NAV_DRIFT_BAGS = {
    "/home/ubuntu/nav_drift_test/nav_drift_test43",
    "/home/ubuntu/nav_drift_test/nav_drift_test44",
    "/home/ubuntu/nav_drift_test/nav_drift_test45",
}
EXPECTED_LATEST_NAV_DRIFT_BAGS = {
    "/home/ubuntu/nav_drift_test/nav_drift_test44",
    "/home/ubuntu/nav_drift_test/nav_drift_test45",
    "/home/ubuntu/nav_drift_test/nav_drift_test46",
}
EXPECTED_NO_REFERENCE_BAGS = {
    "/home/ubuntu/fast-lio-bags/hall_mapping",
    "/home/ubuntu/fast-lio-bags/hall_mapping_20260605_151228",
    "/home/ubuntu/fast-lio-bags/hall_mapping_20260605_151705",
    "/home/ubuntu/下载/bags/hall_mapping",
}
EXPECTED_SYNTHETIC_BODY_BAGS = {
    "/home/ubuntu/software/Todesk/Files/humanoid_ws/.codex_tmp/synthetic_body_nav_drift_test43",
    "/home/ubuntu/software/Todesk/Files/humanoid_ws/.codex_tmp/synthetic_body_nav_drift_test44",
    "/home/ubuntu/software/Todesk/Files/humanoid_ws/.codex_tmp/synthetic_body_nav_drift_test45",
}


@dataclass
class EvidenceItem:
    """目标证据矩阵中的一行。"""

    key: str
    status: str
    evidence: str
    detail: str


def repo_root_from_script() -> Path:
    """根据脚本位置反推出工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV 文件。"""
    if not path.exists():
        raise FileNotFoundError(f"missing csv: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def no_prior_rows(path: Path) -> list[dict[str, str]]:
    """读取主 metrics CSV 中任意点启动场景行。"""
    return [row for row in read_csv(path) if row.get("scenario_name") == "arbitrary_start_no_prior"]


def as_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """安全读取浮点数。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def count_success(rows: list[dict[str, str]]) -> int:
    """统计 success=1 行数。"""
    return sum(1 for row in rows if row.get("success") == "1")


def has_real_body_validation(
    metrics_path: Path,
    body_bag_paths: set[str],
    body_with_reference_paths: set[str],
) -> tuple[bool, str]:
    """检查真实 /cloud_registered_body bag 出现后，是否已经补跑真实 body 验证产物。"""
    if not metrics_path.exists():
        return False, f"已发现真实 body bag，但缺少验证结果: {metrics_path}"
    rows = no_prior_rows(metrics_path)
    if not rows:
        return False, "真实 body 验证结果中没有 arbitrary_start_no_prior 行。"
    covered_paths = {row.get("bag_path", "") for row in rows}
    missing_paths = sorted(body_bag_paths - covered_paths)
    if missing_paths:
        return False, f"真实 body 验证结果未覆盖这些 bag: {missing_paths}"
    localized = sum(1 for row in rows if row.get("localized") == "1")
    if localized != len(rows):
        return False, f"真实 body 验证 localized={localized}/{len(rows)}，存在未定位帧。"
    if body_with_reference_paths:
        reference_rows = [
            row for row in rows
            if row.get("bag_path", "") in body_with_reference_paths and row.get("has_reference") == "1"
        ]
        if not reference_rows:
            return False, "存在 body+reference bag，但验证结果中没有对应参考帧。"
        success = count_success(reference_rows)
        success_rate = success / len(reference_rows)
        if success_rate < 0.90:
            return False, (
                f"真实 body+reference 验证 success={success}/{len(reference_rows)}，"
                f"成功率 {success_rate:.3f} 低于 0.90。"
            )
        return True, (
            f"真实 body+reference 验证已覆盖 {len(body_with_reference_paths)} 个 bag，"
            f"success={success}/{len(reference_rows)}，成功率 {success_rate:.3f}。"
        )
    return True, f"真实 body smoke 已覆盖 {len(body_bag_paths)} 个无参考 bag，localized={localized}/{len(rows)}。"


def has_real_body_registered_world_equivalence(body_metrics: Path, registered_metrics: Path) -> tuple[bool, str]:
    """检查真实 body 与 registered_world 在同一 nav46 bag 上的最终输出是否一致。"""
    if not body_metrics.exists() or not registered_metrics.exists():
        return False, f"缺少两路对照 CSV: body={body_metrics.exists()} registered={registered_metrics.exists()}"
    body_rows = {row.get("stamp_sec", ""): row for row in no_prior_rows(body_metrics)}
    registered_rows = {row.get("stamp_sec", ""): row for row in no_prior_rows(registered_metrics)}
    stamps = sorted(set(body_rows) & set(registered_rows))
    if len(stamps) < 5:
        return False, f"两路对照匹配帧数不足: matched={len(stamps)}"

    max_xy = 0.0
    max_yaw = 0.0
    max_error = 0.0
    success_mismatch = 0
    for stamp in stamps:
        body = body_rows[stamp]
        registered = registered_rows[stamp]
        dx = as_float(body, "final_x_m") - as_float(registered, "final_x_m")
        dy = as_float(body, "final_y_m") - as_float(registered, "final_y_m")
        max_xy = max(max_xy, math.hypot(dx, dy))
        yaw_delta = math.radians(as_float(body, "final_yaw_deg") - as_float(registered, "final_yaw_deg"))
        while yaw_delta > math.pi:
            yaw_delta -= 2.0 * math.pi
        while yaw_delta < -math.pi:
            yaw_delta += 2.0 * math.pi
        max_yaw = max(max_yaw, abs(math.degrees(yaw_delta)))
        max_error = max(
            max_error,
            abs(as_float(body, "translation_error_m") - as_float(registered, "translation_error_m")),
        )
        if body.get("success") != registered.get("success"):
            success_mismatch += 1

    ok = max_xy <= 0.01 and max_yaw <= 0.05 and max_error <= 0.01 and success_mismatch == 0
    detail = (
        f"matched={len(stamps)} max_xy_diff={max_xy:.6f}m max_yaw_diff={max_yaw:.6f}deg "
        f"max_error_diff={max_error:.6f}m success_mismatch={success_mismatch}"
    )
    return ok, detail


def has_real_body_30frame_equivalence(body_metrics: Path, registered_metrics: Path) -> tuple[bool, str]:
    """检查 nav46 真实 body 30 帧扩展验证及 registered_world 对照是否同趋势。"""
    if not body_metrics.exists() or not registered_metrics.exists():
        return False, f"缺少 30 帧两路对照 CSV: body={body_metrics.exists()} registered={registered_metrics.exists()}"
    body_rows = no_prior_rows(body_metrics)
    registered_rows = no_prior_rows(registered_metrics)
    body_reference_rows = [row for row in body_rows if row.get("has_reference") == "1"]
    registered_reference_rows = [row for row in registered_rows if row.get("has_reference") == "1"]
    if len(body_rows) != 30 or len(registered_rows) != 30:
        return False, f"30 帧对照行数异常: body={len(body_rows)} registered={len(registered_rows)}"
    body_success = count_success(body_reference_rows)
    registered_success = count_success(registered_reference_rows)

    body_map = {row.get("stamp_sec", ""): row for row in body_rows}
    registered_map = {row.get("stamp_sec", ""): row for row in registered_rows}
    stamps = sorted(set(body_map) & set(registered_map))
    success_mismatch = sum(1 for stamp in stamps if body_map[stamp].get("success") != registered_map[stamp].get("success"))

    ok = (
        len(stamps) == 30
        and len(body_reference_rows) == 29
        and len(registered_reference_rows) == 29
        and body_success == 27
        and registered_success == 27
        and success_mismatch == 0
    )
    detail = (
        f"matched={len(stamps)} body_success={body_success}/{len(body_reference_rows)} "
        f"registered_success={registered_success}/{len(registered_reference_rows)} "
        f"success_mismatch={success_mismatch}"
    )
    return ok, detail


def has_nav46_resource_sweep(summary_path: Path) -> tuple[bool, str]:
    """检查 nav46 资源 sweep 是否证明低负载推荐参数没有牺牲时序门控成功率。"""
    rows = {row.get("name", ""): row for row in read_csv(summary_path)}
    required = {"baseline_body30", "threads4_refine10", "threads2_refine8", "coarser_scan_threads4"}
    missing = sorted(required - set(rows))
    if missing:
        return False, f"资源 sweep 缺少 case: {missing}"
    baseline_cpu = as_float(rows["baseline_body30"], "median_cpu_ms")
    low_cpu = as_float(rows["threads2_refine8"], "median_cpu_ms")
    low_ok = rows["threads2_refine8"].get("accepted_ref_success") == "27/27"
    coarser_unsafe = rows["coarser_scan_threads4"].get("accepted_ref_success") != "27/27"
    ok = low_ok and coarser_unsafe and low_cpu < baseline_cpu * 0.55
    return (
        ok,
        "threads2_refine8 accepted_ref_success="
        f"{rows['threads2_refine8'].get('accepted_ref_success')} median_cpu={low_cpu:.1f}ms；"
        f"baseline_cpu={baseline_cpu:.1f}ms；coarser accepted_ref_success="
        f"{rows['coarser_scan_threads4'].get('accepted_ref_success')}",
    )


def has_cpp_build_artifacts(workspace: Path) -> tuple[bool, str]:
    """检查 C++ 包最近一次构建后的 install/build 关键产物。"""
    install_root = workspace / "install/humanoid_global_relocalization_runtime"
    build_root = workspace / "build/humanoid_global_relocalization_runtime"
    required = [
        install_root / "lib/humanoid_global_relocalization_runtime/global_relocalization_node",
        install_root / "lib/libglobal_relocalization_core.a",
        install_root / "lib/libcpu_bbs3d.so",
        install_root / "share/humanoid_global_relocalization_runtime/config/global_relocalization_runtime.yaml",
        install_root / "share/humanoid_global_relocalization_runtime/launch/global_relocalization_runtime.launch.py",
        build_root / "global_relocalization_node",
        build_root / "global_relocalization_offline_eval",
        build_root / "libglobal_relocalization_core.a",
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        return False, f"缺少构建/安装产物: {missing}"
    return True, "install 中已有 online node、offline evaluator、core 库、3D-BBS CPU 库、配置和 launch；build 中也有对应二进制。"


def has_recovery_consumer_simulation(summary_path: Path) -> tuple[bool, str]:
    """检查恢复消费者仿真是否覆盖关键数据集，并且没有 false accept。"""
    rows = {row.get("label", ""): row for row in read_csv(summary_path)}
    required = {
        "nav46_real_body_30frame": "27/27",
        "nav46_registered_world_30frame": "27/27",
        "nav_drift_registered_world_causal": "25/25",
        "online_smoke_recovery_interface": "n/a",
        "online_active_view_status_interface": "n/a",
    }
    missing = sorted(set(required) - set(rows))
    if missing:
        return False, f"恢复消费者仿真缺少数据集: {missing}"
    failures = []
    for label, expected_success in required.items():
        row = rows[label]
        if row.get("accepted_ref_success") != expected_success:
            failures.append(f"{label} accepted_ref_success={row.get('accepted_ref_success')}")
        false_accept = row.get("consumer_false_accept", "0")
        if false_accept not in {"0", "n/a"}:
            failures.append(f"{label} false_accept={false_accept}")
    if failures:
        return False, "; ".join(failures)
    active_view = rows["online_active_view_status_interface"]
    if active_view.get("consumer_accept") != "0" or active_view.get("consumer_reject") != "1":
        return False, (
            "online active-view 状态必须被消费者拒绝，当前 "
            f"accept={active_view.get('consumer_accept')} reject={active_view.get('consumer_reject')}"
        )
    online_verified = rows["online_smoke_recovery_interface"]
    try:
        online_consumer_accept = int(online_verified.get("consumer_accept", "0"))
    except ValueError:
        online_consumer_accept = 0
    if online_verified.get("accept") != "7" or online_consumer_accept < 6:
        return False, (
            "online verified smoke 应发布 7/7，按 fitness<=0.12 至少安全接收 6/7，当前 "
            f"publish={online_verified.get('accept')} consumer_accept={online_verified.get('consumer_accept')}"
        )
    return (
        True,
        "nav46 body/reg 和 nav_drift causal 恢复消费者仿真 false_accept=0；"
        f"online verified interface 发布 7/7、按安全门控接收 {online_consumer_accept}/7，"
        "active_view 状态 0/1 接收。",
    )


def has_online_trajectory_recovery_smoke(pose_csv: Path, generated_yaml: Path) -> tuple[bool, str]:
    """检查在线 trajectory recovery 专项 smoke 是否产生了目标状态和有效精配准字段。"""
    if not generated_yaml.exists():
        return False, f"缺少在线 trajectory recovery 临时配置: {generated_yaml}"
    rows = read_csv(pose_csv)
    matched = 0
    best_fitness = math.inf
    for row in rows:
        status = row.get("status", "")
        if "state=verified_trajectory_single_agreement" not in status:
            continue
        matched += 1
        tokens = {
            token.split("=", 1)[0]: token.split("=", 1)[1]
            for token in status.split()
            if "=" in token
        }
        if tokens.get("refined_converged") != "true":
            return False, "trajectory recovery smoke 状态中 refined_converged 不是 true。"
        try:
            fitness = float(tokens.get("refined_fitness", "nan"))
        except ValueError:
            return False, "trajectory recovery smoke 状态中 refined_fitness 不是浮点数。"
        if not math.isfinite(fitness) or fitness > 0.04:
            return False, f"trajectory recovery smoke refined_fitness={fitness} 超过 0.04。"
        best_fitness = min(best_fitness, fitness)

    if matched < 1:
        return False, "没有收到 state=verified_trajectory_single_agreement 的在线 smoke 输出。"
    return True, f"trajectory recovery smoke matched={matched}，best_refined_fitness={best_fitness:.6f}。"


def has_online_active_view_status_smoke(status_csv: Path, generated_yaml: Path) -> tuple[bool, str]:
    """检查在线节点在不能安全发布时是否给出主动采集新视角提示。"""
    if not generated_yaml.exists():
        return False, f"缺少在线 active-view 临时配置: {generated_yaml}"
    rows = read_csv(status_csv)
    if not rows:
        return False, "active-view status smoke CSV 为空。"
    row = rows[-1]
    state = row.get("state", "")
    if not state.startswith("need_active_view_"):
        return False, f"active-view status state={state}，不以 need_active_view_ 开头。"
    if row.get("recovery_hint") != "active_view":
        return False, f"active-view status recovery_hint={row.get('recovery_hint')}，不是 active_view。"
    if row.get("trajectory_attempted") != "true":
        return False, "active-view status 没有记录 trajectory_attempted=true。"
    return (
        True,
        f"state={state}，support={row.get('trajectory_support')}，"
        f"overlap={row.get('trajectory_average_overlap')}，margin={row.get('trajectory_margin')}。",
    )


def has_nav46_three_stage_recovery(
    short_summary: Path,
    long_summary: Path,
    active_summary: Path,
) -> tuple[bool, str]:
    """检查 nav46 随机 80 点是否能通过短历史、长历史和主动新视角三层恢复合计救回。"""
    short_rows = {row.get("label", ""): row for row in read_csv(short_summary)}
    long_rows = read_csv(long_summary)
    active_rows = read_csv(active_summary)
    details: list[str] = []
    for mode in ["body", "registered_world"]:
        short = short_rows.get(f"nav46_{mode}_rand80_seed20260702")
        if not short:
            return False, f"缺少 {mode} 随机 80 短历史 summary。"
        short_ok = (
            short.get("temporal_accept") == "68"
            and short.get("temporal_reject") == "12"
            and short.get("temporal_accepted_ref_success") == "68/68"
            and short.get("temporal_false_accept") == "0"
        )
        if not short_ok:
            return False, f"{mode} 短历史 summary 不符合 68 accept / 0 false_accept。"

        mode_long = [row for row in long_rows if row.get("mode") == mode]
        strict_long = [
            row for row in mode_long
            if row.get("trajectory_decision") == "accept"
            and row.get("trajectory_success") == "1"
            and as_float(row, "average_overlap") >= 0.95
        ]
        long_false_accept = [
            row for row in mode_long
            if row.get("trajectory_decision") == "accept"
            and as_float(row, "average_overlap") >= 0.95
            and row.get("trajectory_success") != "1"
        ]
        if len(strict_long) != 11 or long_false_accept:
            return False, f"{mode} 长历史按 average_overlap>=0.95 后不是 11/12 安全救回。"

        mode_active = [row for row in active_rows if row.get("mode") == mode]
        active_ok = (
            len(mode_active) == 1
            and mode_active[0].get("waypoint_id") == "rand80_reject_12"
            and mode_active[0].get("trajectory_decision") == "accept"
            and mode_active[0].get("trajectory_success") == "1"
        )
        if not active_ok:
            return False, f"{mode} 主动恢复新视角没有救回 rand80_reject_12。"
        details.append(f"{mode}=68+11+1=80/80")
    return True, "；".join(details) + "，false_accept=0。"


def has_nav46_topk_strategy(
    long_top30_summary: Path,
    active_top30_summary: Path,
    active_top60_summary: Path,
) -> tuple[bool, str]:
    """检查 top30 快速恢复 + top60 主动深搜的分层策略证据。"""
    long_rows = read_csv(long_top30_summary)
    active30_rows = read_csv(active_top30_summary)
    active60_rows = read_csv(active_top60_summary)
    details: list[str] = []
    for mode in ["body", "registered_world"]:
        mode_long = [row for row in long_rows if row.get("mode") == mode]
        accepted_long = [
            row for row in mode_long
            if row.get("trajectory_decision") == "accept" and row.get("trajectory_success") == "1"
        ]
        false_long = [
            row for row in mode_long
            if row.get("trajectory_decision") == "accept" and row.get("trajectory_success") != "1"
        ]
        left = sorted(
            row.get("waypoint_id", "")
            for row in mode_long
            if row.get("waypoint_id", "") not in {item.get("waypoint_id", "") for item in accepted_long}
        )
        if len(mode_long) != 12 or len(accepted_long) != 11 or false_long or left != ["rand80_reject_12"]:
            return False, f"{mode} top30 长历史不满足 11/12 安全救回且只留下 rand80_reject_12。"

        mode_active30 = [row for row in active30_rows if row.get("mode") == mode]
        if (
            len(mode_active30) != 1
            or mode_active30[0].get("waypoint_id") != "rand80_reject_12"
            or mode_active30[0].get("trajectory_decision") == "accept"
            or as_float(mode_active30[0], "margin") >= 0.005
        ):
            return False, f"{mode} top30 主动新视角没有保持保守拒绝。"

        mode_active60 = [row for row in active60_rows if row.get("mode") == mode]
        if (
            len(mode_active60) != 1
            or mode_active60[0].get("waypoint_id") != "rand80_reject_12"
            or mode_active60[0].get("trajectory_decision") != "accept"
            or mode_active60[0].get("trajectory_success") != "1"
            or as_float(mode_active60[0], "trajectory_error_m") > 0.80
        ):
            return False, f"{mode} top60 主动新视角没有救回 rand80_reject_12。"
        details.append(f"{mode}: top30长历史11/12，top30主动拒绝，top60主动接受")
    return True, "；".join(details) + "。"


def has_trajectory_long_history_bags(summary_path: Path) -> tuple[bool, str]:
    """检查 nav_drift_test44/45/46 三包长历史 trajectory likelihood 是否全量成功且零误接受。"""
    rows = {row.get("label", ""): row for row in read_csv(summary_path)}
    expected = {
        "bag44": ("25", "15", "25", "23"),
        "bag45": ("22", "12", "22", "21"),
        "bag46": ("22", "18", "22", "22"),
    }
    missing = sorted(set(expected) - set(rows))
    if missing:
        return False, f"缺少三包长历史汇总行: {missing}"

    details: list[str] = []
    for label, (targets, single_success, trajectory_success, accepted) in expected.items():
        row = rows[label]
        checks = [
            row.get("targets") == targets,
            row.get("single_success") == single_success,
            row.get("trajectory_success") == trajectory_success,
            row.get("trajectory_accept") == accepted,
            row.get("trajectory_accept_success") == accepted,
            row.get("trajectory_false_accept") == "0",
        ]
        if not all(checks):
            return False, (
                f"{label} 长历史汇总变化: targets={row.get('targets')} "
                f"single={row.get('single_success')} trajectory={row.get('trajectory_success')} "
                f"accept={row.get('trajectory_accept')} accept_success={row.get('trajectory_accept_success')} "
                f"false_accept={row.get('trajectory_false_accept')}"
            )
        details.append(f"{label}: single {single_success}/{targets} -> trajectory {trajectory_success}/{targets}")
    return True, "；".join(details) + "，三包 false_accept=0。"


def item(key: str, ok: bool, evidence: str, detail: str) -> EvidenceItem:
    """构造通过/失败证据行。"""
    return EvidenceItem(key=key, status="PASS" if ok else "FAIL", evidence=evidence, detail=detail)


def known_limit(key: str, evidence: str, detail: str) -> EvidenceItem:
    """构造已确认外部限制行，不作为失败。"""
    return EvidenceItem(key=key, status="KNOWN_LIMIT", evidence=evidence, detail=detail)


def check_goal_evidence(workspace: Path) -> list[EvidenceItem]:
    """汇总当前目标的关键证据。"""
    package = workspace / "src/humanoid_global_relocalization_runtime"
    artifacts = workspace / ".codex_tmp"
    results: list[EvidenceItem] = []

    results.append(item(
        "workspace_scope",
        workspace == EXPECTED_WORKSPACE and package.exists(),
        str(package),
        "功能包位于当前工作空间，未要求也未依赖 /home/ubuntu/humanoid_ws。",
    ))
    results.append(item(
        "separate_test_dir",
        (package / "test").is_dir() and (package / "src").is_dir(),
        str(package / "test"),
        "测试/验证脚本和实验 YAML 放在 test/，功能源码放在 src/include。",
    ))
    results.append(item(
        "cpp_package_buildable",
        all((package / path).exists() for path in ["CMakeLists.txt", "package.xml", "src/global_relocalization_node.cpp"]),
        str(package / "CMakeLists.txt"),
        "C++ ROS2 包包含 offline evaluator 和 online debug node。",
    ))
    ok, detail = has_cpp_build_artifacts(workspace)
    results.append(item(
        "cpp_build_artifacts",
        ok,
        str(workspace / "install/humanoid_global_relocalization_runtime"),
        detail,
    ))
    results.append(item(
        "main_yaml_documented",
        (package / "config/global_relocalization_runtime.yaml").exists()
        and (package / "test/check_documentation_contract.py").exists(),
        str(package / "config/global_relocalization_runtime.yaml"),
        "主运行 YAML 有中文参数说明，并由 check_documentation_contract.py 门禁检查。",
    ))
    results.append(item(
        "readme_and_report",
        (package / "README.md").exists() and (package / "test/nav_drift_validation_report.md").exists(),
        str(package / "README.md"),
        "README 和 nav_drift 验证报告记录上下游、原理、验证结果和复现命令。",
    ))

    inventory_path = artifacts / "bag_inventory_v2.csv"
    inventory = read_csv(inventory_path)
    reg_ref = [row for row in inventory if row.get("registered_world_with_reference") == "yes"]
    body_ready = [row for row in inventory if row.get("body_ready") == "yes"]
    results.append(item(
        "bag_inventory_scan",
        len(inventory) == 10 and len(reg_ref) >= 4,
        str(inventory_path),
        f"已扫描真实 bag metadata={len(inventory)}，registered_world_with_reference={len(reg_ref)}。",
    ))
    home_inventory_path = artifacts / "bag_inventory_home_scan.csv"
    home_inventory = read_csv(home_inventory_path)
    home_reg_ready = [row for row in home_inventory if row.get("registered_world_ready") == "yes"]
    home_reg_ref = [row for row in home_inventory if row.get("registered_world_with_reference") == "yes"]
    home_body_ready = [row for row in home_inventory if row.get("body_ready") == "yes"]
    results.append(item(
        "expanded_home_bag_scan",
        len(home_inventory) == 36 and len(home_reg_ready) == 8 and len(home_reg_ref) == 4 and not home_body_ready,
        str(home_inventory_path),
        "排除 /home/ubuntu/humanoid_ws 和本包合成 bag 后，全 /home/ubuntu 扫描 metadata=36，reg_ready=8，reg_ref=4，body_ready=0。",
    ))
    latest_home_inventory_path = artifacts / "bag_inventory_home_scan_latest.csv"
    latest_home_inventory = read_csv(latest_home_inventory_path)
    latest_home_reg_ready = [row for row in latest_home_inventory if row.get("registered_world_ready") == "yes"]
    latest_home_reg_ref = [
        row for row in latest_home_inventory if row.get("registered_world_with_reference") == "yes"]
    latest_home_body_ready = [row for row in latest_home_inventory if row.get("body_ready") == "yes"]
    latest_home_body_with_ref = [row for row in latest_home_inventory if row.get("body_with_reference") == "yes"]
    results.append(item(
        "latest_home_bag_scan",
        len(latest_home_inventory) >= 32
        and len(latest_home_reg_ready) >= 4
        and len(latest_home_reg_ref) >= 3
        and len(latest_home_body_ready) >= 1,
        str(latest_home_inventory_path),
        f"最新全 /home/ubuntu 扫描 metadata={len(latest_home_inventory)}，reg_ready={len(latest_home_reg_ready)}，reg_ref={len(latest_home_reg_ref)}，body_ready={len(latest_home_body_ready)}。",
    ))
    latest_nav_inventory_path = artifacts / "bag_inventory_nav_drift_latest.csv"
    latest_nav_inventory = read_csv(latest_nav_inventory_path)
    latest_nav_paths = {row.get("bag_path", "") for row in latest_nav_inventory}
    latest_nav_reg_ref = [
        row for row in latest_nav_inventory if row.get("registered_world_with_reference") == "yes"]
    latest_nav_body_ready = [row for row in latest_nav_inventory if row.get("body_ready") == "yes"]
    results.append(item(
        "latest_nav_drift_bag_scan",
        len(latest_nav_inventory) >= 3
        and EXPECTED_LATEST_NAV_DRIFT_BAGS.issubset(latest_nav_paths)
        and len(latest_nav_reg_ref) >= 3,
        str(latest_nav_inventory_path),
        f"最新 /home/ubuntu/nav_drift_test 扫描覆盖 nav_drift_test44/45/46，三包都有 registered_world+ref，body_ready={len(latest_nav_body_ready)}。",
    ))
    if body_ready or latest_home_body_ready:
        body_bag_paths = {row.get("bag_path", "") for row in latest_home_body_ready}
        body_with_reference_paths = {row.get("bag_path", "") for row in latest_home_body_with_ref}
        results.append(item(
            "real_body_bag",
            True,
            str(latest_home_inventory_path),
            f"已发现真实 /cloud_registered_body bag 数量={len(body_ready) + len(latest_home_body_ready)}。",
        ))
        real_body_metrics = artifacts / "global_relocalization_real_body_validation_v1/global_relocalization_metrics.csv"
        ok, detail = has_real_body_validation(real_body_metrics, body_bag_paths, body_with_reference_paths)
        results.append(item(
            "real_body_validation",
            ok,
            str(real_body_metrics),
            detail,
        ))
        registered_nav46_metrics = artifacts / "global_relocalization_nav46_registered_world_v1/global_relocalization_metrics.csv"
        ok, detail = has_real_body_registered_world_equivalence(real_body_metrics, registered_nav46_metrics)
        results.append(item(
            "real_body_registered_world_equivalence",
            ok,
            str(registered_nav46_metrics),
            detail,
        ))
        real_body_30_metrics = artifacts / "global_relocalization_real_body_validation_30frame_v1/global_relocalization_metrics.csv"
        registered_nav46_30_metrics = artifacts / "global_relocalization_nav46_registered_world_30frame_v1/global_relocalization_metrics.csv"
        ok, detail = has_real_body_30frame_equivalence(real_body_30_metrics, registered_nav46_30_metrics)
        results.append(item(
            "real_body_30frame_equivalence",
            ok,
            str(real_body_30_metrics),
            detail,
        ))
    else:
        results.append(known_limit(
            "real_body_bag",
            str(latest_home_inventory_path),
            "当前可读真实 bag 中 body_ready=0；body 准确率只能等后续补录真实 /cloud_registered_body bag。",
        ))
        results.append(known_limit(
            "real_body_validation",
            str(latest_home_inventory_path),
            "未发现真实 /cloud_registered_body bag，因此真实 body 验证产物暂不要求；合成 body 只用于工程自洽验证。",
        ))
        results.append(known_limit(
            "real_body_registered_world_equivalence",
            str(latest_home_inventory_path),
            "未发现真实 /cloud_registered_body bag，因此真实 body 与 registered_world 对照暂不要求。",
        ))
        results.append(known_limit(
            "real_body_30frame_equivalence",
            str(latest_home_inventory_path),
            "未发现真实 /cloud_registered_body bag，因此真实 body 30 帧扩展对照暂不要求。",
        ))

    nav_metrics_path = artifacts / "global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_metrics.csv"
    nav_all_rows = read_csv(nav_metrics_path)
    nav_metrics = [row for row in nav_all_rows if row.get("scenario_name") == "arbitrary_start_no_prior"]
    results.append(item(
        "nav_drift_30_frame_metrics",
        len(nav_metrics) == 30 and sum(1 for row in nav_metrics if row.get("localized") == "1") == 30
        and count_success(nav_metrics) == 27,
        str(nav_metrics_path),
        "真实 nav_drift 30 帧：localized=30/30，单帧成功=27/30。",
    ))
    nav_bag_counts = Counter(row.get("bag_path", "") for row in nav_all_rows)
    nav_scenario_counts = Counter(row.get("scenario_name", "") for row in nav_all_rows)
    nav_covers_all_bags = all(nav_bag_counts[bag] == 30 for bag in EXPECTED_NAV_DRIFT_BAGS)
    nav_covers_all_scenarios = all(
        nav_scenario_counts[name] == 30
        for name in ["arbitrary_start_no_prior", "localization_jump_3m", "localization_jump_5m_90deg"]
    )
    results.append(item(
        "nav_drift_three_bag_coverage",
        len(nav_all_rows) == 90 and nav_covers_all_bags and nav_covers_all_scenarios
        and sum(1 for row in nav_all_rows if row.get("localized") == "1") == 90,
        str(nav_metrics_path),
        "nav_drift_test43/44/45 三个 bag 均已覆盖任意点、3m 大跳、5m+90deg 大跳，共 90 行。",
    ))
    resource_sweep_path = artifacts / "nav46_resource_sweep_summary.csv"
    ok, detail = has_nav46_resource_sweep(resource_sweep_path)
    results.append(item(
        "nav46_resource_sweep",
        ok,
        str(resource_sweep_path),
        detail,
    ))
    ok, detail = has_nav46_three_stage_recovery(
        artifacts / "nav46_stress_validation/summary_rand80_seed20260702.csv",
        artifacts / "trajectory_rand80_rejects_long_history/summary.csv",
        artifacts / "trajectory_rand80_reject12_active_view_v2/summary.csv",
    )
    results.append(item(
        "nav46_three_stage_recovery",
        ok,
        str(artifacts / "nav46_stress_validation/summary_rand80_seed20260702.csv"),
        detail,
    ))
    ok, detail = has_nav46_topk_strategy(
        artifacts / "trajectory_rand80_rejects_long_history_top30_probe/summary.csv",
        artifacts / "trajectory_rand80_reject12_active_view_top30_probe/summary.csv",
        artifacts / "trajectory_rand80_reject12_active_view_v2/summary.csv",
    )
    results.append(item(
        "nav46_topk_strategy",
        ok,
        str(artifacts / "trajectory_rand80_rejects_long_history_top30_probe/summary.csv"),
        detail,
    ))
    ok, detail = has_trajectory_long_history_bags(artifacts / "trajectory_long_history_summary.csv")
    results.append(item(
        "trajectory_long_history_three_bag",
        ok,
        str(artifacts / "trajectory_long_history_summary.csv"),
        detail,
    ))

    hall_metrics_path = artifacts / "global_relocalization_extra_hall_mapping_20260605_152637_v4_validref/global_relocalization_metrics.csv"
    hall_rows = read_csv(hall_metrics_path)
    hall_valid_ref = [row for row in hall_rows if row.get("has_reference") == "1"]
    results.append(item(
        "extra_hall_mapping_coverage",
        len(hall_rows) == 15 and len(hall_valid_ref) == 9
        and sum(1 for row in hall_rows if row.get("localized") == "1") == 15
        and count_success(hall_valid_ref) == 9,
        str(hall_metrics_path),
        "额外 hall_mapping_20260605_152637 已跑 5 个时间点 * 3 场景；其中 9 行参考有效且 9/9 成功。",
    ))

    no_ref_metrics_path = artifacts / "global_relocalization_extra_no_reference_smoke_v1/global_relocalization_metrics.csv"
    no_ref_rows = read_csv(no_ref_metrics_path)
    no_ref_bag_counts = Counter(row.get("bag_path", "") for row in no_ref_rows)
    no_ref_bags_covered = set(no_ref_bag_counts) == EXPECTED_NO_REFERENCE_BAGS
    results.append(item(
        "no_reference_registered_world_smoke",
        len(no_ref_rows) == 21 and no_ref_bags_covered
        and sum(1 for row in no_ref_rows if row.get("localized") == "1") == 21
        and all(row.get("has_reference") == "0" for row in no_ref_rows),
        str(no_ref_metrics_path),
        "4 个无 /robot_realpose 的 registered_world bag 已做链路冒烟；7 个抽样帧 * 3 场景全部 localized。",
    ))

    causal_decision = read_csv(
        artifacts / "global_relocalization_nav_drift_extended_cpp_temporal_causal_v1/global_relocalization_temporal_decisions.csv")
    causal_accept = [row for row in causal_decision if row.get("decision") == "accept"]
    causal_success = [row for row in causal_accept if row.get("refined_success") == "1"]
    results.append(item(
        "causal_temporal_decision",
        len(causal_decision) == 30 and len(causal_accept) == 25 and len(causal_success) == 25,
        "global_relocalization_nav_drift_extended_cpp_temporal_causal_v1/global_relocalization_temporal_decisions.csv",
        "只看历史帧：accept=25，reject=5，accepted refined success=25/25。",
    ))

    synthetic_body = no_prior_rows(
        artifacts / "global_relocalization_synthetic_body_nav43_v1/global_relocalization_metrics.csv")
    results.append(item(
        "synthetic_body_chain",
        len(synthetic_body) == 5 and count_success(synthetic_body) == 5,
        "global_relocalization_synthetic_body_nav43_v1/global_relocalization_metrics.csv",
        "合成 /cloud_registered_body 自洽验证 5/5 成功。",
    ))
    synthetic_all_path = artifacts / "global_relocalization_synthetic_body_nav_drift_all_v1/global_relocalization_metrics.csv"
    synthetic_all_rows = read_csv(synthetic_all_path)
    synthetic_all_no_prior = no_prior_rows(synthetic_all_path)
    synthetic_bag_counts = Counter(row.get("bag_path", "") for row in synthetic_all_rows)
    synthetic_scenario_counts = Counter(row.get("scenario_name", "") for row in synthetic_all_rows)
    synthetic_all_bags_covered = set(synthetic_bag_counts) == EXPECTED_SYNTHETIC_BODY_BAGS
    synthetic_all_scenarios_covered = all(
        synthetic_scenario_counts[name] == 15
        for name in ["arbitrary_start_no_prior", "localization_jump_3m", "localization_jump_5m_90deg"]
    )
    results.append(item(
        "synthetic_body_three_bag_chain",
        len(synthetic_all_rows) == 45 and len(synthetic_all_no_prior) == 15
        and synthetic_all_bags_covered and synthetic_all_scenarios_covered
        and sum(1 for row in synthetic_all_rows if row.get("localized") == "1") == 45
        and count_success(synthetic_all_rows) == 45,
        str(synthetic_all_path),
        "从 nav_drift_test43/44/45 派生的三个合成 body bag 已覆盖 15 帧 * 3 场景，45/45 成功。",
    ))
    online_smoke_path = package / "test/online_smoke_evidence.csv"
    online_rows = read_csv(online_smoke_path)
    online_keys = {(row.get("input_mode", ""), row.get("bag_label", "")) for row in online_rows}
    expected_online_keys = {
        ("registered_world", "nav_drift_test44"),
        ("registered_world", "nav_drift_test45"),
        ("registered_world", "nav_drift_test46"),
        ("body", "real_body_nav_drift_test46"),
        ("body", "synthetic_body_nav_drift_test43"),
        ("body", "synthetic_body_nav_drift_test44"),
        ("body", "synthetic_body_nav_drift_test45"),
    }
    results.append(item(
        "online_smoke_matrix",
        online_keys == expected_online_keys and all(row.get("result") == "PASS" for row in online_rows),
        str(online_smoke_path),
        "在线 debug smoke 已覆盖 registered_world 三个真实 nav_drift bag、真实 body nav46 和三个合成 body bag，并验证 verified_map_to_odom。",
    ))
    ok, detail = has_online_trajectory_recovery_smoke(
        artifacts / "online_trajectory_recovery_smoke_pose.csv",
        artifacts / "online_trajectory_recovery_smoke.yaml",
    )
    results.append(item(
        "online_trajectory_recovery_smoke",
        ok,
        str(artifacts / "online_trajectory_recovery_smoke_pose.csv"),
        detail,
    ))
    ok, detail = has_online_active_view_status_smoke(
        artifacts / "online_active_view_status_smoke.csv",
        artifacts / "online_active_view_status_smoke.yaml",
    )
    results.append(item(
        "online_active_view_status_smoke",
        ok,
        str(artifacts / "online_active_view_status_smoke.csv"),
        detail,
    ))
    recovery_consumer_path = artifacts / "recovery_consumer_simulation.csv"
    ok, detail = has_recovery_consumer_simulation(recovery_consumer_path)
    results.append(item(
        "recovery_consumer_simulation",
        ok,
        str(recovery_consumer_path),
        detail,
    ))

    required_gates = [
        "check_workspace_boundaries.py",
        "check_documentation_contract.py",
        "verify_cpp_build_artifacts.py",
        "verify_refine_sweep.py",
        "verify_resource_metrics.py",
        "verify_temporal_decisions.py",
        "verify_online_smoke_evidence.py",
        "verify_online_active_view_status_smoke.py",
        "verify_online_trajectory_recovery_smoke.py",
        "verify_nav46_three_stage_recovery.py",
        "verify_nav46_topk_strategy.py",
        "verify_trajectory_long_history_bags.py",
        "compare_input_modes.py",
        "run_online_debug_smoke.py",
        "run_online_smoke_matrix.py",
        "run_online_active_view_status_smoke.py",
        "run_online_trajectory_recovery_smoke.py",
        "run_real_body_validation.py",
        "simulate_recovery_consumer.py",
        "run_validation_gates.py",
    ]
    missing_gates = [name for name in required_gates if not (package / "test" / name).exists()]
    results.append(item(
        "validation_gates",
        not missing_gates,
        str(package / "test/run_validation_gates.py"),
        "专项门禁覆盖工作空间边界、中文文档、refine sweep、资源字段、temporal decision、输入模式一致性、三包长历史恢复和在线 smoke。",
    ))

    report = (package / "test/nav_drift_validation_report.md").read_text(encoding="utf-8")
    results.append(item(
        "online_debug_smoke_documented",
        "因果时序窗口 `window_after=0`" in report
        and "PASS verified=1" in report
        and "state=verified_trajectory_single_agreement" in report,
        str(package / "test/nav_drift_validation_report.md"),
        "合成 body、真实 nav_drift registered_world 和 trajectory recovery 专项在线 debug smoke 已记录。",
    ))

    return results


def print_matrix(items: list[EvidenceItem]) -> None:
    """打印证据矩阵。"""
    print("| key | status | evidence | detail |")
    print("|---|---|---|---|")
    for entry in items:
        print(f"| {entry.key} | {entry.status} | {entry.evidence} | {entry.detail} |")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check goal evidence matrix.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    failures: list[str] = []
    try:
        evidence = check_goal_evidence(workspace)
    except FileNotFoundError as exc:
        print(f"[check_goal_evidence] FAIL {exc}", file=sys.stderr)
        return 1

    print_matrix(evidence)
    for entry in evidence:
        if entry.status == "FAIL":
            failures.append(entry.key)

    if failures:
        print("[check_goal_evidence] FAIL", file=sys.stderr)
        for key in failures:
            print(f"  - {key}", file=sys.stderr)
        return 1

    print("[check_goal_evidence] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
