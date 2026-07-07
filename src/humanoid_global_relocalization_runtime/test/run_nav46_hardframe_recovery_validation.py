#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_nav46_hardframe_recovery_validation.py

文件作用：
  1. 这是 nav_drift_test46 难例补救专项验证脚本，不属于线上功能源码。
  2. 固定读取 test/nav46_rand80_reject_matches.csv 中 12 个短历史门控拒绝点。
  3. 第一阶段重跑“长历史轨迹 likelihood”，默认用 top30 快速验证 12 个难点中 11 个能安全救回。
  4. 第二阶段对最后一个仍有歧义的 rand80_reject_12 加入目标帧后的主动恢复新视角，默认升到 top60 深搜验证它也能救回。
  5. 脚本只编排已有 run_trajectory_likelihood_validation.py，不重复实现算法逻辑。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_nav46_hardframe_recovery_validation.py --run
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from pathlib import Path


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def run_command(workspace: Path, command: list[str]) -> int:
    """运行一个验证子命令，并把 stdout/stderr 透传，方便现场看进度。"""
    process = subprocess.run(
        command,
        cwd=str(workspace),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(process.stdout, end="")
    if process.stderr:
        print(process.stderr, end="", file=sys.stderr)
    return process.returncode


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV；缺文件时抛出清晰错误，让门禁失败原因直接可见。"""
    if not path.exists():
        raise FileNotFoundError(f"missing csv: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def count_success(path: Path, strict_average_overlap: float) -> tuple[int, int, int]:
    """统计 trajectory summary 中安全接受、成功接受和误接受数量。"""
    rows = read_csv(path)
    accepted = [
        row for row in rows
        if row.get("trajectory_decision") == "accept"
        and float(row.get("average_overlap", "0")) >= strict_average_overlap
    ]
    success = [row for row in accepted if row.get("trajectory_success") == "1"]
    false_accept = [row for row in accepted if row.get("trajectory_success") != "1"]
    return len(accepted), len(success), len(false_accept)


def verify_outputs(workspace: Path, strict_average_overlap: float) -> int:
    """复核两个阶段的输出，避免脚本只跑完但没有真正救回难例。"""
    artifacts = workspace / ".codex_tmp"
    long_summary = artifacts / "trajectory_rand80_rejects_long_history/summary.csv"
    active_summary = artifacts / "trajectory_rand80_reject12_active_view_v2/summary.csv"

    long_rows = read_csv(long_summary)
    active_rows = read_csv(active_summary)
    failures: list[str] = []

    for mode in ("body", "registered_world"):
        mode_rows = [row for row in long_rows if row.get("mode") == mode]
        if len(mode_rows) != 12:
            failures.append(f"{mode} long history rows expected 12, got {len(mode_rows)}")
        accepted = [
            row for row in mode_rows
            if row.get("trajectory_decision") == "accept"
            and row.get("trajectory_success") == "1"
            and float(row.get("average_overlap", "0")) >= strict_average_overlap
        ]
        false_accept = [
            row for row in mode_rows
            if row.get("trajectory_decision") == "accept"
            and float(row.get("average_overlap", "0")) >= strict_average_overlap
            and row.get("trajectory_success") != "1"
        ]
        left = sorted(
            row.get("waypoint_id", "")
            for row in mode_rows
            if row.get("waypoint_id", "") not in {item.get("waypoint_id", "") for item in accepted}
        )
        if len(accepted) != 11 or left != ["rand80_reject_12"]:
            failures.append(f"{mode} long history expected 11 safe recoveries and rand80_reject_12 left, got {len(accepted)} left={left}")
        if false_accept:
            failures.append(f"{mode} long history false_accept={len(false_accept)}")

        active = [row for row in active_rows if row.get("mode") == mode]
        if len(active) != 1:
            failures.append(f"{mode} active recovery rows expected 1, got {len(active)}")
            continue
        row = active[0]
        if row.get("waypoint_id") != "rand80_reject_12":
            failures.append(f"{mode} active target should be rand80_reject_12")
        if row.get("trajectory_decision") != "accept" or row.get("trajectory_success") != "1":
            failures.append(f"{mode} active recovery did not accept successfully")

    long_accept, long_success, long_false = count_success(long_summary, strict_average_overlap)
    active_accept, active_success, active_false = count_success(active_summary, 0.0)
    print(
        "[nav46_hardframe_recovery] "
        f"long_history_accept={long_accept} long_success={long_success} long_false_accept={long_false} "
        f"active_accept={active_accept} active_success={active_success} active_false_accept={active_false}"
    )

    if failures:
        print("[nav46_hardframe_recovery] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[nav46_hardframe_recovery] PASS")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Run nav46 hard-frame long-history and active-view recovery validation.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--bag", type=Path, default=Path("/home/ubuntu/nav_drift_test/nav_drift_test46"), help="验证 bag")
    parser.add_argument("--matches", type=Path, default=None, help="难例点位 CSV；默认使用 test/nav46_rand80_reject_matches.csv")
    parser.add_argument("--run", action="store_true", help="实际运行两个 evaluator；不加时只复核已有产物")
    parser.add_argument("--strict-average-overlap", type=float, default=0.95, help="长历史阶段安全接受的平均 overlap 下限")
    parser.add_argument("--long-top-k", type=int, default=30, help="长历史二阶段使用的 BBS top-K，默认 top30 以降低粗搜索成本")
    parser.add_argument("--active-top-k", type=int, default=60, help="主动新视角三阶段使用的 BBS top-K，默认 top60 作为只在最后歧义点触发的深搜")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    script_dir = Path(__file__).resolve().parent
    matches = args.matches or script_dir / "nav46_rand80_reject_matches.csv"
    trajectory_script = script_dir / "run_trajectory_likelihood_validation.py"

    if args.run:
        # 长历史阶段：只使用目标帧之前的 6 个历史视角 + 当前帧，模拟机器人定位飘了以后停下来，
        # 但缓存里还保留最近一段 odom/scan 的情况。
        # 当前 bag46 难例验证显示 top30 已能救回 11/12 个短历史拒绝点，因此默认先用 top30 快速尝试。
        long_command = [
            sys.executable,
            str(trajectory_script),
            "--workspace",
            str(workspace),
            "--bag",
            str(args.bag),
            "--matches",
            str(matches),
            "--output-root",
            ".codex_tmp/trajectory_rand80_rejects_long_history",
            "--modes",
            "body",
            "registered_world",
            "--history-offsets",
            "240",
            "200",
            "160",
            "120",
            "80",
            "40",
            "0",
            "--top-k",
            str(args.long_top_k),
            "--trajectory-max-candidates",
            str(args.long_top_k),
            "--trajectory-min-average-overlap",
            str(args.strict_average_overlap),
            "--run",
        ]
        rc = run_command(workspace, long_command)
        if rc != 0:
            return rc

        # 主动恢复阶段：对最后一个长历史仍歧义的点，额外加入目标帧之后的新视角。
        # 这对应线上状态机收到 need_active_view_* 后，低速转动/小范围移动重新采集视角再触发恢复。
        # top30 在该点仍会保守拒绝；top60 能找到正确候选，因此默认只在这一层使用深搜。
        active_command = [
            sys.executable,
            str(trajectory_script),
            "--workspace",
            str(workspace),
            "--bag",
            str(args.bag),
            "--matches",
            str(matches),
            "--ids",
            "rand80_reject_12",
            "--output-root",
            ".codex_tmp/trajectory_rand80_reject12_active_view_v2",
            "--modes",
            "body",
            "registered_world",
            "--history-offsets",
            "240",
            "200",
            "160",
            "120",
            "80",
            "40",
            "0",
            "-40",
            "-80",
            "-120",
            "-160",
            "-200",
            "-240",
            "--top-k",
            str(args.active_top_k),
            "--trajectory-max-candidates",
            str(args.active_top_k),
            "--trajectory-min-average-overlap",
            str(args.strict_average_overlap),
            "--run",
        ]
        rc = run_command(workspace, active_command)
        if rc != 0:
            return rc

    return verify_outputs(workspace, args.strict_average_overlap)


if __name__ == "__main__":
    raise SystemExit(main())
