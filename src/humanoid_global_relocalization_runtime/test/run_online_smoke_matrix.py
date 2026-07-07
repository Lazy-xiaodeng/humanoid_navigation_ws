#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_online_smoke_matrix.py

文件作用：
  1. 这是在线 debug 冒烟矩阵运行脚本，不属于功能源码。
  2. 顺序调用 run_online_debug_smoke.py，覆盖 registered_world 和 body 两条在线输入链路。
  3. 通过每个场景的姿态证据 CSV 读取 verified_candidate 和 verified_map_to_odom 位姿。
  4. 将结果写入 online_smoke_evidence.csv，供 verify_online_smoke_evidence.py 和总门禁复查。

使用前提：
  - 已经 source /opt/ros/jazzy/setup.bash 和当前工作空间 install/local_setup.bash。
  - 已经存在 .codex_tmp/synthetic_body_nav_drift_test43/44/45。
  - 如果要覆盖真实 body 在线 smoke，需要 /home/ubuntu/nav_drift_test/nav_drift_test46。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_online_smoke_matrix.py
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

@dataclass(frozen=True)
class SmokeCase:
    """在线 smoke 矩阵中的一个场景。"""

    input_mode: str
    bag_label: str
    bag_path: str
    config_path: str
    note: str


CASES = [
    SmokeCase(
        input_mode="registered_world",
        bag_label="nav_drift_test44",
        bag_path="/home/ubuntu/nav_drift_test/nav_drift_test44",
        config_path="src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_online_debug.yaml",
        note="真实 registered_world 在线 smoke，support>=2 后发布 verified_candidate 和 verified_map_to_odom",
    ),
    SmokeCase(
        input_mode="registered_world",
        bag_label="nav_drift_test45",
        bag_path="/home/ubuntu/nav_drift_test/nav_drift_test45",
        config_path="src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_online_debug.yaml",
        note="真实 registered_world 在线 smoke，support>=2 后发布 verified_candidate 和 verified_map_to_odom",
    ),
    SmokeCase(
        input_mode="registered_world",
        bag_label="nav_drift_test46",
        bag_path="/home/ubuntu/nav_drift_test/nav_drift_test46",
        config_path="src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_online_debug.yaml",
        note="真实 registered_world 在线 smoke，support>=2 后发布 verified_candidate 和 verified_map_to_odom",
    ),
    SmokeCase(
        input_mode="body",
        bag_label="real_body_nav_drift_test46",
        bag_path="/home/ubuntu/nav_drift_test/nav_drift_test46",
        config_path="src/humanoid_global_relocalization_runtime/test/synthetic_body_online_debug.yaml",
        note="真实 body 在线 smoke，support>=2 后发布 verified_candidate 和 verified_map_to_odom",
    ),
    SmokeCase(
        input_mode="body",
        bag_label="synthetic_body_nav_drift_test43",
        bag_path="/home/ubuntu/software/Todesk/Files/humanoid_ws/.codex_tmp/synthetic_body_nav_drift_test43",
        config_path="src/humanoid_global_relocalization_runtime/test/synthetic_body_online_debug.yaml",
        note="合成 body 在线 smoke，support>=2 后发布 verified_candidate 和 verified_map_to_odom",
    ),
    SmokeCase(
        input_mode="body",
        bag_label="synthetic_body_nav_drift_test44",
        bag_path="/home/ubuntu/software/Todesk/Files/humanoid_ws/.codex_tmp/synthetic_body_nav_drift_test44",
        config_path="src/humanoid_global_relocalization_runtime/test/synthetic_body_online_debug.yaml",
        note="合成 body 在线 smoke，support>=2 后发布 verified_candidate 和 verified_map_to_odom",
    ),
    SmokeCase(
        input_mode="body",
        bag_label="synthetic_body_nav_drift_test45",
        bag_path="/home/ubuntu/software/Todesk/Files/humanoid_ws/.codex_tmp/synthetic_body_nav_drift_test45",
        config_path="src/humanoid_global_relocalization_runtime/test/synthetic_body_online_debug.yaml",
        note="合成 body 在线 smoke，support>=2 后发布 verified_candidate 和 verified_map_to_odom",
    ),
]


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def run_case(workspace: Path, case: SmokeCase, timeout: float) -> dict[str, str]:
    """运行一个在线 smoke 场景，并返回 CSV 行。"""
    script = workspace / "src/humanoid_global_relocalization_runtime/test/run_online_debug_smoke.py"
    config = workspace / case.config_path
    bag = Path(case.bag_path)
    pose_path = workspace / f".codex_tmp/online_smoke_pose_{case.input_mode}_{case.bag_label}.csv"
    if pose_path.exists():
        pose_path.unlink()
    cmd = [
        sys.executable,
        str(script),
        "--config",
        str(config),
        "--bag",
        str(bag),
        "--timeout",
        str(timeout),
        "--pose-output",
        str(pose_path),
    ]
    print(f"[online_smoke_matrix] running {case.input_mode} {case.bag_label}", flush=True)
    process = subprocess.Popen(
        cmd,
        cwd=str(workspace),
        start_new_session=True,
    )
    timed_out = False
    try:
        returncode = process.wait(timeout=timeout + 90.0)
    except subprocess.TimeoutExpired:
        timed_out = True
        os.killpg(process.pid, signal.SIGINT)
        try:
            returncode = process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            returncode = process.wait(timeout=10.0)

    pose_row: dict[str, str] | None = None
    if pose_path.exists():
        with pose_path.open(newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if rows:
            pose_row = rows[-1]
    ok = returncode == 0 and pose_row is not None
    return {
        "input_mode": case.input_mode,
        "bag_label": case.bag_label,
        "bag_path": case.bag_path,
        "config_path": case.config_path,
        "verified_x_m": pose_row.get("verified_x_m", "nan") if pose_row else "nan",
        "verified_y_m": pose_row.get("verified_y_m", "nan") if pose_row else "nan",
        "verified_z_m": pose_row.get("verified_z_m", "nan") if pose_row else "nan",
        "map_odom_x_m": pose_row.get("map_odom_x_m", "nan") if pose_row else "nan",
        "map_odom_y_m": pose_row.get("map_odom_y_m", "nan") if pose_row else "nan",
        "map_odom_yaw_deg": pose_row.get("map_odom_yaw_deg", "nan") if pose_row else "nan",
        "result": "PASS" if ok else "FAIL",
        "status": pose_row.get("status", "") if pose_row else "",
        "note": case.note if ok else f"online smoke failed returncode={returncode} timed_out={timed_out}",
    }


def write_evidence(path: Path, rows: list[dict[str, str]]) -> None:
    """原子式写入在线 smoke evidence CSV，避免中途失败留下半截文件。"""
    fieldnames = [
        "input_mode",
        "bag_label",
        "bag_path",
        "config_path",
        "verified_x_m",
        "verified_y_m",
        "verified_z_m",
        "map_odom_x_m",
        "map_odom_y_m",
        "map_odom_yaw_deg",
        "result",
        "status",
        "note",
    ]
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    with tmp_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    tmp_path.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run online smoke matrix and write evidence CSV.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--timeout", type=float, default=180.0, help="单个 smoke 等待 verified_candidate 的最长秒数")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="输出 evidence CSV，默认写入包 test/online_smoke_evidence.csv",
    )
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    output = args.output or workspace / "src/humanoid_global_relocalization_runtime/test/online_smoke_evidence.csv"
    rows = [run_case(workspace, case, args.timeout) for case in CASES]
    write_evidence(output, rows)
    failed = [row for row in rows if row["result"] != "PASS"]
    print(f"[online_smoke_matrix] wrote {output} rows={len(rows)} failed={len(failed)}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
