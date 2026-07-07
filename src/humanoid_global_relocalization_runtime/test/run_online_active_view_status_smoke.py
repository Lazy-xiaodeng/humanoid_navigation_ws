#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_online_active_view_status_smoke.py

文件作用：
  1. 这是在线主动恢复状态专项冒烟测试脚本，不属于功能源码。
  2. 基于 synthetic_body_online_debug.yaml 生成临时 YAML，故意让普通 verified 和 fallback verified 都无法发布。
  3. 播放合成 body bag 后，只等待 /global_relocalization/recovery_status 中出现 need_active_view_* 状态。
  4. 该测试用于证明在线节点在“当前证据还不能安全重定位”时，会给上层状态机明确的主动采集新视角提示。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_online_active_view_status_smoke.py --timeout 180
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def repo_root_from_script() -> Path:
    """根据脚本路径反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def params(config: dict[str, Any]) -> dict[str, Any]:
    """取得 ROS 参数字典。"""
    return config["global_relocalization_eval"]["ros__parameters"]


def status_tokens(status: str) -> dict[str, str]:
    """解析 recovery_status 的 key=value 文本。"""
    parsed: dict[str, str] = {}
    for token in status.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        parsed[key] = value
    return parsed


def terminate_process(process: subprocess.Popen, name: str) -> None:
    """结束 ROS 子进程组，避免 smoke 失败后残留节点或 rosbag play。"""
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        print(f"[online_active_view_smoke] force kill {name}", file=sys.stderr)
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.0)


def write_forced_config(template_path: Path, output_path: Path) -> None:
    """写强制进入 need_active_view 状态的临时 YAML。"""
    config = yaml.safe_load(template_path.read_text(encoding="utf-8"))
    p = params(config)

    # 第一层 selected refine 阈值压到 0，避免普通 state=verified 直接通过。
    # fallback 关闭，确保 trajectory recovery 不会发布 verified_trajectory_single_agreement。
    # strict average overlap 提高到 0.999，让当前合成短窗口只能进入 need_active_view 状态。
    p["temporal_consistency_online_max_refine_fitness"] = 0.0
    p["trajectory_single_agreement_fallback_enable"] = False
    p["trajectory_likelihood_min_average_overlap"] = 0.999
    p["output_dir"] = str(output_path.parent / "global_relocalization_online_active_view_status_smoke")

    header = [
        "# online active-view status smoke 自动生成配置。",
        "#",
        "# 作用：",
        "#   - 由 test/run_online_active_view_status_smoke.py 生成。",
        "#   - 强制在线节点无法发布 verified，验证 recovery_status 是否输出 need_active_view_*。",
        "#   - 该文件位于 .codex_tmp，不作为线上运行配置。",
        "",
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        "\n".join(header) + yaml.safe_dump(config, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def write_status_output(path: Path, status: str) -> None:
    """把收到的主动恢复状态写入 CSV，供后续门禁检查。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    tokens = status_tokens(status)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "state",
                "recovery_hint",
                "trajectory_attempted",
                "trajectory_support",
                "trajectory_average_overlap",
                "trajectory_margin",
                "status",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "state": tokens.get("state", ""),
                "recovery_hint": tokens.get("recovery_hint", ""),
                "trajectory_attempted": tokens.get("trajectory_attempted", ""),
                "trajectory_support": tokens.get("trajectory_support", ""),
                "trajectory_average_overlap": tokens.get("trajectory_average_overlap", ""),
                "trajectory_margin": tokens.get("trajectory_margin", ""),
                "status": status,
            }
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Run online active-view recovery status smoke.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument(
        "--template",
        type=Path,
        default=Path("src/humanoid_global_relocalization_runtime/test/synthetic_body_online_debug.yaml"),
        help="在线 smoke YAML 模板",
    )
    parser.add_argument("--bag", type=Path, default=Path(".codex_tmp/synthetic_body_nav_drift_test43"), help="播放 bag")
    parser.add_argument("--timeout", type=float, default=180.0, help="等待 need_active_view 状态的超时时间")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    template = (workspace / args.template).resolve() if not args.template.is_absolute() else args.template
    bag = (workspace / args.bag).resolve() if not args.bag.is_absolute() else args.bag
    config = workspace / ".codex_tmp/online_active_view_status_smoke.yaml"
    status_output = workspace / ".codex_tmp/online_active_view_status_smoke.csv"
    if not template.exists():
        print(f"[online_active_view_smoke] template not found: {template}", file=sys.stderr)
        return 2
    if not (bag / "metadata.yaml").exists():
        print(f"[online_active_view_smoke] bag metadata not found: {bag}", file=sys.stderr)
        return 2
    write_forced_config(template, config)

    env = os.environ.copy()
    node_cmd = [
        "ros2",
        "run",
        "humanoid_global_relocalization_runtime",
        "global_relocalization_node",
        "--ros-args",
        "-p",
        f"config_file:={config}",
    ]
    play_cmd = ["ros2", "bag", "play", str(bag), "--rate", "1.0"]
    print("[online_active_view_smoke] starting node:", " ".join(node_cmd))
    node_process = subprocess.Popen(node_cmd, env=env, start_new_session=True)
    play_process: subprocess.Popen | None = None
    matched_status: list[str] = []

    try:
        rclpy.init()
        node = rclpy.create_node("global_relocalization_active_view_status_listener")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        def status_callback(msg: String) -> None:
            tokens = status_tokens(msg.data)
            state = tokens.get("state", "")
            if state.startswith("need_active_view_") and tokens.get("recovery_hint") == "active_view":
                matched_status.append(msg.data)
                print(f"[online_active_view_smoke] recovery_status {msg.data}")

        node.create_subscription(String, "/global_relocalization/recovery_status", status_callback, qos)

        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node_process.poll() is not None:
                print("[online_active_view_smoke] node exited before bag play", file=sys.stderr)
                return 3

        print("[online_active_view_smoke] playing bag:", " ".join(play_cmd))
        play_process = subprocess.Popen(play_cmd, env=env, start_new_session=True)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline and not matched_status:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node_process.poll() is not None:
                print("[online_active_view_smoke] node exited while waiting for status", file=sys.stderr)
                return 4

        if not matched_status:
            print("[online_active_view_smoke] timeout waiting for need_active_view status", file=sys.stderr)
            return 5

        write_status_output(status_output, matched_status[-1])
        print(f"[online_active_view_smoke] PASS status_output={status_output}")
        return 0
    finally:
        if play_process is not None:
            terminate_process(play_process, "ros2 bag play")
        terminate_process(node_process, "global_relocalization_node")
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
