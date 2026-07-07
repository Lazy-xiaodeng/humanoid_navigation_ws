#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_online_debug_smoke.py

文件作用：
  1. 这是在线 debug 冒烟测试脚本，不属于功能源码。
  2. 启动 global_relocalization_node，默认加载 synthetic_body_online_debug.yaml。
  3. 默认播放合成 /cloud_registered_body 小 bag，也支持通过 --config/--bag 验证真实 nav_drift registered_world 链路。
  4. 订阅 /global_relocalization/verified_candidate、/global_relocalization/verified_map_to_odom
     和 /global_relocalization/recovery_status，确认在线恢复链路能发布指定 state 的 debug 恢复量。

使用前提：
  - 已经 source /opt/ros/jazzy/setup.bash 和当前工作空间 install/local_setup.bash。
  - 已经存在 .codex_tmp/synthetic_body_nav_drift_test43。

注意：
  - 本脚本只验证 debug 话题，不检查导航闭环。
  - 节点只发布 map->odom 的 PoseStamped 调试话题，不发布 TF，不注入 initialpose。
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

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def terminate_process(process: subprocess.Popen, name: str) -> None:
    """温和结束子进程组；如果超时未退出，再强制 kill，避免测试残留 ROS 进程。"""
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        print(f"[online_smoke] force kill {name}", file=sys.stderr)
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.0)


def yaw_from_pose(msg: PoseStamped) -> float:
    """从 PoseStamped 四元数中计算 yaw，单位度。"""
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    import math
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def write_pose_output(path: Path, verified: PoseStamped, map_odom: PoseStamped, status: str) -> None:
    """把收到的 verified_candidate 和 verified_map_to_odom 写成 CSV，供矩阵脚本汇总。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "verified_x_m",
                "verified_y_m",
                "verified_z_m",
                "map_odom_x_m",
                "map_odom_y_m",
                "map_odom_yaw_deg",
                "status",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "verified_x_m": f"{verified.pose.position.x:.3f}",
                "verified_y_m": f"{verified.pose.position.y:.3f}",
                "verified_z_m": f"{verified.pose.position.z:.3f}",
                "map_odom_x_m": f"{map_odom.pose.position.x:.3f}",
                "map_odom_y_m": f"{map_odom.pose.position.y:.3f}",
                "map_odom_yaw_deg": f"{yaw_from_pose(map_odom):.3f}",
                "status": status,
            }
        )


def status_state(status: str) -> str:
    """从 recovery_status 文本中取出 state=xxx，取不到时返回空字符串。"""
    for token in status.split():
        if token.startswith("state="):
            return token.split("=", 1)[1]
    return ""


def main() -> int:
    parser = argparse.ArgumentParser(description="Run online global relocalization smoke test with synthetic body bag.")
    parser.add_argument(
        "--config",
        default="src/humanoid_global_relocalization_runtime/test/synthetic_body_online_debug.yaml",
        help="在线 debug YAML 配置路径",
    )
    parser.add_argument(
        "--bag",
        default=".codex_tmp/synthetic_body_nav_drift_test43",
        help="合成 body rosbag2 目录",
    )
    parser.add_argument("--timeout", type=float, default=120.0, help="等待 verified debug 输出的最长秒数")
    parser.add_argument(
        "--pose-output",
        default="",
        help="可选：把收到的 verified_candidate、verified_map_to_odom 和 status 写入该 CSV",
    )
    parser.add_argument(
        "--expected-state",
        default="verified",
        help="期望收到的 recovery_status state；默认要求普通 verified，也可指定 verified_trajectory_single_agreement",
    )
    args = parser.parse_args()

    config = Path(args.config).resolve()
    bag = Path(args.bag).resolve()
    if not config.exists():
        print(f"[online_smoke] config not found: {config}", file=sys.stderr)
        return 2
    if not (bag / "metadata.yaml").exists():
        print(f"[online_smoke] bag metadata not found: {bag}", file=sys.stderr)
        return 2

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
    play_cmd = [
        "ros2",
        "bag",
        "play",
        str(bag),
        "--rate",
        "1.0",
    ]

    print("[online_smoke] starting node:", " ".join(node_cmd))
    node_process = subprocess.Popen(node_cmd, env=env, start_new_session=True)
    play_process: subprocess.Popen | None = None

    received_verified: list[PoseStamped] = []
    received_map_odom: list[PoseStamped] = []
    received_status: list[str] = []

    try:
        rclpy.init()
        node = rclpy.create_node("global_relocalization_online_smoke_listener")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        def verified_callback(msg: PoseStamped) -> None:
            received_verified.append(msg)
            print(
                "[online_smoke] verified_candidate "
                f"x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f} z={msg.pose.position.z:.3f}"
            )

        def map_odom_callback(msg: PoseStamped) -> None:
            received_map_odom.append(msg)
            print(
                "[online_smoke] verified_map_to_odom "
                f"x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f} yaw={yaw_from_pose(msg):.3f}"
            )

        def status_callback(msg: String) -> None:
            received_status.append(msg.data)
            if status_state(msg.data) == args.expected_state:
                print(f"[online_smoke] recovery_status {msg.data}")

        node.create_subscription(PoseStamped, "/global_relocalization/verified_candidate", verified_callback, qos)
        node.create_subscription(PoseStamped, "/global_relocalization/verified_map_to_odom", map_odom_callback, qos)
        node.create_subscription(String, "/global_relocalization/recovery_status", status_callback, qos)

        # 给在线节点一点时间构建地图索引和创建订阅，再开始播放 bag。
        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node_process.poll() is not None:
                print("[online_smoke] node exited before bag play", file=sys.stderr)
                return 3

        print("[online_smoke] playing bag:", " ".join(play_cmd))
        play_process = subprocess.Popen(play_cmd, env=env, start_new_session=True)

        deadline = time.monotonic() + args.timeout
        def has_verified_outputs() -> bool:
            return bool(received_verified) and bool(received_map_odom) and any(
                status_state(item) == args.expected_state for item in received_status
            )

        while time.monotonic() < deadline and not has_verified_outputs():
            rclpy.spin_once(node, timeout_sec=0.2)
            if play_process.poll() is not None and time.monotonic() > deadline - args.timeout + 5.0:
                # bag 已经播完但还没收到 verified，继续短暂等待 transient/local 或后台计算。
                pass
            if node_process.poll() is not None:
                print("[online_smoke] node exited while waiting for verified_candidate", file=sys.stderr)
                return 4

        if not has_verified_outputs():
            print("[online_smoke] verified debug outputs were not all received before timeout", file=sys.stderr)
            return 5

        if args.pose_output:
            verified_status = next(item for item in reversed(received_status) if status_state(item) == args.expected_state)
            write_pose_output(Path(args.pose_output), received_verified[-1], received_map_odom[-1], verified_status)
        print(
            "[online_smoke] PASS "
            f"verified={len(received_verified)} map_odom={len(received_map_odom)} status={len(received_status)}"
        )
        return 0
    finally:
        if play_process is not None:
            terminate_process(play_process, "bag play")
        terminate_process(node_process, "global_relocalization_node")
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
