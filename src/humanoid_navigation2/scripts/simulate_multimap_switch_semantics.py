#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多地图切换语义模拟器。

本脚本不启动真实 Nav2、雷达或定位节点，而是：
1. 临时生成两张假地图的 registry 和空地图文件。
2. 用假的 switch_navigation_map.sh 代替真实导航层重启脚本。
3. 启动真实 map_context_manager 节点。
4. 模拟 APP 发送 switch_map，模拟定位健康状态变为 ACCEPTED。
5. 验证 map_context_manager 会推送 restart_started、发布初始位姿，并最终推送 map_ready。

它用于验证“控制层常驻 + 导航层重启”的业务状态机，不替代实机地图切换测试。
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String


def write_fake_map_files(root: Path, map_id: str) -> Dict[str, str]:
    """创建 map_context_manager 只需要 exists 校验的假 2D/3D 地图文件。"""
    maps_dir = root / "maps"
    pcd_dir = root / "pcd"
    maps_dir.mkdir(parents=True, exist_ok=True)
    pcd_dir.mkdir(parents=True, exist_ok=True)

    yaml_file = maps_dir / f"{map_id}.yaml"
    pgm_file = maps_dir / f"{map_id}.pgm"
    pcd_file = pcd_dir / f"{map_id}_open3d_grounded.pcd"

    pgm_file.write_text("P2\n1 1\n255\n0\n", encoding="utf-8")
    yaml_file.write_text(
        f"image: {pgm_file}\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    pcd_file.write_text(
        "# .PCD v0.7 - Point Cloud Data file format\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0 0 0\n",
        encoding="utf-8",
    )
    return {
        "map_yaml_file": str(yaml_file),
        "map_pgm_file": str(pgm_file),
        "open3d_prior_map_file": str(pcd_file),
    }


def write_registry(root: Path) -> Path:
    """写入两张假地图，当前地图为 hall，目标地图为 hall2。"""
    hall_paths = write_fake_map_files(root, "hall")
    hall2_paths = write_fake_map_files(root, "hall2")
    registry = {
        "default_map_id": "hall",
        "current_map_id": "hall",
        "maps": [
            {
                "map_id": "hall",
                "display_name": "测试地图 hall",
                "enabled": True,
                "initial_pose": {
                    "frame_id": "map",
                    "position": [0.0, 0.0, 0.0],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                },
                **hall_paths,
            },
            {
                "map_id": "hall2",
                "display_name": "测试地图 hall2",
                "enabled": True,
                "initial_pose": {
                    "frame_id": "map",
                    "position": [1.0, 2.0, 0.0],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                },
                **hall2_paths,
            },
        ],
    }
    registry_path = root / "map_registry.json"
    registry_path.write_text(json.dumps(registry, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return registry_path


def write_fake_switch_script(root: Path) -> Path:
    """创建假的导航层重启脚本，只记录调用参数并成功退出。"""
    script = root / "fake_switch_navigation_map.sh"
    marker = root / "switch_called.json"
    script.write_text(
        "#!/bin/bash\n"
        "set -eo pipefail\n"
        "python3 - \"$1\" \"$MAP_ID\" \"$WORKSPACE\" <<'PY'\n"
        "import json, sys\n"
        f"from pathlib import Path\nmarker = Path({str(marker)!r})\n"
        "marker.write_text(json.dumps({'arg_map_id': sys.argv[1], 'env_map_id': sys.argv[2], 'workspace': sys.argv[3]}, ensure_ascii=False), encoding='utf-8')\n"
        "PY\n"
        "sleep 0.2\n",
        encoding="utf-8",
    )
    script.chmod(0o755)
    return script


class MultiMapSwitchSim(Node):
    """模拟 APP 和定位状态，收集 map_context_manager 输出。"""

    def __init__(self):
        super().__init__("multimap_switch_semantic_sim")
        self.command_pub = self.create_publisher(String, "/app/map_command", 10)
        self.localization_pub = self.create_publisher(String, "/localization/prior_map_odom_bridge_status", 10)
        self.response_sub = self.create_subscription(String, "/map/response", self.on_response, 100)
        self.status_sub = self.create_subscription(String, "/map/status", self.on_status, 100)
        self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 10)
        self.responses: List[Dict[str, Any]] = []
        self.statuses: List[Dict[str, Any]] = []
        self.initialposes: List[PoseWithCovarianceStamped] = []
        self.create_timer(0.05, self.publish_localization_health)

    def publish_localization_health(self):
        """持续发布健康定位状态，让 map_context_manager 能完成 ready 判定。"""
        self.localization_pub.publish(String(data=json.dumps({
            "state": "ACCEPTED",
            "healthy": True,
            "source": "multimap_switch_semantic_sim",
        }, ensure_ascii=False)))

    def on_response(self, msg: String):
        try:
            self.responses.append(json.loads(msg.data))
        except Exception:
            pass

    def on_status(self, msg: String):
        try:
            self.statuses.append(json.loads(msg.data))
        except Exception:
            pass

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        self.initialposes.append(msg)

    def send_switch_map(self, target_map_id: str = "hall2"):
        """模拟 APP 下发 switch_map 命令。"""
        payload = {
            "command_type": "switch_map",
            "request_message_id": "sim_switch_001",
            "target_map_id": target_map_id,
        }
        self.command_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def start_map_context_manager(registry_path: Path, switch_script: Path, timeout_sec: float) -> subprocess.Popen:
    """启动真实 map_context_manager，参数指向临时 registry 和假切图脚本。"""
    command = [
        "ros2",
        "run",
        "humanoid_navigation",
        "map_context_manager",
        "--ros-args",
        "-p",
        f"map_registry_path:={registry_path}",
        "-p",
        "default_map_id:=hall",
        "-p",
        f"map_switch_script:={switch_script}",
        "-p",
        f"switch_localization_timeout_sec:={timeout_sec}",
        "-p",
        "switch_localization_stable_frames:=2",
        "-p",
        "initialpose_repeat_count:=1",
        "-p",
        "initialpose_repeat_interval_sec:=0.1",
    ]
    return subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def spin_until(node: MultiMapSwitchSim, predicate, timeout_sec: float, description: str) -> None:
    """在超时前 spin，直到 predicate 成立。"""
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise RuntimeError(f"等待超时: {description}")


def response_reasons(node: MultiMapSwitchSim) -> List[str]:
    """提取 map_response 里的 result_reason，方便断言。"""
    reasons = []
    for payload in node.responses:
        data = payload.get("data", {}) if isinstance(payload, dict) else {}
        reason = data.get("result_reason")
        if reason:
            reasons.append(str(reason))
    return reasons


def run_simulation(args: argparse.Namespace) -> int:
    with tempfile.TemporaryDirectory(prefix="multimap_switch_sim_") as tmp:
        root = Path(tmp)
        registry_path = write_registry(root)
        switch_script = write_fake_switch_script(root)
        switch_marker = root / "switch_called.json"

        process = start_map_context_manager(registry_path, switch_script, args.timeout)
        try:
            rclpy.init(args=None)
            node = MultiMapSwitchSim()
            try:
                # 等待节点启动并发布初始 map_status。
                spin_until(node, lambda: bool(node.statuses), args.timeout, "map_context_manager 初始 map_status")
                node.send_switch_map("hall2")
                spin_until(
                    node,
                    lambda: "map_switch_restart_started" in response_reasons(node),
                    args.timeout,
                    "收到 map_switch_restart_started",
                )
                spin_until(node, lambda: switch_marker.exists(), args.timeout, "假切图脚本被调用")
                spin_until(node, lambda: bool(node.initialposes), args.timeout, "收到 /initialpose")
                spin_until(
                    node,
                    lambda: "map_ready" in response_reasons(node),
                    args.timeout,
                    "收到 map_ready",
                )
                spin_until(
                    node,
                    lambda: bool(node.statuses)
                    and node.statuses[-1].get("data", {}).get("current_map_id") == "hall2"
                    and node.statuses[-1].get("data", {}).get("map_state") == "ready",
                    args.timeout,
                    "收到 ready 状态的 map_status",
                )

                registry = json.loads(registry_path.read_text(encoding="utf-8"))
                switch_call = json.loads(switch_marker.read_text(encoding="utf-8"))
                latest_status = node.statuses[-1].get("data", {})

                failures = []
                if registry.get("current_map_id") != "hall2":
                    failures.append(f"registry current_map_id 未切到 hall2: {registry.get('current_map_id')}")
                if switch_call.get("arg_map_id") != "hall2" or switch_call.get("env_map_id") != "hall2":
                    failures.append(f"假切图脚本收到的 map_id 不正确: {switch_call}")
                if latest_status.get("current_map_id") != "hall2":
                    failures.append(f"最新 map_status current_map_id 不正确: {latest_status}")
                if latest_status.get("map_state") != "ready":
                    failures.append(f"最新 map_status map_state 不为 ready: {latest_status}")
                if failures:
                    print("多地图切换语义模拟失败：")
                    for failure in failures:
                        print(f"- {failure}")
                    return 1

                print("多地图切换语义模拟通过：switch_map -> 导航层重启脚本 -> initialpose -> map_ready 链路符合预期。")
                return 0
            finally:
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=3.0)
            if args.verbose and process.stdout is not None:
                output = process.stdout.read()
                if output:
                    print("--- map_context_manager output ---")
                    print(output)


def main() -> int:
    parser = argparse.ArgumentParser(description="模拟多地图 switch_map 状态机")
    parser.add_argument("--timeout", type=float, default=8.0, help="每个等待步骤的超时时间")
    parser.add_argument("--verbose", action="store_true", help="输出 map_context_manager 日志")
    args = parser.parse_args()
    return run_simulation(args)


if __name__ == "__main__":
    sys.exit(main())
