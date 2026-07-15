#!/usr/bin/env python3
"""Verify on-demand map/SC/SOLiD loading and failed-switch rollback."""

import json
import os
import pathlib
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Probe(Node):
    def __init__(self):
        super().__init__("global_relocalization_map_asset_probe")
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(String, "/global_relocalization/request", 10)
        self.statuses = []
        self.create_subscription(
            String, "/global_relocalization/recovery_status",
            lambda msg: self.statuses.append(json.loads(msg.data)), qos
        )

    def request(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)


def spin_until(node, condition, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return
    raise RuntimeError(f"timeout waiting for {label}: {node.statuses[-3:]}")


def main():
    workspace = pathlib.Path(__file__).parents[3]
    config = workspace / "src/humanoid_global_relocalization_runtime/config/relocalization_runtime.yaml"
    assets = workspace / "data/maps/hall_global_relocalization"
    map_path = workspace / "src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"
    process = subprocess.Popen(
        ["ros2", "run", "humanoid_global_relocalization_runtime", "global_relocalization_node",
         "--ros-args", "-p", f"config_file:={config}"],
        env=os.environ.copy(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    rclpy.init()
    node = Probe()
    try:
        spin_until(
            node,
            lambda: any(item.get("state") == "idle" and item.get("reason") == "node_ready"
                        for item in node.statuses),
            20.0,
            "on-demand node ready",
        )
        valid = {
            "command": "start",
            "attempt_id": "asset-hall-1",
            "map_id": "hall",
            "map_path": str(map_path),
            "scan_context_database_path": str(assets / "scan_context.csv"),
            "solid_database_path": str(assets / "solid.csv"),
        }
        node.request(valid)
        spin_until(
            node,
            lambda: any(
                item.get("attempt_id") == "asset-hall-1"
                and item.get("state") == "searching"
                and item.get("scan_context_entries", 0) > 0
                and item.get("solid_entries", 0) > 0
                and not item.get("degraded_recall", True)
                for item in node.statuses
            ),
            30.0,
            "hall assets loaded",
        )

        invalid = dict(valid)
        invalid.update(
            {
                "attempt_id": "asset-bad-1",
                "map_id": "missing_map",
                "map_path": str(workspace / "data/maps/does_not_exist.pcd"),
                "scan_context_database_path": "",
                "solid_database_path": "",
            }
        )
        node.request(invalid)
        spin_until(
            node,
            lambda: any(
                item.get("attempt_id") == "asset-bad-1"
                and item.get("state") == "rejected"
                and item.get("reason") == "map_assets_load_failed"
                for item in node.statuses
            ),
            30.0,
            "failed map rollback",
        )
        valid["attempt_id"] = "asset-hall-2"
        node.request(valid)
        spin_until(
            node,
            lambda: any(item.get("attempt_id") == "asset-hall-2" and
                             item.get("state") == "searching" and
                             item.get("map_id") == "hall"
                        for item in node.statuses),
            10.0,
            "old map remains usable",
        )
        print("GLOBAL_RELOCALIZATION_MAP_ASSET_LIFECYCLE_OK")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()


if __name__ == "__main__":
    sys.exit(main())
