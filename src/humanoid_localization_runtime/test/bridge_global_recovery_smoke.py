#!/usr/bin/env python3
"""Verify the bridge's direct map->odom recovery interface."""

import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class Probe(Node):
    def __init__(self):
        super().__init__("bridge_global_recovery_probe")
        self.publisher = self.create_publisher(
            PoseStamped, "/localization/global_recovery_map_to_odom", 10
        )
        self.statuses = []
        self.create_subscription(
            String,
            "/localization/prior_map_odom_bridge_status",
            lambda msg: self.statuses.append(msg.data),
            50,
        )


def spin_until(node, condition, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return
    raise RuntimeError(f"timeout waiting for {label}: {node.statuses[-5:]}")


def main():
    # Keep the smoke test isolated from a navigation stack that may already be
    # running on the workstation with the same node and topic names.
    os.environ["ROS_DOMAIN_ID"] = os.environ.get("TEST_ROS_DOMAIN_ID", "221")
    process = subprocess.Popen(
        [
            "ros2", "run", "humanoid_localization_runtime", "prior_map_odom_bridge_cpp",
            "--ros-args", "-p", "publish_rate:=5.0", "-p", "allow_initial_pose:=false",
        ],
        env=os.environ.copy(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    rclpy.init()
    node = Probe()
    try:
        spin_until(node, lambda: bool(node.statuses), 3.0, "bridge startup")
        wrong = PoseStamped()
        wrong.header.frame_id = "odom"
        wrong.pose.orientation.w = 1.0
        node.publisher.publish(wrong)
        spin_until(
            node,
            lambda: any("global_recovery_wrong_frame" in item for item in node.statuses),
            2.0,
            "wrong-frame rejection",
        )
        accepted = PoseStamped()
        accepted.header.frame_id = "map"
        accepted.pose.position.x = 0.75
        accepted.pose.position.y = -0.25
        accepted.pose.orientation.w = 1.0
        node.publisher.publish(accepted)
        spin_until(
            node,
            lambda: any(item.startswith("ACCEPTED global_recovery") for item in node.statuses),
            2.0,
            "global recovery acceptance",
        )
        print("BRIDGE_GLOBAL_RECOVERY_OK")
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
