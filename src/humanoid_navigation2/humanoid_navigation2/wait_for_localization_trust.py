#!/usr/bin/env python3
"""等待定位可信后退出，供 launch 串起 Nav2 启动顺序。"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WaitForLocalizationTrust(Node):
    def __init__(self):
        super().__init__("wait_for_localization_trust")
        self.trust_status_topic = self.declare_parameter(
            "trust_status_topic", "/localization/trust_status"
        ).value
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 0.0).value)
        self.poll_log_period_sec = float(
            self.declare_parameter("poll_log_period_sec", 3.0).value
        )
        self.start_time = time.time()
        self.last_log_time = 0.0
        self.latest_state = "waiting"
        self.latest_reason = ""

        self.create_subscription(String, self.trust_status_topic, self.on_status, 10)
        self.create_timer(0.2, self.on_timer)
        self.get_logger().info(f"waiting for trusted localization on {self.trust_status_topic}")

    def on_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except json.JSONDecodeError:
            return

        self.latest_state = str(payload.get("state", "unknown") or "unknown")
        self.latest_reason = str(payload.get("reason", "") or "")
        if bool(payload.get("can_start_navigation", False)) and bool(payload.get("pose_trusted", False)):
            self.get_logger().info(
                f"trusted localization ready: {self.latest_state} {self.latest_reason}".strip()
            )
            rclpy.shutdown()

    def on_timer(self):
        now = time.time()
        if self.timeout_sec > 0.0 and now - self.start_time > self.timeout_sec:
            self.get_logger().error(
                f"timeout waiting for trusted localization: {self.latest_state} {self.latest_reason}".strip()
            )
            rclpy.shutdown()
            sys.exit(1)
            return

        if now - self.last_log_time >= self.poll_log_period_sec:
            self.last_log_time = now
            self.get_logger().info(
                f"localization not trusted yet: {self.latest_state} {self.latest_reason}".strip()
            )


def main(args=None):
    rclpy.init(args=args)
    node = WaitForLocalizationTrust()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
