#!/usr/bin/env python3
"""Verify supervisor control fields differ between shadow and enforce modes."""

import importlib.util
import json
import os
import pathlib
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


def load_supervisor():
    source = (
        pathlib.Path(__file__).parents[2]
        / "humanoid_navigation2/humanoid_navigation2/localization_trust_supervisor.py"
    )
    spec = importlib.util.spec_from_file_location("localization_trust_supervisor_mode", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.LocalizationTrustSupervisor


class Probe(Node):
    def __init__(self):
        super().__init__("global_relocalization_mode_probe")
        self.bridge_pub = self.create_publisher(
            String, "/localization/prior_map_odom_bridge_status", 10
        )
        self.statuses = []
        self.create_subscription(
            String, "/localization/trust_status",
            lambda msg: self.statuses.append(json.loads(msg.data)), 20,
        )


def spin_until(executor, condition, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if condition():
            return
    raise RuntimeError(f"timeout waiting for {label}")


def main():
    os.environ["ROS_DOMAIN_ID"] = os.environ.get("TEST_ROS_DOMAIN_ID", "223")
    rclpy.init()
    supervisor = load_supervisor()()
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(supervisor)
    executor.add_node(probe)
    hold = String(data="HOLD large_jump reason=test")
    try:
        probe.bridge_pub.publish(hold)
        transient_deadline = time.monotonic() + 0.8
        while time.monotonic() < transient_deadline:
            executor.spin_once(timeout_sec=0.05)
        if any(item.get("recovery_requires_global_relocalization") for item in probe.statuses):
            raise RuntimeError("transient large-jump HOLD triggered global recovery")

        probe.bridge_pub.publish(String(data="DEGRADED large_jump_hold age=3.10s reason=test"))
        spin_until(
            executor,
            lambda: any(item.get("recovery_requires_global_relocalization") for item in probe.statuses),
            2.0,
            "shadow diagnostic flag",
        )
        shadow = probe.statuses[-1]
        if shadow.get("integration_mode") != "shadow":
            raise RuntimeError(f"unexpected default mode: {shadow}")
        if shadow.get("localization_recovery_required") is not False:
            raise RuntimeError("shadow mode exposed an active recovery control request")

        supervisor.integration_mode = "enforce"
        spin_until(
            executor,
            lambda: any(
                item.get("integration_mode") == "enforce"
                and item.get("localization_recovery_required") is True
                for item in probe.statuses
            ),
            2.0,
            "enforce control flag",
        )
        print("GLOBAL_RELOCALIZATION_MODE_CONTRACT_OK")
        return 0
    finally:
        executor.remove_node(probe)
        executor.remove_node(supervisor)
        probe.destroy_node()
        supervisor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
