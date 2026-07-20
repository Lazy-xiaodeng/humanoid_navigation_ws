#!/usr/bin/env python3
"""Verify retry exhaustion and upper-layer stop cancellation."""

import importlib.util
import json
import pathlib
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


def load_coordinator():
    path = pathlib.Path(__file__).parents[1] / "scripts" / "global_relocalization_coordinator.py"
    spec = importlib.util.spec_from_file_location("global_relocalization_coordinator_failure", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.GlobalRelocalizationCoordinator


class Probe(Node):
    def __init__(self):
        super().__init__("global_relocalization_failure_probe")
        self.map_pub = self.create_publisher(String, "/map/status", 10)
        self.trust_pub = self.create_publisher(String, "/localization/trust_status", 10)
        self.nav_pub = self.create_publisher(String, "/navigation/status", 10)
        self.requests = []
        self.events = []
        self.create_subscription(
            String, "/global_relocalization/request",
            lambda msg: self.requests.append(json.loads(msg.data)), 50
        )
        self.create_subscription(
            String, "/localization/recovery_status",
            lambda msg: self.events.append(json.loads(msg.data)), 50
        )

    def publish(self, publisher, payload):
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)


def spin_until(executor, condition, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.03)
        if condition():
            return
    raise RuntimeError(f"timeout waiting for {label}")


def main():
    rclpy.init()
    coordinator = load_coordinator()()
    coordinator.integration_mode = "enforce"
    coordinator.attempt_timeout_sec = 0.25
    coordinator.retry_backoff_sec = 0.05
    coordinator.max_attempts = 2
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(coordinator)
    executor.add_node(probe)
    trust = {
        "state": "recovery_requires_global_relocalization",
        "startup_requires_global_relocalization": False,
        "recovery_requires_global_relocalization": True,
        "pose_trusted": False,
        "can_start_navigation": False,
    }
    try:
        probe.publish(probe.map_pub, {"current_map_id": "hall", "map_state": "ready"})
        probe.publish(probe.trust_pub, trust)
        spin_until(
            executor,
            lambda: any(item.get("event_type") == "localization_relocalize_failed" and
                             item.get("result_code") == "attempts_exhausted"
                        for item in probe.events),
            3.0,
            "retry exhaustion",
        )
        starts = [item for item in probe.requests if item.get("command") == "start"]
        if len(starts) != 2:
            raise RuntimeError(f"expected exactly two attempts, got {len(starts)}")

        probe.publish(probe.nav_pub, {"event_type": "navigation_pending"})
        spin_until(
            executor,
            lambda: len([item for item in probe.requests if item.get("command") == "start"]) == 3,
            2.0,
            "new navigation attempt",
        )
        probe.publish(probe.nav_pub, {"event_type": "navigation_stopped"})
        spin_until(
            executor,
            lambda: any(item.get("reason") == "navigation_stopped_by_upper_layer" for item in probe.events),
            2.0,
            "upper-layer stop",
        )
        start_count = len([item for item in probe.requests if item.get("command") == "start"])
        probe.publish(probe.trust_pub, trust)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)
        if len([item for item in probe.requests if item.get("command") == "start"]) != start_count:
            raise RuntimeError("recovery restarted after upper-layer stop")
        print("GLOBAL_RELOCALIZATION_FAILURE_POLICY_OK")
        return 0
    finally:
        executor.remove_node(probe)
        executor.remove_node(coordinator)
        probe.destroy_node()
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
