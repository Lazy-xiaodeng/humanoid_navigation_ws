#!/usr/bin/env python3
"""Synthetic contract test for the global relocalization coordinator."""

import importlib.util
import json
import pathlib
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def load_coordinator_class():
    source = pathlib.Path(__file__).parents[1] / "scripts" / "global_relocalization_coordinator.py"
    spec = importlib.util.spec_from_file_location("global_relocalization_coordinator", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.GlobalRelocalizationCoordinator


class Probe(Node):
    def __init__(self):
        super().__init__("global_relocalization_coordinator_probe")
        latched = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(String, "/map/status", 10)
        self.trust_pub = self.create_publisher(String, "/localization/trust_status", 10)
        self.global_status_pub = self.create_publisher(
            String, "/global_relocalization/recovery_status", latched
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, "/global_relocalization/recovery_pose", latched
        )
        self.map_odom_pub = self.create_publisher(
            PoseStamped, "/global_relocalization/recovery_map_to_odom", latched
        )
        self.requests = []
        self.events = []
        self.bridge_applies = []
        self.ro_applies = []
        self.create_subscription(String, "/global_relocalization/request", self._request, 20)
        self.create_subscription(String, "/localization/recovery_status", self._event, 50)
        self.create_subscription(
            PoseStamped, "/localization/global_recovery_map_to_odom",
            lambda msg: self.bridge_applies.append(msg), 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/prior_localization/global_relocalization_pose",
            lambda msg: self.ro_applies.append(msg), 10
        )

    def _request(self, msg):
        self.requests.append(json.loads(msg.data))

    def _event(self, msg):
        self.events.append(json.loads(msg.data))

    def publish_json(self, publisher, payload):
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)


def spin_until(executor, condition, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if condition():
            return
    raise RuntimeError(f"timeout waiting for {label}")


def main():
    rclpy.init()
    coordinator = load_coordinator_class()()
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(coordinator)
    executor.add_node(probe)
    try:
        probe.publish_json(
            probe.map_pub,
            {"current_map_id": "hall", "map_state": "ready"},
        )
        probe.publish_json(
            probe.trust_pub,
            {
                "state": "trusted",
                "startup_requires_global_relocalization": False,
                "recovery_requires_global_relocalization": False,
                "pose_trusted": True,
                "can_start_navigation": True,
            },
        )
        healthy_deadline = time.monotonic() + 0.8
        while time.monotonic() < healthy_deadline:
            executor.spin_once(timeout_sec=0.05)
        if probe.requests:
            raise RuntimeError("healthy localization unexpectedly triggered global recovery")

        probe.publish_json(
            probe.trust_pub,
            {
                "state": "startup_requires_global_relocalization",
                "startup_requires_global_relocalization": True,
                "recovery_requires_global_relocalization": False,
                "pose_trusted": False,
                "can_start_navigation": False,
            },
        )
        spin_until(
            executor,
            lambda: any(item.get("command") == "start" for item in probe.requests),
            3.0,
            "start request",
        )
        start = next(item for item in probe.requests if item.get("command") == "start")
        attempt_id = start["attempt_id"]
        if not attempt_id or start.get("map_id") != "hall":
            raise RuntimeError(f"invalid request contract: {start}")

        stamp = coordinator.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.orientation.w = 1.0
        map_odom = PoseStamped()
        map_odom.header = pose.header
        map_odom.pose.orientation.w = 1.0
        probe.pose_pub.publish(pose)
        probe.map_odom_pub.publish(map_odom)
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        wrong = String()
        wrong.data = json.dumps(
            {
                "protocol_version": 1,
                "state": "accepted",
                "attempt_id": attempt_id,
                "map_id": "wrong_map",
                "stamp_sec": stamp_sec,
            }
        )
        probe.global_status_pub.publish(wrong)
        spin_until(
            executor,
            lambda: any(item.get("result_code") == "map_id_mismatch" for item in probe.events),
            2.0,
            "wrong map rejection",
        )

        accepted = String()
        accepted.data = json.dumps(
            {
                "protocol_version": 1,
                "state": "accepted_trajectory",
                "attempt_id": attempt_id,
                "map_id": "hall",
                "stamp_sec": stamp_sec,
            }
        )
        probe.global_status_pub.publish(accepted)
        spin_until(
            executor,
            lambda: any(item.get("result_code") == "dry_run" for item in probe.events),
            2.0,
            "dry-run acceptance",
        )
        if probe.bridge_applies or probe.ro_applies:
            raise RuntimeError("dry-run unexpectedly published a localization application")

        coordinator.state = "idle"
        coordinator.attempt_id = ""
        coordinator.dry_run = False
        coordinator.auto_apply = True
        probe.publish_json(probe.trust_pub, coordinator.latest_trust)
        spin_until(
            executor,
            lambda: len([item for item in probe.requests if item.get("command") == "start"]) >= 2,
            2.0,
            "application attempt",
        )
        second_start = [item for item in probe.requests if item.get("command") == "start"][-1]
        second_stamp = coordinator.get_clock().now().to_msg()
        second_pose = PoseStamped()
        second_pose.header.stamp = second_stamp
        second_pose.header.frame_id = "map"
        second_pose.pose.position.x = 1.25
        second_pose.pose.orientation.w = 1.0
        second_map_odom = PoseStamped()
        second_map_odom.header = second_pose.header
        second_map_odom.pose.position.x = 0.75
        second_map_odom.pose.orientation.w = 1.0
        probe.pose_pub.publish(second_pose)
        probe.map_odom_pub.publish(second_map_odom)
        second_status = String()
        second_status.data = json.dumps(
            {
                "protocol_version": 1,
                "state": "accepted",
                "attempt_id": second_start["attempt_id"],
                "map_id": "hall",
                "stamp_sec": float(second_stamp.sec) + float(second_stamp.nanosec) * 1e-9,
            }
        )
        probe.global_status_pub.publish(second_status)
        spin_until(
            executor,
            lambda: bool(probe.bridge_applies and probe.ro_applies),
            2.0,
            "explicit pose application topics",
        )
        if abs(probe.bridge_applies[-1].pose.position.x - 0.75) > 1e-6:
            raise RuntimeError("bridge did not receive direct map->odom")
        if abs(probe.ro_applies[-1].pose.pose.position.x - 1.25) > 1e-6:
            raise RuntimeError("RO did not receive map->base recovery pose")
        print("COORDINATOR_DRY_RUN_OK")
        print("COORDINATOR_APPLICATION_TOPICS_OK")
        print("COORDINATOR_HEALTHY_NO_TRIGGER_OK")
        print("ATTEMPT_ID", attempt_id)
        print("EVENTS", len(probe.events))
        return 0
    finally:
        executor.remove_node(probe)
        executor.remove_node(coordinator)
        probe.destroy_node()
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
