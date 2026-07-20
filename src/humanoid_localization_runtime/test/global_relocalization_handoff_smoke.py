#!/usr/bin/env python3
"""Exercise candidate application through bridge acceptance and fresh RO handoff."""

import importlib.util
import json
import os
import pathlib
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def load_class(path, module_name, class_name):
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, class_name)


class Probe(Node):
    def __init__(self):
        super().__init__("global_relocalization_handoff_probe")
        latched = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(String, "/map/status", 10)
        self.bridge_status_pub = self.create_publisher(
            String, "/localization/prior_map_odom_bridge_status", 10
        )
        self.ro_status_pub = self.create_publisher(String, "/prior_localization/robosense_status", 10)
        self.ro_refined_pub = self.create_publisher(
            PoseStamped,
            "/prior_localization/global_relocalization_refined_map_to_odom",
            10,
        )
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
        self.trust = []
        self.ro_initialposes = []
        self.create_subscription(String, "/global_relocalization/request", self._request, 20)
        self.create_subscription(String, "/localization/recovery_status", self._event, 50)
        self.create_subscription(String, "/localization/trust_status", self._trust, 50)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/prior_localization/global_relocalization_pose",
            lambda msg: self.ro_initialposes.append(msg),
            10,
        )

    def publish_json(self, publisher, payload):
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)

    def _request(self, msg):
        self.requests.append(json.loads(msg.data))

    def _event(self, msg):
        self.events.append(json.loads(msg.data))

    def _trust(self, msg):
        self.trust.append(json.loads(msg.data))


def spin_until(executor, condition, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.03)
        if condition():
            return
    raise RuntimeError(f"timeout waiting for {label}")


def main():
    # Keep the smoke test isolated from a navigation stack that may already be
    # running on the workstation with the same node and topic names.
    os.environ["ROS_DOMAIN_ID"] = os.environ.get("TEST_ROS_DOMAIN_ID", "221")
    source_root = pathlib.Path(__file__).parents[2]
    coordinator_class = load_class(
        pathlib.Path(__file__).parents[1] / "scripts" / "global_relocalization_coordinator.py",
        "global_relocalization_coordinator_handoff",
        "GlobalRelocalizationCoordinator",
    )
    supervisor_class = load_class(
        source_root / "humanoid_navigation2" / "humanoid_navigation2" / "localization_trust_supervisor.py",
        "localization_trust_supervisor_handoff",
        "LocalizationTrustSupervisor",
    )
    bridge = subprocess.Popen(
        ["ros2", "run", "humanoid_localization_runtime", "prior_map_odom_bridge_cpp",
         "--ros-args", "-p", "publish_rate:=10.0"],
        env=os.environ.copy(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    rclpy.init()
    coordinator = coordinator_class()
    coordinator.integration_mode = "enforce"
    coordinator.dry_run = False
    coordinator.auto_apply = True
    supervisor = supervisor_class()
    supervisor.integration_mode = "enforce"
    probe = Probe()
    executor = SingleThreadedExecutor()
    for node in (coordinator, supervisor, probe):
        executor.add_node(node)
    try:
        probe.publish_json(probe.map_pub, {"current_map_id": "hall", "map_state": "ready"})
        probe.publish_json(
            probe.ro_status_pub,
            {"status": "NORMAL", "source": "ro_normal_match", "event": "pose_update"},
        )
        bridge_seed = String()
        bridge_seed.data = (
            "ACCEPTED initial_pose origin_seeded=true origin_seed_radius=0.300 "
            "map_odom_xy_norm=0.000 yaw=0.000"
        )
        probe.bridge_status_pub.publish(bridge_seed)
        spin_until(
            executor,
            lambda: any(item.get("command") == "start" for item in probe.requests),
            4.0,
            "cold-start recovery request",
        )
        start = next(item for item in probe.requests if item.get("command") == "start")
        if start.get("recovery_type") != "cold_start":
            raise RuntimeError(f"wrong recovery type: {start}")

        stamp = coordinator.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = 1.0
        pose.pose.orientation.w = 1.0
        map_odom = PoseStamped()
        map_odom.header = pose.header
        map_odom.pose.position.x = 0.8
        map_odom.pose.orientation.w = 1.0
        probe.pose_pub.publish(pose)
        probe.map_odom_pub.publish(map_odom)
        probe.publish_json(
            probe.global_status_pub,
            {
                "protocol_version": 1,
                "state": "accepted",
                "attempt_id": start["attempt_id"],
                "map_id": "hall",
                "stamp_sec": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            },
        )
        spin_until(executor, lambda: bool(probe.ro_initialposes), 3.0, "RO recovery pose")
        refined_stamp = coordinator.get_clock().now().to_msg()
        refined_map_odom = PoseStamped()
        refined_map_odom.header.stamp = refined_stamp
        refined_map_odom.header.frame_id = "map"
        refined_map_odom.pose.position.x = 0.78
        refined_map_odom.pose.orientation.w = 1.0
        probe.ro_refined_pub.publish(refined_map_odom)
        probe.publish_json(
            probe.ro_status_pub,
            {
                "status": "NORMAL",
                "source": "ro_trusted_global_anchor_commit",
                "event": "global_relocalization_refined_commit",
                "anchor_stamp": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
                "stamp": float(refined_stamp.sec) + float(refined_stamp.nanosec) * 1e-9,
            },
        )
        spin_until(
            executor,
            lambda: any(item.get("state") == "verifying" for item in probe.events),
            3.0,
            "bridge acceptance",
        )
        if any(item.get("pose_trusted") for item in probe.trust[-3:]):
            raise RuntimeError("localization trusted before fresh RO verification")

        base_stamp = float(refined_stamp.sec) + float(refined_stamp.nanosec) * 1e-9
        for index in range(5):
            probe.publish_json(
                probe.ro_status_pub,
                {
                    "status": "NORMAL",
                    "source": "ro_normal_match",
                    "event": "pose_update",
                    "stamp": base_stamp + 0.1 * (index + 1),
                },
            )
            executor.spin_once(timeout_sec=0.08)
        spin_until(
            executor,
            lambda: any(item.get("pose_trusted") and item.get("can_start_navigation") for item in probe.trust),
            3.0,
            "fresh RO trust",
        )
        spin_until(
            executor,
            lambda: any(item.get("event_type") == "localization_recovered" for item in probe.events),
            3.0,
            "coordinator completion",
        )
        print("GLOBAL_RELOCALIZATION_HANDOFF_OK")
        return 0
    finally:
        for node in (probe, supervisor, coordinator):
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()
        bridge.terminate()
        try:
            bridge.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            bridge.kill()


if __name__ == "__main__":
    sys.exit(main())
