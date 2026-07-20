#!/usr/bin/env python3
"""验证 large-jump、RO LOST 和状态陈旧的防抖与恢复锁存合同。"""

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
    spec = importlib.util.spec_from_file_location("localization_trust_supervisor_faults", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.LocalizationTrustSupervisor


class Probe(Node):
    def __init__(self):
        super().__init__("global_relocalization_fault_debounce_probe")
        self.bridge_pub = self.create_publisher(
            String, "/localization/prior_map_odom_bridge_status", 10
        )
        self.ro_pub = self.create_publisher(
            String, "/prior_localization/robosense_status", 10
        )
        self.statuses = []
        self.create_subscription(
            String, "/localization/trust_status",
            lambda msg: self.statuses.append(json.loads(msg.data)), 20,
        )

    def publish_ro(self, status: str, stamp: float, source: str = "ro_normal_match"):
        self.ro_pub.publish(String(data=json.dumps({
            "status": status,
            "source": source,
            "stamp": stamp,
        })))


def spin_for(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.03)


def latest_control(probe):
    return bool(probe.statuses and probe.statuses[-1].get("localization_recovery_required"))


def main():
    os.environ["ROS_DOMAIN_ID"] = os.environ.get("TEST_ROS_DOMAIN_ID", "82")
    rclpy.init()
    supervisor = load_supervisor()()
    supervisor.integration_mode = "enforce"
    supervisor.bridge_status_timeout_sec = 30.0
    supervisor.robosense_status_timeout_sec = 0.35
    supervisor.ro_stale_recovery_sec = 0.35
    supervisor.ro_unhealthy_min_duration_sec = 0.25
    supervisor.ro_unhealthy_min_updates = 3
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(supervisor)
    executor.add_node(probe)
    try:
        probe.bridge_pub.publish(String(data="ACCEPTED small_correction map_odom_xy_norm=1.0"))
        spin_for(executor, 0.15)
        for stamp in range(1, 7):
            probe.publish_ro("NORMAL", float(stamp))
            spin_for(executor, 0.05)
        if not supervisor.ever_trusted:
            raise RuntimeError("failed to establish trusted localization precondition")

        probe.bridge_pub.publish(String(data="HOLD large_jump age=0.10s reason=test"))
        spin_for(executor, 0.15)
        if latest_control(probe):
            raise RuntimeError("single large-jump HOLD enabled recovery control")
        probe.bridge_pub.publish(String(data="ACCEPTED small_correction map_odom_xy_norm=1.0"))

        probe.publish_ro("LOST", 10.0, "ro_lost")
        spin_for(executor, 0.08)
        probe.publish_ro("LOST", 11.0, "ro_lost")
        spin_for(executor, 0.08)
        probe.publish_ro("NORMAL", 12.0)
        spin_for(executor, 0.2)
        if latest_control(probe):
            raise RuntimeError("transient two-frame LOST enabled recovery control")

        for stamp in (20.0, 21.0, 22.0, 23.0):
            probe.publish_ro("LOST", stamp, "ro_lost")
            spin_for(executor, 0.10)
        if not latest_control(probe):
            raise RuntimeError("sustained LOST did not enable recovery control")

        probe.bridge_pub.publish(String(data="ACCEPTED global_recovery map_odom_xy_norm=1.0"))
        for stamp in range(30, 36):
            probe.publish_ro("NORMAL", float(stamp))
            spin_for(executor, 0.05)
        if supervisor.recovery_requires_global_relocalization:
            raise RuntimeError("bridge-confirmed recovery did not clear recovery control")

        spin_for(executor, 0.75)
        if not latest_control(probe):
            raise RuntimeError("sustained RO status stale did not enable recovery control")

        print("GLOBAL_RELOCALIZATION_FAULT_DEBOUNCE_OK")
        return 0
    finally:
        executor.remove_node(probe)
        executor.remove_node(supervisor)
        probe.destroy_node()
        supervisor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
