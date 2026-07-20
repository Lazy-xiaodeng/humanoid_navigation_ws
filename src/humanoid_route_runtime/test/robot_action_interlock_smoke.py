#!/usr/bin/env python3
"""Verify active route pause/resume ownership for the robot action interlock."""

import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class RobotActionInterlockSmoke(Node):
    def __init__(self):
        super().__init__("robot_action_interlock_smoke")
        self.command_pub = self.create_publisher(String, "/navigation/requests", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.robot_pub = self.create_publisher(String, "/robot_status_raw", 10)
        self.localization_pub = self.create_publisher(String, "/localization/trust_status", 10)
        self.map_pub = self.create_publisher(String, "/map/status", 10)
        self.create_subscription(String, "/navigation/status", self.on_status, 100)
        self.interlock_client = self.create_client(SetBool, "/navigation/robot_action_interlock")
        self.events = []
        self.create_timer(0.05, self.publish_inputs)

    def publish_inputs(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        self.robot_pub.publish(String(data=json.dumps({
            "values": {
                "robot_status": "Walk",
                "motion_busy": False,
                "control_ready_for_navigation": True,
            }
        })))
        self.localization_pub.publish(String(data=json.dumps({
            "state": "trusted_ro",
            "pose_initialized": True,
            "pose_trusted": True,
            "can_start_navigation": True,
            "localization_recovery_required": False,
            "integration_mode": "enforce",
        })))
        self.map_pub.publish(String(data=json.dumps({
            "data": {
                "current_map_id": "hall",
                "map_state": "ready",
                "localization_state": "stable",
            }
        })))

    def on_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.events.append(payload)

    def start_route(self):
        command = {
            "request_type": "navigation_command",
            "command_data": {
                "command_type": "start_route_task",
                "request_message_id": "interlock-start",
                "task_session_id": "interlock-session",
                "route_id": "interlock-route",
                "map_id": "hall",
                "route_waypoints": [{
                    "waypoint_id": "target-1",
                    "waypoint_role": "task",
                    "frame_id": "map",
                    "map_id": "hall",
                    "position": [1.0, 0.0, 0.0],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                    "need_broadcast": False,
                    "broadcast_blocking": False,
                    "stop_and_align": True,
                }],
            },
        }
        self.command_pub.publish(String(data=json.dumps(command)))

    def has_event(self, event_type, key=None, value=None):
        for event in self.events:
            if event.get("event_type") != event_type:
                continue
            data = event.get("event_data", {})
            if key is None or data.get(key) == value:
                return True
        return False


def spin_until(executor, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def call_interlock(node, executor, enabled):
    request = SetBool.Request()
    request.data = enabled
    future = node.interlock_client.call_async(request)
    if not spin_until(executor, future.done, 5.0):
        raise RuntimeError("interlock service timeout")
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(f"interlock service rejected: {response}")


def main():
    rclpy.init()
    node = RobotActionInterlockSmoke()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        if not spin_until(
            executor,
            lambda: node.command_pub.get_subscription_count() > 0
            and node.interlock_client.service_is_ready(),
            5.0,
        ):
            raise RuntimeError("route runtime interfaces unavailable")
        spin_until(executor, lambda: False, 0.5)
        node.start_route()
        if not spin_until(
            executor,
            lambda: node.has_event(
                "navigation_command_result", "command_type", "start_route_task"
            ),
            5.0,
        ):
            raise RuntimeError("route did not start")

        call_interlock(node, executor, True)
        if not spin_until(
            executor,
            lambda: node.has_event("navigation_paused", "pause_source", "robot_action"),
            3.0,
        ):
            raise RuntimeError("active route was not paused by action interlock")

        call_interlock(node, executor, False)
        if not spin_until(
            executor,
            lambda: node.has_event("navigation_resumed", "resume_source", "robot_action"),
            3.0,
        ):
            raise RuntimeError("action-owned route was not resumed")
        print("ROBOT_ACTION_INTERLOCK_OK")
        return 0
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown(timeout_sec=2.0)
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
