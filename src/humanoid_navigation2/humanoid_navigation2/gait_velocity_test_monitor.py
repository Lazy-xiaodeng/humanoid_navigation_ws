#!/usr/bin/env python3
import json
import math
import time
import uuid
from collections import deque
from typing import Any, Deque, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def decimate_points(points, max_points):
    if len(points) <= max_points:
        return points
    if max_points <= 2:
        return [points[0], points[-1]]
    step = (len(points) - 1) / (max_points - 1)
    return [points[round(i * step)] for i in range(max_points)]


def yaw_from_quat_msg(quat) -> float:
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GaitVelocityTestMonitor(Node):
    def __init__(self):
        super().__init__("gait_velocity_test_monitor")

        self.declare_parameter("command_topic", "/app/robot_control")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("pose_topic", "/robot_realpose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("push_topic", "/integration/push_messages")
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("trajectory_max_points", 4000)
        self.declare_parameter("payload_trajectory_max_points", 900)
        self.declare_parameter("speed_history_max_points", 1200)
        self.declare_parameter("payload_speed_history_max_points", 260)
        self.declare_parameter("logs_max_entries", 120)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 1.5)

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.push_topic = str(self.get_parameter("push_topic").value)
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.trajectory_max_points = int(self.get_parameter("trajectory_max_points").value)
        self.payload_trajectory_max_points = int(self.get_parameter("payload_trajectory_max_points").value)
        self.speed_history_max_points = int(self.get_parameter("speed_history_max_points").value)
        self.payload_speed_history_max_points = int(self.get_parameter("payload_speed_history_max_points").value)
        self.logs_max_entries = int(self.get_parameter("logs_max_entries").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.push_pub = self.create_publisher(String, self.push_topic, 10)
        self.create_subscription(String, self.command_topic, self.command_callback, 20)
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 20)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)

        self.command_timer = self.create_timer(1.0 / max(1.0, self.command_rate_hz), self.command_timer_callback)
        self.publish_timer = self.create_timer(1.0 / max(1.0, self.publish_rate_hz), self.publish_state)

        self.active = False
        self.session_id = ""
        self.started_at: Optional[float] = None
        self.stopped_at: Optional[float] = None
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_pose: Optional[Dict[str, float]] = None
        self.current_pose_stamp: Optional[float] = None
        self.latest_speed: Dict[str, Any] = self.build_speed_payload(0.0, 0.0, 0.0, "idle", valid=False)
        self.latest_odom_speed: Optional[Dict[str, Any]] = None
        self.last_pose_for_speed: Optional[Dict[str, float]] = None
        self.filtered_speed: Optional[Dict[str, float]] = None
        self.last_distance_pose: Optional[Dict[str, float]] = None
        self.distance_m = 0.0
        self.max_actual_speed_mps = 0.0
        self.actual_speed_sum = 0.0
        self.actual_linear_x_sum = 0.0
        self.actual_angular_z_sum = 0.0
        self.sample_count = 0

        self.trajectory: Deque[Dict[str, float]] = deque(maxlen=self.trajectory_max_points)
        self.speed_history: Deque[Dict[str, float]] = deque(maxlen=self.speed_history_max_points)
        self.logs: Deque[Dict[str, Any]] = deque(maxlen=self.logs_max_entries)

        self.log_event("info", "步态速度测试监控节点已启动")
        self.log_event(
            "info",
            f"command={self.command_topic}, cmd_vel={self.cmd_vel_topic}, pose={self.pose_topic}, push={self.push_topic}"
        )

    def log_event(self, level: str, message: str):
        entry = {
            "time": time.time(),
            "level": level,
            "message": message,
        }
        self.logs.append(entry)
        log_fn = {
            "debug": self.get_logger().debug,
            "warn": self.get_logger().warn,
            "warning": self.get_logger().warn,
            "error": self.get_logger().error,
        }.get(level, self.get_logger().info)
        log_fn(message)

    def build_speed_payload(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        source: str,
        valid: bool = True,
        reject_reason: str = "",
    ) -> Dict[str, Any]:
        linear_x = round(linear_x, 3)
        linear_y = round(linear_y, 3)
        angular_z = round(angular_z, 3)
        speed_mps = round(math.hypot(linear_x, linear_y), 3)
        payload = {
            "linear_x": linear_x,
            "linear_y": linear_y,
            "angular_z": angular_z,
            "speed_mps": speed_mps,
            "turn_rate_radps": angular_z,
            "is_moving": speed_mps > 0.0 or abs(angular_z) > 0.0,
            "source": source,
            "valid": valid,
            "timestamp": time.time(),
        }
        if reject_reason:
            payload["reject_reason"] = reject_reason
        return payload

    def reset_session_buffers(self):
        self.trajectory.clear()
        self.speed_history.clear()
        self.distance_m = 0.0
        self.max_actual_speed_mps = 0.0
        self.actual_speed_sum = 0.0
        self.actual_linear_x_sum = 0.0
        self.actual_angular_z_sum = 0.0
        self.sample_count = 0
        self.last_distance_pose = None

    def start_test(self, linear_x: float, angular_z: float):
        linear_x = clamp(linear_x, -self.max_linear_speed, self.max_linear_speed)
        angular_z = clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)
        self.session_id = f"gait_test_{uuid.uuid4().hex[:8]}"
        self.started_at = time.time()
        self.stopped_at = None
        self.target_linear_x = linear_x
        self.target_angular_z = angular_z
        self.active = True
        self.reset_session_buffers()
        if self.current_pose is not None:
            self.last_distance_pose = dict(self.current_pose)
            self.trajectory.append({
                "x": self.current_pose["x"],
                "y": self.current_pose["y"],
                "t": self.started_at,
            })
        self.log_event(
            "info",
            f"开始测试: linear_x={self.target_linear_x:.3f} m/s, angular_z={self.target_angular_z:.3f} rad/s"
        )
        self.publish_cmd_vel(self.target_linear_x, self.target_angular_z)

    def stop_test(self, reason: str = "manual_stop"):
        was_active = self.active
        self.active = False
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.stopped_at = time.time()
        self.publish_cmd_vel(0.0, 0.0)
        if was_active:
            self.log_event("info", f"结束测试: reason={reason}")

    def publish_cmd_vel(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def command_timer_callback(self):
        if self.active:
            self.publish_cmd_vel(self.target_linear_x, self.target_angular_z)

    def command_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        command_type = data.get("command_type", "")
        params = data.get("parameters", {}) or {}
        if command_type != "gait_velocity_test":
            return

        action = str(params.get("action", "")).strip().lower()
        if action == "start":
            linear_x = float(params.get("linear_x", 0.0))
            angular_z = float(params.get("angular_z", 0.0))
            self.start_test(linear_x, angular_z)
        elif action == "stop":
            self.stop_test("manual_stop")
        elif action == "reset":
            self.stop_test("reset")
            self.reset_session_buffers()
            self.log_event("info", "已重置测试缓存")
        else:
            self.log_event("warn", f"收到未知测试动作: {action}")

    def estimate_actual_speed_from_pose(self, pose: Dict[str, float], stamp_sec: float) -> Optional[Dict[str, Any]]:
        previous = self.last_pose_for_speed
        current = {
            "x": pose["x"],
            "y": pose["y"],
            "yaw": pose["yaw"],
            "time": stamp_sec,
        }
        self.last_pose_for_speed = current

        if previous is None:
            self.filtered_speed = None
            return self.build_speed_payload(0.0, 0.0, 0.0, "pose_delta", valid=False, reject_reason="initializing")

        dt = current["time"] - previous["time"]
        if dt < 0.03:
            return None
        if dt > 1.0:
            self.filtered_speed = None
            return self.build_speed_payload(0.0, 0.0, 0.0, "pose_delta", valid=False, reject_reason="pose_gap_reset")

        map_vx = (current["x"] - previous["x"]) / dt
        map_vy = (current["y"] - previous["y"]) / dt
        yaw = current["yaw"]
        linear_x = math.cos(yaw) * map_vx + math.sin(yaw) * map_vy
        linear_y = -math.sin(yaw) * map_vx + math.cos(yaw) * map_vy
        angular_z = normalize_angle(current["yaw"] - previous["yaw"]) / dt

        if math.hypot(linear_x, linear_y) > 2.0 or abs(angular_z) > 4.0:
            self.filtered_speed = None
            return self.build_speed_payload(0.0, 0.0, 0.0, "pose_delta", valid=False, reject_reason="pose_jump_rejected")

        raw = {
            "linear_x": linear_x,
            "linear_y": linear_y,
            "angular_z": angular_z,
        }
        if self.filtered_speed is None:
            self.filtered_speed = raw
        else:
            alpha = 0.35
            self.filtered_speed = {
                key: self.filtered_speed[key] + alpha * (raw[key] - self.filtered_speed[key])
                for key in raw
            }
        return self.build_speed_payload(
            self.filtered_speed["linear_x"],
            self.filtered_speed["linear_y"],
            self.filtered_speed["angular_z"],
            "pose_delta",
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_sec <= 0.0:
            stamp_sec = time.time()
        pose = {
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "z": float(msg.pose.pose.position.z),
            "yaw": yaw_from_quat_msg(msg.pose.pose.orientation),
        }
        self.current_pose = pose
        self.current_pose_stamp = stamp_sec

        speed = self.estimate_actual_speed_from_pose(pose, stamp_sec)
        if speed is not None:
            self.latest_speed = speed

        if not self.active:
            return

        if self.last_distance_pose is None:
            self.last_distance_pose = dict(pose)
        else:
            step = math.hypot(pose["x"] - self.last_distance_pose["x"], pose["y"] - self.last_distance_pose["y"])
            if step <= 1.0:
                self.distance_m += step
            self.last_distance_pose = dict(pose)

        self.trajectory.append({
            "x": pose["x"],
            "y": pose["y"],
            "t": stamp_sec,
        })

        if speed is not None:
            self.append_speed_sample(speed, stamp_sec)

    def odom_callback(self, msg: Odometry):
        linear_x = -float(msg.twist.twist.linear.z)
        linear_y = float(msg.twist.twist.linear.x)
        angular_z = -float(msg.twist.twist.angular.y)
        payload = self.build_speed_payload(linear_x, linear_y, angular_z, "odom_twist")
        self.latest_odom_speed = payload
        pose_speed_age = time.time() - float(self.latest_speed.get("timestamp", 0.0))
        if (not self.latest_speed.get("valid", False)) or pose_speed_age > 0.5:
            self.latest_speed = payload

    def append_speed_sample(self, speed: Dict[str, Any], stamp_sec: float):
        actual_speed_mps = float(speed.get("speed_mps", 0.0))
        actual_linear_x = float(speed.get("linear_x", 0.0))
        actual_angular_z = float(speed.get("angular_z", 0.0))
        self.speed_history.append({
            "t": stamp_sec,
            "elapsed_s": round(max(0.0, stamp_sec - float(self.started_at or stamp_sec)), 3),
            "target_linear_x": round(self.target_linear_x, 3),
            "target_angular_z": round(self.target_angular_z, 3),
            "actual_linear_x": round(actual_linear_x, 3),
            "actual_speed_mps": round(actual_speed_mps, 3),
            "actual_angular_z": round(actual_angular_z, 3),
        })
        self.sample_count += 1
        self.actual_speed_sum += actual_speed_mps
        self.actual_linear_x_sum += actual_linear_x
        self.actual_angular_z_sum += actual_angular_z
        self.max_actual_speed_mps = max(self.max_actual_speed_mps, actual_speed_mps)

    def build_state_payload(self) -> Dict[str, Any]:
        now = time.time()
        active_duration = 0.0
        if self.started_at is not None:
            active_duration = (now if self.active else float(self.stopped_at or now)) - self.started_at

        trajectory_points = list(self.trajectory)
        speed_history = list(self.speed_history)
        avg_speed_mps = self.actual_speed_sum / self.sample_count if self.sample_count else 0.0
        avg_linear_x = self.actual_linear_x_sum / self.sample_count if self.sample_count else 0.0
        avg_angular_z = self.actual_angular_z_sum / self.sample_count if self.sample_count else 0.0

        return {
            "session_id": self.session_id,
            "active": self.active,
            "started_at": self.started_at,
            "stopped_at": self.stopped_at,
            "elapsed_s": round(max(0.0, active_duration), 3),
            "target": {
                "linear_x": round(self.target_linear_x, 3),
                "angular_z": round(self.target_angular_z, 3),
            },
            "actual_speed": self.latest_speed,
            "pose": {
                **(self.current_pose or {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}),
                "stamp": self.current_pose_stamp,
            },
            "stats": {
                "distance_m": round(self.distance_m, 3),
                "avg_speed_mps": round(avg_speed_mps, 3),
                "avg_linear_x": round(avg_linear_x, 3),
                "avg_angular_z": round(avg_angular_z, 3),
                "peak_speed_mps": round(self.max_actual_speed_mps, 3),
                "sample_count": self.sample_count,
            },
            "trajectory": [
                {"x": round(item["x"], 3), "y": round(item["y"], 3), "t": round(item["t"], 3)}
                for item in decimate_points(trajectory_points, self.payload_trajectory_max_points)
            ],
            "speed_series": decimate_points(speed_history, self.payload_speed_history_max_points),
            "logs": list(self.logs),
        }

    def publish_state(self):
        message = {
            "protocol_version": "2.0",
            "message_id": f"gait_velocity_test_{int(time.time() * 1000)}",
            "timestamp": time.time(),
            "message_type": "push",
            "data_type": "gait_velocity_test",
            "source": "gait_velocity_test_monitor",
            "destination": "subscribed",
            "data": self.build_state_payload(),
            "metadata": {
                "status": "success",
                "qos_level": "realtime",
                "data_freshness": 0.0,
            },
        }
        msg = String()
        msg.data = json.dumps(message, ensure_ascii=False)
        self.push_pub.publish(msg)

    def destroy_node(self):
        try:
            self.publish_cmd_vel(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GaitVelocityTestMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
