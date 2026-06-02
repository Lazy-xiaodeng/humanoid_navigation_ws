#!/usr/bin/env python3
import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def path_len(points):
    return sum(dist(a, b) for a, b in zip(points, points[1:])) if len(points) > 1 else 0.0


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def closest_point_on_segment(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return a, dist(p, a), 0.0
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    c = (ax + t * dx, ay + t * dy)
    return c, dist(p, c), t


def closest_point_on_path(p, points):
    if len(points) < 2:
        return None, None, None, None
    best = None
    travelled = 0.0
    best_travelled = 0.0
    for index, (a, b) in enumerate(zip(points, points[1:])):
        c, d, t = closest_point_on_segment(p, a, b)
        seg_len = dist(a, b)
        if best is None or d < best[1]:
            best = (c, d, index, t)
            best_travelled = travelled + seg_len * t
        travelled += seg_len
    return best[0], best[1], best[2], best_travelled


def decimate(points, max_points):
    if len(points) <= max_points:
        return points
    if max_points <= 2:
        return [points[0], points[-1]]
    step = (len(points) - 1) / (max_points - 1)
    return [points[round(i * step)] for i in range(max_points)]


class NavigationPathMonitor(Node):
    def __init__(self):
        super().__init__("navigation_path_monitor")

        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("pose_topic", "/robot_realpose")
        self.declare_parameter("navigation_status_topic", "/navigation/status")
        self.declare_parameter("push_topic", "/integration/push_messages")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("trail_max_points", 20000)
        self.declare_parameter("payload_plan_max_points", 260)
        self.declare_parameter("payload_plan_history_max_points", 1400)
        self.declare_parameter("payload_trail_max_points", 1400)
        self.declare_parameter("plan_history_max_segments", 220)
        self.declare_parameter("new_plan_reset_distance_m", 0.25)
        self.declare_parameter("deviation_warn_m", 0.35)
        self.declare_parameter("deviation_bad_m", 0.70)

        self.plan_topic = self.get_parameter("plan_topic").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.navigation_status_topic = self.get_parameter("navigation_status_topic").value
        self.push_topic = self.get_parameter("push_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.trail_max_points = int(self.get_parameter("trail_max_points").value)
        self.payload_plan_max_points = int(self.get_parameter("payload_plan_max_points").value)
        self.payload_plan_history_max_points = int(
            self.get_parameter("payload_plan_history_max_points").value)
        self.payload_trail_max_points = int(self.get_parameter("payload_trail_max_points").value)
        self.plan_history_max_segments = int(self.get_parameter("plan_history_max_segments").value)
        self.new_plan_reset_distance_m = float(self.get_parameter("new_plan_reset_distance_m").value)
        self.deviation_warn_m = float(self.get_parameter("deviation_warn_m").value)
        self.deviation_bad_m = float(self.get_parameter("deviation_bad_m").value)

        self.plan_points = []
        self.plan_available = False
        self.plan_stamp_sec = None
        self.plan_seq = 0
        self.plan_history = []
        self.trail = deque(maxlen=self.trail_max_points)
        self.current_pose = None
        self.current_yaw = None
        self.pose_stamp_sec = None
        self.last_publish_wall = 0.0
        self.current_waypoint_key = ""
        self.current_event_type = ""

        self.push_pub = self.create_publisher(String, self.push_topic, 10)
        self.create_subscription(Path, self.plan_topic, self.plan_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 20)
        self.create_subscription(String, self.navigation_status_topic, self.navigation_status_callback, 10)
        self.create_timer(1.0 / max(0.5, self.publish_rate_hz), self.publish_monitor)

        self.get_logger().info(
            "navigation_path_monitor started: "
            f"plan={self.plan_topic}, pose={self.pose_topic}, "
            f"status={self.navigation_status_topic}, push={self.push_topic}"
        )

    def stamp_to_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def plan_callback(self, msg):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(points) < 2:
            return

        self.plan_points = points
        self.plan_available = True
        self.plan_stamp_sec = self.stamp_to_sec(msg.header.stamp)
        self.plan_seq += 1
        self.add_plan_history(points)

        self.get_logger().info(
            f"received /plan seq={self.plan_seq}, points={len(points)}, len={path_len(points):.2f}m"
        )

    def add_plan_history(self, points):
        if self.plan_history:
            previous = self.plan_history[-1]
            same_segment = (
                dist(previous[0], points[0]) < 0.03 and
                dist(previous[-1], points[-1]) < 0.03 and
                abs(path_len(previous) - path_len(points)) < 0.03
            )
            if same_segment:
                self.plan_history[-1] = points
                return

        self.plan_history.append(points)
        if len(self.plan_history) > self.plan_history_max_segments:
            self.plan_history = self.plan_history[-self.plan_history_max_segments:]

    def navigation_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        event_type = data.get("event_type", "")
        self.current_event_type = event_type or data.get("current_state", "")
        event_data = data.get("event_data", {}) or {}
        goal = data.get("current_goal", {}) or {}
        waypoint_id = (
            goal.get("waypoint_id") or
            event_data.get("waypoint_id") or
            str(data.get("current_waypoint_index", ""))
        )
        waypoint_index = event_data.get("waypoint_index", data.get("current_waypoint_index", ""))
        waypoint_key = f"{waypoint_index}:{waypoint_id}"

        if event_type == "navigation_started":
            self.reset_full_route(f"{event_type}:{waypoint_key}")
            self.current_waypoint_key = waypoint_key
        elif event_type == "waypoint_started":
            if waypoint_key != self.current_waypoint_key or event_type == "navigation_started":
                self.current_waypoint_key = waypoint_key
                self.mark_waiting_for_plan(f"{event_type}:{waypoint_key}")
        elif event_type in (
            "waypoint_reached",
            "navigation_completed",
            "navigation_stopped",
            "navigation_failed",
            "navigation_cancelled",
            "navigation_aborted",
        ):
            self.mark_waiting_for_plan(event_type)

    def reset_full_route(self, reason):
        self.plan_points = []
        self.plan_available = False
        self.plan_stamp_sec = None
        self.plan_history.clear()
        self.trail.clear()
        self.plan_seq += 1
        self.publish_monitor_state(reason)
        self.get_logger().info(f"reset path monitor route: {reason}")

    def mark_waiting_for_plan(self, reason):
        self.plan_points = []
        self.plan_available = False
        self.plan_stamp_sec = None
        self.plan_seq += 1
        self.publish_monitor_state(reason)
        self.get_logger().info(f"path monitor waiting for next plan: {reason}")

    def pose_callback(self, msg):
        p = msg.pose.pose.position
        self.current_pose = (float(p.x), float(p.y))
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.pose_stamp_sec = self.stamp_to_sec(msg.header.stamp)
        self.trail.append(self.current_pose)

    def deviation_level(self, deviation):
        if deviation is None:
            return "none"
        if deviation >= self.deviation_bad_m:
            return "bad"
        if deviation >= self.deviation_warn_m:
            return "warn"
        return "ok"

    def publish_monitor(self):
        if self.current_pose is None:
            return

        if not self.plan_available or not self.plan_points:
            self.publish_monitor_state("waiting_for_plan")
            return

        closest, deviation, segment_index, travelled = closest_point_on_path(self.current_pose, self.plan_points)
        total_len = path_len(self.plan_points)
        remaining = max(0.0, total_len - travelled) if travelled is not None else None
        goal = self.plan_points[-1]
        start = self.plan_points[0]

        payload = {
            "protocol_version": "2.0",
            "message_id": f"path_monitor_{int(time.time() * 1000)}",
            "timestamp": time.time(),
            "message_type": "push",
            "data_type": "navigation_path_monitor",
            "source": "navigation_path_monitor",
            "destination": "subscribed",
            "data": {
                "plan_seq": self.plan_seq,
                "plan_stamp": self.plan_stamp_sec,
                "pose_stamp": self.pose_stamp_sec,
                "current_pose": {
                    "x": self.current_pose[0],
                    "y": self.current_pose[1],
                    "yaw": self.current_yaw,
                },
                "closest_point": {
                    "x": closest[0],
                    "y": closest[1],
                    "segment_index": segment_index,
                } if closest is not None else None,
                "deviation_m": deviation,
                "deviation_level": self.deviation_level(deviation),
                "distance_to_goal_m": dist(self.current_pose, goal),
                "remaining_path_m": remaining,
                "progress_ratio": max(0.0, min(1.0, travelled / total_len)) if total_len > 1e-6 else 0.0,
                "plan_length_m": total_len,
                "trail_length_m": path_len(list(self.trail)),
                "start": {"x": start[0], "y": start[1]},
                "goal": {"x": goal[0], "y": goal[1]},
                "plan_available": True,
                "plan_history_segments": self.encode_plan_history(),
                "plan_points": [{"x": x, "y": y} for x, y in decimate(self.plan_points, self.payload_plan_max_points)],
                "actual_trail": [{"x": x, "y": y} for x, y in decimate(list(self.trail), self.payload_trail_max_points)],
            },
            "metadata": {
                "status": "success",
                "qos_level": "realtime",
                "data_freshness": max(0.0, time.time() - (self.pose_stamp_sec or time.time())),
            },
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.push_pub.publish(msg)

    def encode_plan_history(self):
        if not self.plan_history:
            return []
        per_segment = max(2, self.payload_plan_history_max_points // max(1, len(self.plan_history)))
        return [
            [{"x": x, "y": y} for x, y in decimate(segment, per_segment)]
            for segment in self.plan_history
        ]

    def publish_monitor_state(self, reason):
        payload = {
            "protocol_version": "2.0",
            "message_id": f"path_monitor_clear_{int(time.time() * 1000)}",
            "timestamp": time.time(),
            "message_type": "push",
            "data_type": "navigation_path_monitor",
            "source": "navigation_path_monitor",
            "destination": "subscribed",
            "data": {
                "plan_seq": self.plan_seq,
                "plan_stamp": self.plan_stamp_sec,
                "pose_stamp": self.pose_stamp_sec,
                "current_pose": {
                    "x": self.current_pose[0],
                    "y": self.current_pose[1],
                    "yaw": self.current_yaw,
                } if self.current_pose is not None else None,
                "closest_point": None,
                "deviation_m": None,
                "deviation_level": "none",
                "distance_to_goal_m": None,
                "remaining_path_m": None,
                "progress_ratio": 0.0,
                "plan_length_m": 0.0,
                "trail_length_m": path_len(list(self.trail)),
                "start": None,
                "goal": None,
                "plan_history_segments": self.encode_plan_history(),
                "plan_points": [],
                "actual_trail": [{"x": x, "y": y} for x, y in decimate(list(self.trail), self.payload_trail_max_points)],
                "plan_available": False,
                "reset_reason": reason,
                "navigation_event": self.current_event_type,
            },
            "metadata": {
                "status": "success",
                "qos_level": "realtime",
                "data_freshness": 0.0,
            },
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.push_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationPathMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
