#!/usr/bin/env python3
import csv
import json
import math
import os
from collections import Counter
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PriorMapBagMonitor(Node):
    def __init__(self):
        super().__init__("prior_map_bag_monitor")
        self.output_dir = self.declare_parameter("output_dir", "/tmp/prior_map_bag_monitor").value
        self.sample_period = float(self.declare_parameter("sample_period", 0.5).value)
        self.robot_pose_topic = self.declare_parameter("robot_pose_topic", "/bag/robot_realpose").value

        os.makedirs(self.output_dir, exist_ok=True)

        self.latest_confidence: Optional[float] = None
        self.latest_bridge_status = ""
        self.latest_robot_pose: Optional[Tuple[float, float, float]] = None
        self.latest_prior_pose: Optional[Tuple[float, float, float]] = None
        self.latest_map_odom: Optional[Tuple[float, float, float]] = None
        self.samples = []
        self.status_counter = Counter()

        self.create_subscription(Float32, "/prior_localization/confidence", self.on_confidence, 10)
        self.create_subscription(String, "/localization/prior_map_odom_bridge_status", self.on_status, 50)
        self.create_subscription(Odometry, "/prior_localization/odom", self.on_prior_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.robot_pose_topic, self.on_robot_pose, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.create_timer(self.sample_period, self.on_timer)
        self.get_logger().info(f"monitor output_dir={self.output_dir} robot_pose_topic={self.robot_pose_topic}")

    def on_confidence(self, msg: Float32):
        self.latest_confidence = float(msg.data)

    def on_status(self, msg: String):
        self.latest_bridge_status = msg.data
        key = msg.data.split()[0] if msg.data else ""
        if key:
            self.status_counter[key] += 1

    def on_prior_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_prior_pose = (float(p.x), float(p.y), yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_robot_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_robot_pose = (float(p.x), float(p.y), yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_tf(self, msg: TFMessage):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                t = transform.transform.translation
                q = transform.transform.rotation
                self.latest_map_odom = (
                    float(t.x),
                    float(t.y),
                    yaw_from_quaternion(q.x, q.y, q.z, q.w),
                )

    def on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        row = {
            "sim_time": now,
            "confidence": self.latest_confidence,
            "bridge_status": self.latest_bridge_status,
            "robot_x": None,
            "robot_y": None,
            "robot_yaw": None,
            "prior_x": None,
            "prior_y": None,
            "prior_yaw": None,
            "map_odom_x": None,
            "map_odom_y": None,
            "map_odom_yaw": None,
        }
        if self.latest_robot_pose is not None:
            row["robot_x"], row["robot_y"], row["robot_yaw"] = self.latest_robot_pose
        if self.latest_prior_pose is not None:
            row["prior_x"], row["prior_y"], row["prior_yaw"] = self.latest_prior_pose
        if self.latest_map_odom is not None:
            row["map_odom_x"], row["map_odom_y"], row["map_odom_yaw"] = self.latest_map_odom
        self.samples.append(row)

    def write_outputs(self):
        path = os.path.join(self.output_dir, "samples.csv")
        fields = [
            "sim_time", "confidence", "bridge_status",
            "robot_x", "robot_y", "robot_yaw",
            "prior_x", "prior_y", "prior_yaw",
            "map_odom_x", "map_odom_y", "map_odom_yaw",
        ]
        with open(path, "w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.samples)
        with open(os.path.join(self.output_dir, "summary.json"), "w") as fp:
            json.dump({
                "sample_count": len(self.samples),
                "bridge_status_counter": dict(self.status_counter),
            }, fp, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = PriorMapBagMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.write_outputs()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
