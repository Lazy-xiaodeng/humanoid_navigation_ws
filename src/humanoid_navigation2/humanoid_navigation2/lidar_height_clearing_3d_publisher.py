#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


OUTPUT_CLEARING_CLOUD_TOPIC = "/clearing_cloud_3d_lidar"
PARENT_FRAME = "base_footprint"
CLEARING_FRAME = "clearing_lidar"

LIDAR_X = 0.0
LIDAR_Y = 0.0
LIDAR_Z = 1.215

ANGLE_MIN_DEG = -180.0
ANGLE_MAX_DEG = 180.0
ANGLE_INCREMENT_DEG = 0.5

CLEARING_RANGE_MIN = 0.30
CLEARING_RANGE_MAX = 5.0
CLEARING_RANGE_STEP = 0.50

BASE_HEIGHT_LAYERS = [
    0.05,
    0.08,
    0.10,
    0.20,
    0.30,
    0.40,
    0.50,
    0.60,
    0.70,
    0.80,
    0.90,
    1.00,
    1.10,
    1.20,
    1.30,
    1.40,
    1.50,
    1.60,
    1.70,
]

PUBLISH_RATE_HZ = 2.0
QOS_DEPTH = 5
DEBUG = False


class LidarHeightClearing3DPublisher(Node):
    def __init__(self):
        super().__init__("lidar_height_clearing_3d_publisher")

        self.output_topic = self.declare_parameter(
            "output_topic", OUTPUT_CLEARING_CLOUD_TOPIC
        ).value
        self.parent_frame = self.declare_parameter("parent_frame", PARENT_FRAME).value
        self.clearing_frame = self.declare_parameter("clearing_frame", CLEARING_FRAME).value
        self.lidar_x = float(self.declare_parameter("lidar_x", LIDAR_X).value)
        self.lidar_y = float(self.declare_parameter("lidar_y", LIDAR_Y).value)
        self.lidar_z = float(self.declare_parameter("lidar_z", LIDAR_Z).value)
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", PUBLISH_RATE_HZ).value
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=QOS_DEPTH,
        )

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        self.pub = self.create_publisher(PointCloud2, self.output_topic, sensor_qos)
        self.clearing_points = self.build_clearing_points()

        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 0.1),
            self.publish_clearing_cloud,
        )

        self.get_logger().info("lidar_height_clearing_3d_publisher started.")
        self.get_logger().info(f"output_clearing_cloud : {self.output_topic}")
        self.get_logger().info(
            f"clearing frame        : {self.parent_frame} -> {self.clearing_frame}"
        )
        self.get_logger().info(
            f"lidar origin          : x={self.lidar_x}, y={self.lidar_y}, z={self.lidar_z}"
        )
        self.get_logger().info(
            f"clearing range        : {CLEARING_RANGE_MIN} ~ {CLEARING_RANGE_MAX}, "
            f"step={CLEARING_RANGE_STEP}"
        )
        self.get_logger().info(f"base height layers    : {BASE_HEIGHT_LAYERS}")
        self.get_logger().info(f"precomputed points    : {len(self.clearing_points)}")
        self.get_logger().info(f"publish_rate_hz       : {self.publish_rate_hz}")

    def publish_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.clearing_frame
        transform.transform.translation.x = self.lidar_x
        transform.transform.translation.y = self.lidar_y
        transform.transform.translation.z = self.lidar_z
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def build_ranges(self):
        ranges = []
        distance = CLEARING_RANGE_MIN
        while distance <= CLEARING_RANGE_MAX + 1e-6:
            ranges.append(distance)
            distance += CLEARING_RANGE_STEP
        return ranges

    def build_angles(self):
        angle_min = math.radians(ANGLE_MIN_DEG)
        angle_max = math.radians(ANGLE_MAX_DEG)
        angle_increment = math.radians(ANGLE_INCREMENT_DEG)
        count = int(math.floor((angle_max - angle_min) / angle_increment)) + 1
        return [angle_min + index * angle_increment for index in range(count)]

    def build_clearing_points(self):
        points = []
        ranges = self.build_ranges()
        angles = self.build_angles()

        for angle in angles:
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            for distance in ranges:
                base_x = distance * cos_a
                base_y = distance * sin_a
                for base_z in BASE_HEIGHT_LAYERS:
                    points.append(
                        [
                            base_x - self.lidar_x,
                            base_y - self.lidar_y,
                            base_z - self.lidar_z,
                        ]
                    )

        return points

    def publish_clearing_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.clearing_frame

        self.pub.publish(pc2.create_cloud_xyz32(header, self.clearing_points))

        if DEBUG:
            self.get_logger().info(
                f"lidar-height clearing published: points={len(self.clearing_points)}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarHeightClearing3DPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
