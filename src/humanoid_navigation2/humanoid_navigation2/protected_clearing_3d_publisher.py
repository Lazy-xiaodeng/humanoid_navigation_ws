#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


HEARTBEAT_POINTS_TOPIC = "/fast_lio/cloud_registered"
OCCUPANCY_POINTS_TOPIC = "/airy_points_filtered"
OUTPUT_CLEARING_CLOUD_TOPIC = "/protected_clearing_cloud_3d"
TARGET_FRAME = "base_footprint"

ANGLE_MIN_DEG = -180.0
ANGLE_MAX_DEG = 180.0
ANGLE_INCREMENT_DEG = 1.0

CLEARING_RANGE_MIN = 0.30
CLEARING_RANGE_MAX = 5.0
CLEARING_RANGE_STEP = 0.50
CLEARING_HEIGHT_LAYERS = [0.05, 0.10, 0.30, 0.50, 0.70, 0.90, 1.10, 1.30, 1.50, 1.70]
MAX_PUBLISH_RATE_HZ = 1.0

OCCUPIED_MIN_RANGE = 0.35
OCCUPIED_MAX_RANGE = 3.0
OCCUPIED_MIN_Z = 0.25
OCCUPIED_MAX_Z = 1.70
OCCUPIED_ANGLE_PADDING_BINS = 5
OCCUPANCY_TIMEOUT_SEC = 1.5
CLEARING_STOP_BEFORE_OBSTACLE = 0.10
EMPTY_OCCUPANCY_CLEAR_DELAY_SEC = 0.5


class ProtectedClearing3DPublisher(Node):
    def __init__(self):
        super().__init__("protected_clearing_3d_publisher")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.angle_min = math.radians(ANGLE_MIN_DEG)
        self.angle_max = math.radians(ANGLE_MAX_DEG)
        self.angle_increment = math.radians(ANGLE_INCREMENT_DEG)
        self.bin_count = int(math.floor((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        self.bin_angles = [self.angle_min + i * self.angle_increment for i in range(self.bin_count)]
        self.ranges = self.build_ranges()

        self.occupied_ranges = [math.inf] * self.bin_count
        self.last_occupancy_time = 0.0
        self.last_nonempty_occupancy_time = 0.0
        self.last_publish_time = 0.0
        self.last_warn_time = 0.0
        self.last_info_time = 0.0

        self.heartbeat_sub = self.create_subscription(
            PointCloud2,
            HEARTBEAT_POINTS_TOPIC,
            self.heartbeat_callback,
            sensor_qos,
        )
        self.occupancy_sub = self.create_subscription(
            PointCloud2,
            OCCUPANCY_POINTS_TOPIC,
            self.occupancy_callback,
            sensor_qos,
        )
        self.pub = self.create_publisher(PointCloud2, OUTPUT_CLEARING_CLOUD_TOPIC, sensor_qos)

        self.get_logger().info("protected_clearing_3d_publisher started.")
        self.get_logger().info(f"heartbeat_points_topic    : {HEARTBEAT_POINTS_TOPIC}")
        self.get_logger().info(f"occupancy_points_topic    : {OCCUPANCY_POINTS_TOPIC}")
        self.get_logger().info(f"output_clearing_cloud     : {OUTPUT_CLEARING_CLOUD_TOPIC}")
        self.get_logger().info(f"target_frame              : {TARGET_FRAME}")
        self.get_logger().info(f"bin_count                 : {self.bin_count}")
        self.get_logger().info(f"occupied_padding_bins     : {OCCUPIED_ANGLE_PADDING_BINS}")
        self.get_logger().info(f"occupancy_timeout_sec     : {OCCUPANCY_TIMEOUT_SEC}")
        self.get_logger().info(f"clear_stop_before_obstacle: {CLEARING_STOP_BEFORE_OBSTACLE}")

    def build_ranges(self):
        ranges = []
        r = CLEARING_RANGE_MIN
        while r <= CLEARING_RANGE_MAX + 1e-6:
            ranges.append(r)
            r += CLEARING_RANGE_STEP
        return ranges

    def should_publish_now(self):
        now = time.time()
        if now - self.last_publish_time < 1.0 / MAX_PUBLISH_RATE_HZ:
            return False
        self.last_publish_time = now
        return True

    def warn_throttled(self, msg):
        now = time.time()
        if now - self.last_warn_time > 2.0:
            self.get_logger().warn(msg)
            self.last_warn_time = now

    def info_throttled(self, msg):
        now = time.time()
        if now - self.last_info_time > 5.0:
            self.get_logger().info(msg)
            self.last_info_time = now

    def occupancy_callback(self, cloud_msg):
        occupied_ranges = [math.inf] * self.bin_count
        accepted_points = 0

        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            distance = math.hypot(x, y)

            if distance < OCCUPIED_MIN_RANGE or distance > OCCUPIED_MAX_RANGE:
                continue
            if z < OCCUPIED_MIN_Z or z > OCCUPIED_MAX_Z:
                continue

            accepted_points += 1
            bin_index = int(round((math.atan2(y, x) - self.angle_min) / self.angle_increment))
            if bin_index < 0 or bin_index >= self.bin_count:
                continue

            for offset in range(-OCCUPIED_ANGLE_PADDING_BINS, OCCUPIED_ANGLE_PADDING_BINS + 1):
                padded_index = bin_index + offset
                if 0 <= padded_index < self.bin_count:
                    occupied_ranges[padded_index] = min(occupied_ranges[padded_index], distance)

        self.occupied_ranges = occupied_ranges
        self.last_occupancy_time = time.time()
        if accepted_points > 0:
            self.last_nonempty_occupancy_time = self.last_occupancy_time

    def build_clearing_points(self, occupied_ranges):
        points = []
        for bin_index, angle in enumerate(self.bin_angles):
            nearest_obstacle = occupied_ranges[bin_index]
            max_clear_range = CLEARING_RANGE_MAX
            if math.isfinite(nearest_obstacle):
                max_clear_range = max(
                    CLEARING_RANGE_MIN,
                    nearest_obstacle - CLEARING_STOP_BEFORE_OBSTACLE,
                )

            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            for distance in self.ranges:
                if distance > max_clear_range:
                    continue
                x = distance * cos_a
                y = distance * sin_a
                for z in CLEARING_HEIGHT_LAYERS:
                    points.append([x, y, z])
        return points

    def heartbeat_callback(self, _cloud_msg):
        if not self.should_publish_now():
            return

        now = time.time()
        has_recent_occupancy_msg = now - self.last_occupancy_time <= OCCUPANCY_TIMEOUT_SEC
        has_recent_obstacles = now - self.last_nonempty_occupancy_time <= EMPTY_OCCUPANCY_CLEAR_DELAY_SEC
        if has_recent_occupancy_msg or has_recent_obstacles:
            occupied_ranges = self.occupied_ranges
        else:
            occupied_ranges = [math.inf] * self.bin_count

        clearing_points = self.build_clearing_points(occupied_ranges)
        if not clearing_points:
            self.warn_throttled("protected clearing cloud is empty; skip publish")
            return

        protected_bins = sum(1 for distance in occupied_ranges if math.isfinite(distance))
        self.info_throttled(
            f"protected clearing publish: protected_bins={protected_bins}, points={len(clearing_points)}"
        )

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = TARGET_FRAME
        self.pub.publish(pc2.create_cloud_xyz32(header, clearing_points))


def main(args=None):
    rclpy.init(args=args)
    node = ProtectedClearing3DPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
