#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class RvizInitialPoseAdapter(Node):
    def __init__(self):
        super().__init__("rviz_initialpose_adapter")

        self.input_topic = self.declare_parameter("input_topic", "/initialpose").value
        self.bridge_pose_topic = self.declare_parameter(
            "bridge_pose_topic", "/prior_localization/pose_with_covariance"
        ).value
        self.robosense_pose_topic = self.declare_parameter(
            "robosense_pose_topic", "/prior_localization/manual_initialpose"
        ).value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.map_ground_frame = self.declare_parameter("map_ground_frame", "map_ground").value
        self.tf_timeout_sec = float(self.declare_parameter("tf_timeout_sec", 0.2).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.bridge_pose_topic, 10
        )
        self.robosense_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.robosense_pose_topic, 10
        )
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.input_topic, self.initialpose_callback, 10
        )

        self.get_logger().info(
            f"RViz initial pose adapter started: {self.input_topic} -> "
            f"{self.bridge_pose_topic}, {self.robosense_pose_topic}"
        )

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        converted = self.convert_to_map(msg)
        if converted is None:
            return

        converted.header.stamp = self.get_clock().now().to_msg()
        self.bridge_pose_pub.publish(converted)
        self.robosense_pose_pub.publish(converted)

        pose = converted.pose.pose.position
        yaw = yaw_from_quaternion(converted.pose.pose.orientation)
        self.get_logger().info(
            f"forwarded manual initial pose in {self.map_frame}: "
            f"x={pose.x:.3f} y={pose.y:.3f} yaw={math.degrees(yaw):.1f}deg"
        )

    def convert_to_map(self, msg: PoseWithCovarianceStamped):
        frame_id = msg.header.frame_id or self.map_ground_frame
        if frame_id == self.map_frame:
            return msg

        if frame_id != self.map_ground_frame:
            self.get_logger().warn(
                f"ignore initial pose in unsupported frame {frame_id}; "
                f"expected {self.map_ground_frame} or {self.map_frame}",
                throttle_duration_sec=2.0,
            )
            return None

        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = Time()

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.map_ground_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"waiting for TF {self.map_frame}->{self.map_ground_frame} before "
                f"using manual initial pose: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

        tf_yaw = yaw_from_quaternion(transform.transform.rotation)
        pose_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        out_yaw = normalize_angle(tf_yaw + pose_yaw)

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        cos_yaw = math.cos(tf_yaw)
        sin_yaw = math.sin(tf_yaw)

        converted = PoseWithCovarianceStamped()
        converted.header = msg.header
        converted.header.frame_id = self.map_frame
        converted.pose.covariance = list(msg.pose.covariance)
        converted.pose.pose.position.x = (
            transform.transform.translation.x + cos_yaw * px - sin_yaw * py
        )
        converted.pose.pose.position.y = (
            transform.transform.translation.y + sin_yaw * px + cos_yaw * py
        )
        converted.pose.pose.position.z = (
            transform.transform.translation.z + msg.pose.pose.position.z
        )
        qx, qy, qz, qw = quaternion_from_yaw(out_yaw)
        converted.pose.pose.orientation.x = qx
        converted.pose.pose.orientation.y = qy
        converted.pose.pose.orientation.z = qz
        converted.pose.pose.orientation.w = qw
        return converted


def main(args=None):
    rclpy.init(args=args)
    node = RvizInitialPoseAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
