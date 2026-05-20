#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class DynamicOdomGroundPublisher(Node):
    def __init__(self):
        super().__init__("dynamic_odom_ground_publisher")

        self.parent_frame = self.declare_parameter("parent_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.child_frame = self.declare_parameter("child_frame", "odom_ground").value
        self.z_offset = float(self.declare_parameter("z_offset", 0.0).value)
        self.tf_timeout_sec = float(self.declare_parameter("tf_timeout_sec", 0.05).value)
        publish_rate = float(self.declare_parameter("publish_rate", 30.0).value)

        if publish_rate <= 0.0:
            publish_rate = 30.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_ground_tf)

        self.get_logger().info(
            f"publishing {self.parent_frame}->{self.child_frame} from "
            f"{self.parent_frame}->{self.base_frame} height"
        )

    def publish_ground_tf(self):
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"waiting for {self.parent_frame}->{self.base_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame
        msg.transform.translation.x = 0.0
        msg.transform.translation.y = 0.0
        msg.transform.translation.z = odom_to_base.transform.translation.z + self.z_offset
        msg.transform.rotation.x = 0.0
        msg.transform.rotation.y = 0.0
        msg.transform.rotation.z = 0.0
        msg.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicOdomGroundPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
