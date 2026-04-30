#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformException

class RobotRealPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_realpose_publisher')

        self.declare_parameter('global_frame', 'map_ground')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 10.0)

        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/robot_realpose', 10)

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'robot_realpose_publisher started: {self.global_frame} -> {self.base_frame}, '
            f'publish to /robot_realpose'
        )

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.global_frame

            msg.pose.pose.position.x = transform.transform.translation.x
            msg.pose.pose.position.y = transform.transform.translation.y
            msg.pose.pose.position.z = transform.transform.translation.z

            msg.pose.pose.orientation.x = transform.transform.rotation.x
            msg.pose.pose.orientation.y = transform.transform.rotation.y
            msg.pose.pose.orientation.z = transform.transform.rotation.z
            msg.pose.pose.orientation.w = transform.transform.rotation.w

            # 默认协方差
            msg.pose.covariance = [0.0] * 36

            self.pub.publish(msg)

        except TransformException as e:
            self.get_logger().warn(f'Failed to get TF {self.global_frame}->{self.base_frame}: {e}')

def main(args=None):
    # 1. 初始化 ROS2 环境
    rclpy.init(args=args)
    
    # 2. 创建节点对象
    node = RobotRealPosePublisher()
    
    try:
        # 3. 保持节点持续运行（Spin）
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕获 Ctrl+C 中断信号，不抛出异常
        pass
    finally:
        # 4. 销毁节点
        node.destroy_node()
        # 5. 安全关闭 rclpy
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()