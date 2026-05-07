#!/usr/bin/env python3
"""
IMU 坐标系转换节点

将 /airy_imu 的原始 IMU 数据从非标准 body 坐标系
(X左 Y下 Z后) 转换到 ROS 标准坐标系 (X前 Y左 Z上)，
发布到 /imu_standard 供 hdl_localization 使用。

转换使用 body → base_footprint 的静态 TF 中定义的旋转：
  q_body_to_standard = (x=0.5, y=0.5, z=-0.5, w=0.5)
  旋转矩阵 R = [[0, 1, 0], [0, 0, -1], [-1, 0, 0]]

  标准_X = body_Y      (前 = 左)
  标准_Y = -body_Z     (左 = 上)
  标准_Z = -body_X     (上 = 前)

用法:
  ros2 run humanoid_navigation2 imu_transformer.py
"""

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuTransformer(Node):
    """将 IMU 数据从非标准坐标系旋转到 ROS 标准坐标系"""

    def __init__(self):
        super().__init__('imu_transformer')

        self.declare_parameter('input_topic', '/airy_imu')
        self.declare_parameter('output_topic', '/imu_standard')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # body → 标准 的旋转矩阵
        # q(x=0.5, y=0.5, z=-0.5, w=0.5) → R = [[0,1,0],[0,0,-1],[-1,0,0]]
        self.R = np.array([
            [0.0,  1.0,  0.0],
            [0.0,  0.0, -1.0],
            [-1.0, 0.0,  0.0]
        ])

        self.sub = self.create_subscription(Imu, input_topic, self.callback, 10)
        self.pub = self.create_publisher(Imu, output_topic, 10)

        self.get_logger().info(f'IMU坐标转换: {input_topic} → {output_topic}')
        self.get_logger().info('  旋转矩阵: R_body_to_standard = [[0,1,0],[0,0,-1],[-1,0,0]]')

    def callback(self, msg: Imu):
        # 转换线性加速度: a_std = R * a_body
        a_body = np.array([msg.linear_acceleration.x,
                           msg.linear_acceleration.y,
                           msg.linear_acceleration.z])
        a_std = self.R @ a_body

        # 转换角速度: ω_std = R * ω_body (角速度在旋转下按向量变换)
        w_body = np.array([msg.angular_velocity.x,
                           msg.angular_velocity.y,
                           msg.angular_velocity.z])
        w_std = self.R @ w_body

        # 转换四元数姿态 (如果IMU提供了有效姿态)
        # q_new = q_body_to_std * q_orig  (四元数乘法，先旋转orig再旋转body_to_std)
        # hdl_localization 不使用 IMU orientation，此处保留以备其他用途
        if (abs(msg.orientation.w) + abs(msg.orientation.x) +
            abs(msg.orientation.y) + abs(msg.orientation.z)) > 0.001:
            q_orig = np.array([msg.orientation.w, msg.orientation.x,
                               msg.orientation.y, msg.orientation.z])
            # q_body_to_std = (w=0.5, x=0.5, y=0.5, z=-0.5)
            q_rot = np.array([0.5, 0.5, 0.5, -0.5])  # w,x,y,z
            # 四元数乘法: q_new = q_rot * q_orig
            w1, x1, y1, z1 = q_rot
            w2, x2, y2, z2 = q_orig
            msg.orientation.w = w1*w2 - x1*x2 - y1*y2 - z1*z2
            msg.orientation.x = w1*x2 + x1*w2 + y1*z2 - z1*y2
            msg.orientation.y = w1*y2 - x1*z2 + y1*w2 + z1*x2
            msg.orientation.z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        # 更新消息
        msg.linear_acceleration.x = a_std[0]
        msg.linear_acceleration.y = a_std[1]
        msg.linear_acceleration.z = a_std[2]

        msg.angular_velocity.x = w_std[0]
        msg.angular_velocity.y = w_std[1]
        msg.angular_velocity.z = w_std[2]

        # 更新帧ID
        msg.header.frame_id = 'base_footprint'

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
