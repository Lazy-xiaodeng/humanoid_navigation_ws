#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import math

class RotatePoseSimulator(Node):
    def __init__(self):
        super().__init__('rotate_pose_simulator')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pcl_pose', 10)
        
        # 旋转参数
        self.yaw_angle = 0.0     # 初始偏航角
        self.speed = 0.5         # 旋转速度 (rad/s)
        
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.get_logger().info('✅ 正在向 /pcl_pose 发布原地顺时针旋转(只改变朝向)的测试数据...')

    def euler_to_quaternion(self, yaw):
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # 顺时针旋转，角度递减 (每次减去 0.1s * speed)
        self.yaw_angle -= self.speed * 0.1
        
        # 保持角度在 -PI 到 PI 之间（防止数值积压过大）
        if self.yaw_angle < -math.pi:
            self.yaw_angle += 2 * math.pi

        # 坐标固定原地不动
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        # 仅更新朝向
        msg.pose.pose.orientation = self.euler_to_quaternion(self.yaw_angle)
        
        msg.pose.covariance = [0.0] * 36

        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RotatePoseSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
