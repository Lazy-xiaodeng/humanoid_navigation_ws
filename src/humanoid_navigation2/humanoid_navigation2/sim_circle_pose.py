#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import math

class CirclePoseSimulator(Node):
    def __init__(self):
        super().__init__('circle_pose_simulator')
        # ★ 改为 PoseWithCovarianceStamped 类型，解决话题类型冲突
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pcl_pose', 10)
        
        self.radius = 2.0
        self.angle = 0.0
        self.speed = 0.1
        
        self.timer = self.create_timer(0.1, self.publish_pose)
        self.get_logger().info('✅ 已切换为 PoseWithCovarianceStamped 格式，正在发布 /pcl_pose...')

    def euler_to_quaternion(self, yaw):
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        self.angle -= self.speed * 0.1
        x = self.radius * math.cos(self.angle)
        y = self.radius * math.sin(self.angle)
        yaw = self.angle - (math.pi / 2.0)

        # 填充位置和朝向
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = self.euler_to_quaternion(yaw)
        
        # 填充协方差矩阵（36个元素），全 0 会让置信度显示为 100%
        msg.pose.covariance = [0.0] * 36

        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CirclePoseSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
