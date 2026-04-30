#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Synthetic Data Publisher (模拟数据发布器)
功能：生成模拟的机器人定位、路径、导航状态和系统状态数据，
     用于测试 data_integration_node 和 APP 的通信链路。
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu
import json
import math
import time
import random

class SyntheticDataPublisher(Node):
    def __init__(self):
        super().__init__('synthetic_data_publisher')
        
        self.get_logger().info('🚀 模拟数据生成器启动...')
        self.get_logger().info('   正在模拟：定位(AMCL), 路径(Plan), 导航状态, 电池, IMU...')

        # ================= 发布器配置 (对应 data_integration_node 的订阅) =================
        
        # 1. 模拟定位输出
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # 2. 模拟路径规划
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        
        # 3. 模拟业务状态 (JSON String)
        self.nav_status_pub = self.create_publisher(String, '/navigation/status', 10)
        self.sys_status_pub = self.create_publisher(String, '/robot_status_processed', 10)
        
        # 4. 模拟 IMU
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        # ================= 模拟状态变量 =================
        self.timer_period = 0.1  # 10Hz 更新频率
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 生成一个 "8字形" 的模拟路径点集
        self.sim_path_points = self.generate_figure_eight_path(scale=5.0, points=200)
        self.current_idx = 0
        self.direction = 1  # 1: 正向移动
        
        # 模拟电池
        self.battery_level = 100.0
        
        # 缓存的路径消息（只需生成一次）
        self.cached_path_msg = self.create_path_message()

    def generate_figure_eight_path(self, scale=1.0, points=100):
        """生成一个8字形路径点列表"""
        path = []
        for i in range(points):
            t = 2 * math.pi * i / points
            x = scale * math.sin(t)
            y = scale * math.sin(t) * math.cos(t)
            # 计算切线角度作为朝向
            next_t = 2 * math.pi * (i + 1) / points
            next_x = scale * math.sin(next_t)
            next_y = scale * math.sin(next_t) * math.cos(next_t)
            yaw = math.atan2(next_y - y, next_x - x)
            path.append({'x': x, 'y': y, 'yaw': yaw})
        return path

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def create_path_message(self):
        """创建 ROS Path 消息"""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        for p in self.sim_path_points:
            pose = PoseStamped()
            pose.pose.position.x = p['x']
            pose.pose.position.y = p['y']
            pose.pose.orientation = self.get_quaternion_from_euler(0, 0, p['yaw'])
            msg.poses.append(pose)
        
        return msg

    def timer_callback(self):
        """主循环：更新状态并发布所有话题"""
        current_time = self.get_clock().now().to_msg()
        
        # 1. 获取当前模拟位置
        current_point = self.sim_path_points[self.current_idx]
        
        # 更新索引（循环移动）
        self.current_idx = (self.current_idx + 1) % len(self.sim_path_points)
        
        # ================= 发布 /amcl_pose (定位) =================
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = current_point['x']
        pose_msg.pose.pose.position.y = current_point['y']
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation = self.get_quaternion_from_euler(0, 0, current_point['yaw'])
        # 模拟高置信度
        pose_msg.pose.covariance = [0.01] * 36 
        self.pose_pub.publish(pose_msg)

        # ================= 发布 /odom (里程计速度) =================
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = pose_msg.pose.pose
        odom_msg.twist.twist.linear.x = 0.5  # 模拟 0.5m/s 前进
        odom_msg.twist.twist.angular.z = 0.1 # 模拟稍微有点转向
        self.odom_pub.publish(odom_msg)

        # ================= 发布 /plan (路径) =================
        # 路径通常发布频率较低，每1秒发一次即可 (10次循环发一次)
        if self.current_idx % 10 == 0:
            self.cached_path_msg.header.stamp = current_time
            self.path_pub.publish(self.cached_path_msg)

        # ================= 发布 /navigation/status (导航Json) =================
        # 计算进度
        total_points = len(self.sim_path_points)
        progress = (self.current_idx / total_points) * 100
        dist_remain = (total_points - self.current_idx) * 0.1 # 假设每点间距0.1m
        
        nav_status = {
            "navigation_state": "navigating",  # 状态: navigating, reached, idle
            "current_target_id": "waypoint_simulation",
            "current_waypoint_index": self.current_idx,
            "total_waypoints": total_points,
            "distance_remaining": float(f"{dist_remain:.2f}"),
            "progress": float(f"{progress:.1f}"),
            "average_speed": 0.5,
            "time_remaining": float(f"{dist_remain/0.5:.1f}"),
            "error_msg": ""
        }
        self.nav_status_pub.publish(String(data=json.dumps(nav_status)))

        # ================= 发布 /robot_status_processed (系统Json) =================
        # 模拟耗电
        self.battery_level = max(0.0, self.battery_level - 0.005)
        if self.battery_level <= 0: self.battery_level = 100.0
        
        sys_status = {
            "battery_level": float(f"{self.battery_level:.1f}"),
            "battery_status": "discharging",
            "wifi_signal": -60 + random.randint(-5, 5), # 模拟信号波动
            "cpu_usage": 15.2,
            "robot_mode": "autonomous",
            "error_count": 0,
            "emergency_stop": False
        }
        self.sys_status_pub.publish(String(data=json.dumps(sys_status)))
        
        # ================= 发布 /imu (惯性测量单元) =================
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation = pose_msg.pose.pose.orientation
        imu_msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = SyntheticDataPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('停止模拟...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()