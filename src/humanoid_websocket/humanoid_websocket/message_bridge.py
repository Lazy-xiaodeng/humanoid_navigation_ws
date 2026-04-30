#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
消息桥接节点
功能：将原始数据转换为标准的ROS消息格式，供其他ROS节点使用
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu,JointState
from geometry_msgs.msg import Quaternion, Vector3
import json
import math

class MessageBridge(Node):
    """消息桥接节点 - 负责数据格式转换"""
    
    def __init__(self):
        try:
            super().__init__('message_bridge_node')
            
            # 确保参数正确读取
            self.declare_parameter('update_rate', 10.0)
            self.declare_parameter('buffer_size', 1000)
            
            update_rate = self.get_parameter('update_rate').value
            buffer_size = self.get_parameter('buffer_size').value
            
            self.get_logger().info(f"更新率: {update_rate}Hz, 缓冲区大小: {buffer_size}")
            
            # 初始化其他组件
            #self.initialize_bridge_components()
            
            self.get_logger().info("✅ Message bridge node 初始化完成")

            # --- 新增：定义 31 个全关节的名称（必须与机器人固件/URDF一致） ---
            self.joint_names = [
    # 腿部 (1-12)
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", 
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", 
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    
    # 腰部 (13-15)
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    
    # 头部 (16-17)
    "head_yaw_joint", "head_pitch_joint",
    
    # 左臂 (18-24)
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", 
    "left_elbow_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint",
    
    # 右臂 (25-31)
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", 
    "right_elbow_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint"
]

            
        except Exception as e:
            self.get_logger().error(f"❌ Message bridge node 初始化失败: {e}")
            import traceback
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")
            raise  # 重新抛出异常以查看错误信息
        
        # 订阅原始数据话题
        self.setup_subscribers()
        
        # 发布标准ROS消息
        self.setup_publishers()
        
        # IMU坐标转换参数（根据机器人实际情况调整）
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('imu_transform.x', 0.0)
        self.declare_parameter('imu_transform.y', 0.0)
        self.declare_parameter('imu_transform.z', 0.0)
        
        self.get_logger().info('消息桥接节点启动')
    
    def setup_subscribers(self):
        """设置数据订阅器"""
        # 订阅原始IMU数据
        self.imu_raw_subscriber = self.create_subscription(
            String, '/imu_raw', self.imu_raw_callback, 10
        )
        
        # 订阅其他需要转换的原始数据（可选）
        self.robot_status_subscriber = self.create_subscription(
            String, '/robot_status_raw', self.robot_status_callback, 10
        )
        # --- 新增：订阅原始关节数据 ---
        self.joint_raw_sub = self.create_subscription(
            String, '/joint_states_raw', self.joint_states_raw_callback, 10
        )

    def setup_publishers(self):
        """设置标准消息发布器"""
        # 发布标准IMU消息（供导航功能包订阅）
        self.imu_publisher = self.create_publisher(
            Imu, '/imu', 10
        )
        
        # 发布处理后的机器人状态
        self.robot_status_publisher = self.create_publisher(
            String, '/robot_status_processed', 10
        )
        
        # --- 新增：发布标准关节状态消息 ---
        self.joint_states_pub = self.create_publisher(
            JointState, '/joint_states', 10 # 供您的自研功能包订阅
        )

    
    def imu_raw_callback(self, msg):
        """处理原始IMU数据，转换为标准ROS Imu消息"""
        try:
            # 解析原始IMU数据
            full_json = json.loads(msg.data)
            raw_data = full_json.get("raw_data", {})
            
            # 创建标准Imu消息
            imu_msg = Imu()
            
            # 设置消息头
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.get_parameter('imu_frame_id').value
            
            # 提取方向数据（四元数）
            quat_list = raw_data.get("quat", [1.0, 0.0, 0.0, 0.0])
            if isinstance(quat_list, list) and len(quat_list) >= 4:
            # 顺序转换：硬件(w,x,y,z) -> ROS(x,y,z,w)
               imu_msg.orientation.w = float(quat_list[0])
               imu_msg.orientation.x = float(quat_list[1])
               imu_msg.orientation.y = float(quat_list[2])
               imu_msg.orientation.z = float(quat_list[3])
            
            # 应用坐标转换（如果需要）
            self.apply_imu_transform(imu_msg.orientation)
            
            # 提取角速度数据
            gyro_list = raw_data.get("gyro", [0.0, 0.0, 0.0])
            if isinstance(gyro_list, list) and len(gyro_list) >= 3:
               imu_msg.angular_velocity.x = float(gyro_list[0])
               imu_msg.angular_velocity.y = float(gyro_list[1])
               imu_msg.angular_velocity.z = float(gyro_list[2])
            
            # 提取线加速度数据
            acc_list = raw_data.get("acc", [0.0, 0.0, 0.0])
            if isinstance(acc_list, list) and len(acc_list) >= 3:
               imu_msg.linear_acceleration.x = float(acc_list[0])
               imu_msg.linear_acceleration.y = float(acc_list[1])
               imu_msg.linear_acceleration.z = float(acc_list[2])
            
            # 设置协方差矩阵（可根据实际传感器精度调整）
            # 方向协方差（未知）
            imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # 角速度协方差
            imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            
            # 线加速度协方差
            imu_msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
            
            # 发布标准IMU消息
            self.imu_publisher.publish(imu_msg)
            
            self.get_logger().debug('已转换并发布标准IMU消息', throttle_duration_sec=5.0)
            
        except Exception as e:
            self.get_logger().error(f'转换IMU数据错误: {e}')
    
    def apply_imu_transform(self, orientation):
        """应用IMU坐标转换（根据机器人的安装方向调整）"""
        # 这里可以根据机器人的实际情况进行坐标转换
        # 例如，如果IMU安装方向不同，需要调整四元数
        # 当前版本不进行转换，根据实际情况调整参数
        
        # 获取转换参数
        transform_x = self.get_parameter('imu_transform.x').value
        transform_y = self.get_parameter('imu_transform.y').value
        transform_z = self.get_parameter('imu_transform.z').value
        
        # 如果有非零转换参数，应用坐标转换
        if any([transform_x, transform_y, transform_z]):
            # 这里实现坐标转换逻辑
            # 例如，使用四元数乘法进行旋转
            pass
    
    def robot_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            vals = data.get("values", {})
            health = data.get("health", {})
            latency = data.get("latency", 0)


            # --- 核心算法：将延迟映射为信号质量 ---
            # 这是一个标准的网络质量评估算法
            if latency < 0: latency = 5.0 # 容错处理：时钟同步微小误差
            
            signal_pct = 100
            signal_desc = "Excellent"
            
            if latency < 50:     # 极速：< 50ms
                signal_pct = 100
                signal_desc = "Excellent"
            elif latency < 150:  # 良好：50ms ~ 150ms
                signal_pct = 85
                signal_desc = "Good"
            elif latency < 500:  # 一般：150ms ~ 500ms
                signal_pct = 50
                signal_desc = "Fair"
            elif latency < 2000: # 极差：0.5s ~ 2s
                signal_pct = 20
                signal_desc = "Poor"
            else:                # 几乎断开
                signal_pct = 5
                signal_desc = "Unstable"

            # 提取 APP 最关心的核心字段
            battery_pct = vals.get("battery", 0)
            robot_state = vals.get("robot_status", "Unknown")
            is_estop = vals.get("estop", "OFF") == "ON"
            
            # 丰富处理后的信息
            processed = {
                "battery_level": int(battery_pct),
                "signal_quality": signal_pct,       # 👈 APP 显示进度条 (0-100)
                "signal_status": signal_desc,        # 👈 APP 显示文字 (良好)
                "network_latency": f"{int(latency)}ms", # 👈 诊断用
                "robot_state": robot_state,
                "power_info": {
                    "total_voltage": vals.get("bat_vol", 0.0),
                    "total_current": vals.get("bat_cur", 0.0),
                    "bat_temperature": vals.get("bat_temp0", 0.0)
                },
                "system_mode": vals.get("mode", "Unknown"),
                "estop_active": is_estop,
                "health_check": health,
                "timestamp": time.time()
            }

            # 发布给 data_integration_node
            processed_msg = String()
            processed_msg.data = json.dumps(processed, ensure_ascii = False)
            self.robot_status_publisher.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f"Bridge 处理错误: {e}")

    def calculate_battery_health(self, battery_level):
        """计算电池健康状态"""
        if battery_level > 80:
            return "excellent"
        elif battery_level > 60:
            return "good"
        elif battery_level > 40:
            return "fair"
        elif battery_level > 20:
            return "poor"
        else:
            return "critical"
    
    def estimate_remaining_time(self, battery_level):
        """估算剩余使用时间（分钟）"""
        # 简单的线性估算，实际应根据机器人功耗调整
        estimated_minutes = battery_level * 2  # 假设每1%电量可用2分钟
        return max(0, estimated_minutes)
    
    def calculate_signal_quality(self, signal_strength):
        """计算信号质量"""
        if signal_strength > 80:
            return "excellent"
        elif signal_strength > 60:
            return "good"
        elif signal_strength > 40:
            return "fair"
        elif signal_strength > 20:
            return "poor"
        else:
            return "disconnected"

    def joint_states_raw_callback(self, msg):
        """将机器人回传的 JSON 关节数据转换为标准 ROS JointState 消息"""
        try:
            # 1. 解析 WebSocket 传来的原始 JSON
            raw_data = json.loads(msg.data)
            
            # 2. 创建标准消息对象
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 3. 填充关节名称（从上面定义的 self.joint_names 获取）
            joint_msg.name = self.joint_names
            
            # 4. 提取数值（位置、速度、扭矩）
            # 注意：如果回传的数据长度不满足 31，此处会自动填充 0 防止算法崩溃
            positions = raw_data.get("positions", []) or raw_data.get("q", [])
            velocities = raw_data.get("velocities", []) or raw_data.get("dq", [])
            efforts = raw_data.get("efforts", []) or raw_data.get("tau", [])
            
            # 补齐长度到 31
            joint_msg.position = [float(p) for p in positions] + [0.0] * (31 - len(positions))
            joint_msg.velocity = [float(v) for v in velocities] + [0.0] * (31 - len(velocities))
            joint_msg.effort = [float(e) for e in efforts] + [0.0] * (31 - len(efforts))
            
            # 5. 发布到 /joint_states 话题
            self.joint_states_pub.publish(joint_msg)
            
        except Exception as e:
            self.get_logger().error(f'解析关节数据失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge_node = MessageBridge()
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()