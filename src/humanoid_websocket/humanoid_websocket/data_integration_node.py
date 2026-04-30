#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一数据整合节点 
功能：订阅各类ROS话题，转换为统一JSON格式，支持按需获取和主动推送
集成了统一的订阅管理器
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu
import json
import time
import uuid
import threading
from typing import Dict, Any, List, Optional
from concurrent.futures import ThreadPoolExecutor
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory

# ==================== 统一的订阅管理器类 ====================
class SubscriptionManager:
    """统一的订阅管理器 """
    
    def __init__(self, logger=None):
        self.subscriptions = {}  # 数据结构: {data_type: {client_id: subscription_info}}
        self.logger = logger
        self.lock = threading.RLock()  # 线程安全锁
        
    def subscribe(self, client_id: str, data_types: List[str], 
                 frequency: float = 1.0, subscription_info: Dict = None) -> bool:
        """添加订阅"""
        with self.lock:
            try:
                for data_type in data_types:
                    if data_type not in self.subscriptions:
                        self.subscriptions[data_type] = {}
                    
                    self.subscriptions[data_type][client_id] = {
                        "frequency": frequency,
                        "last_push_time": 0,
                        "subscription_info": subscription_info or {},
                        "active": True,
                        "subscription_time": time.time()
                    }
                
                if self.logger:
                    self.logger.info(f'✅ 客户端 {client_id} 订阅了: {data_types}')
                return True
                
            except Exception as e:
                if self.logger:
                    self.logger.error(f'❌ 添加订阅失败: {e}')
                return False
    
    def unsubscribe(self, client_id: str, data_types: List[str] = None) -> bool:
        """取消订阅"""
        with self.lock:
            try:
                if not data_types:  # 取消所有订阅
                    for data_type in list(self.subscriptions.keys()):
                        if client_id in self.subscriptions[data_type]:
                            del self.subscriptions[data_type][client_id]
                    if self.logger:
                        self.logger.info(f'✅ 客户端 {client_id} 取消所有订阅')
                else:
                    for data_type in data_types:
                        if (data_type in self.subscriptions and 
                            client_id in self.subscriptions[data_type]):
                            del self.subscriptions[data_type][client_id]
                    if self.logger:
                        self.logger.info(f'✅ 客户端 {client_id} 取消订阅: {data_types}')
                
                return True
                
            except Exception as e:
                if self.logger:
                    self.logger.error(f'❌ 取消订阅失败: {e}')
                return False
    
    def get_subscribers(self, data_type: str, current_time: float = None) -> List[Dict]:
        """获取订阅指定数据类型的客户端列表（带频率控制）"""
        if current_time is None:
            current_time = time.time()
            
        with self.lock:
            if data_type not in self.subscriptions:
                return []
            
            active_subscribers = []
            for client_id, info in self.subscriptions[data_type].items():
                if not info.get("active", True):
                    continue
                
                # 检查推送频率
                time_since_last_push = current_time - info.get("last_push_time", 0)
                min_interval = 1.0 / info["frequency"] if info["frequency"] > 0 else float('inf')
                
                if time_since_last_push >= min_interval:
                    active_subscribers.append({
                        "client_id": client_id,
                        **info
                    })
            
            return active_subscribers
    
    def update_push_time(self, client_id: str, data_type: str, current_time: float = None):
        """更新最后推送时间"""
        if current_time is None:
            current_time = time.time()
            
        with self.lock:
            if (data_type in self.subscriptions and 
                client_id in self.subscriptions[data_type]):
                self.subscriptions[data_type][client_id]["last_push_time"] = current_time
    
    def get_client_subscriptions(self, client_id: str) -> List[str]:
        """获取客户端订阅的所有数据类型"""
        with self.lock:
            subscribed_types = []
            for data_type, clients in self.subscriptions.items():
                if client_id in clients:
                    subscribed_types.append(data_type)
            return subscribed_types
    
    def remove_client(self, client_id: str):
        """移除客户端的所有订阅"""
        with self.lock:
            self.unsubscribe(client_id)
    
    def get_statistics(self) -> Dict:
        """获取订阅统计信息"""
        with self.lock:
            stats = {
                "total_data_types": len(self.subscriptions),
                "total_subscriptions": 0,
                "clients_by_type": {}
            }
            
            for data_type, clients in self.subscriptions.items():
                stats["total_subscriptions"] += len(clients)
                stats["clients_by_type"][data_type] = len(clients)
                
            return stats
# ==================== 主节点类 ====================
class UnifiedDataIntegrationNode(Node):
    """
    统一数据整合节点
    负责：数据订阅、格式转换、存储管理、推送控制
    """
    
    def __init__(self):
        super().__init__('unified_data_integration_node')
        
        
        # ==================== 初始化配置 ====================
        self.protocol_version = "2.0"
        
        # 数据存储结构
        self.data_storage = {}           # 存储各类数据：{data_type: data}
        self.last_update_times = {}      # 最后更新时间：{data_type: timestamp}
        self.data_expiry_config = {      # 数据过期时间配置（秒）
            'robot_pose': 5.0,           # 定位数据5秒过期
            'odom_raw': 5.0,             # 里程计原始数据5秒过期
            'navigation_status': 10.0,   # 导航状态10秒过期
            'navigation_path': 30.0,     # 路径数据30秒过期
            'system_status': 60.0,       # 系统状态60秒过期
            'sensor_data': 2.0           # 传感器数据2秒过期
        }
        
        # 推送配置
        self.push_configs = {
            'robot_pose': {
                'frequency': 5.0,        # 5Hz推送频率
                'last_push_time': 0,
                'qos_levels': ['realtime', 'standard']
            },
            'navigation_status': {
                'frequency': 3.0,        # 3Hz推送频率
                'last_push_time': 0,
                'qos_levels': ['standard', 'low_priority']
            },
            'navigation_path': {
                'frequency': 0.5,        # 0.5Hz推送频率
                'last_push_time': 0,
                'qos_levels': ['standard']
            },
            'system_status': {
                'frequency': 0.2,        # 0.2Hz推送频率
                'last_push_time': 0,
                'qos_levels': ['low_priority']
            },
            'gesture_list':{
                'frequency': 0.1,        # 0.1Hz推送频率
                'last_push_time': 0,
                'qos_levels': ['standard']
            },
            'facial_gesture_list': {    
                'frequency': 0.1,       # 0.1Hz推送频率
                'last_push_time': 0,
                'qos_levels': ['standard']
            }
        }
        
        # 订阅管理
        self.subscription_manager = SubscriptionManager(logger=self.get_logger())
        self.client_subscriptions = {}
        # 线程安全锁
        self.data_lock = threading.RLock()
        self.subscription_lock = threading.RLock()
        
        # ==================== 设置ROS通信接口 ====================
        self.setup_ros_communication()
        
        # ==================== 启动定时任务 ====================
        self.setup_timers()

        # 在初始化完成后立即加载动作库和表情库
        self.load_gesture_list()
        self.load_facial_gesture_list()
        
        self.get_logger().info('🎯 统一数据整合节点启动完成')
        self.get_logger().info(f'📊 协议版本: {self.protocol_version}')
        self.get_logger().info(f'🔄 支持的数据类型: {list(self.push_configs.keys())}')
        self.initial_sync_done = False
        self.create_timer(2.0, self._delayed_initial_sync)


    def _delayed_initial_sync(self):
        """延迟2秒后给服务器同步动作库，执行完即销毁该定时器"""
        if not self.initial_sync_done:
           self.trigger_initial_sync()
           self.initial_sync_done = True 
    

    def setup_ros_communication(self):
        """设置ROS话题订阅和发布"""
        try:
            # ==================== 数据订阅器 ====================
            # 使用最佳effort QoS策略，确保实时性
            qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE
            )
            
            # 路径规划数据订阅（Nav2全局路径）
            self.create_subscription(
                Path, '/plan', 
                self.path_callback, qos_profile
            )
            
            # 导航状态数据订阅
            self.create_subscription(
                String, '/navigation/status', 
                self.navigation_status_callback, qos_profile
            )

            # 订阅导航命令确认消息
            self.navigation_ack_sub = self.create_subscription(
                String, '/navigation/acknowledgments', self.navigation_ack_callback, 10
            )
            
            # 里程计数据订阅（备用，用于调试或未来扩展）
            # 注意：主定位数据现在来自NDT的/pcl_pose话题
            self.create_subscription(
                Odometry, '/odom',
                self.odom_callback, qos_profile
            )

            # NDT定位数据订阅（主定位数据源）
            self.create_subscription(
                PoseWithCovarianceStamped, '/robot_realpose',
                self.real_pose_callback, qos_profile
            )
            
            # 机器人状态数据订阅
            self.create_subscription(
                String, '/robot_status_processed', 
                self.robot_status_callback, qos_profile
            )
            
            # IMU数据订阅（可选，用于内部处理）
            self.create_subscription(
                Imu, '/imu', 
                self.imu_callback, qos_profile
            )
            
            # 面部控制指令发布器
            self.facial_cmd_pub = self.create_publisher(
            String, '/robot/facial_raw_cmd', 10  # 与 facial_driver 订阅的话题一致
            )

            # ==================== 服务接口 ====================
            # WebSocket数据请求订阅
            self.data_request_sub = self.create_subscription(
                String, '/websocket/data_requests', 
                self.data_request_callback, 10
            )
            
            # 数据订阅管理订阅
            self.subscription_sub = self.create_subscription(
                String, '/websocket/data_subscriptions', 
                self.subscription_callback, 10
            )
            
            # ==================== 发布器 ====================
            # 数据响应发布器
            self.data_response_pub = self.create_publisher(
                String, '/integration/data_responses', 10
            )
            
            # 推送消息发布器
            self.push_message_pub = self.create_publisher(
                String, '/integration/push_messages', 10
            )
            
            # 订阅响应发布器
            self.subscription_response_pub = self.create_publisher(
                String, '/integration/subscription_responses', 10
            )
            
            self.get_logger().info('✅ ROS通信接口设置完成')
            
        except Exception as e:
            self.get_logger().error(f'❌ 设置ROS通信失败: {e}')
            raise
    
    def setup_timers(self):
        """设置定时任务"""
        # 数据推送定时器（10Hz检查）
        self.push_timer = self.create_timer(0.1, self.push_data_updates)
        
        # 数据清理定时器（1Hz检查过期数据）
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_expired_data)
        
        # 状态监控定时器（5秒一次）
        self.monitor_timer = self.create_timer(5.0, self.monitor_system_status)
        
        self.get_logger().info('✅ 定时任务设置完成')
    
    def generate_message_id(self, prefix: str = "msg") -> str:
        """生成唯一消息ID"""
        timestamp = int(time.time() * 1000)  # 毫秒级时间戳
        random_suffix = uuid.uuid4().hex[:6]  # 6位随机数
        return f"{prefix}_{timestamp}_{random_suffix}"
    
    def create_base_message(self, message_type: str, data_type: str, 
                          source: str = "data_integration", 
                          destination: str = "all") -> Dict[str, Any]:
        """创建基础消息结构"""
        return {
            "protocol_version": self.protocol_version,
            "message_id": self.generate_message_id(message_type),
            "timestamp": time.time(),
            "message_type": message_type,# request|response|push|subscription
            "data_type": data_type, # 数据类型标识
            "source": source,# 消息来源
            "destination": destination,# 消息目的地
            "data": {},# 业务数据载体
            "metadata": {# 元数据区
                "status": "success",# 操作状态
                "error_code": "",# 错误代码
                "error_message": "",# 错误描述
                "request_id": "",# 关联请求ID
                "data_freshness": 0.0, # 数据新鲜度（秒）
                "qos_level": "standard"# 服务质量等级
            }
        }
    
    # ==================== 数据回调处理函数 ====================
    
    def path_callback(self, msg: Path):
        """处理路径规划数据回调"""
        try:
            with self.data_lock:
                path_data = {
                    "path_id": f"path_{int(time.time())}",
                    "frame_id": str(msg.header.frame_id),
                    "timestamp": time.time(),
                    "total_length": 0.0,
                    "estimated_duration": 0.0,
                    "waypoint_count": len(msg.poses),
                    "path_poses": [],
                    "path_properties": {
                        "smoothness": "unknown",
                        "safety_level": "unknown",
                        "complexity": "unknown"
                    }
                }
                
                total_length = 0.0
                previous_point = None
                
                # 处理每个路径点
                for i, pose_stamped in enumerate(msg.poses):
                    current_point = pose_stamped.pose.position
                    
                    # 计算段长度
                    segment_length = 0.0
                    if previous_point is not None:
                        dx = current_point.x - previous_point.x
                        dy = current_point.y - previous_point.y
                        segment_length = math.sqrt(dx**2 + dy**2)
                    
                    total_length += segment_length
                    
                    pose_data = {
                        "sequence": i,
                        "position": {
                            "x": float(current_point.x),
                            "y": float(current_point.y),
                            "z": float(current_point.z)
                        },
                        "orientation": {
                            "x": float(pose_stamped.pose.orientation.x),
                            "y": float(pose_stamped.pose.orientation.y),
                            "z": float(pose_stamped.pose.orientation.z),
                            "w": float(pose_stamped.pose.orientation.w)
                        },
                        "segment_length": float(segment_length),
                        "cumulative_length": float(total_length)
                    }
                    path_data["path_poses"].append(pose_data)
                    previous_point = current_point
                
                path_data["total_length"] = float(total_length)
                path_data["estimated_duration"] = self.estimate_path_duration(total_length)
                path_data["path_properties"] = self.analyze_path_properties(path_data)
                
                # 存储数据
                self.data_storage['navigation_path'] = path_data
                self.last_update_times['navigation_path'] = time.time()
                
                self.get_logger().debug('🛣️ 路径数据已更新', throttle_duration_sec=5.0)
                
        except Exception as e:
            self.get_logger().error(f'❌ 处理路径数据错误: {e}')
    
    def navigation_status_callback(self, msg: String):
        """处理导航状态数据回调"""
        try:
            with self.data_lock:
                # 解析原始状态数据
                basic_status = json.loads(msg.data)
                
                # 获取相关数据用于增强状态
                current_pose = self.data_storage.get('robot_pose', {})
                current_path = self.data_storage.get('navigation_path', {})
                
                # 增强导航状态信息
                enhanced_status = {
                    **basic_status,
                    "current_pose": current_pose,
                    "current_path": current_path,
                    "progress_percentage": self.calculate_navigation_progress(basic_status, current_path),
                    "estimated_remaining_time": self.estimate_remaining_time(basic_status, current_pose, current_path),
                    "system_timestamp": time.time(),
                    "performance_metrics": self.calculate_performance_metrics(basic_status)
                }
                
                # 存储数据
                self.data_storage['navigation_status'] = enhanced_status
                self.last_update_times['navigation_status'] = time.time()
                
                self.get_logger().debug('🎯 导航状态已更新', throttle_duration_sec=2.0)
                
        except Exception as e:
            self.get_logger().error(f'❌ 处理导航状态错误: {e}')
    

    def navigation_ack_callback(self, msg: String):
        """处理导航命令确认消息，转发给APP（统一格式）"""
        try:
            ack_data = json.loads(msg.data)
            ack_type = ack_data.get("ack_type", "")
            status = ack_data.get("status", "")
            message = ack_data.get("message", "")
    
            # 使用统一消息格式
            push_msg = self.create_base_message(
               message_type="push",
               data_type="navigation_command_result",
               source="data_integration",
               destination="all"
            )
        
            # 填充业务数据
            push_msg["data"] = {
               "ack_type": ack_type,
               "status": status,
               "message": message,
               "timestamp": ack_data.get("timestamp", time.time())
            }
        
            # 填充元数据
            push_msg["metadata"]["status"] = status
            if status == "error":
                push_msg["metadata"]["error_code"] = "nav_error"
                push_msg["metadata"]["error_message"] = message
    
            # 发布推送消息
            push_str = String()
            push_str.data = json.dumps(push_msg, ensure_ascii=False)
            self.push_message_pub.publish(push_str)
    
            self.get_logger().info(f"������ 转发导航确认: {ack_type} - {status}")
    
        except Exception as e:
            self.get_logger().error(f'❌ 处理导航确认消息错误: {e}')

    def odom_callback(self, msg: Odometry):
        """处理 Fast-LIO 的非标准里程计数据，并转换为标准速度推送给 APP"""
        try:
            with self.data_lock:
                # --- 核心：坐标系重新映射 ---
            
                # 1. 线速度映射
                # 我们需要的前向速度 (Standard X) 实际上是 Fast-LIO 的 负Z方向
                # (因为 Fast-LIO 此时 Z 指向后)
                forward_velocity = -msg.twist.twist.linear.z
            
                # 2. 角速度映射
                # 我们需要的左转角速度 (Standard Z旋转) 实际上是绕标准 Up 轴转。
                # 你的标准 Up 轴对应 Fast-LIO 的 -Y 轴。
                # 所以标准的左转（正值）对应于 Fast-LIO 绕 Y 轴的 负旋转 (-angular.y)
                turn_velocity = -msg.twist.twist.angular.y

                # 3. 容错处理
                # 机器人静止时会有微小的漂移噪声（例如 0.001），建议做个消噪
                if abs(forward_velocity) < 0.01: forward_velocity = 0.0
                if abs(turn_velocity) < 0.01: turn_velocity = 0.0

                # 构造要推送给 App 的数据
                speed_data = {
                    "linear_x": round(float(forward_velocity), 3),  # 前向线速度 (m/s)
                    "angular_z": round(float(turn_velocity), 3),   # 左右转向速度 (rad/s)
                    "timestamp": time.time()
                }

                self.data_storage['robot_speed'] = speed_data
                self.last_update_times['robot_speed'] = time.time()

                # 调试日志（建议每5秒打印一次，确认映射是否符合直觉）
                self.get_logger().debug(
                    f"🚀 速度推送 - 前向: {speed_data['linear_x']} m/s, 转向: {speed_data['angular_z']} rad/s",
                    throttle_duration_sec=5.0
                )

        except Exception as e:
            self.get_logger().error(f'❌ 速度数据坐标转换错误: {e}')

    def real_pose_callback(self, msg: PoseWithCovarianceStamped):
        """处理NDT定位数据回调 —— 此函数现在是主定位数据源"""
        try:
            with self.data_lock:
                # ============ 提取位姿 ============
                pose_data = {
                    "position": {
                        "x": float(msg.pose.pose.position.x),
                        "y": float(msg.pose.pose.position.y),
                        "z": float(msg.pose.pose.position.z)
                    },
                    "orientation": {
                        "x": float(msg.pose.pose.orientation.x),
                        "y": float(msg.pose.pose.orientation.y),
                        "z": float(msg.pose.pose.orientation.z),
                        "w": float(msg.pose.pose.orientation.w)
                    },
                    "covariance": [float(x) for x in msg.pose.covariance],
                    "frame_id": str(msg.header.frame_id),
                    "timestamp": time.time(),
                    "pose_quality": self.estimate_pose_quality(msg.pose.covariance),
                    "location_confidence": self.calculate_confidence(msg.pose.covariance),
                    "source": "tf_realpose"  # 标记数据来源为NDT
                }

                # ============ 存储：robot_pose（供所有业务使用） ============
                self.data_storage['robot_pose'] = pose_data
                self.last_update_times['robot_pose'] = time.time()

                self.get_logger().debug(
                    f'📍 NDT定位数据已更新: [{pose_data["position"]["x"]:.2f}, '
                    f'{pose_data["position"]["y"]:.2f}, {pose_data["position"]["z"]:.2f}]',
                    throttle_duration_sec=5.0
                )

        except Exception as e:
            self.get_logger().error(f'❌ 处理NDT定位数据错误: {e}')
    
    def robot_status_callback(self, msg: String):
        """处理机器人状态数据回调"""
        try:
            with self.data_lock:
                robot_status = json.loads(msg.data)
                
                # 增强系统状态信息
                system_status = {
                    "battery_level": robot_status.get("battery_level", 0),
                    "signal_quality": robot_status.get("signal_quality", 0),  
                    "signal_status": robot_status.get("signal_status", "N/A"), 
                    "network_latency": robot_status.get("network_latency", "0ms"), 
                    "robot_status": robot_status.get("robot_state", "Unknown"),
                    
                    # 评估健康和运行状态
                    "system_health": self.assess_system_health(robot_status),
                    "operational_status": self.determine_operational_status(robot_status),
                    
                    # 将包含电压、电流、温控的详细信息放在 details 里
                    "details": robot_status, 
                    "timestamp": time.time()
                }
                
                # 存储数据
                self.data_storage['system_status'] = system_status
                self.last_update_times['system_status'] = time.time()
                
        except Exception as e:
            self.get_logger().error(f'❌ 处理机器人状态错误: {e}')
    
    def imu_callback(self, msg: Imu):
        """处理IMU数据回调（内部使用，不发送给APP）"""
        try:
            with self.data_lock:
                imu_data = {
                    "orientation": {
                        "x": float(msg.orientation.x),
                        "y": float(msg.orientation.y),
                        "z": float(msg.orientation.z),
                        "w": float(msg.orientation.w)
                    },
                    "angular_velocity": {
                        "x": float(msg.angular_velocity.x),
                        "y": float(msg.angular_velocity.y),
                        "z": float(msg.angular_velocity.z)
                    },
                    "linear_acceleration": {
                        "x": float(msg.linear_acceleration.x),
                        "y": float(msg.linear_acceleration.y),
                        "z": float(msg.linear_acceleration.z)
                    },
                    "timestamp": time.time()
                }
                
                # 存储数据（内部使用）
                self.data_storage['imu'] = imu_data
                self.last_update_times['imu'] = time.time()
                
        except Exception as e:
            self.get_logger().error(f'❌ 处理IMU数据错误: {e}')
    
    def load_gesture_list(self):
        """从 gestures.yaml 动态加载动作列表，并存入 data_storage"""
        try:
            yaml_path = os.path.join(
                get_package_share_directory('humanoid_locomotion'),
                'config',
                'gestures.yaml'
            )
            with open(yaml_path, 'r', encoding='utf-8') as f:
                 config = yaml.safe_load(f)
                 actions = config.get('actions', {})
        
            gesture_list = []
            
            # 动态遍历 YAML 文件
            for action_key, action_info in actions.items():
                if isinstance(action_info, dict):
                    gesture_list.append({
                        "id": action_key,  
                        "name": action_info.get("name", action_key),  
                        "type": action_info.get("type", "upper_body")
                    })
        
            # 存储到 data_storage
            self.data_storage['gesture_list'] = {
                 "timestamp": time.time(),
                 "gestures": gesture_list
            }
            self.last_update_times['gesture_list'] = time.time()
        
            self.get_logger().info(f'✅ 已从 YAML 动态加载 {len(gesture_list)} 个动作')
        
        except Exception as e:
            self.get_logger().error(f'❌ 加载动作库失败: {e}')
            self.data_storage['gesture_list'] = {"gestures": []}

    def load_facial_gesture_list(self):
        """从 facial_gestures.yaml 加载面部表情，存入 data_storage['facial_gesture_list']"""
        try:
            yaml_path = os.path.join(
            get_package_share_directory('humanoid_locomotion'), 
            'config', 'facial_gestures.yaml'
            )
        
            with open(yaml_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                gestures_raw = config.get('facial_gestures', {})
        
            # 中文映射（手动定义）
            name_map = {
              "idle": "待机", "surprised": "惊讶", "thinking": "思考",
              "speak_start": "说话开始", "speak_stop": "说话停止", "sleeping": "休眠"
            }
        
            facial_list = []
            for action_id in gestures_raw.keys():
                facial_list.append({
                    "id": action_id,
                    "name": name_map.get(action_id, action_id)
                })
            
            # 存入独立的数据位
            self.data_storage['facial_gesture_list'] = {
                "timestamp": time.time(),
                "gestures": facial_list,
                "metadata": config.get("metadata", {})
            }
            self.last_update_times['facial_gesture_list'] = time.time()
            self.get_logger().info(f'✅ 已独立加载 {len(facial_list)} 个面部表情')
        
        except Exception as e:
            self.get_logger().error(f'❌ 加载表情库失败: {e}')
    # ==================== 数据请求处理 ====================
    
    def data_request_callback(self, msg: String):
        """处理APP的数据请求"""
        try:
            request_msg = json.loads(msg.data)
            
            # 验证请求消息格式
            if not self.validate_request_message(request_msg):
                self.send_error_response(request_msg, "无效的请求格式")
                return
            
            # 解析请求参数
            client_id = request_msg.get("source", "unknown")
            request_id = request_msg.get("message_id", "")
            data_type = request_msg.get("data_type", "")
            request_params = request_msg.get("data", {})
            
            # 处理数据请求
            if request_msg.get("message_type") == "request":
                self.handle_data_request(client_id, request_id, data_type, request_params)
            else:
                self.send_error_response(request_msg, f"不支持的消息类型: {request_msg.get('message_type')}")
                
        except Exception as e:
            self.get_logger().error(f'❌ 处理数据请求错误: {e}')
    
    def handle_data_request(self, client_id: str, request_id: str, 
                          data_type: str, request_params: Dict):
        """处理具体的数据请求"""
        try:
            # 检查数据类型是否支持
            if data_type not in self.push_configs:
                self.send_specific_error(client_id, request_id, data_type, 
                                       "UNSUPPORTED_DATA_TYPE", f"不支持的数据类型: {data_type}")
                return
            
            # 拦截动作库请求，触发"热重载"
            if data_type == "gesture_list":
                self.get_logger().info('🔄 收到动作库请求，正在从文件热重载动作配置...')
                self.load_gesture_list()  # 重新读取 YAML 文件并更新内存
            
            # 检查数据是否可用
            if not self.is_data_available(data_type):
                self.send_specific_error(client_id, request_id, data_type,
                                       "DATA_UNAVAILABLE", f"数据暂不可用: {data_type}")
                return
            
            # 检查数据是否新鲜
            if not self.is_data_fresh(data_type):
                self.send_specific_error(client_id, request_id, data_type,
                                       "DATA_STALE", f"数据已过期: {data_type}")
                return
            
            # 准备响应数据
            response_data = self.prepare_data_response(data_type, request_params)
            
            # 发送响应
            self.send_data_response(client_id, request_id, data_type, response_data)
            
            self.get_logger().info(f'✅ 已响应数据请求: {data_type} -> {client_id}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理数据请求失败 {data_type}: {e}')
            self.send_specific_error(client_id, request_id, data_type,
                                   "PROCESSING_ERROR", f"处理请求时出错: {str(e)}")
    
    def prepare_data_response(self, data_type: str, request_params: Dict) -> Dict[str, Any]:
        """准备数据响应"""
        try:
            # 获取基础数据
            base_data = self.data_storage.get(data_type, {})
            
            if not base_data:
                return {}
            
            # 根据数据类型进行定制化处理
            if data_type == "robot_pose":
                return self.prepare_pose_response(base_data, request_params)
            elif data_type == "navigation_path":
                return self.prepare_path_response(base_data, request_params)
            elif data_type == "navigation_status":
                return self.prepare_status_response(base_data, request_params)
            elif data_type == "system_status":
                return self.prepare_system_response(base_data, request_params)
            elif data_type == "gesture_list":
                return self.data_storage.get('gesture_list', {"gestures": []})
            else:
                return base_data  # 默认返回原始数据
                
        except Exception as e:
            self.get_logger().error(f'❌ 准备数据响应失败 {data_type}: {e}')
            return {}
    
    def prepare_pose_response(self, pose: Dict, request_params: Dict) -> Dict:
        """准备定位数据响应"""
        try:
            detail_level = request_params.get("detail_level", "standard")
            
            if detail_level == "minimal":
                return {
                    "position": pose.get("position", {}),
                    "orientation": pose.get("orientation", {}),
                    "timestamp": pose.get("timestamp", 0)
                }
            elif detail_level == "standard":
                return pose  # 返回完整数据
            elif detail_level == "enhanced":
                return {
                    **pose,
                    "additional_info": {
                        "location_confidence": pose.get("location_confidence", 0),
                        "pose_quality": pose.get("pose_quality", "unknown"),
                        "coordinate_system": "map"
                    }
                }
            else:
                return pose
                
        except Exception as e:
            self.get_logger().error(f'❌ 准备定位响应失败: {e}')
            return pose  # 发生错误时返回原始数据
    
    def prepare_path_response(self, path_data: Dict, request_params: Dict) -> Dict:
        """准备路径数据响应"""
        try:
            simplify_factor = request_params.get("simplify_factor", 1)
            include_waypoints = request_params.get("include_waypoints", True)
            
            response = path_data.copy()
            
            # 路径简化
            if simplify_factor > 1 and "path_poses" in path_data:
                original_poses = path_data["path_poses"]
                if len(original_poses) > simplify_factor:
                    simplified_poses = []
                    step = max(1, len(original_poses) // simplify_factor)
                    for i in range(0, len(original_poses), step):
                        simplified_poses.append(original_poses[i])
                    response["path_poses"] = simplified_poses
                    response["simplified"] = True
                    response["original_waypoint_count"] = len(original_poses)
            
            # 是否包含路径点详情
            if not include_waypoints:
                response.pop("path_poses", None)
                response["summary_only"] = True
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'❌ 准备路径响应失败: {e}')
            return path_data  # 发生错误时返回原始数据
    
    def prepare_status_response(self, status_data: Dict, request_params: Dict) -> Dict:
        """准备状态数据响应"""
        try:
            include_details = request_params.get("include_details", True)
            include_performance = request_params.get("include_performance", True)
            
            if not include_details:
                # 返回简化状态
                return {
                    "navigation_state": status_data.get("navigation_state", "unknown"),
                    "progress_percentage": status_data.get("progress_percentage", 0),
                    "timestamp": status_data.get("system_timestamp", 0)
                }
            
            response = status_data.copy()
            
            # 性能数据可选
            if not include_performance and "performance_metrics" in response:
                response.pop("performance_metrics")
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'❌ 准备状态响应失败: {e}')
            return status_data
    
    def trigger_initial_sync(self):
        """向服务器推送初始动作库和表情库数据，使其写入 business_state"""
        try:
            for d_type in ['gesture_list', 'facial_gesture_list']:
                if d_type in self.data_storage:
                    # 构建主动推送消息，目标设为 "all"
                    msg_dict = self.create_base_message("push", d_type, "data_integration", "all")
                    msg_dict["data"] = self.data_storage[d_type]
                    
                    msg_ros = String()
                    msg_ros.data = json.dumps(msg_dict, ensure_ascii=False)
                    self.push_message_pub.publish(msg_ros)
            self.get_logger().info('📤 已触发初始动作库和表情库同步到服务器')
        except Exception as e:
            self.get_logger().error(f'❌ 初始同步推送失败: {e}')

    def prepare_system_response(self, system_data: Dict, request_params: Dict) -> Dict:
        """准备系统状态响应"""
        try:
        # 系统状态通常不需要特殊处理
            return system_data
        except Exception as e:
             self.get_logger().error(f'❌ 准备系统响应失败: {e}')
             return system_data
    
    # ==================== 订阅管理 ====================
    
    def subscription_callback(self, msg: String):
        """处理APP的数据订阅请求 - 使用新的订阅管理器"""
        try:
            subscription_msg = json.loads(msg.data)
            
            # 提取订阅信息
            client_id = subscription_msg.get("source", "unknown")
            action = subscription_msg.get("data", {}).get("action", "subscribe")
            data_types = subscription_msg.get("data", {}).get("data_types", [])
            frequency = subscription_msg.get("data", {}).get("push_frequency", 1.0)
            
            # 使用新的订阅管理器
            if action == "subscribe":
                success = self.subscription_manager.subscribe(
                    client_id, data_types, frequency, subscription_msg
                )
                message = "订阅成功" if success else "订阅失败"
            else:
                success = self.subscription_manager.unsubscribe(client_id, data_types)
                message = "取消订阅成功" if success else "取消订阅失败"
            
            # 发送响应
            self.send_subscription_response(subscription_msg, success, message)
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理订阅请求错误: {e}')
            self.send_subscription_response(subscription_msg, False, f"处理失败: {str(e)}")
    
    
    def handle_data_request(self, client_id: str, request_id: str, 
                          data_type: str, request_params: Dict):
        """处理具体的数据请求"""
        try:
            # 检查数据是否可用
            if not self.is_data_available(data_type):
                self.send_specific_error(client_id, request_id, data_type,
                                       "DATA_UNAVAILABLE", f"数据暂不可用: {data_type}")
                return
            
            # 准备响应数据
            response_data = self.prepare_data_response(data_type, request_params)
            
            # 发送响应
            self.send_data_response(client_id, request_id, data_type, response_data)
            
            self.get_logger().info(f'✅ 已响应数据请求: {data_type} -> {client_id}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理数据请求失败 {data_type}: {e}')
            self.send_specific_error(client_id, request_id, data_type,
                                   "PROCESSING_ERROR", f"处理请求时出错: {str(e)}")

    # ==================== 数据推送管理 ====================

    def push_data_updates(self):
        """主动推送数据更新给订阅的客户端 - 使用新的订阅管理器"""
        try:
            current_time = time.time()
            
            # 检查每种数据类型的推送条件
            for data_type, config in self.push_configs.items():
                # 检查数据是否可用和新鲜
                if not self.is_data_available(data_type) or not self.is_data_fresh(data_type):
                    continue
                
                # 检查推送频率
                time_since_last_push = current_time - config['last_push_time']
                push_interval = 1.0 / config['frequency'] if config['frequency'] > 0 else float('inf')
                
                if time_since_last_push >= push_interval:
                    # 执行推送
                    self.push_data_to_subscribers(data_type, current_time)
                    config['last_push_time'] = current_time
                    
        except Exception as e:
            self.get_logger().error(f'❌ 推送数据更新错误: {e}')
    
    def push_data_to_subscribers(self, data_type: str, current_time: float):
        """推送数据给订阅的客户端 - 使用新的订阅管理器"""
        try:
            # 使用新的订阅管理器获取订阅者
            subscribers = self.subscription_manager.get_subscribers(data_type, current_time)
            if not subscribers:
                return
        
            # 准备推送数据
            push_data = self.data_storage.get(data_type, {})
            if not push_data:
                return
        
            # 为每个订阅的客户端创建推送消息
            for client_info in subscribers:
                client_id = client_info["client_id"]
                
                # 创建推送消息
                push_message = self.create_push_message(data_type, push_data, client_id, current_time)
                
                # 发布推送消息
                self.publish_push_message(push_message)
                
                # 更新最后推送时间
                self.subscription_manager.update_push_time(client_id, data_type, current_time)
            
            self.get_logger().debug(f'📤 推送 {data_type} 给 {len(subscribers)} 个客户端', 
                               throttle_duration_sec=10.0)
        
        except Exception as e:
            self.get_logger().error(f'❌ 推送数据给订阅者错误 {data_type}: {e}')
    
    def create_push_message(self, data_type: str, data:Dict, 
                           client_id: str, timestamp: float) -> Dict[str, Any]:
        """创建推送消息"""
        try:
            message = self.create_base_message("push", data_type, "data_integration", client_id)
            message["timestamp"] = timestamp
            message["data"] = data
            
            # 添加推送特定元数据
            message["metadata"].update({
                "push_reason": "periodic_update",
                "data_freshness": timestamp - self.last_update_times.get(data_type, timestamp),
                "subscription_count": len(self.subscription_manager.get_subscribers(data_type, timestamp))
            })
            
            return message
            
        except Exception as e:
            self.get_logger().error(f'❌ 创建推送消息错误: {e}')
            return {}

    # ==================== 消息发布函数 ====================

    def send_data_response(self, client_id: str, request_id: str, 
                      data_type: str, response: Dict):
        """发送数据响应给客户端"""
        try:
            message = self.create_base_message("response", data_type, "data_integration", client_id)
            message["data"] = response
            message["metadata"]["request_id"] = request_id
            message["metadata"]["data_freshness"] = time.time() - self.last_update_times.get(data_type, 0)
        
            # 发布消息
            self.publish_data_response(message)
        
        except Exception as e:
            self.get_logger().error(f'❌ 发送数据响应错误: {e}')

    def send_subscription_response(self, original_msg: Dict, success: bool, message: str = ""):
        """发送订阅操作响应"""
        try:
            response = self.create_base_message(
                "response", "subscription_manage", 
                "data_integration", original_msg.get("source", "unknown")
            )
            
            response["metadata"]["request_id"] = original_msg.get("message_id", "")
            response["metadata"]["status"] = "success" if success else "error"
            
            if not success:
                response["metadata"].update({
                    "error_code": "SUBSCRIPTION_FAILED",
                    "error_message": message
                })
            
            response["data"] = {
                "action": original_msg.get("data", {}).get("action", "unknown"),
                "data_types": original_msg.get("data", {}).get("data_types", []),
                "result": "success" if success else "failed",
                "message": message
            }
            
            self.publish_subscription_response(response)
            
        except Exception as e:
            self.get_logger().error(f'❌ 发送订阅响应错误: {e}')

    def send_error_response(self, original_msg: Dict, error_message: str, error_code: str = "PROCESSING_ERROR"):
        """发送错误响应"""
        try:
            if not original_msg:
                return
                
            response = self.create_base_message(
                "response", original_msg.get("data_type", "unknown"),
                "data_integration", original_msg.get("source", "unknown")
            )
            
            response["metadata"].update({
                "status": "error",
                "error_code": error_code,
                "error_message": error_message,
                "request_id": original_msg.get("message_id", "")
            })
            
            self.publish_data_response(response)
            
        except Exception as e:
            self.get_logger().error(f'❌ 发送错误响应错误: {e}')

    def send_specific_error(self, client_id: str, request_id: str, 
                           data_type: str, error_code: str, error_message: str):
        """发送特定的错误响应"""
        try:
            response = self.create_base_message("response", data_type, "data_integration", client_id)
            response["metadata"].update({
                "status": "error",
                "error_code": error_code,
                "error_message": error_message,
                "request_id": request_id
            })
            
            self.publish_data_response(response)
            
        except Exception as e:
            self.get_logger().error(f'❌ 发送特定错误响应错误: {e}')

    def publish_data_response(self, message: Dict):
        """发布数据响应消息"""
        try:
            msg = String()
            msg.data = json.dumps(message, ensure_ascii=False, separators=(',', ':'))
            self.data_response_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'❌ 发布数据响应错误: {e}')

    def publish_push_message(self, message: Dict):
        """发布推送消息"""
        try:
            msg = String()
            msg.data = json.dumps(message, ensure_ascii=False, separators=(',', ':'))
            self.push_message_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'❌ 发布推送消息错误: {e}')

    def publish_subscription_response(self, message: Dict):
        """发布订阅响应消息"""
        try:
            msg = String()
            msg.data = json.dumps(message, ensure_ascii=False, separators=(',', ':'))
            self.subscription_response_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'❌ 发布订阅响应错误: {e}')

    # ==================== 验证函数 ====================

    def validate_request_message(self, message: Dict) -> bool:
        """验证请求消息格式"""
        try:
            required_fields = ["protocol_version", "message_id", "message_type", "data_type", "source"]
            
            for field in required_fields:
                if field not in message:
                    self.get_logger().error(f'❌ 请求消息缺少必需字段: {field}')
                    return False
            
            if message.get("message_type") not in ["request", "subscription"]:
                self.get_logger().error(f'❌ 不支持的消息类型: {message.get("message_type")}')
                return False
            
            if message.get("protocol_version") != self.protocol_version:
                self.get_logger().warning(f'⚠️ 协议版本不匹配: {message.get("protocol_version")}')
                # 不返回False，允许不同版本间的兼容
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ 验证请求消息错误: {e}')
            return False

    def validate_subscription_message(self, message: Dict) -> bool:
        """验证订阅消息格式"""
        try:
            if not self.validate_request_message(message):
                return False
            
            if "data" not in message:
                self.get_logger().error('❌ 订阅消息缺少data字段')
                return False
            
            data = message.get("data", {})
            if "action" not in data or "data_types" not in data:
                self.get_logger().error('❌ 订阅消息缺少action或data_types字段')
                return False
            
            if data["action"] not in ["subscribe", "unsubscribe"]:
                self.get_logger().error(f'❌ 无效的订阅操作: {data["action"]}')
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ 验证订阅消息错误: {e}')
            return False

    # ==================== 工具函数 ====================

    def is_data_available(self, data_type: str) -> bool:
        """检查数据是否可用"""
        return data_type in self.data_storage and self.data_storage[data_type] is not None

    def is_data_fresh(self, data_type: str) -> bool:
        """检查数据是否新鲜（未过期）"""
        if data_type not in self.last_update_times:
            return False
        
        if data_type not in self.data_expiry_config:
            return True  # 没有配置过期时间的数据视为永久新鲜
        
        expiry_time = self.data_expiry_config[data_type]
        time_since_update = time.time() - self.last_update_times[data_type]
        
        return time_since_update <= expiry_time

    def estimate_pose_quality(self, covariance: List) -> str:
        """估计定位质量"""
        try:
            if not covariance or len(covariance) < 36:
                return "unknown"
            
            # 提取位置协方差（前3个对角线元素）
            pos_variance = max(covariance[0], covariance[7], covariance[14])  # x, y, z的方差
            
            if pos_variance < 0.01:  # 10cm精度
                return "excellent"
            elif pos_variance < 0.1:  # 1m精度
                return "good"
            elif pos_variance < 1.0:  # 10m精度
                return "fair"
            else:
                return "poor"
                
        except Exception as e:
            self.get_logger().debug(f'估计定位质量错误: {e}')
            return "unknown"

    def calculate_confidence(self, covariance: List) -> float:
        """计算定位置信度"""
        try:
            if not covariance or len(covariance) < 36:
                return 0.0
            
            pos_variance = max(covariance[0], covariance[7], covariance[14])
            
            # 将方差转换为置信度（方差越小，置信度越高）
            confidence = 1.0 / (1.0 + pos_variance * 10.0)
            return max(0.0, min(1.0, confidence))
            
        except Exception as e:
            self.get_logger().debug(f'计算定位置信度错误: {e}')
            return 0.5

    def estimate_path_duration(self, path_length: float) -> float:
        """估算路径耗时"""
        avg_speed = 0.5  # 平均速度 0.5 m/s (可配置)
        return path_length / avg_speed if avg_speed > 0 else 0.0

    def analyze_path_properties(self, path_data: Dict) -> Dict:
        """
    分析路径属性
    基于路径点数据计算平滑度、安全级别和复杂度
    """
        try:
            angle_diff = 0.0
            curvature = 0.0
            
            poses = path_data.get("path_poses", [])
            total_distance = 0.0
            if len(poses) < 2:
               return {
                "smoothness": "unknown", 
                "safety_level": "unknown", 
                "complexity": "simple",
                "turn_count": 0,
                "total_distance": 0.0,
                "avg_segment_length": 0.0
               }
        
        # 计算路径基本属性
            total_distance = 0.0
            segment_lengths = []
            angles = []
        
        # 提取所有点的位置
            points = []
            for pose in poses:
                pos = pose.get("position", {})
                if pos:
                    points.append((pos.get("x", 0.0), pos.get("y", 0.0)))
        
        # 计算路径长度和角度变化
            turn_count = 0
            max_angle_change = 0.0
        
            for i in range(len(points) - 1):
            # 计算段长度
                x1, y1 = points[i]
                x2, y2 = points[i + 1]
                dx = x2 - x1
                dy = y2 - y1
                segment_length = math.sqrt(dx**2 + dy**2)
            
                if segment_length > 0:
                    segment_lengths.append(segment_length)
                    total_distance += segment_length
                
                # 计算方向角
                    angle = math.atan2(dy, dx)
                    angles.append(angle)
                
                # 计算角度变化（如果至少有2个角度）
                    if len(angles) >= 2:
                        angle_diff = abs(angles[-1] - angles[-2])
                    # 规范化角度差到[-π, π]
                        while angle_diff > math.pi:
                              angle_diff -= 2 * math.pi
                        while angle_diff < -math.pi:
                              angle_diff += 2 * math.pi
                        angle_diff = abs(angle_diff)
                    
                    # 如果角度变化大于阈值（30度），计为一个转弯
                    if angle_diff > math.radians(30):
                        turn_count += 1
                    
                    max_angle_change = max(max_angle_change, angle_diff)
        
        # 计算平均段长度
            avg_segment_length = sum(segment_lengths) / len(segment_lengths) if segment_lengths else 0.0
        
        # 计算平滑度
            smoothness = self.calculate_smoothness(segment_lengths, angles)
        
        # 计算复杂度
            complexity = self.calculate_complexity(
                len(points), turn_count, total_distance, avg_segment_length
            )
        
        # 计算安全级别（基于路径长度和转弯次数）
            safety_level = self.calculate_safety_level(segment_lengths, turn_count, avg_segment_length)
        
            return {
            "smoothness": smoothness,
            "safety_level": safety_level,
            "complexity": complexity,
            "turn_count": turn_count,
            "total_distance": total_distance,
            "avg_segment_length": avg_segment_length,
            "max_angle_change": math.degrees(max_angle_change) if max_angle_change > 0 else 0.0,
            "segment_count": len(segment_lengths)
            }
        
        except Exception as e:
            self.get_logger().error(f'分析路径属性错误: {e}')
            return {
            "smoothness": "unknown", 
            "safety_level": "unknown", 
            "complexity": "unknown",
            "turn_count": 0,
            "total_distance": 0.0,
            "avg_segment_length": 0.0
            }

    def calculate_smoothness(self, segment_lengths: List[float], angles: List[float]) -> str:
        """计算路径平滑度"""
        try:
            if not segment_lengths or len(segment_lengths) < 2:
               return "unknown"
        
        # 计算段长度标准差
            length_std = self.calculate_std(segment_lengths)
        
        # 计算角度变化标准差
            angle_changes = []
            for i in range(1, len(angles)):
                 angle_diff = abs(angles[i] - angles[i-1])
            # 规范化角度差
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            angle_changes.append(abs(angle_diff))
        
            if angle_changes:
                angle_std = self.calculate_std(angle_changes)
            else:
                angle_std = 0.0
        
        # 评估平滑度
            length_threshold = 0.5  # 米
            angle_threshold = math.radians(20)  # 20度
        
            if length_std < length_threshold and (not angle_changes or angle_std < angle_threshold):
                return "smooth"
            elif length_std < length_threshold * 2 and (not angle_changes or angle_std < angle_threshold * 2):
                return "moderate"
            else:
                return "rough"
            
        except Exception as e:
            self.get_logger().debug(f'计算平滑度错误: {e}')
            return "unknown"
    def calculate_complexity(self, point_count: int, turn_count: int, 
                        total_distance: float, avg_segment_length: float) -> str:
        """计算路径复杂度"""
        try:
        # 基于路径点数量、转弯次数、路径长度和平均段长度评估复杂度
            complexity_score = 0
        
        # 路径点数量因素
            if point_count > 50:
               complexity_score += 3
            elif point_count > 20:
               complexity_score += 2
            elif point_count > 10:
               complexity_score += 1
        
        # 转弯次数因素
            if turn_count > 10:
               complexity_score += 3
            elif turn_count > 5:
               complexity_score += 2
            elif turn_count > 2:
               complexity_score += 1
        
        # 路径长度因素
            if total_distance > 50:
               complexity_score += 2
            elif total_distance > 20:
               complexity_score += 1
        
        # 段长度变化因素
            if avg_segment_length < 0.5:  # 非常短的段
               complexity_score += 2
            elif avg_segment_length < 1.0:  # 较短的段
               complexity_score += 1
        
        # 评估复杂度等级
            if complexity_score >= 5:
               return "complex"
            elif complexity_score >= 3:
               return "moderate"
            else:
               return "simple"
            
        except Exception as e:
            self.get_logger().debug(f'计算复杂度错误: {e}')
            return "unknown"

    def calculate_safety_level(self, segment_lengths: List[float], turn_count: int, 
                          avg_segment_length: float) -> str:
        """计算路径安全级别"""
        try:
            if not segment_lengths:
              return "unknown"
        
        # 计算段长度的变异系数
            if len(segment_lengths) > 1:
                mean_length = sum(segment_lengths) / len(segment_lengths)
                variance = sum((x - mean_length) ** 2 for x in segment_lengths) / len(segment_lengths)
                std_dev = math.sqrt(variance)
                coeff_variation = std_dev / mean_length if mean_length > 0 else 0.0
            else:
                coeff_variation = 0.0
        
        # 评估安全级别
        # 安全因素：段长度变化小、转弯少、平均段长度适中
            safety_score = 0
        
        # 段长度稳定性
            if coeff_variation < 0.3:  # 变化较小
               safety_score += 2
            elif coeff_variation < 0.6:  # 变化中等
               safety_score += 1
        
        # 转弯次数
            turn_density = turn_count / len(segment_lengths) if segment_lengths else 0
            if turn_density < 0.1:  # 转弯密度低
                safety_score += 2
            elif turn_density < 0.3:  # 转弯密度中等
                safety_score += 1
        
        # 平均段长度
            if 1.0 <= avg_segment_length <= 5.0:  # 理想长度
                safety_score += 2
            elif 0.5 <= avg_segment_length < 1.0 or avg_segment_length > 5.0:  # 可接受范围
                safety_score += 1
        
        # 评估安全等级
            if safety_score >= 5:
               return "safe"
            elif safety_score >= 3:
               return "caution"
            else:
               return "danger"
            
        except Exception as e:
             self.get_logger().debug(f'计算安全级别错误: {e}')
             return "unknown"

    def calculate_std(self, values: List[float]) -> float:
        """计算标准差"""
        try:
            if not values:
               return 0.0
        
            mean = sum(values) / len(values)
            variance = sum((x - mean) ** 2 for x in values) / len(values)
            return math.sqrt(variance)
        except Exception as e:
            self.get_logger().debug(f'计算标准差错误: {e}')
            return 0.0

    def estimate_remaining_time(self, status_data: Dict, pose_data: Dict, path_data: Dict) -> float:
        """估算剩余时间（基于剩余距离和速度）"""
        try:
           # 获取当前速度（从里程计或状态数据）
            current_speed = status_data.get("current_speed", 0.0)
        
        # 如果状态数据中没有速度，尝试从里程计数据获取
            if current_speed == 0 and "odometry" in self.data_storage:
                odom = self.data_storage.get("odometry", {})
                velocity = odom.get("velocity", {}).get("linear", {})
                current_speed = math.sqrt(
                   velocity.get("x", 0.0)**2 + 
                   velocity.get("y", 0.0)**2 + 
                   velocity.get("z", 0.0)**2
                )
        
        # 如果仍然没有速度数据，使用默认值
            if current_speed == 0:
                current_speed = 0.5  # 默认0.5 m/s
        
        # 计算剩余距离
            remaining_distance = status_data.get("remaining_distance", 0.0)
        
        # 如果状态数据中没有剩余距离，尝试从路径数据计算
            if remaining_distance == 0 and path_data and "path_poses" in path_data:
            # 获取当前位姿
                current_pose = pose_data.get("position", {})
                current_x = current_pose.get("x", 0.0)
                current_y = current_pose.get("y", 0.0)
            
            # 获取当前路点索引
                current_index = status_data.get("current_waypoint_index", 0)
                poses = path_data.get("path_poses", [])
            
            # 计算到下一个路点的距离
            if current_index < len(poses):
                next_pose = poses[current_index].get("position", {})
                next_x = next_pose.get("x", 0.0)
                next_y = next_pose.get("y", 0.0)
                
                remaining_distance = math.sqrt(
                    (next_x - current_x)**2 + (next_y - current_y)**2
                )
                
                # 加上后续路点的总距离
                if current_index + 1 < len(poses):
                    for i in range(current_index + 1, len(poses) - 1):
                        p1 = poses[i].get("position", {})
                        p2 = poses[i + 1].get("position", {})
                        
                        x1, y1 = p1.get("x", 0.0), p1.get("y", 0.0)
                        x2, y2 = p2.get("x", 0.0), p2.get("y", 0.0)
                        
                        segment_distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                        remaining_distance += segment_distance
        
        # 计算剩余时间（考虑加速度、减速等因素）
            if current_speed > 0 and remaining_distance > 0:
            # 基本时间计算：距离/速度
                base_time = remaining_distance / current_speed
            
            # 考虑转弯因素（每个转弯增加2秒）
                turn_count = path_data.get("path_properties", {}).get("turn_count", 0)
                turn_time = turn_count * 2.0
            
            # 考虑路径复杂度
                complexity = path_data.get("path_properties", {}).get("complexity", "simple")
                if complexity == "complex":
                    complexity_factor = 1.5
                elif complexity == "moderate":
                    complexity_factor = 1.2
                else:
                    complexity_factor = 1.0
            
            # 最终估算时间
                estimated_time = base_time * complexity_factor + turn_time
            
                return max(estimated_time, 0.0)
            else:
                return 0.0
            
        except Exception as e:
            self.get_logger().debug(f'估算剩余时间错误: {e}')
            return 0.0


    def calculate_navigation_progress(self, status_data: Dict, path_data: Dict) -> float:
        """计算导航进度百分比"""
        try:
            current_index = status_data.get("current_waypoint_index", 0)
            total_waypoints = status_data.get("total_waypoints", 1)
        
            if total_waypoints > 0:
                progress = (current_index / total_waypoints) * 100
                return min(100.0, max(0.0, progress))
            else:
            # 如果没有路点，检查是否使用距离计算
                current_distance = status_data.get("traveled_distance", 0.0)
                total_distance = status_data.get("total_distance", 0.0)
            
                if total_distance > 0:
                    progress = (current_distance / total_distance) * 100
                    return min(100.0, max(0.0, progress))
                else:
                    return 0.0  # 返回0.0而不是字符串
                
        except Exception as e:
            self.get_logger().debug(f'计算导航进度错误: {e}')
            return 0.0


    
    def calculate_performance_metrics(self, status_data: Dict) -> Dict:
        """计算性能指标"""
        try:
            return {
               "average_speed": status_data.get("average_speed", 0),
               "max_speed": status_data.get("max_speed", 0),
               "total_distance": status_data.get("total_distance", 0),
               "navigation_time": status_data.get("navigation_time", 0),
               "efficiency": status_data.get("efficiency", 0)
            }
        except Exception as e:
            self.get_logger().debug(f'计算性能指标错误: {e}')
        return {}

    def assess_system_health(self, robot_status: Dict) -> str:
        """评估系统健康状态"""
        try:
            battery_level = robot_status.get("battery_level", 100)
            error_count = robot_status.get("error_count", 0)
            
            if error_count > 0 or battery_level < 10:
                return "error"
            elif battery_level < 30:
                return "warning"
            else:
                return "normal"
        except Exception as e:
            self.get_logger().debug(f'评估系统健康错误: {e}')
            return "unknown"

    def determine_operational_status(self, robot_status: Dict) -> str:
        """确定运行状态"""
        try:
            if robot_status.get("emergency_stop", False):
                return "emergency_stop"
            elif robot_status.get("paused", False):
                return "paused"
            elif robot_status.get("navigating", False):
                return "navigating"
            else:
                return "idle"
        except Exception as e:
            self.get_logger().debug(f'确定运行状态错误: {e}')
            return "unknown"

    # ==================== 系统维护函数 ====================

    def cleanup_expired_data(self):
        """清理过期数据"""
        try:
            current_time = time.time()
            expired_types = []
            
            for data_type, last_update in self.last_update_times.items():
                expiry_time = self.data_expiry_config.get(data_type, float('inf'))
                if current_time - last_update > expiry_time:
                    expired_types.append(data_type)
            
            for data_type in expired_types:
                with self.data_lock:
                    if data_type in self.data_storage:
                        del self.data_storage[data_type]
                    if data_type in self.last_update_times:
                        del self.last_update_times[data_type]
                
                self.get_logger().info(f'🧹 清理过期数据: {data_type}')
                
        except Exception as e:
            self.get_logger().error(f'❌ 清理过期数据错误: {e}')

    def monitor_system_status(self):
        """监控系统状态"""
        try:
            # 检查数据可用性
            available_data = []
            for data_type in self.push_configs.keys():
                if self.is_data_available(data_type) and self.is_data_fresh(data_type):
                    available_data.append(data_type)
            
            # 统计订阅情况
            total_subscriptions = sum(len(subs) for subs in self.client_subscriptions.values())
            
            # 记录状态信息
            self.get_logger().info(
                f'📊 系统状态 - 可用数据: {len(available_data)}/{len(self.push_configs)} | '
                f'活跃订阅: {total_subscriptions} | '
                f'客户端数: {len(self.client_subscriptions)}',
                throttle_duration_sec=30.0  # 每30秒记录一次
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ 监控系统状态错误: {e}')

    def destroy_node(self):
        """节点销毁前的清理工作"""
        try:
            self.get_logger().info('🛑 开始销毁统一数据整合节点...')
            
            # 停止所有定时器
            if hasattr(self, 'push_timer'):
                self.push_timer.cancel()
            if hasattr(self, 'cleanup_timer'):
                self.cleanup_timer.cancel()
            if hasattr(self, 'monitor_timer'):
                self.monitor_timer.cancel()
            
            # 清理数据存储
            with self.data_lock:
                self.data_storage.clear()
                self.last_update_times.clear()
            
            with self.subscription_lock:
                self.client_subscriptions.clear()
            
            self.get_logger().info('✅ 统一数据整合节点销毁完成')
            
        except Exception as e:
            self.get_logger().error(f'❌ 销毁节点错误: {e}')
        finally:
            super().destroy_node()

# ==================== 主函数 ====================

def main(args=None):
    """主函数 - 启动统一数据整合节点"""
    rclpy.init(args=args)
    
    try:
        # 创建节点实例
        node = UnifiedDataIntegrationNode()
        
        # 设置优雅退出处理
        def signal_handler(signum, frame):
            node.get_logger().info('📡 收到退出信号，开始关闭节点...')
            node.destroy_node()
            rclpy.shutdown()
        
        import signal
        signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C
        signal.signal(signal.SIGTERM, signal_handler) # 终止信号
        
        # 启动节点
        node.get_logger().info('🚀 启动统一数据整合节点主循环...')
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('⚠️ 收到键盘中断信号')
    except Exception as e:
        node.get_logger().error(f'💥 节点运行错误: {e}')
    finally:
        # 确保资源清理
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print('✅ 统一数据整合节点已关闭')

if __name__ == '__main__':
    main()