#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动态路点管理器 - 仅负责路点管理
功能：管理路点数据，接收APP命令，向状态管理器发送导航请求
不执行实际导航，不发布导航状态
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
from enum import Enum
from typing import Dict, List, Optional, Any
import os
from datetime import datetime
import uuid

# ========== 枚举定义 ==========
class WaypointType(Enum):
    """点位类型枚举"""
    NAVIGATION_TARGET = "navigation_target"
    EXHIBITION_POINT = "exhibition_point" 
    OBSTACLE_POINT = "obstacle_point"
    CHARGING_POINT = "charging_point"
    REST_POINT = "rest_point"
    LANDMARK_POINT = "landmark_point"

class NavigationMode(Enum):
    """导航模式枚举"""
    SINGLE_POINT = "single_point"
    MULTI_POINT = "multi_point"
    EXHIBITION_TOUR = "exhibition_tour"
    CHARGING_ROUTE = "charging_route"

# ========== 数据类定义 ==========
class WaypointData:
    """点位数据类"""
    def __init__(self, id: str, name: str, waypoint_type: WaypointType, 
                 position: List[float], orientation: List[float], 
                 frame_id: str = "map", properties: Dict[str, Any] = None):
        self.id = id
        self.name = name
        self.waypoint_type = waypoint_type
        self.position = position
        self.orientation = orientation
        self.frame_id = frame_id
        self.properties = properties or {}
        self.created_time = time.time()
        self.last_modified = time.time()
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式"""
        return {
            "id": self.id,
            "name": self.name,
            "type": self.waypoint_type.value,
            "position": self.position,
            "orientation": self.orientation,
            "frame_id": self.frame_id,
            "properties": self.properties,
            "created_time": self.created_time,
            "last_modified": self.last_modified
        }

class NavigationSequence:
    """导航序列类"""
    def __init__(self, id: str, name: str, waypoint_ids: List[str], 
                 sequence_type: str, properties: Dict[str, Any] = None):
        self.id = id
        self.name = name
        self.waypoint_ids = waypoint_ids
        self.sequence_type = sequence_type
        self.created_time = time.time()
        self.properties = properties or {}

class DynamicWaypointsManager(Node):
    """
    动态路点管理器节点 - 仅负责路点管理
    不执行导航，不发布导航状态
    """
    
    def __init__(self):
        super().__init__('dynamic_waypoints_manager')
        
        # ========== 参数声明 ==========
        self.declare_parameters(namespace='', parameters=[
            ('data_storage.enabled', True),
            ('data_storage.file_path', '/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json'),
            ('data_storage.auto_save_interval', 300.0),
        ])
        
        # ========== 数据存储 ==========
        self.waypoints: Dict[str, Dict[str, WaypointData]] = {
            wp_type.value: {} for wp_type in WaypointType
        }
        self.navigation_sequences: Dict[str, NavigationSequence] = {}
        
        # ========== 状态管理器通信 ==========
        self.navigation_request_pub = None
        self.waypoints_data_pub = None
        
        # ========== 设置ROS2通信 ==========
        self.setup_communication()
        
        # ========== 数据持久化 ==========
        self.setup_data_persistence()

        self.initial_waypoints_publish_count = 0
        self.initial_waypoints_publish_max = 5
        self.initial_waypoints_publish_timer = None
        if self.get_total_waypoints_count() > 0 or self.navigation_sequences:
            self.initial_waypoints_publish_timer = self.create_timer(
                1.0, self.publish_initial_waypoints_data
            )
        
        self.get_logger().info("动态路点管理器启动完成 - 仅负责路点管理")

    @staticmethod
    def normalize_waypoint_speed_properties(properties: Dict[str, Any]) -> tuple:
        """Validate waypoint speed and store it as properties.speed."""
        if not isinstance(properties, dict):
            return False, "properties 必须是对象", {}

        normalized = dict(properties)
        speed_keys = ("speed", "target_speed", "navigation_speed")
        speed_key = next((key for key in speed_keys if key in normalized), None)
        if speed_key is None:
            return True, "", normalized

        raw_speed = normalized.get(speed_key)
        if isinstance(raw_speed, bool) or not isinstance(raw_speed, (int, float)):
            return False, "路点速度 speed 必须是数字，单位 m/s", normalized

        speed = float(raw_speed)
        min_speed = 0.15
        max_speed = 1.0
        if speed < min_speed or speed > max_speed:
            return False, f"路点速度 speed 必须在 {min_speed:.2f}~{max_speed:.2f} m/s 范围内", normalized

        normalized["speed"] = speed
        return True, "", normalized
    
    def setup_communication(self):
        """设置ROS2通信接口"""
        # ========== 发布器 ==========
        # 发布导航请求给状态管理器（不包含状态）
        self.navigation_request_pub = self.create_publisher(
            String, '/navigation/requests', 10
        )
        
        # 发布点位数据给状态管理器
        self.waypoints_data_pub = self.create_publisher(
            String, '/navigation/waypoints_data', 10
        )
        
        # ========== 订阅器 ==========
        # 订阅APP的点位管理命令
        self.app_waypoint_sub = self.create_subscription(
            String, '/app/waypoint_command', self.app_waypoint_callback, 10
        )
        
        # 订阅APP的导航命令
        self.app_navigation_sub = self.create_subscription(
            String, '/app/navigation_command', self.app_navigation_callback, 10
        )
        
        # 订阅状态管理器的确认消息
        self.navigation_ack_sub = self.create_subscription(
            String, '/navigation/acknowledgments', self.navigation_ack_callback, 10
        )
    
    
    
    def app_waypoint_callback(self, msg: String):
        """处理APP的点位管理命令"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get("command_type", "")
            
            self.get_logger().info(f"收到APP点位命令: {command_type}")
            
            if command_type == "set_waypoint":
                self.handle_set_waypoint(command_data)
            elif command_type == "update_waypoint":
                self.handle_update_waypoint(command_data)
            elif command_type == "delete_waypoint":
                self.handle_delete_waypoint(command_data)
            elif command_type == "get_waypoints":
                self.handle_get_waypoints(command_data)
            elif command_type == "clear_waypoints":
                self.handle_clear_waypoints(command_data)
            else:
                self.get_logger().warning(f"未知的点位命令: {command_type}")
                self.send_app_response("error", f"未知命令: {command_type}")
                
        except Exception as e:
            self.get_logger().error(f"处理APP点位命令错误: {e}")
            self.send_app_response("error", f"处理命令失败: {str(e)}")
    
    def app_navigation_callback(self, msg: String):
        """处理APP的导航命令 - 只转发请求，不执行导航"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get("command_type", "")
            
            self.get_logger().info(f"收到APP导航命令: {command_type} - 转发给状态管理器")
            
            # 桥接层只做最小字段完整性校验，避免明显坏包进入状态机。
            # jump 合法性、task/transit 语义、播报上下文匹配仍由 navigation_state_manager 负责。
            if not self.validate_navigation_command(command_data):
                self.send_app_response("error", f"导航命令校验失败: {command_type}")
                return

            # 所有业务 ID 在进入状态机前统一为字符串，避免 15 和 "15" 比较不相等。
            command_data = self.normalize_navigation_command(command_data)

            # 转发导航请求给状态管理器
            self.send_navigation_request(command_data)
            
        except Exception as e:
            self.get_logger().error(f"处理APP导航命令错误: {e}")
            self.send_app_response("error", f"处理导航命令失败: {str(e)}")
    
    def validate_navigation_command(self, command_data: Dict[str, Any]) -> bool:
        """验证导航命令的有效性"""
        try:
            command_type = command_data.get("command_type", "")
            
            if command_type in ["start_single_navigation", "start_multi_point_navigation", "start_exhibition_navigation"]:
                # 验证必要的参数
                if command_type == "start_single_navigation":
                    waypoint_id = command_data.get("waypoint_id")
                    if not waypoint_id or not self.find_waypoint_by_id(waypoint_id):
                        self.get_logger().error(f"单点导航验证失败: 点位 '{waypoint_id}' 不存在")
                        return False
                elif command_type == "start_multi_point_navigation":  # 新增多点导航验证
                    waypoint_ids = command_data.get("waypoint_ids", [])
                    if not waypoint_ids:
                        self.get_logger().error("多点导航验证失败: 点位列表为空")
                        return False
                # 验证所有点位存在
                    for wp_id in waypoint_ids:
                        if not self.find_waypoint_by_id(wp_id):
                           self.get_logger().error(f"多点导航验证失败: 点位 '{wp_id}' 不存在")
                           return False
                elif command_type == "start_exhibition_navigation":
                    exhibition_points = list(self.waypoints[WaypointType.EXHIBITION_POINT.value].keys())
                    if not exhibition_points:
                        self.get_logger().error("展台导航验证失败: 没有设置展台点")
                        return False

            elif command_type == "start_route_task":
                # route task 的业务 ack 统一由 navigation_state_manager 通过
                # navigation_command_result 返回。桥接层不再硬拒绝缺字段，
                # 只记录明显问题并继续转发，让状态机返回 missing_task_session_id、
                # missing_route_id、invalid_route_waypoints 等精确错误码。
                # 这样 APP 不需要同时等待 waypoint_response 和 navigation_command_result 两套业务结果。
                if not command_data.get("task_session_id"):
                    self.get_logger().warning("路线任务启动字段缺失: task_session_id，继续转发给状态机返回业务 ack")
                if not command_data.get("route_id"):
                    self.get_logger().warning("路线任务启动字段缺失: route_id，继续转发给状态机返回业务 ack")
                route_waypoints = command_data.get("route_waypoints", [])
                if not isinstance(route_waypoints, list) or not route_waypoints:
                    self.get_logger().warning("路线任务启动字段异常: route_waypoints 为空或不是数组，继续转发给状态机返回业务 ack")

            elif command_type == "jump_to_waypoint":
                # jump 的会话、目标点、是否允许跳到该点，都由状态机统一返回业务 ack。
                # 桥接层只做日志提示，避免字段缺失时 APP 收不到 navigation_command_result。
                if not command_data.get("task_session_id") or not command_data.get("target_waypoint_id"):
                    self.get_logger().warning("路线任务跳转字段缺失: task_session_id 或 target_waypoint_id，继续转发给状态机返回业务 ack")

            elif command_type == "broadcast_finished":
                # 播报完成回执也交给状态机做严格上下文匹配。
                # 即使缺少字段，也继续转发，让状态机通过 navigation_command_result
                # 返回 invalid_task_session 或 broadcast_context_mismatch 等业务错误码。
                required_fields = ("task_session_id", "waypoint_id", "broadcast_id")
                missing_fields = [field for field in required_fields if not command_data.get(field)]
                if missing_fields:
                    self.get_logger().warning(f"播报完成字段缺失: {missing_fields}，继续转发给状态机返回业务 ack")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"验证导航命令错误: {e}")
            return False

    def normalize_navigation_command(self, command_data: Dict[str, Any]) -> Dict[str, Any]:
        """在转发给状态机前归一化导航命令 ID 字段。

        APP 可能把 waypoint_id 传成数字，也可能传成字符串。状态机内部统一按字符串比较，
        这里提前归一化可以降低 completed/skipped/jump 匹配时的类型风险。
        """
        normalized = dict(command_data)

        # 顶层 ID 字段统一转字符串；字段不存在时不主动补业务默认值。
        for field in (
            "waypoint_id",
            "target_waypoint_id",
            "task_session_id",
            "route_id",
            "broadcast_id",
            "request_message_id",
        ):
            if field in normalized and normalized[field] is not None:
                normalized[field] = str(normalized[field])

        if "waypoint_ids" in normalized and isinstance(normalized["waypoint_ids"], list):
            # 兼容旧多点导航，避免旧 waypoint_ids 列表里混入数字 ID。
            normalized["waypoint_ids"] = [str(waypoint_id) for waypoint_id in normalized["waypoint_ids"]]

        route_waypoints = normalized.get("route_waypoints")
        if isinstance(route_waypoints, list):
            normalized_route_waypoints = []
            for waypoint in route_waypoints:
                if isinstance(waypoint, dict):
                    # route_waypoints 是 APP 下发的完整路线，只复制并规范 waypoint_id，
                    # 不在桥接层改 waypoint_role、need_broadcast、pose 等业务字段。
                    normalized_waypoint = dict(waypoint)
                    if normalized_waypoint.get("waypoint_id") is not None:
                        normalized_waypoint["waypoint_id"] = str(normalized_waypoint["waypoint_id"])
                    normalized_route_waypoints.append(normalized_waypoint)
                else:
                    normalized_route_waypoints.append(waypoint)
            normalized["route_waypoints"] = normalized_route_waypoints

        return normalized
    
    def send_navigation_request(self, command_data: Dict[str, Any]):
        """发送导航请求给状态管理器"""
        try:
            # 准备请求数据
            request_data = {
                "request_type": "navigation_command",
                "command_data": command_data,
                "timestamp": time.time(),
                "source": "waypoints_manager"
            }
            
            command_type = command_data.get("command_type")
            if command_type == "start_single_navigation":
               waypoint_id = command_data.get("waypoint_id")
               waypoint = self.find_waypoint_by_id(waypoint_id)
               if waypoint:
                request_data["waypoint_data"] = waypoint.to_dict()
        
            elif command_type == "start_multi_point_navigation":  
                waypoint_ids = command_data.get("waypoint_ids", [])
    
                # === 新增校验逻辑 ===
                invalid_ids = [wp_id for wp_id in waypoint_ids if not self.find_waypoint_by_id(wp_id)]
                if invalid_ids:
                    self.get_logger().error(f"❌ 拒绝请求：以下点位不存在: {invalid_ids}")
                    self.send_app_response("error", f"点位不存在: {invalid_ids}")
                    return
                # 添加所有点位数据
                multi_points = {}
                for wp_id in waypoint_ids: 
                    waypoint = self.find_waypoint_by_id(wp_id)
                    if waypoint:
                       multi_points[wp_id] = waypoint.to_dict()
                request_data["waypoints_data"] = multi_points
        
            elif command_type == "start_exhibition_navigation":
                # 添加所有展台点数据
                exhibition_points = {}
                for wp_id in self.waypoints[WaypointType.EXHIBITION_POINT.value]:
                    waypoint = self.waypoints[WaypointType.EXHIBITION_POINT.value][wp_id]
                    exhibition_points[wp_id] = waypoint.to_dict()
                request_data["waypoints_data"] = exhibition_points
            
            # 发布请求
            request_msg = String()
            request_msg.data = json.dumps(request_data)
            self.navigation_request_pub.publish(request_msg)
            
            self.get_logger().info(f"导航请求已发送: {command_data.get('command_type')}")
            
        except Exception as e:
            self.get_logger().error(f"发送导航请求错误: {e}")
    
    # 添加多点导航序列创建方法
    def create_multi_point_sequence(self, name: str, waypoint_ids: List[str]) -> str: 
        """创建多点导航序列"""
        try:
            # 验证点位ID
            if not self.validate_waypoint_ids(waypoint_ids):
               raise ValueError("包含无效的点位ID")
        
            # 创建序列
            sequence_id = self.create_navigation_sequence(name, waypoint_ids, "multi_point")
        
            self.get_logger().info(f"创建多点导航序列: {name}, 包含 {len(waypoint_ids)} 个点位")
            return sequence_id
        
        except Exception as e:
            self.get_logger().error(f"创建多点导航序列错误: {e}")
            raise

    def navigation_ack_callback(self, msg: String):
        """处理状态管理器的确认消息"""
        try:
            ack_data = json.loads(msg.data)
            ack_type = ack_data.get("ack_type", "")
            status = ack_data.get("status", "")
            message = ack_data.get("message", "")
            
            self.get_logger().info(f"收到状态管理器确认: {ack_type} - {status}")
            
            # 根据确认类型处理
            if ack_type == "navigation_started":
                if status == "success":
                    self.get_logger().info("导航已成功启动")
                else:
                    self.get_logger().error(f"导航启动失败: {message}")
            
            elif ack_type == "navigation_completed":
                self.get_logger().info(f"导航完成: {message}")
            
            elif ack_type == "navigation_failed":
                self.get_logger().error(f"导航失败: {message}")
            
        except Exception as e:
            self.get_logger().error(f"处理确认消息错误: {e}")
    
    def handle_set_waypoint(self, command_data: Dict[str, Any]):
        """处理设置点位"""
        try:
            waypoint_data = command_data.get("waypoint_data", {})
            waypoint_id = waypoint_data.get("id")
            waypoint_type_str = waypoint_data.get("type")
            
            if not waypoint_id or not waypoint_type_str:
                self.send_app_response("error", "缺少必要参数: id 或 type")
                return

            properties = waypoint_data.get("properties", {})
            ok, error_message, normalized_properties = self.normalize_waypoint_speed_properties(properties)
            if not ok:
                self.send_app_response("error", error_message)
                return
            
            # 创建并存储点位
            waypoint_type = WaypointType(waypoint_type_str)
            waypoint = WaypointData(
                id=waypoint_id,
                name=waypoint_data.get("name", waypoint_id),
                waypoint_type=waypoint_type,
                position=waypoint_data.get("position", [0.0, 0.0, 0.0]),
                orientation=waypoint_data.get("orientation", [0.0, 0.0, 0.0, 1.0]),
                frame_id=waypoint_data.get("frame_id", "map"),
                properties=normalized_properties
            )
            
            self.waypoints[waypoint_type.value][waypoint_id] = waypoint
            
            # 保存数据
            if self.data_storage_enabled:
                self.save_waypoints_data()
            
            # 发布点位数据更新
            self.publish_waypoints_data()
            
            self.send_app_response("success", f"点位 '{waypoint.name}' 设置成功")
            
        except Exception as e:
            self.get_logger().error(f"设置点位错误: {e}")
            self.send_app_response("error", f"设置点位失败: {str(e)}")
    
    def handle_update_waypoint(self, command_data: Dict[str, Any]):
        """处理更新点位"""
        try:
            waypoint_data = command_data.get("waypoint_data", {})
            waypoint_id = waypoint_data.get("id")
            waypoint_type_str = waypoint_data.get("type")
            
            if not waypoint_id or not waypoint_type_str:
                self.send_app_response("error", "缺少必要参数: id 或 type")
                return
            
            # 查找现有点位
            waypoint_type = WaypointType(waypoint_type_str)
            if waypoint_id not in self.waypoints[waypoint_type.value]:
                self.send_app_response("error", f"点位不存在: {waypoint_id}")
                return

            incoming_properties = waypoint_data.get("properties", {})
            ok, error_message, normalized_properties = self.normalize_waypoint_speed_properties(incoming_properties)
            if not ok:
                self.send_app_response("error", error_message)
                return
            
            # 更新点位数据
            waypoint = self.waypoints[waypoint_type.value][waypoint_id]
            waypoint.name = waypoint_data.get("name", waypoint.name)
            waypoint.position = waypoint_data.get("position", waypoint.position)
            waypoint.orientation = waypoint_data.get("orientation", waypoint.orientation)
            waypoint.frame_id = waypoint_data.get("frame_id", waypoint.frame_id)
            waypoint.properties.update(normalized_properties)
            waypoint.last_modified = time.time()
            
            # 保存数据
            if self.data_storage_enabled:
                self.save_waypoints_data()
            
            # 发布点位数据更新
            self.publish_waypoints_data()
            
            self.send_app_response("success", f"点位 '{waypoint.name}' 更新成功")
            
        except Exception as e:
            self.get_logger().error(f"更新点位错误: {e}")
            self.send_app_response("error", f"更新点位失败: {str(e)}")
    
    def handle_delete_waypoint(self, command_data: Dict[str, Any]):
        """处理删除点位"""
        try:
            waypoint_id = command_data.get("waypoint_id")
            waypoint_type_str = command_data.get("waypoint_type")
            
            if not waypoint_id or not waypoint_type_str:
                self.send_app_response("error", "缺少必要参数: waypoint_id 或 waypoint_type")
                return
            
            waypoint_type = WaypointType(waypoint_type_str)
            
            # 检查点位是否存在
            if waypoint_id not in self.waypoints[waypoint_type.value]:
                self.send_app_response("error", f"点位不存在: {waypoint_id}")
                return
            
            # 删除点位
            waypoint_name = self.waypoints[waypoint_type.value][waypoint_id].name
            del self.waypoints[waypoint_type.value][waypoint_id]
            
            # 从所有导航序列中移除该点位
            for sequence_id, sequence in self.navigation_sequences.items():
                if waypoint_id in sequence.waypoint_ids:
                    sequence.waypoint_ids.remove(waypoint_id)
            
            # 保存数据
            if self.data_storage_enabled:
                self.save_waypoints_data()
            
            # 发布点位数据更新
            self.publish_waypoints_data()
            
            self.send_app_response("success", f"点位 '{waypoint_name}' 删除成功")
            
        except Exception as e:
            self.get_logger().error(f"删除点位错误: {e}")
            self.send_app_response("error", f"删除点位失败: {str(e)}")
    
    def handle_get_waypoints(self, command_data: Dict[str, Any]):
        """处理获取点位列表"""
        try:
            waypoint_type_str = command_data.get("waypoint_type")
            include_details = command_data.get("include_details", True)
        
            response_data = {}
        
            if waypoint_type_str and waypoint_type_str != "all":  # ← 修复：允许空值或"all"
                # 获取特定类型的点位
                try:
                    waypoint_type = WaypointType(waypoint_type_str)
                    waypoints = self.waypoints[waypoint_type.value]
                
                    if include_details:
                        response_data[waypoint_type.value] = {
                           wp_id: wp.to_dict() for wp_id, wp in waypoints.items()
                        }
                    else:
                        response_data[waypoint_type.value] = list(waypoints.keys())
                except ValueError:
                    self.send_app_response("error", f"无效的点位类型: {waypoint_type_str}")
                    return
            else:
                # 获取所有点位（waypoint_type为空或"all"）
                for wp_type, waypoints in self.waypoints.items():
                    if include_details:
                        response_data[wp_type] = {
                            wp_id: wp.to_dict() for wp_id, wp in waypoints.items()
                        }
                    else:
                        response_data[wp_type] = list(waypoints.keys())
        
            # 添加序列信息
            if include_details:
                response_data["sequences"] = {
                    seq_id: {
                       "id": seq.id,
                       "name": seq.name,
                       "waypoint_ids": seq.waypoint_ids,
                       "sequence_type": seq.sequence_type,
                       "properties": seq.properties
                    }
                    for seq_id, seq in self.navigation_sequences.items()
                }
        
            self.send_app_response("success", "获取点位列表成功", response_data)
        
        except Exception as e:
            self.get_logger().error(f"获取点位列表错误: {e}")
            self.send_app_response("error", f"获取点位列表失败: {str(e)}")
    
    def handle_clear_waypoints(self, command_data: Dict[str, Any]):
        """处理清空点位"""
        try:
            waypoint_type_str = command_data.get("waypoint_type")
            
            if waypoint_type_str:
                # 清空特定类型的点位
                waypoint_type = WaypointType(waypoint_type_str)
                cleared_count = len(self.waypoints[waypoint_type.value])
                self.waypoints[waypoint_type.value].clear()
                
                # 从序列中移除该类型的点位
                for sequence in self.navigation_sequences.values():
                    sequence.waypoint_ids = [
                        wp_id for wp_id in sequence.waypoint_ids
                        if wp_id not in self.waypoints[waypoint_type.value]
                    ]
                
                self.send_app_response("success", f"清空 {waypoint_type.value} 类型点位成功，共 {cleared_count} 个")
            else:
                # 清空所有点位
                total_count = self.get_total_waypoints_count()
                for waypoints in self.waypoints.values():
                    waypoints.clear()
                self.navigation_sequences.clear()
                
                self.send_app_response("success", f"清空所有点位成功，共 {total_count} 个")
            
            # 保存数据
            if self.data_storage_enabled:
                self.save_waypoints_data()
            
            # 发布点位数据更新
            self.publish_waypoints_data()
            
        except Exception as e:
            self.get_logger().error(f"清空点位错误: {e}")
            self.send_app_response("error", f"清空点位失败: {str(e)}")
    
    def publish_waypoints_data(self, update_type="full_update"):
        """发布路点数据到 /navigation/waypoints_data"""
        try:
            waypoints_data = {
                "update_type": update_type,
                "timestamp": time.time(),
                "data": {
                    "waypoints": {
                        wp_type: {wp_id: wp.to_dict() for wp_id, wp in waypoints.items()}
                        for wp_type, waypoints in self.waypoints.items()
                    },
                    "sequences": {
                        seq_id: {
                            "id": seq.id,
                            "name": seq.name,
                            "waypoint_ids": seq.waypoint_ids,
                            "sequence_type": seq.sequence_type,
                            "properties": seq.properties
                        }
                        for seq_id, seq in self.navigation_sequences.items()
                    }
                },
                "metadata": {
                    "total_count": self.get_total_waypoints_count(),
                    "sequence_count": len(self.navigation_sequences)
                }
            }
        
            unified_msg = self.create_unified_message(
                message_type="push",
                data_type="waypoints_data",
                source="waypoints_manager",
                destination="all",
                data=waypoints_data
            )
        
            msg = String()
            msg.data = json.dumps(unified_msg, ensure_ascii=False)
            self.waypoints_data_pub.publish(msg)
        
            self.get_logger().info(f' 发布路点数据')
        
        except Exception as e:
            self.get_logger().error(f'发布路点数据错误: {e}')

    def publish_initial_waypoints_data(self):
        """启动后把本地加载的点位同步给导航状态管理器。"""
        try:
            self.publish_waypoints_data(update_type="initial_load")
            self.initial_waypoints_publish_count += 1

            if self.initial_waypoints_publish_count >= self.initial_waypoints_publish_max:
                if self.initial_waypoints_publish_timer is not None:
                    self.initial_waypoints_publish_timer.cancel()
                    self.initial_waypoints_publish_timer = None
                self.get_logger().info(
                    f"本地加载点位已完成初始同步，共 {self.get_total_waypoints_count()} 个点位"
                )
        except Exception as e:
            self.get_logger().error(f"发布初始点位数据错误: {e}")

    def create_unified_message(self, message_type: str, data_type: str, 
                         source: str, destination: str, data: Dict) -> Dict:
        """创建统一格式消息"""
        return {
        "protocol_version": "2.0",
        "message_id": f"{message_type}_{int(time.time())}",
        "timestamp": time.time(),
        "message_type": message_type,
        "data_type": data_type,
        "source": source,
        "destination": destination,
        "data": data,
        "metadata": {
            "status": "success",
            "error_code": "",
            "error_message": "",
            "request_id": ""
        }
        }


    def send_app_response(self, response_type: str, message: str, data: Dict = None):
        """发送响应给APP（通过ROS topic发布）"""
        try:
            response_msg = self.create_unified_message(
                message_type="response",
                data_type="waypoint_response",
                source="waypoints_manager",
                destination="all",
                data={
                   "response_type": response_type,
                   "message": message,
                   "result": data or {}
                }
            )
        
            response_msg["metadata"]["status"] = "success" if response_type == "success" else "error"
            if response_type == "error":
                response_msg["metadata"]["error_message"] = message
        
            msg = String()
            msg.data = json.dumps(response_msg, ensure_ascii=False)
            self.waypoints_data_pub.publish(msg)
        
            self.get_logger().info(f"✅ 响应: {message}")
        
        except Exception as e:
            self.get_logger().error(f"❌ 发送响应失败: {e}")
    
    
    # ========== 数据持久化方法 ==========
    def setup_data_persistence(self):
        """设置数据持久化"""
        try:
            # 获取参数
            self.data_storage_enabled = self.get_parameter('data_storage.enabled').value
            self.storage_file_path = self.get_parameter('data_storage.file_path').value
        
            # 展开用户主目录路径
            if self.storage_file_path.startswith('~/'):
               self.storage_file_path = os.path.expanduser(self.storage_file_path)
        
            # 创建存储目录
            storage_dir = os.path.dirname(self.storage_file_path)
            if storage_dir:
               os.makedirs(storage_dir, exist_ok=True)
        
            if self.data_storage_enabled:
                self.get_logger().info(f"数据持久化已启用，文件路径: {self.storage_file_path}")
            
                # 如果文件存在，加载现有数据
                if os.path.exists(self.storage_file_path):
                    self.load_waypoints_data()
                else:
                    self.get_logger().info("没有找到现有的路点数据文件，将在首次保存时创建")
            else:
                self.get_logger().info("数据持久化已禁用")
            
        except Exception as e:
                self.get_logger().error(f"设置数据持久化失败: {e}")
                # 设置默认值以确保功能可用
                self.data_storage_enabled = True
                self.storage_file_path = os.path.expanduser('~/humanoid_ws/data/dynamic_waypoints.json')

    def save_waypoints_data(self):
        """保存点位数据"""
        try:
            data_to_save = {
               "waypoints": {
                  wp_type: {wp_id: wp.to_dict() for wp_id, wp in waypoints.items()}
                  for wp_type, waypoints in self.waypoints.items()
                },
                "sequences": {
                  seq_id: {
                    "id": seq.id,
                    "name": seq.name,
                    "waypoint_ids": seq.waypoint_ids,
                    "sequence_type": seq.sequence_type,
                    "properties": seq.properties
                  }
                  for seq_id, seq in self.navigation_sequences.items()
                },
                "timestamp": time.time()
            }
            
            with open(self.storage_file_path, 'w', encoding='utf-8') as f:
                 json.dump(data_to_save, f, indent=2, ensure_ascii=False)
            self.get_logger().info("点位数据保存成功")    
        except Exception as e:
            self.get_logger().error(f"保存点位数据错误: {e}")
    
    def load_waypoints_data(self):
        """加载点位数据"""
        if not os.path.exists(self.storage_file_path):
           return
        
        try:
            with open(self.storage_file_path, 'r', encoding='utf-8') as f:
                 data = json.load(f)
            
            # 加载点位数据
            waypoints_data = data.get("waypoints", {})
            for wp_type, waypoints_dict in waypoints_data.items():
                if wp_type in self.waypoints:
                   for wp_id, wp_data in waypoints_dict.items():
                    try:
                        waypoint = WaypointData(
                            id=wp_data["id"],
                            name=wp_data["name"],
                            waypoint_type=WaypointType(wp_data["type"]),
                            position=wp_data["position"],
                            orientation=wp_data["orientation"],
                            frame_id=wp_data.get("frame_id", "map"),
                            properties=wp_data.get("properties", {})
                        )
                        self.waypoints[wp_type][wp_id] = waypoint
                    except Exception as e:
                        self.get_logger().warning(f"加载点位失败: {wp_id} - {e}")
            
            # 加载导航序列
            sequences_data = data.get("sequences", {})
            for seq_id, seq_data in sequences_data.items():
                try:
                    sequence = NavigationSequence(
                    id=seq_data["id"],
                    name=seq_data["name"],
                    waypoint_ids=seq_data["waypoint_ids"],
                    sequence_type=seq_data["sequence_type"],
                    properties=seq_data.get("properties", {})
                   )
                    self.navigation_sequences[seq_id] = sequence
                except Exception as e:
                    self.get_logger().warning(f"加载导航序列失败: {seq_id} - {e}")

            total_count = self.get_total_waypoints_count()    
            self.get_logger().info(f"从文件加载点位数据完成，共 {total_count} 个点位，{len(self.navigation_sequences)} 个序列")
            
        except Exception as e:
            self.get_logger().error(f"加载点位数据错误: {e}")
    
    def get_total_waypoints_count(self) -> int:
        """获取总点位数量"""
        return sum(len(waypoints) for waypoints in self.waypoints.values())
    
    def find_waypoint_by_id(self, waypoint_id: str) -> Optional[WaypointData]:
        """根据ID查找点位"""
        for waypoints_dict in self.waypoints.values():
            if waypoint_id in waypoints_dict:
               return waypoints_dict[waypoint_id]
        return None
    
    def create_navigation_sequence(self, name: str, waypoint_ids: List[str], 
                                 sequence_type: str = "custom") -> str:
        """创建导航序列"""
        sequence_id = f"seq_{int(time.time())}_{uuid.uuid4().hex[:8]}"
        sequence = NavigationSequence(
           id=sequence_id,
           name=name,
           waypoint_ids=waypoint_ids,
           sequence_type=sequence_type
        )
        self.navigation_sequences[sequence_id] = sequence
        return sequence_id
    
    def validate_waypoint_ids(self, waypoint_ids: List[str]) -> bool:
        """验证点位ID列表是否有效"""
        for wp_id in waypoint_ids:
            if not self.find_waypoint_by_id(wp_id):
                return False
        return True

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = DynamicWaypointsManager()
        node.get_logger().info("动态路点管理器启动成功")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("收到键盘中断信号，正在关闭...")
        else:
            print("收到键盘中断信号")
            
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"动态路点管理器运行错误: {e}")
        else:
            print(f"动态路点管理器运行错误: {e}")
            
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
