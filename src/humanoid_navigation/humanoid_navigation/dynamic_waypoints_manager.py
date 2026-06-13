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

# ========== 枚举定义 ==========
class WaypointType(Enum):
    """点位类型枚举"""
    NAVIGATION_TARGET = "navigation_target"
    EXHIBITION_POINT = "exhibition_point" 
    OBSTACLE_POINT = "obstacle_point"
    CHARGING_POINT = "charging_point"
    REST_POINT = "rest_point"
    LANDMARK_POINT = "landmark_point"

# ========== 数据类定义 ==========
class WaypointData:
    """点位数据类"""
    def __init__(self, id: str, name: str, waypoint_type: WaypointType, 
                 position: List[float], orientation: List[float], 
                 frame_id: str = "map", properties: Dict[str, Any] = None,
                 map_id: str = "hall", created_time: float = None,
                 last_modified: float = None):
        self.id = str(id)
        self.name = name
        self.waypoint_type = waypoint_type
        self.position = position
        self.orientation = orientation
        self.frame_id = frame_id
        self.map_id = str(map_id or "hall").strip() or "hall"
        self.properties = properties or {}
        self.created_time = float(created_time) if created_time is not None else time.time()
        self.last_modified = float(last_modified) if last_modified is not None else time.time()
    
    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式"""
        return {
            "id": self.id,
            "name": self.name,
            "type": self.waypoint_type.value,
            "map_id": self.map_id,
            "position": self.position,
            "orientation": self.orientation,
            "frame_id": self.frame_id,
            "properties": self.properties,
            "created_time": self.created_time,
            "last_modified": self.last_modified
        }

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
            ('data_storage.file_path', '/home/ubuntu/software/Todesk/Files/humanoid_ws/data/dynamic_waypoints.json'),
            ('data_storage.waypoints_dir', '/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints'),
            ('data_storage.default_map_id', 'hall'),
            ('data_storage.auto_save_interval', 300.0),
        ])
        
        # ========== 数据存储 ==========
        self.default_map_id = "hall"
        self.current_map_id = "hall"
        self.waypoints_by_map: Dict[str, Dict[str, Dict[str, WaypointData]]] = {}
        self.waypoints_revisions_by_map: Dict[str, str] = {}
        self.waypoints: Dict[str, Dict[str, WaypointData]] = {
            wp_type.value: {} for wp_type in WaypointType
        }
        # 点位库版本号：APP 使用“只下发 waypoint_id 列表”启动路线时，
        # 需要携带该版本号，状态管理器会校验 APP 和 ROS 使用的是同一份点位库。
        # 这样可以避免 APP 只发 ID，而 ROS 本地点位还是旧坐标时机器人按旧路线执行。
        self.waypoints_revision = ""
        
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
        if self.get_total_waypoints_count() > 0:
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

    def normalize_map_id(self, map_id: Any) -> str:
        """归一化地图 ID；旧 APP 没传时回退到默认地图。"""
        normalized = str(map_id or self.default_map_id or "hall").strip()
        return normalized or "hall"

    def empty_waypoint_bucket(self) -> Dict[str, Dict[str, WaypointData]]:
        """创建一张地图自己的点位桶。"""
        return {wp_type.value: {} for wp_type in WaypointType}

    def ensure_map_cache(self, map_id: Any) -> str:
        """确保指定地图的内存缓存和 revision 都存在。"""
        normalized_map_id = self.normalize_map_id(map_id)
        if normalized_map_id not in self.waypoints_by_map:
            self.waypoints_by_map[normalized_map_id] = self.empty_waypoint_bucket()
        if normalized_map_id not in self.waypoints_revisions_by_map:
            self.waypoints_revisions_by_map[normalized_map_id] = f"{time.time():.3f}"
        if normalized_map_id == self.current_map_id:
            self.waypoints = self.waypoints_by_map[normalized_map_id]
            self.waypoints_revision = self.waypoints_revisions_by_map[normalized_map_id]
        return normalized_map_id

    def get_map_waypoints(self, map_id: Any) -> Dict[str, Dict[str, WaypointData]]:
        """获取指定地图的点位缓存。"""
        normalized_map_id = self.ensure_map_cache(map_id)
        return self.waypoints_by_map[normalized_map_id]

    def get_map_revision(self, map_id: Any) -> str:
        """获取指定地图的点位版本号。"""
        normalized_map_id = self.ensure_map_cache(map_id)
        return self.waypoints_revisions_by_map.get(normalized_map_id, "")

    def refresh_waypoints_revision(self, map_id: Any = None):
        """刷新指定地图的点位库版本号。

        多地图后 revision 必须按地图独立维护：
        APP 保存 hall1 点位时只刷新 hall1，不能让 hall 的路线任务版本失效。
        """
        normalized_map_id = self.ensure_map_cache(map_id or self.current_map_id)
        self.waypoints_revisions_by_map[normalized_map_id] = f"{time.time():.3f}"
        if normalized_map_id == self.current_map_id:
            self.waypoints_revision = self.waypoints_revisions_by_map[normalized_map_id]

    def waypoint_file_path(self, map_id: Any) -> str:
        """返回某张地图对应的独立点位 JSON 文件路径。"""
        normalized_map_id = self.normalize_map_id(map_id)
        safe_map_id = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in normalized_map_id)
        return os.path.join(self.waypoints_storage_dir, f"{safe_map_id}.json")

    def extract_command_map_id(self, command_data: Dict[str, Any], waypoint_data: Dict[str, Any] = None) -> str:
        """从命令或点位数据里提取 map_id，旧包默认归入 default_map_id。"""
        waypoint_data = waypoint_data if isinstance(waypoint_data, dict) else {}
        return self.normalize_map_id(
            command_data.get("map_id")
            or waypoint_data.get("map_id")
            or waypoint_data.get("properties", {}).get("map_id", "")
        )
    
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
            
            if command_type == "start_route_task":
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
                route_waypoint_ids = command_data.get("route_waypoint_ids", [])
                has_route_waypoints = isinstance(route_waypoints, list) and len(route_waypoints) > 0
                has_route_waypoint_ids = isinstance(route_waypoint_ids, list) and len(route_waypoint_ids) > 0
                if not has_route_waypoints and not has_route_waypoint_ids:
                    self.get_logger().warning(
                        "路线任务启动字段异常: route_waypoints/route_waypoint_ids 均为空或不是数组，继续转发给状态机返回业务 ack"
                    )
                if has_route_waypoints and has_route_waypoint_ids:
                    self.get_logger().warning(
                        "路线任务启动字段异常: route_waypoints 和 route_waypoint_ids 同时存在，继续转发给状态机返回业务 ack"
                    )
                if has_route_waypoint_ids and not command_data.get("waypoints_revision"):
                    self.get_logger().warning(
                        "路线任务 ID 列表模式缺少 waypoints_revision，继续转发给状态机返回 missing_waypoints_revision"
                    )

            elif command_type == "jump_to_waypoint":
                # jump 的会话、目标点、是否允许跳到该点，都由状态机统一返回业务 ack。
                # 桥接层只做日志提示，避免字段缺失时 APP 收不到 navigation_command_result。
                if not command_data.get("task_session_id") or not command_data.get("target_waypoint_id"):
                    self.get_logger().warning("路线任务跳转字段缺失: task_session_id 或 target_waypoint_id，继续转发给状态机返回业务 ack")

            elif command_type in ["pause_route_task", "resume_route_task", "stop_route_task"]:
                # route task 专属控制命令需要精确匹配当前 task_session_id 和 route_id。
                # 桥接层只做提示，不硬拒绝；真正的 route_task_not_running /
                # invalid_task_session / invalid_route_id 由状态机通过 navigation_command_result 返回。
                if not command_data.get("task_session_id") or not command_data.get("route_id"):
                    self.get_logger().warning(
                        f"路线任务控制字段缺失: {command_type} 需要 task_session_id 和 route_id，继续转发给状态机返回业务 ack"
                    )

            elif command_type == "broadcast_finished":
                # 播报完成回执也交给状态机做严格上下文匹配。
                # 即使缺少字段，也继续转发，让状态机通过 navigation_command_result
                # 返回 invalid_task_session 或 broadcast_context_mismatch 等业务错误码。
                required_fields = ("task_session_id", "route_id", "waypoint_id", "broadcast_id")
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
            "map_id",
            "broadcast_id",
            "request_message_id",
            "waypoints_revision",
        ):
            if field in normalized and normalized[field] is not None:
                normalized[field] = str(normalized[field])

        route_waypoint_ids = normalized.get("route_waypoint_ids")
        if isinstance(route_waypoint_ids, list):
            # ID 列表模式只做字符串化，不改顺序、不查点位；真正校验和补全由状态机完成。
            # None/空字符串也保留下来交给状态机返回 invalid_route_waypoint_ids，
            # 不能在桥接层静默删除，否则 APP 传错数组时路线顺序会被悄悄改写。
            normalized["route_waypoint_ids"] = [
                str(item).strip() if item is not None else ""
                for item in route_waypoint_ids
            ]

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
                    if normalized_waypoint.get("map_id") is not None:
                        normalized_waypoint["map_id"] = self.normalize_map_id(normalized_waypoint["map_id"])
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
            
            # 导航命令对外只保留 route task 新协议。
            # route_waypoints 已由 APP 后端一次性组好并透传到状态机，
            # 桥接层不再为旧单点/多点/展台命令补 waypoint_data，避免两套路线上下文并存。
            
            # 发布请求
            request_msg = String()
            request_msg.data = json.dumps(request_data)
            self.navigation_request_pub.publish(request_msg)
            
            self.get_logger().info(f"导航请求已发送: {command_data.get('command_type')}")
            
        except Exception as e:
            self.get_logger().error(f"发送导航请求错误: {e}")
    
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
            map_id = self.extract_command_map_id(command_data, waypoint_data)
            
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
                properties=normalized_properties,
                map_id=map_id
            )
            
            map_waypoints = self.get_map_waypoints(map_id)
            map_waypoints[waypoint_type.value][str(waypoint_id)] = waypoint

            # 点位内容发生真实变更后刷新版本号；保存、推送、响应都使用同一版 revision。
            self.refresh_waypoints_revision(map_id)
            
            # 保存数据
            if self.data_storage_enabled:
                self.save_waypoints_data(map_id)
            
            # 发布点位数据更新
            self.publish_waypoints_data(map_id=map_id)
            
            self.send_app_response("success", f"点位 '{waypoint.name}' 设置成功", {"map_id": map_id})
            
        except Exception as e:
            self.get_logger().error(f"设置点位错误: {e}")
            self.send_app_response("error", f"设置点位失败: {str(e)}")
    
    def handle_update_waypoint(self, command_data: Dict[str, Any]):
        """处理更新点位"""
        try:
            waypoint_data = command_data.get("waypoint_data", {})
            waypoint_id = waypoint_data.get("id")
            waypoint_type_str = waypoint_data.get("type")
            map_id = self.extract_command_map_id(command_data, waypoint_data)
            
            if not waypoint_id or not waypoint_type_str:
                self.send_app_response("error", "缺少必要参数: id 或 type")
                return
            
            # 查找现有点位
            waypoint_type = WaypointType(waypoint_type_str)
            map_waypoints = self.get_map_waypoints(map_id)
            waypoint_key = str(waypoint_id)
            if waypoint_key not in map_waypoints[waypoint_type.value]:
                self.send_app_response("error", f"地图 {map_id} 下点位不存在: {waypoint_id}")
                return

            incoming_properties = waypoint_data.get("properties", {})
            ok, error_message, normalized_properties = self.normalize_waypoint_speed_properties(incoming_properties)
            if not ok:
                self.send_app_response("error", error_message)
                return
            
            # 更新点位数据
            waypoint = map_waypoints[waypoint_type.value][waypoint_key]
            waypoint.name = waypoint_data.get("name", waypoint.name)
            waypoint.position = waypoint_data.get("position", waypoint.position)
            waypoint.orientation = waypoint_data.get("orientation", waypoint.orientation)
            waypoint.frame_id = waypoint_data.get("frame_id", waypoint.frame_id)
            waypoint.map_id = map_id
            waypoint.properties.update(normalized_properties)
            waypoint.last_modified = time.time()

            # 点位内容发生真实变更后刷新版本号，供 APP 后续 ID 列表启动时做一致性校验。
            self.refresh_waypoints_revision(map_id)
            
            # 保存数据
            if self.data_storage_enabled:
                self.save_waypoints_data(map_id)
            
            # 发布点位数据更新
            self.publish_waypoints_data(map_id=map_id)
            
            self.send_app_response("success", f"点位 '{waypoint.name}' 更新成功", {"map_id": map_id})
            
        except Exception as e:
            self.get_logger().error(f"更新点位错误: {e}")
            self.send_app_response("error", f"更新点位失败: {str(e)}")
    
    def handle_delete_waypoint(self, command_data: Dict[str, Any]):
        """处理删除点位"""
        try:
            waypoint_id = command_data.get("waypoint_id")
            waypoint_type_str = command_data.get("waypoint_type")
            map_id = self.extract_command_map_id(command_data)
            
            if not waypoint_id or not waypoint_type_str:
                self.send_app_response("error", "缺少必要参数: waypoint_id 或 waypoint_type")
                return
            
            waypoint_type = WaypointType(waypoint_type_str)
            map_waypoints = self.get_map_waypoints(map_id)
            waypoint_key = str(waypoint_id)
            
            # 检查点位是否存在
            if waypoint_key not in map_waypoints[waypoint_type.value]:
                self.send_app_response("error", f"地图 {map_id} 下点位不存在: {waypoint_id}")
                return
            
            # 删除点位
            waypoint_name = map_waypoints[waypoint_type.value][waypoint_key].name
            del map_waypoints[waypoint_type.value][waypoint_key]

            # 点位库删除后也要刷新版本号，禁止 APP 用删除前的 ID 列表继续启动。
            self.refresh_waypoints_revision(map_id)
            
            # 保存数据
            if self.data_storage_enabled:
                self.save_waypoints_data(map_id)
            
            # 发布点位数据更新
            self.publish_waypoints_data(map_id=map_id)
            
            self.send_app_response("success", f"点位 '{waypoint_name}' 删除成功", {"map_id": map_id})
            
        except Exception as e:
            self.get_logger().error(f"删除点位错误: {e}")
            self.send_app_response("error", f"删除点位失败: {str(e)}")
    
    def handle_get_waypoints(self, command_data: Dict[str, Any]):
        """处理获取点位列表"""
        try:
            waypoint_type_str = command_data.get("waypoint_type")
            include_details = command_data.get("include_details", True)
            map_id = self.extract_command_map_id(command_data)
            map_waypoints = self.get_map_waypoints(map_id)
        
            response_data = {"map_id": map_id}
        
            if waypoint_type_str and waypoint_type_str != "all":  # ← 修复：允许空值或"all"
                # 获取特定类型的点位
                try:
                    waypoint_type = WaypointType(waypoint_type_str)
                    waypoints = map_waypoints[waypoint_type.value]
                
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
                # 获取指定地图下的所有点位（waypoint_type为空或"all"）
                for wp_type, waypoints in map_waypoints.items():
                    if include_details:
                        response_data[wp_type] = {
                            wp_id: wp.to_dict() for wp_id, wp in waypoints.items()
                        }
                    else:
                        response_data[wp_type] = list(waypoints.keys())

            response_data["waypoints_revision"] = self.get_map_revision(map_id)
        
            self.send_app_response("success", f"获取地图 {map_id} 点位列表成功", response_data, map_id=map_id)
        
        except Exception as e:
            self.get_logger().error(f"获取点位列表错误: {e}")
            self.send_app_response("error", f"获取点位列表失败: {str(e)}")
    
    def handle_clear_waypoints(self, command_data: Dict[str, Any]):
        """处理清空点位"""
        try:
            waypoint_type_str = command_data.get("waypoint_type")
            raw_map_id = command_data.get("map_id")
            clear_scope = str(command_data.get("clear_scope", "") or "").strip()
            if not raw_map_id and clear_scope != "all_maps":
                self.send_app_response(
                    "error",
                    "多地图模式下 clear_waypoints 必须携带 map_id，避免误删其他地图点位",
                    {"error_code": "missing_map_id"}
                )
                return

            map_id = self.extract_command_map_id(command_data)
            
            if clear_scope == "all_maps":
                total_count = self.get_total_waypoints_count()
                for target_map_id in list(self.waypoints_by_map.keys()):
                    self.waypoints_by_map[target_map_id] = self.empty_waypoint_bucket()
                    self.refresh_waypoints_revision(target_map_id)
                    if self.data_storage_enabled:
                        self.save_waypoints_data(target_map_id)
                self.publish_waypoints_data(update_type="clear_all_maps", map_id=map_id)
                self.send_app_response("success", f"清空所有地图点位成功，共 {total_count} 个", {
                    "map_id": map_id,
                    "clear_scope": "all_maps",
                    "cleared_count": total_count
                }, map_id=map_id)
                return

            map_waypoints = self.get_map_waypoints(map_id)

            if waypoint_type_str:
                # 清空特定类型的点位
                waypoint_type = WaypointType(waypoint_type_str)
                cleared_count = len(map_waypoints[waypoint_type.value])
                map_waypoints[waypoint_type.value].clear()
            else:
                # 清空指定地图的所有点位
                total_count = self.get_total_waypoints_count(map_id)
                for waypoints in map_waypoints.values():
                    waypoints.clear()
                
                message = f"清空地图 {map_id} 所有点位成功，共 {total_count} 个"

            # 清空动作成功后先刷新版本号，再保存、推送和响应，保证 APP 拿到的是最新版本。
            self.refresh_waypoints_revision(map_id)
            
            # 保存数据
            if self.data_storage_enabled:
                self.save_waypoints_data(map_id)
            
            # 发布点位数据更新
            self.publish_waypoints_data(map_id=map_id)

            if waypoint_type_str:
                self.send_app_response("success", f"清空地图 {map_id} 的 {waypoint_type.value} 类型点位成功，共 {cleared_count} 个", {
                    "map_id": map_id,
                    "cleared_count": cleared_count
                }, map_id=map_id)
            else:
                self.send_app_response("success", message, {"map_id": map_id, "cleared_count": total_count}, map_id=map_id)
            
        except Exception as e:
            self.get_logger().error(f"清空点位错误: {e}")
            self.send_app_response("error", f"清空点位失败: {str(e)}")
    
    def publish_waypoints_data(self, update_type="full_update", map_id: str = None):
        """发布路点数据到 /navigation/waypoints_data"""
        try:
            normalized_map_id = self.ensure_map_cache(map_id or self.current_map_id)
            map_waypoints = self.get_map_waypoints(normalized_map_id)
            waypoints_data = {
                "update_type": update_type,
                "timestamp": time.time(),
                "map_id": normalized_map_id,
                "default_map_id": self.default_map_id,
                "waypoints_revision": self.get_map_revision(normalized_map_id),
                "waypoints_revisions_by_map": dict(self.waypoints_revisions_by_map),
                "data": {
                    "waypoints": {
                        wp_type: {wp_id: wp.to_dict() for wp_id, wp in waypoints.items()}
                        for wp_type, waypoints in map_waypoints.items()
                    },
                    # 给 navigation_state_manager 用的全量缓存。
                    # start_route_task(map_id=hall1, route_waypoint_ids=[...]) 必须按地图查点，
                    # 不能再只靠 waypoint_id 全局查找，否则不同地图的 1 号点会冲突。
                    "waypoints_by_map": self.serialize_waypoints_by_map()
                },
                "metadata": {
                    "total_count": self.get_total_waypoints_count(normalized_map_id),
                    "total_count_all_maps": self.get_total_waypoints_count(),
                    "map_id": normalized_map_id,
                    "waypoints_revision": self.get_map_revision(normalized_map_id),
                    "waypoints_revisions_by_map": dict(self.waypoints_revisions_by_map)
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
        
            self.get_logger().info(f' 发布地图 {normalized_map_id} 路点数据')
        
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


    def send_app_response(self, response_type: str, message: str, data: Dict = None, map_id: str = None):
        """发送响应给APP（通过ROS topic发布）"""
        try:
            normalized_map_id = self.ensure_map_cache(map_id or self.current_map_id)
            result_data = dict(data or {})
            result_data.setdefault("map_id", normalized_map_id)
            result_data.setdefault("waypoints_revision", self.get_map_revision(normalized_map_id))
            result_data.setdefault("waypoints_revisions_by_map", dict(self.waypoints_revisions_by_map))
            response_msg = self.create_unified_message(
                message_type="response",
                data_type="waypoint_response",
                source="waypoints_manager",
                destination="all",
                data={
                   "response_type": response_type,
                   "message": message,
                   "map_id": normalized_map_id,
                   "waypoints_revision": self.get_map_revision(normalized_map_id),
                   "result": result_data
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
            self.waypoints_storage_dir = self.get_parameter('data_storage.waypoints_dir').value
            self.default_map_id = self.normalize_map_id(self.get_parameter('data_storage.default_map_id').value)
            self.current_map_id = self.default_map_id
        
            # 展开用户主目录路径
            if self.storage_file_path.startswith('~/'):
               self.storage_file_path = os.path.expanduser(self.storage_file_path)
            if self.waypoints_storage_dir.startswith('~/'):
               self.waypoints_storage_dir = os.path.expanduser(self.waypoints_storage_dir)
        
            # 创建存储目录。多地图后主存储为 data/waypoints/<map_id>.json。
            storage_dir = os.path.dirname(self.storage_file_path)
            if storage_dir:
               os.makedirs(storage_dir, exist_ok=True)
            os.makedirs(self.waypoints_storage_dir, exist_ok=True)
            self.ensure_map_cache(self.default_map_id)
        
            if self.data_storage_enabled:
                self.get_logger().info(
                    f"数据持久化已启用，多地图点位目录: {self.waypoints_storage_dir}，旧文件: {self.storage_file_path}"
                )
            
                # 优先加载多地图点位文件；首次升级时再从旧 dynamic_waypoints.json 迁移。
                if self.load_waypoints_data():
                    pass
                elif os.path.exists(self.storage_file_path):
                    self.get_logger().warning("未发现多地图点位文件，开始从旧 dynamic_waypoints.json 迁移到默认地图")
                    self.load_legacy_waypoints_data(self.default_map_id)
                    self.save_waypoints_data(self.default_map_id)
                else:
                    self.get_logger().info("没有找到现有的路点数据文件，将在首次保存时创建")
                    self.refresh_waypoints_revision(self.default_map_id)
            else:
                self.get_logger().info("数据持久化已禁用")
                self.refresh_waypoints_revision(self.default_map_id)
            
        except Exception as e:
                self.get_logger().error(f"设置数据持久化失败: {e}")
                # 设置默认值以确保功能可用
                self.data_storage_enabled = True
                self.storage_file_path = '/home/ubuntu/software/Todesk/Files/humanoid_ws/data/dynamic_waypoints.json'
                self.waypoints_storage_dir = '/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints'
                self.default_map_id = "hall"
                self.current_map_id = "hall"
                self.ensure_map_cache(self.default_map_id)

    def serialize_waypoints_by_map(self) -> Dict[str, Dict[str, Dict[str, Dict[str, Any]]]]:
        """把所有地图点位缓存转换成 JSON 可序列化结构。"""
        return {
            map_id: {
                wp_type: {wp_id: wp.to_dict() for wp_id, wp in waypoints.items()}
                for wp_type, waypoints in map_waypoints.items()
            }
            for map_id, map_waypoints in self.waypoints_by_map.items()
        }

    def save_waypoints_data(self, map_id: Any = None):
        """保存点位数据"""
        try:
            normalized_map_id = self.ensure_map_cache(map_id or self.current_map_id)
            if not self.get_map_revision(normalized_map_id):
                self.refresh_waypoints_revision(normalized_map_id)
            map_waypoints = self.get_map_waypoints(normalized_map_id)
            data_to_save = {
               "map_id": normalized_map_id,
               "waypoints_revision": self.get_map_revision(normalized_map_id),
               "waypoints": {
                  wp_type: {wp_id: wp.to_dict() for wp_id, wp in waypoints.items()}
                  for wp_type, waypoints in map_waypoints.items()
                },
                "timestamp": time.time()
            }
            
            file_path = self.waypoint_file_path(normalized_map_id)
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, 'w', encoding='utf-8') as f:
                 json.dump(data_to_save, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f"地图 {normalized_map_id} 点位数据保存成功: {file_path}")    
        except Exception as e:
            self.get_logger().error(f"保存点位数据错误: {e}")
    
    def load_waypoints_data(self) -> bool:
        """加载多地图点位数据。返回是否成功加载到至少一个地图文件。"""
        if not os.path.isdir(self.waypoints_storage_dir):
           return False

        loaded_any = False
        for file_name in sorted(os.listdir(self.waypoints_storage_dir)):
            if not file_name.endswith(".json"):
                continue
            file_path = os.path.join(self.waypoints_storage_dir, file_name)
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                map_id = self.normalize_map_id(data.get("map_id") or os.path.splitext(file_name)[0])
                self.load_waypoints_payload_into_map(data, map_id)
                loaded_any = True
            except Exception as e:
                self.get_logger().warning(f"加载地图点位文件失败: {file_path} - {e}")

        self.waypoints = self.get_map_waypoints(self.current_map_id)
        self.waypoints_revision = self.get_map_revision(self.current_map_id)
        if loaded_any:
            self.get_logger().info(
                f"从多地图点位目录加载完成，共 {len(self.waypoints_by_map)} 张地图，{self.get_total_waypoints_count()} 个点位"
            )
        return loaded_any

    def load_legacy_waypoints_data(self, map_id: Any):
        """从旧 dynamic_waypoints.json 读取点位并归入默认地图。"""
        if not os.path.exists(self.storage_file_path):
           return
        
        try:
            with open(self.storage_file_path, 'r', encoding='utf-8') as f:
                 data = json.load(f)
            data["map_id"] = self.normalize_map_id(map_id)
            self.load_waypoints_payload_into_map(data, map_id)
            self.get_logger().info(
                f"旧点位文件迁移到地图 {self.normalize_map_id(map_id)} 完成，共 {self.get_total_waypoints_count(map_id)} 个点位"
            )
        except Exception as e:
            self.get_logger().error(f"加载旧点位数据错误: {e}")

    def load_waypoints_payload_into_map(self, data: Dict[str, Any], map_id: Any):
        """把一个点位 JSON payload 加载进指定地图缓存。"""
        normalized_map_id = self.ensure_map_cache(map_id)
        map_waypoints = self.get_map_waypoints(normalized_map_id)
        self.waypoints_revisions_by_map[normalized_map_id] = str(
            data.get("waypoints_revision") or data.get("timestamp") or f"{time.time():.3f}"
        )
            
        waypoints_data = data.get("waypoints", {})
        for wp_type, waypoints_dict in waypoints_data.items():
            if wp_type in map_waypoints:
               for wp_id, wp_data in waypoints_dict.items():
                try:
                    point_map_id = self.normalize_map_id(wp_data.get("map_id") or normalized_map_id)
                    waypoint = WaypointData(
                        id=wp_data["id"],
                        name=wp_data["name"],
                        waypoint_type=WaypointType(wp_data["type"]),
                        position=wp_data["position"],
                        orientation=wp_data["orientation"],
                        frame_id=wp_data.get("frame_id", "map"),
                        properties=wp_data.get("properties", {}),
                        map_id=point_map_id,
                        created_time=wp_data.get("created_time"),
                        last_modified=wp_data.get("last_modified"),
                    )
                    map_waypoints[wp_type][str(wp_id)] = waypoint
                except Exception as e:
                    self.get_logger().warning(f"加载地图 {normalized_map_id} 点位失败: {wp_id} - {e}")
    
    def get_total_waypoints_count(self, map_id: Any = None) -> int:
        """获取点位数量。

        map_id 为空时统计所有地图；传入 map_id 时只统计该地图。
        多地图后不能再只看 self.waypoints，否则 APP 查询 hall1 时可能拿到 hall 的数量。
        """
        if map_id is not None:
            map_waypoints = self.get_map_waypoints(map_id)
            return sum(len(waypoints) for waypoints in map_waypoints.values())

        return sum(
            len(waypoints)
            for map_waypoints in self.waypoints_by_map.values()
            for waypoints in map_waypoints.values()
        )
    
    def find_waypoint_by_id(self, waypoint_id: str, map_id: Any = None) -> Optional[WaypointData]:
        """根据 ID 查找点位。

        新路线任务必须传 map_id 精确查找，避免不同地图都有 1 号点时串图。
        不传 map_id 只作为旧内部调试兼容，会在所有地图里顺序查找。
        """
        waypoint_key = str(waypoint_id)
        if map_id is not None:
            for waypoints_dict in self.get_map_waypoints(map_id).values():
                if waypoint_key in waypoints_dict:
                    return waypoints_dict[waypoint_key]
            return None

        for map_waypoints in self.waypoints_by_map.values():
            for waypoints_dict in map_waypoints.values():
                if waypoint_key in waypoints_dict:
                    return waypoints_dict[waypoint_key]
        return None
    
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
