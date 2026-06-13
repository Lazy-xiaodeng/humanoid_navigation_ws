#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图上下文管理器 - 多地图一期基础能力。

一期支持“导航前显式切图”：APP 选择地图时先发 switch_map，ROS 在空闲状态下
切换 current_map_id、发布目标地图初始位姿，并等待定位状态稳定后进入 ready。
导航执行过程中仍禁止切图，真正重启底层 Nav2/定位进程的热切留到后续阶段增强。
"""

import json
import os
import time
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


class MapContextManager(Node):
    """维护当前地图上下文，并响应 APP 的 map_management 查询命令。"""

    def __init__(self):
        super().__init__("map_context_manager")
        self.declare_parameters(namespace="", parameters=[
            ("map_registry_path", "/home/ubuntu/software/Todesk/Files/humanoid_ws/data/maps/map_registry.json"),
            ("default_map_id", "hall"),
            ("localization_health_status_topic", "/localization/prior_map_odom_bridge_status"),
            ("initialpose_topic", "/initialpose"),
            ("switch_localization_timeout_sec", 8.0),
            ("switch_localization_stable_frames", 2),
            ("initialpose_repeat_count", 3),
            ("initialpose_repeat_interval_sec", 0.5),
            ("navigation_status_topic", "/navigation/status"),
        ])

        self.map_registry_path = self.expand_path(self.get_parameter("map_registry_path").value)
        self.default_map_id = self.normalize_map_id(self.get_parameter("default_map_id").value)
        self.localization_health_status_topic = str(self.get_parameter("localization_health_status_topic").value)
        self.initialpose_topic = str(self.get_parameter("initialpose_topic").value)
        self.switch_localization_timeout_sec = float(self.get_parameter("switch_localization_timeout_sec").value)
        self.switch_localization_stable_frames = max(1, int(self.get_parameter("switch_localization_stable_frames").value))
        self.initialpose_repeat_count = max(1, int(self.get_parameter("initialpose_repeat_count").value))
        self.initialpose_repeat_interval_sec = max(0.1, float(self.get_parameter("initialpose_repeat_interval_sec").value))
        self.navigation_status_topic = str(self.get_parameter("navigation_status_topic").value)
        self.current_map_id = self.default_map_id
        self.maps = self.load_map_registry()
        self.map_state = "ready"
        self.localization_state = "unknown"
        self.localization_stable_count = 0
        self.switch_context = None
        self.navigation_active = False
        self.navigation_detailed_state = ""

        self.map_command_sub = self.create_subscription(
            String, "/app/map_command", self.map_command_callback, 10
        )
        self.navigation_status_sub = self.create_subscription(
            String, self.navigation_status_topic, self.navigation_status_callback, 10
        )
        self.localization_status_sub = self.create_subscription(
            String, self.localization_health_status_topic, self.localization_status_callback, 10
        )
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 10)
        self.map_response_pub = self.create_publisher(String, "/map/response", 10)
        self.map_status_pub = self.create_publisher(String, "/map/status", 10)
        self.create_timer(0.5, self.switch_watchdog_tick)
        self.create_timer(5.0, self.publish_map_status)
        self.initial_map_status_timer = self.create_timer(0.2, self.publish_initial_map_status_once)

        self.get_logger().info(
            f"地图上下文管理器启动完成: current_map_id={self.current_map_id}, maps={len(self.maps)}"
        )

    def publish_initial_map_status_once(self):
        """启动后尽快同步一次地图状态。"""
        self.publish_map_status()
        if self.initial_map_status_timer is not None:
            self.initial_map_status_timer.cancel()
            self.initial_map_status_timer = None

    @staticmethod
    def expand_path(path: Any) -> str:
        """展开 ~/ 形式路径。"""
        text = str(path or "").strip()
        return os.path.expanduser(text) if text.startswith("~/") else text

    @staticmethod
    def normalize_map_id(map_id: Any) -> str:
        """归一化地图 ID。"""
        normalized = str(map_id or "hall").strip()
        return normalized or "hall"

    def load_map_registry(self) -> List[Dict[str, Any]]:
        """加载地图注册表；不存在时创建 hall 的最小注册表。"""
        path = Path(self.map_registry_path)
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            default_registry = {
                "default_map_id": self.default_map_id,
                "current_map_id": self.current_map_id,
                "maps": [
                    {
                        "map_id": self.default_map_id,
                        "display_name": "默认地图",
                        "enabled": True,
                        "description": "由多地图一期自动创建的默认地图",
                        "initial_pose": {
                            "frame_id": "map",
                            "position": [0.0, 0.0, 0.0],
                            "orientation": [0.0, 0.0, 0.0, 1.0],
                        },
                    }
                ],
            }
            path.write_text(json.dumps(default_registry, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            return default_registry["maps"]

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            self.default_map_id = self.normalize_map_id(data.get("default_map_id", self.default_map_id))
            self.current_map_id = self.normalize_map_id(data.get("current_map_id", self.default_map_id))
            maps = data.get("maps", [])
            if isinstance(maps, list) and maps:
                return [item for item in maps if isinstance(item, dict)]
        except Exception as exc:
            self.get_logger().error(f"加载地图注册表失败: {path} - {exc}")

        return [{
            "map_id": self.default_map_id,
            "display_name": "默认地图",
            "enabled": True,
            "description": "地图注册表异常时的兜底地图",
        }]

    def build_base_message(self, message_type: str, data_type: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """构造 APP/数据集成层统一消息。"""
        return {
            "protocol_version": "2.0",
            "message_id": f"{data_type}_{int(time.time() * 1000)}",
            "timestamp": time.time(),
            "message_type": message_type,
            "data_type": data_type,
            "source": "map_context_manager",
            "destination": "all",
            "data": data,
            "metadata": {
                "status": data.get("status", "success"),
                "error_code": data.get("error_code", ""),
                "error_message": data.get("message", "") if data.get("status") == "error" else "",
                "request_id": data.get("request_message_id", ""),
            },
        }

    def map_command_callback(self, msg: String):
        """处理 APP 地图管理命令。"""
        try:
            command = json.loads(msg.data)
            command_type = str(command.get("command_type", "")).strip()
            request_message_id = str(command.get("request_message_id", "") or "")
            if command_type == "get_map_list":
                self.send_map_response(command_type, request_message_id, {
                    "status": "success",
                    "current_map_id": self.current_map_id,
                    "default_map_id": self.default_map_id,
                    "maps": self.maps,
                })
            elif command_type == "get_current_map":
                self.send_map_response(command_type, request_message_id, {
                    "status": "success",
                    "current_map_id": self.current_map_id,
                    "default_map_id": self.default_map_id,
                    "current_map": self.find_map(self.current_map_id),
                })
            elif command_type == "switch_map":
                self.handle_switch_map(command, request_message_id)
            else:
                self.send_map_response(command_type or "unknown", request_message_id, {
                    "status": "error",
                    "error_code": "unknown_map_command",
                    "message": f"未知地图命令: {command_type}",
                    "current_map_id": self.current_map_id,
                })
        except Exception as exc:
            self.get_logger().error(f"处理地图命令失败: {exc}")
            self.send_map_response("unknown", "", {
                "status": "error",
                "error_code": "invalid_map_command",
                "message": str(exc),
                "current_map_id": self.current_map_id,
            })

    def find_map(self, map_id: str) -> Dict[str, Any]:
        """按 map_id 查找注册表条目。"""
        normalized_map_id = self.normalize_map_id(map_id)
        for item in self.maps:
            if self.normalize_map_id(item.get("map_id")) == normalized_map_id:
                return dict(item)
        return {}

    def handle_switch_map(self, command: Dict[str, Any], request_message_id: str):
        """执行导航前地图切换。

        一期切图不在导航过程中重启 Nav2；它的职责是把业务 active map 切到目标地图，
        注入该地图初始位姿，并等待 prior-map 定位健康后向 APP 宣告 ready。
        """
        target_map_id = self.normalize_map_id(command.get("target_map_id", ""))
        target_map = self.find_map(target_map_id)
        if not target_map:
            self.send_map_response("switch_map", request_message_id, {
                "status": "error",
                "error_code": "map_not_registered",
                "message": f"地图未注册: {target_map_id}",
                "current_map_id": self.current_map_id,
                "target_map_id": target_map_id,
            })
            return
        if self.navigation_active:
            self.send_map_response("switch_map", request_message_id, {
                "status": "error",
                "error_code": "map_switch_rejected_route_task_active",
                "message": f"当前导航状态不允许切图: {self.navigation_detailed_state}",
                "current_map_id": self.current_map_id,
                "target_map_id": target_map_id,
                "map_state": self.map_state,
            })
            return
        if not target_map.get("enabled", True):
            self.send_map_response("switch_map", request_message_id, {
                "status": "error",
                "error_code": "map_disabled",
                "message": f"地图已禁用: {target_map_id}",
                "current_map_id": self.current_map_id,
                "target_map_id": target_map_id,
            })
            return
        map_yaml_file = str(target_map.get("map_yaml_file", "") or "")
        if map_yaml_file and not Path(map_yaml_file).exists():
            self.send_map_response("switch_map", request_message_id, {
                "status": "error",
                "error_code": "map_file_missing",
                "message": f"地图文件不存在: {map_yaml_file}",
                "current_map_id": self.current_map_id,
                "target_map_id": target_map_id,
                "map_yaml_file": map_yaml_file,
            })
            return
        if self.map_state in ("switching", "localization_resetting", "waiting_localization"):
            self.send_map_response("switch_map", request_message_id, {
                "status": "error",
                "error_code": "map_switch_in_progress",
                "message": "已有地图切换正在进行",
                "current_map_id": self.current_map_id,
                "target_map_id": self.switch_context.get("target_map_id", "") if self.switch_context else target_map_id,
            })
            return
        if target_map_id == self.current_map_id and self.map_state == "ready":
            self.send_map_response("switch_map", request_message_id, {
                "status": "success",
                "result_reason": "already_active",
                "message": "目标地图已经处于 ready 状态",
                "current_map_id": self.current_map_id,
                "target_map_id": target_map_id,
                "map_state": self.map_state,
                "localization_state": self.localization_state,
            })
            return

        self.map_state = "switching"
        self.localization_state = "resetting"
        self.localization_stable_count = 0
        self.switch_context = {
            "request_message_id": request_message_id,
            "target_map_id": target_map_id,
            "target_map": target_map,
            "started_at": time.time(),
            "initialpose_sent": 0,
            "last_initialpose_time": 0.0,
        }
        self.current_map_id = target_map_id
        self.persist_current_map_id(target_map_id)
        self.send_map_response("switch_map", request_message_id, {
            "status": "success",
            "result_reason": "map_switch_started",
            "message": "地图切换已开始，等待定位稳定",
            "current_map_id": self.current_map_id,
            "target_map_id": target_map_id,
            "map_state": self.map_state,
            "localization_state": self.localization_state,
        })
        self.publish_map_status()
        self.switch_watchdog_tick()

    def persist_current_map_id(self, map_id: str):
        """把当前激活地图写回注册表，机器人重启后仍能知道上次激活地图。"""
        path = Path(self.map_registry_path)
        try:
            data = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
            data["default_map_id"] = self.default_map_id
            data["current_map_id"] = self.normalize_map_id(map_id)
            data["maps"] = self.maps
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        except Exception as exc:
            self.get_logger().warning(f"写入当前地图失败: {exc}")

    def localization_status_callback(self, msg: String):
        """监听 prior-map 定位健康状态，用于切图后的 ready 判定。"""
        try:
            data = json.loads(msg.data)
            state = str(data.get("state", data.get("status", ""))).upper()
            healthy = bool(data.get("healthy", False)) or state in ("ACCEPTED", "LOCALIZED", "HEALTHY", "OK", "READY")
        except Exception:
            text = msg.data.strip()
            state = text.split(" ", 1)[0].upper() if text else "UNKNOWN"
            healthy = state == "ACCEPTED"
        if healthy:
            self.localization_stable_count += 1
            self.localization_state = "stable"
        else:
            self.localization_stable_count = 0
            self.localization_state = state.lower() if state else "unstable"

    def navigation_status_callback(self, msg: String):
        """监听导航状态，导航/暂停/播报等待期间拒绝切图。"""
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if payload.get("event_type"):
            return
        self.navigation_active = bool(payload.get("is_active", False))
        detailed_state = str(payload.get("detailed_state", "") or "")
        current_state = str(payload.get("current_state", "") or "")
        route_task = payload.get("route_task", {}) if isinstance(payload.get("route_task", {}), dict) else {}
        if route_task.get("awaiting_broadcast"):
            self.navigation_active = True
        if current_state in ("executing", "paused", "reached_waypoint"):
            self.navigation_active = True
        self.navigation_detailed_state = detailed_state or current_state

    def switch_watchdog_tick(self):
        """推进切图状态机：发初始位姿、等待定位稳定或超时。"""
        if not self.switch_context:
            return

        now = time.time()
        target_map_id = self.switch_context["target_map_id"]
        request_message_id = self.switch_context["request_message_id"]

        if now - self.switch_context["started_at"] > self.switch_localization_timeout_sec:
            self.map_state = "failed"
            self.send_map_response("switch_map", request_message_id, {
                "status": "error",
                "error_code": "map_switch_localization_timeout",
                "message": "切图后等待定位稳定超时",
                "current_map_id": self.current_map_id,
                "target_map_id": target_map_id,
                "map_state": self.map_state,
                "localization_state": self.localization_state,
            })
            self.switch_context = None
            self.publish_map_status()
            return

        if self.switch_context["initialpose_sent"] < self.initialpose_repeat_count:
            if now - self.switch_context["last_initialpose_time"] >= self.initialpose_repeat_interval_sec:
                self.map_state = "localization_resetting"
                self.publish_initial_pose(self.switch_context["target_map"])
                self.switch_context["initialpose_sent"] += 1
                self.switch_context["last_initialpose_time"] = now
                self.publish_map_status()
            return

        self.map_state = "waiting_localization"
        if self.localization_stable_count >= self.switch_localization_stable_frames:
            self.map_state = "ready"
            self.localization_state = "stable"
            self.send_map_response("switch_map", request_message_id, {
                "status": "success",
                "result_reason": "map_ready",
                "message": "地图切换完成，定位已稳定，可以开始导航",
                "current_map_id": self.current_map_id,
                "target_map_id": target_map_id,
                "map_state": self.map_state,
                "localization_state": self.localization_state,
            })
            self.switch_context = None
        self.publish_map_status()

    def publish_initial_pose(self, map_info: Dict[str, Any]):
        """发布目标地图预设初始位姿到 /initialpose。"""
        initial_pose = map_info.get("initial_pose", {}) if isinstance(map_info, dict) else {}
        position = initial_pose.get("position", [0.0, 0.0, 0.0])
        orientation = initial_pose.get("orientation", [0.0, 0.0, 0.0, 1.0])
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(initial_pose.get("frame_id", "map") or "map")
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2] if len(position) > 2 else 0.0)
        msg.pose.pose.orientation.x = float(orientation[0])
        msg.pose.pose.orientation.y = float(orientation[1])
        msg.pose.pose.orientation.z = float(orientation[2])
        msg.pose.pose.orientation.w = float(orientation[3])
        # 给初始位姿一个保守协方差，避免定位端把它当作绝对精确。
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initialpose_pub.publish(msg)

    def send_map_response(self, command_type: str, request_message_id: str, data: Dict[str, Any]):
        """发布地图管理响应。"""
        payload = dict(data)
        payload.update({
            "command_type": command_type,
            "request_message_id": request_message_id,
            "timestamp": time.time(),
        })
        msg = String()
        msg.data = json.dumps(self.build_base_message("response", "map_response", payload), ensure_ascii=False)
        self.map_response_pub.publish(msg)

    def publish_map_status(self):
        """周期发布当前地图快照，方便调试面板/APP 状态栏显示。"""
        data = {
            "status": "success",
            "current_map_id": self.current_map_id,
            "default_map_id": self.default_map_id,
            "map_state": self.map_state,
            "localization_state": self.localization_state,
            "localization_stable_count": self.localization_stable_count,
            "switch_target_map_id": self.switch_context.get("target_map_id", "") if self.switch_context else "",
            "navigation_active": self.navigation_active,
            "navigation_detailed_state": self.navigation_detailed_state,
            "maps_count": len(self.maps),
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(self.build_base_message("push", "map_status", data), ensure_ascii=False)
        self.map_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapContextManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到键盘中断，关闭地图上下文管理器")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
