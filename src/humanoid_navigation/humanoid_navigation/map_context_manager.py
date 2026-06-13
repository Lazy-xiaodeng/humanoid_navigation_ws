#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图上下文管理器 - 多地图一期基础能力。

一期只做“地图列表/当前地图查询”和“切图命令受控拒绝”，不自动重启定位和 Nav2。
这样 APP 可以先建立 map_id 业务闭环，真正切图与定位重置放到下一阶段联调。
"""

import json
import os
import time
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MapContextManager(Node):
    """维护当前地图上下文，并响应 APP 的 map_management 查询命令。"""

    def __init__(self):
        super().__init__("map_context_manager")
        self.declare_parameters(namespace="", parameters=[
            ("map_registry_path", "/home/ubuntu/software/Todesk/Files/humanoid_ws/data/maps/map_registry.json"),
            ("default_map_id", "hall"),
        ])

        self.map_registry_path = self.expand_path(self.get_parameter("map_registry_path").value)
        self.default_map_id = self.normalize_map_id(self.get_parameter("default_map_id").value)
        self.current_map_id = self.default_map_id
        self.maps = self.load_map_registry()

        self.map_command_sub = self.create_subscription(
            String, "/app/map_command", self.map_command_callback, 10
        )
        self.map_response_pub = self.create_publisher(String, "/map/response", 10)
        self.map_status_pub = self.create_publisher(String, "/map/status", 10)
        self.create_timer(5.0, self.publish_map_status)

        self.get_logger().info(
            f"地图上下文管理器启动完成: current_map_id={self.current_map_id}, maps={len(self.maps)}"
        )

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
                # 一期不做自动切图，避免未重启定位/Nav2 时 APP 误以为已经切到新地图可导航。
                self.send_map_response(command_type, request_message_id, {
                    "status": "error",
                    "error_code": "switch_map_not_implemented",
                    "message": "多地图一期仅支持地图查询，自动切图和定位重置将在下一阶段实现",
                    "current_map_id": self.current_map_id,
                    "target_map_id": self.normalize_map_id(command.get("target_map_id", "")),
                })
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
