#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整WebSocket服务器节点 - 结合业务功能与统一数据流
功能：管理WebSocket连接，处理APP消息，路由数据请求，支持主动推送
"""

import asyncio
import concurrent
import websockets
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import time
import uuid
import threading
from typing import Dict, Set, Optional, Any, List
from concurrent.futures import ThreadPoolExecutor

class CompleteWebSocketServer(Node):
    """
    完整WebSocket服务器节点
    功能：
    1. WebSocket连接管理
    2. APP消息路由处理
    3. 业务功能处理（导航控制、路点管理）
    4. 统一数据流处理（通过数据整合节点）
    5. 主动数据推送
    """

    def __init__(self):
        super().__init__('complete_websocket_server')
        
        # ==================== 参数配置 ====================
        self.declare_parameters(namespace='', parameters=[
            ('server.port', 8765),              # WebSocket服务器端口
            ('server.host', '0.0.0.0'),         # 监听地址
            ('server.max_connections', 50),     # 最大连接数
            ('protocol_version', '2.0'),        # 协议版本
            ('heartbeat_interval', 30.0),       # 心跳间隔（秒）
        ])
        
        # 获取参数
        self.server_port = self.get_parameter('server.port').value
        self.server_host = self.get_parameter('server.host').value
        self.protocol_version = self.get_parameter('protocol_version').value
        
        # ==================== 业务功能状态 ====================
        # 保留原有业务功能的状态
        self.connected_clients = {}              # 客户端连接管理
        self.client_sessions = {}               # 客户端会话信息
        self.business_state = {                  # 业务状态存储
            'dynamic_waypoints': {},            # 动态路点数据
            'navigation_sequences': {},         # 导航序列
            'current_navigation_mode': None,    # 当前导航模式
            'current_waypoint_sequence': [],    # 当前导航序列
            'current_waypoint_index': 0,        # 当前路点索引
        }
        
        # 客户端订阅管理
        self.client_subscriptions = {}          # 客户端数据订阅：{client_id: {data_type: subscription_info}}
        
        # 线程锁
        self.client_lock = threading.RLock()
        self.subscription_lock = threading.RLock()
        
        # 服务器状态
        self.server_running = False
        self.server_instance = None
        self.server_loop = None
        
         # 修复：添加事件循环和线程池
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=4)
        self.server_thread = None
        # ==================== 设置完整ROS通信接口 ====================
        self.setup_complete_ros_communication()
        
        # ==================== 启动WebSocket服务器 ====================
        self.start_websocket_server_in_thread()
        
        # ==================== 启动定时任务 ====================
        self.setup_timers()
        
        self.get_logger().info('🚀 完整WebSocket服务器启动完成')
        self.get_logger().info(f'📡 服务器地址: ws://{self.server_host}:{self.server_port}')
        self.get_logger().info(f'📊 协议版本: {self.protocol_version}')
    
    def setup_complete_ros_communication(self):
        """设置完整的ROS通信接口 - 保留业务功能，集成统一数据流"""
        try:
            # ==================== 业务功能发布器 ====================
            
            # 1. 动态路点管理命令
            self.waypoint_command_pub = self.create_publisher(
                String, '/app/waypoint_command', 10
            )
            
            # 2. 导航控制命令
            self.navigation_command_pub = self.create_publisher(
                String, '/app/navigation_command', 10
            )
            
            # 3. 机器人控制命令
            self.robot_control_pub = self.create_publisher(
                String, '/app/robot_control', 10
            )
            
            # 4. 系统管理命令
            self.system_command_pub = self.create_publisher(
                String, '/app/system_command', 10
            )
            # 5. 面部控制指令发布器 ---
            self.facial_cmd_pub = self.create_publisher(
                String, '/robot/facial_raw_cmd', 10
            )
            
            # ==================== 统一数据流接口 ====================
            
            # 1. 数据请求发布器（向数据整合节点请求数据）
            self.data_request_pub = self.create_publisher(
                String, '/websocket/data_requests', 10
            )
            
            # 2. 数据订阅管理发布器
            self.subscription_pub = self.create_publisher(
                String, '/websocket/data_subscriptions', 10
            )
            
            # ==================== 业务功能订阅器 ====================
            
            # 1. 动态路点更新
            self.waypoints_update_sub = self.create_subscription(
                 String, '/navigation/waypoints_data',  
                 self.business_waypoints_callback, 10
            )
            
            # 3. 导航序列更新
            self.navigation_sequences_sub = self.create_subscription(
                String, '/navigation/sequences', 
                self.business_navigation_sequences_callback, 10
            )
            
            # ==================== 统一数据流订阅器 ====================
            
            # 1. 数据整合节点响应
            self.data_response_sub = self.create_subscription(
                String, '/integration/data_responses', 
                self.unified_data_response_callback, 10
            )
            
            # 2. 主动推送消息
            self.push_message_sub = self.create_subscription(
                String, '/integration/push_messages', 
                self.unified_push_message_callback, 10
            )
            
            # 3. 订阅管理响应
            self.subscription_response_sub = self.create_subscription(
                String, '/integration/subscription_responses', 
                self.unified_subscription_response_callback, 10
            )
            
            self.get_logger().info('✅ 完整ROS通信接口设置完成')
            
        except Exception as e:
            self.get_logger().error(f'❌ 设置ROS通信失败: {e}')
            raise
    
    def start_websocket_server_in_thread(self):
        """在线程中启动WebSocket服务器（避免事件循环冲突）"""
        try:
            # 在线程池中启动服务器
            future = self.thread_pool.submit(self._run_websocket_server_blocking)
            self.server_thread = future
            
            self.get_logger().info(f'✅ 开始启动WebSocket服务器: {self.server_host}:{self.server_port}')
            
            # 检查服务器是否成功启动
            time.sleep(1)  # 给服务器一点启动时间
            
        except Exception as e:
            self.get_logger().error(f'❌ 启动WebSocket服务器失败: {e}')
            self.server_running = False

    def _run_websocket_server_blocking(self):
        """阻塞方式运行WebSocket服务器（在线程中）"""
        try:
            # 创建新的事件循环
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self.server_loop = loop
            
            # 创建服务器
            self.get_logger().info('🔄 创建WebSocket服务器实例...')
            
            # 使用异步函数创建服务器
            async def create_server():
                server = await websockets.serve(
                    self.handle_client_connection,
                    self.server_host,
                    self.server_port,
                    ping_interval=20,
                    ping_timeout=10,
                    max_size=10 * 1024 * 1024,
                )
                return server
            
            # 运行异步函数
            self.server_instance = loop.run_until_complete(create_server())
            self.server_running = True
            
            self.get_logger().info(f'✅ WebSocket服务器启动成功: {self.server_host}:{self.server_port}')
            self.get_logger().info('🔄 WebSocket服务器正在运行...')
            
            # 运行事件循环
            loop.run_forever()
            
        except Exception as e:
            self.get_logger().error(f'❌ WebSocket服务器运行错误: {e}')
            import traceback
            self.get_logger().error(f'❌ 详细错误信息: {traceback.format_exc()}')
            self.server_running = False
    
    def setup_timers(self):
        """设置定时任务"""
        # 连接健康检查定时器
        self.health_check_timer = self.create_timer(30.0, self.check_connections_health)
        
        # 状态报告定时器
        self.status_report_timer = self.create_timer(60.0, self.report_server_status)
        
        self.get_logger().info('✅ 定时任务设置完成')
    
    def generate_client_id(self) -> str:
        """生成唯一客户端ID"""
        timestamp = int(time.time() * 1000)
        random_suffix = uuid.uuid4().hex[:8]
        return f"client_{timestamp}_{random_suffix}"
    
    def generate_message_id(self, prefix: str = "msg") -> str:
        """生成唯一消息ID"""
        timestamp = int(time.time() * 1000)
        random_suffix = uuid.uuid4().hex[:8]
        return f"{prefix}_{timestamp}_{random_suffix}"
    
    def create_base_message(self, message_type: str, data_type: str, 
                          source: str = "websocket_server",
                          destination: str = "all") -> Dict[str, Any]:
        """
        创建基础消息结构
        所有发送给APP的消息都使用此格式
        
        字段说明：
        - protocol_version: 协议版本，用于兼容性管理
        - message_id: 唯一消息ID，用于消息追踪
        - timestamp: 消息创建时间戳（Unix时间，秒.毫秒）
        - message_type: 消息类型（request/response/push/subscription/command）
        - data_type: 数据类型，标识消息承载的数据内容类型
        - source: 消息来源标识
        - destination: 消息目的地标识或"all"
        - data: 业务数据载体
        - metadata: 元数据区，包含状态、错误信息等
        """
        return {
            "protocol_version": self.protocol_version,
            "message_id": self.generate_message_id(message_type),
            "timestamp": time.time(),
            "message_type": message_type,
            "data_type": data_type,
            "source": source,
            "destination": destination,
            "data": {},
            "metadata": {
                "status": "success",           # 操作状态：success/error/pending
                "error_code": "",              # 错误代码，标准化错误标识
                "error_message": "",          # 错误描述，人类可读的错误信息
                "request_id": "",             # 关联请求ID，用于请求-响应匹配
                "data_freshness": 0.0,        # 数据新鲜度，数据产生到发送的延迟（秒）
                "qos_level": "standard"       # 服务质量：realtime/standard/low_priority
            }
        }
    
    # ==================== WebSocket连接管理 ====================
    
    async def handle_client_connection(self, websocket):
        """处理客户端WebSocket连接"""
        client_id = self.generate_client_id()
        client_ip = websocket.remote_address[0] if websocket.remote_address else "unknown"
        #self.get_logger().debug(f"客户端连接路径: {path}")
        try:
            # 注册客户端
            with self.client_lock:
                self.connected_clients[client_id] = websocket
                self.client_sessions[client_id] = {
                    'id': client_id,
                    'ip': client_ip,
                    'connected_time': time.time(),
                    'last_activity': time.time(),
                    'subscriptions': set(),  # 默认无订阅
                    'authenticated': False
                }
            
            self.get_logger().info(f'🔗 客户端连接: {client_id} (IP: {client_ip})')
            
            # 发送连接确认
            await self.send_connection_ack(websocket, client_id)
            
            # 发送初始业务数据
            await self.send_initial_business_data(websocket, client_id)
            
            # 主消息处理循环
            async for message in websocket:
                try:
                    # 更新活动时间
                    self.client_sessions[client_id]['last_activity'] = time.time()
                    
                    # 处理客户端消息
                    await self.handle_client_message(websocket, message, client_id)
                    
                except websockets.exceptions.ConnectionClosed:
                    self.get_logger().info(f'🔌 客户端 {client_id} 连接已关闭')
                    break
                except Exception as e:
                    self.get_logger().error(f'❌ 处理客户端消息错误 {client_id}: {e}')
                    await self.send_error_to_client(websocket, client_id, f"处理消息失败: {str(e)}")
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理客户端连接错误 {client_id}: {e}')
        finally:
            # 清理客户端资源
            await self.cleanup_client(client_id)
            self.get_logger().info(f'🔌 客户端 {client_id} 已断开连接')
    
    async def send_connection_ack(self, websocket, client_id: str):
        """发送连接确认消息"""
        try:
            ack_message = self.create_base_message("response", "connection_ack", "websocket_server", client_id)
            ack_message["data"] = {
                "status": "connected",                      # 连接状态
                "client_id": client_id,                     # 分配的客户端ID
                "server_time": time.time(),                 # 服务器当前时间
                "protocol_version": self.protocol_version,  # 支持的协议版本
                "supported_data_types": [                   # 支持的数据类型列表
                    "robot_pose",          # 机器人定位数据
                    "robot_speed",         # 机器人实时速度               
                    "navigation_path",     # 路径规划数据
                    "navigation_status",   # 导航状态数据
                    "system_status",       # 系统状态数据
                    "waypoints_data",      # 路点数据
                ],
                "supported_commands": [                     # 支持的命令类型列表
                    "waypoint_management",  # 路点管理命令
                    "navigation_control",   # 导航控制命令
                    "robot_control",        # 机器人控制命令
                    "system_command",       # 系统管理命令
                ],
                "subscription_supported": True,             # 是否支持订阅功能
                "heartbeat_interval": 30.0,                 # 建议的心跳间隔
            }
            ack_message["metadata"]["message"] = "WebSocket连接成功"
            
            await websocket.send(json.dumps(ack_message, ensure_ascii=False))
            
            self.get_logger().info(f'✅ 发送连接确认给 {client_id}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 发送连接确认错误: {e}')
    
    async def send_initial_business_data(self, websocket, client_id: str):
        """发送初始业务数据给新连接的客户端"""
        try:
            # 发送动态路点数据
            if self.business_state['dynamic_waypoints']:
                waypoints_msg = self.create_base_message("push", "waypoints_data", "websocket_server", client_id)
                waypoints_msg["data"] = self.business_state['dynamic_waypoints']
                waypoints_msg["metadata"]["push_reason"] = "initial_data"
                await websocket.send(json.dumps(waypoints_msg, ensure_ascii=False))
            
            # 发送导航序列数据
            if self.business_state['navigation_sequences']:
                sequences_msg = self.create_base_message("push", "navigation_sequences", "websocket_server", client_id)
                sequences_msg["data"] = self.business_state['navigation_sequences']
                sequences_msg["metadata"]["push_reason"] = "initial_data"
                await websocket.send(json.dumps(sequences_msg, ensure_ascii=False))
            
            # 发送动作库数据
            if 'gesture_list' in self.business_state:
                gesture_msg = self.create_base_message("push", "gesture_list", "websocket_server", client_id)
                gesture_msg["data"] = self.business_state['gesture_list']
                gesture_msg["metadata"]["push_reason"] = "initial_data_sync"
                await websocket.send(json.dumps(gesture_msg, ensure_ascii=False))
                self.get_logger().info(f'🤖 已向 {client_id} 推送动作库')
            
            # 推送面部表情库
            if 'facial_gesture_list' in self.business_state:
                facial_msg = self.create_base_message("push", "facial_gesture_list", "websocket_server", client_id)
                facial_msg["data"] = self.business_state['facial_gesture_list']
                facial_msg["metadata"]["push_reason"] = "initial_data_sync"
                await websocket.send(json.dumps(facial_msg, ensure_ascii=False))
                self.get_logger().info(f'🎭 已向 {client_id} 推送面部表情库')

            # 发送当前导航状态
            if self.business_state['current_navigation_mode']:
                status_msg = self.create_base_message("push", "navigation_status", "websocket_server", client_id)
                status_msg["data"] = {
                    "navigation_mode": self.business_state['current_navigation_mode'],
                    "current_waypoint_index": self.business_state['current_waypoint_index'],
                    "waypoint_sequence": self.business_state['current_waypoint_sequence']
                }
                status_msg["metadata"]["push_reason"] = "initial_data"
                await websocket.send(json.dumps(status_msg, ensure_ascii=False))
            
            self.get_logger().info(f'📊 发送初始业务数据给 {client_id}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 发送初始业务数据错误 {client_id}: {e}')
    
    async def cleanup_client(self, client_id: str):
        """清理客户端资源"""
        try:
            subscriptions_to_remove = []  # 用于临时存储需要取消的订阅

            with self.client_lock:
                if client_id in self.connected_clients:
                    del self.connected_clients[client_id]
                if client_id in self.client_sessions:
                    del self.client_sessions[client_id]
                
                # 先取出订阅列表，再从内存中删除
                # 否则一旦删除了，后面就不知道通过ROS告诉整合节点要取消什么了
                if client_id in self.client_subscriptions:
                    subscriptions_to_remove = list(self.client_subscriptions[client_id].keys())
                    del self.client_subscriptions[client_id]
            
            # 通知数据整合节点取消订阅 (如果该客户端有订阅的话)
            if subscriptions_to_remove:
                await self.remove_client_subscriptions(client_id, subscriptions_to_remove)
            
            self.get_logger().info(f'🧹 清理客户端资源完成: {client_id}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 清理客户端资源错误 {client_id}: {e}')


    async def remove_client_subscriptions(self, client_id: str, data_types: List[str]):
        """
        [补全] 通知数据整合节点取消该客户端的订阅
        """
        try:
            if not data_types:
                return

            # 构建符合数据整合节点协议的“取消订阅”消息
            unsubscribe_msg = {
                "protocol_version": self.protocol_version,
                "message_id": self.generate_message_id("unsub"),
                "timestamp": time.time(),
                "message_type": "subscription",
                "data_type": "subscription_manage",
                "source": client_id,
                "destination": "data_integration",
                "data": {
                    "action": "unsubscribe",
                    "data_types": data_types  # 告诉整合节点：这些数据不要再发给这个人了
                }
            }
            
            # 发布 ROS 消息
            msg = String()
            msg.data = json.dumps(unsubscribe_msg, ensure_ascii=False)
            self.subscription_pub.publish(msg)
            
            self.get_logger().info(f'📉 已发送退订请求: {client_id} -> {data_types}')
                
        except Exception as e:
            self.get_logger().error(f'❌ 发送取消订阅请求失败: {e}')
    
    # ==================== 客户端消息处理 ====================
    async def handle_client_message(self, websocket, message: str, client_id: str):
        """
        处理客户端发送的消息，出现异常时同时向客户端和终端打印详细错误信息
        """
        try:
            # 先检查连接状态
            if client_id not in self.connected_clients or self.connected_clients[client_id] != websocket:
                self.get_logger().warning(f"客户端 {client_id} 连接状态异常，跳过消息处理")
                return

            # 解析JSON
            message_data = json.loads(message)

            # 验证格式
            is_valid, error_msg = self.validate_client_message(message_data)
            if not is_valid:
                await self.send_error_to_client(websocket, client_id, error_msg)
                return

            # 获取消息类型
            message_type = message_data.get("message_type", "unknown")
            data_type = message_data.get("data_type", "unknown")

            self.get_logger().info(f'📨 收到消息: {message_type} {data_type} from {client_id}')

            # 分发处理
            if message_type == "request":
                await self.handle_data_request(websocket, message_data, client_id)
            elif message_type == "subscription":
                await self.handle_subscription_request(websocket, message_data, client_id)
            elif message_type == "command":
                await self.handle_business_command(websocket, message_data, client_id)
            else:
                await self.send_error_to_client(websocket, client_id, f"不支持的消息类型: {message_type}")

        except json.JSONDecodeError as e:
            # 解析JSON错误，立即反馈错误信息给客户端，打印详细日志
            error_detail = f"JSON解析错误：{str(e)}"
            self.get_logger().error(error_detail, exc_info=True)
            await self.safe_send_error(websocket, client_id, error_detail)

        except Exception as e:
            # 其他异常，打印完整堆栈，向客户端发送错误消息
            error_detail = f"处理消息时遇到异常：{str(e)}"
            self.get_logger().error(error_detail, exc_info=True)
            await self.safe_send_error(websocket, client_id, error_detail)

    async def safe_send_error(self, websocket, client_id: str, error_message: str):
        """
        安全发送错误消息给客户端，防止发送过程中异常导致递归错误
        """
        try:
            # 构建标准错误消息格式
            error_msg = self.create_base_message("response", "error", "websocket_server", client_id)
            error_msg["metadata"].update({
                "status": "error",
                "error_code": "PROCESSING_ERROR",
                "error_message": error_message
            })

            await websocket.send(json.dumps(error_msg, ensure_ascii=False))

        except Exception as e:
            # 如果发送错误消息过程中再发生异常，打印日志但不再抛出
            self.get_logger().error(f"发送错误消息失败: {e}", exc_info=True)
    
    
    def validate_client_message(self, message: Dict) -> tuple[bool, str]:
        """验证客户端消息格式"""
        try:
            # 检查必需字段
            required_fields = ["protocol_version", "message_id", "message_type", "data_type", "source"]
            for field in required_fields:
                if field not in message:
                    return False, f"缺少必需字段: {field}"
            
            # 检查消息类型
            valid_message_types = ["request", "response", "push", "subscription", "command"]
            if message["message_type"] not in valid_message_types:
                return False, f"无效的消息类型: {message['message_type']}"
            
            # 检查协议版本兼容性
            if message["protocol_version"] != self.protocol_version:
                self.get_logger().warning(f'⚠️ 协议版本不匹配: {message["protocol_version"]}')
                # 不拒绝，允许版本协商
            
            return True, "验证通过"
            
        except Exception as e:
            return False, f"消息验证错误: {str(e)}"
    
    # ==================== 统一数据流处理 ====================
    
    async def handle_data_request(self, websocket, message_data: Dict, client_id: str):
        """处理数据请求消息 - 转发到数据整合节点"""
        try:
            # 设置消息来源为客户端
            message_data["source"] = client_id
            
            # 发布数据请求到数据整合节点
            request_msg = String()
            request_msg.data = json.dumps(message_data, ensure_ascii=False)
            self.data_request_pub.publish(request_msg)
            
            # 立即返回接收确认
            ack_message = self.create_base_message("response", "request_ack", "websocket_server", client_id)
            ack_message["data"] = {
                "request_id": message_data.get("message_id", ""),
                "status": "received",
                "message": "数据请求已接收，正在处理"
            }
            ack_message["metadata"]["request_id"] = message_data.get("message_id", "")
            
            await websocket.send(json.dumps(ack_message, ensure_ascii=False))
            
            self.get_logger().info(f'📤 转发数据请求: {message_data.get("data_type", "unknown")} from {client_id}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理数据请求错误: {e}')
            await self.send_error_to_client(websocket, client_id, f"处理数据请求失败: {str(e)}")
    
    async def handle_subscription_request(self, websocket, message_data: Dict, client_id: str):
        """处理订阅请求消息"""
        try:
            # 设置消息来源为客户端
            message_data["source"] = client_id
        
            # 发布订阅请求到数据整合节点
            subscription_msg = String()
            subscription_msg.data = json.dumps(message_data, ensure_ascii=False)
            self.subscription_pub.publish(subscription_msg)
            
            # 发送订阅确认
            ack_message = self.create_base_message("response", "subscription_ack", "websocket_server", client_id)
            ack_message["data"] = {
                "status": "forwarded",
                "message": "订阅请求已转发到数据服务"
            }
            ack_message["metadata"]["request_id"] = message_data.get("message_id", "")
            
            await websocket.send(json.dumps(ack_message, ensure_ascii=False))
            
            self.get_logger().info(f'📋 转发订阅请求: {message_data.get("data_type", "unknown")}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理订阅请求错误: {e}')
            await self.send_error_to_client(websocket, client_id, f"处理订阅请求失败: {str(e)}")
    
    # ==================== 业务功能处理 ====================
    
    async def handle_business_command(self, websocket, message_data: Dict, client_id: str):
        """处理业务命令消息"""
        try:
            command_type = message_data.get("data_type", "")
            command_data = message_data.get("data", {})
            
            # 验证命令格式
            if not command_type:
                await self.send_error_to_client(websocket, client_id, "命令类型不能为空")
                return
            
            # 路由到相应的业务命令处理函数
            if command_type in ["waypoint_management", "navigation_control"]:
                await self.route_to_waypoint_manager(command_type, command_data, client_id)
            # 机器人控制命令直接处理
            elif command_type == "robot_control":
                await self.handle_robot_control(websocket, command_data, client_id)
            elif command_type == "facial_control":
                await self.handle_facial_control(websocket, message_data, client_id)
            else:
                await self.send_error_to_client(websocket, client_id, f"不支持的命令类型: {command_type}")
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理业务命令错误: {e}')
            await self.send_error_to_client(websocket, client_id, f"处理命令失败: {str(e)}")
    
    async def route_to_waypoint_manager(self, command_type: str, command_data: Dict, client_id: str) -> bool:
        """路由命令到动态路点管理器"""
        try:
            # 展开嵌套的命令结构
            if command_type == "waypoint_management":
            # 提取内层的实际命令类型
                inner_command = command_data.get("command_type", "")
                if not inner_command:
                    self.get_logger().error("路点管理命令缺少 command_type")
                    return False
            
                # 构建扁平化的消息
                route_msg = {
                    "command_type": inner_command,
                    "waypoint_data": command_data.get("waypoint_data", {}),
                    "waypoint_id": command_data.get("waypoint_id", ""),
                    "waypoint_type": command_data.get("waypoint_type", ""),
                    "include_details": command_data.get("include_details", True),
                    "client_id": client_id,
                    "timestamp": time.time()
                }
            
                # 发布到路点管理话题
                msg = String()
                msg.data = json.dumps(route_msg, ensure_ascii=False)
                self.waypoint_command_pub.publish(msg)
                self.get_logger().info(f' 路由路点命令: {inner_command}')
            
            elif command_type == "navigation_control":
                # 提取内层的实际命令类型
                inner_command = command_data.get("command_type", "")
                if not inner_command:
                    self.get_logger().error("导航控制命令缺少 command_type")
                    return False
            
                # 构建扁平化的消息
                route_msg = {
                    "command_type": inner_command,
                    "waypoint_id": command_data.get("waypoint_id", ""),
                    "waypoint_ids": command_data.get("waypoint_ids", []),
                    "exhibition_ids": command_data.get("exhibition_ids", []),
                    "client_id": client_id,
                    "timestamp": time.time()
                }
            
                # 发布到导航控制话题
                msg = String()
                msg.data = json.dumps(route_msg, ensure_ascii=False)
                self.navigation_command_pub.publish(msg)
                self.get_logger().info(f' 路由导航命令: {inner_command}')
            
            else:
                self.get_logger().error(f'不支持的命令类型: {command_type}')
                return False
        
            return True
        
        except Exception as e:
            self.get_logger().error(f'路由命令错误: {e}')
            return False
        

    async def handle_robot_control(self, websocket, command_data: Dict, client_id: str):
        """处理机器人控制命令"""
        try:
            action = command_data.get("action", "")
            control_params = command_data.get("parameters", {})
            
            # 发布到机器人控制系统
            robot_command = {
                "command_type": action,
                "parameters": control_params,
                "client_id": client_id,
                "timestamp": time.time()
            }
        
            command_msg = String()
            command_msg.data = json.dumps(robot_command, ensure_ascii=False)
            self.robot_control_pub.publish(command_msg)
            
            # 发送命令接收确认
            ack_message = self.create_base_message("response", "command_ack", "websocket_server", client_id)
            ack_message["data"] = {
                "command_type": "robot_control",
                "action": action,
                "status": "received",
                "message": f"机器人控制命令'{action}'已接收"
            }
            
            await websocket.send(json.dumps(ack_message, ensure_ascii=False))
            
            self.get_logger().info(f'🤖 处理机器人控制命令: {action} from {client_id}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理机器人控制命令错误: {e}')
            await self.send_error_to_client(websocket, client_id, f"处理机器人命令失败: {str(e)}")
    
    async def handle_facial_control(self, websocket, message_data: Dict, client_id: str):
        """处理面部控制命令（眨眼/动嘴）"""
        try:
            # 1. 获取原始请求的 ID
            request_id = message_data.get("message_id", "")
        
            # 2. 从 data 字段获取具体的 action
            command_inner_data = message_data.get("data", {})
            action = command_inner_data.get("action", "")
        
            # 校验必要字段
            if not action:
               await self.send_error_to_client(websocket, client_id, "面部控制命令缺少 'action' 字段")
               return

            # 构建 ROS 消息并发布 (保持不变)
            ros_msg = String()
            ros_msg.data = action
            if hasattr(self, 'facial_cmd_pub') and self.facial_cmd_pub:
               self.facial_cmd_pub.publish(ros_msg)
               self.get_logger().info(f"✅ [Facial] 已转发指令ID {request_id}: '{action}'")
            else:
               await self.send_error_to_client(websocket, client_id, "面部控制发布器未初始化")
               return

            # 3. 创建响应消息
            ack_message = self.create_base_message("response", "command_ack", "websocket_server", client_id)
        
            # 4. 填充业务数据
            ack_message["data"] = {
               "command_type": "facial_control",
               "action": action,
               "status": "executed",
               "message": f"面部动作 '{action}' 已执行"
            }
        
            # 5. 将请求ID填入元数据的 request_id 中
            ack_message["metadata"]["request_id"] = request_id

            # 发送回 APP
            await websocket.send(json.dumps(ack_message, ensure_ascii=False))
        
        except Exception as e:
            self.get_logger().error(f'❌ 处理面部控制命令错误: {e}')
            # 报错时也尝试带回 ID
            await self.send_error_to_client(websocket, client_id, f"失败: {str(e)}")

    async def handle_system_command(self, websocket, command_data: Dict, client_id: str):
        """处理系统管理命令"""
        try:
            action = command_data.get("action", "")
            sys_params = command_data.get("parameters", {})
            
            # 发布到系统管理器
            system_command = {
                "command_type": action,
                "parameters": sys_params,
                "client_id": client_id,
                "timestamp": time.time()
            }
            
            command_msg = String()
            command_msg.data = json.dumps(system_command, ensure_ascii=False)
            self.system_command_pub.publish(command_msg)
            
            # 发送命令接收确认
            ack_message = self.create_base_message("response", "command_ack", "websocket_server", client_id)
            ack_message["data"] = {
                "command_type": "system_command",
                "action": action,
                "status": "received",
                "message": f"系统管理命令'{action}'已接收"
            }
            
            await websocket.send(json.dumps(ack_message, ensure_ascii=False))
            
            self.get_logger().info(f'⚙️ 处理系统管理命令: {action} from {client_id}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理系统管理命令错误: {e}')
            await self.send_error_to_client(websocket, client_id, f"处理系统命令失败: {str(e)}")
    
    # ==================== 业务功能回调处理 ====================
    
    def business_waypoints_callback(self, msg: String):
        """处理路点数据更新和响应"""
        try:
            message_data = json.loads(msg.data)
        
            # 检查是否是统一格式消息
            if "protocol_version" not in message_data:
                return
        
            data_type = message_data.get("data_type", "")
        
            # 处理路点数据更新
            if data_type == "waypoints_data":
                inner_data = message_data.get("data", {})
                update_type = inner_data.get("update_type", "")
            
                if update_type == "full_update":
                    waypoints_data = inner_data.get("data", {})
                    self.business_state['dynamic_waypoints'] = waypoints_data.get("waypoints", {})
                    self.business_state['navigation_sequences'] = waypoints_data.get("sequences", {})
                
                    self.get_logger().info(f' 路点数据已更新')
        
            # 处理路点操作响应
            elif data_type == "waypoint_response":
                self.get_logger().info(f' 收到路点操作响应')
        
            # 广播给所有客户端
            if self.server_loop and self.server_loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    self.broadcast_to_all_clients(message_data),
                    self.server_loop
                )
        
        except Exception as e:
            self.get_logger().error(f'❌ 处理路点回调错误: {e}')
    
    
    def business_navigation_sequences_callback(self, msg: String):
        """处理导航序列更新回调"""
        try:
            sequences_data = json.loads(msg.data)
            
            # 更新服务器状态
            self.business_state['navigation_sequences'] = sequences_data.get("sequences", {})
            
            # 转换为统一消息格式并广播
            if self.server_loop and self.server_loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    self.broadcast_business_update("navigation_sequences", sequences_data),
                    self.server_loop
                )
            
            self.get_logger().info(f'🛣️ 收到导航序列更新，共 {len(self.business_state["navigation_sequences"])} 个序列')
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理导航序列更新错误: {e}')
    
    
    async def broadcast_business_update(self, data_type: str, data: Dict):
        """广播业务数据更新给订阅的客户端"""
        try:
            # 转换为统一消息格式
            push_message = self.create_base_message("push", data_type, "websocket_server", "subscribed")
            push_message["data"] = data
            push_message["metadata"]["push_reason"] = "business_update"
            
            # 获取订阅了该数据类型的客户端
            subscribed_clients = self.get_subscribed_clients(data_type)
            
            for client_id in subscribed_clients:
                if client_id in self.connected_clients:
                    try:
                        push_message["destination"] = client_id
                        await self.connected_clients[client_id].send(
                            json.dumps(push_message, ensure_ascii=False)
                        )
                    except Exception as e:
                        self.get_logger().debug(f'广播业务更新到 {client_id} 失败: {e}')
            
            self.get_logger().debug(f'📢 广播业务更新 {data_type} 给 {len(subscribed_clients)} 个客户端')
            
        except Exception as e:
            self.get_logger().error(f'❌ 广播业务更新错误: {e}')
    
    def get_subscribed_clients(self, data_type: str) -> List[str]:
        """获取订阅了特定数据类型的客户端列表"""
        try:
            subscribed_clients = []
            for client_id, subscriptions in self.client_subscriptions.items():
                if data_type in subscriptions:
                    subscribed_clients.append(client_id)
            return subscribed_clients
        except Exception as e:
            self.get_logger().error(f'❌ 获取订阅客户端错误: {e}')
            return []
    
    # ==================== 统一数据流回调处理 ====================
    
    def unified_data_response_callback(self, msg: String):
        """处理数据整合节点的响应消息"""
        try:
            response_data = json.loads(msg.data)
            destination = response_data.get("destination", "all")
            
             # 【关键防御】确保 server_loop 存在且在运行
            if not self.server_loop or not self.server_loop.is_running():
                self.get_logger().warning(f"⚠️ 无法路由响应消息（server_loop不可用）: {response_data.get('data_type', 'unknown')}")
                return

            # 路由响应消息到目标客户端
            asyncio.run_coroutine_threadsafe(
                self.route_data_response(response_data, destination),
                self.server_loop
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理数据响应错误: {e}')
    
    async def route_data_response(self, response_data: Dict, destination: str):
        """路由数据响应到目标客户端"""
        try:
            if destination == "all":
                # 广播给所有客户端
                await self.broadcast_to_all_clients(response_data)
            else:
                # 发送给特定客户端
                if destination in self.connected_clients:
                    await self.connected_clients[destination].send(
                        json.dumps(response_data, ensure_ascii=False)
                    )
            
            self.get_logger().debug(f'📤 路由数据响应: {response_data.get("data_type", "unknown")} to {destination}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 路由数据响应错误: {e}')
    
    def unified_push_message_callback(self, msg: String):
        """处理主动推送消息"""
        try:
            push_data = json.loads(msg.data)
            inner_data = push_data.get("data", {})
            data_type = push_data.get("data_type", "").replace("_update", "")
            
            # --- 新增的第 3 步：内部状态同步 ---
            if data_type == "navigation_status":
                event_type = inner_data.get("event_type", "")
                if event_type == "navigation_started":
                    self.business_state['current_navigation_mode'] = inner_data.get("navigation_mode")
                    self.business_state['current_waypoint_sequence'] = inner_data.get("waypoint_sequence", [])
                    self.business_state['current_waypoint_index'] = 0
                elif event_type == "waypoint_reached":
                    # 注意：这里从整合节点取索引，NSM 里发出的索引在 inner_data 的 event_data 中
                    event_data = inner_data.get("event_data", {})
                    self.business_state['current_waypoint_index'] = event_data.get("waypoint_index", 0)
                elif event_type in ["navigation_completed", "navigation_stopped"]:
                    self.business_state['current_navigation_mode'] = None
                    self.business_state['current_waypoint_sequence'] = []
                    self.business_state['current_waypoint_index'] = 0
            if data_type == "gesture_list":
                self.business_state['gesture_list'] = push_data.get("data", {})
                # 直接转发给所有已连接客户端
                if self.server_loop and self.server_loop.is_running():
                    asyncio.run_coroutine_threadsafe(
                        self.broadcast_to_all_clients(push_data),
                        self.server_loop
                    )
                return  
            if data_type == "facial_gesture_list":
                self.get_logger().info("🍭 收到面部表情库更新")
                self.business_state['facial_gesture_list'] = push_data.get("data", {})
                # 广播给客户端让 APP 更新 UI
                if self.server_loop and self.server_loop.is_running():
                    asyncio.run_coroutine_threadsafe(
                        self.broadcast_to_all_clients(push_data),
                        self.server_loop
                )
                return
            # -----------------------------------

            destination = push_data.get("destination", "all")
            
            # 原有的路由推送消息逻辑
            if self.server_loop and self.server_loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    self.route_push_message(push_data, destination),
                    self.server_loop
                )
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理推送消息错误: {e}')

    
    async def route_push_message(self, push_data: Dict, destination: str):
        """路由推送消息"""
        try:
            data_type = push_data.get("data_type", "").replace("_update", "")
            
            if destination == "subscribed":
                # 发送给订阅了该数据类型的客户端
                subscribed_clients = self.get_subscribed_clients(data_type)
                for client_id in subscribed_clients:
                    if client_id in self.connected_clients:
                        try:
                            push_data["destination"] = client_id
                            await self.connected_clients[client_id].send(
                                json.dumps(push_data, ensure_ascii=False)
                            )
                        except Exception as e:
                            self.get_logger().debug(f'推送消息到 {client_id} 失败: {e}')
            elif destination == "all":
                # 广播给所有客户端
                await self.broadcast_to_all_clients(push_data)
            else:
                # 发送给特定客户端
                if destination in self.connected_clients:
                    await self.connected_clients[destination].send(
                        json.dumps(push_data, ensure_ascii=False)
                    )
            
            self.get_logger().debug(f'📤 路由推送消息: {data_type} to {len(self.get_subscribed_clients(data_type))} clients')
            
        except Exception as e:
            self.get_logger().error(f'❌ 路由推送消息错误: {e}')
    
    def unified_subscription_response_callback(self, msg: String):
        """处理订阅管理响应"""
        try:
            response_data = json.loads(msg.data)
            destination = response_data.get("destination", "all")
            
            # 路由订阅响应
            if self.server_loop and self.server_loop.is_running():
                asyncio.run_coroutine_threadsafe(
                    self.route_subscription_response(response_data, destination),
                    self.server_loop
                )
            
        except Exception as e:
            self.get_logger().error(f'❌ 处理订阅响应错误: {e}')
    
    async def route_subscription_response(self, response_data: Dict, destination: str):
        """路由订阅响应"""
        try:
            if destination in self.connected_clients:
                await self.connected_clients[destination].send(
                    json.dumps(response_data, ensure_ascii=False)
                )
            
            self.get_logger().debug(f'📤 路由订阅响应 to {destination}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 路由订阅响应错误: {e}')
    
    async def broadcast_to_all_clients(self, message: Dict):
        """广播消息给所有客户端"""
        try:
            disconnected_clients = []
            
            with self.client_lock:
                clients_to_notify = list(self.connected_clients.items())
            
            for client_id, websocket in clients_to_notify:
                try:
                    await websocket.send(json.dumps(message, ensure_ascii=False))
                except Exception as e:
                    self.get_logger().debug(f'广播到 {client_id} 失败: {e}')
                    disconnected_clients.append(client_id)
            
            # 清理断开连接的客户端
            for client_id in disconnected_clients:
                await self.cleanup_client(client_id)
            
            self.get_logger().debug(f'📢 广播消息给 {len(clients_to_notify)} 个客户端')
            
        except Exception as e:
            self.get_logger().error(f'❌ 广播消息错误: {e}')
    
    # ==================== 错误处理和工具函数 ====================
    
    async def send_error_to_client(self, websocket, client_id: str, error_message: str, error_code: str = "PROCESSING_ERROR"):
        """发送错误消息到客户端"""
        try:
            error_msg = self.create_base_message("response", "error", "websocket_server", client_id)
            error_msg["metadata"].update({
                "status": "error",
                "error_code": error_code,
                "error_message": error_message
            })
            
            await websocket.send(json.dumps(error_msg, ensure_ascii=False))
            
        except Exception as e:
            self.get_logger().error(f'❌ 发送错误消息错误: {e}')
    
    def check_connections_health(self):
        """检查连接健康状态"""
        try:
            if not hasattr(self, 'server_loop') or self.server_loop is None:
                self.get_logger().warning("⚠️ server_loop 未初始化，跳过健康检查")
                return
            current_time = time.time()
            inactive_threshold = 120.0  # 2分钟无活动视为不活跃
            
            inactive_clients = []
            with self.client_lock:
                for client_id, session in self.client_sessions.items():
                    if current_time - session['last_activity'] > inactive_threshold:
                        inactive_clients.append(client_id)
            
            if inactive_clients:
                self.get_logger().info(f'🩺 发现 {len(inactive_clients)} 个不活跃客户端')

                
        except Exception as e:
            self.get_logger().error(f'❌ 检查连接健康状态错误: {e}')
    
    def report_server_status(self):
        """报告服务器状态"""
        try:
            active_clients = len(self.connected_clients)
            waypoint_count = len(self.business_state.get('dynamic_waypoints', {}))
            sequence_count = len(self.business_state.get('navigation_sequences', {}))
            
            status_report = {
                "active_clients": active_clients,
                "dynamic_waypoints": waypoint_count,
                "navigation_sequences": sequence_count,
                "current_navigation_mode": self.business_state.get('current_navigation_mode'),
                "server_uptime": time.time() - self.get_clock().now().seconds_nanoseconds()[0]
            }
            
            self.get_logger().info(
                f'📊 服务器状态 - 客户端: {active_clients} | '
                f'路点: {waypoint_count} | 序列: {sequence_count} | '
                f'导航模式: {self.business_state.get("current_navigation_mode") or "无"}'
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ 报告服务器状态错误: {e}')
    
    def destroy_node(self):
        """销毁节点"""
        try:
            self.get_logger().info('🛑 开始销毁WebSocket服务器节点...')
            
            # 停止所有定时器
            if hasattr(self, 'health_check_timer'):
                self.health_check_timer.cancel()
            if hasattr(self, 'status_report_timer'):
                self.status_report_timer.cancel()
            
            # 停止WebSocket服务器
            if self.server_running and self.server_loop:
                self.server_loop.call_soon_threadsafe(self.server_loop.stop)
            
            # 等待服务器线程结束
            if hasattr(self, 'server_thread') and self.server_thread.is_alive():
                self.server_thread.join(timeout=5.0)
            
            # 清理资源
            with self.client_lock:
                self.connected_clients.clear()
                self.client_sessions.clear()
                self.client_subscriptions.clear()
            
            self.get_logger().info('✅ WebSocket服务器节点销毁完成')
            
        except Exception as e:
            self.get_logger().error(f'❌ 销毁节点错误: {e}')
        finally:
            super().destroy_node()

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = None
    try:
        node = CompleteWebSocketServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('收到中断信号')
        else:
            print('节点未完全启动，收到中断信号')
            
    except Exception as e:
        if node:
            node.get_logger().error(f'节点运行错误: {e}')
        else:
            print(f'节点创建失败: {e}')
            
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()