#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket客户端节点 
功能：连接机器人本体的WebSocket服务器，获取原始数据并发布到相应的ROS话题
"""

import asyncio
import websockets
import json
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Joy
import time
from enum import Enum
import threading

class RobotState(Enum):
    """机器人状态枚举，用于跟踪当前控制模式"""
    UNKNOWN = "Unknown"         # 未知模式
    ZERO_TORQUE = "ZeroTorque"  # 零力矩模式
    DAMPING = "Damped"          # 阻尼模式
    IKSTAND = "IkStand"         # 发送准备指令后进入的原地站立模式
    WALK = "Walk"               # 行走模式
    MENU = "Menu"               # 动作库模式

class HumanoidWebSocketClient(Node):
    """从机器人本体获取数据并发布到ROS"""
    
    def __init__(self):
        super().__init__('websocket_client')
        
        # 声明参数：机器人本体的WebSocket服务器地址
        self.declare_parameter('robot_ws_server', 'ws://10.192.1.2:5000')
        self.declare_parameter('reconnect_interval', 5.0)
        self.robot_ws_server = self.get_parameter('robot_ws_server').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        
        # 机器人状态管理
        self.robot_state = RobotState.UNKNOWN  # 初始状态
        self.is_executing_motion = False       # 标记上半身是否正在做动作
        self.response_events = {}  # 用于等待命令响应：guid -> Event
        self.current_motion_event = None
        self.current_motion_name = None
        self.motion_completion_events = {}  # 用于等待动作完成通知
        self.accid = "HU_D04_01_233"  # 机器人序列号，需根据实际修改

        # 设置数据发布器 - 将不同数据发布到不同话题
        self.setup_publishers()
        
        # 连接状态
        self.connected = False
        self.last_connection_time = 0
        self.current_velocity = Twist()
        #测试用
        self.target_velocity = Twist()          # ROS2 回调写入的目标速度（实时变化）
        self.walk_command_timer = None         # 定时器线程对象
        self.walk_command_timer_running = False  # 控制定时器启停的 flag
        #测试用

        # 启动WebSocket客户端
        self.start_client()
        
        self.get_logger().info(f'WebSocket客户端启动，连接服务器: {self.robot_ws_server}')
    
    def setup_publishers(self):
        """设置ROS话题发布器"""
        # 发布原始机器人状态数据（电量、信号等）供数据整合节点订阅
        self.robot_status_publisher = self.create_publisher(
            String, '/robot_status_raw', 10
        )
         # 👇 新增：用于发布遥控器数据的话题
        self.joy_publisher = self.create_publisher(
            Joy, '/joy_raw', 10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',           # 由 teleop_twist_keyboard 发布
            self.cmd_vel_callback,
            10
        )
        
        self.cmd_sub = self.create_subscription(
            String, '/app/robot_control', self.robot_control_callback, 10
        )
        self.get_logger().info('ROS组件初始化完成')
    
    async def wait_for_state(self, target_state, timeout=10.0):
        """
        闭环等待函数：持续检查状态直到匹配目标或超时
        :param target_state: 期望达到的 RobotState 枚举值
        :param timeout: 最大等待时间（秒）
        :return: True-成功切换, False-超时失败
        """
        start_time = time.time()
        self.get_logger().info(f"⏳ 正在等待机器人进入状态: {target_state.value}...")
        
        while (time.time() - start_time) < timeout:
            # 实时检查被 notify_robot_info 更新的变量
            if self.robot_state == target_state:
                self.get_logger().info(f"✅ 状态闭环确认：已进入 {target_state.value}")
                return True
            
            # 每隔 100ms 检查一次，不阻塞协程
            await asyncio.sleep(0.1)
            
        self.get_logger().error(f"❌ 状态切换超时：未能进入 {target_state.value} (当前状态: {self.robot_state.value})")
        return False
    
    async def send_command(self, title, data, timeout=5.0):
        """
        发送命令到机器人并等待响应
        参数：
            title: 命令标题，如"request_prepare"
            data: 命令数据
            timeout: 超时时间（秒）
        返回：响应数据或None
        """
        try:
            guid = self.generate_guid()
            message = {
                "accid": self.accid,
                "title": title,
                "timestamp": int(time.time() * 1000),
                "guid": guid,
                "data": data
            }
            
            # 创建事件用于等待响应
            event = asyncio.Event()
            self.response_events[guid] = {"event": event, "response": None}
            
            # 发送命令
            if hasattr(self, 'websocket') and self.websocket:
                await self.websocket.send(json.dumps(message))
                self.get_logger().debug(f'已发送命令: {title}')
                
                # 等待响应（带超时）
                try:
                    await asyncio.wait_for(event.wait(), timeout=timeout)
                    response_data = self.response_events[guid]["response"]
                    return response_data
                except asyncio.TimeoutError:
                    self.get_logger().error(f'命令 {title} 响应超时')
                    return None
                finally:
                    # 清理事件
                    if guid in self.response_events:
                        del self.response_events[guid]
            else:
                self.get_logger().error('WebSocket未连接，无法发送命令')
                return None
                
        except Exception as e:
            self.get_logger().error(f'发送命令 {title} 错误: {e}')
            return None
    
    def generate_guid(self):
        """生成唯一标识符"""
        import uuid
        return str(uuid.uuid4())
    
    #测试用
    async def send_walk_vel_command(self, x=0.0, y=0.0, yaw=0.0):
        """发送行走指令：现支持在 Walk 和 Menu 状态下直接调用"""

        #正在做动作时绝不下发速度指令
        if self.is_executing_motion:
            self.get_logger().debug("上半身正在执行动作，已在 ROS 端主动拦截速度指令", throttle_duration_sec=1.0)
            return
            
        # 如果不是这两种状态，不允许走动
        if self.robot_state not in [RobotState.WALK, RobotState.MENU]:
            return
            
        # 安全限幅（协议要求）
        safe_x = max(-1.0, min(float(x), 1.0))
        safe_y = max(-1.0, min(float(y), 1.0))
        safe_yaw = max(-1.0, min(float(yaw), 1.0))

        data = {
            "x": round(safe_x, 3),
            "y": round(safe_y, 3),
            "yaw": round(safe_yaw, 3)
        }

        # 发送命令（异步）
        response = await self.send_command("request_set_walk_vel_sync", data)
        
        #此命令执行成功不返回成功消息，指令执行失败时返回此消息。
        if response and response.get("data", {}).get("result") == "fail_motor":
            self.get_logger().debug(f"电机错误")
        if response and response.get("data", {}).get("result") == "fail_imu":
            self.get_logger().warn(f" IMU 错误")
        if response and response.get("data", {}).get("result") == "fail_invalid_cmd":
            self.get_logger().warn(f" 参数错误 ")
        if response and response.get("data", {}).get("result") == "fail_invalid_mode":
            self.get_logger().warn(f" 当前状态不允许执行 ")
        if response and response.get("data", {}).get("result") == "fail_timeout":
            self.get_logger().warn(f" 切换状态超时 ")
    
    def cmd_vel_callback(self, msg: Twist):
        """
        键盘速度回调：仅更新本地缓存，负责启动线程。
        关闭线程的动作交给 walk_command_loop 自身优雅处理。
        """
        # 1. 更新目标速度缓存（供后台线程读取）
        self.target_velocity = msg

        # 2. 判断是否有速度输入
        has_speed = (abs(msg.linear.x) > 1e-3 or
                     abs(msg.linear.y) > 1e-3 or
                     abs(msg.angular.z) > 1e-3)

        # 3. 如果有速度，且线程没开，则启动定时器线程
        if has_speed and not self.walk_command_timer_running:
            self.walk_command_timer_running = True
            self.walk_command_timer = threading.Thread(
                target=self.walk_command_loop,
                name="walk_command_timer",
                daemon=True
            )
            self.walk_command_timer.start()
            self.get_logger().info("🔄 检测到速度指令，已启动行走控制循环")
         
    def walk_command_loop(self):
        """
        后台定时器线程：由于最新固件支持在 Walk/Menu 模式直接行走，这里仅做纯粹的指令通传
        """
        frequency = 50.0  # 设置为 50Hz (响应更快)
        interval = 1.0 / frequency
        self.get_logger().info("🔄 行走定时器启动，通过 /cmd_vel 监听动作信号...")

        import time as _time
        next_time = time.perf_counter() + interval

        while rclpy.ok() and self.walk_command_timer_running:
            tv = self.target_velocity
            has_input = (abs(tv.linear.x) > 1e-3 or abs(tv.linear.y) > 1e-3 or abs(tv.angular.z) > 1e-3)

            try:
                if has_input:
                    if self.robot_state not in [RobotState.WALK, RobotState.MENU]:
                        self.get_logger().warn( f"⚠️ 拦截指令: 当前处于 {self.robot_state.value} 状态，无法行走！请先用遥控器切入 Walk 模式。",
                            throttle_duration_sec=2.0)  # 限制每2秒最多打印一次，防止刷屏
                        continue   
                    else:
                        # 投递到 ws_loop 执行，而不是新建 loop
                        future = asyncio.run_coroutine_threadsafe(
                            self.send_walk_vel_command(tv.linear.x, tv.linear.y, tv.angular.z),
                            self.ws_loop
                        )
                        # 给一个合理的超时，不要无限等
                        future.result(timeout=0.5)
                else:
                    # 松开按键：发送 0 速度停车
                    if self.robot_state in [RobotState.WALK, RobotState.MENU]:
                        future = asyncio.run_coroutine_threadsafe(
                            self.send_walk_vel_command(0.0, 0.0, 0.0),
                            self.ws_loop
                        )
                        future.result(timeout=0.5)
                    
                    self.get_logger().info("⏹️ 速度归零，机器底盘就地待命。控制线程即将休眠。")
                    self.walk_command_timer_running = False # 安全退出本线程

            except Exception as e:
                self.get_logger().error(f"WalkLoop 获取控制端参数异常: {e}")

            current_time = time.perf_counter()
            sleep_time = next_time - current_time
            
            if sleep_time > 0:
                # 如果当前任务执行得很快（比如只花了 2ms），我们就睡够剩下的 18ms
                time.sleep(sleep_time)
            else:
                # ⚠️ 关键防爆雷：如果在极端情况下，网络卡了，发指令花了 30ms（超过了20ms周期）
                # 不要补偿！丢弃过去的帧，以当前时间为基准重新计算下一帧，防止"连发堆积"导致机器人抽搐
                self.get_logger().debug(f"⚠️ 网络或处理延时导致丢帧 (超时 {-sleep_time*1000:.1f}ms)", throttle_duration_sec=2.0)
                next_time = current_time 
            
            # 更新下一个周期的目标时间
            next_time += interval
    

    def robot_control_callback(self, msg: String):
        """处理 APP 经过 websocket_server 转发来的控制指令"""
        try:
            cmd_data = json.loads(msg.data)
            action_type = cmd_data.get("command_type", "")
            params = cmd_data.get("parameters", {})

            # 根据你《功能指令库》里的定义，动作执行指令一般叫 "execute_gesture"
            if action_type == "execute_gesture":
                # 获取 APP 传过来的 gesture_id (即 "wave_greet_bye")
                motion_name = params.get("gesture_id", "") 
                
                # 关于 loop：我们这里根本不去 get("loop")，就相当于自动忽略了
                
                if motion_name:
                    self.get_logger().info(f"📥 客户端收到执行动作指令: {motion_name}")
                    # 使用线程执行异步动作序列，防止阻塞 ROS 回调
                    threading.Thread(
                        target=self._run_motion_task, 
                        args=(motion_name,), 
                        daemon=True
                    ).start()
                else:
                    self.get_logger().warn("⚠️ 收到动作指令，但未包含 gesture_id")

        except Exception as e:
            self.get_logger().error(f'❌ 解析机器人控制指令出错: {e}')
    
    def _run_motion_task(self, motion_name):
        """在 ws_loop 上执行上半身动作序列（线程安全投递）"""
        future = asyncio.run_coroutine_threadsafe(
            self.execute_upper_body_motion(motion_name),
            self.ws_loop
        )
        try:
            future.result(timeout=30.0)  # 动作最长等 30 秒
        except Exception as e:
            self.get_logger().error(f"❌ 动作执行异常: {e}")
            self.is_executing_motion = False  # 异常时也要释放锁

    
    async def execute_upper_body_motion(self, motion_name: str):
        """执行上半身动作：切 Menu -> 发动作 -> 等待完成通知 -> 切 Walk"""
        self.get_logger().info(f"������ 准备开始执行动作: {motion_name}")

        try:
            if self.robot_state != RobotState.MENU:
                self.get_logger().info("➡️ 切换至动作库模式 (Mode: 1)...")
                res_mode1 = await self.send_command("request_set_motion_engine", {"mode": 1})
                self.get_logger().info(f"������ 切动作库模式响应: {res_mode1}")

                if not res_mode1 or res_mode1.get("data", {}).get("result") != "success":
                    self.get_logger().error("❌ 切换动作库模式失败！")
                    return

                ok = await self.wait_for_state(RobotState.MENU, timeout=3.0)
                if not ok:
                    self.get_logger().error("❌ 未成功进入 Menu 模式，取消动作执行")
                    return

                # 等待引擎稳定
                self.get_logger().info("⏳ Menu状态已到达，等待动作引擎稳定...")
                await asyncio.sleep(1.5)
            else:
                self.get_logger().info("ℹ️ 当前已处于 Menu 模式，跳过模式切换")
                await asyncio.sleep(0.5)

            self.is_executing_motion = True

            # 2. 注册当前动作等待事件
            completion_event = asyncio.Event()
            self.current_motion_event = completion_event
            self.current_motion_name = motion_name

            # 3. 下发动作
            self.get_logger().info(f"➡️ 下发动作指令: {motion_name}...")
            res_exec = await self.send_command("request_execute_atomic_motion",{"motion_name": motion_name})
            self.get_logger().info(f"������ 动作执行命令响应: {res_exec}")

            if not res_exec:
                self.get_logger().error(f"❌ 动作 {motion_name} 下发失败：未收到响应")
                return

            exec_result = res_exec.get("data", {}).get("result", "")

            if exec_result == "fail_invalid_mode":
                self.get_logger().warn("⚠️ 动作引擎尚未稳定，1秒后自动重试一次...")
                await asyncio.sleep(1.0)

                res_exec = await self.send_command("request_execute_atomic_motion",{"motion_name": motion_name})
                self.get_logger().info(f"������ 动作执行重试响应: {res_exec}")
                
                if not res_exec:
                    self.get_logger().error(f"❌ 动作 {motion_name} 重试失败：未收到响应")
                    return

                exec_result = res_exec.get("data", {}).get("result", "")

            if exec_result != "success":
                self.get_logger().error(f"❌ 动作 {motion_name} 下发失败，result={exec_result}, 完整响应={res_exec}")
                return

            self.get_logger().info("✅ 已下发动作，正在等待机器人完成通知...")

            try:
                await asyncio.wait_for(completion_event.wait(), timeout=20.0)
                self.get_logger().info(f"✅ 动作 {motion_name} 执行完成！")
            except asyncio.TimeoutError:
                self.get_logger().error(f"❌ 动作 {motion_name} 执行超时！")

        finally:
            self.is_executing_motion = False
            self.current_motion_event = None
            self.current_motion_name = None

            # ✅ 给底层一点收尾时间
            await asyncio.sleep(0.5)

            if self.robot_state != RobotState.WALK:
                self.get_logger().info("➡️ 退出动作模式，切回行走模式 (Mode: 0)...")
                res_mode0 = await self.send_command("request_set_motion_engine", {"mode": 0})
                self.get_logger().info(f"切回行走模式响应: {res_mode0}")

                if res_mode0 and res_mode0.get("data", {}).get("result") == "success":
                    await self.wait_for_state(RobotState.WALK, timeout=3.0)
                    self.get_logger().info("✅ 已安全切回行走控制模式。")
                else:
                    self.get_logger().warn("⚠️ 切回行走模式失败，请注意机器人状态。")
    
    def start_client(self):
        """启动WebSocket客户端线程（异步）"""
        self.client_thread = threading.Thread(target=self.run_client, daemon=True)
        self.client_thread.start()
    
    async def connect_to_robot(self):
        """连接到机器人本体的WebSocket服务器"""
        try:
            async with websockets.connect(self.robot_ws_server) as websocket:
                self.websocket = websocket
                self.connected = True
                self.robot_state = RobotState.UNKNOWN  # 重置状态
                self.last_connection_time = time.time()
                self.get_logger().info('已连接到机器人本体WebSocket服务器')

                #连接成功后，尝试进入准备模式
                #asyncio.create_task(self.auto_startup_sequence())
                
                # 持续接收数据
                async for message in websocket:
                    await self.handle_robot_message(message)
                    
        except Exception as e:
            self.get_logger().error(f'连接机器人服务器失败: {e}')
            self.connected = False
            self.websocket = None
    
    async def auto_startup_sequence(self):
        """连接成功后的自动启动序列（后台非阻塞运行）"""
        self.get_logger().info("⏳ 正在等待底层同步初始物理状态...")
        
        # 1. 稍微等一等，让 async for 循环有时间接收第一条 notify_robot_info 消息
        for _ in range(20):
            if self.robot_state != RobotState.UNKNOWN:
                break
            await asyncio.sleep(0.1)
            
        self.get_logger().info(f"🤖 读取到初始状态为: {self.robot_state.value}，开始自动状态校验...")
        
        # 2. 调用真正的拉起逻辑（如果已经是 Walk 则直接忽略，如果是 ZeroTorque 则自动起立）
        #await self.ensure_safe_startup()

    async def handle_robot_message(self, message):
        """处理从机器人接收到的原始数据消息"""
        try:
            data = json.loads(message)
            # self.get_logger().info(f"收到原始数据: {data}")
            data_type = data.get("title", "")
            message_data = data.get("data", {})
            guid = data.get("guid", "")

            # 如果这个消息是某个命令的响应，激活对应的 Event
            if guid in self.response_events:
                self.get_logger().debug(f"收到指令响应，解析 GUID: {guid}")
                self.response_events[guid]["response"] = data
                self.response_events[guid]["event"].set() 

            # 根据数据类型发布到不同的ROS话题
            if data_type == "response_prepare":
                self.get_logger().info('收到准备状态响应')
            elif data_type == "notify_robot_info":   
                 await self.publish_notify_robot_info(message_data) 
            elif data_type == "notify_joy_data":
                await self.publish_notify_joy_data(message_data)
            elif data_type == "notify_execute_atomic_motion":
                await self.handle_notify_execute_motion(message_data)
            elif data_type == "response_set_motion_engine":
                self.get_logger().info(f"收到动作引擎切换响应: {data}")
            elif data_type == "response_execute_atomic_motion":
                self.get_logger().info(f"收到动作执行响应: {data}")
            elif data_type == "response_set_walk_vel_sync":
                pass
            else:
                self.get_logger().warn(f'收到未知数据类型: {data_type}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON解析错误: {e}')
        except Exception as e:
            self.get_logger().error(f'处理机器人消息错误: {e}')

    async def publish_robot_status(self, status_data):
        """发布机器人状态数据到ROS话题"""
        try:
            # 构建机器人状态消息
            robot_status = {
                "battery": status_data.get("battery", 0),
                "signal_strength": status_data.get("signal", 0),
                "robot_state": status_data.get("state", "unknown"),
                "errors": status_data.get("errors", []),
                "temperature": status_data.get("temperature", 0),
                "timestamp": time.time()
            }
            
            # 发布到ROS话题
            msg = String()
            msg.data = json.dumps(robot_status)
            self.robot_status_publisher.publish(msg)
            
            # 限流日志
            self.get_logger().debug('已发布机器人状态数据', throttle_duration_sec=2.0)
            
        except Exception as e:
            self.get_logger().error(f'发布机器人状态错误: {e}')
    
    async def handle_notify_execute_motion(self, message_data):
        """解析并处理动作执行完毕的底层推送通知"""
        try:
            result = message_data.get("result", "")
            
            self.get_logger().info(f"收到动作底层通知 -> 当前动作: {self.current_motion_name}, 结果: {result}")
            
            # 如果这个动作恰好是我们在 execute_upper_body_motion 中等待的事件
            if self.current_motion_event is not None:
                if result == "success":
                  self.get_logger().info(f"✅ 动作 {self.current_motion_name} 顺利完成，正在释放等待锁...")
                else:
                  self.get_logger().warning(f"⚠️ 动作 {self.current_motion_name} 底层反馈异常: {result}，强制放行")
                
                # 无论成功失败，都必须释放 event 锁，让线程继续往下走（切回 Walk 模式）
                self.current_motion_event.set()
                
        except Exception as e:
            self.get_logger().error(f'❌ 解析/处理动作完成通知失败: {e}')

    async def publish_notify_robot_info(self, info_raw):
        """解析嵌套的 notify_robot_info (data -> result -> values)"""
        try:

            # ========== 1. 基于消息间隔估算通信质量 ==========
            current_time = time.time()
        
            # 初始化
            if not hasattr(self, '_last_msg_time'):
               self._last_msg_time = current_time
               self._msg_intervals = []
        
            # 计算本次消息与上次的间隔
            interval_ms = (current_time - self._last_msg_time) * 1000
            self._last_msg_time = current_time
        
            # 收集最近 20 个间隔样本，计算平均值
            self._msg_intervals.append(interval_ms)
            if len(self._msg_intervals) > 20:
               self._msg_intervals.pop(0)
        
            # 计算平均间隔和抖动
            if len(self._msg_intervals) >= 3:
               avg_interval = sum(self._msg_intervals) / len(self._msg_intervals)
            # 抖动 = 间隔的标准差
               variance = sum((x - avg_interval) ** 2 for x in self._msg_intervals) / len(self._msg_intervals)
               jitter = variance ** 0.5
            else:
               avg_interval = interval_ms
               jitter = 0
        
            # ★ 估算"延迟"：用抖动 + 超出正常间隔的部分
            # 假设正常推送频率是 10Hz (100ms)，超出部分视为延迟
            expected_interval = 500.0  # 根据机器人实际推送频率调整
            latency = max(0, avg_interval - expected_interval) + jitter
        
             # 限制范围
            latency = min(latency, 5000)
        
            self.get_logger().debug(
                f"通信质量: 间隔={interval_ms:.0f}ms, 平均={avg_interval:.0f}ms, 抖动={jitter:.0f}ms, 估算延迟={latency:.0f}ms",
                throttle_duration_sec=5.0
            )

            # ========== 2. 解析数据 ==========
            # 这里的 info_raw 是 data 字段的内容
            results = info_raw.get("result", [])
            
            all_parsed_values = {}
            for component in results:
                comp_values = component.get("values", [])
                for item in comp_values:
                    key = item.get("key", "")
                    val = item.get("value", "")
                    all_parsed_values[key] = val # 存储原始数据
                    if key == "robot_status":
                        try:
                            # 尝试匹配枚举，如果匹配不上则设为 UNKNOWN
                            new_state = RobotState(val)
                            if self.robot_state != new_state:
                                self.get_logger().info(f"🔄 机器人状态变更: {self.robot_state.value} -> {new_state.value}")
                                self.robot_state = new_state
                        except ValueError:
                            pass

            component_status = {}

            # 遍历 result 数组中的每一个组件 (peripheral, system_info 等)
            for component in results:
                comp_name = component.get("name", "unknown")
                component_status[comp_name] = component.get("message", "")
                
                # 提取该组件下的 values 键值对
                comp_values = component.get("values", [])
                for item in comp_values:
                    key = item.get("key", "")
                    val = item.get("value", "")
                    
                    # 进行基本的单位转换（转为标准物理单位）
                    if key.endswith("_vol") or key == "bat_vol":
                        all_parsed_values[key] = float(val) / 1000.0 # mV -> V
                    elif key.endswith("_cur") or key == "bat_cur":
                        all_parsed_values[key] = float(val) / 1000.0 # mA -> A
                    elif key.startswith("bat_temp"):
                        all_parsed_values[key] = float(val) / 10.0   # 0.1°C -> °C
                    elif val.isdigit():
                        all_parsed_values[key] = int(val)
                    else:
                        all_parsed_values[key] = val

            # 封装并发布原始数据
            payload = {
                "values": all_parsed_values,
                "health": component_status,
                "latency": round(latency, 1),
                "jitter": round(jitter, 1),           
                "msg_interval": round(avg_interval, 1), 
                "timestamp": current_time
            }
            
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.robot_status_publisher.publish(msg)
            
            #self.get_logger().info(f"✅ 成功从 {len(results)} 个模块解析了 {len(all_parsed_values)} 项指标")

        except Exception as e:
            self.get_logger().error(f"❌ 解析嵌套消息失败: {e}")
    async def publish_notify_joy_data(self, joy_data):
        """解析并发布 notify_joy_data 消息到 /joy_raw 话题"""
        try:
            # 提取 axes 和 buttons
            axes = joy_data.get("axes", [])
            buttons = joy_data.get("buttons", [])

            # 构建 Joy 消息
            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.header.frame_id = "joy_controller"  # 可自定义
            joy_msg.axes = [float(x) for x in axes]      # 确保是 float
            joy_msg.buttons = [int(x) for x in buttons]  # 确保是 int

            # 发布
            self.joy_publisher.publish(joy_msg)
            self.get_logger().debug(f'✅ 已发布遥控器数据: axes={len(axes)}, buttons={len(buttons)}')

        except Exception as e:
            self.get_logger().error(f'❌ 解析/发布遥控器数据失败: {e}')

    def run_client(self):
        """运行WebSocket客户端主循环"""
        self.ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)
        self.ws_loop.run_until_complete(self._client_loop())
    
    async def _client_loop(self):
        """异步客户端主循环，保证 ws_loop 始终在运行"""
        while rclpy.ok():
            try:
                if not self.connected:
                    self.get_logger().info('尝试连接机器人服务器...')
                    await self.connect_to_robot()
                else:
                    # 保持连接，定期发送心跳
                    await asyncio.sleep(0.1)
                    
            except Exception as e:
                self.get_logger().error(f'WebSocket客户端运行错误: {e}')
                self.connected = False
                self.robot_state = RobotState.UNKNOWN  
            
            # 连接断开时等待重连
            if not self.connected:
                await asyncio.sleep(self.reconnect_interval)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        client_node = HumanoidWebSocketClient()
        rclpy.spin(client_node)
    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()