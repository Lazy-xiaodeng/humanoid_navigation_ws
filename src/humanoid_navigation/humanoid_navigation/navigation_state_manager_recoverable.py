#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航状态管理器 - 负责导航执行和状态管理
功能：执行实际导航，监控导航状态，统一发布状态信息
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog
import rclpy.duration
from tf2_ros import Buffer, TransformException, TransformListener

import json
import time
import math
from enum import Enum
from typing import Dict, List, Optional, Any

# ========== 枚举定义 ==========
class NavigationState(Enum):
    """导航状态枚举"""
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing" 
    PAUSED = "paused"
    REACHED_WAYPOINT = "reached_waypoint"
    COMPLETED = "completed"
    FAILED = "failed"
    RECOVERABLE_FAILED = "recoverable_failed"
    CANCELLED = "cancelled"

class NavigationMode(Enum):
    """导航模式枚举"""
    SINGLE_POINT = "single_point"
    MULTI_POINT = "multi_point"
    EXHIBITION_TOUR = "exhibition_tour"
    CHARGING_ROUTE = "charging_route"

class NavigationStateManager(Node):
    """
    导航状态管理器节点 - 负责导航执行和状态管理
    """
    
    def __init__(self):
        super().__init__('navigation_state_manager')
        
        # ========== 参数声明 ==========
        self.declare_parameters(namespace='', parameters=[
            ('position_tolerance', 0.15),
            ('orientation_tolerance', 0.3),
            ('waypoint_timeout', 300.0),
            ('status_publish_rate', 2.0),
            ('default_frame_id', 'map'),
            ('obstacle_block_timeout', 5.0),  # 障碍物阻塞超时时间（秒）
            ('velocity_threshold', 0.10),  # 判断机器人是否停滞的速度阈值（m/s）
            ('require_walk_mode_for_navigation', True),
            ('robot_status_timeout', 2.0),
            ('pending_navigation_timeout', 90.0),
            ('obstacle_block_near_goal_distance', 0.7),
            ('navigation_failure_policy', 'pause_on_failed'),
            ('auto_pause_on_localization_recovery', True),
            ('localization_recovery_status_topic', '/localization/recovery_status'),
            ('map_frame', 'map'),
            ('base_frame', 'base_footprint'),
            ('pose_tf_timeout_sec', 0.05)
        ])

        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value
        self.status_publish_rate = self.get_parameter('status_publish_rate').value
        self.default_frame_id = self.get_parameter('default_frame_id').value
        self.obstacle_block_timeout = self.get_parameter('obstacle_block_timeout').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.require_walk_mode_for_navigation = self.get_parameter('require_walk_mode_for_navigation').value
        self.robot_status_timeout = float(self.get_parameter('robot_status_timeout').value)
        self.pending_navigation_timeout = float(self.get_parameter('pending_navigation_timeout').value)
        self.obstacle_block_near_goal_distance = float(self.get_parameter('obstacle_block_near_goal_distance').value)
        self.navigation_failure_policy = str(self.get_parameter('navigation_failure_policy').value)
        self.auto_pause_on_localization_recovery = bool(
            self.get_parameter('auto_pause_on_localization_recovery').value)
        self.localization_recovery_status_topic = str(
            self.get_parameter('localization_recovery_status_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.pose_tf_timeout_sec = float(self.get_parameter('pose_tf_timeout_sec').value)
        
        # ========== 导航状态 ==========
        self.current_state = NavigationState.IDLE
        self.current_detailed_state = "IDLE"
        self.current_navigation_mode = None
        self.current_sequence_id = None
        self.current_waypoint_index = 0
        self.total_waypoints = 0
        self.current_waypoint = None
        self.waypoint_ids = []
        self.skipped_waypoints = []
        self.last_failure_context = {}
        self.waypoint_arrived_by_position = False
        self.pause_time = 0
        self.pause_duration_limit = 0
        
        # ========== 机器人状态 ==========
        self.current_pose = None
        self.current_pose_frame = self.map_frame
        self.current_velocity = None
        self.pose_derived_speed = None
        self.last_motion_pose = None
        self.last_motion_pose_time = None
        self.last_pose_update = 0
        self.robot_control_state = "Unknown"
        self.robot_motion_busy = False
        self.robot_current_motion = ""
        self.robot_ready_for_navigation = False
        self.last_robot_status_update = 0.0
        self.pending_navigation_request = None
        self.pending_navigation_created_at = 0.0
        self.pending_navigation_reason = ""
        self.nav2_blockage_suppression_nodes = set()
        self.distance_remaining = float('inf')
        self.estimated_time_remaining = 0.0

        # ========== 障碍物阻塞检测状态 ==========
        self.is_blocked_by_obstacle = False
        self.block_start_time = None
        self.block_reported = False  # 防止重复上报
        
        # ========== 从路点管理器接收的数据 ==========
        self.waypoints_data = {}
        self.navigation_start_time = 0
        self.current_goal_pose = None
        
        # ========== Nav2动作客户端 ==========
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ========== 定位恢复联动状态 ==========
        self.localization_recovery_active = False
        self.localization_auto_paused = False
        self.localization_resume_pending = False
        self.localization_recovery_reason = ""
        self.localization_recovery_started_at = 0.0
        self.localization_recovery_last_status = {}
        
        # ========== 设置ROS2通信 ==========
        self.setup_communication()
        
        # ========== 启动定时器 ==========
        self.setup_timers()
        
        self.get_logger().info("导航状态管理器启动完成 - 负责导航执行和状态管理")
        self.waypoint_arrived_locked = False
        self.get_logger().info(f"位置容差: {self.position_tolerance}m, 方向容差: {self.orientation_tolerance}rad")
    
    def setup_communication(self):
        """设置ROS2通信接口"""
        # ========== 发布器 ==========
        # 统一发布导航状态（这是主要的输出接口）
        self.navigation_status_pub = self.create_publisher(String, '/navigation/status', 10)
        
        # 发布确认消息给路点管理器
        self.navigation_ack_pub = self.create_publisher(String, '/navigation/acknowledgments', 10)
        
        # 发布当前目标给Nav2
        self.navigation_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 发布导航路径（用于可视化）
        self.navigation_path_pub = self.create_publisher(Path, '/navigation/current_path', 10)

        # 定位异常自动暂停时，立即补一帧零速度，避免取消 Nav2 goal 前后继续沿旧速度滑行
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ========== 订阅器 ==========
        # 订阅路点管理器的导航请求
        self.navigation_request_sub = self.create_subscription(
            String, '/navigation/requests', self.navigation_request_callback, 10
        )
        
        # 订阅路点管理器的点位数据
        self.waypoints_data_sub = self.create_subscription(
            String, '/navigation/waypoints_data', self.waypoints_data_callback, 10
        )
        
        # 订阅里程计数据
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.robot_status_sub = self.create_subscription(
            String, '/robot_status_raw', self.robot_status_callback, 10
        )
        
        self.nav2_behavior_log_sub = self.create_subscription(BehaviorTreeLog,'/behavior_tree_log',self.nav2_log_callback,10)

        self.localization_recovery_sub = self.create_subscription(
            String,
            self.localization_recovery_status_topic,
            self.localization_recovery_status_callback,
            10
        )
    
    def setup_timers(self):
        """设置定时器"""
        # 状态发布定时器
        self.create_timer(1.0 / self.status_publish_rate, self.publish_navigation_status)
        
        # 导航检查定时器
        self.create_timer(0.5, self.check_navigation_status)  # 2Hz
        
        # 超时检查定时器
        self.create_timer(5.0, self.check_timeout)  # 每5秒检查一次超时

        # 待启动导航检查定时器
        self.create_timer(0.2, self.try_execute_pending_navigation)

        # 定位恢复后自动继续未完成导航
        self.create_timer(0.5, self.try_resume_after_localization_recovery)
    
    def navigation_request_callback(self, msg: String):
        """处理路点管理器的导航请求"""
        try:
            request_data = json.loads(msg.data)
            request_type = request_data.get("request_type", "")
            
            self.get_logger().info(f"收到导航请求: {request_type}")
            
            if request_type == "navigation_command":
                self.handle_navigation_command(request_data)
            else:
                self.get_logger().warning(f"未知的请求类型: {request_type}")
                self.send_acknowledgment("error", f"未知请求类型: {request_type}")
                
        except Exception as e:
            self.get_logger().error(f"处理导航请求错误: {e}")
            self.send_acknowledgment("error", f"处理请求失败: {str(e)}")
    
    def waypoints_data_callback(self, msg: String):
        """处理统一格式的路点数据"""
        try:
            message_data = json.loads(msg.data)
        
        # 检测消息格式
            if "protocol_version" in message_data:  # 统一格式
                data_type = message_data.get("data_type", "")
                if data_type == "waypoints_data":
                    legacy_data = message_data.get("data", {})
                else:
                    return  # 不是路点数据，忽略
            else:  # 传统格式
                legacy_data = message_data
            
            # 提取路点数据
            waypoints_data = legacy_data.get("data", {})
            self.waypoints_data = waypoints_data.get("waypoints", {})
        
            self.get_logger().info(
                f'收到路点数据更新，共 {self.count_cached_waypoints()} 个点位'
            )
        
        except Exception as e:
            self.get_logger().error(f'❌❌ 处理路点数据错误: {e}')

    def count_cached_waypoints(self) -> int:
        """统计缓存中的真实点位数，兼容扁平和按类型嵌套两种结构。"""
        count = 0
        for key, value in self.waypoints_data.items():
            if isinstance(value, dict) and value.get("id") == key:
                count += 1
            elif isinstance(value, dict):
                count += sum(1 for item in value.values() if isinstance(item, dict))
        return count

    def merge_request_waypoints_data(self, request_data: Dict[str, Any]):
        """把导航请求里携带的点位数据并入本节点缓存，避免启动时序导致缓存为空。"""
        request_waypoints = request_data.get("waypoints_data", {})
        if not isinstance(request_waypoints, dict) or not request_waypoints:
            return

        merged_count = 0
        for waypoint_id, waypoint_data in request_waypoints.items():
            if isinstance(waypoint_data, dict):
                self.waypoints_data[waypoint_id] = waypoint_data
                merged_count += 1

        if merged_count > 0:
            self.get_logger().info(f"已从导航请求同步 {merged_count} 个点位到状态管理器缓存")
    
    @staticmethod
    def apply_velocity_deadzone(value: float, threshold: float = 0.01) -> float:
        return 0.0 if abs(value) < threshold else value

    def convert_fastlio_velocity_to_standard(self, msg: Odometry) -> Twist:
        """Fast-LIO速度轴为x左/y下/z后，这里转换为ROS标准机器人速度。"""
        velocity = Twist()
        velocity.linear.x = self.apply_velocity_deadzone(-float(msg.twist.twist.linear.z))
        velocity.linear.y = self.apply_velocity_deadzone(float(msg.twist.twist.linear.x))
        velocity.linear.z = self.apply_velocity_deadzone(-float(msg.twist.twist.linear.y))
        velocity.angular.x = self.apply_velocity_deadzone(-float(msg.twist.twist.angular.z))
        velocity.angular.y = self.apply_velocity_deadzone(float(msg.twist.twist.angular.x))
        velocity.angular.z = self.apply_velocity_deadzone(-float(msg.twist.twist.angular.y))
        return velocity

    def update_pose_derived_speed(self, msg: Odometry):
        """基于 Fast-LIO 位姿差分估算水平运动速度，避免 /odom.twist 未填充导致误判停滞。"""
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp <= 0.0:
            stamp = time.time()

        position = msg.pose.pose.position
        current_pose = (
            float(position.x),
            float(position.y),
            float(position.z),
        )

        if self.last_motion_pose is None or self.last_motion_pose_time is None:
            self.last_motion_pose = current_pose
            self.last_motion_pose_time = stamp
            self.pose_derived_speed = None
            return

        dt = stamp - self.last_motion_pose_time
        if dt <= 0.0 or dt > 2.0:
            self.last_motion_pose = current_pose
            self.last_motion_pose_time = stamp
            self.pose_derived_speed = None
            return

        dx = current_pose[0] - self.last_motion_pose[0]
        dz = current_pose[2] - self.last_motion_pose[2]
        # Fast-LIO 原始坐标约为 x=左、y=下、z=后，水平位移主要在 x/z 平面。
        horizontal_delta = math.sqrt(dx * dx + dz * dz)
        self.pose_derived_speed = horizontal_delta / dt
        self.last_motion_pose = current_pose
        self.last_motion_pose_time = stamp

    def get_blockage_motion_speed(self):
        """返回阻塞检测使用的运动速度及其来源。"""
        twist_speed = None
        if self.current_velocity is not None:
            linear_velocity = math.sqrt(
                self.current_velocity.linear.x**2 + self.current_velocity.linear.y**2
            )
            angular_velocity = abs(self.current_velocity.angular.z)
            twist_speed = math.sqrt(linear_velocity**2 + angular_velocity**2)

        if self.pose_derived_speed is None:
            return twist_speed, "odom_twist"

        if twist_speed is None or self.pose_derived_speed >= twist_speed:
            return self.pose_derived_speed, "pose_delta"

        return twist_speed, "odom_twist"

    def lookup_current_map_pose(self):
        """优先通过 TF 读取标准 map->base 位姿，避免直接使用 Fast-LIO 原始 /odom 坐标。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.pose_tf_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'等待标准位姿 TF {self.map_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        pose_stamped = PoseStamped()
        pose_stamped.header = transform.header
        pose_stamped.pose.position.x = transform.transform.translation.x
        pose_stamped.pose.position.y = transform.transform.translation.y
        pose_stamped.pose.position.z = transform.transform.translation.z
        pose_stamped.pose.orientation = transform.transform.rotation
        return pose_stamped.pose

    def odom_callback(self, msg: Odometry):
        """处理里程计数据回调 —— 现在同时提供位姿和速度"""
        try:
            # 更新位姿。/odom 是 Fast-LIO 非标准坐标，位姿必须走 TF 链得到 map->base_footprint。
            standard_pose = self.lookup_current_map_pose()
            if standard_pose is not None:
                self.current_pose = standard_pose
                self.current_pose_frame = self.map_frame
                self.last_pose_update = time.time()

            # 更新速度。/odom来自Fast-LIO非标准坐标系，必须先转换再用于APP状态和阻塞检测。
            self.current_velocity = self.convert_fastlio_velocity_to_standard(msg)
            self.update_pose_derived_speed(msg)

            # 障碍物阻塞检测
            self.check_obstacle_blockage()

            self.get_logger().debug(
                f'🔄 位姿 & 速度更新: pos=({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}, {msg.pose.pose.position.z:.3f}), '
                f'vel={msg.twist.twist.linear.x:.3f} m/s',
                throttle_duration_sec=2.0
            )
        except Exception as e:
            self.get_logger().error(f'❌ 处理里程计数据错误: {e}')

    def robot_status_callback(self, msg: String):
        """跟踪机器人底层控制状态，导航启动前必须确认已回到 Walk。"""
        try:
            status = json.loads(msg.data)
            values = status.get("values", {}) if isinstance(status, dict) else {}
            if not isinstance(values, dict):
                return

            robot_state = values.get("robot_status") or values.get("robot_state") or self.robot_control_state
            self.robot_control_state = str(robot_state)
            self.robot_motion_busy = self._as_bool(values.get("motion_busy", False))
            self.robot_current_motion = str(values.get("current_motion", "") or "")

            ready_value = values.get("control_ready_for_navigation")
            if ready_value is None:
                self.robot_ready_for_navigation = (
                    self.robot_control_state == "Walk" and not self.robot_motion_busy
                )
            else:
                self.robot_ready_for_navigation = self._as_bool(ready_value)

            self.last_robot_status_update = time.time()

        except Exception as e:
            self.get_logger().error(f'❌ 处理机器人控制状态错误: {e}')

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "y", "on")
        return False

    def get_navigation_start_block_reason(self) -> Optional[str]:
        if not self.require_walk_mode_for_navigation:
            return None

        now = time.time()
        if self.last_robot_status_update <= 0:
            return "尚未收到机器人底层状态，暂不启动导航"

        status_age = now - self.last_robot_status_update
        if status_age > self.robot_status_timeout:
            return f"机器人底层状态超时 {status_age:.1f}s，暂不启动导航"

        if self.robot_ready_for_navigation:
            return None

        if self.robot_motion_busy:
            motion_text = f" ({self.robot_current_motion})" if self.robot_current_motion else ""
            return f"机器人正在执行动作{motion_text}，尚未回到 Walk，暂不启动导航"

        return f"机器人当前状态为 {self.robot_control_state}，尚未回到 Walk，暂不启动导航"

    def reject_navigation_start_if_robot_not_ready(self, ack_type: str) -> bool:
        reason = self.get_navigation_start_block_reason()
        if not reason:
            return False

        self.get_logger().warning(f"拒绝启动导航: {reason}")
        self.send_acknowledgment(ack_type, "error", reason)
        self.publish_status_update("navigation_start_rejected", {"reason": reason})
        return True

    def defer_navigation_start_if_robot_not_ready(self, request_data: Dict[str, Any]) -> bool:
        """动作/Menu 状态下缓存启动导航请求，等机器人回到 Walk 后自动执行。"""
        reason = self.get_navigation_start_block_reason()
        if not reason:
            return False

        status_fresh = (
            self.last_robot_status_update > 0 and
            time.time() - self.last_robot_status_update <= self.robot_status_timeout
        )
        can_defer = status_fresh and (
            self.robot_motion_busy or self.robot_control_state == "Menu"
        )

        if not can_defer:
            self.get_logger().warning(f"拒绝启动导航: {reason}")
            self.send_acknowledgment("navigation_started", "error", reason)
            self.publish_status_update("navigation_start_rejected", {"reason": reason})
            return True

        if self.pending_navigation_request is not None:
            self.get_logger().warning("新的导航请求覆盖上一条待启动导航请求")

        self.pending_navigation_request = json.loads(json.dumps(request_data))
        self.pending_navigation_created_at = time.time()
        if self.robot_motion_busy:
            pending_message = f"{reason}。已缓存导航请求，等待动作执行完成并回到 Walk 后再开始导航。"
        else:
            pending_message = f"{reason}。已缓存导航请求，等待机器人回到 Walk 后再开始导航。"
        self.pending_navigation_reason = pending_message

        command_type = request_data.get("command_data", {}).get("command_type", "")
        self.get_logger().warning(pending_message)
        self.send_acknowledgment("navigation_pending", "pending", pending_message)
        self.publish_status_update("navigation_pending", {
            "reason": pending_message,
            "block_reason": reason,
            "command_type": command_type,
            "timeout_sec": self.pending_navigation_timeout
        })
        return True

    def try_execute_pending_navigation(self):
        """机器人重新就绪后执行缓存的导航启动请求。"""
        if self.pending_navigation_request is None:
            return

        if self.current_state != NavigationState.IDLE:
            pending_request = self.pending_navigation_request
            self.pending_navigation_request = None
            self.pending_navigation_reason = ""
            command_type = pending_request.get("command_data", {}).get("command_type", "")
            message = "已有其他导航任务启动，取消待执行导航"
            self.get_logger().warning(f"{message}: {command_type}")
            self.send_acknowledgment("navigation_pending_cancelled", "error", message)
            self.publish_status_update("navigation_pending_cancelled", {
                "reason": message,
                "command_type": command_type
            })
            return

        reason = self.get_navigation_start_block_reason()
        if reason:
            if time.time() - self.pending_navigation_created_at > self.pending_navigation_timeout:
                pending_request = self.pending_navigation_request
                self.pending_navigation_request = None
                self.pending_navigation_reason = ""
                command_type = pending_request.get("command_data", {}).get("command_type", "")
                message = f"待执行导航超时: {reason}"
                self.get_logger().warning(message)
                self.send_acknowledgment("navigation_started", "error", message)
                self.publish_status_update("navigation_pending_timeout", {
                    "reason": reason,
                    "command_type": command_type
                })
            return

        request_data = self.pending_navigation_request
        self.pending_navigation_request = None
        self.pending_navigation_reason = ""

        command_type = request_data.get("command_data", {}).get("command_type", "")
        self.get_logger().info(f"机器人已回到 Walk，执行待启动导航: {command_type}")
        self.send_acknowledgment("navigation_pending", "success", "动作执行完成，机器人已回到 Walk，开始执行待启动导航")
        self.handle_navigation_command(request_data)

    def check_obstacle_blockage(self):
        """检测机器人是否被障碍物阻塞（速度接近0且正在执行导航）"""
        # 仅在导航执行中且未报告过阻塞时检测
        if self.current_state != NavigationState.EXECUTING:
            # 如果状态不是EXECUTING，重置阻塞检测
            if self.is_blocked_by_obstacle:
                self.reset_block_detection()
            return

        # 检查运动速度是否接近0
        total_velocity, velocity_source = self.get_blockage_motion_speed()
        if total_velocity is None:
            return

        if self.block_reported:
            # 阻塞异常已经上报过；等机器人真正恢复运动后再允许下一次阻塞上报。
            if total_velocity >= self.velocity_threshold:
                self.get_logger().info(
                    f"✅ 机器人阻塞后已恢复运动，速度: {total_velocity:.4f} m/s ({velocity_source})，允许后续阻塞重新上报"
                )
                self.reset_block_detection()
            return

        suppression_reason = self.get_obstacle_blockage_suppression_reason()
        if suppression_reason:
            if self.is_blocked_by_obstacle:
                self.get_logger().info(
                    f"当前处于{suppression_reason}，重置阻塞计时，避免误报障碍物阻塞",
                    throttle_duration_sec=2.0
                )
                self.reset_block_detection()
            return

        if total_velocity < self.velocity_threshold:
            # 速度低于阈值，开始计时
            if not self.is_blocked_by_obstacle:
                self.is_blocked_by_obstacle = True
                self.block_start_time = time.time()
                self.current_detailed_state = "BLOCKED_BY_OBSTACLE"
                self.get_logger().warning(
                    f"⚠️ 检测到机器人停滞，速度: {total_velocity:.4f} m/s ({velocity_source})，开始计时阻塞..."
                )
            else:
                # 检查阻塞是否超时
                block_duration = time.time() - self.block_start_time
                if block_duration > self.obstacle_block_timeout:
                    self.handle_obstacle_block_timeout(block_duration)
        else:
            # 速度正常，重置阻塞检测
            if self.is_blocked_by_obstacle:
                self.get_logger().info(
                    f"✅ 机器人恢复运动，速度: {total_velocity:.4f} m/s ({velocity_source})，重置阻塞检测"
                )
                self.reset_block_detection()

    def get_obstacle_blockage_suppression_reason(self) -> Optional[str]:
        """返回当前是否应暂停障碍物阻塞计时，以及暂停原因。"""
        if self.robot_motion_busy:
            motion_text = f"({self.robot_current_motion})" if self.robot_current_motion else ""
            return f"机器人动作执行阶段{motion_text}"

        if self.robot_control_state == "Menu":
            return "机器人动作库模式"

        if self.nav2_blockage_suppression_nodes:
            active_nodes = ", ".join(sorted(self.nav2_blockage_suppression_nodes))
            return f"Nav2主动转向/后退阶段({active_nodes})"

        if self.obstacle_block_near_goal_distance <= 0:
            return None

        threshold = self.obstacle_block_near_goal_distance
        if math.isfinite(self.distance_remaining) and self.distance_remaining <= threshold:
            return f"接近目标点阶段(剩余路径 {self.distance_remaining:.2f}m)"

        distance_to_goal = self.calculate_distance_to_waypoint()
        if math.isfinite(distance_to_goal) and distance_to_goal <= threshold:
            return f"接近目标点阶段(直线距离 {distance_to_goal:.2f}m)"

        return None

    @staticmethod
    def is_nav2_blockage_suppression_node(node_name: str) -> bool:
        # 只在真正会主动驱动机器人运动的 Nav2 动作期间暂停阻塞计时。
        # RecoveryNode / Wait 只是行为树控制节点，不能屏蔽前方障碍物长时间阻塞上报。
        return node_name in {"SpinToPose", "Spin", "BackUp"}

    def reset_block_detection(self):
        """重置阻塞检测状态"""
        self.is_blocked_by_obstacle = False
        self.block_start_time = None
        self.block_reported = False
        if self.current_detailed_state == "BLOCKED_BY_OBSTACLE":
            self.current_detailed_state = "EXECUTING"

    def handle_obstacle_block_timeout(self, block_duration: float):
        """处理障碍物阻塞超时，上报异常给APP"""
        if self.block_reported:
            return  # 防止重复上报

        self.block_reported = True
        self.current_detailed_state = "BLOCKED_BY_OBSTACLE"

        self.get_logger().error(
            f"🚨 障碍物阻塞超时 ({block_duration:.1f}秒)，上报导航异常"
        )

        # 上报阻塞异常
        error_reason = "检测到障碍物，前方路径被挡住"
        self.publish_status_update("navigation_obstacle_blocked", {
            "reason": error_reason,
            "block_duration": round(block_duration, 1),
            "blocked_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "blocked_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "blocked_waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "position": self.current_waypoint.get("position", []) if self.current_waypoint else []
        })

        # 发送错误确认消息
        self.send_acknowledgment("navigation_obstacle_blocked", "error", error_reason)

        # 注意：这里不取消导航，机器人继续尝试，但APP已收到异常通知
        # 如果需要取消导航，可以取消注释下面的代码
        # if self.current_goal_handle:
        #     self.cancel_navigation()
    def nav2_goal_response_callback(self, future):
        """处理Nav2目标响应"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Nav2目标被拒绝")
                self.handle_navigation_failed("Nav2目标被拒绝")
                return
            
            self.current_goal_handle = goal_handle
            if self.current_state == NavigationState.PAUSED and self.localization_auto_paused:
                self.get_logger().info("Nav2目标已接受，但定位恢复暂停中，立即取消该目标")
                self.cancel_navigation()
                return

            self.get_logger().info("Nav2目标已接受")
        
            # 获取结果
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav2_result_callback)
        
        except Exception as e:
            self.get_logger().error(f"处理Nav2目标响应错误: {e}")


    def nav2_result_callback(self, future):
        """处理Nav2导航结果"""
        try:
            result = future.result()
            status = result.status
        
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.handle_nav2_succeeded()
            elif status == GoalStatus.STATUS_CANCELED:
                self.handle_nav2_cancelled()       # ← 区分取消和失败
            else:
                self.handle_nav2_failed()
            
        except Exception as e:
            self.get_logger().error(f"处理Nav2结果错误: {e}")
            self.handle_nav2_failed()
    
    def nav2_feedback_callback(self, feedback_msg):
        try:
            feedback = feedback_msg.feedback
            
            # 获取 Nav2 原生计算的剩余距离
            self.distance_remaining = float(getattr(feedback, 'distance_remaining', float('inf')))
            
            # 提取预计剩余时间（解析 ROS Duration 对象）
            est_time = getattr(feedback, 'estimated_time_remaining', None)
            if est_time and hasattr(est_time, 'sec') and hasattr(est_time, 'nanosec'):
                self.estimated_time_remaining = float(est_time.sec + est_time.nanosec * 1e-9)
            else:
                self.estimated_time_remaining = 0.0
                
            # 提取已导航时间（解析 ROS Duration 对象）
            nav_time = getattr(feedback, 'navigation_time', None)
            nav_time_sec = 0.0
            if nav_time and hasattr(nav_time, 'sec') and hasattr(nav_time, 'nanosec'):
                nav_time_sec = float(nav_time.sec + nav_time.nanosec * 1e-9)
            elif nav_time and hasattr(nav_time, 'sec'): # fallback
                nav_time_sec = float(nav_time.sec)
        
            # 实时发布进度更新事件，让上层立刻感知
            self.publish_status_update("navigation_progress_update", {
                "current_pose": {
                    "position": {
                        "x": float(feedback.current_pose.pose.position.x),
                        "y": float(feedback.current_pose.pose.position.y),
                        "z": float(feedback.current_pose.pose.position.z)
                    }
                },
                "distance_remaining": self.distance_remaining,
                "estimated_time_remaining": self.estimated_time_remaining,
                "navigation_time_sec": nav_time_sec
            })
        except Exception as e:
            # 修改为更明显的错误信息方便以后排查，从 debug 提升到 warning 或 error
            self.get_logger().warning(f"解析 Nav2 反馈或发布状态失败: {e}")

    # 优化进度百分比计算逻辑
    def calculate_progress_percentage(self) -> float:
        if self.total_waypoints == 0: return 0.0
    
        # 基础进度：已完成的路点数
        base_progress = (self.current_waypoint_index / self.total_waypoints) * 100
    
        # 精细进度：在当前路点段内的推进程度
        if self.current_state == NavigationState.EXECUTING and hasattr(self, 'distance_remaining'):
        # 假设一段路点的平均长度，或者直接基于剩余距离倒推
        # 这里的 100 / self.total_waypoints 是当前这一段的总权重
            segment_weight = 100.0 / self.total_waypoints
        
        # 更加科学的估算：如果剩余距离减小，当前段进度就增加
        # 我们可以简单地设定：当前段进度 = (1 - 剩余距离/5.0) * segment_weight (5米是一个典型的段长)
        # 更好的做法是记录本段起始距离，但如果没有，1.0m 对应 10% 也是一种平滑算法
            current_segment_progress = max(0, (1.0 - min(self.distance_remaining / 5.0, 1.0))) * segment_weight
            return round(base_progress + current_segment_progress, 1)
        
        return round(base_progress, 1)
    
    def handle_navigation_command(self, request_data: Dict[str, Any]):
        """处理导航命令"""
        try:
            command_data = request_data.get("command_data", {})
            command_type = command_data.get("command_type", "")
            
            self.get_logger().info(f"执行导航命令: {command_type}")
            
            if command_type == "start_single_navigation":
                self.handle_start_single_navigation(command_data, request_data)
            elif command_type == "start_multi_point_navigation":  # 新增多点导航
                self.handle_start_multi_point_navigation(command_data, request_data)    
            elif command_type == "start_exhibition_navigation":
                self.handle_start_exhibition_navigation(command_data, request_data)
            elif command_type == "stop_navigation":
                self.handle_stop_navigation(command_data)
            elif command_type == "pause_navigation":
                self.handle_pause_navigation(command_data)
            elif command_type == "resume_navigation":
                self.handle_resume_navigation(command_data)
            elif command_type == "retry_failed_waypoint":
                self.handle_retry_failed_waypoint(command_data)
            elif command_type == "skip_failed_waypoint":
                self.handle_skip_failed_waypoint(command_data)
            elif command_type == "abort_failed_navigation":
                self.handle_abort_failed_navigation(command_data)
            else:
                self.get_logger().warning(f"未知的导航命令: {command_type}")
                self.send_acknowledgment("error", f"未知命令: {command_type}")
                
        except Exception as e:
            self.get_logger().error(f"执行导航命令错误: {e}")
            self.send_acknowledgment("error", f"执行命令失败: {str(e)}")
    
    def handle_start_single_navigation(self, command_data: Dict[str, Any], request_data: Dict[str, Any]):
        """处理单点导航"""
        try:
            # 检查当前状态
            if self.current_state != NavigationState.IDLE:
                self.send_acknowledgment("start_single_navigation", "error", "当前正在执行其他导航任务")
                return

            if self.defer_navigation_start_if_robot_not_ready(request_data):
                return
            
            waypoint_id = command_data.get("waypoint_id")
            waypoint_data = request_data.get("waypoint_data", {})
            
            if not waypoint_data:                # 从本地数据中查找点位
                waypoint_data = self.find_waypoint_data_by_id(waypoint_id)
            
                if not waypoint_data:                
                   self.send_acknowledgment("error", f"点位 '{waypoint_id}' 不存在")
                   return
            
            # 设置导航状态
            self.current_state = NavigationState.PLANNING
            self.current_navigation_mode = NavigationMode.SINGLE_POINT
            self.current_sequence_id = f"single_{int(time.time())}"
            self.waypoint_ids = [waypoint_id]
            self.total_waypoints = 1
            self.current_waypoint_index = 0
            self.navigation_start_time = time.time()
            
            # 发送确认消息
            self.send_acknowledgment("navigation_started", "success", 
                                   f"开始单点导航到 '{waypoint_data.get('name', waypoint_id)}'")
            
            # 开始导航
            self.navigate_to_waypoint(waypoint_data)
            
        except Exception as e:
            self.get_logger().error(f"开始单点导航错误: {e}")
            self.send_acknowledgment("navigation_started", "error", f"开始导航失败: {str(e)}")
    
    def handle_start_multi_point_navigation(self, command_data: Dict[str, Any], request_data: Dict[str, Any]):
        """处理多点导航"""
        try:
             # 检查当前状态
            if self.current_state != NavigationState.IDLE:
               self.send_acknowledgment("start_multi_point_navigation", "error", "当前正在执行其他导航任务")
               return

            if self.defer_navigation_start_if_robot_not_ready(request_data):
               return

            self.merge_request_waypoints_data(request_data)
        
        # 获取点位ID列表
            waypoint_ids = command_data.get("waypoint_ids", [])
            if not waypoint_ids:
                self.send_acknowledgment("error", "点位列表不能为空")
                return
        
        # 验证所有点位是否存在
            valid_waypoints = []
            for wp_id in waypoint_ids:
                waypoint_data = self.find_waypoint_data_by_id(wp_id)
                if not waypoint_data:
                    self.send_acknowledgment("error", f"点位 '{wp_id}' 不存在")
                    return
                valid_waypoints.append(waypoint_data)
          
        # 设置导航状态
            self.current_state = NavigationState.PLANNING
            self.current_navigation_mode = NavigationMode.MULTI_POINT
            self.current_sequence_id = f"multi_{int(time.time())}"
            self.waypoint_ids = waypoint_ids
            self.total_waypoints = len(waypoint_ids)
            self.current_waypoint_index = 0
            self.navigation_start_time = time.time()
        
        # 发送确认消息
            self.send_acknowledgment("navigation_started", "success", 
                               f"开始多点导航，共 {self.total_waypoints} 个点位")
        
        # 开始导航序列
            self.start_navigation_sequence()
        
        except Exception as e:                 
            self.get_logger().error(f"开始多点导航错误: {e}")
            self.send_acknowledgment("navigation_started", "error", f"开始导航失败: {str(e)}")

    def handle_start_exhibition_navigation(self, command_data: Dict[str, Any], request_data: Dict[str, Any]):
        """处理展台导航"""
        try:
            # 检查当前状态
            if self.current_state != NavigationState.IDLE:
                self.send_acknowledgment("start_exhibition_navigation", "error", "当前正在执行其他导航任务")
                return

            if self.defer_navigation_start_if_robot_not_ready(request_data):
                return
            
            # 获取展台点位数据
            exhibition_points = request_data.get("waypoints_data", {})
            if exhibition_points:
                self.merge_request_waypoints_data(request_data)

            if not exhibition_points:
                # 从本地数据中获取展台点
                exhibition_points = self.waypoints_data.get("exhibition_point", {})
            
                if not exhibition_points:
                   self.send_acknowledgment("error", "没有可用的展台点位")
                   return
            
            # 提取点位ID列表
            waypoint_ids = list(exhibition_points.keys())
            
            # 设置导航状态
            self.current_state = NavigationState.PLANNING
            self.current_navigation_mode = NavigationMode.EXHIBITION_TOUR
            self.current_sequence_id = f"exhibition_{int(time.time())}"
            self.waypoint_ids = waypoint_ids
            self.total_waypoints = len(waypoint_ids)
            self.current_waypoint_index = 0
            self.navigation_start_time = time.time()
            
            # 发送确认消息
            self.send_acknowledgment("navigation_started", "success", 
                                   f"开始展台导航，共 {self.total_waypoints} 个点位")
            
            # 开始导航序列
            self.start_navigation_sequence()
            
        except Exception as e:
            self.get_logger().error(f"开始展台导航错误: {e}")
            self.send_acknowledgment("navigation_started", "error", f"开始导航失败: {str(e)}")
    
    def handle_stop_navigation(self, command_data: Dict[str, Any]):
        """处理停止导航"""
        try:
            if self.current_state == NavigationState.IDLE and self.pending_navigation_request is not None:
                self.pending_navigation_request = None
                self.pending_navigation_reason = ""
                self.send_acknowledgment("navigation_pending_cancelled", "success", "已取消待执行导航")
                self.publish_status_update("navigation_pending_cancelled", {
                    "reason": "user_stop"
                })
                self.get_logger().info("已取消待执行导航")
                return

            if self.current_state == NavigationState.IDLE:
                self.send_acknowledgment("stop_navigation", "error", "当前没有在执行导航")
                return
            
            stop_params = command_data.get("stop_parameters", {})
            emergency_stop = stop_params.get("emergency_stop", False)
            stop_reason = stop_params.get("reason", "user_request")

            # 取消当前导航目标
            if self.current_goal_handle:
                self.cancel_navigation()
            self.publish_zero_cmd_vel()
            
            # 更新状态
            completed = self.current_waypoint_index
            total = self.total_waypoints
            
            self.current_state = NavigationState.CANCELLED
            
            # 计算导航总结
            navigation_duration = time.time() - self.navigation_start_time if self.navigation_start_time > 0 else 0
            completion_percentage = (completed / total * 100) if total > 0 else 0
            # 发送确认消息
            self.send_acknowledgment("navigation_stopped", "success", 
                                   f"导航已停止，完成 {completed}/{total} 个点位")
            
            # 发布状态更新
            self.publish_status_update("navigation_stopped", {
                "reason": stop_reason,
                "emergency_stop": emergency_stop,
                "completed_waypoints": completed,
                "total_waypoints": total,
                "navigation_duration": round(navigation_duration, 1),
                "completion_percentage": round(completion_percentage, 1),
                "last_waypoint_reached": self.waypoint_ids[completed - 1] if completed > 0 else None
            })
            
            # 重置状态
            self.reset_navigation_state()
            
            self.get_logger().info(f"导航已停止 (原因: {stop_reason}, 紧急: {emergency_stop})")
            
        except Exception as e:
            self.get_logger().error(f"停止导航错误: {e}")
            self.send_acknowledgment("navigation_stopped", "error", f"停止导航失败: {str(e)}")
    
    def handle_pause_navigation(self, command_data: Dict[str, Any]):
        """处理暂停导航"""
        try:
            if self.current_state != NavigationState.EXECUTING:
                self.send_acknowledgment("pause_navigation", "error", "当前没有在执行导航")
                return
        
            # ✅ 使用 command_data 中的参数
            pause_params = command_data.get("pause_parameters", {})
            pause_duration = pause_params.get("pause_duration", 0)  # 0=无限期
        
            # 暂停导航
            self.current_state = NavigationState.PAUSED
            self.pause_time = time.time()
            self.pause_duration_limit = pause_duration
            self.reset_block_detection()
        
            # 物理打断底盘：取消当前 Nav2 目标
            if self.current_goal_handle:
                self.cancel_navigation()
        
            # 记录暂停位置
            pause_location = None
            if self.current_pose:
                pause_location = {
                "x": self.current_pose.position.x,
                "y": self.current_pose.position.y,
                "z": self.current_pose.position.z
                }
        
            # 发送确认消息
            self.send_acknowledgment("pause_navigation", "success", "导航已暂停")
        
            # 发布状态更新
            self.publish_status_update("navigation_paused", {
            "pause_location": pause_location,
            "pause_time": self.pause_time,
            "pause_duration": pause_duration,
            "current_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "current_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints
             })
        
            self.get_logger().info(f"导航已暂停 (暂停时长限制: {'无限期' if pause_duration == 0 else f'{pause_duration}秒'})")
        
        except Exception as e:
            self.get_logger().error(f"暂停导航错误: {e}")
            self.send_acknowledgment("pause_navigation", "error", f"暂停导航失败: {str(e)}")
    
    def handle_resume_navigation(self, command_data: Dict[str, Any]):
        """处理恢复导航"""
        try:
            if self.current_state == NavigationState.RECOVERABLE_FAILED:
                self.send_acknowledgment(
                    "navigation_resumed",
                    "error",
                    "导航处于失败可恢复状态，请使用 retry_failed_waypoint 或 skip_failed_waypoint"
                )
                return

            if self.current_state != NavigationState.PAUSED:
                self.send_acknowledgment("resume_navigation", "error", "导航未暂停")
                return
            
            # 检查是否有可恢复的路点
            if not self.current_waypoint:
                self.send_acknowledgment("navigation_resumed", "error", "没有可恢复的导航目标")
                self.reset_navigation_state()
                return

            if self.reject_navigation_start_if_robot_not_ready("navigation_resumed"):
                return
            
            # ✅ 使用 command_data（虽然当前表格中 resume 没有额外参数，但预留扩展性）
            resume_reason = command_data.get("reason", "user_request")
        
            # 计算暂停了多久
            pause_elapsed = time.time() - self.pause_time if hasattr(self, 'pause_time') else 0

            # 恢复导航
            self.current_state = NavigationState.EXECUTING
            
            # 发送确认消息
            self.send_acknowledgment("navigation_resumed", "success", "导航已恢复")
            
            # 发布状态更新
            self.publish_status_update("navigation_resumed", {
            "resumed_waypoint_id": self.current_waypoint.get("id", ""),
            "resumed_waypoint_name": self.current_waypoint.get("name", ""),
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "pause_duration_actual": round(pause_elapsed, 1),
            "resume_reason": resume_reason
            })
            
            # 重新导航到当前路点
            self.navigate_to_waypoint(self.current_waypoint)
        
            self.get_logger().info(
                 f"导航已恢复，暂停了 {pause_elapsed:.1f}秒，继续前往: "
                 f"{self.current_waypoint.get('name', '')} ({self.current_waypoint_index + 1}/{self.total_waypoints})"
            )
        
        except Exception as e:
            self.get_logger().error(f"恢复导航错误: {e}")
            self.send_acknowledgment("navigation_resumed", "error", f"恢复导航失败: {str(e)}")

    def localization_recovery_status_callback(self, msg: String):
        """定位恢复状态回调：定位失锁时暂停导航，恢复后继续当前未完成路点。"""
        try:
            status = json.loads(msg.data)
            event_type = status.get("event_type", "")
            self.localization_recovery_last_status = status

            if event_type == "localization_recovery_started":
                self.handle_localization_recovery_started(status)
            elif event_type in (
                "localization_recovery_prior_published",
                "localization_relocalize_requested",
                "localization_relocalize_completed",
                "localization_initialpose_published",
                "localization_recovery_waiting",
            ):
                self.handle_localization_recovery_progress(status)
            elif event_type == "localization_relocalize_failed":
                self.handle_localization_recovery_failed(status)
            elif event_type == "localization_recovered":
                self.handle_localization_recovered(status)

        except Exception as e:
            self.get_logger().error(f"处理定位恢复状态错误: {e}")

    def handle_localization_recovery_started(self, status: Dict[str, Any]):
        if not self.auto_pause_on_localization_recovery:
            return

        reason = status.get("reason", "定位异常，正在重定位")
        self.localization_recovery_active = True
        self.localization_recovery_reason = reason
        self.localization_recovery_started_at = time.time()

        if self.current_state in (NavigationState.EXECUTING, NavigationState.PLANNING):
            self.localization_auto_paused = True
            self.localization_resume_pending = False
            self.current_state = NavigationState.PAUSED
            self.current_detailed_state = "LOCALIZATION_RECOVERY"
            self.pause_time = time.time()
            self.pause_duration_limit = 0
            self.reset_block_detection()

            if self.current_goal_handle:
                self.cancel_navigation()

            event_data = self.build_localization_pause_context(status)
            self.publish_status_update("navigation_paused", event_data)
            self.publish_status_update("navigation_localization_recovery_started", event_data)
            self.send_acknowledgment(
                "navigation_auto_paused",
                "success",
                "定位异常，已暂停导航并开始自动重定位",
                event_data
            )
            self.get_logger().warning(f"定位异常，自动暂停导航: {reason}")
            return

        if self.current_state == NavigationState.PAUSED and self.localization_auto_paused:
            self.publish_status_update(
                "navigation_localization_recovery_started",
                self.build_localization_pause_context(status)
            )

    def handle_localization_recovery_progress(self, status: Dict[str, Any]):
        if not self.localization_recovery_active:
            return

        self.publish_status_update("navigation_localization_recovery_progress", {
            "reason": status.get("reason", self.localization_recovery_reason),
            "localization_event": status.get("event_type", ""),
            "recovery_count": status.get("recovery_count", 0),
            "relocalize_attempts": status.get("relocalize_attempts", 0),
            "current_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "current_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "auto_resume_pending": self.localization_auto_paused,
        })

    def handle_localization_recovery_failed(self, status: Dict[str, Any]):
        self.localization_recovery_active = True
        self.localization_recovery_reason = status.get("reason", self.localization_recovery_reason)
        self.publish_status_update("navigation_localization_recovery_failed", {
            "reason": self.localization_recovery_reason,
            "current_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "current_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "auto_resume_pending": self.localization_auto_paused,
        })

    def handle_localization_recovered(self, status: Dict[str, Any]):
        self.localization_recovery_active = False
        self.localization_recovery_reason = status.get("reason", "定位已恢复")

        event_data = {
            "reason": self.localization_recovery_reason,
            "recovery_duration": round(time.time() - self.localization_recovery_started_at, 1)
                if self.localization_recovery_started_at else 0.0,
            "current_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "current_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "auto_resume_pending": self.localization_auto_paused,
        }
        self.publish_status_update("navigation_localization_recovered", event_data)

        if self.localization_auto_paused:
            self.localization_resume_pending = True
            self.try_resume_after_localization_recovery()

    def try_resume_after_localization_recovery(self):
        if not self.localization_resume_pending:
            return

        if self.current_state != NavigationState.PAUSED or not self.localization_auto_paused:
            self.localization_resume_pending = False
            return

        if not self.current_waypoint:
            self.localization_resume_pending = False
            self.localization_auto_paused = False
            self.publish_status_update("navigation_localization_resume_failed", {
                "reason": "没有可恢复的导航目标"
            })
            self.reset_navigation_state()
            return

        block_reason = self.get_navigation_start_block_reason()
        if block_reason:
            self.publish_status_update("navigation_localization_resume_waiting", {
                "reason": block_reason,
                "current_waypoint_id": self.current_waypoint.get("id", ""),
                "current_waypoint_name": self.current_waypoint.get("name", ""),
                "waypoint_index": self.current_waypoint_index,
                "total_waypoints": self.total_waypoints,
            })
            return

        pause_elapsed = time.time() - self.pause_time if self.pause_time else 0.0
        event_data = {
            "resumed_waypoint_id": self.current_waypoint.get("id", ""),
            "resumed_waypoint_name": self.current_waypoint.get("name", ""),
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "pause_duration_actual": round(pause_elapsed, 1),
            "resume_reason": "localization_recovered",
            "localization_reason": self.localization_recovery_reason,
        }

        self.localization_resume_pending = False
        self.localization_auto_paused = False
        self.current_state = NavigationState.EXECUTING
        self.current_detailed_state = "EXECUTING"

        self.send_acknowledgment(
            "navigation_auto_resumed",
            "success",
            "定位已恢复，继续未完成导航",
            event_data
        )
        self.publish_status_update("navigation_resumed", event_data)
        self.navigate_to_waypoint(self.current_waypoint)

        self.get_logger().info(
            f"定位恢复后自动继续导航: {self.current_waypoint.get('name', '')} "
            f"({self.current_waypoint_index + 1}/{self.total_waypoints})"
        )

    def publish_zero_cmd_vel(self):
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception as e:
            self.get_logger().error(f"发布零速度失败: {e}")

    def build_localization_pause_context(self, status: Dict[str, Any]) -> Dict[str, Any]:
        pause_location = None
        if self.current_pose:
            pause_location = {
                "x": self.current_pose.position.x,
                "y": self.current_pose.position.y,
                "z": self.current_pose.position.z,
            }

        return {
            "pause_source": "localization_recovery",
            "reason": status.get("reason", "定位异常，正在重定位"),
            "pause_location": pause_location,
            "pause_time": self.pause_time,
            "pause_duration": 0,
            "current_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "current_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "localization_event": status.get("event_type", ""),
            "recovery_count": status.get("recovery_count", 0),
            "use_prior": status.get("use_prior", False),
        }

    def handle_retry_failed_waypoint(self, command_data: Dict[str, Any]):
        """失败可恢复状态下，重试当前失败点。"""
        try:
            if self.current_state != NavigationState.RECOVERABLE_FAILED:
                self.send_acknowledgment("retry_failed_waypoint", "error", "当前没有可重试的失败导航")
                return

            if not self.current_waypoint:
                self.send_acknowledgment("retry_failed_waypoint", "error", "失败点位信息缺失，无法重试")
                self.reset_navigation_state()
                return

            if self.reject_navigation_start_if_robot_not_ready("retry_failed_waypoint"):
                return

            context = self.build_failure_recovery_context()
            self.current_state = NavigationState.EXECUTING
            self.current_detailed_state = "EXECUTING"
            self.reset_block_detection()

            self.send_acknowledgment(
                "retry_failed_waypoint",
                "success",
                f"重试失败点: {self.current_waypoint.get('name', '')}",
                context
            )
            self.publish_status_update("navigation_retry_failed_waypoint", context)
            self.navigate_to_waypoint(self.current_waypoint)

        except Exception as e:
            self.get_logger().error(f"重试失败点错误: {e}")
            self.send_acknowledgment("retry_failed_waypoint", "error", f"重试失败点失败: {str(e)}")

    def handle_skip_failed_waypoint(self, command_data: Dict[str, Any]):
        """失败可恢复状态下，跳过当前失败点继续后续点位。"""
        try:
            if self.current_state != NavigationState.RECOVERABLE_FAILED:
                self.send_acknowledgment("skip_failed_waypoint", "error", "当前没有可跳过的失败导航")
                return

            failed_context = self.build_failure_recovery_context()
            failed_waypoint = self.current_waypoint or {}
            next_waypoint_index = self.current_waypoint_index + 1
            has_next_waypoint = next_waypoint_index < self.total_waypoints
            next_waypoint_data = None
            next_waypoint_id = ""

            if has_next_waypoint:
                if self.reject_navigation_start_if_robot_not_ready("skip_failed_waypoint"):
                    return

                next_waypoint_id = self.waypoint_ids[next_waypoint_index]
                next_waypoint_data = self.find_waypoint_data_by_id(next_waypoint_id)
                if not next_waypoint_data:
                    self.send_acknowledgment(
                        "skip_failed_waypoint",
                        "error",
                        f"下一个路点 '{next_waypoint_id}' 不存在",
                        failed_context
                    )
                    return

            skipped_waypoint = {
                "waypoint_index": self.current_waypoint_index,
                "waypoint_id": failed_waypoint.get("id", ""),
                "waypoint_name": failed_waypoint.get("name", ""),
                "reason": failed_context.get("reason", "")
            }
            self.skipped_waypoints.append(skipped_waypoint)

            self.current_waypoint_index = next_waypoint_index
            self.current_goal_handle = None
            self.reset_block_detection()

            if not has_next_waypoint:
                completion_context = dict(failed_context)
                completion_context.update({
                    "recoverable": False,
                    "available_actions": [],
                    "skipped_waypoint": skipped_waypoint,
                    "skipped_waypoints": self.skipped_waypoints,
                    "remaining_waypoint_ids": []
                })
                self.send_acknowledgment(
                    "skip_failed_waypoint",
                    "success",
                    "已跳过失败点，后续没有更多点位，导航完成",
                    completion_context
                )
                self.handle_navigation_completed()
                return

            context = dict(failed_context)
            context.update({
                "recoverable": False,
                "available_actions": ["pause_navigation", "stop_navigation"],
                "skipped_from": failed_context,
                "skipped_waypoint": skipped_waypoint,
                "skipped_waypoints": self.skipped_waypoints,
                "next_waypoint_index": self.current_waypoint_index,
                "next_waypoint_id": next_waypoint_id,
                "next_waypoint_name": next_waypoint_data.get("name", ""),
                "remaining_waypoint_ids": self.waypoint_ids[self.current_waypoint_index:]
            })
            self.current_state = NavigationState.PLANNING
            self.current_detailed_state = "PLANNING"
            self.send_acknowledgment(
                "skip_failed_waypoint",
                "success",
                f"已跳过失败点，继续导航到: {next_waypoint_data.get('name', next_waypoint_id)}",
                context
            )
            self.publish_status_update("navigation_skip_failed_waypoint", context)
            self.navigate_to_waypoint(next_waypoint_data)

        except Exception as e:
            self.get_logger().error(f"跳过失败点错误: {e}")
            self.send_acknowledgment("skip_failed_waypoint", "error", f"跳过失败点失败: {str(e)}")

    def handle_abort_failed_navigation(self, command_data: Dict[str, Any]):
        """失败可恢复状态下，终止整轮任务并清空上下文。"""
        try:
            if self.current_state not in (NavigationState.RECOVERABLE_FAILED, NavigationState.FAILED):
                self.send_acknowledgment("abort_failed_navigation", "error", "当前没有可终止的失败导航")
                return

            context = self.build_failure_recovery_context()
            self.send_acknowledgment(
                "abort_failed_navigation",
                "success",
                "已终止失败导航任务",
                context
            )
            self.publish_status_update("navigation_aborted", context)
            self.reset_navigation_state()

        except Exception as e:
            self.get_logger().error(f"终止失败导航错误: {e}")
            self.send_acknowledgment("abort_failed_navigation", "error", f"终止失败导航失败: {str(e)}")
    
    def start_navigation_sequence(self):
        """开始导航序列"""
        if not self.waypoint_ids or self.current_waypoint_index >= len(self.waypoint_ids):
            self.get_logger().error("导航序列为空或已完成")
            return
        
        # 获取第一个路点
        waypoint_id = self.waypoint_ids[self.current_waypoint_index]
        waypoint_data = self.find_waypoint_data_by_id(waypoint_id)
        
        if not waypoint_data:           
            self.get_logger().error(f"路点 '{waypoint_id}' 不存在")
            self.handle_navigation_failed(f"路点 '{waypoint_id}' 不存在")
            return
        
        # 开始导航到第一个路点
        self.navigate_to_waypoint(waypoint_data)
    
    def navigate_to_waypoint(self, waypoint_data: Dict[str, Any]):
        """导航到指定路点"""
        try:
            self.current_waypoint = waypoint_data
            self.current_state = NavigationState.EXECUTING
            self.current_waypoint_start_time = time.time()
            
            # 创建导航目标
            goal_pose = self.waypoint_to_pose_stamped(waypoint_data)
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose
        
            # 等待动作服务器
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Nav2动作服务器不可用")
                self.handle_navigation_failed("Nav2服务器不可用")
                return
        
            # 发送目标
            self.future = self.nav_to_pose_client.send_goal_async(
               goal_msg, 
               feedback_callback=self.nav2_feedback_callback
            )
            self.future.add_done_callback(self.nav2_goal_response_callback)
            
            # 发布路点开始状态
            self.publish_status_update("waypoint_started", {
                "waypoint_id": waypoint_data.get("id", ""),
                "waypoint_name": waypoint_data.get("name", ""),
                "waypoint_index": self.current_waypoint_index,
                "total_waypoints": self.total_waypoints,
                "position": waypoint_data.get("position", [])
            })
            
            self.get_logger().info(
                f"开始导航到路点: {waypoint_data.get('name', '')} "
                f"({self.current_waypoint_index + 1}/{self.total_waypoints})"
            )
            
        except Exception as e:
            self.get_logger().error(f"导航到路点错误: {e}")
            self.handle_navigation_failed(f"导航到路点失败: {str(e)}")
    
    def waypoint_to_pose_stamped(self, waypoint_data: Dict[str, Any]) -> PoseStamped:
        """将点位数据转换为PoseStamped"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = waypoint_data.get("frame_id", self.default_frame_id)
        
        position = waypoint_data.get("position", [0.0, 0.0, 0.0])
        orientation = waypoint_data.get("orientation", [0.0, 0.0, 0.0, 1.0])
        
        # 设置位置
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        
        # 设置方向
        pose.pose.orientation.x = float(orientation[0])
        pose.pose.orientation.y = float(orientation[1])
        pose.pose.orientation.z = float(orientation[2])
        pose.pose.orientation.w = float(orientation[3])
        
        return pose
    
    def check_navigation_status(self):
        """检查导航状态 - 方案A：仅用于超时监控，不触发到达逻辑"""
        if self.current_state != NavigationState.EXECUTING:
            return

        # 记录距离，用于 APP 端的 UI 进度条显示
        if self.current_pose and self.current_waypoint:
            self.last_known_distance = self.calculate_distance_to_waypoint()
    
    def check_timeout(self):
        """检查导航超时"""
        if (self.current_state == NavigationState.EXECUTING and 
            self.current_waypoint is not None):
            
            current_time = time.time()
            waypoint_duration = current_time - self.current_waypoint_start_time
            
            if waypoint_duration > self.waypoint_timeout:
                self.get_logger().warning(f"路点导航超时，持续时间: {waypoint_duration:.1f}秒")
                self.handle_navigation_failed("路点导航超时")
    
    def calculate_distance_to_waypoint(self) -> float:
        """计算到当前路点的距离"""
        if not self.current_pose or not self.current_waypoint:
            return float('inf')
        
        try:
            current_pos = self.current_pose.position
            waypoint_pos = self.current_waypoint.get("position", [0.0, 0.0, 0.0])
            
            dx = current_pos.x - waypoint_pos[0]
            dy = current_pos.y - waypoint_pos[1]
            dz = current_pos.z - waypoint_pos[2]
            
            return math.sqrt(dx**2 + dy**2 + dz**2)
            
        except Exception as e:
            self.get_logger().error(f"计算距离错误: {e}")
            return float('inf')
    
    def handle_nav2_succeeded(self):
        """处理Nav2成功到达"""
        if (self.current_state == NavigationState.EXECUTING and 
            self.current_waypoint is not None):

            self.waypoint_arrived_by_nav2 = True
            waypoint_id = self.current_waypoint.get("id", "")
            waypoint_name = self.current_waypoint.get("name", "")
            
            # 发布路点到达状态
            self.publish_status_update("waypoint_reached", {
                "waypoint_id": waypoint_id,
                "waypoint_name": waypoint_name,
                "waypoint_index": self.current_waypoint_index,
                "total_waypoints": self.total_waypoints,
                "confirmation_source": "nav2"
            })
            
            self.get_logger().info(f"Nav2确认到达路点: {waypoint_name}")
            
            # 移动到下一个路点或完成导航
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= self.total_waypoints:
                self.handle_navigation_completed()
            else:
                next_waypoint_id = self.waypoint_ids[self.current_waypoint_index]
                next_waypoint_data = self.find_waypoint_data_by_id(next_waypoint_id)
            
                if next_waypoint_data:
                    def trigger_next_waypoint():
                        # 第一时间取消定时器，防止它无限循环（实现 oneshot 的效果）
                        if hasattr(self, '_next_waypoint_timer') and self._next_waypoint_timer:
                            self._next_waypoint_timer.cancel()
                            self._next_waypoint_timer = None
                        # 执行前往下一个路点的指令
                        self.navigate_to_waypoint(next_waypoint_data)
                        
                    # 创建普通的定时器，把包装好的销毁函数绑上去
                    self._next_waypoint_timer = self.create_timer(1.0, trigger_next_waypoint)
                else:
                    self.handle_navigation_failed(f"下一个路点 '{next_waypoint_id}' 不存在")
    
    def handle_nav2_failed(self):
        """处理Nav2失败"""
        self.handle_navigation_failed("Nav2导航失败")
    
    def handle_nav2_cancelled(self):
        """处理Nav2取消"""
        if self.current_state == NavigationState.PAUSED:
            self.get_logger().info("Nav2 取消是由用户暂停触发，忽略重置操作")
            return
        if self.current_state == NavigationState.EXECUTING:
            self.current_state = NavigationState.CANCELLED
            self.publish_status_update("navigation_cancelled", {
                "reason": "nav2_cancelled"
            })
            self.reset_navigation_state()
    
    def handle_navigation_completed(self):
        """处理导航完成"""
        self.current_state = NavigationState.COMPLETED
        completion_context = {
            "completed_waypoints": self.total_waypoints,
            "total_waypoints": self.total_waypoints,
            "navigation_mode": self.current_navigation_mode.value if self.current_navigation_mode else None,
            "skipped_waypoints": self.skipped_waypoints
        }
        
        # 发送确认消息
        self.send_acknowledgment("navigation_completed", "success", 
                               f"导航完成，共完成 {self.total_waypoints} 个点位",
                               completion_context)
        
        # 发布状态更新
        self.publish_status_update("navigation_completed", completion_context)
        
        self.get_logger().info("导航完成")
        
        # 重置状态
        self.reset_navigation_state()

    def build_failure_recovery_context(self, reason: str = "") -> Dict[str, Any]:
        """构建失败恢复上下文，供 APP 展示和决策使用。"""
        current_waypoint = self.current_waypoint or {}
        failed_index = self.current_waypoint_index
        remaining_waypoint_ids = []
        if self.waypoint_ids and 0 <= failed_index < len(self.waypoint_ids):
            remaining_waypoint_ids = self.waypoint_ids[failed_index:]

        return {
            "recoverable": True,
            "reason": reason or self.last_failure_context.get("reason", ""),
            "failed_waypoint_index": failed_index,
            "failed_waypoint_id": current_waypoint.get("id", ""),
            "failed_waypoint_name": current_waypoint.get("name", ""),
            "completed_waypoints": failed_index,
            "total_waypoints": self.total_waypoints,
            "remaining_waypoint_ids": remaining_waypoint_ids,
            "current_sequence_id": self.current_sequence_id,
            "navigation_mode": self.current_navigation_mode.value if self.current_navigation_mode else None,
            "skipped_waypoints": self.skipped_waypoints,
            "available_actions": [
                "retry_failed_waypoint",
                "skip_failed_waypoint",
                "stop_navigation"
            ],
            "resume_navigation_allowed": False
        }
    
    def handle_navigation_failed(self, reason: str):
        """处理导航失败"""
        # 如果当前状态是 PAUSED(暂停)，说明是我们为了互动主动取消的，不要标记为失败，也不要重置数据
        if self.current_state == NavigationState.PAUSED:
            self.get_logger().info("检测到导航由用户主动暂停，保留数据以备恢复...")
            return

        if self.navigation_failure_policy == "abort_all":
            self.current_state = NavigationState.FAILED
            self.send_acknowledgment("navigation_failed", "error", reason)
            self.publish_status_update("navigation_failed", {
                "reason": reason,
                "failed_waypoint_index": self.current_waypoint_index,
                "failed_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else ""
            })
            self.get_logger().error(f"导航失败: {reason}")
            self.reset_navigation_state()
            return

        if self.navigation_failure_policy == "skip_failed_continue":
            if self.current_goal_handle:
                self.cancel_navigation()
            self.current_state = NavigationState.RECOVERABLE_FAILED
            self.last_failure_context = self.build_failure_recovery_context(reason)
            self.handle_skip_failed_waypoint({})
            return

        # 只有在非暂停状态下的取消/报错，才视为真实失败
        if self.current_goal_handle:
            self.cancel_navigation()

        self.current_state = NavigationState.RECOVERABLE_FAILED
        self.current_detailed_state = "RECOVERABLE_FAILED"
        self.current_goal_handle = None
        self.reset_block_detection()
        self.last_failure_context = self.build_failure_recovery_context(reason)
        
        # 发送确认消息
        self.send_acknowledgment("navigation_failed", "error", reason, self.last_failure_context)
        
        # 发布状态更新
        self.publish_status_update("navigation_failed", self.last_failure_context)
        
        self.get_logger().error(f"导航失败，进入可恢复失败状态: {reason}")
    
    def nav2_log_callback(self, msg):
        """解析 Nav2 行为树日志，识别当前正在进行的具体动作"""
        for event in msg.event_log:
            # 检测是否正在执行会主动驱动机器人运动的行为
            if self.is_nav2_blockage_suppression_node(event.node_name):
                if event.current_status == "RUNNING":
                    was_active = event.node_name in self.nav2_blockage_suppression_nodes
                    self.nav2_blockage_suppression_nodes.add(event.node_name)
                    self.current_detailed_state = "TURNING" if event.node_name == "SpinToPose" else "RECOVERING"
                    if not was_active:
                        self.get_logger().info(f"Nav2 正在执行主动转向/后退行为: {event.node_name}")
                else:
                    self.nav2_blockage_suppression_nodes.discard(event.node_name)
                    if (
                        not self.nav2_blockage_suppression_nodes and
                        self.current_detailed_state in {"RECOVERING", "TURNING"}
                    ):
                        self.current_detailed_state = "EXECUTING" if self.current_state == NavigationState.EXECUTING else self.current_state.value
                
             # 检测是否有规划错误
            if "ComputePathToPose" in event.node_name and event.current_status == "FAILURE":
                self.current_detailed_state = "PLANNING_FAILED"
                self.get_logger().error("Nav2 路径规划失败！")

    def cancel_navigation(self):
        """取消当前导航"""
        try:
            if self.current_goal_handle:
               future = self.current_goal_handle.cancel_goal_async()
               future.add_done_callback(self.nav2_cancel_callback)
               self.get_logger().info("发送取消导航请求")
            else:
               self.get_logger().warning("没有活动的导航目标可取消")
        except Exception as e:
             self.get_logger().error(f"取消导航错误: {e}")
    
    def nav2_cancel_callback(self, future):
        """处理取消导航结果"""
        try:
            response = future.result()
            if response.return_code == CancelGoal.Response.ERROR_NONE:
               self.current_goal_handle = None
               self.get_logger().info("导航已成功取消")
            else:
               self.get_logger().warning(f"取消导航请求未被接受，return_code={response.return_code}")
        except Exception as e:
            self.current_goal_handle = None
            self.get_logger().error(f"处理取消导航响应错误: {e}")

    def find_waypoint_data_by_id(self, waypoint_id: str) -> Optional[Dict[str, Any]]:
        """根据ID查找点位数据（支持扁平化、嵌套、或字典内部结构）"""
        if not waypoint_id:
            return None

        # 方案1: 直接顶层查找
        if waypoint_id in self.waypoints_data:
            data = self.waypoints_data[waypoint_id]
            return data if isinstance(data, dict) else None
   
        # 方案2: 深入一层
        for category_dict in self.waypoints_data.values():
            if isinstance(category_dict, dict) and waypoint_id in category_dict:
              return category_dict[waypoint_id]

         # 方案3: 再深入一层（兼容三级结构）
        for category_dict in self.waypoints_data.values():
            if isinstance(category_dict, dict):
              for sub_dict in category_dict.values():
                if isinstance(sub_dict, dict) and waypoint_id in sub_dict:
                    return sub_dict[waypoint_id]

        return None
    
    def send_acknowledgment(
        self,
        ack_type: str,
        status: str,
        message: str = "",
        extra_data: Optional[Dict[str, Any]] = None
    ):
        """发送确认消息给路点管理器"""
        try:
            ack_msg = String()
            payload = {
                "ack_type": ack_type,
                "status": status,
                "message": message,
                "timestamp": time.time()
            }
            if extra_data:
                payload.update(extra_data)
            ack_msg.data = json.dumps(payload, ensure_ascii=False)
            self.navigation_ack_pub.publish(ack_msg)
        except Exception as e:
            self.get_logger().error(f"发送确认消息错误: {e}")
    
    def publish_navigation_status(self):
        """发布完整的导航状态信息"""
        try:
            status_data = self.get_current_status_summary()
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.navigation_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"发布导航状态错误: {e}")
    
    def publish_status_update(self, event_type: str, event_data: Dict[str, Any]):
        """发布状态更新事件"""
        try:
            update_data = {
                "event_type": event_type,
                "event_data": event_data,
                "timestamp": time.time(),
                "current_state": self.current_state.value,
                "navigation_mode": self.current_navigation_mode.value if self.current_navigation_mode else None,
                "sequence_id": self.current_sequence_id
            }
            
            update_msg = String()
            update_msg.data = json.dumps(update_data)
            self.navigation_status_pub.publish(update_msg)
            
            self.get_logger().debug(f"发布状态更新: {event_type}")
            
        except Exception as e:
            self.get_logger().error(f"发布状态更新错误: {e}")
    
    def publish_navigation_path(self):
        """发布导航路径（用于可视化）"""
        try:
            if not self.waypoint_ids or self.current_waypoint_index >= len(self.waypoint_ids):
                return
            
            # 创建路径消息
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = self.default_frame_id
            
            # 从当前路点开始添加剩余路径点
            remaining_waypoint_ids = self.waypoint_ids[self.current_waypoint_index:]
            
            for waypoint_id in remaining_waypoint_ids:
                waypoint_data = self.find_waypoint_data_by_id(waypoint_id)
                if waypoint_data:
                    pose_stamped = self.waypoint_to_pose_stamped(waypoint_data)
                    path_msg.poses.append(pose_stamped)
            
            # 发布路径
            self.navigation_path_pub.publish(path_msg)
            
        except Exception as e:
            self.get_logger().error(f"发布导航路径错误: {e}")
    
    def get_current_status_summary(self) -> Dict[str, Any]:
        """获取当前状态摘要"""
        status_summary = {
            "timestamp": time.time(),
            "current_state": self.current_state.value,
            "navigation_mode": self.current_navigation_mode.value if self.current_navigation_mode else None,
            "sequence_id": self.current_sequence_id,
            "current_waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "progress_percentage": self.calculate_progress_percentage(),
            "is_active": self.is_navigation_active(),
            "navigation_duration": time.time() - self.navigation_start_time if self.navigation_start_time > 0 else 0,
            "detailed_state": self.current_detailed_state,
            "recovery_active": self.current_detailed_state == "RECOVERING",
            "obstacle_blocked": self.is_blocked_by_obstacle,
            "block_duration": (time.time() - self.block_start_time) if self.block_start_time else 0,
            "block_reported": self.block_reported,
            "obstacle_block_suppressed": bool(self.get_obstacle_blockage_suppression_reason()),
            "obstacle_block_suppression_reason": self.get_obstacle_blockage_suppression_reason() or "",
            "pending_navigation": self.pending_navigation_request is not None,
            "pending_navigation_age": (
                time.time() - self.pending_navigation_created_at
                if self.pending_navigation_request is not None else 0
            ),
            "pending_navigation_reason": self.pending_navigation_reason,
            "failure_recoverable": self.current_state == NavigationState.RECOVERABLE_FAILED,
            "failure_context": self.last_failure_context,
            "skipped_waypoints": self.skipped_waypoints,
            "localization_recovery_active": self.localization_recovery_active,
            "localization_auto_paused": self.localization_auto_paused,
            "localization_resume_pending": self.localization_resume_pending,
            "localization_recovery_reason": self.localization_recovery_reason,
            "localization_recovery_status": self.localization_recovery_last_status
        }
        
        # 添加当前位置信息
        if self.current_pose:
            status_summary["current_pose"] = {
                "frame_id": self.current_pose_frame,
                "position": {
                    "x": self.current_pose.position.x,
                    "y": self.current_pose.position.y,
                    "z": self.current_pose.position.z
                },
                "orientation": {
                    "x": self.current_pose.orientation.x,
                    "y": self.current_pose.orientation.y,
                    "z": self.current_pose.orientation.z,
                    "w": self.current_pose.orientation.w
                }
            }
        
        # 添加当前目标信息
        if self.current_waypoint:
            status_summary["current_goal"] = {
                "waypoint_id": self.current_waypoint.get("id", ""),
                "waypoint_name": self.current_waypoint.get("name", ""),
                "position": self.current_waypoint.get("position", []),
                "waypoint_index": self.current_waypoint_index
            }
            
            # 计算到目标的距离
            distance = self.calculate_distance_to_waypoint()
            status_summary["distance_to_goal"] = distance
        
        # 添加机器人速度信息
        if self.current_velocity:
            status_summary["current_velocity"] = {
                "linear": {
                    "x": self.current_velocity.linear.x,
                    "y": self.current_velocity.linear.y,
                    "z": self.current_velocity.linear.z
                },
                "angular": {
                    "x": self.current_velocity.angular.x,
                    "y": self.current_velocity.angular.y,
                    "z": self.current_velocity.angular.z
                }
            }
        
        return status_summary
    
    def calculate_progress_percentage(self) -> float:
        """计算导航进度百分比"""
        if self.total_waypoints == 0:
            return 0.0
        
        # 如果导航完成但还在发布状态，显示100%
        if self.current_state == NavigationState.COMPLETED:
            return 100.0
        
        # 计算基于已完成路点的进度
        progress = (self.current_waypoint_index / max(self.total_waypoints, 1)) * 100
        
        # 如果正在执行当前路点，可以基于距离进一步细化进度
        if (self.current_state == NavigationState.EXECUTING and 
            self.current_pose and self.current_waypoint):
            
            distance = self.calculate_distance_to_waypoint()
            max_distance = 10.0  # 假设最大距离为10米
            
            if distance < max_distance:
                # 在当前路点内进一步细化进度
                waypoint_progress = (1 - min(distance / max_distance, 1)) * (100 / self.total_waypoints)
                progress = min(progress + waypoint_progress, 100.0)
        
        return round(progress, 1)
    
    def is_navigation_active(self) -> bool:
        """判断导航是否处于活动状态"""
        active_states = [
            NavigationState.PLANNING,
            NavigationState.EXECUTING,
            NavigationState.PAUSED,
            NavigationState.RECOVERABLE_FAILED
        ]
        return self.current_state in active_states
    
    def reset_navigation_state(self):
        """重置导航状态"""
        self.current_state = NavigationState.IDLE
        self.current_navigation_mode = None
        self.current_sequence_id = None
        self.current_waypoint_index = 0
        self.total_waypoints = 0
        self.current_waypoint = None
        self.waypoint_ids = []
        self.skipped_waypoints = []
        self.last_failure_context = {}
        self.navigation_start_time = 0
        self.current_goal_handle = None
        self.waypoint_arrived_by_nav2 = False
        self.localization_recovery_active = False
        self.localization_auto_paused = False
        self.localization_resume_pending = False
        self.localization_recovery_reason = ""
        self.localization_recovery_started_at = 0.0
        self.localization_recovery_last_status = {}
        
        # 重置阻塞检测状态
        self.reset_block_detection()
        self.nav2_blockage_suppression_nodes.clear()
        self.distance_remaining = float('inf')
        self.estimated_time_remaining = 0.0
    
    def destroy_node(self):
        """销毁节点前的清理工作"""
        try:
            # 停止当前导航
            if self.is_navigation_active():
                self.cancel_navigation()
                self.publish_status_update("navigation_stopped", {
                    "reason": "node_shutdown"
                })
            
            self.get_logger().info("导航状态管理器节点销毁完成")
        except Exception as e:
            self.get_logger().error(f"销毁节点错误: {e}")
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = NavigationStateManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到键盘中断信号")
    except Exception as e:
        node.get_logger().error(f"导航状态管理器运行错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
