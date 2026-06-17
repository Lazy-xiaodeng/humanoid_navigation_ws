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
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.msg import BehaviorTreeLog
import rclpy.duration
from tf2_ros import Buffer, TransformException, TransformListener

import json
import time
import math
from enum import Enum
from typing import Dict, List, Optional, Any, Tuple

def yaw_from_pose(pose) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

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
    CANCELLED = "cancelled"

class NavigationMode(Enum):
    """导航模式枚举"""
    SINGLE_POINT = "single_point"
    MULTI_POINT = "multi_point"
    EXHIBITION_TOUR = "exhibition_tour"
    CHARGING_ROUTE = "charging_route"

class NavigationStateManager(Node):
    """
    导航状态管理器节点 - prior-map 定位版本。

    当前工作区已收口为 ro/RoboSense 与 op/Open3D 两条 prior-map 定位链路；
    状态管理器只认 prior_map_odom_bridge 发布的定位健康状态。
    """

    def __init__(self):
        super().__init__('navigation_state_manager')
        
        # ========== 参数声明 ==========
        self.declare_parameters(namespace='', parameters=[
            ('position_tolerance', 0.15),
            ('orientation_tolerance', 0.3),
            ('status_publish_rate', 2.0),
            ('default_frame_id', 'map'),
            ('obstacle_block_timeout', 4.0),  # 障碍物兜底阻塞超时时间（秒）：主判断交给 Nav2/RPP 失败接管
            ('velocity_threshold', 0.10),  # 判断机器人是否停滞的速度阈值（m/s）
            ('blockage_pose_delta_deadzone', 0.10),  # 速度停滞兜底阈值，降低误把短暂停顿当障碍的概率
            ('blockage_recovery_velocity_threshold', 0.15),  # 解除阻塞需要更明确的持续运动
            ('blockage_recovery_confirm_sec', 1.0),
            ('obstacle_wait_enable', True),
            ('obstacle_wait_push_interval_sec', 4.0),
            ('obstacle_clear_required_frames', 5),
            ('obstacle_clear_check_rate_hz', 5.0),
            ('obstacle_clear_cost_threshold', 100),  # /local_costmap/costmap 是 OccupancyGrid，致命障碍通常为 100。
            ('obstacle_clear_front_min_x_m', 0.15),
            ('obstacle_clear_front_max_x_m', 0.80),  # 障碍恢复只看机器人近前方，降低墙/玻璃门误判。
            ('obstacle_clear_half_width_m', 0.30),  # 左右各 0.30m；这是状态机 clear 窗口，不是 RPP 碰撞参数。
            ('local_costmap_topic', '/local_costmap/costmap'),
            ('require_walk_mode_for_navigation', True),
            ('robot_status_timeout', 2.0),
            ('pending_navigation_timeout', 90.0),
            ('obstacle_block_near_goal_distance', 0.7),
            ('localization_resume_stable_frames', 3),
            ('localization_health_status_topic', '/localization/prior_map_odom_bridge_status'),
            ('map_status_topic', '/map/status'),
            ('localization_health_timeout_sec', 3.0),
            ('localization_allow_start_with_last_good_tf', True),
            ('localization_last_good_tf_max_age_sec', 0.0),
            ('route_task.first_task_reached_tolerance_m', 0.4),
            ('route_task.transit_passed_tolerance_m', 0.5),
            ('route_task.transit_projection_passed_enabled', True),
            ('route_task.nav2_feedback_timeout_sec', 3.0),
            ('route_task.goal_cancel_timeout_sec', 2.0),
            ('route_task.goal_reject_retry_timeout_sec', 8.0),
            ('route_task.default_interrupt_broadcast', True),
            ('map_frame', 'map'),
            ('base_frame', 'base_footprint'),
            ('pose_tf_timeout_sec', 0.05),
            (
                'reverse_navigation_bt_xml',
                '/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/config/behavior_tree/'
                'navigate_reverse_xy_then_yaw.xml'
            )
        ])

        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.status_publish_rate = self.get_parameter('status_publish_rate').value
        self.default_frame_id = self.get_parameter('default_frame_id').value
        self.obstacle_block_timeout = self.get_parameter('obstacle_block_timeout').value
        self.velocity_threshold = self.get_parameter('velocity_threshold').value
        self.blockage_pose_delta_deadzone = float(self.get_parameter('blockage_pose_delta_deadzone').value)
        self.blockage_recovery_velocity_threshold = float(
            self.get_parameter('blockage_recovery_velocity_threshold').value)
        self.blockage_recovery_confirm_sec = float(
            self.get_parameter('blockage_recovery_confirm_sec').value)
        self.obstacle_wait_enable = bool(self.get_parameter('obstacle_wait_enable').value)
        self.obstacle_wait_push_interval_sec = float(
            self.get_parameter('obstacle_wait_push_interval_sec').value)
        self.obstacle_clear_required_frames = max(
            1, int(self.get_parameter('obstacle_clear_required_frames').value))
        self.obstacle_clear_check_rate_hz = max(
            1.0, float(self.get_parameter('obstacle_clear_check_rate_hz').value))
        self.obstacle_clear_cost_threshold = int(
            self.get_parameter('obstacle_clear_cost_threshold').value)
        self.obstacle_clear_front_min_x_m = float(
            self.get_parameter('obstacle_clear_front_min_x_m').value)
        self.obstacle_clear_front_max_x_m = float(
            self.get_parameter('obstacle_clear_front_max_x_m').value)
        self.obstacle_clear_half_width_m = float(
            self.get_parameter('obstacle_clear_half_width_m').value)
        self.local_costmap_topic = str(self.get_parameter('local_costmap_topic').value)
        self.require_walk_mode_for_navigation = self.get_parameter('require_walk_mode_for_navigation').value
        self.robot_status_timeout = float(self.get_parameter('robot_status_timeout').value)
        self.pending_navigation_timeout = float(self.get_parameter('pending_navigation_timeout').value)
        self.obstacle_block_near_goal_distance = float(self.get_parameter('obstacle_block_near_goal_distance').value)
        self.localization_resume_stable_frames = int(
            self.get_parameter('localization_resume_stable_frames').value)
        self.localization_health_status_topic = str(
            self.get_parameter('localization_health_status_topic').value)
        self.localization_health_timeout_sec = float(
            self.get_parameter('localization_health_timeout_sec').value)
        self.localization_allow_start_with_last_good_tf = bool(
            self.get_parameter('localization_allow_start_with_last_good_tf').value)
        self.localization_last_good_tf_max_age_sec = float(
            self.get_parameter('localization_last_good_tf_max_age_sec').value)
        self.reverse_navigation_bt_xml = str(self.get_parameter('reverse_navigation_bt_xml').value)
        self.route_task_first_task_reached_tolerance_m = float(
            self.get_parameter('route_task.first_task_reached_tolerance_m').value)
        self.route_task_transit_passed_tolerance_m = float(
            self.get_parameter('route_task.transit_passed_tolerance_m').value)
        self.route_task_transit_projection_passed_enabled = bool(
            self.get_parameter('route_task.transit_projection_passed_enabled').value)
        self.route_task_nav2_feedback_timeout_sec = float(
            self.get_parameter('route_task.nav2_feedback_timeout_sec').value)
        self.route_task_goal_cancel_timeout_sec = float(
            self.get_parameter('route_task.goal_cancel_timeout_sec').value)
        self.route_task_goal_reject_retry_timeout_sec = float(
            self.get_parameter('route_task.goal_reject_retry_timeout_sec').value)
        self.route_task_default_interrupt_broadcast = bool(
            self.get_parameter('route_task.default_interrupt_broadcast').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.pose_tf_timeout_sec = float(self.get_parameter('pose_tf_timeout_sec').value)
        
        # ========== 导航状态 ==========
        self.current_state = NavigationState.IDLE
        self.current_detailed_state = "IDLE"
        self.current_navigation_mode = None
        self.current_waypoint_index = 0
        self.total_waypoints = 0
        self.current_waypoint = None
        self.waypoint_arrived_by_position = False
        self.pause_time = 0
        self.pause_duration_limit = 0
        self.current_pause_source = ""
        self.current_pause_reason = ""
        self.current_resume_mode = ""
        
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
        self.block_recovery_candidate_start_time = None
        self.block_recovery_candidate_source = ""

        # ========== 动态障碍物等待恢复状态 ==========
        # 单独维护“因障碍暂停”的内部状态，避免和 APP/用户手动暂停混在一起。
        self.obstacle_wait_active = False
        self.obstacle_wait_started_at = 0.0
        self.obstacle_wait_last_push_time = 0.0
        self.obstacle_clear_confirm_count = 0
        self.latest_front_obstacle_blocked = False
        self.latest_front_obstacle_stats = {}
        self.latest_local_costmap_stamp = 0.0
        
        # ========== 从路点管理器接收的数据 ==========
        self.waypoints_data = {}
        # 多地图点位缓存：key 是 map_id，value 是该地图下按类型分组的点位数据。
        # route_waypoint_ids 模式必须用 map_id + waypoint_id 精确补全，不能全局查同名点。
        self.waypoints_data_by_map = {}
        # 当前缓存点位库的版本号。APP 使用 route_waypoint_ids 启动路线时必须携带同一版本，
        # 否则状态管理器会拒绝启动，避免按旧坐标/旧点位属性补全路线。
        self.current_waypoints_revision = ""
        self.current_waypoints_revisions_by_map = {}
        self.current_waypoints_map_id = ""
        # 当前激活地图由 map_context_manager 维护；路线启动前必须确认地图 ready 且匹配命令 map_id。
        self.active_map_id = ""
        self.map_state = "unknown"
        self.map_localization_state = "unknown"
        self.last_map_status_update = 0.0
        self.navigation_start_time = 0
        self.current_goal_pose = None

        # Route task 运行态字段。
        # 这些字段只服务新路线任务，不改变旧单点、多点、展厅导航的状态语义。
        # 后续 through feedback、jump、broadcast_finished 都必须通过这里的状态做隔离。
        self.active_route_task = None
        self.master_route_task_ids = []
        self.completed_task_ids = []
        self.skipped_task_ids = []
        self.current_anchor_task_id = ""
        self.current_anchor_task_index = -1
        self.current_target_task_id = ""
        self.current_target_task_index = -1
        self.active_segment = None
        self.awaiting_broadcast = False
        self.waiting_broadcast_waypoint_id = ""
        self.waiting_broadcast_id = ""
        self.jump_interrupts_broadcast = False
        self.route_task_version = 0
        # route task 事件 ID 的单调计数器。
        # 不在 reset_route_task_state() 中清零，避免同一进程内任务重启后短时间事件 ID 重复。
        self.route_task_event_counter = 0
        self.active_goal_generation = 0
        self.current_route_task_goal_generation = 0
        self.route_task_goal_handle = None
        self.route_task_goal_reject_retry_deadline = 0.0
        self.route_task_goal_reject_retry_count = 0
        self.route_task_goal_reject_retry_segment_id = ""
        # through feedback 最近一次到达时间。
        # 段启动时先置为当前时间，收到 Nav2 feedback 后刷新；周期检查用它识别 action 卡住。
        self.route_task_last_feedback_time = 0.0
        self.last_completed_task_id = ""
        self.last_completed_broadcast = {
            "task_session_id": "",
            "waypoint_id": "",
            "broadcast_id": ""
        }
        
        # ========== Nav2动作客户端 ==========
        # route task 采用“状态管理器分段编排 + 专用行为树”的方式：
        # 1. 有 transit 的段先走 NavigateThroughPoses，辅助点只丝滑通过；
        # 2. 最终 task 再走 NavigateToPose，复用正常/倒走 BT 完成最终 yaw 对齐；
        # 3. 无 transit 的段直接走 NavigateToPose，避免任务点丢失原有的到点后 Spin 对齐。
        # 旧 APP 单点/多点/展台命令仍然下线，这里的 NavigateToPose 只服务新 route task。
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.current_goal_handle = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ========== prior-map 定位健康状态 ==========
        # 状态管理器只监听 prior_map_odom_bridge 的健康状态，用于控制 route task 是否允许启动/恢复；
        # 不再向全局重定位节点发送 recovery_requests，也不再消费 recovery_status。
        self.localization_auto_paused = False
        self.localization_resume_stable_count = 0
        self.last_localization_status_summary = {}
        self.localization_has_last_good_tf = False
        self.localization_last_good_tf_time = 0.0
        
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
        
        # 动态障碍等待、手动暂停等主动取消 goal 的场景，立即补零速度压制底盘残余速度。
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

        # 订阅 local costmap，用前方窗口做“障碍已清除”的连续多帧确认。
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, self.local_costmap_topic, self.local_costmap_callback, 10
        )

        self.robot_status_sub = self.create_subscription(
            String, '/robot_status_raw', self.robot_status_callback, 10
        )
        
        self.nav2_behavior_log_sub = self.create_subscription(BehaviorTreeLog,'/behavior_tree_log',self.nav2_log_callback,10)

        # prior_map_odom_bridge 是新定位链路里唯一维护 map->odom 的节点。
        # 状态管理器不参与点云匹配，也不直接发布 TF；这里只判断导航启动
        # 是否至少有一个可用的 last good map->odom。
        self._is_localization_healthy = False
        self._localization_healthy_count = 0
        self._localization_last_status_time = 0.0
        self._localization_status_sub = self.create_subscription(
            String,
            self.localization_health_status_topic,
            self._on_localization_status,
            10
        )
        self.map_status_sub = self.create_subscription(
            String,
            self.get_parameter('map_status_topic').value,
            self.map_status_callback,
            10
        )
        self.create_timer(1.0, self._check_localization_status_timeout)

    # =========================================================================
    # prior-map 定位健康监听
    # =========================================================================

    def map_status_callback(self, msg: String):
        """接收地图上下文状态。

        APP 可以先发 switch_map，map_context_manager 完成初始位姿注入和定位稳定等待后，
        会把 map_state 置为 ready。路线任务启动必须看到这里的 ready 状态。
        """
        try:
            payload = json.loads(msg.data)
            data = payload.get("data", payload)
            if not isinstance(data, dict):
                return
            self.active_map_id = self.route_task_id(data.get("current_map_id", ""))
            self.map_state = str(data.get("map_state", "unknown") or "unknown")
            self.map_localization_state = str(data.get("localization_state", "unknown") or "unknown")
            self.last_map_status_update = time.time()
        except Exception as exc:
            self.get_logger().warning(f"处理地图状态失败: {exc}")

    def _on_localization_status(self, msg: String):
        """
        监听 prior_map_odom_bridge 的状态字符串。

        bridge 的 TF 策略是“接受好结果，拒绝坏结果，持续发布 last good TF”。
        因此这里不因为单帧 REJECTED 立刻暂停导航：
        - ACCEPTED：bridge 接受了新的 map->odom，刷新 last good TF；
        - REJECTED/PENDING：拒绝当前候选，但如果已有 last good TF，仍允许靠 odom 推进；
        - WAITING no_accepted_map_to_odom：还没有任何可发布的 map->odom，不能启动。
        """
        now = time.time()
        text = msg.data.strip()
        kind = text.split(" ", 1)[0] if text else "UNKNOWN"

        self._localization_last_status_time = now
        self.last_localization_status_summary = {
            "topic": self.localization_health_status_topic,
            "state": kind,
            "text": text,
            "stamp_sec": now,
        }

        if kind == "ACCEPTED":
            self.localization_has_last_good_tf = True
            self.localization_last_good_tf_time = now
            self._is_localization_healthy = True
            self._localization_healthy_count += 1
            self.localization_resume_stable_count += 1
            return

        # SPIN_GUARD 表示 bridge 正在按设计冻结 map->odom，不能说明定位失效；
        # REJECTED/PENDING 表示当前候选未被接受，但 bridge 会继续发布 last good TF。
        # 只要已有 last good TF，就允许导航继续或启动，由 odom 在冻结的 map->odom 下推进。
        if self.localization_can_use_last_good_tf(now):
            self._is_localization_healthy = True
            return

        self._is_localization_healthy = False
        self._localization_healthy_count = 0
        self.localization_resume_stable_count = 0

    def _check_localization_status_timeout(self):
        """prior-map 状态超时后禁止新导航启动；运行中不主动打断 Nav2。"""
        if self._localization_last_status_time <= 0.0:
            return
        age = time.time() - self._localization_last_status_time
        if age > self.localization_health_timeout_sec and self._is_localization_healthy:
            self._is_localization_healthy = False
            self._localization_healthy_count = 0
            self.localization_resume_stable_count = 0
            self.get_logger().warn(
                f'[prior_map] 定位状态超时 {age:.1f}s，暂停新的导航启动')

    def localization_can_use_last_good_tf(self, now: Optional[float] = None) -> bool:
        """已有 last good map->odom 时，允许在 bridge 拒绝坏候选期间靠 odom 推进。"""
        if not self.localization_allow_start_with_last_good_tf:
            return False
        if not self.localization_has_last_good_tf:
            return False

        max_age = max(0.0, self.localization_last_good_tf_max_age_sec)
        if max_age <= 0.0:
            return True

        now = time.time() if now is None else now
        return (now - self.localization_last_good_tf_time) <= max_age

    def setup_timers(self):
        """设置定时器"""
        # 状态发布定时器
        self.create_timer(1.0 / self.status_publish_rate, self.publish_navigation_status)
        
        # 导航检查定时器
        self.create_timer(0.5, self.check_navigation_status)  # 2Hz
        
        # 待启动导航检查定时器
        self.create_timer(0.2, self.try_execute_pending_navigation)

        # 动态障碍物等待恢复定时器：
        # 1. 每 4s 持续给 APP 推等待文案
        # 2. 使用 costmap clear 多帧确认后自动恢复导航
        self.create_timer(1.0 / self.obstacle_clear_check_rate_hz, self.process_obstacle_wait_state)
    
    def navigation_request_callback(self, msg: String):
        """处理路点管理器的导航请求"""
        request_data: Dict[str, Any] = {}
        request_type = ""
        try:
            request_data = json.loads(msg.data)
            if not isinstance(request_data, dict):
                # JSON 能解析但不是对象，说明 APP/桥接层发来的不是协议 payload。
                # 这属于请求格式错误，不应误报 internal_error；APP 可提示升级协议或检查 data_type/request_type。
                self.send_acknowledgment(
                    "navigation_request",
                    "error",
                    "导航请求 payload 必须是 JSON 对象",
                    {
                        "error_code": "invalid_request_payload",
                        "request_type": "",
                    }
                )
                return
            request_type = request_data.get("request_type", "")
            
            self.get_logger().info(f"收到导航请求: {request_type}")
            
            if request_type == "navigation_command":
                self.handle_navigation_command(request_data)
            else:
                self.get_logger().warning(f"未知的请求类型: {request_type}")
                # send_acknowledgment() 的第二个参数是 status，不能把错误文案放进去。
                # 这里按标准旧 ack 结构返回，data_integration 才能包装成 status=error 的 APP 事件。
                self.send_acknowledgment(
                    "unknown_request_type",
                    "error",
                    f"未知请求类型: {request_type}",
                    {
                        "error_code": "unknown_request_type",
                        "request_type": request_type,
                    }
                )
                
        except Exception as e:
            self.get_logger().error(f"处理导航请求错误: {e}")
            self.send_acknowledgment(
                "navigation_request",
                "error",
                f"处理请求失败: {str(e)}",
                {
                    "error_code": "internal_error",
                    # JSON 解析失败时 request_data 可能仍为空，但 request_type 已有稳定默认值。
                    # 这样错误 ack 仍能被 data_integration 包装给 APP，而不会在 except 中二次崩溃。
                    "request_type": request_type,
                }
            )
    
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
            
            # 提取路点数据。当前统一消息是 data.waypoints，旧格式可能直接是 waypoints。
            # 两种都兼容，避免点位管理器/状态管理器版本不一致时 ID 模式误判缓存为空。
            waypoints_data = legacy_data.get("data", {})
            if not isinstance(waypoints_data, dict) or "waypoints" not in waypoints_data:
                waypoints_data = legacy_data
            legacy_metadata = legacy_data.get("metadata", {}) if isinstance(legacy_data.get("metadata", {}), dict) else {}
            self.waypoints_data = waypoints_data.get("waypoints", {})
            self.current_waypoints_map_id = self.route_task_id(
                legacy_data.get("map_id")
                or waypoints_data.get("map_id")
                or legacy_metadata.get("map_id", "")
            )
            incoming_by_map = waypoints_data.get("waypoints_by_map", {})
            if isinstance(incoming_by_map, dict) and incoming_by_map:
                # 新协议：点位管理器一次推送所有地图缓存，状态机按 map_id 精确查点。
                self.waypoints_data_by_map = incoming_by_map
            elif self.current_waypoints_map_id:
                # 兼容过渡期：如果只收到当前地图点位，也挂到对应 map_id 下。
                self.waypoints_data_by_map[self.current_waypoints_map_id] = self.waypoints_data
            elif self.waypoints_data:
                # 旧格式没有 map_id，只能放到 default bucket；新 APP 不应依赖这条路径。
                self.waypoints_data_by_map.setdefault("default", self.waypoints_data)
            self.current_waypoints_revision = self.extract_waypoints_revision(message_data, legacy_data)
            self.current_waypoints_revisions_by_map = self.extract_waypoints_revisions_by_map(message_data, legacy_data)
        
            self.get_logger().info(
                f'收到路点数据更新，共 {self.count_cached_waypoints()} 个点位，'
                f'map_id={self.current_waypoints_map_id or "未提供"}，revision={self.current_waypoints_revision or "未提供"}'
            )
        
        except Exception as e:
            self.get_logger().error(f'❌❌ 处理路点数据错误: {e}')

    def extract_waypoints_revision(self, message_data: Dict[str, Any], legacy_data: Dict[str, Any]) -> str:
        """从点位推送消息中提取点位库版本号，兼容当前和旧格式。

        dynamic_waypoints_manager 会在 data.waypoints_revision 和 data.metadata 中都放一份；
        这里多路径读取是为了兼容联调期间不同节点版本，避免因为包装层差异导致 ID 模式误判缓存未就绪。
        """
        candidates = [
            legacy_data.get("waypoints_revision"),
            legacy_data.get("metadata", {}).get("waypoints_revision")
            if isinstance(legacy_data.get("metadata", {}), dict) else "",
            legacy_data.get("data", {}).get("waypoints_revision")
            if isinstance(legacy_data.get("data", {}), dict) else "",
            legacy_data.get("data", {}).get("metadata", {}).get("waypoints_revision")
            if isinstance(legacy_data.get("data", {}), dict) and isinstance(legacy_data.get("data", {}).get("metadata", {}), dict) else "",
            message_data.get("waypoints_revision"),
            message_data.get("metadata", {}).get("waypoints_revision")
            if isinstance(message_data.get("metadata", {}), dict) else "",
        ]
        for candidate in candidates:
            revision = self.route_task_id(candidate)
            if revision:
                return revision
        return ""

    def extract_waypoints_revisions_by_map(self, message_data: Dict[str, Any], legacy_data: Dict[str, Any]) -> Dict[str, str]:
        """提取每张地图自己的点位库版本号。"""
        candidates = [
            legacy_data.get("waypoints_revisions_by_map"),
            legacy_data.get("metadata", {}).get("waypoints_revisions_by_map")
            if isinstance(legacy_data.get("metadata", {}), dict) else {},
            legacy_data.get("data", {}).get("waypoints_revisions_by_map")
            if isinstance(legacy_data.get("data", {}), dict) else {},
            legacy_data.get("data", {}).get("metadata", {}).get("waypoints_revisions_by_map")
            if isinstance(legacy_data.get("data", {}), dict) and isinstance(legacy_data.get("data", {}).get("metadata", {}), dict) else {},
            message_data.get("waypoints_revisions_by_map"),
            message_data.get("metadata", {}).get("waypoints_revisions_by_map")
            if isinstance(message_data.get("metadata", {}), dict) else {},
        ]
        for candidate in candidates:
            if isinstance(candidate, dict) and candidate:
                return {
                    self.route_task_id(map_id): self.route_task_id(revision)
                    for map_id, revision in candidate.items()
                    if self.route_task_id(map_id)
                }
        return {}

    def count_cached_waypoints(self) -> int:
        """统计缓存中的真实点位数，兼容单地图和多地图结构。"""
        if isinstance(self.waypoints_data_by_map, dict) and self.waypoints_data_by_map:
            return sum(
                self.count_waypoints_in_bucket(map_waypoints)
                for map_waypoints in self.waypoints_data_by_map.values()
                if isinstance(map_waypoints, dict)
            )
        return self.count_waypoints_in_bucket(self.waypoints_data)

    def count_waypoints_in_bucket(self, waypoint_bucket: Dict[str, Any]) -> int:
        """统计一个地图 bucket 中的真实点位数。"""
        count = 0
        for key, value in waypoint_bucket.items():
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

        pose_delta_speed = self.pose_derived_speed
        if pose_delta_speed is not None and pose_delta_speed < self.blockage_pose_delta_deadzone:
            pose_delta_speed = 0.0

        if pose_delta_speed is None:
            return twist_speed, "odom_twist"

        if twist_speed is None or pose_delta_speed >= twist_speed:
            return pose_delta_speed, "pose_delta"

        return twist_speed, "odom_twist"

    def clear_block_recovery_candidate(self):
        self.block_recovery_candidate_start_time = None
        self.block_recovery_candidate_source = ""

    def has_confirmed_blockage_recovery(self, total_velocity: float, velocity_source: str) -> bool:
        if total_velocity < self.blockage_recovery_velocity_threshold:
            self.clear_block_recovery_candidate()
            return False

        now = time.time()
        if self.block_recovery_candidate_start_time is None:
            self.block_recovery_candidate_start_time = now
            self.block_recovery_candidate_source = velocity_source
            return self.blockage_recovery_confirm_sec <= 0.0

        return now - self.block_recovery_candidate_start_time >= self.blockage_recovery_confirm_sec

    def is_stopped_for_blockage(self, total_velocity: float, velocity_source: str) -> bool:
        if velocity_source == "pose_delta":
            return total_velocity < self.blockage_recovery_velocity_threshold
        return total_velocity < self.velocity_threshold

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

    def local_costmap_callback(self, msg: OccupancyGrid):
        """处理 local costmap。

        这里不做 Nav2 那套完整碰撞检查，只做一个稳定、可调的“机器人前方窗口”
        是否仍有高代价障碍物判断，用来支撑“障碍消失后自动恢复导航”。
        """
        try:
            self.latest_local_costmap_stamp = time.time()
            blocked, stats = self.analyze_front_obstacle_window(msg)
            self.latest_front_obstacle_blocked = blocked
            self.latest_front_obstacle_stats = stats
        except Exception as e:
            self.get_logger().error(f'❌ 处理 local costmap 错误: {e}')

    def lookup_robot_pose_in_frame(self, target_frame: str):
        """读取 base_footprint 在指定 frame 下的实时位姿。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.pose_tf_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().debug(
                f'等待 TF {target_frame}->{self.base_frame}: {exc}',
                throttle_duration_sec=1.0,
            )
            return None

        pose_stamped = PoseStamped()
        pose_stamped.header = transform.header
        pose_stamped.pose.position.x = transform.transform.translation.x
        pose_stamped.pose.position.y = transform.transform.translation.y
        pose_stamped.pose.position.z = transform.transform.translation.z
        pose_stamped.pose.orientation = transform.transform.rotation
        return pose_stamped.pose

    def analyze_front_obstacle_window(self, msg: OccupancyGrid) -> Tuple[bool, Dict[str, Any]]:
        """分析机器人前方矩形窗口是否仍被障碍物占据。"""
        robot_pose = self.lookup_robot_pose_in_frame(msg.header.frame_id)
        if robot_pose is None:
            return True, {
                "available": False,
                "reason": f"missing_tf:{msg.header.frame_id}->{self.base_frame}",
            }

        resolution = float(msg.info.resolution)
        width = int(msg.info.width)
        height = int(msg.info.height)
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)
        robot_x = float(robot_pose.position.x)
        robot_y = float(robot_pose.position.y)
        robot_yaw = yaw_from_pose(robot_pose)

        occupied_cells = 0
        max_cost = 0
        sample_cells = 0

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        for cell_y in range(height):
            world_y = origin_y + (cell_y + 0.5) * resolution
            for cell_x in range(width):
                world_x = origin_x + (cell_x + 0.5) * resolution

                dx = world_x - robot_x
                dy = world_y - robot_y

                # 把 costmap 中的世界坐标转到“机器人前向坐标系”。
                forward_x = cos_yaw * dx + sin_yaw * dy
                lateral_y = -sin_yaw * dx + cos_yaw * dy

                if forward_x < self.obstacle_clear_front_min_x_m:
                    continue
                if forward_x > self.obstacle_clear_front_max_x_m:
                    continue
                if abs(lateral_y) > self.obstacle_clear_half_width_m:
                    continue

                sample_cells += 1
                idx = cell_y * width + cell_x
                if idx < 0 or idx >= len(msg.data):
                    continue

                cost = int(msg.data[idx])
                if cost > max_cost:
                    max_cost = cost
                if cost >= self.obstacle_clear_cost_threshold:
                    occupied_cells += 1

        blocked = occupied_cells > 0
        stats = {
            "available": True,
            "frame_id": msg.header.frame_id,
            "window_front_min_x_m": round(self.obstacle_clear_front_min_x_m, 3),
            "window_front_max_x_m": round(self.obstacle_clear_front_max_x_m, 3),
            "window_half_width_m": round(self.obstacle_clear_half_width_m, 3),
            "sample_cells": sample_cells,
            "occupied_cells": occupied_cells,
            "max_cost": max_cost,
            "blocked": blocked,
        }
        return blocked, stats

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

    def get_localization_start_block_reason(self) -> Optional[str]:
        """判断 prior-map 定位是否足够健康，能否启动新的 Nav2 导航。"""
        if self._is_localization_healthy:
            return None

        if self._localization_last_status_time <= 0.0:
            return f"尚未收到定位状态 {self.localization_health_status_topic}，暂不启动导航"

        age = time.time() - self._localization_last_status_time
        if age > self.localization_health_timeout_sec:
            return (
                f"定位状态超时 {age:.1f}s "
                f"> {self.localization_health_timeout_sec:.1f}s，暂不启动导航"
            )

        if self.localization_can_use_last_good_tf():
            return None

        if not self.localization_has_last_good_tf:
            return "prior-map bridge 尚未接受过 map->odom，暂不启动导航"

        state = self.last_localization_status_summary.get("state", "UNKNOWN")
        text = self.last_localization_status_summary.get("text", "")
        if not self.localization_allow_start_with_last_good_tf:
            return f"prior-map定位未接受最新结果: {state} {text}".strip()

        last_good_age = time.time() - self.localization_last_good_tf_time
        return (
            f"last good map->odom 已超限 {last_good_age:.1f}s "
            f"> {self.localization_last_good_tf_max_age_sec:.1f}s，暂不启动导航"
        )

    def reject_navigation_start_if_robot_not_ready(self, ack_type: str) -> bool:
        reason = self.get_navigation_start_block_reason()
        if not reason:
            return False

        self.get_logger().warning(f"拒绝启动导航: {reason}")
        self.send_acknowledgment(ack_type, "error", reason)
        self.publish_status_update("navigation_start_rejected", {"reason": reason})
        return True

    def defer_navigation_start_if_robot_not_ready(self, request_data: Dict[str, Any]) -> bool:
        """机器人底层状态暂不可用或未就绪时缓存启动请求，等恢复后自动执行。"""
        reason = self.get_navigation_start_block_reason()
        if not reason:
            return False

        status_fresh = (
            self.last_robot_status_update > 0 and
            time.time() - self.last_robot_status_update <= self.robot_status_timeout
        )
        status_unavailable = not status_fresh
        can_defer = status_fresh and (
            self.robot_motion_busy or self.robot_control_state == "Menu"
        )

        if not (can_defer or status_unavailable):
            self.get_logger().warning(f"拒绝启动导航: {reason}")
            self.send_acknowledgment("navigation_started", "error", reason)
            self.publish_status_update("navigation_start_rejected", {"reason": reason})
            return True

        if self.pending_navigation_request is not None:
            self.get_logger().warning("新的导航请求覆盖上一条待启动导航请求")

        self.pending_navigation_request = json.loads(json.dumps(request_data))
        self.pending_navigation_created_at = time.time()
        if status_unavailable:
            pending_message = f"{reason}。已缓存导航请求，等待底层状态恢复并确认 Walk 后再开始导航。"
        elif self.robot_motion_busy:
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

        reason = self.get_localization_start_block_reason()
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
        self.send_acknowledgment("navigation_pending", "success", "机器人状态已就绪，开始执行待启动导航")
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
            if self.has_confirmed_blockage_recovery(total_velocity, velocity_source):
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

        if self.is_stopped_for_blockage(total_velocity, velocity_source):
            self.clear_block_recovery_candidate()
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
            # 速度正常时也要先确认持续恢复，避免 pose_delta 单帧抖动打断阻塞计时。
            if self.is_blocked_by_obstacle:
                if self.has_confirmed_blockage_recovery(total_velocity, velocity_source):
                    self.get_logger().info(
                        f"✅ 机器人恢复运动，速度: {total_velocity:.4f} m/s ({velocity_source})，重置阻塞检测"
                    )
                    self.reset_block_detection()
                else:
                    block_duration = time.time() - self.block_start_time
                    if block_duration > self.obstacle_block_timeout:
                        self.handle_obstacle_block_timeout(block_duration)

    def get_obstacle_blockage_suppression_reason(self) -> Optional[str]:
        """返回当前是否应暂停障碍物阻塞计时，以及暂停原因。"""
        if self.active_route_task and self.awaiting_broadcast:
            return "route task 等待 APP 播报完成"

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

    def clear_obstacle_wait_state(self):
        """清理“因障碍等待恢复”的内部状态。"""
        self.obstacle_wait_active = False
        self.obstacle_wait_started_at = 0.0
        self.obstacle_wait_last_push_time = 0.0
        self.obstacle_clear_confirm_count = 0

    def build_pause_event_data(
        self,
        pause_source: str,
        reason: str,
        resume_mode: str,
        extra_data: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """统一构造暂停事件字段，确保 APP 能稳定区分暂停来源。"""
        pause_location = None
        if self.current_pose:
            pause_location = {
                "x": self.current_pose.position.x,
                "y": self.current_pose.position.y,
                "z": self.current_pose.position.z,
            }

        event_data = {
            "pause_source": pause_source,
            "reason": reason,
            "resume_mode": resume_mode,
            "waiting_for_obstacle_clear": pause_source == "obstacle_wait",
            "pause_location": pause_location,
            "pause_time": self.pause_time,
            "pause_duration": self.pause_duration_limit,
            "current_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "current_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
        }
        if extra_data:
            event_data.update(extra_data)
        return event_data

    def reset_block_detection(self):
        """重置阻塞检测状态"""
        self.is_blocked_by_obstacle = False
        self.block_start_time = None
        self.block_reported = False
        self.clear_block_recovery_candidate()
        if self.current_detailed_state == "BLOCKED_BY_OBSTACLE":
            self.current_detailed_state = "EXECUTING"

    def publish_obstacle_blocked_event(self, block_duration: float, send_ack: bool = False):
        """向 APP 推送“障碍仍在等待中”的事件。"""
        error_reason = "检测到障碍物，前方路径被挡住"
        event_data = {
            "reason": error_reason,
            "block_duration": round(block_duration, 1),
            "blocked_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "blocked_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "blocked_waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "position": self.current_waypoint.get("position", []) if self.current_waypoint else [],
            "waiting_for_obstacle_clear": True,
            "clear_confirmed_frames": self.obstacle_clear_confirm_count,
            "clear_required_frames": self.obstacle_clear_required_frames,
            "pause_source": "obstacle_wait",
        }
        if self.latest_front_obstacle_stats:
            event_data["front_obstacle_stats"] = self.latest_front_obstacle_stats

        self.publish_status_update("navigation_obstacle_blocked", event_data)
        if send_ack:
            # 首次进入障碍等待时保留一次兼容确认；周期提醒只走 navigation_status，
            # 避免 APP 每 4 秒同时收到“命令结果”和“状态事件”两类提示。
            self.send_acknowledgment("navigation_obstacle_blocked", "error", error_reason)

    def enter_obstacle_wait_state(self, block_duration: float):
        """进入“动态障碍物等待恢复”流程。

        这里不再让 Nav2 自己 Wait 10s 后盲重试，而是：
        1. 复用现有暂停导航流程取消当前 goal；
        2. 对 APP 明确标记 pause_source=obstacle_wait；
        3. 持续监听 costmap，直到前方 clear 多帧后再自动恢复。
        """
        if self.obstacle_wait_active:
            return

        self.block_reported = True
        self.current_state = NavigationState.PAUSED
        self.current_detailed_state = "OBSTACLE_WAITING"
        self.pause_time = time.time()
        self.pause_duration_limit = 0
        self.current_pause_source = "obstacle_wait"
        self.current_pause_reason = "检测到障碍物，前方路径被挡住"
        self.current_resume_mode = "auto"
        self.obstacle_wait_active = True
        self.obstacle_wait_started_at = self.pause_time
        self.obstacle_wait_last_push_time = 0.0
        self.obstacle_clear_confirm_count = 0

        # 进入等待态后重置普通阻塞计时，避免重复触发“刚暂停又马上超时”。
        self.reset_block_detection()

        if self.current_goal_handle:
            if self.active_route_task:
                # route task 的 through goal 不能走普通 cancel_navigation()：
                # 普通 cancel 回调会无条件清空 current_goal_handle，若障碍消失后新段已经恢复启动，
                # 旧 cancel 回调晚到可能误清掉新 goal handle。
                self.cancel_current_route_goal_safely("obstacle_wait")
            else:
                self.cancel_navigation()
        self.publish_zero_cmd_vel()

        pause_event = self.build_pause_event_data(
            pause_source="obstacle_wait",
            reason=self.current_pause_reason,
            resume_mode="auto",
            extra_data={
                "block_duration": round(block_duration, 1),
                "waiting_for_obstacle_clear": True,
                "clear_confirmed_frames": self.obstacle_clear_confirm_count,
                "clear_required_frames": self.obstacle_clear_required_frames,
            },
        )
        self.publish_status_update("navigation_paused", pause_event)
        self.send_acknowledgment("navigation_paused", "success", "导航已因障碍物暂停", pause_event)

        # 进入等待态时立即推一次障碍提示，后续再由定时器按 4s 周期重复推送。
        self.publish_obstacle_blocked_event(block_duration, send_ack=True)
        self.obstacle_wait_last_push_time = time.time()

        self.get_logger().warning(
            f"🚧 检测到前方障碍持续阻塞 {block_duration:.1f}s，已暂停导航，等待障碍物消失后自动恢复"
        )

    def handle_obstacle_block_timeout(self, block_duration: float):
        """处理障碍物阻塞超时：改为进入“暂停等待障碍消失”的恢复流程。"""
        if self.block_reported or not self.obstacle_wait_enable:
            return
        self.enter_obstacle_wait_state(block_duration)

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
        command_data = {}
        command_type = ""
        route_task_command_types = {
            "start_route_task",
            "pause_route_task",
            "resume_route_task",
            "stop_route_task",
            "jump_to_waypoint",
            "broadcast_finished",
        }
        try:
            command_data = request_data.get("command_data", {})
            command_type = command_data.get("command_type", "")
            
            self.get_logger().info(f"执行导航命令: {command_type}")

            # 对 APP 暴露的导航命令现在只保留 route task 新协议。
            # 旧单点/多点/展台/旧暂停继续终止入口已经下线，避免 APP 和 ROS 同时维护两套路线上下文。
            if command_type == "start_route_task":
                self.handle_start_route_task(command_data, request_data)
            elif command_type == "pause_route_task":
                self.handle_pause_route_task(command_data, request_data)
            elif command_type == "resume_route_task":
                self.handle_resume_route_task(command_data, request_data)
            elif command_type == "stop_route_task":
                self.handle_stop_route_task(command_data, request_data)
            elif command_type == "jump_to_waypoint":
                self.handle_jump_to_waypoint(command_data, request_data)
            elif command_type == "broadcast_finished":
                self.handle_broadcast_finished(command_data, request_data)
            else:
                self.get_logger().warning(f"未知的导航命令: {command_type}")
                # 旧 ack 链路中第二个参数是 status，必须明确写成 "error"。
                # 如果把错误文案误放到 status 字段，data_integration 无法按标准错误事件包装给 APP。
                self.send_acknowledgment(
                    "unknown_navigation_command",
                    "error",
                    f"未知命令: {command_type}",
                    {
                        "error_code": "unknown_navigation_command",
                        "command_type": command_type,
                    }
                )
                
        except Exception as e:
            self.get_logger().error(f"执行导航命令错误: {e}")
            if command_type in route_task_command_types:
                # route task 的业务 ack 必须统一走 navigation_command_result。
                # 即使 handler 内部出现未预期异常，也不能退回旧 send_acknowledgment，
                # 否则 APP 会等不到 start/jump/broadcast 对应的业务结果。
                self.send_route_task_ack(command_type, "error", f"执行命令失败: {str(e)}", command_data, error_code="internal_error")
                return
            # 非 route task 的旧命令仍走 /navigation/acknowledgments 兼容链路，
            # 但也必须保持 status=error 和 error_code=internal_error，方便 APP 统一弹窗和埋点。
            self.send_acknowledgment(
                command_type or "navigation_command",
                "error",
                f"执行命令失败: {str(e)}",
                {
                    "error_code": "internal_error",
                    "command_type": command_type,
                }
            )

    def build_route_task_event_id(self, event_type: str, session_id: str = "") -> str:
        """生成 route task 事件 ID，供 APP 去重和日志追踪使用。"""
        self.route_task_event_counter += 1
        # event_id 也属于对外协议追踪字段，必须复用 route_task_id() 的 ID 规则。
        # 不能写成 session_id or ""，否则未来如果 APP/测试数据传入数字 0，
        # 会被 Python truthy/falsy 规则误判为缺失，进而退化成 no_session。
        session_id = self.route_task_id(session_id)
        if not session_id and self.active_route_task:
            session_id = self.route_task_id(self.active_route_task.get("task_session_id", ""))
        # 事件 ID 同时包含 session、事件类型、单调计数和时间戳。
        # 单调计数解决同一毫秒内多个事件可能撞 ID 的问题，时间戳方便人工排查日志。
        return (
            f"route_task_{session_id or 'no_session'}_"
            f"{event_type}_{self.route_task_event_counter}_{int(time.time() * 1000)}"
        )

    def publish_route_task_event(self, event_type: str, event_data: Dict[str, Any]):
        """发布 route task 事件到 /navigation/status。

        data_integration_node_recoverable.py 会把这些离散事件立即推送给 APP。
        如果当前存在 active_route_task，则自动补齐 task_session_id 和 route_id。
        """
        payload = dict(event_data or {})
        # APP 应按 data.event_type 消费这些 route task 业务事件，而不是用顶层 data_type 区分。
        # data_type 只表示这是一条 navigation_status 推送；真正的业务动作由 event_type 决定，
        # 例如 navigation_command_result 表示命令业务 ack，broadcast_requested 表示 APP 需要播报，
        # waypoint_passed / jump_updated / route_task_completed 则用于刷新路线 UI 和进度。
        # event_id 优先使用 payload 中已有的 task_session_id。
        # event_id 用于 APP 去重、埋点和问题复盘；同一 session 下的同类事件可以稳定关联，
        # 也能避免 websocket 重连、立即推送和周期状态刷新混在一起时前端重复弹窗。
        # 这样 start_route_task 早期错误（例如缺 route_id）即使还没建立 active_route_task，
        # 也能生成带 session 的 event_id，方便 APP 和日志系统关联同一次路线任务。
        payload.setdefault("event_id", self.build_route_task_event_id(event_type, payload.get("task_session_id", "")))
        # route task 事件既有 publish_status_update() 的外层 timestamp，也要在 event_data
        # 内补一份业务事件时间。APP 如果只读取 event_data 做排序、去重或问题复盘，
        # 就不必依赖外层包装结构；已有 timestamp 时保留上游显式传入的值。
        payload.setdefault("timestamp", time.time())
        if self.active_route_task:
            payload.setdefault("task_session_id", self.active_route_task.get("task_session_id", ""))
            payload.setdefault("route_id", self.active_route_task.get("route_id", ""))
            payload.setdefault("map_id", self.active_route_task.get("map_id", ""))
        self.publish_status_update(event_type, payload)

    def send_route_task_ack(
        self,
        command_type: str,
        status: str,
        message: str,
        command_data: Optional[Dict[str, Any]] = None,
        error_code: str = "",
        result_reason: str = ""
    ):
        """按最终协议发布 route task 业务 ack。

        websocket 的 command_ack 只代表“收到包”；真正业务是否接受，
        必须由这里发布 navigation_command_result 告诉 APP。
        """
        command_data = command_data or {}
        # APP 收到 status=success 时，只能说明本次命令已被 ROS 业务层接受或完成了同步校验；
        # 后续机器人是否到点、是否需要播报、是否整条路线完成，仍要继续消费
        # broadcast_requested / task_waypoint_completed / route_task_completed 等异步事件。
        # APP 收到 status=error 时，应优先按 error_code 展示推荐文案，并用 request_message_id
        # 对回原始按钮操作；不要再等待同一命令的成功事件，以免 UI 卡在“发送中”。
        # request_message_id 允许缺失，但必须走 route_task_id() 做安全归一化。
        # 否则 None 会变成字符串 "None"，APP 会拿到一个看似有效、实际无法回溯的请求 ID。
        request_message_id = self.route_task_id(command_data.get("request_message_id"))
        if not request_message_id:
            # request_message_id 来源于 APP 外层 message_id。
            # 缺失时协议允许置空，但必须打日志，方便联调时发现 APP 没有携带可回溯的请求 ID。
            self.get_logger().warning(
                f"route task ack 缺少 request_message_id: command_type={command_type}, status={status}"
            )
        event_data = {
            "request_message_id": request_message_id,
            "ack_type": "navigation_command_result",
            "command_type": command_type,
            "task_session_id": self.route_task_id(command_data.get("task_session_id")),
            "route_id": self.route_task_id(command_data.get("route_id")),
            "map_id": self.route_task_id(command_data.get("map_id")) or (
                self.active_route_task.get("map_id", "") if self.active_route_task else ""
            ),
            "status": status,
            "result_reason": result_reason if status == "success" else "",
            "error_code": error_code if status == "error" else "",
            "message": message
        }
        # navigation_command_result 是“本次命令”的业务 ack。
        # 这里不能用 active_route_task 回填缺失的 task_session_id / route_id：
        # 如果 APP 漏传 route_id，却收到带当前 route_id 的 invalid_route_id，
        # 前端和现场日志会误以为自己传了正确路线 ID。路线上下文事件会由
        # publish_route_task_event() 对非 ack 业务事件自动补齐；ack 必须保留原始命令语义。
        event_data["event_id"] = self.build_route_task_event_id(
            "navigation_command_result", event_data.get("task_session_id", "")
        )
        self.publish_route_task_event("navigation_command_result", event_data)

    @staticmethod
    def route_task_id(value: Any) -> str:
        """把 route task 协议中的业务 ID 安全归一化为字符串。

        APP 可能传数字 ID、字符串 ID、None 或带首尾空格的 ID。
        缺失 ID 返回空字符串，表示“没有可比较的业务 ID”，由上层必填校验或上下文匹配逻辑返回具体错误码。
        这里不能使用 `str(value)`，因为 None 会变成伪 ID "None"；
        也不能使用 `value or ""`，因为数字 0 会被误当成缺失。
        """
        if value is None:
            return ""
        return str(value).strip()

    def normalize_route_task_frame_id(self, value: Any) -> str:
        """归一化 route waypoint 的 frame_id。

        frame_id 只决定 PoseStamped 使用哪个坐标系。APP 缺失或传空时，
        使用节点默认 frame_id，避免保存 None 或空字符串导致 Nav2 goal 头部不完整。
        """
        frame_id = self.route_task_id(value)
        return frame_id or self.default_frame_id

    @staticmethod
    def normalize_route_task_position(value: Any) -> Tuple[List[float], str]:
        """归一化 route waypoint 的 position 为 [x, y, z]。

        route task 首版 through goal 必须能直接构造 PoseStamped。
        position 缺失、长度不足或不是有限数字时，启动阶段直接返回 missing_waypoint_pose，
        不等到 NavigateThroughPoses 下发前才失败。
        """
        if isinstance(value, dict):
            raw_position = [value.get("x"), value.get("y"), value.get("z", 0.0)]
        elif isinstance(value, (list, tuple)):
            raw_position = list(value)
        else:
            return [], "position must be an array"

        if len(raw_position) < 2:
            return [], "position must contain at least x and y"

        if len(raw_position) < 3:
            raw_position.append(0.0)

        try:
            position = [float(raw_position[0]), float(raw_position[1]), float(raw_position[2])]
        except (TypeError, ValueError):
            return [], "position values must be numbers"

        if not all(math.isfinite(component) for component in position):
            return [], "position values must be finite numbers"

        return position, ""

    @staticmethod
    def normalize_route_task_orientation(value: Any) -> Tuple[List[float], str]:
        """归一化 route waypoint 的 orientation 为四元数 [x, y, z, w]。

        清单要求首版 orientation 统一四元数，不在同一字段里混传 yaw。
        因此这里不为缺失 orientation 静默填默认值，避免机器人朝向被悄悄改成默认朝向。
        """
        if isinstance(value, dict):
            raw_orientation = [value.get("x"), value.get("y"), value.get("z"), value.get("w")]
        elif isinstance(value, (list, tuple)):
            raw_orientation = list(value)
        else:
            return [], "orientation must be a quaternion array"

        if len(raw_orientation) < 4:
            return [], "orientation must contain x, y, z and w"

        try:
            orientation = [
                float(raw_orientation[0]),
                float(raw_orientation[1]),
                float(raw_orientation[2]),
                float(raw_orientation[3]),
            ]
        except (TypeError, ValueError):
            return [], "orientation values must be numbers"

        if not all(math.isfinite(component) for component in orientation):
            return [], "orientation values must be finite numbers"

        norm = math.sqrt(sum(component * component for component in orientation))
        if norm <= 1e-6:
            return [], "orientation quaternion norm must be greater than zero"

        orientation = [component / norm for component in orientation]
        return orientation, ""

    @staticmethod
    def route_task_bool(value: Any, default: bool = False) -> bool:
        """把 APP/properties 中的布尔配置统一转成 bool。

        APP 正常应发送 JSON boolean，但现场配置或旧数据里可能出现 "true"/"false"/"1"/"0"。
        不能直接使用 bool(value)，因为 Python 中 bool("false") 会得到 True。
        """
        if value is None:
            return default
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"true", "1", "yes", "y", "on"}:
                return True
            if normalized in {"false", "0", "no", "n", "off", ""}:
                return False
        # 无法识别的布尔值回退默认值，由调用方决定是按保守拒绝还是按配置默认继续。
        # 例如 interrupt_broadcast 缺失时可用节点默认值，避免 APP 传入奇怪字符串直接打断流程。
        return default

    def current_navigation_mode_value(self) -> Optional[str]:
        """返回对外发布的导航模式字符串。

        route task 首版内部复用 Nav2 through 和部分旧导航状态字段，
        但对 APP 来说它不是旧的 multi_point 导航。只要 active_route_task 存在，
        状态流和事件流都应明确发布 navigation_mode=route_task，
        避免 APP 把路线任务事件误归类为普通多点导航。
        """
        if self.active_route_task is not None:
            return "route_task"
        return self.current_navigation_mode.value if self.current_navigation_mode else None

    def build_route_task_status_summary(self) -> Dict[str, Any]:
        """构建周期状态中的 route task 上下文摘要。

        事件流负责告诉 APP “刚刚发生了什么”，周期状态负责告诉 APP “现在处在哪”。
        因此这里只在 active_route_task 存在时生成快照，方便 APP/日志面板随时看到
        当前目标 task、当前段、播报等待和已完成/已跳过集合。
        """
        if not self.active_route_task:
            return {}

        # APP 读取周期 route_task 摘要时，应把它当成“当前态快照”：
        # awaiting_broadcast 用于恢复播报等待 UI，active_segment 用于恢复路线高亮，
        # completed/skipped 列表用于展示整体进度；它不是替代离散事件的命令 ack。
        # active_segment 是运行态字典，先浅拷贝，再把内部列表字段拷贝出来，
        # 避免周期状态发布后被后续 jump、feedback 或 reset 修改引用内容。
        active_segment_snapshot = dict(self.active_segment) if self.active_segment else None
        if active_segment_snapshot:
            for list_field in (
                "transit_waypoint_ids",
                "execution_waypoint_ids",
                "passed_transit_waypoint_ids",
            ):
                active_segment_snapshot[list_field] = list(active_segment_snapshot.get(list_field, []))

        return {
            "task_session_id": self.active_route_task.get("task_session_id", ""),
            "route_id": self.active_route_task.get("route_id", ""),
            "map_id": self.active_route_task.get("map_id", ""),
            "route_waypoint_source": self.active_route_task.get("route_waypoint_source", ""),
            "waypoints_revision": self.active_route_task.get("waypoints_revision", ""),
            "current_anchor_task_id": self.current_anchor_task_id,
            "current_anchor_task_index": self.current_anchor_task_index,
            "current_target_task_id": self.current_target_task_id,
            "current_target_task_index": self.current_target_task_index,
            "master_route_task_ids": list(self.master_route_task_ids),
            "completed_task_ids": list(self.completed_task_ids),
            "skipped_task_ids": list(self.skipped_task_ids),
            "awaiting_broadcast": self.awaiting_broadcast,
            "waiting_broadcast_waypoint_id": self.waiting_broadcast_waypoint_id,
            "waiting_broadcast_id": self.waiting_broadcast_id,
            "active_segment": active_segment_snapshot,
            "route_task_version": self.route_task_version,
            "active_goal_generation": self.current_route_task_goal_generation,
            "last_feedback_age_sec": (
                time.time() - self.route_task_last_feedback_time
                if self.route_task_last_feedback_time > 0 else 0
            )
        }

    def normalize_route_task_waypoints(
        self,
        route_waypoints: List[Dict[str, Any]],
        route_map_id: str = ""
    ) -> Tuple[List[Dict[str, Any]], str, str]:
        """归一化 APP 下发的完整 route_waypoints。

        本函数只做 route task 必需的基础清洗：
        1. waypoint_id 统一转字符串；
        2. waypoint_role 从显式字段读取，缺失时 fallback 到 properties.waypoint_role；
        3. task 的播报/停靠字段按显式字段 -> properties -> 默认值归一化；
        4. transit 强制关闭播报和停靠语义；
        5. source_index 记录原数组顺序，后续段重建只能按数组顺序计算。
        """
        normalized_waypoints = []
        task_count = 0
        seen_waypoint_ids = set()
        normalized_route_map_id = self.route_task_id(route_map_id)

        # websocket/桥接层会尽量把 route task 命令转给状态机返回统一业务 ack。
        # 因此这里必须自己兜住 None、字符串、对象等非数组输入，
        # 不能假设上游已经拦截，否则坏包会触发 TypeError 而不是 navigation_command_result。
        if not isinstance(route_waypoints, list):
            return [], "invalid_route_waypoints", "route_waypoints must be an array"

        for index, waypoint in enumerate(route_waypoints):
            if not isinstance(waypoint, dict):
                return [], "invalid_route_waypoints", f"route_waypoints[{index}] must be an object"

            # route_waypoints 内的 ID 也必须和命令 ID 使用同一套归一化规则。
            # 否则 APP 下发 waypoint_id=" 15 "，后续 jump_to_waypoint 传 "15" 时会查不到目标。
            waypoint_id = self.route_task_id(waypoint.get("waypoint_id"))
            if not waypoint_id:
                return [], "invalid_route_waypoints", f"route_waypoints[{index}] missing waypoint_id"
            if waypoint_id in seen_waypoint_ids:
                return [], "duplicate_waypoint_id", f"route_waypoints contains duplicate waypoint_id: {waypoint_id}"
            seen_waypoint_ids.add(waypoint_id)

            properties = waypoint.get("properties", {})
            if not isinstance(properties, dict):
                properties = {}

            waypoint_map_id = self.route_task_id(waypoint.get("map_id") or properties.get("map_id", ""))
            if normalized_route_map_id:
                if waypoint_map_id and waypoint_map_id != normalized_route_map_id:
                    return [], "route_map_mismatch", (
                        f"waypoint {waypoint_id} map_id mismatch: "
                        f"route={normalized_route_map_id}, waypoint={waypoint_map_id}"
                    )
                waypoint_map_id = normalized_route_map_id

            frame_id = self.normalize_route_task_frame_id(waypoint.get("frame_id", properties.get("frame_id", "")))
            position, position_error = self.normalize_route_task_position(waypoint.get("position"))
            if position_error:
                return [], "missing_waypoint_pose", f"waypoint {waypoint_id} invalid position: {position_error}"
            orientation, orientation_error = self.normalize_route_task_orientation(waypoint.get("orientation"))
            if orientation_error:
                return [], "missing_waypoint_pose", f"waypoint {waypoint_id} invalid orientation: {orientation_error}"
            raw_waypoint_role = waypoint.get("waypoint_role", properties.get("waypoint_role", ""))
            # APP/配置里可能出现 "Task"、" task " 这类写法。
            # 状态机内部只认小写 task/transit，避免后续分支反复做大小写兼容判断。
            waypoint_role = str(raw_waypoint_role or "").strip().lower()
            if waypoint_role not in ("task", "transit"):
                return [], "invalid_waypoint_role", f"waypoint {waypoint_id} missing valid waypoint_role"

            normalized = dict(waypoint)
            normalized["waypoint_id"] = waypoint_id
            normalized["map_id"] = waypoint_map_id
            normalized["waypoint_role"] = waypoint_role
            normalized["frame_id"] = frame_id
            normalized["position"] = position
            normalized["orientation"] = orientation
            normalized["source_index"] = index
            normalized["raw_payload"] = dict(waypoint)

            if waypoint_role == "task":
                task_need_broadcast = waypoint.get("need_broadcast", properties.get("need_broadcast", False))
                task_broadcast_id = waypoint.get("broadcast_id", properties.get("broadcast_id", ""))
                task_broadcast_blocking = waypoint.get(
                    "broadcast_blocking",
                    properties.get("broadcast_blocking", True)
                )
                task_stop_and_align = waypoint.get("stop_and_align", properties.get("stop_and_align", True))

                # task 才有播报/停靠语义。字段来源优先级和默认值在这里统一固化，
                # 后续业务流程只读归一化结果，避免不同分支各自解释 properties。
                normalized["need_broadcast"] = self.route_task_bool(task_need_broadcast, False)
                normalized["broadcast_id"] = self.route_task_id(task_broadcast_id)
                if normalized["need_broadcast"] and not normalized["broadcast_id"]:
                    return [], "missing_broadcast_id", f"waypoint {waypoint_id} need_broadcast=true but missing broadcast_id"
                normalized["broadcast_blocking"] = self.route_task_bool(task_broadcast_blocking, True)
                normalized["stop_and_align"] = self.route_task_bool(task_stop_and_align, True)
                task_count += 1
            else:
                # transit 只作为 through 途经点：不播报、不阻塞播报、不停车对齐。
                # 即使 APP 误传 true，也在归一化阶段强制关闭，保护后续状态机语义。
                normalized["need_broadcast"] = False
                normalized["broadcast_id"] = ""
                normalized["broadcast_blocking"] = False
                normalized["stop_and_align"] = False
            normalized_waypoints.append(normalized)

        if task_count == 0:
            return [], "missing_task_waypoints", "route must contain at least one task waypoint"

        return normalized_waypoints, "", ""

    def validate_route_waypoint_source(self, command_data: Dict[str, Any]) -> Tuple[str, str, str]:
        """判断 start_route_task 使用完整快照还是 ID 列表。

        两种输入只能二选一：
        - route_waypoints：APP 已经把完整点位快照下发给 ROS；
        - route_waypoint_ids：APP 只下发有序 ID，ROS 必须用本地点位库补全。
        同时出现时直接拒绝，避免两份路线内容不一致时状态机猜测使用哪一份。
        """
        route_waypoints = command_data.get("route_waypoints")
        route_waypoint_ids = command_data.get("route_waypoint_ids")
        has_route_waypoints = isinstance(route_waypoints, list) and len(route_waypoints) > 0
        has_route_waypoint_ids = isinstance(route_waypoint_ids, list) and len(route_waypoint_ids) > 0

        if has_route_waypoints and has_route_waypoint_ids:
            return "", "ambiguous_route_waypoint_source", "route_waypoints and route_waypoint_ids cannot both be provided"
        if has_route_waypoints:
            return "inline_route_waypoints", "", ""
        if has_route_waypoint_ids:
            return "stored_waypoint_ids", "", ""
        if isinstance(route_waypoint_ids, list):
            return "", "invalid_route_waypoint_ids", "route_waypoint_ids must not be empty"
        return "", "invalid_route_waypoints", "route_waypoints or route_waypoint_ids is required"

    def validate_waypoints_revision_for_id_mode(self, command_data: Dict[str, Any], map_id: str) -> Tuple[str, str]:
        """校验 ID 列表启动使用的点位库版本号。"""
        requested_revision = self.route_task_id(command_data.get("waypoints_revision"))
        command_data["waypoints_revision"] = requested_revision
        normalized_map_id = self.route_task_id(map_id)
        if not requested_revision:
            return "missing_waypoints_revision", "route_waypoint_ids mode requires waypoints_revision"
        if not normalized_map_id:
            return "missing_map_id", "route_waypoint_ids mode requires map_id"

        current_revision = self.current_waypoints_revisions_by_map.get(normalized_map_id, "")
        if not current_revision and normalized_map_id == self.current_waypoints_map_id:
            # 兼容过渡期：旧点位推送没有 revisions_by_map，但当前地图 revision 还可用。
            current_revision = self.current_waypoints_revision
        if not current_revision:
            return "waypoints_cache_not_ready", "waypoints cache revision is not ready"
        if requested_revision != current_revision:
            return (
                "waypoints_revision_mismatch",
                f"waypoints_revision mismatch: map_id={normalized_map_id}, app={requested_revision}, ros={current_revision}"
            )
        return "", ""

    def normalize_route_waypoint_ids(self, route_waypoint_ids: Any) -> Tuple[List[str], str, str]:
        """归一化 APP 下发的有序点位 ID 列表。"""
        if not isinstance(route_waypoint_ids, list):
            return [], "invalid_route_waypoint_ids", "route_waypoint_ids must be an array"

        normalized_ids = []
        seen_ids = set()
        for index, raw_id in enumerate(route_waypoint_ids):
            waypoint_id = self.route_task_id(raw_id)
            if not waypoint_id:
                return [], "invalid_route_waypoint_ids", f"route_waypoint_ids[{index}] is empty"
            if waypoint_id in seen_ids:
                return [], "duplicate_waypoint_id", f"route_waypoint_ids contains duplicate waypoint_id: {waypoint_id}"
            seen_ids.add(waypoint_id)
            normalized_ids.append(waypoint_id)

        if not normalized_ids:
            return [], "invalid_route_waypoint_ids", "route_waypoint_ids must not be empty"
        return normalized_ids, "", ""

    def validate_active_map_ready(self, route_map_id: str) -> Tuple[str, str]:
        """校验当前激活地图是否允许启动路线任务。

        多地图一期要求 APP 在点击“切换地图”时先完成 switch_map。
        start_route_task 这里只做兜底保护：目标地图必须等于 map_context_manager 发布的
        current_map_id，且 map_state 必须 ready。
        """
        normalized_map_id = self.route_task_id(route_map_id)
        if not normalized_map_id:
            return "missing_map_id", "map_id is required"
        if not self.active_map_id:
            return "map_status_not_ready", "map status is not ready; call switch_map/get_current_map first"
        if self.active_map_id != normalized_map_id:
            return (
                "active_map_mismatch",
                f"active map mismatch: active={self.active_map_id}, route={normalized_map_id}"
            )
        if self.map_state != "ready":
            return (
                "map_not_ready",
                f"map is not ready: map_id={normalized_map_id}, map_state={self.map_state}, "
                f"localization_state={self.map_localization_state}"
            )
        return "", ""

    def build_route_waypoints_from_ids(
        self,
        route_waypoint_ids: Any,
        map_id: str
    ) -> Tuple[List[Dict[str, Any]], str, str]:
        """根据 route_waypoint_ids 从本地点位缓存补全完整 route_waypoints。

        注意：这里严格保留 APP 数组顺序，不按 ID 数字排序。辅助点吸收、正反向跳步、
        后续 source_index 计算都依赖这份路线顺序。
        """
        normalized_map_id = self.route_task_id(map_id)
        if not normalized_map_id:
            return [], "missing_map_id", "route_waypoint_ids mode requires map_id"
        map_waypoints = self.waypoints_data_by_map.get(normalized_map_id, {})
        if not isinstance(map_waypoints, dict) or self.count_waypoints_in_bucket(map_waypoints) <= 0:
            return [], "waypoints_cache_not_ready", f"waypoints cache is empty for map_id={normalized_map_id}"

        normalized_ids, error_code, message = self.normalize_route_waypoint_ids(route_waypoint_ids)
        if error_code:
            return [], error_code, message

        route_waypoints = []
        for index, waypoint_id in enumerate(normalized_ids):
            waypoint_data = self.find_waypoint_data_by_id(waypoint_id, normalized_map_id)
            if not isinstance(waypoint_data, dict):
                return [], "waypoint_id_not_found", (
                    f"route_waypoint_ids[{index}] waypoint_id not found: {waypoint_id} in map_id={normalized_map_id}"
                )

            properties = waypoint_data.get("properties", {})
            if not isinstance(properties, dict):
                properties = {}

            # 动态点位库保存字段叫 id/name/type；route task 运行态统一使用 waypoint_id/waypoint_name。
            # 业务属性仍优先放在 properties 中，normalize_route_task_waypoints() 会做最终强校验。
            route_waypoints.append({
                "waypoint_id": waypoint_id,
                "map_id": normalized_map_id,
                "waypoint_name": waypoint_data.get("name", waypoint_id),
                "waypoint_role": waypoint_data.get("waypoint_role", properties.get("waypoint_role", "")),
                "frame_id": waypoint_data.get("frame_id", properties.get("frame_id", self.default_frame_id)),
                "position": waypoint_data.get("position"),
                "orientation": waypoint_data.get("orientation"),
                "need_broadcast": waypoint_data.get("need_broadcast", properties.get("need_broadcast", False)),
                "broadcast_id": waypoint_data.get("broadcast_id", properties.get("broadcast_id", "")),
                "broadcast_text": waypoint_data.get("broadcast_text", properties.get("broadcast_text", "")),
                "broadcast_blocking": waypoint_data.get(
                    "broadcast_blocking",
                    properties.get("broadcast_blocking", True)
                ),
                "stop_and_align": waypoint_data.get("stop_and_align", properties.get("stop_and_align", True)),
                "walk_direction": waypoint_data.get("walk_direction", properties.get("walk_direction", "forward")),
                "properties": dict(properties),
                "raw_stored_waypoint": dict(waypoint_data),
            })

        return route_waypoints, "", ""

    def build_master_route_task_ids(self, route_waypoints: List[Dict[str, Any]]) -> List[str]:
        """提取主任务点 ID 列表。

        只有 waypoint_role=task 的点进入主任务序列；transit 只是 through 途经点，
        不参与 completed_task_ids / skipped_task_ids / 主任务进度计算。
        """
        return [
            waypoint["waypoint_id"]
            for waypoint in route_waypoints
            if waypoint.get("waypoint_role") == "task"
        ]

    def find_route_waypoint_by_id(self, waypoint_id: str) -> Optional[Dict[str, Any]]:
        """从当前 active route 中按字符串 ID 查找路线点。"""
        if not self.active_route_task:
            return None
        waypoint_id = str(waypoint_id)
        for waypoint in self.active_route_task.get("route_waypoints", []):
            if str(waypoint.get("waypoint_id", "")) == waypoint_id:
                return waypoint
        return None

    def build_active_segment_by_indices(
        self,
        start_index_exclusive: int,
        target_index: int,
        anchor_task_id: str,
        target_task_id: str
    ) -> Dict[str, Any]:
        """按 route_waypoints 数组顺序构建当前执行段。

        首段从机器人当前位置出发，但不会创建“当前位置虚拟 waypoint”；
        execution_waypoint_ids 只包含真实 transit 和最终目标 task。
        """
        route_waypoints = self.active_route_task.get("route_waypoints", []) if self.active_route_task else []
        segment_waypoints = route_waypoints[start_index_exclusive + 1:target_index + 1]
        transit_ids = [
            waypoint["waypoint_id"]
            for waypoint in segment_waypoints
            if waypoint.get("waypoint_role") == "transit"
        ]
        execution_ids = transit_ids + [target_task_id]
        return {
            "segment_id": f"seg_{self.route_task_version}_{self.active_goal_generation + 1}",
            "segment_direction": "forward",
            "segment_start_task_id": anchor_task_id,
            "segment_target_task_id": target_task_id,
            "segment_start_source_index": start_index_exclusive,
            "segment_target_source_index": target_index,
            "transit_waypoint_ids": transit_ids,
            "execution_waypoint_ids": execution_ids,
            "passed_transit_waypoint_ids": [],
            "current_segment_progress_index": 0,
            "segment_goal_generation": self.active_goal_generation + 1
        }

    def compute_segment_direction(self, start_index: int, target_index: int) -> str:
        """根据当前进度点和目标点在 route_waypoints 数组中的位置判断段方向。"""
        return "forward" if target_index >= start_index else "backward"

    def collect_route_interval_waypoints(self, start_index: int, target_index: int) -> List[Dict[str, Any]]:
        """按方向收集完整 route 区间内的真实 waypoint。

        start_index 表示当前进度所在 source_index；返回结果不包含 start_index，
        包含 target_index。反向 jump 时返回顺序也按反向执行顺序排列。
        """
        route_waypoints = self.active_route_task.get("route_waypoints", []) if self.active_route_task else []
        if target_index >= start_index:
            return route_waypoints[start_index + 1:target_index + 1]
        return list(reversed(route_waypoints[target_index:start_index]))

    def resolve_current_progress_source_index(self) -> int:
        """解析当前 route task 的 source_index 进度锚点。

        正在等待播报时，机器人已经到达当前 target task，因此进度锚点就是 target task。
        正在 through 执行时，优先使用当前段最后一个已 passed transit；没有 passed transit
        时使用当前段的 segment_start_source_index。首段可能是 -1，表示从机器人当前位置出发。
        """
        if not self.active_segment:
            return -1
        if self.awaiting_broadcast and self.current_target_task_id:
            target = self.find_route_waypoint_by_id(self.current_target_task_id)
            if target:
                return int(target.get("source_index", -1))

        passed_ids = self.active_segment.get("passed_transit_waypoint_ids", [])
        if passed_ids:
            last_passed = self.find_route_waypoint_by_id(passed_ids[-1])
            if last_passed:
                return int(last_passed.get("source_index", self.active_segment.get("segment_start_source_index", -1)))
        return int(self.active_segment.get("segment_start_source_index", -1))

    def resolve_current_progress_anchor_task_id(self) -> str:
        """解析当前进度所属的业务锚点 task。

        jump 重建段时不能盲目把“旧目标 task”当作新段起点：
        1. 如果正在 WAITING_BROADCAST，说明旧目标 task 已经到达，只是等待 APP 播报回执，此时锚点就是当前目标 task；
        2. 如果仍在 through 导航途中，旧目标 task 还没完成，锚点应继续沿用上一已完成/已确认的 task；
        3. 如果没有显式锚点，则退回最后完成 task 或空字符串，避免把未到达的目标写入段起点。
        """
        if self.awaiting_broadcast and self.current_target_task_id:
            return str(self.current_target_task_id)
        if self.current_anchor_task_id:
            return str(self.current_anchor_task_id)
        if self.last_completed_task_id:
            return str(self.last_completed_task_id)
        if self.completed_task_ids:
            return str(self.completed_task_ids[-1])
        return ""

    def resolve_route_task_index(self, task_id: str) -> int:
        """把 task_id 转成 master_route_task_ids 内的索引，找不到时返回 -1。"""
        task_id = str(task_id or "")
        if task_id and task_id in self.master_route_task_ids:
            return self.master_route_task_ids.index(task_id)
        return -1

    def rebuild_segment_from_current_progress(self, target_task_id: str) -> Tuple[Optional[Dict[str, Any]], List[str], str]:
        """基于完整 route 区间重建 jump 后的新 active_segment。

        不能只看旧 active_segment 剩余点，否则目标跨出旧段时会漏收新区间 transit。
        只扣除“当前 active_segment 内已经 passed 的 transit”，不做全局 transit 去重。
        """
        target_task = self.find_route_waypoint_by_id(target_task_id)
        if not target_task:
            return None, [], "target task not found"

        progress_index = self.resolve_current_progress_source_index()
        progress_anchor_task_id = self.resolve_current_progress_anchor_task_id()
        target_index = int(target_task.get("source_index", -1))
        direction = self.compute_segment_direction(progress_index, target_index)
        interval_waypoints = self.collect_route_interval_waypoints(progress_index, target_index)
        # 只有仍在 through 导航途中再次跳步时，才扣除当前段已经实际通过的 transit。
        # 如果已到达 task 并正在等待播报，上一段已经结束；此时反向跳步必须重新吸收
        # 区间内 transit，例如 G -> B 应重新经过 F/E，而不能因为 B -> G 时通过过就去重。
        current_passed_transit = set()
        if self.active_segment and not self.awaiting_broadcast:
            current_passed_transit = set(self.active_segment.get("passed_transit_waypoint_ids", []))

        transit_ids = [
            waypoint["waypoint_id"]
            for waypoint in interval_waypoints
            if waypoint.get("waypoint_role") == "transit"
            and waypoint.get("waypoint_id") not in current_passed_transit
        ]
        execution_ids = transit_ids + [str(target_task_id)]

        skipped_task_ids = []
        for waypoint in interval_waypoints:
            waypoint_id = waypoint.get("waypoint_id", "")
            if waypoint.get("waypoint_role") != "task":
                continue
            if waypoint_id == str(target_task_id):
                continue
            if waypoint_id in self.completed_task_ids:
                continue
            if waypoint_id not in skipped_task_ids:
                skipped_task_ids.append(waypoint_id)

        segment = {
            "segment_id": f"seg_{self.route_task_version}_{self.active_goal_generation + 1}",
            "segment_direction": direction,
            "segment_start_task_id": progress_anchor_task_id,
            "segment_target_task_id": str(target_task_id),
            "segment_start_source_index": progress_index,
            "segment_target_source_index": target_index,
            "transit_waypoint_ids": transit_ids,
            "execution_waypoint_ids": execution_ids,
            "passed_transit_waypoint_ids": [],
            "current_segment_progress_index": 0,
            "segment_goal_generation": self.active_goal_generation + 1
        }
        return segment, skipped_task_ids, ""

    def build_first_active_segment(self) -> Tuple[Optional[Dict[str, Any]], str]:
        """构建首段 active_segment。

        首个 task 是业务目标：如果首个 task 前存在 transit，也会纳入首段 through goal；
        如果首个 task 正好是 route_waypoints[0]，则首段只导航到该 task。
        """
        if not self.active_route_task or not self.master_route_task_ids:
            return None, "route task has no task waypoint"

        first_task_id = self.master_route_task_ids[0]
        first_task = self.find_route_waypoint_by_id(first_task_id)
        if not first_task:
            return None, f"first task {first_task_id} not found"

        return self.build_active_segment_by_indices(
            start_index_exclusive=-1,
            target_index=int(first_task.get("source_index", 0)),
            anchor_task_id="",
            target_task_id=first_task_id
        ), ""

    def build_next_active_segment(self) -> Tuple[Optional[Dict[str, Any]], str]:
        """当前 task 完成后，按主任务顺序构建下一段。"""
        next_task_index = self.current_target_task_index + 1
        if next_task_index >= len(self.master_route_task_ids):
            return None, ""

        anchor_task_id = self.current_target_task_id
        next_task_id = self.master_route_task_ids[next_task_index]
        anchor_task = self.find_route_waypoint_by_id(anchor_task_id)
        next_task = self.find_route_waypoint_by_id(next_task_id)
        if not anchor_task or not next_task:
            return None, "next segment waypoint missing"

        return self.build_active_segment_by_indices(
            start_index_exclusive=int(anchor_task.get("source_index", 0)),
            target_index=int(next_task.get("source_index", 0)),
            anchor_task_id=anchor_task_id,
            target_task_id=next_task_id
        ), ""

    def route_waypoint_to_pose_stamped(self, waypoint_id: str) -> Optional[PoseStamped]:
        """将归一化后的 route waypoint 转换为 NavigateThroughPoses 使用的 PoseStamped。"""
        waypoint = self.find_route_waypoint_by_id(waypoint_id)
        if not waypoint:
            return None
        try:
            # route task 在 start_route_task 阶段已经校验并归一化 frame_id/position/orientation。
            # 这里不要复用旧普通导航的 waypoint_to_pose_stamped()，因为旧函数会给缺失字段填默认值；
            # route task 需要明确使用归一化后的字段，保证 through goal 与启动校验口径一致。
            pose = PoseStamped()
            # Route task 的 through goal 可能在 jump/定位刚恢复/lifecycle 激活边界被 Nav2 延后处理。
            # 使用 stamp=0 请求 TF latest，避免旧时间戳在 map->map_ground 转换时触发 extrapolation。
            pose.header.stamp = Time().to_msg()
            pose.header.frame_id = waypoint.get("frame_id", self.default_frame_id)

            position = waypoint.get("position", [])
            orientation = waypoint.get("orientation", [])

            pose.pose.position.x = float(position[0])
            pose.pose.position.y = float(position[1])
            pose.pose.position.z = float(position[2])
            pose.pose.orientation.x = float(orientation[0])
            pose.pose.orientation.y = float(orientation[1])
            pose.pose.orientation.z = float(orientation[2])
            pose.pose.orientation.w = float(orientation[3])
            return pose
        except Exception as exc:
            self.get_logger().error(f"route waypoint 转 PoseStamped 失败: {waypoint_id}, {exc}")
            return None

    def get_route_waypoint_walk_direction(self, waypoint: Dict[str, Any]) -> str:
        """读取 route waypoint 的行走方向。

        APP/后台可能把行走方向放在顶层 walk_direction，也可能放在 properties 里。
        这里统一兼容旧字段名，只有明确配置 backward/reverse/倒走 时才走倒走 BT；
        其他值全部按 forward 处理，避免误触发倒车。
        """
        properties = waypoint.get("properties", {}) or {}
        direction = (
            waypoint.get("walk_direction")
            or properties.get("walk_direction")
            or properties.get("navigation_direction")
            or properties.get("drive_direction")
            or properties.get("motion_direction")
            or "forward"
        )
        if isinstance(direction, bool):
            return "backward" if direction else "forward"
        normalized = str(direction).strip().lower()
        if normalized in {"backward", "reverse", "back", "倒走", "倒车", "后退"}:
            return "backward"
        return "forward"

    def start_active_segment_final_pose_navigation(
        self,
        ack_command_type: str,
        ack_command_data: Dict[str, Any],
        send_failure_ack: bool,
        detailed_state: str
    ) -> bool:
        """下发当前段最终 task 的 NavigateToPose。

        这个函数只服务 route task 的最终任务点：
        - 普通 task：不设置 behavior_tree，使用 Nav2 默认 navigate_xy_then_yaw.xml；
        - 倒走 task：设置 reverse_navigation_bt_xml，复用倒走 BT；
        - transit：不会调用这里，因此辅助点不会触发 final yaw align。
        """
        target_task = self.find_route_waypoint_by_id(self.current_target_task_id)
        if not target_task:
            return self.reject_active_segment_start(
                ack_command_type,
                ack_command_data,
                "target task missing",
                "target_task_missing",
                send_failure_ack
            )

        pose = self.route_waypoint_to_pose_stamped(self.current_target_task_id)
        if pose is None:
            return self.reject_active_segment_start(
                ack_command_type,
                ack_command_data,
                f"waypoint {self.current_target_task_id} has no valid pose",
                "missing_waypoint_pose",
                send_failure_ack
            )

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            return self.reject_active_segment_start(
                ack_command_type,
                ack_command_data,
                "NavigateToPose action server unavailable",
                "navigation_busy",
                send_failure_ack
            )

        self.active_goal_generation += 1
        self.current_route_task_goal_generation = self.active_goal_generation
        if self.active_segment is not None:
            self.active_segment["segment_goal_generation"] = self.current_route_task_goal_generation

        self.current_state = NavigationState.EXECUTING
        self.current_detailed_state = detailed_state
        self.current_navigation_mode = NavigationMode.MULTI_POINT
        self.navigation_start_time = time.time()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        walk_direction = self.get_route_waypoint_walk_direction(target_task)
        if walk_direction == "backward":
            goal_msg.behavior_tree = self.reverse_navigation_bt_xml

        route_task_version = self.route_task_version
        try:
            # 注意：这里不是恢复旧 APP 单点导航，而是 route task 的“最终 task 收尾动作”。
            # 只有最终 task 会走到这里，transit 辅助点不会触发对齐、播报或停车等待。
            send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(
                lambda future,
                generation=self.current_route_task_goal_generation,
                version=route_task_version: (
                    self.route_task_final_pose_goal_response_callback(future, generation, version)
                )
            )
        except Exception as exc:
            if send_failure_ack:
                self.send_route_task_ack(
                    ack_command_type,
                    "error",
                    f"NavigateToPose send goal failed: {exc}",
                    ack_command_data,
                    error_code="send_goal_failed"
                )
            self.handle_route_task_navigation_failed(
                f"NavigateToPose send goal failed: {exc}",
                failure_code="send_goal_failed"
            )
            return False

        self.publish_route_task_event("final_align_started", {
            "segment_id": self.active_segment.get("segment_id", "") if self.active_segment else "",
            "waypoint_id": self.current_target_task_id,
            "walk_direction": walk_direction,
            "behavior_tree": goal_msg.behavior_tree,
            "align_reason": detailed_state
        })
        self.get_logger().info(
            "route task final pose navigation started: "
            f"segment_id={self.active_segment.get('segment_id', '') if self.active_segment else ''}, "
            f"target={self.current_target_task_id}, walk_direction={walk_direction}, "
            f"generation={self.current_route_task_goal_generation}"
        )
        return True

    def route_task_final_pose_goal_response_callback(self, future, generation: int, route_task_version: int):
        """处理最终 task NavigateToPose goal response，并隔离旧回调。"""
        if (
            route_task_version != self.route_task_version or
            generation != self.current_route_task_goal_generation
        ):
            self.get_logger().info(
                f"忽略旧 final pose goal response: version={route_task_version}, generation={generation}"
            )
            return
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.handle_route_task_navigation_failed(
                    "NavigateToPose goal rejected",
                    failure_code="final_pose_goal_rejected"
                )
                return
            self.route_task_goal_handle = goal_handle
            self.current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result_future,
                result_generation=generation,
                result_version=route_task_version: (
                    self.route_task_final_pose_result_callback(result_future, result_generation, result_version)
                )
            )
        except Exception as exc:
            self.handle_route_task_navigation_failed(
                f"final pose goal response error: {exc}",
                failure_code="final_pose_goal_response_error"
            )

    def route_task_final_pose_result_callback(self, future, generation: int, route_task_version: int):
        """处理最终 task NavigateToPose result。

        NavigateToPose 成功才表示最终 task 已经完成 XY 到达和 yaw 对齐；
        因此只有这里成功后，才允许进入播报等待或 task 完成流程。
        """
        if (
            route_task_version != self.route_task_version or
            generation != self.current_route_task_goal_generation
        ):
            self.get_logger().info(
                f"忽略旧 final pose result: version={route_task_version}, generation={generation}"
            )
            return
        try:
            result = future.result()
            self.clear_route_task_goal_after_terminal_result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.publish_route_task_event("final_align_completed", {
                    "segment_id": self.active_segment.get("segment_id", "") if self.active_segment else "",
                    "waypoint_id": self.current_target_task_id
                })
                self.handle_target_task_arrived()
            elif result.status == GoalStatus.STATUS_CANCELED:
                if self.obstacle_wait_active:
                    self.get_logger().info("NavigateToPose 取消是由障碍等待暂停触发，保留 route task 现场")
                    return
                self.handle_route_task_navigation_failed(
                    "NavigateToPose goal canceled",
                    failure_code="final_pose_goal_canceled"
                )
            else:
                if self.try_enter_obstacle_wait_from_nav_failure("NavigateToPose goal failed"):
                    return
                self.handle_route_task_navigation_failed(
                    "NavigateToPose goal failed",
                    failure_code="final_pose_goal_failed"
                )
        except Exception as exc:
            self.handle_route_task_navigation_failed(
                f"final pose result error: {exc}",
                failure_code="final_pose_result_error"
            )

    def start_active_segment_navigation(
        self,
        ack_command_type: str = "start_route_task",
        ack_command_data: Optional[Dict[str, Any]] = None,
        send_failure_ack: bool = True
    ) -> bool:
        """按当前 active_segment 的内容选择 route task 执行策略。

        有 transit 的段先交给 NavigateThroughPoses，保证辅助点不停车、不 Spin、不播报；
        没有 transit 的段直接交给 NavigateToPose，让任务点继续复用“到 XY 后 Spin 对齐 yaw”的 BT。
        send_failure_ack 只用于命令触发的段启动：start/jump 失败需要业务 ack；
        自动进入下一段时没有新的 APP 命令，不应伪造旧命令 ack。
        """
        ack_command_data = ack_command_data or self.active_route_task or {}
        if not self.active_segment:
            return self.reject_active_segment_start(
                ack_command_type,
                ack_command_data,
                "active segment is empty",
                "invalid_route_waypoints",
                send_failure_ack
            )

        if self.should_complete_active_segment_without_navigation():
            # 即使无需真正下发 Nav2 goal，也要推进 generation，保证 segment_id/回调隔离
            # 与真实 through goal 一样单调变化，避免 APP 复盘时看到重复段号。
            self.active_goal_generation += 1
            self.current_route_task_goal_generation = self.active_goal_generation
            self.active_segment["segment_goal_generation"] = self.current_route_task_goal_generation
            self.complete_active_segment_without_navigation()
            return True

        if not self.active_segment.get("transit_waypoint_ids", []):
            return self.start_active_segment_final_pose_navigation(
                ack_command_type,
                ack_command_data,
                send_failure_ack,
                detailed_state="ROUTE_TASK_FINAL_POSE_NAVIGATING"
            )

        if not self.nav_through_poses_client.wait_for_server(timeout_sec=5.0):
            return self.reject_active_segment_start(
                ack_command_type,
                ack_command_data,
                "NavigateThroughPoses action server unavailable",
                "navigation_busy",
                send_failure_ack
            )

        poses = []
        # 等 action server 可用后再构造 PoseStamped，避免 server/定位等待期间生成的旧 stamp
        # 在 Nav2 内部转换 map->map_ground 时触发 TF extrapolation into the past。
        for waypoint_id in self.active_segment.get("execution_waypoint_ids", []):
            pose = self.route_waypoint_to_pose_stamped(waypoint_id)
            if pose is None:
                return self.reject_active_segment_start(
                    ack_command_type,
                    ack_command_data,
                    f"waypoint {waypoint_id} has no valid pose",
                    "missing_waypoint_pose",
                    send_failure_ack
                )
            poses.append(pose)

        if not poses:
            return self.reject_active_segment_start(
                ack_command_type,
                ack_command_data,
                "active segment has no execution waypoint",
                "invalid_route_waypoints",
                send_failure_ack
            )

        self.active_goal_generation += 1
        self.current_route_task_goal_generation = self.active_goal_generation
        self.active_segment["segment_goal_generation"] = self.current_route_task_goal_generation
        segment_id = self.active_segment.get("segment_id", "")
        if self.route_task_goal_reject_retry_segment_id != segment_id:
            self.route_task_goal_reject_retry_segment_id = segment_id
            self.route_task_goal_reject_retry_deadline = (
                time.time() + max(0.0, self.route_task_goal_reject_retry_timeout_sec)
            )
            self.route_task_goal_reject_retry_count = 0
        # 新 through 段刚下发时先从当前时刻开始计时。
        # 如果 Nav2 action server 接受 goal 后长期没有 feedback，周期检查会按该时间触发失败。
        self.route_task_last_feedback_time = time.time()
        self.current_state = NavigationState.EXECUTING
        self.current_detailed_state = "ROUTE_TASK_SEGMENT_NAVIGATING"
        self.current_navigation_mode = NavigationMode.MULTI_POINT
        self.navigation_start_time = time.time()

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        route_task_version = self.route_task_version
        try:
            # 从这里开始，状态机已经进入 ROUTE_TASK_SEGMENT_NAVIGATING。
            # 如果 send_goal_async 或回调注册阶段抛异常，必须走 route task 失败流程，
            # 由 handle_route_task_navigation_failed() 发布复盘事件并清理 goal/feedback 状态。
            send_goal_future = self.nav_through_poses_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback_msg,
                generation=self.current_route_task_goal_generation,
                version=route_task_version: (
                    self.route_task_through_feedback_callback(feedback_msg, generation, version)
                )
            )
            send_goal_future.add_done_callback(
                lambda future,
                generation=self.current_route_task_goal_generation,
                version=route_task_version: (
                    self.route_task_through_goal_response_callback(future, generation, version)
                )
            )
        except Exception as exc:
            if send_failure_ack:
                # start_route_task / jump_to_waypoint 是 APP 主动发起的命令。
                # 即使后续还会发布 navigation_failed 复盘，也必须先给本次命令返回
                # navigation_command_result(error)，避免 APP 侧业务 ack 等待超时。
                # 自动下一段调用会传 send_failure_ack=False，因此不会伪造新的命令 ack。
                self.send_route_task_ack(
                    ack_command_type,
                    "error",
                    f"NavigateThroughPoses send goal failed: {exc}",
                    ack_command_data,
                    error_code="send_goal_failed"
                )
            self.handle_route_task_navigation_failed(
                f"NavigateThroughPoses send goal failed: {exc}",
                failure_code="send_goal_failed"
            )
            return False
        self.get_logger().info(
            "route task through segment started: "
            f"segment_id={self.active_segment.get('segment_id', '')}, "
            f"target={self.current_target_task_id}, "
            f"generation={self.current_route_task_goal_generation}"
        )
        return True

    def reject_active_segment_start(
        self,
        ack_command_type: str,
        ack_command_data: Dict[str, Any],
        message: str,
        error_code: str,
        send_failure_ack: bool
    ) -> bool:
        """统一处理 active_segment 启动失败。

        命令入口失败时发 navigation_command_result；自动下一段失败时只写日志，
        由调用方进入 route task 失败流程，避免给 APP 推送一个并不存在的新命令 ack。
        """
        if send_failure_ack:
            self.send_route_task_ack(
                ack_command_type,
                "error",
                message,
                ack_command_data,
                error_code=error_code
            )
        self.get_logger().error(
            f"route task active segment start failed: command={ack_command_type}, "
            f"error_code={error_code}, message={message}"
        )
        return False

    def cleanup_route_task_segment_start_failure(self):
        """清理命令触发的 route task 段启动失败状态。

        start_route_task 首段启动失败时，状态机已经创建了 active_route_task / active_segment，
        但 through goal 还没有真正进入可执行状态。这里必须同时清理 route task 专属状态
        和导航大状态，避免 APP 收到错误 ack 后 ROS 仍残留 EXECUTING 或 feedback 计时上下文。
        """
        self.current_route_task_goal_generation = -1
        self.route_task_last_feedback_time = 0.0
        self.current_goal_handle = None
        self.route_task_goal_handle = None
        self.reset_route_task_state()
        self.reset_navigation_state()

    def route_task_through_goal_response_callback(self, future, generation: int, route_task_version: int):
        """处理 NavigateThroughPoses goal response，并用 version + generation 隔离旧回调。"""
        if (
            route_task_version != self.route_task_version or
            generation != self.current_route_task_goal_generation
        ):
            self.get_logger().info(
                f"忽略旧 through goal response: version={route_task_version}, "
                f"generation={generation}"
            )
            return
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                if self.retry_route_task_goal_rejected(generation, route_task_version):
                    return
                self.handle_route_task_navigation_failed(
                    "NavigateThroughPoses goal rejected",
                    failure_code="goal_rejected"
                )
                return
            # goal 真正 accepted 后重置 feedback 计时。
            # 段下发到 accepted 之间可能存在排队/调度延迟，不能把这段时间算成 Nav2 feedback 静默。
            self.route_task_last_feedback_time = time.time()
            self.route_task_goal_handle = goal_handle
            self.current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda result_future,
                result_generation=generation,
                result_version=route_task_version: (
                    self.route_task_through_result_callback(result_future, result_generation, result_version)
                )
            )
        except Exception as exc:
            self.handle_route_task_navigation_failed(
                f"through goal response error: {exc}",
                failure_code="goal_response_error"
            )

    def retry_route_task_goal_rejected(self, generation: int, route_task_version: int) -> bool:
        """Nav2 lifecycle 刚激活边界可能短暂拒绝 through goal，允许同段短时重试。"""
        if (
            route_task_version != self.route_task_version or
            generation != self.current_route_task_goal_generation or
            not self.active_route_task or
            not self.active_segment
        ):
            return False

        now = time.time()
        if now > self.route_task_goal_reject_retry_deadline:
            return False

        self.route_task_goal_reject_retry_count += 1
        retry_count = self.route_task_goal_reject_retry_count
        retry_delay_sec = 0.5
        self.get_logger().warning(
            "NavigateThroughPoses goal rejected，可能是 bt_navigator lifecycle 尚未 active，"
            f"{retry_delay_sec:.1f}s 后重试同一路线段: "
            f"segment_id={self.active_segment.get('segment_id', '')}, retry={retry_count}"
        )

        timer_ref = {}

        def _retry_once():
            timer = timer_ref.get("timer")
            if timer:
                timer.cancel()
            if (
                route_task_version != self.route_task_version or
                generation != self.current_route_task_goal_generation or
                not self.active_route_task or
                not self.active_segment
            ):
                return
            if not self.start_active_segment_navigation(
                ack_command_type="route_task_goal_retry",
                ack_command_data=self.active_route_task,
                send_failure_ack=False
            ):
                self.handle_route_task_navigation_failed(
                    "NavigateThroughPoses retry start failed",
                    failure_code="goal_rejected_retry_start_failed"
                )

        timer_ref["timer"] = self.create_timer(retry_delay_sec, _retry_once)
        return True

    def route_task_through_feedback_callback(self, feedback_msg, generation: int, route_task_version: int):
        """处理 NavigateThroughPoses feedback。

        优先使用 Nav2 through feedback 的 number_of_poses_remaining 推进 transit 进度；
        这里只发布 waypoint_passed，不直接完成 task。
        """
        if (
            route_task_version != self.route_task_version or
            generation != self.current_route_task_goal_generation
        ):
            return
        try:
            self.route_task_last_feedback_time = time.time()
            feedback = feedback_msg.feedback
            self.distance_remaining = float(getattr(feedback, "distance_remaining", self.distance_remaining))
            poses_remaining = getattr(feedback, "number_of_poses_remaining", None)
            if poses_remaining is not None:
                self.resolve_transit_progress_from_feedback(int(poses_remaining))
            else:
                # 部分 Nav2 版本或封装不会在 NavigateThroughPoses feedback 中提供
                # number_of_poses_remaining。此时仍然要靠机器人实时位姿判断 transit 是否已通过，
                # 否则 APP 侧会迟迟收不到 waypoint_passed 进度事件。
                self.resolve_transit_progress_from_pose()
        except Exception as exc:
            self.get_logger().debug(f"处理 through feedback 失败: {exc}", throttle_duration_sec=2.0)

    def clear_route_task_goal_after_terminal_result(self):
        """清理已经进入终态的 route task through goal handle。

        NavigateThroughPoses result 已经返回成功/取消/失败后，这个 goal 不再是活动 goal。
        如果不清理，WAITING_BROADCAST 阶段收到 jump 时可能会尝试取消一个已结束 goal，
        造成多余日志或旧 cancel 回调干扰排查。
        """
        self.current_goal_handle = None
        self.route_task_goal_handle = None
        self.route_task_last_feedback_time = 0.0

    def resolve_transit_progress_from_feedback(self, poses_remaining: int):
        """根据 through feedback 推断当前段 transit passed。

        execution_waypoint_ids = transit... + target_task。
        当 Nav2 告知剩余 poses 减少时，只把已经越过的 transit 标记为 passed；
        最后一个 target task 仍由 result success 进入 handle_target_task_arrived()。
        """
        if not self.active_segment:
            return
        execution_ids = self.active_segment.get("execution_waypoint_ids", [])
        transit_ids = self.active_segment.get("transit_waypoint_ids", [])
        if not execution_ids or not transit_ids:
            return

        passed_pose_count = max(0, min(len(execution_ids), len(execution_ids) - max(0, poses_remaining)))
        passed_transit_count = min(passed_pose_count, len(transit_ids))
        for waypoint_id in transit_ids[:passed_transit_count]:
            self.mark_transit_passed(waypoint_id)

    def resolve_transit_progress_from_pose(self):
        """用当前机器人位姿补偿 through feedback 缺字段时的 transit 进度。

        这个函数只处理当前 active_segment 内的 transit 点：
        1. 先用“机器人到 transit 的水平距离 <= 阈值”判定已通过；
        2. 如果开启 projection fallback，再用“机器人已经沿上一点到 transit 的线段投影越过终点”
           兜底，避免机器人从 transit 附近擦过但没有刚好落在阈值圆内时漏报；
        3. 不会把结果写入全局去重集合，因此反向/跳转后的新段可以重新经过同一个 transit。
        """
        if not self.active_segment or not self.current_pose:
            return

        transit_ids = [str(item) for item in self.active_segment.get("transit_waypoint_ids", [])]
        passed_ids = set(str(item) for item in self.active_segment.get("passed_transit_waypoint_ids", []))
        if not transit_ids:
            return

        for waypoint_id in transit_ids:
            if waypoint_id in passed_ids:
                continue
            if self.is_transit_passed_by_current_pose(waypoint_id):
                self.mark_transit_passed(waypoint_id)
                passed_ids.add(waypoint_id)
                continue

            # transit 是按 active_segment 的执行顺序逐个经过的。
            # 遇到第一个尚未通过且当前位姿也不能确认通过的 transit 后，后续 transit 暂不判断，
            # 避免机器人距离后续点更近时误把中间点跳过去。
            break

    def is_transit_passed_by_current_pose(self, waypoint_id: str) -> bool:
        """判断当前机器人位姿是否已经经过指定 transit 点。"""
        waypoint = self.find_route_waypoint_by_id(waypoint_id)
        waypoint_position = self.waypoint_position_tuple(waypoint)
        if not waypoint_position or not self.current_pose:
            return False

        robot_x = float(self.current_pose.position.x)
        robot_y = float(self.current_pose.position.y)
        dx = robot_x - float(waypoint_position[0])
        dy = robot_y - float(waypoint_position[1])
        distance = math.hypot(dx, dy)
        threshold = max(0.0, float(self.route_task_transit_passed_tolerance_m))

        # 主判定：机器人进入 transit 点附近阈值圆，就认为该 transit 已经过。
        if distance <= threshold:
            return True

        if not self.route_task_transit_projection_passed_enabled:
            return False

        previous_position = self.resolve_previous_execution_position_for_transit(waypoint_id)
        if previous_position is None:
            return False

        seg_dx = float(waypoint_position[0]) - float(previous_position[0])
        seg_dy = float(waypoint_position[1]) - float(previous_position[1])
        seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
        if seg_len_sq <= 1e-6:
            return False

        # 兜底判定：机器人沿“上一执行点 -> 当前 transit”的方向已经越过 transit。
        # 同时要求横向偏离不要太大，避免远处绕行时把 transit 误判为已通过。
        projection = ((robot_x - float(previous_position[0])) * seg_dx +
                      (robot_y - float(previous_position[1])) * seg_dy) / seg_len_sq
        if projection <= 1.0:
            return False

        cross_track_distance = abs(
            (robot_x - float(previous_position[0])) * seg_dy -
            (robot_y - float(previous_position[1])) * seg_dx
        ) / math.sqrt(seg_len_sq)
        return cross_track_distance <= max(threshold * 2.0, threshold + 0.2)

    def resolve_previous_execution_position_for_transit(self, waypoint_id: str) -> Optional[Tuple[float, float, float]]:
        """找到当前 transit 在 active_segment 执行链上的上一个参考点位置。

        第一段从机器人当前位置出发，不会把当前位置塞进 execution_waypoint_ids。
        因此这里优先找 execution_waypoint_ids 中 transit 前一个真实点；
        如果 transit 是本段第一个执行点，再退回 segment_start_source_index 对应的路线点。
        """
        if not self.active_segment:
            return None

        waypoint_id = str(waypoint_id)
        execution_ids = [str(item) for item in self.active_segment.get("execution_waypoint_ids", [])]
        if waypoint_id in execution_ids:
            waypoint_index = execution_ids.index(waypoint_id)
            if waypoint_index > 0:
                previous_waypoint = self.find_route_waypoint_by_id(execution_ids[waypoint_index - 1])
                return self.waypoint_position_tuple(previous_waypoint)

        start_source_index = int(self.active_segment.get("segment_start_source_index", -1))
        route_waypoints = self.active_route_task.get("route_waypoints", []) if self.active_route_task else []
        if 0 <= start_source_index < len(route_waypoints):
            return self.waypoint_position_tuple(route_waypoints[start_source_index])
        return None

    def mark_transit_passed(self, waypoint_id: str):
        """标记当前 active_segment 内的 transit 已通过，并推送 waypoint_passed。"""
        if not self.active_segment:
            return
        waypoint_id = str(waypoint_id)
        passed_ids = self.active_segment.setdefault("passed_transit_waypoint_ids", [])
        if waypoint_id in passed_ids:
            return
        passed_ids.append(waypoint_id)
        self.active_segment["current_segment_progress_index"] = len(passed_ids)
        # APP 收到 waypoint_passed 后，只更新 through 进度和路线高亮。
        # transit 不触发播报、不需要 APP 回命令，也不能被当成 task 完成。
        self.publish_route_task_event("waypoint_passed", {
            "segment_id": self.active_segment.get("segment_id", ""),
            "waypoint_id": waypoint_id,
            "waypoint_role": "transit",
            # 事件 payload 使用快照拷贝，避免后续段替换或 reset 清理运行态时
            # 影响 APP 已收到的 transit 通过复盘数据。
            "passed_transit_waypoint_ids": list(passed_ids),
            "current_target_task_id": self.current_target_task_id
        })

    def route_task_through_result_callback(self, future, generation: int, route_task_version: int):
        """处理 NavigateThroughPoses result，成功时进入目标 task 到达流程。"""
        if (
            route_task_version != self.route_task_version or
            generation != self.current_route_task_goal_generation
        ):
            self.get_logger().info(
                f"忽略旧 through result: version={route_task_version}, generation={generation}"
            )
            return
        try:
            result = future.result()
            self.clear_route_task_goal_after_terminal_result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                # through 成功只代表 transit 和最终 task 的 XY 路径已经走完；
                # 最终 task 仍必须进入 NavigateToPose 收尾，让正常/倒走 BT 完成 yaw 对齐。
                # 这样 11/12 这类 transit 不会触发 Spin，只有 segment_target_task_id 会对齐。
                if not self.start_active_segment_final_pose_navigation(
                    "route_task_final_align",
                    self.active_route_task or {},
                    send_failure_ack=False,
                    detailed_state="ROUTE_TASK_FINAL_ALIGNING"
                ) and self.active_route_task is not None:
                    self.handle_route_task_navigation_failed(
                        "final task alignment start failed",
                        failure_code="final_align_start_failed"
                    )
            elif result.status == GoalStatus.STATUS_CANCELED:
                if self.obstacle_wait_active:
                    self.get_logger().info("NavigateThroughPoses 取消是由障碍等待暂停触发，保留 route task 现场")
                    return
                self.handle_route_task_navigation_failed(
                    "NavigateThroughPoses goal canceled",
                    failure_code="goal_canceled"
                )
            else:
                if self.try_enter_obstacle_wait_from_nav_failure("NavigateThroughPoses goal failed"):
                    return
                self.handle_route_task_navigation_failed(
                    "NavigateThroughPoses goal failed",
                    failure_code="goal_failed"
                )
        except Exception as exc:
            self.handle_route_task_navigation_failed(
                f"through result error: {exc}",
                failure_code="result_error"
            )

    def handle_target_task_arrived(self):
        """当前 active_segment 的目标 task 已到达，进入播报或完成流程。"""
        target_task = self.find_route_waypoint_by_id(self.current_target_task_id)
        if not target_task:
            self.handle_route_task_navigation_failed(
                "target task missing",
                failure_code="target_task_missing"
            )
            return

        need_broadcast = bool(target_task.get("need_broadcast", False))
        broadcast_id = str(target_task.get("broadcast_id", ""))
        if need_broadcast:
            self.awaiting_broadcast = True
            self.waiting_broadcast_waypoint_id = self.current_target_task_id
            self.waiting_broadcast_id = broadcast_id
            self.current_detailed_state = "WAITING_BROADCAST"
            # APP 收到 broadcast_requested 后，应进入等待播报/正在播报状态，
            # 根据 broadcast_id 启动播报，并在播报完成后回传 broadcast_finished。
            self.publish_route_task_event("broadcast_requested", {
                "segment_id": self.active_segment.get("segment_id", "") if self.active_segment else "",
                "waypoint_id": self.current_target_task_id,
                "broadcast_id": broadcast_id,
                "current_target_task_id": self.current_target_task_id
            })
            return

        self.finalize_task_waypoint_completion(self.current_target_task_id)

    def finalize_task_waypoint_completion(self, waypoint_id: str):
        """完成一个 task，并自动推进下一段或结束整条路线。"""
        waypoint_id = str(waypoint_id)
        if waypoint_id not in self.completed_task_ids:
            self.completed_task_ids.append(waypoint_id)
        # 反向跳回后，之前被 jump 标记为 skipped 的 task 可能会被重新实际执行。
        # 真实完成优先级高于跳过，避免最终 summary 同时出现“已完成”和“已跳过”。
        if waypoint_id in self.skipped_task_ids:
            self.skipped_task_ids = [
                skipped_task_id
                for skipped_task_id in self.skipped_task_ids
                if skipped_task_id != waypoint_id
            ]
        self.last_completed_task_id = waypoint_id
        self.awaiting_broadcast = False
        self.waiting_broadcast_waypoint_id = ""
        self.waiting_broadcast_id = ""
        # APP 收到 task_waypoint_completed 后，应标记当前 task 已完成，
        # 清理该 task 的播报等待态，并根据 next_target_task_id 刷新下一业务目标。
        self.publish_route_task_event("task_waypoint_completed", {
            "segment_id": self.active_segment.get("segment_id", "") if self.active_segment else "",
            "waypoint_id": waypoint_id,
            # completed_task_ids 是运行态列表，发事件时必须拷贝成快照，
            # 否则下一段推进或 route reset 后会让复盘语义不稳定。
            "completed_task_ids": list(self.completed_task_ids),
            "skipped_task_ids": list(self.skipped_task_ids),
            "next_target_task_id": (
                self.master_route_task_ids[self.current_target_task_index + 1]
                if self.current_target_task_index + 1 < len(self.master_route_task_ids)
                else ""
            )
        })

        next_segment, error_message = self.build_next_active_segment()
        if error_message:
            self.handle_route_task_navigation_failed(
                error_message,
                failure_code="next_segment_build_failed"
            )
            return
        if next_segment is None:
            self.complete_route_task()
            return

        self.current_anchor_task_id = self.current_target_task_id
        self.current_anchor_task_index = self.current_target_task_index
        self.current_target_task_index += 1
        self.current_target_task_id = self.master_route_task_ids[self.current_target_task_index]
        self.active_segment = next_segment
        if not self.start_active_segment_navigation(send_failure_ack=False):
            self.handle_route_task_navigation_failed(
                "next route segment start failed",
                failure_code="next_segment_start_failed"
            )

    def complete_route_task(self):
        """发布 route_task_completed 摘要后，再清理 route task 运行态。"""
        summary = {
            "task_session_id": self.active_route_task.get("task_session_id", "") if self.active_route_task else "",
            "route_id": self.active_route_task.get("route_id", "") if self.active_route_task else "",
            "completed_waypoint_id": self.last_completed_task_id,
            # APP 收到 route_task_completed 后，应把路线 UI 切到完成态，并用下面两个列表
            # 展示最终完成/跳过摘要；这也是为什么事件必须在 reset_route_task_state() 之前发布。
            # 完成摘要要使用列表拷贝，避免后续 reset_route_task_state() 清理运行态时
            # 影响已发布事件的复盘数据结构。
            "completed_task_ids": list(self.completed_task_ids),
            "skipped_task_ids": list(self.skipped_task_ids),
            "completed_at": time.time(),
            "result": "success",
            "summary": {
                "task_count": len(self.master_route_task_ids),
                "completed_count": len(self.completed_task_ids),
                "skipped_count": len(self.skipped_task_ids)
            }
        }
        self.publish_route_task_event("route_task_completed", summary)
        self.reset_route_task_state()
        self.reset_navigation_state()

    def reset_route_task_state(self):
        """清理 route task 专属状态，不影响旧导航字段的 reset_navigation_state 语义。"""
        # route task 结束或失败清理时，必须让当前 through goal generation 立即失效。
        # 否则 goal response / feedback / result 晚到时，可能仍通过 generation 校验，
        # 把旧 goal handle 或旧进度重新写回已经 reset 的状态机。
        self.current_route_task_goal_generation = -1
        self.active_goal_generation = 0
        self.active_route_task = None
        self.master_route_task_ids = []
        self.completed_task_ids = []
        self.skipped_task_ids = []
        self.current_anchor_task_id = ""
        self.current_anchor_task_index = -1
        self.current_target_task_id = ""
        self.current_target_task_index = -1
        self.active_segment = None
        self.awaiting_broadcast = False
        self.waiting_broadcast_waypoint_id = ""
        self.waiting_broadcast_id = ""
        self.jump_interrupts_broadcast = False
        self.route_task_goal_handle = None
        self.route_task_goal_reject_retry_deadline = 0.0
        self.route_task_goal_reject_retry_count = 0
        self.route_task_goal_reject_retry_segment_id = ""
        self.route_task_last_feedback_time = 0.0
        self.last_completed_task_id = ""
        self.last_completed_broadcast = {
            "task_session_id": "",
            "waypoint_id": "",
            "broadcast_id": ""
        }

    def handle_route_task_navigation_failed(self, message: str, failure_code: str = "navigation_failed"):
        """处理 route task 失败，并避免进入普通 waypoint 导航失败策略。

        首版不新增 route_task_failed 事件，仍沿用 navigation_failed 通道通知 APP。
        但不能继续调用普通 handle_navigation_failed()：普通失败策略面向单点 goal，
        这会破坏 route task 的 active_segment / task / transit 语义。
        """
        # route task 失败沿用 navigation_failed 通道，但 payload 必须带完整 route 上下文。
        # APP 可用这些字段展示“哪条路线、哪个目标 task、哪一段失败”，
        # 现场复盘时也能看到当时已经完成/跳过了哪些 task，以及当前段还要执行哪些 waypoint。
        # failure_code 是机器可读分类，APP 可以据此区分 rejected、timeout、canceled 等失败类型；
        # reason/message 保留给人阅读，便于现场排障。
        active_segment = self.active_segment or {}
        failure_context = {
            "reason": message,
            "failure_code": failure_code,
            "route_task": True,
            "task_session_id": self.active_route_task.get("task_session_id", "") if self.active_route_task else "",
            "route_id": self.active_route_task.get("route_id", "") if self.active_route_task else "",
            "current_target_task_id": self.current_target_task_id,
            "segment_id": active_segment.get("segment_id", ""),
            "segment_direction": active_segment.get("segment_direction", ""),
            # 失败复盘字段必须是当时现场快照，不能直接引用 active_segment 内部列表。
            "execution_waypoint_ids": list(active_segment.get("execution_waypoint_ids", [])),
            "passed_transit_waypoint_ids": list(active_segment.get("passed_transit_waypoint_ids", [])),
            "completed_task_ids": list(self.completed_task_ids),
            "skipped_task_ids": list(self.skipped_task_ids),
            "failed_at": time.time()
        }
        # APP 收到 navigation_failed(route_task=true) 后，应退出路线执行中/播报等待 UI，
        # 按 failure_code 选择展示文案，并把 route_id、current_target_task_id 和 segment_id
        # 放进详情或日志，方便现场复盘是哪一段 through 或播报闭环失败。
        self.publish_status_update("navigation_failed", failure_context)
        # route task 失败是 /navigation/status 上的状态事件，不是某个 APP 命令的业务 ack。
        # 这里不能再走旧 /navigation/acknowledgments，否则 data_integration 会额外包装出
        # navigation_command_result(command_type=navigation_failed)，导致 APP 收到重复且语义混乱的失败通知。
        if self.current_goal_handle:
            self.cancel_current_route_goal_safely("failure_cleanup")
        self.reset_route_task_state()
        self.reset_navigation_state()

    def is_current_pose_near_route_waypoint(self, waypoint_id: str, tolerance_m: float) -> bool:
        """判断机器人当前位置是否已经在指定 route waypoint 附近。

        首版只用水平距离做初始 task 到达判断，避免 z 轴噪声导致“明明在点上却不算到达”。
        这个判断只作为 start_route_task 的首 task 快速进入播报/完成流程使用，
        不会把机器人当前位置写成虚拟 waypoint，也不会改变 route_waypoints 的数组顺序。
        """
        if not self.current_pose:
            return False
        waypoint = self.find_route_waypoint_by_id(waypoint_id)
        waypoint_position = self.waypoint_position_tuple(waypoint)
        if waypoint_position is None:
            return False
        dx = float(self.current_pose.position.x) - float(waypoint_position[0])
        dy = float(self.current_pose.position.y) - float(waypoint_position[1])
        return math.hypot(dx, dy) <= max(0.0, float(tolerance_m))

    def should_complete_first_task_without_navigation(self) -> bool:
        """判断首个 task 是否可以不下发 goal、直接进入到达流程。

        只有当前段没有前置 transit 时才允许快速完成首 task：
        如果首 task 前面配置了 transit，仍必须交给 NavigateThroughPoses 执行，
        避免为了“当前位置离 task 很近”而漏发中间 transit 的 waypoint_passed。
        """
        if not self.active_segment or self.current_target_task_index != 0:
            return False
        if self.active_segment.get("transit_waypoint_ids", []):
            return False
        return self.is_current_pose_near_route_waypoint(
            self.current_target_task_id,
            self.route_task_first_task_reached_tolerance_m
        )

    def should_complete_active_segment_without_navigation(self) -> bool:
        """当前段所有执行点都已在当前位置附近时，直接完成段。

        只有 transit 和最终 task 全部满足距离容差才短路，避免漏走真正需要经过的辅助点。
        """
        if not self.active_segment or not self.current_pose:
            return False

        execution_ids = [str(item) for item in self.active_segment.get("execution_waypoint_ids", [])]
        if not execution_ids:
            return False

        target_task_id = str(self.current_target_task_id)
        for waypoint_id in execution_ids:
            tolerance = (
                self.route_task_first_task_reached_tolerance_m
                if waypoint_id == target_task_id
                else self.route_task_transit_passed_tolerance_m
            )
            if not self.is_current_pose_near_route_waypoint(waypoint_id, tolerance):
                return False
        return True

    def complete_active_segment_without_navigation(self):
        """当前段距离上已经满足要求时，跳过 through 并进入最终 task 对齐。

        这里不能直接 handle_target_task_arrived()：
        距离近只说明 XY 已经足够接近，不能证明最终 yaw 已经对齐。
        因此仍然要走一次 NavigateToPose，让正常/倒走 BT 完成最终朝向收尾。
        """
        self.current_state = NavigationState.EXECUTING
        self.current_detailed_state = "ROUTE_TASK_SEGMENT_ALREADY_REACHED"
        self.current_navigation_mode = NavigationMode.MULTI_POINT
        self.navigation_start_time = time.time()

        for waypoint_id in self.active_segment.get("transit_waypoint_ids", []):
            self.mark_transit_passed(str(waypoint_id))

        if not self.start_active_segment_final_pose_navigation(
            "route_task_final_align",
            self.active_route_task or {},
            send_failure_ack=False,
            detailed_state="ROUTE_TASK_FINAL_ALIGNING"
        ) and self.active_route_task is not None:
            self.handle_route_task_navigation_failed(
                "final task alignment start failed",
                failure_code="final_align_start_failed"
            )

    def handle_start_route_task(self, command_data: Dict[str, Any], request_data: Dict[str, Any]):
        """处理 start_route_task 命令，初始化 route task 并启动首段 through 导航。"""
        if self.active_route_task is not None:
            self.send_route_task_ack(
                "start_route_task", "error", "route task already running",
                command_data, error_code="route_task_already_running"
            )
            return

        if self.is_navigation_active():
            self.send_route_task_ack(
                "start_route_task", "error", "navigation is busy",
                command_data, error_code="navigation_busy"
            )
            return

        # route task 启动是后续所有事件和状态摘要的源头。
        # 这里先统一归一化 ID，避免 active_route_task 保存 "None" 或带首尾空格的业务 ID。
        task_session_id = self.route_task_id(command_data.get("task_session_id"))
        route_id = self.route_task_id(command_data.get("route_id"))
        request_message_id = self.route_task_id(command_data.get("request_message_id"))
        map_id = self.route_task_id(command_data.get("map_id"))
        command_data["task_session_id"] = task_session_id
        command_data["route_id"] = route_id
        command_data["request_message_id"] = request_message_id
        command_data["map_id"] = map_id

        # 状态机层做二次必填保护：即使有其它节点绕过 dynamic_waypoints_manager，
        # 也不能让空 task_session_id / route_id 的 route task 进入运行态。
        if not task_session_id:
            self.send_route_task_ack(
                "start_route_task", "error", "task_session_id is required",
                command_data, error_code="missing_task_session_id"
            )
            return
        if not route_id:
            self.send_route_task_ack(
                "start_route_task", "error", "route_id is required",
                command_data, error_code="missing_route_id"
            )
            return
        if not map_id:
            self.send_route_task_ack(
                "start_route_task", "error", "map_id is required",
                command_data, error_code="missing_map_id"
            )
            return
        map_error_code, map_message = self.validate_active_map_ready(map_id)
        if map_error_code:
            self.send_route_task_ack(
                "start_route_task", "error", map_message,
                command_data, error_code=map_error_code
            )
            return

        route_waypoint_source, source_error_code, source_message = self.validate_route_waypoint_source(command_data)
        if source_error_code:
            self.send_route_task_ack(
                "start_route_task",
                "error",
                source_message,
                command_data,
                error_code=source_error_code
            )
            return

        if route_waypoint_source == "stored_waypoint_ids":
            revision_error_code, revision_message = self.validate_waypoints_revision_for_id_mode(command_data, map_id)
            if revision_error_code:
                self.send_route_task_ack(
                    "start_route_task",
                    "error",
                    revision_message,
                    command_data,
                    error_code=revision_error_code
                )
                return
            route_waypoints, id_error_code, id_message = self.build_route_waypoints_from_ids(
                command_data.get("route_waypoint_ids", []),
                map_id
            )
            if id_error_code:
                self.send_route_task_ack(
                    "start_route_task",
                    "error",
                    id_message,
                    command_data,
                    error_code=id_error_code
                )
                return
            # 后续流程只处理完整 route_waypoints 快照；ID 模式补全后也立即冻结，
            # jump_to_waypoint 不再重新查点位库，避免导航中点位变化影响本次任务。
            command_data["route_waypoints"] = route_waypoints
        else:
            route_waypoints = command_data.get("route_waypoints", [])

        normalized_waypoints, error_code, message = self.normalize_route_task_waypoints(route_waypoints, map_id)
        if error_code:
            self.send_route_task_ack("start_route_task", "error", message, command_data, error_code=error_code)
            return

        self.route_task_version += 1
        self.active_goal_generation = 0
        self.current_route_task_goal_generation = 0
        self.active_route_task = {
            "task_session_id": task_session_id,
            "route_id": route_id,
            "map_id": map_id,
            "request_message_id": request_message_id,
            "route_waypoints": normalized_waypoints,
            "route_waypoint_source": route_waypoint_source,
            "route_waypoint_ids": [
                self.route_task_id(item) for item in command_data.get("route_waypoint_ids", [])
            ] if route_waypoint_source == "stored_waypoint_ids" else [],
            "waypoints_revision": self.route_task_id(command_data.get("waypoints_revision")),
            "started_at": time.time(),
            "route_task_version": self.route_task_version
        }
        self.master_route_task_ids = self.build_master_route_task_ids(normalized_waypoints)
        self.completed_task_ids = []
        self.skipped_task_ids = []
        self.current_anchor_task_id = ""
        self.current_anchor_task_index = -1
        self.current_target_task_index = 0
        self.current_target_task_id = self.master_route_task_ids[0]
        self.last_completed_broadcast = {
            "task_session_id": "",
            "waypoint_id": "",
            "broadcast_id": ""
        }

        first_segment, segment_error = self.build_first_active_segment()
        if segment_error:
            self.send_route_task_ack("start_route_task", "error", segment_error, command_data, error_code="invalid_route_waypoints")
            self.reset_route_task_state()
            return
        self.active_segment = first_segment

        if self.should_complete_first_task_without_navigation():
            # 首点近距离命中也属于一个真实执行段；即使不下发 Nav2 goal，
            # 也要占用一次 generation，避免后续 jump/下一段复用 seg_?_1。
            self.active_goal_generation += 1
            self.current_route_task_goal_generation = self.active_goal_generation
            self.active_segment["segment_goal_generation"] = self.current_route_task_goal_generation
            self.current_state = NavigationState.EXECUTING
            self.current_detailed_state = "ROUTE_TASK_FIRST_TASK_ALREADY_REACHED"
            # 首 task 已在附近时不会真正下发 through goal，但 APP 仍会进入 route task 执行流。
            # 这里补齐普通 through 段会设置的运行态字段，保证等待播报期间的
            # navigation_mode 和 navigation_duration 与正常首段执行路径一致。
            self.current_navigation_mode = NavigationMode.MULTI_POINT
            self.navigation_start_time = time.time()
            self.send_route_task_ack(
                "start_route_task", "success", "first route task waypoint already reached",
                command_data, result_reason="first_task_already_reached"
            )
            self.complete_active_segment_without_navigation()
            return

        if not self.start_active_segment_navigation():
            self.cleanup_route_task_segment_start_failure()
            return

        self.send_route_task_ack(
            "start_route_task", "success", "route task accepted and first segment started",
            command_data
        )

    def validate_active_route_task_control(self, command_type: str, command_data: Dict[str, Any]) -> bool:
        """校验 route task 控制命令是否属于当前正在运行的路线任务。

        pause_route_task / resume_route_task / stop_route_task 都是“会改变当前任务状态”的命令，
        所以不能只看按钮动作本身，还必须校验 APP 带来的 task_session_id 和 route_id。
        这样可以避免 APP 重连后发出旧 session 的控制命令，误暂停或终止当前正在跑的新路线。
        """
        if self.active_route_task is None:
            self.send_route_task_ack(
                command_type, "error", "route task is not running",
                command_data, error_code="route_task_not_running"
            )
            return False

        task_session_id = self.route_task_id(command_data.get("task_session_id"))
        route_id = self.route_task_id(command_data.get("route_id"))
        if task_session_id != self.active_route_task.get("task_session_id", ""):
            self.send_route_task_ack(
                command_type, "error", "invalid task session",
                command_data, error_code="invalid_task_session"
            )
            return False
        if route_id != self.active_route_task.get("route_id", ""):
            self.send_route_task_ack(
                command_type, "error", "invalid route id",
                command_data, error_code="invalid_route_id"
            )
            return False
        return True

    def handle_pause_route_task(self, command_data: Dict[str, Any], request_data: Dict[str, Any]):
        """处理 pause_route_task 命令，暂停当前路线任务但保留 route task 上下文。

        普通导航暂停逻辑可能清理或覆盖 route task 的 active_segment。
        这里使用 route task 专属暂停：只安全取消当前 route task goal，保留
        active_route_task / active_segment / completed_task_ids / skipped_task_ids / 播报等待态，
        后续 resume_route_task 可以从当前段继续恢复。
        """
        if not self.validate_active_route_task_control("pause_route_task", command_data):
            return

        if self.current_state == NavigationState.PAUSED:
            self.send_route_task_ack(
                "pause_route_task", "success", "route task already paused",
                command_data, result_reason="route_task_already_paused"
            )
            return

        if self.current_state != NavigationState.EXECUTING and not self.awaiting_broadcast:
            self.send_route_task_ack(
                "pause_route_task", "error", "route task is not executing",
                command_data, error_code="invalid_route_task_state"
            )
            return

        pause_params = command_data.get("pause_parameters", {})
        if not isinstance(pause_params, dict):
            pause_params = {}
        pause_duration = pause_params.get("pause_duration", 0)
        self.current_state = NavigationState.PAUSED
        self.current_detailed_state = "PAUSED"
        self.pause_time = time.time()
        self.pause_duration_limit = pause_duration
        self.current_pause_source = "route_task_user_request"
        self.current_pause_reason = command_data.get("reason", "用户手动暂停路线任务")
        self.current_resume_mode = "manual"
        self.clear_obstacle_wait_state()
        self.reset_block_detection()

        # route task 的当前 goal 必须用专属安全取消，不能复用普通 cancel_navigation()。
        # 普通 cancel 回调可能晚到并清掉新恢复的 goal handle，造成恢复后又被旧回调打断。
        if self.current_goal_handle or self.route_task_goal_handle:
            self.cancel_current_route_goal_safely("route_task_pause")
        self.publish_zero_cmd_vel()

        event_data = self.build_pause_event_data(
            pause_source="route_task_user_request",
            reason=self.current_pause_reason,
            resume_mode="manual",
            extra_data={
                "task_session_id": self.active_route_task.get("task_session_id", ""),
                "route_id": self.active_route_task.get("route_id", ""),
                "current_target_task_id": self.current_target_task_id,
                "segment_id": self.active_segment.get("segment_id", "") if self.active_segment else "",
                "route_task": True,
            },
        )
        self.send_route_task_ack(
            "pause_route_task", "success", "route task paused",
            command_data, result_reason="route_task_paused"
        )
        self.publish_status_update("navigation_paused", event_data)

    def handle_resume_route_task(self, command_data: Dict[str, Any], request_data: Dict[str, Any]):
        """处理 resume_route_task 命令，恢复当前暂停中的路线任务。

        该命令同时覆盖两类场景：
        1. APP 手动暂停后点击继续；
        2. 障碍物等待期间，现场人工清障后 APP 点击继续。

        恢复时不能调用旧 APP 单点命令链路；必须重新走 start_active_segment_navigation()，
        由它根据当前段是否包含 transit 选择 through 或最终 task NavigateToPose 收尾。
        """
        if not self.validate_active_route_task_control("resume_route_task", command_data):
            return

        if self.current_state != NavigationState.PAUSED:
            self.send_route_task_ack(
                "resume_route_task", "error", "route task is not paused",
                command_data, error_code="route_task_not_paused"
            )
            return

        if self.is_localization_resume_blocked():
            self.send_route_task_ack(
                "resume_route_task", "error",
                "localization recovery is active, route task resume is blocked",
                command_data, error_code="localization_resume_blocked"
            )
            return

        if not self.active_segment and not self.awaiting_broadcast:
            self.send_route_task_ack(
                "resume_route_task", "error", "route task segment is missing",
                command_data, error_code="missing_active_segment"
            )
            return

        pause_elapsed = time.time() - self.pause_time if self.pause_time else 0.0
        resume_source = self.current_pause_source or "route_task_user_request"
        if self.obstacle_wait_active:
            # 人工清障后点击继续时，退出障碍等待态，后面重新启动当前 active_segment。
            self.clear_obstacle_wait_state()

        self.current_state = NavigationState.EXECUTING
        self.current_detailed_state = "WAITING_BROADCAST" if self.awaiting_broadcast else "EXECUTING"
        self.current_pause_source = ""
        self.current_pause_reason = ""
        self.current_resume_mode = ""

        event_data = {
            "task_session_id": self.active_route_task.get("task_session_id", ""),
            "route_id": self.active_route_task.get("route_id", ""),
            "route_task": True,
            "current_target_task_id": self.current_target_task_id,
            "segment_id": self.active_segment.get("segment_id", "") if self.active_segment else "",
            "pause_duration_actual": round(pause_elapsed, 1),
            "resume_reason": command_data.get("reason", "route_task_user_request"),
            "resume_source": resume_source,
            "awaiting_broadcast": self.awaiting_broadcast,
            "waiting_broadcast_waypoint_id": self.waiting_broadcast_waypoint_id,
            "waiting_broadcast_id": self.waiting_broadcast_id,
        }

        if self.awaiting_broadcast:
            # 如果暂停发生在“已到点等待 APP 播报”阶段，恢复时不重新发导航 goal；
            # APP 继续完成当前 broadcast_finished 闭环即可。
            self.send_route_task_ack(
                "resume_route_task", "success", "route task broadcast wait resumed",
                command_data, result_reason="route_task_broadcast_wait_resumed"
            )
            self.publish_status_update("navigation_resumed", event_data)
            return

        if not self.start_active_segment_navigation(
            ack_command_type="resume_route_task",
            ack_command_data=command_data,
            send_failure_ack=False,
        ):
            self.handle_route_task_navigation_failed(
                "route task manual resume failed",
                failure_code="route_task_resume_failed",
            )
            return

        self.send_route_task_ack(
            "resume_route_task", "success", "route task resumed",
            command_data, result_reason="route_task_resumed"
        )
        self.publish_status_update("navigation_resumed", event_data)

    def handle_stop_route_task(self, command_data: Dict[str, Any], request_data: Dict[str, Any]):
        """处理 stop_route_task 命令，终止当前路线任务并清理 route task 状态。

        终止路线任务不能只做普通取消：除了取消 Nav2 goal，还要清理
        active_route_task、active_segment、播报等待态、goal generation 等专属状态。
        这样终止后重新 start_route_task 不会继承上一条路线的残留上下文。
        """
        if not self.validate_active_route_task_control("stop_route_task", command_data):
            return

        stop_params = command_data.get("stop_parameters", {})
        if not isinstance(stop_params, dict):
            stop_params = {}
        emergency_stop = self.route_task_bool(stop_params.get("emergency_stop", False), False)
        stop_reason = command_data.get("reason") or stop_params.get("reason", "route_task_user_stop")
        completed_task_ids = list(self.completed_task_ids)
        skipped_task_ids = list(self.skipped_task_ids)
        active_segment_snapshot = dict(self.active_segment) if self.active_segment else {}

        if self.current_goal_handle or self.route_task_goal_handle:
            self.cancel_current_route_goal_safely("route_task_stop")
        self.publish_zero_cmd_vel()

        event_data = {
            "task_session_id": self.active_route_task.get("task_session_id", ""),
            "route_id": self.active_route_task.get("route_id", ""),
            "route_task": True,
            "reason": stop_reason,
            "emergency_stop": emergency_stop,
            "current_target_task_id": self.current_target_task_id,
            "segment_id": active_segment_snapshot.get("segment_id", ""),
            "completed_task_ids": completed_task_ids,
            "skipped_task_ids": skipped_task_ids,
            "completed_count": len(completed_task_ids),
            "skipped_count": len(skipped_task_ids),
            "task_count": len(self.master_route_task_ids),
            "stopped_at": time.time(),
        }
        self.current_state = NavigationState.CANCELLED
        self.current_detailed_state = "CANCELLED"
        self.send_route_task_ack(
            "stop_route_task", "success", "route task stopped",
            command_data, result_reason="route_task_stopped"
        )
        self.publish_status_update("navigation_stopped", event_data)
        self.reset_route_task_state()
        self.reset_navigation_state()

    def cancel_current_route_goal_safely(self, reason: str = "route_task"):
        """安全取消当前 route task goal。

        route task 不能直接复用普通 cancel_navigation()：
        普通 cancel 回调会无条件清理 current_goal_handle，若 APP 很快启动新路线，
        旧 cancel 回调可能晚于新 goal response 返回，从而把新 goal handle 误置空。
        这里捕获“被取消的旧 goal handle”，回调里只允许清理同一个旧对象，
        用于 jump 重规划、route task 失败清理等所有 route task 专属取消场景。
        """
        goal_handle = self.current_goal_handle
        cancel_generation = self.current_route_task_goal_generation
        if not goal_handle:
            self.get_logger().info(f"route task {reason} 无旧 goal 需要取消")
            return

        # 先让旧 result callback 失效；jump 场景下随后 start_active_segment_navigation 会写入新的 generation。
        # 失败清理场景下 reset_route_task_state() 会清空运行态，新旧回调也都不能再推进任务。
        self.current_route_task_goal_generation = -1
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda cancel_future, old_goal_handle=goal_handle, old_generation=cancel_generation: (
                    self.route_task_cancel_callback(cancel_future, old_goal_handle, old_generation)
                )
            )
            self.get_logger().info(
                f"route task {reason} 已发送旧 goal 取消请求: generation={cancel_generation}"
            )
        except Exception as exc:
            self.get_logger().warning(f"route task {reason} 取消旧 goal 失败: {exc}")

    def cancel_current_route_goal_for_replan(self):
        """为 route task jump 重规划取消旧 goal。

        这个包装函数保留 jump 语义，实际取消逻辑统一走 cancel_current_route_goal_safely()。
        """
        self.cancel_current_route_goal_safely("jump_replan")

    def route_task_cancel_callback(self, future, old_goal_handle, old_generation: int):
        """处理 route task 专用取消回调，避免旧取消结果污染新 goal。"""
        try:
            response = future.result()
            if response.return_code != CancelGoal.Response.ERROR_NONE:
                self.get_logger().warning(
                    f"route task 旧 goal 取消未被接受: generation={old_generation}, "
                    f"return_code={response.return_code}"
                )
                return

            if self.current_goal_handle is old_goal_handle:
                # 只有当前仍然指向旧 goal 时才清空；如果新 goal 已经写入，这里绝不覆盖。
                self.current_goal_handle = None
                if self.route_task_goal_handle is old_goal_handle:
                    self.route_task_goal_handle = None
            self.get_logger().info(f"route task 旧 goal 已取消: generation={old_generation}")
        except Exception as exc:
            self.get_logger().warning(f"route task 旧 goal 取消回调处理失败: {exc}")

    def handle_jump_to_waypoint(self, command_data: Dict[str, Any], request_data: Dict[str, Any]):
        """处理 jump_to_waypoint 命令。

        所有合法性校验和新段重建都先在本地变量中完成；
        只有确认合法后才会取消旧 goal、清理等待播报状态并替换 active_segment。
        """
        if self.active_route_task is None:
            self.send_route_task_ack(
                "jump_to_waypoint", "error", "route task is not running",
                command_data, error_code="route_task_not_running"
            )
            return

        # route task 命令 ID 统一走 route_task_id()，避免 None 被转成字符串 "None"。
        task_session_id = self.route_task_id(command_data.get("task_session_id"))
        if task_session_id != self.active_route_task.get("task_session_id", ""):
            self.send_route_task_ack(
                "jump_to_waypoint", "error", "invalid task session",
                command_data, error_code="invalid_task_session"
            )
            return
        route_id = self.route_task_id(command_data.get("route_id"))
        if route_id != self.active_route_task.get("route_id", ""):
            self.send_route_task_ack(
                "jump_to_waypoint", "error", "invalid route id",
                command_data, error_code="invalid_route_id"
            )
            return

        target_waypoint_id = self.route_task_id(command_data.get("target_waypoint_id"))
        target_waypoint = self.find_route_waypoint_by_id(target_waypoint_id)
        if not target_waypoint:
            self.send_route_task_ack(
                "jump_to_waypoint", "error", "target waypoint not found",
                command_data, error_code="invalid_target_waypoint"
            )
            return
        if target_waypoint.get("waypoint_role") != "task":
            self.send_route_task_ack(
                "jump_to_waypoint", "error", "target waypoint is not task",
                command_data, error_code="target_waypoint_not_task"
            )
            return

        if target_waypoint_id == self.current_target_task_id:
            self.send_route_task_ack(
                "jump_to_waypoint", "success", "target is already current route task target",
                command_data, result_reason="already_current_target"
            )
            return

        # APP 可能把布尔值传成字符串，例如 "false"。
        # 这里必须走 route_task_bool()，不能直接用 Python bool("false")，
        # 否则字符串 "false" 会被误判为 True，导致本应拒绝的播报等待 jump 被错误放行。
        interrupt_broadcast = self.route_task_bool(
            command_data.get("interrupt_broadcast", self.route_task_default_interrupt_broadcast),
            self.route_task_default_interrupt_broadcast
        )
        if self.awaiting_broadcast and not interrupt_broadcast:
            self.send_route_task_ack(
                "jump_to_waypoint", "error", "interrupt_broadcast=false is not supported",
                command_data, error_code="interrupt_broadcast_false_not_supported"
            )
            return

        new_segment, new_skipped_task_ids, rebuild_error = self.rebuild_segment_from_current_progress(target_waypoint_id)
        if rebuild_error:
            self.send_route_task_ack(
                "jump_to_waypoint", "error", rebuild_error,
                command_data, error_code="invalid_target_waypoint"
            )
            return

        interrupted_broadcast = {}
        if self.awaiting_broadcast:
            interrupted_broadcast = {
                "waypoint_id": self.waiting_broadcast_waypoint_id,
                "broadcast_id": self.waiting_broadcast_id
            }
            interrupted_waypoint_id = self.waiting_broadcast_waypoint_id
            if (
                interrupted_waypoint_id
                and interrupted_waypoint_id != target_waypoint_id
                and interrupted_waypoint_id not in self.completed_task_ids
                and interrupted_waypoint_id not in self.skipped_task_ids
            ):
                # 已到达但尚未收到 broadcast_finished 的 task，被 jump 打断后不能算完成。
                # 先计入 skipped，保证最终 task_count = completed + skipped；
                # 如果后面又跳回该点并完成，finalize_task_waypoint_completion() 会自动移除。
                self.skipped_task_ids.append(interrupted_waypoint_id)
            self.awaiting_broadcast = False
            self.waiting_broadcast_waypoint_id = ""
            self.waiting_broadcast_id = ""

        if self.current_goal_handle:
            self.cancel_current_route_goal_for_replan()

        for skipped_task_id in new_skipped_task_ids:
            if skipped_task_id not in self.skipped_task_ids and skipped_task_id not in self.completed_task_ids:
                self.skipped_task_ids.append(skipped_task_id)
        if target_waypoint_id in self.skipped_task_ids:
            # 用户主动跳回某个之前被跳过的 task 时，该 task 已重新成为当前目标，
            # 不应在 jump_updated 中继续展示为 skipped，避免 APP 同时高亮目标和跳过态。
            self.skipped_task_ids = [
                skipped_task_id
                for skipped_task_id in self.skipped_task_ids
                if skipped_task_id != target_waypoint_id
            ]

        self.current_target_task_id = target_waypoint_id
        self.current_target_task_index = self.master_route_task_ids.index(target_waypoint_id)
        self.active_segment = new_segment
        # jump 后的 anchor 必须来自重建段时解析出的“当前进度锚点”。
        # 这样正在去 15 的路上跳走时不会把未到达的 15 错写为锚点；
        # 但已经在 15 等播报时，锚点会正确记录为 15。
        self.current_anchor_task_id = self.active_segment.get("segment_start_task_id", "")
        self.current_anchor_task_index = self.resolve_route_task_index(self.current_anchor_task_id)
        self.current_detailed_state = "JUMP_REPLANNING"

        jump_event_data = {
            "segment_id": self.active_segment.get("segment_id", ""),
            "target_waypoint_id": target_waypoint_id,
            "segment_direction": self.active_segment.get("segment_direction", ""),
            # jump_updated 是 APP 更新当前执行段 UI 的依据，列表字段同样使用快照，
            # 避免后续继续追加 skipped task 或替换 active_segment 时影响本次事件。
            "execution_waypoint_ids": list(self.active_segment.get("execution_waypoint_ids", [])),
            "skipped_task_ids": list(self.skipped_task_ids),
            "interrupt_broadcast": interrupt_broadcast,
            "interrupted_broadcast": interrupted_broadcast
        }
        will_complete_without_navigation = self.should_complete_active_segment_without_navigation()
        if will_complete_without_navigation:
            # 快捷完成分支会同步调用 handle_target_task_arrived()/finalize_task_waypoint_completion()，
            # 甚至可能一路推进到 route_task_completed 并 reset active_segment。
            # 因此必须先发 jump_updated 和业务 ack，保证 APP 能收到本次 jump 已被接受，
            # 再接收后续 waypoint_passed/task_completed/route_completed 事件。
            self.publish_route_task_event("jump_updated", jump_event_data)
            self.send_route_task_ack(
                "jump_to_waypoint", "success", "jump request accepted",
                command_data
            )

        if not self.start_active_segment_navigation("jump_to_waypoint", command_data):
            # start_active_segment_navigation() 内部可能已经进入 route task 失败流程
            # 并清理 active_route_task，例如 send_goal_async 抛异常时会发布 send_goal_failed。
            # 这里仅在任务仍然存在时补发 jump 语义的失败，避免 APP 收到两份 navigation_failed。
            if self.active_route_task is not None:
                self.handle_route_task_navigation_failed(
                    "jump route segment start failed",
                    failure_code="jump_segment_start_failed"
                )
            return
        if will_complete_without_navigation:
            return

        # 只有新 route goal 已经成功发起后，才发布 jump_updated。
        # 这样 APP 不会先切到新执行段 UI，随后又马上收到新段启动失败事件。
        # APP 收到 jump_updated 后，应切换当前目标 task、高亮新 active_segment，
        # 清理旧目标高亮；若 interrupted_broadcast 非空，还应退出旧播报等待 UI。
        self.publish_route_task_event("jump_updated", jump_event_data)

        self.send_route_task_ack(
            "jump_to_waypoint", "success", "jump request accepted",
            command_data
        )

    def handle_broadcast_finished(self, command_data: Dict[str, Any], request_data: Dict[str, Any]):
        """处理 broadcast_finished 命令，校验播报上下文并推进 task 完成。"""
        if self.active_route_task is None:
            self.send_route_task_ack(
                "broadcast_finished", "error", "route task is not running",
                command_data, error_code="route_task_not_running"
            )
            return
        # 播报回执中的三个 ID 都要和等待播报上下文做精确比较。
        # 统一使用 route_task_id()，可以同时处理数字 ID、首尾空格和 None。
        task_session_id = self.route_task_id(command_data.get("task_session_id"))
        waypoint_id = self.route_task_id(command_data.get("waypoint_id"))
        broadcast_id = self.route_task_id(command_data.get("broadcast_id"))
        route_id = self.route_task_id(command_data.get("route_id"))
        # APP 回传播报结果时可能出现 "Completed" 或 " completed " 这类大小写/空格差异。
        # 这里不能写成 `command_data.get(...) or "completed"`：显式空字符串代表 APP
        # 回传了无效结果，必须返回 unsupported_broadcast_result，不能被误当成缺失字段自动完成。
        raw_broadcast_result = command_data.get("broadcast_result", "completed")
        if raw_broadcast_result is None:
            raw_broadcast_result = "completed"
        broadcast_result = str(raw_broadcast_result).strip().lower()

        # 先单独校验 task_session_id，避免把“会话不匹配”混进 broadcast_context_mismatch。
        # APP 可以据此提示用户刷新路线任务状态，而不是误以为只是播报点位上下文错误。
        if task_session_id != self.active_route_task.get("task_session_id", ""):
            self.send_route_task_ack(
                "broadcast_finished", "error", "invalid task session",
                command_data, error_code="invalid_task_session"
            )
            return

        if route_id != self.active_route_task.get("route_id", ""):
            self.send_route_task_ack(
                "broadcast_finished", "error", "invalid route id",
                command_data, error_code="invalid_route_id"
            )
            return

        if broadcast_result != "completed":
            self.send_route_task_ack(
                "broadcast_finished", "error", "unsupported broadcast result",
                command_data, error_code="unsupported_broadcast_result"
            )
            return

        last_broadcast = self.last_completed_broadcast or {}
        if (
            task_session_id == last_broadcast.get("task_session_id", "") and
            waypoint_id == last_broadcast.get("waypoint_id", "") and
            broadcast_id == last_broadcast.get("broadcast_id", "")
        ):
            self.send_route_task_ack(
                "broadcast_finished", "success", "duplicate broadcast_finished ignored",
                command_data, result_reason="duplicate_broadcast_finished"
            )
            return

        if not self.awaiting_broadcast:
            self.send_route_task_ack(
                "broadcast_finished", "error", "route task is not waiting for broadcast",
                command_data, error_code="broadcast_not_waiting"
            )
            return

        if waypoint_id != self.waiting_broadcast_waypoint_id or broadcast_id != self.waiting_broadcast_id:
            self.send_route_task_ack(
                "broadcast_finished", "error", "broadcast context mismatch",
                command_data, error_code="broadcast_context_mismatch"
            )
            return

        self.last_completed_broadcast = {
            "task_session_id": task_session_id,
            "waypoint_id": waypoint_id,
            "broadcast_id": broadcast_id
        }
        self.send_route_task_ack(
            "broadcast_finished", "success", "broadcast completion accepted",
            command_data
        )
        # APP 收到 broadcast_finished success ack 后，只能说明播报完成回执已被 ROS 接受；
        # 单个 task 是否完成、是否进入下一段或整条路线是否结束，要继续消费随后发布的
        # task_waypoint_completed 或 route_task_completed 事件，不能只靠这个 ack 更新最终进度。
        self.finalize_task_waypoint_completion(waypoint_id)

    def is_localization_resume_blocked(self) -> bool:
        """prior-map 定位未达到启动条件时，禁止 APP 恢复 route task。

        旧链路会等待 /localization/recovery_status 和 recovery_done；
        当前新链路只认 prior_map_odom_bridge_status，避免恢复按钮被已下线的全局重定位状态卡住。
        """
        return self.get_localization_start_block_reason() is not None

    def publish_zero_cmd_vel(self):
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception as e:
            self.get_logger().error(f"发布零速度失败: {e}")

    def check_navigation_status(self):
        """检查导航状态 - 方案A：仅用于超时监控，不触发到达逻辑"""
        if self.current_state != NavigationState.EXECUTING:
            return

        self.check_route_task_feedback_timeout()

        # 记录距离，用于 APP 端的 UI 进度条显示
        if self.current_pose and self.current_waypoint:
            self.last_known_distance = self.calculate_distance_to_waypoint()

    def check_route_task_feedback_timeout(self):
        """检查 route task through feedback 是否长时间静默。

        这里只检查 `ROUTE_TASK_SEGMENT_NAVIGATING`：
        1. 该阶段应该由 NavigateThroughPoses 持续反馈；
        2. `WAITING_BROADCAST` 阶段已经到达目标 task，等待的是 APP 播报回执，不应按 Nav2 feedback 超时处理；
        3. 超时后进入 route task 专用失败流程，避免旧普通导航失败策略自动 skip。
        """
        if self.active_route_task is None:
            return
        if self.current_detailed_state != "ROUTE_TASK_SEGMENT_NAVIGATING":
            return
        if not self.current_goal_handle:
            return
        timeout_sec = max(0.0, float(self.route_task_nav2_feedback_timeout_sec))
        if timeout_sec <= 0:
            return
        if self.route_task_last_feedback_time <= 0:
            return

        feedback_age = time.time() - self.route_task_last_feedback_time
        if feedback_age <= timeout_sec:
            return

        self.get_logger().error(
            f"route task through feedback 超时: age={feedback_age:.2f}s, "
            f"timeout={timeout_sec:.2f}s, segment_id={self.active_segment.get('segment_id', '') if self.active_segment else ''}"
        )
        self.handle_route_task_navigation_failed(
            "NavigateThroughPoses feedback timeout",
            failure_code="feedback_timeout"
        )

    def process_obstacle_wait_state(self):
        """处理“因动态障碍物暂停等待”的周期逻辑。"""
        if not self.obstacle_wait_active:
            return
        if self.current_state != NavigationState.PAUSED:
            return

        now = time.time()
        block_duration = now - self.obstacle_wait_started_at if self.obstacle_wait_started_at > 0.0 else 0.0

        # 每 4 秒持续给 APP 推送一次等待文案，保证 UI 和播报都能维持“仍在受阻”。
        if (
            self.obstacle_wait_push_interval_sec > 0.0 and
            now - self.obstacle_wait_last_push_time >= self.obstacle_wait_push_interval_sec
        ):
            self.publish_obstacle_blocked_event(block_duration, send_ack=False)
            self.obstacle_wait_last_push_time = now

        # costmap 太久没更新时，不做 clear 判定，避免用陈旧数据误恢复导航。
        max_costmap_age = max(1.0, 2.0 / self.obstacle_clear_check_rate_hz)
        if now - self.latest_local_costmap_stamp > max_costmap_age:
            self.obstacle_clear_confirm_count = 0
            return

        if self.latest_front_obstacle_blocked:
            self.obstacle_clear_confirm_count = 0
            return

        self.obstacle_clear_confirm_count += 1
        if self.obstacle_clear_confirm_count < self.obstacle_clear_required_frames:
            return

        self.resume_from_obstacle_wait()

    def resume_from_obstacle_wait(self):
        """障碍物消失后自动恢复当前导航目标。

        route task 会恢复当前 NavigateThroughPoses 段；非 route task 只作为内部兜底，
        不再对应任何 APP 旧导航控制命令。
        """
        if not self.obstacle_wait_active:
            return
        if self.current_state != NavigationState.PAUSED:
            return
        if self.active_route_task and self.active_segment:
            # route task 的 through 段不一定维护普通 current_waypoint，
            # 自动恢复时以 active_segment 为准，避免障碍清除后误判“当前点丢失”。
            resumed_waypoint_id = str(self.active_segment.get("segment_target_task_id", ""))
            resumed_waypoint = self.find_route_waypoint_by_id(resumed_waypoint_id)
            resumed_waypoint_name = resumed_waypoint.get("name", "") if resumed_waypoint else resumed_waypoint_id
        else:
            if not self.current_waypoint:
                self.get_logger().warning("障碍物已清除，但当前 waypoint 丢失，无法自动恢复导航")
                self.clear_obstacle_wait_state()
                self.reset_navigation_state()
                return
            resumed_waypoint_id = self.current_waypoint.get("id", "")
            resumed_waypoint_name = self.current_waypoint.get("name", "")

        pause_elapsed = time.time() - self.pause_time if self.pause_time else 0.0
        self.clear_obstacle_wait_state()
        self.current_state = NavigationState.EXECUTING
        self.current_detailed_state = "EXECUTING"
        self.current_pause_source = ""
        self.current_pause_reason = ""
        self.current_resume_mode = ""

        event_data = {
            "resumed_waypoint_id": resumed_waypoint_id,
            "resumed_waypoint_name": resumed_waypoint_name,
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "pause_duration_actual": round(pause_elapsed, 1),
            "resume_reason": "obstacle_cleared_auto_resume",
            "resume_source": "obstacle_wait",
        }
        if self.active_segment:
            event_data["segment_id"] = self.active_segment.get("segment_id", "")
        if self.latest_front_obstacle_stats:
            event_data["front_obstacle_stats"] = self.latest_front_obstacle_stats

        self.send_acknowledgment("navigation_resumed", "success", "障碍物已消失，导航自动恢复", event_data)
        self.publish_status_update("navigation_resumed", event_data)
        if self.active_route_task and self.active_segment:
            # 障碍恢复时必须重启当前 active_segment，并继续走 route task 策略选择器；
            # 不能退回旧 navigate_to_waypoint 命令链路，否则会破坏 task/transit/播报语义。
            if not self.start_active_segment_navigation(
                ack_command_type="obstacle_cleared_auto_resume",
                ack_command_data=self.active_route_task,
                send_failure_ack=False,
            ):
                self.handle_route_task_navigation_failed(
                    "obstacle cleared auto resume failed",
                    failure_code="obstacle_auto_resume_failed",
                )
                return
        else:
            # 旧 APP 单点/多点导航入口已下线；如果没有 active_segment，说明当前状态不属于
            # 新路线任务不能绕开 start_active_segment_navigation()，否则会丢失
            # 跳步、transit、最终对齐和播报闭环上下文。
            self.get_logger().error("障碍恢复失败：当前没有 route task active_segment")
            self.handle_navigation_failed("missing route task active segment on obstacle resume")
            return
        self.get_logger().info("前方障碍物已连续 clear 多帧，自动恢复当前导航目标")
    
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
    
    @staticmethod
    def waypoint_position_tuple(waypoint: Optional[Dict[str, Any]]) -> Optional[Tuple[float, float, float]]:
        if not waypoint:
            return None
        position = waypoint.get("position", [0.0, 0.0, 0.0])
        if not isinstance(position, (list, tuple)) or len(position) < 2:
            return None
        try:
            return (
                float(position[0]),
                float(position[1]),
                float(position[2]) if len(position) >= 3 else 0.0,
            )
        except (TypeError, ValueError):
            return None

    def handle_navigation_failed(self, reason: str):
        """处理导航失败"""
        # 如果当前状态是 PAUSED(暂停)，说明是我们为了互动主动取消的，不要标记为失败，也不要重置数据
        if self.current_state == NavigationState.PAUSED:
            if self.obstacle_wait_active:
                self.get_logger().info("检测到导航由障碍等待自动暂停，保留数据以备障碍物消失后恢复...")
            else:
                self.get_logger().info("检测到导航由用户主动暂停，保留数据以备恢复...")
            return

        # 只有在非暂停状态下的取消/报错，才视为真实失败
        if self.current_goal_handle:
            self.cancel_navigation()

        self.current_state = NavigationState.FAILED
        self.current_detailed_state = "FAILED"
        self.current_goal_handle = None
        self.reset_block_detection()

        failure_context = {
            "reason": reason,
            "failed_waypoint_index": self.current_waypoint_index,
            "failed_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "failed_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "navigation_mode": self.current_navigation_mode_value(),
            "route_task": self.active_route_task is not None,
        }
        self.send_acknowledgment("navigation_failed", "error", reason, failure_context)
        self.publish_status_update("navigation_failed", failure_context)

        self.get_logger().error(f"导航失败: {reason}")

    def try_enter_obstacle_wait_from_nav_failure(self, reason: str) -> bool:
        """Nav2/RPP 执行阶段失败时，优先交给动态障碍等待状态机接管。"""
        if not self.obstacle_wait_enable or self.obstacle_wait_active:
            return False
        if self.current_state not in (NavigationState.EXECUTING, NavigationState.PLANNING):
            return False
        if not self.active_route_task or not self.active_segment:
            return False

        suppression_reason = self.get_obstacle_blockage_suppression_reason()
        if suppression_reason:
            self.get_logger().info(
                f"Nav2失败但当前处于{suppression_reason}，不进入障碍等待: {reason}"
            )
            return False

        self.get_logger().warning(
            f"Nav2/RPP 执行失败，按前方障碍阻塞接管并暂停等待: {reason}"
        )
        self.enter_obstacle_wait_state(0.0)
        return True
    
    def nav2_log_callback(self, msg):
        """解析 Nav2 行为树日志，识别当前正在进行的具体动作"""
        if self.current_state not in (NavigationState.EXECUTING, NavigationState.PLANNING):
            if self.nav2_blockage_suppression_nodes:
                self.nav2_blockage_suppression_nodes.clear()
            if self.current_detailed_state in {"RECOVERING", "TURNING"}:
                self.current_detailed_state = self.current_state.value
            return

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

    def find_waypoint_data_by_id(self, waypoint_id: str, map_id: str = "") -> Optional[Dict[str, Any]]:
        """根据 ID 查找点位数据。

        多地图路线任务必须传 map_id，只在该地图缓存里找；不传 map_id 的全局查找仅作旧格式兼容。
        """
        if not waypoint_id:
            return None
        waypoint_id = self.route_task_id(waypoint_id)
        normalized_map_id = self.route_task_id(map_id)
        if normalized_map_id:
            return self.find_waypoint_data_in_bucket(
                self.waypoints_data_by_map.get(normalized_map_id, {}),
                waypoint_id
            )

        return self.find_waypoint_data_in_bucket(self.waypoints_data, waypoint_id)

    def find_waypoint_data_in_bucket(self, waypoint_bucket: Dict[str, Any], waypoint_id: str) -> Optional[Dict[str, Any]]:
        """在一个地图 bucket 内查找点位，兼容按类型分组和旧扁平结构。"""
        if not isinstance(waypoint_bucket, dict) or not waypoint_id:
            return None

        # 方案1: 直接顶层查找
        if waypoint_id in waypoint_bucket:
            data = waypoint_bucket[waypoint_id]
            return data if isinstance(data, dict) else None
   
        # 方案2: 深入一层
        for category_dict in waypoint_bucket.values():
            if isinstance(category_dict, dict) and waypoint_id in category_dict:
              return category_dict[waypoint_id]

         # 方案3: 再深入一层（兼容三级结构）
        for category_dict in waypoint_bucket.values():
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
                "navigation_mode": self.current_navigation_mode_value()
            }
            
            update_msg = String()
            update_msg.data = json.dumps(update_data)
            self.navigation_status_pub.publish(update_msg)
            
            self.get_logger().debug(f"发布状态更新: {event_type}")
            
        except Exception as e:
            self.get_logger().error(f"发布状态更新错误: {e}")
    
    def get_current_status_summary(self) -> Dict[str, Any]:
        """获取当前状态摘要"""
        status_summary = {
            "timestamp": time.time(),
            "current_state": self.current_state.value,
            "navigation_mode": self.current_navigation_mode_value(),
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
            "localization_auto_paused": self.localization_auto_paused,
            "localization_health_status": self.last_localization_status_summary,
            "localization_healthy": self._is_localization_healthy,
            "localization_has_last_good_tf": self.localization_has_last_good_tf,
            "active_map_id": self.active_map_id,
            "map_state": self.map_state,
            "map_localization_state": self.map_localization_state,
            "map_status_age": (
                time.time() - self.last_map_status_update
                if self.last_map_status_update > 0.0 else 0.0
            ),
            "pause_source": self.current_pause_source,
            "pause_reason": self.current_pause_reason,
            "resume_mode": self.current_resume_mode,
            "waiting_for_obstacle_clear": self.obstacle_wait_active,
            "obstacle_wait_active": self.obstacle_wait_active,
            "obstacle_wait_duration": (
                time.time() - self.obstacle_wait_started_at
                if self.obstacle_wait_active and self.obstacle_wait_started_at > 0.0 else 0.0
            ),
            "obstacle_clear_confirm_count": self.obstacle_clear_confirm_count,
            "obstacle_clear_required_frames": self.obstacle_clear_required_frames,
            "front_obstacle_blocked": self.latest_front_obstacle_blocked,
            "front_obstacle_stats": self.latest_front_obstacle_stats,
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

        if self.active_route_task is not None:
            # 周期状态补充 route task 上下文，帮助 APP/调试端在没有新事件时也能知道当前路线任务进度。
            status_summary["route_task"] = self.build_route_task_status_summary()
        
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
            NavigationState.PAUSED
        ]
        return self.current_state in active_states
    
    def reset_navigation_state(self):
        """重置导航状态"""
        self.current_state = NavigationState.IDLE
        self.current_detailed_state = "IDLE"
        self.current_navigation_mode = None
        self.current_waypoint_index = 0
        self.total_waypoints = 0
        self.current_waypoint = None
        self.navigation_start_time = 0
        self.current_goal_handle = None
        self.localization_auto_paused = False
        self.current_pause_source = ""
        self.current_pause_reason = ""
        self.current_resume_mode = ""
        self.clear_obstacle_wait_state()
        
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
    """prior-map 定位版本入口。"""
    rclpy.init(args=args)

    node = None
    try:
        node = NavigationStateManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("收到键盘中断信号")
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"导航状态管理器运行错误: {e}")
        else:
            print(f"导航状态管理器初始化失败: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
