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
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid
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
    导航状态管理器节点 - prior-map 定位版本。

    这个文件是从 navigation_state_manager_fusion.py 拆出来的新入口。
    旧 fusion 文件保留原定位逻辑，便于回退；本文件只认
    prior_map_odom_bridge 发布的定位健康状态。
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
            ('obstacle_block_timeout', 4.0),  # 障碍物兜底阻塞超时时间（秒）：主判断交给 Nav2/RPP 失败接管
            ('velocity_threshold', 0.10),  # 进入停滞检测的速度阈值（m/s），适用于 odom 与 pose_delta
            ('blockage_pose_delta_deadzone', 0.10),  # 速度停滞兜底阈值，降低误把短暂停顿当障碍的概率
            ('blockage_recovery_velocity_threshold', 0.10),  # 连续高于该速度 1 秒后清除停滞计时
            ('blockage_recovery_confirm_sec', 1.0),
            ('obstacle_wait_enable', True),
            # 低速只能说明机器人没有按预期前进，不能单独证明前方有障碍。
            # 默认必须由 ROI 点云或 local costmap 至少一方确认，才允许取消 Nav2 进入障碍等待。
            ('obstacle_block_require_sensor_confirmation', True),
            ('obstacle_block_sensor_timeout_sec', 1.0),
            ('obstacle_wait_push_interval_sec', 4.0),
            ('obstacle_clear_required_frames', 5),
            ('obstacle_clear_check_rate_hz', 5.0),
            ('obstacle_clear_cost_threshold', 100),  # /local_costmap/costmap 是 OccupancyGrid，致命障碍通常为 100。
            ('obstacle_clear_front_min_x_m', 0.15),
            ('obstacle_clear_front_max_x_m', 0.80),  # 障碍恢复只看机器人近前方，降低墙/玻璃门误判。
            ('obstacle_clear_half_width_m', 0.30),  # 左右各 0.30m；这是状态机 clear 窗口，不是 RPP 碰撞参数。
            ('obstacle_min_wait_before_resume_sec', 2.0),  # 避免刚暂停 1 秒左右就因单帧 clear 误恢复。
            ('obstacle_clear_required_duration_sec', 3.0),  # 恢复前要求前方连续 clear 的时长。
            ('obstacle_clear_required_duration_after_false_resume_sec', 4.0),  # 误恢复后下一次恢复更保守。
            ('obstacle_false_resume_window_sec', 3.0),  # 恢复后很快再次阻塞，视为一次“误恢复”。
            ('obstacle_resume_use_roi', True),
            ('obstacle_roi_has_obstacle_topic', '/front_obstacle/has_obstacle'),
            ('obstacle_roi_timeout_sec', 1.0),
            ('obstacle_roi_required_clear_frames', 3),
            ('local_costmap_topic', '/local_costmap/costmap'),
            ('require_walk_mode_for_navigation', True),
            ('robot_status_timeout', 2.0),
            ('pending_navigation_timeout', 90.0),
            ('obstacle_block_near_goal_distance', 0.7),
            ('navigation_failure_policy', 'pause_on_failed'),
            ('auto_pause_on_localization_recovery', False),
            ('localization_stop_hold_sec', 2.0),
            ('localization_resume_settle_sec', 1.0),
            ('localization_auto_resume_require_recovery_done', True),
            ('localization_resume_stable_frames', 3),
            ('localization_health_status_topic', '/localization/prior_map_odom_bridge_status'),
            ('localization_health_timeout_sec', 3.0),
            ('localization_allow_start_with_last_good_tf', True),
            ('localization_last_good_tf_max_age_sec', 0.0),
            ('localization_recovery_status_topic', '/localization/recovery_status'),
            ('localization_recovery_request_topic', '/localization/recovery_requests'),
            ('request_localization_recovery_on_nav_failure', False),
            ('request_navigation_context_recovery_on_localization_failure', False),
            ('localization_recovery_request_cooldown_sec', 20.0),
            ('localization_recovery_prior_radius_m', 10.0),
            ('localization_context_recovery_request_cooldown_sec', 4.0),
            ('localization_context_prior_radius_m', 5.0),
            ('localization_context_prior_max_previous_age_sec', 300.0),
            ('localization_context_prior_min_segment_length_m', 0.2),
            ('localization_resume_reverse_enabled', True),
            ('localization_resume_reverse_max_distance_m', 2.0),
            ('localization_resume_reverse_rear_angle_deg', 70.0),
            ('map_frame', 'map'),
            ('base_frame', 'base_footprint'),
            ('pose_tf_timeout_sec', 0.05),
            (
                'reverse_navigation_bt_xml',
                '/home/ubuntu/humanoid_ws/src/humanoid_navigation2/config/behavior_tree/'
                'navigate_reverse_xy_then_yaw.xml'
            )
        ])

        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.waypoint_timeout = self.get_parameter('waypoint_timeout').value
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
        self.obstacle_block_require_sensor_confirmation = bool(
            self.get_parameter('obstacle_block_require_sensor_confirmation').value)
        self.obstacle_block_sensor_timeout_sec = max(
            0.1, float(self.get_parameter('obstacle_block_sensor_timeout_sec').value))
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
        self.obstacle_min_wait_before_resume_sec = float(
            self.get_parameter('obstacle_min_wait_before_resume_sec').value)
        self.obstacle_clear_required_duration_sec = float(
            self.get_parameter('obstacle_clear_required_duration_sec').value)
        self.obstacle_clear_required_duration_after_false_resume_sec = float(
            self.get_parameter('obstacle_clear_required_duration_after_false_resume_sec').value)
        self.obstacle_false_resume_window_sec = float(
            self.get_parameter('obstacle_false_resume_window_sec').value)
        self.obstacle_resume_use_roi = bool(
            self.get_parameter('obstacle_resume_use_roi').value)
        self.obstacle_roi_has_obstacle_topic = str(
            self.get_parameter('obstacle_roi_has_obstacle_topic').value)
        self.obstacle_roi_timeout_sec = float(
            self.get_parameter('obstacle_roi_timeout_sec').value)
        self.obstacle_roi_required_clear_frames = max(
            1, int(self.get_parameter('obstacle_roi_required_clear_frames').value))
        self.local_costmap_topic = str(self.get_parameter('local_costmap_topic').value)
        self.require_walk_mode_for_navigation = self.get_parameter('require_walk_mode_for_navigation').value
        self.robot_status_timeout = float(self.get_parameter('robot_status_timeout').value)
        self.pending_navigation_timeout = float(self.get_parameter('pending_navigation_timeout').value)
        self.obstacle_block_near_goal_distance = float(self.get_parameter('obstacle_block_near_goal_distance').value)
        self.navigation_failure_policy = str(self.get_parameter('navigation_failure_policy').value)
        self.auto_pause_on_localization_recovery = bool(
            self.get_parameter('auto_pause_on_localization_recovery').value)
        self.localization_stop_hold_sec = float(
            self.get_parameter('localization_stop_hold_sec').value)
        self.localization_resume_settle_sec = float(
            self.get_parameter('localization_resume_settle_sec').value)
        self.localization_auto_resume_require_recovery_done = bool(
            self.get_parameter('localization_auto_resume_require_recovery_done').value)
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
        self.localization_recovery_status_topic = str(
            self.get_parameter('localization_recovery_status_topic').value)
        self.localization_recovery_request_topic = str(
            self.get_parameter('localization_recovery_request_topic').value)
        self.request_localization_recovery_on_nav_failure = bool(
            self.get_parameter('request_localization_recovery_on_nav_failure').value)
        self.request_navigation_context_recovery_on_localization_failure = bool(
            self.get_parameter('request_navigation_context_recovery_on_localization_failure').value)
        self.localization_recovery_request_cooldown_sec = float(
            self.get_parameter('localization_recovery_request_cooldown_sec').value)
        self.localization_recovery_prior_radius_m = float(
            self.get_parameter('localization_recovery_prior_radius_m').value)
        self.localization_context_recovery_request_cooldown_sec = float(
            self.get_parameter('localization_context_recovery_request_cooldown_sec').value)
        self.localization_context_prior_radius_m = float(
            self.get_parameter('localization_context_prior_radius_m').value)
        self.localization_context_prior_max_previous_age_sec = float(
            self.get_parameter('localization_context_prior_max_previous_age_sec').value)
        self.localization_context_prior_min_segment_length_m = float(
            self.get_parameter('localization_context_prior_min_segment_length_m').value)
        self.localization_resume_reverse_enabled = bool(
            self.get_parameter('localization_resume_reverse_enabled').value)
        self.localization_resume_reverse_max_distance_m = float(
            self.get_parameter('localization_resume_reverse_max_distance_m').value)
        self.localization_resume_reverse_rear_angle_rad = math.radians(float(
            self.get_parameter('localization_resume_reverse_rear_angle_deg').value))
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
        # WS 动作执行互锁：只自动恢复由本互锁主动暂停的导航任务。
        self.robot_action_interlock_active = False
        self.robot_action_interlock_paused_navigation = False
        self.robot_action_resume_pending = False
        self.robot_action_interlock_started_at = 0.0
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
        # 这里单独维护“因障碍暂停”的内部状态，让它和“用户手动暂停”彻底区分。
        self.obstacle_wait_active = False
        self.obstacle_wait_started_at = 0.0
        self.obstacle_wait_last_push_time = 0.0
        self.obstacle_clear_confirm_count = 0
        self.obstacle_clear_started_at = 0.0
        self.last_obstacle_resume_time = 0.0
        self.obstacle_recent_false_resume_count = 0
        self.latest_front_obstacle_blocked = False
        self.latest_front_obstacle_stats = {}
        self.latest_local_costmap = None
        self.latest_local_costmap_stamp = 0.0
        self.latest_roi_obstacle_has_obstacle: Optional[bool] = None
        self.latest_roi_obstacle_stamp = 0.0
        self.roi_obstacle_clear_confirm_count = 0
        
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
        self.localization_stop_until = 0.0
        self.localization_recovered_at = 0.0
        self.localization_recovery_done = False
        self.localization_resume_stable_count = 0
        self.last_localization_status_summary = {}
        self.localization_has_last_good_tf = False
        self.localization_last_good_tf_time = 0.0
        self.last_resume_wait_log_time = 0.0
        self.last_localization_recovery_request_time = 0.0
        self.last_navigation_context_recovery_request_time = 0.0
        self.last_navigation_context_recovery_key = ""
        self.last_succeeded_waypoint = None
        self.last_succeeded_waypoint_index = -1
        self.last_succeeded_pose = None
        self.last_succeeded_time = 0.0
        
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

        self.localization_recovery_request_pub = self.create_publisher(
            String,
            self.localization_recovery_request_topic,
            10
        )
        
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

        # 订阅直接点云 ROI 障碍检测结果。ROI 用来确认真实前方点云里是否还有障碍；
        # costmap 仍作为 Nav2 视角兜底，两者都 clear 才自动恢复。
        self.roi_obstacle_sub = self.create_subscription(
            Bool, self.obstacle_roi_has_obstacle_topic, self.roi_obstacle_callback, 10
        )

        self.robot_status_sub = self.create_subscription(
            String, '/robot_status_raw', self.robot_status_callback, 10
        )

        self.robot_action_interlock_service = self.create_service(
            SetBool,
            '/navigation/robot_action_interlock',
            self.robot_action_interlock_callback,
        )
        
        self.nav2_behavior_log_sub = self.create_subscription(BehaviorTreeLog,'/behavior_tree_log',self.nav2_log_callback,10)

        self.localization_recovery_sub = self.create_subscription(
            String,
            self.localization_recovery_status_topic,
            self.localization_recovery_status_callback,
            10
        )

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
        self.create_timer(1.0, self._check_localization_status_timeout)

    # =========================================================================
    # prior-map 定位健康监听
    # =========================================================================

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
            if self._localization_healthy_count >= max(1, self.localization_resume_stable_frames):
                self._handle_localization_healthy()
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

    def _handle_localization_degraded(self, reason: str):
        """定位链路确认失效时暂停导航，并请求外部重定位。"""
        if self.current_state not in (NavigationState.EXECUTING, NavigationState.PLANNING):
            return

        self.localization_auto_paused = True
        self.localization_resume_pending = False
        self.localization_recovery_active = True
        self.localization_recovery_done = False
        self.localization_resume_stable_count = 0
        self.localization_recovery_reason = f"prior-map定位异常: {reason}"
        self.localization_recovered_at = 0.0

        self.current_state = NavigationState.PAUSED
        self.current_detailed_state = "LOCALIZATION_RECOVERY"
        self.pause_time = time.time()
        self.pause_duration_limit = 0
        self.current_pause_source = "prior_map_degraded"
        self.current_pause_reason = self.localization_recovery_reason
        self.current_resume_mode = "auto"
        self.clear_obstacle_wait_state()
        self.reset_block_detection()
        self.begin_localization_stop_hold()

        if self.current_goal_handle:
            self.cancel_navigation()

        event_data = {
            "pause_source": "prior_map_degraded",
            "localization_reason": reason,
            "reason": self.localization_recovery_reason,
            "current_waypoint_id": self.current_waypoint.get("id", ""),
            "current_waypoint_name": self.current_waypoint.get("name", ""),
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
        }
        self.publish_status_update("navigation_paused", event_data)
        self.send_acknowledgment(
            "navigation_auto_paused", "success",
            "定位异常，已暂停导航并开始自动重定位", event_data)
        self.get_logger().warn(
            f'[prior_map] 定位异常 ({reason}), 暂停导航 + zero cmd hold')

        # 发 recovery 请求
        status = {"event_type": "localization_failure", "reason": reason}
        self.request_navigation_context_recovery_for_localization(reason, status)

    def _handle_localization_healthy(self):
        """prior-map 定位恢复健康后，尝试恢复被定位流程暂停的导航。"""
        if not self.localization_auto_paused:
            return
        if self.localization_auto_resume_require_recovery_done and not self.localization_recovery_done:
            now = time.time()
            if now - self.last_resume_wait_log_time > 2.0:
                self.last_resume_wait_log_time = now
                self.publish_status_update("navigation_localization_resume_waiting", {
                    "reason": "prior-map定位已接受，但定位恢复流程尚未确认完成，暂不恢复导航",
                    "localization_event": self.localization_recovery_last_status.get("event_type", ""),
                    "localization_stable_count": self.localization_resume_stable_count,
                    "required_localization_stable_frames": self.localization_resume_stable_frames,
                    "localization_status": self.last_localization_status_summary,
                })
            return
        self._localization_healthy_count = 0
        if not self.localization_resume_pending:
            self.localization_resume_pending = True     # R4: try_resume依赖此标志
        if self.localization_recovered_at <= 0.0:
            self.localization_recovered_at = time.time()
        self.get_logger().info('[prior_map] 定位恢复健康, 尝试 resume 导航')
        self.try_resume_after_localization_recovery()

    def _cache_navigation_for_recovery(self, request_data: dict):
        """定位恢复期间收到的新点位请求 → 缓存 + 告知 APP pending

        Pending 语义:
          - 只保留最后一个请求 (新请求覆盖旧请求)
          - 覆盖时通知 APP: "pending_overwritten"
          - 用户 cancel_navigation 时清除 pending request
          - 恢复后自动执行 (延迟 1s 等定位/costmap 稳定)
          - pending 超时 (pending_navigation_timeout: 90s) → 通知 APP 失败
        """
        was_pending = self.pending_navigation_request is not None
        self.pending_navigation_request = json.loads(json.dumps(request_data))
        self.pending_navigation_created_at = time.time()
        self.pending_navigation_reason = "等待定位恢复后自动执行"
        self.send_acknowledgment(
            "navigation_pending_overwritten" if was_pending else "navigation_pending",
            "pending",
            f"定位异常(prior-map不健康)，导航请求已{'覆盖' if was_pending else '缓存'}，定位恢复后自动执行")
        self.get_logger().info(
            f'[prior_map] 新点位请求已缓存 (定位不健康), '
            f'{"覆盖旧请求" if was_pending else "首次缓存"}')

    def _execute_pending_navigation_later(self, delay_sec: float = 1.0):
        """延迟执行缓存的新点位 (等定位/costmap 稳定)"""
        if self.pending_navigation_request is None:
            return
        self.get_logger().info(
            f'[prior_map] 将在 {delay_sec}s 后执行缓存的新点位导航')
        self.create_timer(delay_sec, self._execute_pending_navigation_now, oneshot=True)

    def _execute_pending_navigation_now(self):
        """立即执行缓存的导航请求 (复用现有 dispatch)"""
        if self.pending_navigation_request is None:
            return
        if self.current_state != NavigationState.IDLE:
            self.get_logger().warn(
                '[prior_map] 待执行缓存导航时状态非 IDLE, 跳过')
            return

        if not self._is_localization_healthy:
            self.get_logger().info(
                '[prior_map] 待执行缓存导航时定位仍不健康，继续等待')
            return

        request_data = self.pending_navigation_request
        self.pending_navigation_request = None
        self.pending_navigation_reason = ""

        command_type = request_data.get("command_data", {}).get("command_type", "")
        self.get_logger().info(
            f'[prior_map] 定位恢复, 执行缓存的新点位导航: {command_type}')
        # 复用现有导航命令 dispatch
        self.handle_navigation_command(request_data)


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

        # 动作完成释放服务时只登记恢复请求，避免在服务回调里等待 Nav2 action server。
        self.create_timer(0.1, self.try_resume_after_robot_action)

        # 动作切换 Menu 前后持续压零速度，覆盖 Nav2 异步取消窗口。
        self.create_timer(0.033, self.enforce_robot_action_stop)

        # 定位恢复后自动继续未完成导航
        self.create_timer(0.5, self.try_resume_after_localization_recovery)

        # 定位恢复期间持续压零速度，避免异步 cancel 期间沿旧 cmd_vel 继续走
        # P1-1: 提高到 30Hz 压制 Nav2 controller (20Hz), 防止竞态导致 robot 继续移动
        self.create_timer(0.033, self.enforce_localization_stop)

        # 动态障碍物等待恢复定时器：
        # 1. 每 4s 持续给 APP 推等待文案
        # 2. 使用 costmap clear 多帧确认后自动恢复导航
        self.create_timer(1.0 / self.obstacle_clear_check_rate_hz, self.process_obstacle_wait_state)
    
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
        # 进入停滞检测统一使用 0.10m/s。
        # 之前 pose_delta 在这里误用了 0.15m/s，导致 0.10~0.15m/s 的正常慢走也开始阻塞计时。
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
            self.latest_local_costmap = msg
            blocked, stats = self.analyze_front_obstacle_window(msg)
            self.latest_front_obstacle_blocked = blocked
            self.latest_front_obstacle_stats = stats
        except Exception as e:
            self.get_logger().error(f'❌ 处理 local costmap 错误: {e}')

    def roi_obstacle_callback(self, msg: Bool):
        """处理前方 ROI 点云障碍检测结果。"""
        self.latest_roi_obstacle_has_obstacle = bool(msg.data)
        self.latest_roi_obstacle_stamp = time.time()
        if self.latest_roi_obstacle_has_obstacle:
            self.roi_obstacle_clear_confirm_count = 0
        else:
            self.roi_obstacle_clear_confirm_count += 1

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

        robot_x = float(robot_pose.position.x)
        robot_y = float(robot_pose.position.y)
        robot_yaw = yaw_from_pose(robot_pose)

        resolution = float(msg.info.resolution)
        width = int(msg.info.width)
        height = int(msg.info.height)
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)

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

    def robot_action_interlock_callback(self, request, response):
        """动作执行前暂停活动导航，动作结束且回到 Walk 后登记自动恢复。"""
        if request.data:
            if self.robot_action_interlock_active:
                response.success = True
                response.message = "导航动作互锁已启用"
                return response

            self.robot_action_interlock_active = True
            self.robot_action_interlock_started_at = time.time()
            self.robot_action_resume_pending = False
            self.robot_action_interlock_paused_navigation = (
                self.current_state in (NavigationState.EXECUTING, NavigationState.PLANNING)
                and self.current_waypoint is not None
            )

            if self.robot_action_interlock_paused_navigation:
                self.current_state = NavigationState.PAUSED
                self.current_detailed_state = "ROBOT_ACTION_PAUSED"
                self.pause_time = time.time()
                self.pause_duration_limit = 0
                self.current_pause_source = "robot_action"
                self.current_pause_reason = "机器人执行动作，导航已自动暂停"
                self.current_resume_mode = "auto"
                self.clear_obstacle_wait_state()
                self.reset_block_detection()
                if self.current_goal_handle:
                    self.cancel_navigation()
                # 连续补几帧零速度，缩短取消 Nav2 goal 的异步窗口。
                for _ in range(3):
                    self.publish_zero_cmd_vel()

                event_data = self.build_pause_event_data(
                    pause_source="robot_action",
                    reason=self.current_pause_reason,
                    resume_mode="auto",
                    extra_data={"action_interlock_active": True},
                )
                self.publish_status_update("navigation_paused", event_data)
                self.send_acknowledgment(
                    "navigation_auto_paused",
                    "success",
                    "收到机器人动作，已暂停导航并保存当前任务",
                    event_data,
                )
                response.message = "活动导航已暂停并保存，允许执行机器人动作"
                self.get_logger().info("机器人动作互锁已暂停当前导航")
            else:
                # 原本就是手动暂停/障碍等待/定位恢复时，不取得自动恢复所有权。
                response.message = "当前无可由动作互锁暂停的活动导航，允许执行机器人动作"

            response.success = True
            return response

        if not self.robot_action_interlock_active:
            response.success = True
            response.message = "导航动作互锁已释放"
            return response

        self.robot_action_interlock_active = False
        should_resume = (
            self.robot_action_interlock_paused_navigation
            and self.current_state == NavigationState.PAUSED
            and self.current_pause_source == "robot_action"
            and self.current_waypoint is not None
        )
        self.robot_action_resume_pending = should_resume
        if not should_resume:
            self.robot_action_interlock_paused_navigation = False
            self.robot_action_interlock_started_at = 0.0

        response.success = True
        response.message = (
            "动作完成，已登记恢复导航" if should_resume else
            "动作完成，已释放导航互锁（原导航状态保持不变）"
        )
        return response

    def enforce_robot_action_stop(self):
        """互锁主动暂停导航期间持续压零速度，避免 Nav2 取消竞态。"""
        if (
            self.robot_action_interlock_active
            and self.robot_action_interlock_paused_navigation
            and self.current_state == NavigationState.PAUSED
            and self.current_pause_source == "robot_action"
        ):
            self.publish_zero_cmd_vel()

    def try_resume_after_robot_action(self):
        """动作结束、WS 确认回到 Walk 并释放互锁后恢复原 waypoint。"""
        if not self.robot_action_resume_pending:
            return

        self.robot_action_resume_pending = False
        should_resume = (
            not self.robot_action_interlock_active
            and self.robot_action_interlock_paused_navigation
            and self.current_state == NavigationState.PAUSED
            and self.current_pause_source == "robot_action"
            and self.current_waypoint is not None
        )
        pause_elapsed = (
            time.time() - self.robot_action_interlock_started_at
            if self.robot_action_interlock_started_at > 0.0 else 0.0
        )
        self.robot_action_interlock_paused_navigation = False
        self.robot_action_interlock_started_at = 0.0
        if not should_resume:
            self.get_logger().info("动作互锁释放后导航状态已变化，不再自动恢复")
            return

        self.current_state = NavigationState.EXECUTING
        self.current_detailed_state = "EXECUTING"
        self.current_pause_source = ""
        self.current_pause_reason = ""
        self.current_resume_mode = ""
        event_data = {
            "resumed_waypoint_id": self.current_waypoint.get("id", ""),
            "resumed_waypoint_name": self.current_waypoint.get("name", ""),
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "pause_duration_actual": round(pause_elapsed, 1),
            "resume_reason": "robot_action_completed",
            "resume_source": "robot_action",
        }
        self.publish_status_update("navigation_resumed", event_data)
        self.send_acknowledgment(
            "navigation_auto_resumed",
            "success",
            "机器人动作完成，已恢复原导航任务",
            event_data,
        )
        waypoint = self.current_waypoint
        self.navigate_to_waypoint(waypoint)
        self.get_logger().info(
            f"机器人动作完成，恢复前往路点: {waypoint.get('name', '')}"
        )

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
        if self.robot_action_interlock_active:
            return "机器人动作互锁中，暂不启动导航"

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
        can_defer = self.robot_action_interlock_active or (
            status_fresh and (
                self.robot_motion_busy or self.robot_control_state == "Menu"
            )
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
        if self.robot_action_interlock_active:
            pending_message = f"{reason}。已缓存导航请求，等待动作完成并回到 Walk 后再开始导航。"
        elif status_unavailable:
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
            # R5: 定位恢复期间(PAUSED+LOCALIZATION_RECOVERY)不清除pending,
            # 等待恢复后执行。其他非IDLE状态才是真正的冲突。
            if (self.current_state == NavigationState.PAUSED and
                self.localization_auto_paused):
                return  # 等待定位恢复
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
        if self.robot_action_interlock_active:
            return "机器人动作导航互锁阶段"

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
        if self.current_detailed_state in ("BLOCKED_BY_OBSTACLE", "STALLED_UNCONFIRMED"):
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
            "clear_duration_sec": round(
                time.time() - self.obstacle_clear_started_at
                if self.obstacle_clear_started_at > 0.0 else 0.0,
                1,
            ),
            "clear_required_duration_sec": self.get_current_obstacle_clear_required_duration(),
            "min_wait_before_resume_sec": self.obstacle_min_wait_before_resume_sec,
            "false_resume_count": self.obstacle_recent_false_resume_count,
            "pause_source": "obstacle_wait",
        }
        if self.latest_front_obstacle_stats:
            event_data["front_obstacle_stats"] = self.latest_front_obstacle_stats
        event_data["roi_obstacle_stats"] = self.build_roi_obstacle_stats()

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
        self.obstacle_clear_started_at = 0.0
        self.roi_obstacle_clear_confirm_count = 0

        # 如果刚刚自动恢复后很快又被 RPP/Nav2 判定不可通行，说明上一轮恢复偏乐观；
        # 下一轮会自动拉长连续 clear 要求，避免“恢复-碰撞-暂停”来回抖动。
        if (
            self.last_obstacle_resume_time > 0.0 and
            self.pause_time - self.last_obstacle_resume_time <= self.obstacle_false_resume_window_sec
        ):
            self.obstacle_recent_false_resume_count += 1
            self.get_logger().warning(
                f"障碍等待刚恢复 {self.pause_time - self.last_obstacle_resume_time:.1f}s 后再次触发，"
                f"本轮采用更保守恢复条件 false_resume_count={self.obstacle_recent_false_resume_count}"
            )
        else:
            self.obstacle_recent_false_resume_count = 0

        # 进入等待态后重置普通阻塞计时，避免重复触发“刚暂停又马上超时”。
        self.reset_block_detection()

        if self.current_goal_handle:
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
                "clear_required_duration_sec": self.get_current_obstacle_clear_required_duration(),
                "false_resume_count": self.obstacle_recent_false_resume_count,
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
        """处理低速超时；传感器确认前不得把低速直接当成障碍。"""
        if self.block_reported or not self.obstacle_wait_enable:
            return

        obstacle_confirmed, confirmation_stats = self.get_obstacle_block_confirmation()
        if not obstacle_confirmed:
            self.current_detailed_state = "STALLED_UNCONFIRMED"
            self.get_logger().warning(
                f"机器人持续低速 {block_duration:.1f}s，但 ROI/costmap 未确认前方障碍，"
                f"保持 Nav2 运行: {confirmation_stats}",
                throttle_duration_sec=2.0,
            )
            return

        self.get_logger().warning(
            f"机器人持续低速且前方障碍已确认，进入障碍等待: {confirmation_stats}"
        )
        self.enter_obstacle_wait_state(block_duration)

    def get_obstacle_block_confirmation(
        self, now: Optional[float] = None
    ) -> Tuple[bool, Dict[str, Any]]:
        """确认低速是否确由前方障碍引起。

        ROI 和 local costmap 是两条独立观测链。任一条新鲜且明确 blocked 即可确认；
        clear、陈旧或尚未收到的数据都不能把普通低速升级成“障碍停车”。
        """
        now = time.time() if now is None else now
        if not self.obstacle_block_require_sensor_confirmation:
            return True, {
                "required": False,
                "decision": "legacy_speed_only",
                "confirmed_by": ["speed_only"],
            }

        roi_stats = self.build_roi_obstacle_stats(now)
        roi_enabled = bool(self.obstacle_resume_use_roi)
        roi_confirmed = bool(
            roi_enabled
            and roi_stats["fresh"]
            and self.latest_roi_obstacle_has_obstacle is True
        )

        costmap_age = (
            now - self.latest_local_costmap_stamp
            if self.latest_local_costmap_stamp > 0.0
            else None
        )
        costmap_fresh = bool(
            costmap_age is not None
            and costmap_age <= self.obstacle_block_sensor_timeout_sec
        )
        costmap_confirmed = bool(costmap_fresh and self.latest_front_obstacle_blocked)

        confirmed_by = []
        if roi_confirmed:
            confirmed_by.append("roi")
        if costmap_confirmed:
            confirmed_by.append("costmap")

        stats = {
            "required": True,
            "decision": "obstacle_confirmed" if confirmed_by else "low_speed_unconfirmed",
            "confirmed_by": confirmed_by,
            "roi": roi_stats,
            "costmap": {
                "topic": self.local_costmap_topic,
                "blocked": self.latest_front_obstacle_blocked,
                "age_sec": round(costmap_age, 3) if costmap_age is not None else None,
                "fresh": costmap_fresh,
                "front_obstacle_stats": self.latest_front_obstacle_stats,
            },
        }
        return bool(confirmed_by), stats

    def get_current_obstacle_clear_required_duration(self) -> float:
        """误恢复后自动拉长 clear 确认时间，普通场景保持较快恢复。"""
        if self.obstacle_recent_false_resume_count > 0:
            return max(
                self.obstacle_clear_required_duration_sec,
                self.obstacle_clear_required_duration_after_false_resume_sec,
            )
        return self.obstacle_clear_required_duration_sec

    def build_roi_obstacle_stats(self, now: Optional[float] = None) -> Dict[str, Any]:
        """构造 ROI 障碍检测状态，方便日志和 APP 调试。"""
        now = time.time() if now is None else now
        age = now - self.latest_roi_obstacle_stamp if self.latest_roi_obstacle_stamp > 0.0 else None
        return {
            "enabled": self.obstacle_resume_use_roi,
            "topic": self.obstacle_roi_has_obstacle_topic,
            "has_obstacle": self.latest_roi_obstacle_has_obstacle,
            "age_sec": round(age, 3) if age is not None else None,
            "fresh": age is not None and age <= self.obstacle_roi_timeout_sec,
            "clear_confirmed_frames": self.roi_obstacle_clear_confirm_count,
            "clear_required_frames": self.obstacle_roi_required_clear_frames,
        }

    def is_roi_obstacle_clear_for_resume(self, now: float) -> Tuple[bool, Dict[str, Any]]:
        """恢复前确认 ROI 点云检测已 clear；ROI 不在线时降级回 costmap，避免现场卡死。"""
        stats = self.build_roi_obstacle_stats(now)
        if not self.obstacle_resume_use_roi:
            stats["decision"] = "disabled"
            return True, stats

        if not stats["fresh"]:
            # ROI 节点没有启动或话题超时时，不让机器人永远等死，降级由 costmap 决定。
            stats["decision"] = "stale_fallback_to_costmap"
            self.get_logger().warning(
                f"ROI 障碍检测超时或未收到，恢复判断降级为 costmap: {stats}",
                throttle_duration_sec=2.0,
            )
            return True, stats

        if self.latest_roi_obstacle_has_obstacle:
            stats["decision"] = "roi_blocked"
            return False, stats

        if self.roi_obstacle_clear_confirm_count < self.obstacle_roi_required_clear_frames:
            stats["decision"] = "roi_clear_frames_not_enough"
            return False, stats

        stats["decision"] = "roi_clear"
        return True, stats


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

            # prior-map 定位未接受结果时缓存请求，不立即执行。
            if self.get_localization_start_block_reason():
                self._cache_navigation_for_recovery(request_data)
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

            # prior-map 定位未接受结果时缓存请求，不立即执行。
            if self.get_localization_start_block_reason():
                self._cache_navigation_for_recovery(request_data)
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

            # prior-map 定位未接受结果时缓存请求，不立即执行。
            if self.get_localization_start_block_reason():
                self._cache_navigation_for_recovery(request_data)
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
            self.current_detailed_state = "PAUSED"
            self.pause_time = time.time()
            self.pause_duration_limit = pause_duration
            self.current_pause_source = "user_request"
            self.current_pause_reason = "用户手动暂停导航"
            self.current_resume_mode = "manual"
            self.clear_obstacle_wait_state()
            self.reset_block_detection()
        
            # 物理打断底盘：取消当前 Nav2 目标
            if self.current_goal_handle:
                self.cancel_navigation()
        
            # 发送确认消息
            self.send_acknowledgment("pause_navigation", "success", "导航已暂停")
        
            # 发布状态更新
            self.publish_status_update(
                "navigation_paused",
                self.build_pause_event_data(
                    pause_source="user_request",
                    reason=self.current_pause_reason,
                    resume_mode="manual",
                ),
            )
        
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

            if self.is_localization_resume_blocked():
                self.reject_resume_during_localization_recovery()
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

            resume_source = self.current_pause_source or "user_request"
            if self.obstacle_wait_active:
                # 允许现场人员在障碍等待暂停期间点“继续”手动恢复；
                # 这时直接退出等待态，重新发当前 waypoint。
                self.clear_obstacle_wait_state()

            # 恢复导航
            self.current_state = NavigationState.EXECUTING
            self.current_detailed_state = "EXECUTING"
            self.current_pause_source = ""
            self.current_pause_reason = ""
            self.current_resume_mode = ""
            
            # 发送确认消息
            self.send_acknowledgment("navigation_resumed", "success", "导航已恢复")
            
            # 发布状态更新
            self.publish_status_update("navigation_resumed", {
                "resumed_waypoint_id": self.current_waypoint.get("id", ""),
                "resumed_waypoint_name": self.current_waypoint.get("name", ""),
                "waypoint_index": self.current_waypoint_index,
                "total_waypoints": self.total_waypoints,
                "pause_duration_actual": round(pause_elapsed, 1),
                "resume_reason": resume_reason,
                "resume_source": resume_source,
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

    def is_localization_resume_blocked(self) -> bool:
        """定位恢复链路还没完成时，禁止 App 手动恢复覆盖自动保护。"""
        return (
            self.localization_recovery_active or
            self.localization_auto_paused or
            self.localization_resume_pending or
            self.current_detailed_state == "LOCALIZATION_RECOVERY"
        )

    def reject_resume_during_localization_recovery(self):
        reason = "定位恢复中，暂不能手动恢复导航；定位准确且 TF 恢复后系统会自动继续未完成导航"
        last_event = ""
        if isinstance(self.localization_recovery_last_status, dict):
            last_event = self.localization_recovery_last_status.get("event_type", "")

        event_data = {
            "reason": reason,
            "blocked_command": "resume_navigation",
            "manual_resume_rejected": True,
            "localization_recovery_active": self.localization_recovery_active,
            "localization_auto_paused": self.localization_auto_paused,
            "localization_resume_pending": self.localization_resume_pending,
            "localization_reason": self.localization_recovery_reason,
            "localization_event": last_event,
            "current_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "current_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "auto_resume_pending": self.localization_auto_paused or self.localization_resume_pending,
        }

        self.send_acknowledgment(
            "resume_navigation",
            "error",
            reason,
            event_data
        )
        self.publish_status_update("navigation_localization_resume_waiting", event_data)
        self.get_logger().warning(f"拒绝手动恢复导航: {reason}")

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
                "localization_recovery_clearing_buffer",
                "localization_recovery_buffer_cleared",
            ):
                self.handle_localization_recovery_progress(status)
            elif event_type == "localization_relocalize_failed":
                self.handle_localization_recovery_failed(status)
            elif event_type == "localization_manual_initialpose_override":
                self.handle_localization_manual_initialpose_override(status)
            elif event_type == "localization_recovered":
                self.handle_localization_recovered(status)

        except Exception as e:
            self.get_logger().error(f"处理定位恢复状态错误: {e}")

    def handle_localization_recovery_started(self, status: Dict[str, Any]):
        if not self.auto_pause_on_localization_recovery:
            return

        # recovery_started 事件用于跟踪状态和发送 nav_context prior。
        reason = status.get("reason", "定位异常，正在重定位")
        self.localization_recovery_active = True
        self.localization_recovery_done = False
        self.localization_resume_stable_count = 0
        self.localization_recovery_reason = reason
        self.localization_recovery_started_at = time.time()
        self.localization_recovered_at = 0.0

        if self.current_state in (NavigationState.EXECUTING, NavigationState.PLANNING):
            self.localization_auto_paused = True
            self.localization_resume_pending = False
            self.current_state = NavigationState.PAUSED
            self.current_detailed_state = "LOCALIZATION_RECOVERY"
            self.pause_time = time.time()
            self.pause_duration_limit = 0
            self.reset_block_detection()
            self.begin_localization_stop_hold()

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
            self.request_navigation_context_recovery_for_localization(reason, status)
            return

        if self.current_state == NavigationState.PAUSED and self.localization_auto_paused:
            self.publish_status_update(
                "navigation_localization_recovery_started",
                self.build_localization_pause_context(status)
            )
            self.request_navigation_context_recovery_for_localization(reason, status)

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
        self.localization_recovery_done = False
        self.localization_resume_pending = False
        self.localization_resume_stable_count = 0
        self.localization_recovery_reason = status.get("reason", self.localization_recovery_reason)
        self.localization_recovered_at = 0.0
        self.publish_status_update("navigation_localization_recovery_failed", {
            "reason": self.localization_recovery_reason,
            "current_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "current_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "auto_resume_pending": self.localization_auto_paused,
        })

    def handle_localization_manual_initialpose_override(self, status: Dict[str, Any]):
        self.localization_recovery_active = False
        self.localization_recovery_done = False
        self.localization_resume_pending = False
        self.localization_resume_stable_count = 0
        self.localization_recovery_reason = status.get("reason", "人工重定位已接管")

        self.publish_status_update("navigation_localization_manual_override", {
            "reason": self.localization_recovery_reason,
            "localization_event": status.get("event_type", ""),
            "manual_lockout_sec": status.get("manual_lockout_sec", 0.0),
            "current_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
            "current_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "auto_resume_pending": self.localization_auto_paused,
        })

    def handle_localization_recovered(self, status: Dict[str, Any]):
        self.localization_recovery_active = False
        self.localization_recovery_done = True
        self.localization_resume_stable_count = 0
        self.localization_recovery_reason = status.get("reason", "定位已恢复")
        self.localization_recovered_at = time.time()

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

        if self.localization_auto_resume_require_recovery_done and (
                self.localization_recovery_active or not self.localization_recovery_done):
            self.publish_status_update("navigation_localization_resume_waiting", {
                "reason": "定位恢复流程尚未确认完成，暂不恢复导航",
                "localization_event": self.localization_recovery_last_status.get("event_type", ""),
                "current_waypoint_id": self.current_waypoint.get("id", "") if self.current_waypoint else "",
                "current_waypoint_name": self.current_waypoint.get("name", "") if self.current_waypoint else "",
                "waypoint_index": self.current_waypoint_index,
                "total_waypoints": self.total_waypoints,
            })
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

        required_stable_frames = max(0, self.localization_resume_stable_frames)
        if (not self._is_localization_healthy or
                self.localization_resume_stable_count < required_stable_frames):
            self.publish_status_update("navigation_localization_resume_waiting", {
                "reason": (
                    f"等待 prior-map 定位稳定帧 "
                    f"{self.localization_resume_stable_count}/{required_stable_frames}"
                ),
                "localization_stable_count": self.localization_resume_stable_count,
                "required_localization_stable_frames": required_stable_frames,
                "localization_status": self.last_localization_status_summary,
                "current_waypoint_id": self.current_waypoint.get("id", ""),
                "current_waypoint_name": self.current_waypoint.get("name", ""),
                "waypoint_index": self.current_waypoint_index,
                "total_waypoints": self.total_waypoints,
            })
            return

        settle_remaining = (
            self.localization_recovered_at +
            max(0.0, self.localization_resume_settle_sec) -
            time.time()
        )
        if settle_remaining > 0.0:
            self.publish_status_update("navigation_localization_resume_waiting", {
                "reason": f"定位恢复后等待 prior-map/代价地图稳定 {settle_remaining:.1f}s",
                "settle_remaining_sec": round(settle_remaining, 2),
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
        reverse_resume, reverse_context = self.should_reverse_resume_to_current_waypoint()
        event_data.update(reverse_context)

        # ★ Phase 3: Waypoint 保留强化
        # 1. 清除 waypoint 到达标志 — 防止 recovery 期间 pose 跳变导致误判到达
        self.waypoint_arrived_by_position = False
        self.waypoint_arrived_locked = False
        # 2. 验证 current_waypoint 数据完整性
        if not isinstance(self.current_waypoint, dict) or "position" not in self.current_waypoint:
            self.get_logger().error(
                '[prior_map] current_waypoint 数据损坏, 无法恢复导航')
            self.localization_resume_pending = False
            self.localization_auto_paused = False
            self.localization_stop_until = 0.0
            self.send_acknowledgment(
                "navigation_resumed", "error",
                "无法恢复导航: waypoint 数据损坏, 请联系管理员")
            self.reset_navigation_state()
            return

        self.localization_resume_pending = False
        self.localization_auto_paused = False
        self.localization_stop_until = 0.0
        self.current_state = NavigationState.EXECUTING
        self.current_detailed_state = "EXECUTING"

        self.send_acknowledgment(
            "navigation_auto_resumed",
            "success",
            "定位已恢复，继续未完成导航",
            event_data
        )
        self.publish_status_update("navigation_resumed", event_data)
        self.navigate_to_waypoint(
            self.current_waypoint,
            force_walk_direction="backward" if reverse_resume else None,
        )

        if reverse_resume:
            self.get_logger().info(
                f"定位恢复后当前点在身后不远，使用倒走回补: "
                f"{self.current_waypoint.get('name', '')} "
                f"({self.current_waypoint_index + 1}/{self.total_waypoints}), "
                f"distance={reverse_context.get('reverse_resume_distance_m', 0.0):.2f}m"
            )
        else:
            self.get_logger().info(
                f"定位恢复后自动继续导航: {self.current_waypoint.get('name', '')} "
                f"({self.current_waypoint_index + 1}/{self.total_waypoints})"
            )

    def should_reverse_resume_to_current_waypoint(self) -> Tuple[bool, Dict[str, Any]]:
        context: Dict[str, Any] = {
            "reverse_resume_selected": False,
            "reverse_resume_reason": "",
        }
        if not self.localization_resume_reverse_enabled:
            context["reverse_resume_reason"] = "disabled"
            return False, context
        if not self.current_pose or not self.current_waypoint:
            context["reverse_resume_reason"] = "missing current pose or waypoint"
            return False, context

        current_position = self.waypoint_position_tuple(self.current_waypoint)
        if current_position is None:
            context["reverse_resume_reason"] = "current waypoint has no valid position"
            return False, context

        dx = float(current_position[0]) - float(self.current_pose.position.x)
        dy = float(current_position[1]) - float(self.current_pose.position.y)
        distance = math.hypot(dx, dy)
        context["reverse_resume_distance_m"] = round(distance, 3)
        if distance <= max(self.position_tolerance, 0.05):
            context["reverse_resume_reason"] = "already within waypoint tolerance"
            return False, context
        if distance > max(0.0, self.localization_resume_reverse_max_distance_m):
            context["reverse_resume_reason"] = (
                f"waypoint too far for reverse resume: {distance:.2f}m"
            )
            return False, context

        robot_yaw = yaw_from_pose(self.current_pose)
        target_bearing = math.atan2(dy, dx)
        rear_error = abs(normalize_angle(target_bearing - robot_yaw - math.pi))
        context["reverse_resume_rear_angle_deg"] = round(math.degrees(rear_error), 1)
        if rear_error > max(0.0, self.localization_resume_reverse_rear_angle_rad):
            context["reverse_resume_reason"] = (
                f"waypoint is not behind robot: rear_error={math.degrees(rear_error):.1f}deg"
            )
            return False, context

        previous_waypoint, _, previous_source = self.resolve_previous_waypoint_context(time.time())
        previous_position = self.waypoint_position_tuple(previous_waypoint)
        overshot_segment = False
        raw_projection = None
        if previous_position is not None:
            seg_dx = float(current_position[0]) - float(previous_position[0])
            seg_dy = float(current_position[1]) - float(previous_position[1])
            seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
            if seg_len_sq > 1e-6:
                raw_projection = (
                    ((float(self.current_pose.position.x) - float(previous_position[0])) * seg_dx +
                     (float(self.current_pose.position.y) - float(previous_position[1])) * seg_dy) /
                    seg_len_sq
                )
                overshot_segment = raw_projection > 1.0

        context.update({
            "reverse_resume_selected": True,
            "reverse_resume_reason": "current waypoint is behind robot after localization recovery",
            "reverse_resume_overshot_segment": overshot_segment,
            "reverse_resume_projection_ratio": (
                round(raw_projection, 3) if raw_projection is not None else None
            ),
            "reverse_resume_previous_source": previous_source,
            "reverse_resume_behavior_tree": self.reverse_navigation_bt_xml,
        })
        return True, context

    def publish_zero_cmd_vel(self):
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception as e:
            self.get_logger().error(f"发布零速度失败: {e}")

    def begin_localization_stop_hold(self):
        now = time.time()
        self.localization_stop_until = max(
            self.localization_stop_until,
            now + max(0.0, self.localization_stop_hold_sec)
        )
        # P1-1: 连续发 3 帧零速度 (间隔 0.01s), 压制 Nav2 controller 的残余非零 cmd_vel
        for _ in range(3):
            self.publish_zero_cmd_vel()
            time.sleep(0.01)

    def enforce_localization_stop(self):
        now = time.time()
        localization_pause_active = (
            self.localization_auto_paused and
            self.current_state == NavigationState.PAUSED and
            self.current_detailed_state == "LOCALIZATION_RECOVERY"
        )
        if localization_pause_active or now < self.localization_stop_until:
            self.publish_zero_cmd_vel()

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
            "resume_mode": "auto",
            "waiting_for_obstacle_clear": False,
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

    def get_waypoint_walk_direction(self, waypoint_data: Dict[str, Any]) -> str:
        """读取点位行走方向。默认正走；properties.walk_direction=backward 时倒走。"""
        properties = waypoint_data.get("properties", {}) or {}
        direction = (
            properties.get("walk_direction")
            or properties.get("navigation_direction")
            or properties.get("drive_direction")
            or properties.get("motion_direction")
            or waypoint_data.get("walk_direction")
            or "forward"
        )

        if isinstance(direction, bool):
            return "backward" if direction else "forward"

        normalized = str(direction).strip().lower()
        if normalized in {"backward", "reverse", "back", "倒走", "倒车", "后退"}:
            return "backward"
        return "forward"
    
    def navigate_to_waypoint(self, waypoint_data: Dict[str, Any], force_walk_direction: Optional[str] = None):
        """导航到指定路点"""
        try:
            self.current_waypoint = waypoint_data
            self.current_state = NavigationState.EXECUTING
            self.current_waypoint_start_time = time.time()
            
            # 创建导航目标
            goal_pose = self.waypoint_to_pose_stamped(waypoint_data)
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose
            walk_direction = force_walk_direction or self.get_waypoint_walk_direction(waypoint_data)
            if walk_direction == "backward":
                goal_msg.behavior_tree = self.reverse_navigation_bt_xml
        
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
                "position": waypoint_data.get("position", []),
                "walk_direction": walk_direction,
                "behavior_tree": goal_msg.behavior_tree
            })
            
            self.get_logger().info(
                f"开始导航到路点: {waypoint_data.get('name', '')} "
                f"({self.current_waypoint_index + 1}/{self.total_waypoints}), "
                f"walk_direction={walk_direction}"
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

    def process_obstacle_wait_state(self):
        """处理“因动态障碍物暂停等待”的周期逻辑。"""
        if not self.obstacle_wait_active:
            return
        if self.current_state != NavigationState.PAUSED:
            return

        now = time.time()
        block_duration = now - self.obstacle_wait_started_at if self.obstacle_wait_started_at > 0.0 else 0.0

        # 障碍阻塞事件只在首次进入 obstacle_wait 时推一次。
        # APP 当前会把带播报词的 navigation_obstacle_blocked 直接播出来，
        # 如果这里周期重复推送，就会造成“同一次受阻重复播报”。
        #
        # 等待中的持续状态改由周期 navigation_status 承担：
        # obstacle_wait_active / obstacle_wait_duration / pause_source
        # 会持续反映“仍在等待障碍清除”，但不再重复触发提示音。

        # costmap 太久没更新时，不做 clear 判定，避免用陈旧数据误恢复导航。
        max_costmap_age = max(1.0, 2.0 / self.obstacle_clear_check_rate_hz)
        if now - self.latest_local_costmap_stamp > max_costmap_age:
            self.obstacle_clear_confirm_count = 0
            self.obstacle_clear_started_at = 0.0
            return

        if self.latest_front_obstacle_blocked:
            self.obstacle_clear_confirm_count = 0
            self.obstacle_clear_started_at = 0.0
            return

        roi_clear, roi_stats = self.is_roi_obstacle_clear_for_resume(now)
        if not roi_clear:
            # ROI 直接来自点云，优先用它确认“人/障碍物确实离开了正前方”。
            # ROI 还没 clear 时，不累计 costmap clear 时间，避免两套检测窗口不同步导致误恢复。
            self.obstacle_clear_confirm_count = 0
            self.obstacle_clear_started_at = 0.0
            self.get_logger().info(
                f"ROI 障碍检测尚未 clear，继续暂停等待: {roi_stats}",
                throttle_duration_sec=1.0,
            )
            return

        self.obstacle_clear_confirm_count += 1
        if self.obstacle_clear_confirm_count < self.obstacle_clear_required_frames:
            return

        if block_duration < self.obstacle_min_wait_before_resume_sec:
            self.get_logger().info(
                f"障碍等待已 clear，但暂停时间 {block_duration:.1f}s "
                f"< 最短等待 {self.obstacle_min_wait_before_resume_sec:.1f}s，继续观察",
                throttle_duration_sec=1.0,
            )
            return

        if self.obstacle_clear_started_at <= 0.0:
            self.obstacle_clear_started_at = now
            return

        clear_duration = now - self.obstacle_clear_started_at
        required_clear_duration = self.get_current_obstacle_clear_required_duration()
        if clear_duration < required_clear_duration:
            self.get_logger().info(
                f"障碍 clear 持续 {clear_duration:.1f}s "
                f"< 要求 {required_clear_duration:.1f}s，继续等待",
                throttle_duration_sec=1.0,
            )
            return

        self.resume_navigation_from_obstacle_wait()

    def resume_navigation_from_obstacle_wait(self):
        """障碍物消失后自动恢复到当前 waypoint。"""
        if not self.obstacle_wait_active:
            return
        if self.current_state != NavigationState.PAUSED:
            return
        if not self.current_waypoint:
            self.get_logger().warning("障碍物已清除，但当前 waypoint 丢失，无法自动恢复导航")
            self.clear_obstacle_wait_state()
            self.reset_navigation_state()
            return

        pause_elapsed = time.time() - self.pause_time if self.pause_time else 0.0
        self.clear_obstacle_wait_state()
        self.current_state = NavigationState.EXECUTING
        self.current_detailed_state = "EXECUTING"
        self.current_pause_source = ""
        self.current_pause_reason = ""
        self.current_resume_mode = ""

        event_data = {
            "resumed_waypoint_id": self.current_waypoint.get("id", ""),
            "resumed_waypoint_name": self.current_waypoint.get("name", ""),
            "waypoint_index": self.current_waypoint_index,
            "total_waypoints": self.total_waypoints,
            "pause_duration_actual": round(pause_elapsed, 1),
            "resume_reason": "obstacle_cleared_auto_resume",
            "resume_source": "obstacle_wait",
        }
        if self.latest_front_obstacle_stats:
            event_data["front_obstacle_stats"] = self.latest_front_obstacle_stats
        event_data["roi_obstacle_stats"] = self.build_roi_obstacle_stats()

        self.send_acknowledgment("navigation_resumed", "success", "障碍物已消失，导航自动恢复", event_data)
        self.publish_status_update("navigation_resumed", event_data)
        self.last_obstacle_resume_time = time.time()
        self.navigate_to_waypoint(self.current_waypoint)
        self.get_logger().info("前方 ROI 与 costmap 已连续 clear，自动恢复当前导航目标")
    
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
            self.record_last_succeeded_waypoint(self.current_waypoint, self.current_waypoint_index)
            
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
        if self.try_enter_obstacle_wait_from_nav_failure("Nav2导航失败"):
            return
        self.handle_navigation_failed("Nav2导航失败")

    def try_enter_obstacle_wait_from_nav_failure(self, reason: str) -> bool:
        """Nav2/RPP 执行阶段失败时，优先交给动态障碍等待状态机接管。"""
        if not self.obstacle_wait_enable or self.obstacle_wait_active:
            return False
        if self.current_state not in (NavigationState.EXECUTING, NavigationState.PLANNING):
            return False
        if not self.current_waypoint:
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
    
    def handle_nav2_cancelled(self):
        """处理Nav2取消"""
        if self.current_state == NavigationState.PAUSED:
            if self.localization_auto_paused:
                self.get_logger().info("Nav2 取消是由定位恢复自动暂停触发，忽略重置操作")
            elif self.obstacle_wait_active:
                self.get_logger().info("Nav2 取消是由障碍等待暂停触发，忽略重置操作")
            else:
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

    @staticmethod
    def pose_orientation_dict(orientation: Any) -> Dict[str, float]:
        if isinstance(orientation, dict):
            return {
                "x": float(orientation.get("x", 0.0)),
                "y": float(orientation.get("y", 0.0)),
                "z": float(orientation.get("z", 0.0)),
                "w": float(orientation.get("w", 1.0)),
            }
        if isinstance(orientation, (list, tuple)) and len(orientation) >= 4:
            return {
                "x": float(orientation[0]),
                "y": float(orientation[1]),
                "z": float(orientation[2]),
                "w": float(orientation[3]),
            }
        return {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}

    @staticmethod
    def quaternion_from_yaw(yaw: float) -> Dict[str, float]:
        half_yaw = yaw * 0.5
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(half_yaw),
            "w": math.cos(half_yaw),
        }

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

    def waypoint_context_dict(self, waypoint: Optional[Dict[str, Any]], index: int) -> Optional[Dict[str, Any]]:
        position = self.waypoint_position_tuple(waypoint)
        if not waypoint or position is None:
            return None
        return {
            "id": waypoint.get("id", ""),
            "name": waypoint.get("name", ""),
            "index": index,
            "frame_id": waypoint.get("frame_id", self.default_frame_id),
            "position": {
                "x": position[0],
                "y": position[1],
                "z": position[2],
            },
        }

    def resolve_previous_waypoint_context(self, now: float):
        if self.waypoint_ids and self.current_waypoint_index > 0:
            previous_index = self.current_waypoint_index - 1
            previous_waypoint = self.find_waypoint_data_by_id(self.waypoint_ids[previous_index])
            if previous_waypoint:
                return previous_waypoint, previous_index, "current_sequence_previous"

        if not self.last_succeeded_waypoint:
            return None, -1, "none"

        current_id = self.current_waypoint.get("id", "") if self.current_waypoint else ""
        previous_id = self.last_succeeded_waypoint.get("id", "")
        if current_id and previous_id and current_id == previous_id:
            return None, -1, "none"

        age = now - self.last_succeeded_time if self.last_succeeded_time > 0.0 else float("inf")
        max_age = self.localization_context_prior_max_previous_age_sec
        if max_age > 0.0 and age > max_age:
            return None, -1, "last_succeeded_too_old"

        return self.last_succeeded_waypoint, self.last_succeeded_waypoint_index, "last_succeeded_waypoint"

    def resolve_next_waypoint_context(self):
        if not self.waypoint_ids:
            return None, -1
        next_index = self.current_waypoint_index + 1
        if next_index < 0 or next_index >= len(self.waypoint_ids):
            return None, -1
        next_waypoint = self.find_waypoint_data_by_id(self.waypoint_ids[next_index])
        return next_waypoint, next_index

    def build_navigation_context_recovery_request(
        self,
        reason: str,
        trigger_event: str,
        radius_m: float,
        status: Optional[Dict[str, Any]] = None,
        event_type: str = "navigation_context_recovery_request",
    ) -> Optional[Dict[str, Any]]:
        current_waypoint = self.current_waypoint
        current_position = self.waypoint_position_tuple(current_waypoint)
        if not current_waypoint or current_position is None:
            return None

        now = time.time()
        previous_waypoint, previous_index, previous_source = self.resolve_previous_waypoint_context(now)
        previous_position = self.waypoint_position_tuple(previous_waypoint)
        next_waypoint, next_index = self.resolve_next_waypoint_context()

        selected_prior_source = "navigation_context_current_goal"
        selected_prior = {
            "position": {
                "x": current_position[0],
                "y": current_position[1],
                "z": current_position[2],
            },
            "orientation": self.pose_orientation_dict(
                current_waypoint.get("orientation", [0.0, 0.0, 0.0, 1.0])
            ),
        }
        selected_prior_meta = {
            "method": "current_goal",
            "previous_source": previous_source,
        }

        if previous_position is not None:
            dx = current_position[0] - previous_position[0]
            dy = current_position[1] - previous_position[1]
            dz = current_position[2] - previous_position[2]
            segment_length = math.hypot(dx, dy)

            if segment_length >= max(0.0, self.localization_context_prior_min_segment_length_m):
                if self.current_pose:
                    reference_x = float(self.current_pose.position.x)
                    reference_y = float(self.current_pose.position.y)
                else:
                    reference_x = current_position[0]
                    reference_y = current_position[1]

                projection = (
                    ((reference_x - previous_position[0]) * dx + (reference_y - previous_position[1]) * dy) /
                    max(segment_length * segment_length, 1e-6)
                )
                projection_clamped = max(0.0, min(1.0, projection))
                prior_x = previous_position[0] + projection_clamped * dx
                prior_y = previous_position[1] + projection_clamped * dy
                prior_z = previous_position[2] + projection_clamped * dz
                prior_yaw = math.atan2(dy, dx)

                selected_prior_source = "navigation_context_segment"
                selected_prior = {
                    "position": {
                        "x": prior_x,
                        "y": prior_y,
                        "z": prior_z,
                    },
                    "orientation": self.quaternion_from_yaw(prior_yaw),
                }
                selected_prior_meta = {
                    "method": "projected_previous_to_current_segment",
                    "previous_source": previous_source,
                    "segment_length_m": round(segment_length, 3),
                    "projection_ratio": round(projection_clamped, 3),
                    "raw_projection_ratio": round(projection, 3),
                    "reference_pose_source": "current_map_pose" if self.current_pose else "current_goal",
                }

        radius_m = max(0.0, float(radius_m))
        current_context = self.waypoint_context_dict(current_waypoint, self.current_waypoint_index)
        previous_context = self.waypoint_context_dict(previous_waypoint, previous_index)
        next_context = self.waypoint_context_dict(next_waypoint, next_index)

        payload = {
            "event_type": event_type,
            "source": "navigation_state_manager",
            "reason": reason,
            "timestamp": now,
            "trigger_event": trigger_event,
            "prior_source": selected_prior_source,
            "prior_frame_id": current_waypoint.get("frame_id", self.default_frame_id),
            "prior_pose": selected_prior,
            "search_radius_m": radius_m,
            "prior_max_xy_m": radius_m,
            "allow_full_global_fallback": True,
            "failed_waypoint_index": self.current_waypoint_index,
            "failed_waypoint_id": current_waypoint.get("id", ""),
            "failed_waypoint_name": current_waypoint.get("name", ""),
            "current_pose": self.pose_to_dict(self.current_pose) if self.current_pose else None,
            "current_detailed_state": self.current_detailed_state,
            "navigation_context": {
                "current_waypoint": current_context,
                "previous_waypoint": previous_context,
                "next_waypoint": next_context,
                "selected_prior": selected_prior_meta,
                "current_sequence_id": self.current_sequence_id,
                "navigation_mode": self.current_navigation_mode.value if self.current_navigation_mode else None,
                "total_waypoints": self.total_waypoints,
            },
        }

        if status:
            payload["localization_status"] = {
                "event_type": status.get("event_type", ""),
                "recovery_count": status.get("recovery_count", 0),
                "relocalize_attempts": status.get("relocalize_attempts", 0),
                "prior_reason": status.get("prior_reason", ""),
            }

        return payload

    def publish_localization_recovery_request(self, payload: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.localization_recovery_request_pub.publish(msg)
        self.publish_status_update("navigation_localization_recovery_requested", payload)

    def request_navigation_context_recovery_for_localization(self, reason: str, status: Dict[str, Any]):
        """定位异常触发重定位时，把当前导航上下文作为动态先验发给 HDL。"""
        if not self.request_navigation_context_recovery_on_localization_failure:
            return
        if not self.current_waypoint:
            return

        now = time.time()
        current_id = self.current_waypoint.get("id", "")
        recovery_count = status.get("recovery_count", 0) if isinstance(status, dict) else 0
        request_key = f"{self.current_sequence_id}:{self.current_waypoint_index}:{current_id}:{recovery_count}"
        if request_key == self.last_navigation_context_recovery_key:
            return

        if (
            now - self.last_navigation_context_recovery_request_time <
            self.localization_context_recovery_request_cooldown_sec
        ):
            self.get_logger().warning(
                "定位异常上下文恢复请求仍在冷却中，跳过本次触发",
                throttle_duration_sec=3.0,
            )
            return

        payload = self.build_navigation_context_recovery_request(
            reason=reason,
            trigger_event="localization_failure",
            radius_m=self.localization_context_prior_radius_m,
            status=status,
            event_type="localization_failure_navigation_context_recovery_request",
        )
        if not payload:
            return

        self.publish_localization_recovery_request(payload)
        self.last_navigation_context_recovery_request_time = now
        self.last_navigation_context_recovery_key = request_key

        context = payload.get("navigation_context", {})
        previous_name = (context.get("previous_waypoint") or {}).get("name", "")
        current_name = (context.get("current_waypoint") or {}).get("name", "")
        selected = context.get("selected_prior", {})
        self.get_logger().warning(
            f"定位异常后请求 HDL 使用导航上下文重定位: "
            f"prior={payload['prior_source']}, prev={previous_name}, current={current_name}, "
            f"method={selected.get('method', '')}, 半径={payload['prior_max_xy_m']:.1f}m, reason={reason}"
        )

    def record_last_succeeded_waypoint(self, waypoint: Dict[str, Any], index: int):
        self.last_succeeded_waypoint = dict(waypoint)
        self.last_succeeded_waypoint_index = index
        self.last_succeeded_pose = self.pose_to_dict(self.current_pose) if self.current_pose else None
        self.last_succeeded_time = time.time()

    def request_localization_recovery_for_failed_navigation(self, reason: str):
        """导航失败后按需唤醒 HDL，全局重定位优先搜索失败目标点附近。"""
        if not self.request_localization_recovery_on_nav_failure:
            return
        if not self.current_waypoint:
            return

        now = time.time()
        if now - self.last_localization_recovery_request_time < self.localization_recovery_request_cooldown_sec:
            self.get_logger().warning(
                "定位恢复请求仍在冷却中，跳过本次导航失败触发",
                throttle_duration_sec=3.0,
            )
            return

        payload = self.build_navigation_context_recovery_request(
            reason=reason,
            trigger_event="navigation_failure",
            radius_m=self.localization_recovery_prior_radius_m,
            event_type="navigation_failure_recovery_request",
        )
        if not payload:
            return

        self.publish_localization_recovery_request(payload)
        self.last_localization_recovery_request_time = now
        context = payload.get("navigation_context", {})
        selected = context.get("selected_prior", {})
        self.get_logger().warning(
            f"导航失败后请求 HDL 按需重定位: 先验={payload['prior_source']} "
            f"目标={payload['failed_waypoint_name']}, method={selected.get('method', '')}, "
            f"半径={payload['prior_max_xy_m']:.1f}m, reason={reason}"
        )

    @staticmethod
    def pose_to_dict(pose) -> Dict[str, Any]:
        return {
            "position": {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(pose.position.z),
            },
            "orientation": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
        }
    
    def handle_navigation_failed(self, reason: str):
        """处理导航失败"""
        # 如果当前状态是 PAUSED(暂停)，说明是我们为了互动主动取消的，不要标记为失败，也不要重置数据
        if self.current_state == NavigationState.PAUSED:
            if self.localization_auto_paused:
                self.get_logger().info("检测到导航由定位恢复自动暂停，保留数据以备恢复...")
            elif self.obstacle_wait_active:
                self.get_logger().info("检测到导航由障碍等待自动暂停，保留数据以备障碍物消失后恢复...")
            else:
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
        self.request_localization_recovery_for_failed_navigation(reason)
        
        self.get_logger().error(f"导航失败，进入可恢复失败状态: {reason}")
    
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
            "robot_action_interlock_active": self.robot_action_interlock_active,
            "robot_action_interlock_paused_navigation": self.robot_action_interlock_paused_navigation,
            "robot_action_resume_pending": self.robot_action_resume_pending,
            "failure_recoverable": self.current_state == NavigationState.RECOVERABLE_FAILED,
            "failure_context": self.last_failure_context,
            "skipped_waypoints": self.skipped_waypoints,
            "localization_recovery_active": self.localization_recovery_active,
            "localization_auto_paused": self.localization_auto_paused,
            "localization_resume_pending": self.localization_resume_pending,
            "localization_recovery_reason": self.localization_recovery_reason,
            "localization_recovery_status": self.localization_recovery_last_status,
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
        self.current_detailed_state = "IDLE"
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
            print(f"导航状态管理器启动失败: {e}")
    finally:
        # 节点可能在 ROS participant 创建阶段就失败，此时不能再访问未创建的 node。
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
