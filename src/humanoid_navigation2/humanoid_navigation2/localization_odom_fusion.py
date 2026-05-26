#!/usr/bin/env python3
"""
localization_odom_fusion.py — NDT定位 + 里程计融合节点

================================================================================
功能说明
================================================================================

当 NDT 定位因环境几何混叠而漂移时（matching_error 飙升），冻结 map->odom，
让机器人依靠 Fast-LIO 里程计（camera_init->body）继续在位姿上运动。
NDT 恢复后平滑切回。

状态机:
  HEALTHY ──matching_error > 0.5──▶ DEGRADED
  DEGRADED──matching_error < 0.15 连续3帧──▶ TRANSITIONING
  TRANSITIONING──平滑过渡完成──▶ HEALTHY
  DEGRADED──odom位移>30m 或 持续>120s──▶ LOST
  LOST──recovery 成功──▶ HEALTHY

================================================================================
核心原理
================================================================================

NDT 定位的本质是计算 map_T_odom（map到odom坐标系的变换）。
在理想情况下 map_T_odom 是恒定的——odom 原点（camera_init）一旦确定就不会移动。

当 NDT 漂移时:
  - 错误行为: map_T_odom 跳变 5-13m → robot_map 位置瞬移 → 下游崩溃
  - 融合行为: 冻结 map_T_odom → odom 驱动机器人运动 → robot_map 保持正确

TF 树:
  map → odom → camera_init → body → base_footprint
            ↑ 融合节点发布      ↑ Fast-LIO 发布  ↑ 静态TF

================================================================================
作者: Claude Opus 4.7
日期: 2026-05-25
================================================================================
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Float64
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster

# ── 四元数工具函数 ──────────────────────────────────────────────────────────
def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """从四元数提取 yaw 角（绕 Z 轴旋转）"""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def quat_from_yaw(yaw: float):
    """从 yaw 角生成四元数 (x, y, z, w)"""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quat_slerp(q1, q2, t):
    """
    四元数球面线性插值（slerp）
    q1, q2: (x, y, z, w) 元组
    t: 0→1 插值系数
    返回: (x, y, z, w)
    """
    q1 = np.array(q1)
    q2 = np.array(q2)

    # 确保走最短路径
    dot = np.dot(q1, q2)
    if dot < 0.0:
        q2 = -q2
        dot = -dot

    dot = np.clip(dot, -1.0, 1.0)
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)

    if sin_theta_0 < 1e-6:
        # 角度太小，退化为线性插值
        result = q1 * (1.0 - t) + q2 * t
        result = result / np.linalg.norm(result)
        return tuple(result)

    s0 = math.sin((1.0 - t) * theta_0) / sin_theta_0
    s1 = math.sin(t * theta_0) / sin_theta_0
    result = q1 * s0 + q2 * s1
    return tuple(result)


def smoothstep(t: float) -> float:
    """
    Smoothstep 函数: 0→1 的 C2 连续平滑过渡
    公式: t²(3 - 2t)
    用于 DEGRADED→HEALTHY 过渡时避免位姿"瞬移"
    """
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


# ── 坐标系常量 ──────────────────────────────────────────────────────────────
FRAME_MAP = 'map'
FRAME_ODOM = 'odom'
FRAME_CAMERA_INIT = 'camera_init'
FRAME_BODY = 'body'
FRAME_BASE_FOOTPRINT = 'base_footprint'

# ── 状态枚举 ────────────────────────────────────────────────────────────────
class FusionState:
    """融合节点状态机枚举"""
    INITIALIZING = 'INITIALIZING'   # 启动阶段，等待首次 NDT/SC 定位成功
    HEALTHY = 'HEALTHY'             # NDT 正常，直通转发
    DEGRADED = 'DEGRADED'           # NDT 退化，冻结 map->odom + odom 传播
    TRANSITIONING = 'TRANSITIONING' # 从 DEGRADED 平滑过渡回 HEALTHY
    LOST = 'LOST'                   # 长时间退化，触发 recovery


class LocalizationOdomFusion(Node):
    """
    NDT定位 + 里程计融合节点

    核心职责:
    1. 监控 NDT 定位健康状态（通过 /localization/ndt_status）
    2. HEALTHY 时直通 NDT 的 map->odom TF
    3. DEGRADED 时冻结 map->odom，让 Fast-LIO 里程计驱动机器人运动
    4. NDT 恢复后平滑过渡切回，避免位姿跳变
    """

    def __init__(self):
        super().__init__('localization_odom_fusion')

        # =====================================================================
        # 参数声明（所有阈值均可通过 launch 文件覆盖）
        # =====================================================================

        # ── 状态转换阈值 ──
        # 进入 DEGRADED 的 NDT matching_error 阈值
        # 高于此值认为 NDT 定位不可靠，冻结 map->odom
        self.degraded_error_threshold = float(
            self.declare_parameter('degraded_error_threshold', 0.5).value)

        # 恢复 HEALTHY 的 NDT matching_error 阈值
        # 低于此值认为 NDT 定位已恢复
        self.healthy_error_threshold = float(
            self.declare_parameter('healthy_error_threshold', 0.15).value)

        # 恢复 HEALTHY 需要连续满足 healthy 条件的次数
        # 防止 NDT 短暂恢复后马上又退化（振荡保护）
        self.healthy_consecutive_frames = int(
            self.declare_parameter('healthy_consecutive_frames', 3).value)

        # 进入 DEGRADED 需要连续满足 degraded 条件的次数
        # 防止 NDT 短暂抖动就触发冻结（防抖保护）
        self.degraded_consecutive_frames = int(
            self.declare_parameter('degraded_consecutive_frames', 2).value)

        # ── 超时和距离限制 ──
        # 最长 DEGRADED 持续时间（秒），超时进入 LOST 触发 recovery
        self.max_degraded_duration_sec = float(
            self.declare_parameter('max_degraded_duration_sec', 120.0).value)

        # 最大单段 odom 位移（米），每到一个路点重置
        # 防止单段导航中 odom 漂移过大（一段最长约30m走到下一个点）
        self.max_odom_displacement_m = float(
            self.declare_parameter('max_odom_displacement_m', 30.0).value)

        # ★ 累计 odom 位移上限（米），从进入 DEGRADED 算起，永不重置
        # Fast-LIO 漂移率约 0.5cm/m，100m → 50cm 误差，是导航可接受的极限
        # 超过此值即使单段没超标也要进入 LOST，防止跨路点累积漂移
        self.max_total_odom_displacement_m = float(
            self.declare_parameter('max_total_odom_displacement_m', 100.0).value)

        # ── 导航感知超时（★ 关键: 区分导航中 vs 已到达静止）──
        # 导航中（EXECUTING/PLANNING）: LOST 超时较短，定位不准会影响导航
        self.nav_active_lost_timeout_sec = float(
            self.declare_parameter('nav_active_lost_timeout_sec', 120.0).value)

        # 已到达/静止（IDLE/COMPLETED/PAUSED）: LOST 超时很长
        # 因为机器人正在播报讲解词，不需要定位精度，等播报完再说
        self.nav_idle_lost_timeout_sec = float(
            self.declare_parameter('nav_idle_lost_timeout_sec', 600.0).value)

        # 静止时极端 NDT error 阈值: 超过此值即使静止也触发 LOST
        # 因为 NDT 已经彻底挂了，不是短暂的几何混叠
        self.nav_idle_extreme_error = float(
            self.declare_parameter('nav_idle_extreme_error', 5.0).value)

        # 导航状态 topic (由 navigation_state_manager 发布)
        self.nav_status_topic = str(
            self.declare_parameter('nav_status_topic', '/navigation_status').value)

        # LOST / INITIALIZING 时请求 recovery 的冷却时间（秒），防止重复请求
        self.recovery_request_cooldown_sec = float(
            self.declare_parameter('recovery_request_cooldown_sec', 15.0).value)

        # INITIALIZING 状态超时（秒），超时后主动请求 SC recovery
        # 防止启动阶段死锁: SC bridge 等 fusion → fusion 等 NDT → NDT 等 SC bridge
        self.init_timeout_sec = float(
            self.declare_parameter('init_timeout_sec', 20.0).value)

        # ── LOST recovery 软验收参数 ──
        # 只拦截明显离谱的重定位结果；不使用 yaw 硬阈值，避免 odom/冻结TF 误差导致正确恢复被拒。
        self.recovery_pose_soft_gate_enabled = bool(
            self.declare_parameter('recovery_pose_soft_gate_enabled', True).value)
        self.recovery_pose_max_xy_error_m = float(
            self.declare_parameter('recovery_pose_max_xy_error_m', 5.0).value)
        self.recovery_pose_accept_if_ndt_error_below = float(
            self.declare_parameter('recovery_pose_accept_if_ndt_error_below', 0.03).value)
        self.recovery_pose_skip_odom_after_displacement_m = float(
            self.declare_parameter('recovery_pose_skip_odom_after_displacement_m', 20.0).value)
        self.recovery_pose_max_status_age_sec = float(
            self.declare_parameter('recovery_pose_max_status_age_sec', 2.0).value)
        self.recovery_pose_max_pcl_age_sec = float(
            self.declare_parameter('recovery_pose_max_pcl_age_sec', 2.0).value)

        # ── NDT pose jump 检测参数 ──
        # NDT 几何混叠场景下可能 fitness 极低但收敛到错误位置
        # (fitness=0.003 同时跳变 1.4m)。以下参数在 fitness 判据之外
        # 增加对 NDT 位姿跳变的感知，防止保护链被旁路。
        self.pose_jump_degraded_from_status = bool(
            self.declare_parameter('pose_jump_degraded_from_status', True).value)
        # status reason 字段触发 DEGRADED 的模式:
        #   "pose_jump_candidate" — NDT 正在确认跳变（仍在重发布旧 pose）
        #   "confirmed_pose_jump" — NDT 已接受跳变
        self.pose_jump_degraded_from_pcl = bool(
            self.declare_parameter('pose_jump_degraded_from_pcl', True).value)
        self.pose_jump_pcl_threshold_m = float(
            self.declare_parameter('pose_jump_pcl_threshold_m', 0.5).value)
        # NDT correction_translation 直接触发 DEGRADED 的阈值（比 NDT 的 0.8 更敏感）
        self.pose_jump_correction_threshold_m = float(
            self.declare_parameter('pose_jump_correction_threshold_m', 0.5).value)

        # ── 平滑过渡参数 ──
        # DEGRADED→HEALTHY 平滑过渡时间（秒）
        # 在此时长内从 frozen_map_odom 插值到 ndt_current_map_odom
        self.transition_duration_sec = float(
            self.declare_parameter('transition_duration_sec', 2.0).value)

        # ── DEGRADED 锁定期参数 ──
        # 进入 DEGRADED 后的最短锁定期，此期间拒绝 NDT 恢复信号
        # 防止 NDT 快速 pose_jump→错误收敛→report ok→fusion 假恢复循环
        self.min_degraded_lock_sec = float(
            self.declare_parameter('min_degraded_lock_sec', 30.0).value)
        # DEGRADED 总超时（包含锁定期），超时→LOST→SC
        self.max_degraded_lock_sec = float(
            self.declare_parameter('max_degraded_lock_sec', 180.0).value)
        # 锁定期后恢复验证参数
        # 恢复时需连续健康的帧数（比默认的 3 更严格）
        self.lock_recovery_healthy_consecutive_frames = int(
            self.declare_parameter('lock_recovery_healthy_consecutive_frames', 10).value)
        # 恢复时允许的最大 NDT correction_translation (m)
        self.lock_recovery_max_correction_m = float(
            self.declare_parameter('lock_recovery_max_correction_m', 0.3).value)
        # 锁定期内 NDT 拒绝率超过此比例 → 提前 LOST（NDT 明显无法工作）
        self.lock_early_lost_rejection_rate = float(
            self.declare_parameter('lock_early_lost_rejection_rate', 0.9).value)
        # 锁定期内最少累积帧数后才允许提前 LOST
        self.lock_early_lost_min_frames = int(
            self.declare_parameter('lock_early_lost_min_frames', 30).value)
        # 恢复验证: frozen map→odom 与 NDT 恢复后的 map→odom 跳变超过此值 → 拒绝恢复
        self.recovery_pose_jump_max_m = float(
            self.declare_parameter('recovery_pose_jump_max_m', 5.0).value)

        # ── NDT inlier=0 虚假健康检测 (P0-2) ──
        # 长廊中 NDT 收敛到错误位置时 fitness 极低(<0.01) 但 inlier 始终为 0
        # 这是几何混叠的强特征，应加速触发 LOST 而非坐等 180s 超时
        self.inlier_zero_degraded_early_lost_sec = float(
            self.declare_parameter('inlier_zero_degraded_early_lost_sec', 30.0).value)
        self.inlier_zero_error_ceiling = float(
            self.declare_parameter('inlier_zero_error_ceiling', 0.01).value)
        # 追踪 inlier=0 的持续时间（进入 DEGRADED 后累加）
        self._inlier_zero_elapsed = 0.0
        self._inlier_zero_start = 0.0

        # ── DEGRADED 静止检测定时器重置 (P0-3) ──
        # 替代不可靠的 nav_state 变更检测。如果机器人在 DEGRADED 期间
        # 里程计位移在 5 秒内 <0.1m（即确实停在路点上），自动重置计时器
        self._odom_stationary_start = 0.0     # 静止开始时间
        self._odom_stationary_threshold_m = 0.1   # 5秒内位移阈值
        self._odom_stationary_duration_sec = 5.0  # 需要持续静止的时长
        self._last_odom_body_for_stationary = None  # 上次检查时的 odom_body

        # ── 发布参数 ──
        # 融合节点发布 map->odom TF 的频率（Hz）
        # 高于 lidar_localization 的发布频率（10Hz），确保 DEGRADED 时
        # 融合节点的 map->odom 被 tf2 采用
        self.publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 30.0).value)

        # ── 诊断参数 ──
        # 是否输出详细诊断日志
        self.verbose_logging = bool(
            self.declare_parameter('verbose_logging', True).value)

        # =====================================================================
        # TF 基础设施
        # =====================================================================

        # TF 缓冲区：用于查询 camera_init->body 等变换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF 广播器：用于发布 map->odom（在 DEGRADED/LOST 时）
        self.tf_broadcaster = TransformBroadcaster(self)

        # =====================================================================
        # 订阅 NDT 定位状态
        # =====================================================================

        # 订阅 NDT 扫描匹配状态（/localization/ndt_status）
        # 消息类型: std_msgs/String (JSON)
        # JSON 字段: state(accepted/rejected), has_converged(bool), fitness_score(float),
        #            correction_translation(float), correction_yaw(float)
        self.status_sub = self.create_subscription(
            String,
            '/localization/ndt_status',
            self._on_ndt_status,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        )

        # ★ 订阅 NDT 原始 map->odom (通过 topic 而非 TF, 避免融合节点自己发布的 TF 污染)
        # /pcl_pose 由 lidar_localization 发布，geometry_msgs/PoseWithCovarianceStamped
        # 其 position 字段即为 NDT 估计的 map->odom 平移量
        self.pcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pcl_pose',
            self._on_pcl_pose,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        )

        # 订阅 recovery 状态（用于监听 recovery 成功事件）
        self.recovery_status_sub = self.create_subscription(
            String,
            '/localization/recovery_status',
            self._on_recovery_status,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        )

        # 订阅导航状态（★ 用于区分导航中 vs 已到达静止）
        # navigation_state_manager 发布 /navigation_status topic
        # 格式: {"state": "EXECUTING"|"IDLE"|"COMPLETED"|"PAUSED"|...}
        self.nav_state = "IDLE"  # 默认空闲
        self.nav_status_sub = self.create_subscription(
            String,
            self.nav_status_topic,
            self._on_nav_status,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        )

        # =====================================================================
        # 发布融合状态诊断信息
        # =====================================================================

        # 融合状态发布（供外部监控）
        self.fusion_status_pub = self.create_publisher(
            String,
            '/localization/fusion_status',
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        )

        # 里程计累积位移发布（供监控）
        self.odom_displacement_pub = self.create_publisher(
            Float64,
            '/localization/fusion_odom_displacement',
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        )

        # ★ LOST 时主动请求 recovery（通过 /localization/recovery_requests 触发 HDL 重定位）
        self.recovery_request_pub = self.create_publisher(
            String,
            '/localization/recovery_requests',
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE),
        )

        # =====================================================================
        # 内部状态初始化
        # =====================================================================

        # 当前状态
        self.state = FusionState.INITIALIZING

        # 最新的 NDT 状态数据
        self.latest_ndt_error = float('inf')
        self.latest_ndt_inlier = 0.0
        self.latest_ndt_converged = False
        self.latest_ndt_state = ''  # "accepted" | "rejected" | "confirming"
        self.latest_ndt_status_time = 0.0

        # 最新的 NDT map->odom（从 TF 获取）
        self.latest_ndt_map_odom = None  # dict with keys: x, y, z, qx, qy, qz, qw

        # ★ 最新的 NDT map->odom（从 /pcl_pose topic 获取，不受融合 TF 污染）
        self.latest_pcl_map_odom = None  # dict with keys: x, y, z, qx, qy, qz, qw
        self.latest_pcl_pose_time = 0.0
        self.prev_pcl_map_odom = None   # 上一帧 /pcl_pose (用于帧间跳变检测)
        self._pcl_pose_jump_detected = False  # 帧间跳变标志

        # NDT 状态增强字段（从 ndt_status JSON 解析）
        self.latest_ndt_reason = ''                # "ok" | "pose_jump_candidate" | "confirmed_pose_jump" | ...
        self.latest_ndt_correction_translation = 0.0  # NDT 本帧平移修正量
        self.latest_ndt_correction_yaw = 0.0          # NDT 本帧旋转修正量

        # 最后健康时的快照——用于冻结和恢复
        self.last_healthy_map_odom = None     # 最后健康的 map->odom
        self.last_healthy_odom_body = None    # 对应的 odom->body（camera_init->body）
        self.last_healthy_time = 0.0          # 快照时间

        # DEGRADED 状态的冻结值
        self.frozen_map_odom = None           # 冻结的 map->odom
        self.frozen_odom_body = None          # 冻结时的 odom->body
        self.degraded_start_time = 0.0        # 进入 DEGRADED 的时间

        # 连续帧计数器（用于振荡保护）
        self.consecutive_healthy = 0          # 连续健康帧数
        self.consecutive_degraded = 0         # 连续退化帧数

        # TRANSITIONING 状态变量
        self.transition_start_time = 0.0      # 过渡开始时间
        self.transition_from = None           # 过渡起始 map->odom
        self.transition_to = None             # 过渡目标 map->odom

        # Recovery 状态跟踪（防止重复请求）
        self._recovery_in_progress = False    # SC/HDL 正在执行 recovery
        self.init_start_time = time.monotonic()  # INITIALIZING 启动时间，用于超时检测

        # 累计 odom 位移追踪（从进入 DEGRADED 算起，永不重置）
        self.total_odom_displacement = 0.0

        # =====================================================================
        # 定时器：以固定频率运行融合主循环
        # =====================================================================
        period_sec = 1.0 / self.publish_rate_hz
        self.update_timer = self.create_timer(period_sec, self._update)

        # 状态日志限流（避免刷屏）
        self.last_state_log_time = 0.0
        self.state_log_interval_sec = 2.0

        self.get_logger().info(
            '========== 定位融合节点已启动 ==========\n'
            f'  启动状态: INITIALIZING，等待首次 NDT/SC 定位成功\n'
            f'  状态转换: HEALTHY ←→ DEGRADED (error>{self.degraded_error_threshold})\n'
            f'  恢复阈值: error<{self.healthy_error_threshold} 连续{self.healthy_consecutive_frames}帧\n'
            f'  DEGRADED超时: {self.max_degraded_lock_sec}s (锁定期{self.min_degraded_lock_sec}s)\n'
            f'  inlier=0 虚假健康检测: >{self.inlier_zero_degraded_early_lost_sec}s+error<{self.inlier_zero_error_ceiling} → LOST\n'
            f'  静止定时器重置: odom<{self._odom_stationary_threshold_m}m 持续{self._odom_stationary_duration_sec}s → 归零\n'
            f'  过渡时间: {self.transition_duration_sec}s\n'
            f'  发布频率: {self.publish_rate_hz}Hz'
        )

    # =========================================================================
    # 回调函数
    # =========================================================================

    def _on_ndt_status(self, msg: String):
        """
        接收 NDT 扫描匹配状态回调

        由 lidar_localization 节点在每次 NDT 匹配后发布。
        JSON 格式:
          - state: "accepted" | "rejected" | "confirming"
          - has_converged: 是否收敛
          - fitness_score: 匹配误差（替代 matching_error）
          - reason: 拒绝原因（rejected 时）
          - correction_translation: 修正平移量
          - correction_yaw: 修正旋转量

        Args:
            msg: std_msgs/String (JSON)
        """
        try:
            import json
            status = json.loads(msg.data)
            self.latest_ndt_error = float(status.get('fitness_score', float('inf')))
            self.latest_ndt_inlier = float(status.get('inlier_fraction', 0.0))
            self.latest_ndt_converged = bool(status.get('has_converged', False))
            self.latest_ndt_state = str(status.get('state', ''))
            self.latest_ndt_reason = str(status.get('reason', ''))
            self.latest_ndt_correction_translation = float(status.get('correction_translation', 0.0))
            self.latest_ndt_correction_yaw = float(status.get('correction_yaw', 0.0))
            self.latest_ndt_status_time = time.monotonic()
        except Exception:
            pass

    def _on_pcl_pose(self, msg: PoseWithCovarianceStamped):
        """
        接收 NDT 发布的 map->odom (通过 /pcl_pose topic)

        相比 TF 查询 _lookup_ndt_map_odom()，此回调不受融合节点自己
        发布的 TF 污染。在 DEGRADED/TRANSITIONING 状态下，融合节点
        也会发布 map->odom TF，此时 TF 查询会拿到自己的值而非 NDT 的。
        /pcl_pose 直接来自 NDT 节点，数据纯净。

        Args:
            msg: geometry_msgs/PoseWithCovarianceStamped
        """
        self.latest_pcl_map_odom = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'qx': msg.pose.pose.orientation.x,
            'qy': msg.pose.pose.orientation.y,
            'qz': msg.pose.pose.orientation.z,
            'qw': msg.pose.pose.orientation.w,
        }
        self.latest_pcl_pose_time = time.monotonic()

    def _on_recovery_status(self, msg: String):
        """
        接收 recovery 状态回调

        处理 SC/HDL 恢复事件流程:
          localization_recovery_started → localization_relocalize_requested →
          localization_initialpose_published → localization_recovery_waiting →
          localization_recovered (成功) 或 localization_relocalize_failed (失败)

        当 recovery 成功完成时，从 LOST 状态恢复到 HEALTHY。

        Args:
            msg: 恢复状态 JSON 字符串
        """
        event_type = ''
        recovery_reason = ''
        try:
            import json as _json
            data = _json.loads(msg.data)
            event_type = data.get('event_type', '')
            recovery_reason = data.get('reason', '')
        except Exception:
            pass

        # 跟踪 recovery 进行状态（防止 fusion 重复请求）
        if event_type == 'localization_recovery_started':
            self._recovery_in_progress = True
            self.get_logger().info('[RECOVERY] 检测到 recovery 已启动，抑制重复请求')
        elif event_type == 'localization_relocalize_failed':
            self._recovery_in_progress = False
            if self.state == FusionState.LOST:
                self.get_logger().warn('[RECOVERY] SC 重定位失败，允许后续重试')

        # LOST 状态下检测 recovery 成功
        if self.state != FusionState.LOST:
            return

        recovered_event = event_type == 'localization_recovered'
        if not recovered_event and not event_type:
            # 兼容非 JSON 格式的旧消息，但仍然必须走 LOST recovery 软验收。
            recovered_event = (
                'localization_recovered' in msg.data or
                'localization_initialpose_published' in msg.data
            )

        if recovered_event:
            if not self._validate_lost_recovery_soft():
                self._recovery_in_progress = False
                self.last_recovery_request_time = 0.0
                self.get_logger().warn(
                    '[LOST] recovery 软验收拒绝本次结果，继续请求 SC 重定位'
                    + (f' (bridge_reason={recovery_reason})' if recovery_reason else ''))
                self._request_recovery()
                return
            pose_text = 'pcl_pose=unavailable'
            if self.latest_pcl_map_odom is not None:
                pose_text = (
                    f'pcl_pose=({self.latest_pcl_map_odom["x"]:.3f}, '
                    f'{self.latest_pcl_map_odom["y"]:.3f})')
            self.get_logger().info(
                '[LOST→HEALTHY] recovery 通过软验收，切回 HEALTHY 并接受新 map->odom '
                f'({pose_text}, ndt_state={self.latest_ndt_state or "none"}, '
                f'converged={self.latest_ndt_converged}, '
                f'error={self.latest_ndt_error:.4f})')
            self._reset_state()
            self.state = FusionState.HEALTHY
            self._recovery_in_progress = False
            self._publish_fusion_status()

    def _on_nav_status(self, msg: String):
        """
        接收导航状态回调（★ 用于区分导航中 vs 已到达静止）

        navigation_state_manager 发布 /navigation_status topic，
        格式: {"state": "EXECUTING"|"IDLE"|"COMPLETED"|"PAUSED"|...}

        核心逻辑:
        - 导航中 (EXECUTING): LOST 超时 120s——需要准确定位
        - 静止播报 (IDLE/COMPLETED): LOST 超时 600s——不需要定位精度
        - ★ 到达点位时 (EXECUTING→IDLE): 重置 DEGRADED 计时器
          因为机器人静止时 odom 零漂移，且知道自己在路点位置，
          之前累积的 odom 误差可以在此时"归零"

        Args:
            msg: 导航状态 JSON 字符串
        """
        try:
            import json
            data = json.loads(msg.data)
            new_state = data.get('state', 'IDLE')
            old_state = self.nav_state

            if new_state != old_state:
                self.get_logger().info(
                    f'[NAV] 导航状态变更: {old_state} → {new_state}')

            self.nav_state = new_state

            # ★ 到达点位时重置 DEGRADED 超时计时器
            # 旧状态是导航中，新状态是空闲 → 机器人刚到达一个路点
            navigating_states = ('EXECUTING', 'PLANNING', 'MOVING', 'RUNNING')
            idle_states = ('IDLE', 'COMPLETED')

            if (old_state in navigating_states and
                new_state in idle_states and
                self.state == FusionState.DEGRADED):
                old_elapsed = time.monotonic() - self.degraded_start_time

                # ★ 到达路点: 重置超时计时器 + odom 位移计数器
                # 原理: 静止时 odom 零漂移, 且知道自己在路点位置,
                #       之前的累积误差可以在此归零
                self.degraded_start_time = time.monotonic()

                # 更新 odom 参考点到当前位置 (位移从当前路点开始重新算)
                odom_body = self._lookup_odom_body()
                if odom_body is not None and self.frozen_odom_body is not None:
                    old_displacement = math.hypot(
                        odom_body['x'] - self.frozen_odom_body[0],
                        odom_body['z'] - self.frozen_odom_body[2])
                else:
                    old_displacement = 0.0

                if odom_body is not None:
                    self.frozen_odom_body = (odom_body['x'], odom_body['y'], odom_body['z'])

                self.get_logger().info(
                    '[NAV] 到达路点，重置 DEGRADED 计时器 '
                    f'(用时{old_elapsed:.0f}s, 位移{old_displacement:.1f}m → 归零)')
        except Exception:
            pass

    def _is_robot_navigating(self) -> bool:
        """
        判断机器人是否正在导航中（需要精确定位）

        Returns:
            True: 导航中 (EXECUTING, PLANNING) — LOST 超时使用较短值
            False: 静止/空闲 (IDLE, COMPLETED, PAUSED) — LOST 超时使用较长值
        """
        navigating_states = ('EXECUTING', 'PLANNING', 'MOVING', 'RUNNING')
        return self.nav_state in navigating_states

    # =========================================================================
    # 主循环
    # =========================================================================

    def _update(self):
        """
        融合主循环（定时器回调，30Hz）

        每次调用执行:
        1. 从 TF 获取最新的 map->odom 和 camera_init->body
        2. 根据 NDT 状态更新状态机
        3. 根据当前状态决定如何发布 map->odom
        """
        try:
            # ── 步骤1: 获取 NDT 当前的 map->odom（从 TF 树）──
            # 注意: lidar_localization 发布 map->odom TF
            # 我们也发布 map->odom，但只在 DEGRADED/LOST 时发布
            # 需要用最新的时间戳去查询
            ndt_map_odom = self._lookup_ndt_map_odom()

            # ── 步骤2: 获取里程计 camera_init->body（Fast-LIO 输出）──
            odom_body = self._lookup_odom_body()

            # ── 步骤3: 更新状态机 ──
            if self.state == FusionState.INITIALIZING:
                self._update_initializing(ndt_map_odom, odom_body)
            elif self.state == FusionState.HEALTHY:
                self._update_healthy(ndt_map_odom, odom_body)
            elif self.state == FusionState.DEGRADED:
                self._update_degraded(ndt_map_odom, odom_body)
            elif self.state == FusionState.TRANSITIONING:
                self._update_transitioning(ndt_map_odom, odom_body)
            elif self.state == FusionState.LOST:
                self._update_lost(ndt_map_odom, odom_body)

            # ── 步骤4: 发布诊断信息 ──
            self._publish_diagnostics()

        except Exception as e:
            self.get_logger().debug(f'融合主循环异常: {e}')

    # =========================================================================
    # TF 查询
    # =========================================================================

    def _lookup_ndt_map_odom(self) -> dict:
        """
        从 TF 树查询 NDT 当前发布的 map->odom 变换

        lidar_localization 在每次 NDT 匹配后发布此 TF。
        当无法查询到时返回 None。

        Returns:
            dict with keys x, y, z, qx, qy, qz, qw，或 None
        """
        try:
            # 查询最新的 map->odom 变换
            transform = self.tf_buffer.lookup_transform(
                FRAME_MAP,
                FRAME_ODOM,
                Time(),  # 最新时间
                timeout=Duration(seconds=0.1),
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            return {
                'x': t.x, 'y': t.y, 'z': t.z,
                'qx': r.x, 'qy': r.y, 'qz': r.z, 'qw': r.w,
            }
        except Exception:
            return None

    def _lookup_odom_body(self) -> dict:
        """
        从 TF 树查询里程计 camera_init->body 变换（Fast-LIO 输出）

        这是机器人真实运动的"地面真相"——Fast-LIO 的 LiDAR-IMU 里程计
        在短时间内（<2分钟）漂移极小（<0.5cm/m）。

        Returns:
            dict with keys x, y, z, qx, qy, qz, qw，或 None
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                FRAME_CAMERA_INIT,
                FRAME_BODY,
                Time(),  # 最新时间
                timeout=Duration(seconds=0.1),
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            return {
                'x': t.x, 'y': t.y, 'z': t.z,
                'qx': r.x, 'qy': r.y, 'qz': r.z, 'qw': r.w,
            }
        except Exception:
            return None

    # =========================================================================
    # 状态更新函数
    # =========================================================================

    def _update_initializing(self, ndt_map_odom: dict, odom_body: dict):
        """
        INITIALIZING state: wait for first reliable localization.

        At startup, NDT hasn't received /initialpose yet, so ndt_status will
        be rejected/error=inf for an extended period. This should NOT be
        interpreted as degradation from HEALTHY.

        Timeout protection: if NDT doesn't become healthy within init_timeout_sec,
        proactively request SC recovery to prevent deadlock.
        (SC bridge waits for fusion -> fusion waits for NDT -> NDT waits for SC bridge)
        """
        if ndt_map_odom is not None:
            self.latest_ndt_map_odom = ndt_map_odom

        initial_pose = ndt_map_odom if ndt_map_odom is not None else self.latest_pcl_map_odom
        if initial_pose is not None and odom_body is not None and self._is_healthy():
            self.last_healthy_map_odom = initial_pose.copy()
            self.last_healthy_odom_body = odom_body.copy()
            self.last_healthy_time = time.monotonic()
            self.state = FusionState.HEALTHY
            self.consecutive_healthy = 0
            self.consecutive_degraded = 0
            self.get_logger().info(
                '========== [INITIALIZING→HEALTHY] 首次定位成功 ==========\n'
                f'  NDT error: {self.latest_ndt_error:.4f}\n'
                f'  map->odom: ({initial_pose["x"]:.3f}, {initial_pose["y"]:.3f})')
            self._publish_fusion_status()
            return

        # ★ 超时保护: INITIALIZING 超时后主动请求 SC recovery
        #    防止启动死锁: 如果 SC bridge 的 startup 窗口已过,
        #    fusion 必须主动触发 recovery 而不是被动等待
        now = time.monotonic()
        init_elapsed = now - self.init_start_time
        if init_elapsed > self.init_timeout_sec:
            last_req = getattr(self, 'last_recovery_request_time', 0.0)
            if now - last_req > self.recovery_request_cooldown_sec:
                self.get_logger().warn(
                    f'[INITIALIZING] 启动超时 ({init_elapsed:.0f}s > '
                    f'{self.init_timeout_sec:.0f}s)，主动请求 SC recovery...')
                self._request_recovery()

        if self.verbose_logging and now - self.last_state_log_time > self.state_log_interval_sec:
            self.last_state_log_time = now
            reason = '等待 /initialpose 后 NDT 首次 accepted'
            if self.latest_ndt_status_time <= 0.0:
                reason = '等待 NDT 状态'
            elif initial_pose is None:
                reason = '等待 map->odom 或 /pcl_pose'
            elif odom_body is None:
                reason = '等待 odom->body TF'
            elif init_elapsed > self.init_timeout_sec:
                reason = (f'启动超时 ({init_elapsed:.0f}s), '
                          f'已请求 recovery, 等待 NDT 接受 /initialpose')
            self.get_logger().info(
                f'[INITIALIZING] {reason} '
                f'(ndt_state={self.latest_ndt_state or "none"}, '
                f'error={self.latest_ndt_error:.4f}, '
                f'converged={self.latest_ndt_converged})')

    def _update_healthy(self, ndt_map_odom: dict, odom_body: dict):
        """
        HEALTHY 状态：NDT 定位正常

        行为:
        - 不发布 map->odom（让 lidar_localization 发布，保持直通）
        - 持续更新 last_healthy 快照（用于可能的冻结）
        - 检测 degradation 并切换状态

        Args:
            ndt_map_odom: 当前 NDT 的 map->odom
            odom_body: 当前 Fast-LIO 的 camera_init->body
        """
        if ndt_map_odom is not None:
            self.latest_ndt_map_odom = ndt_map_odom

            # 当 NDT matching_error 很低时，更新"最后健康"快照
            # 这个快照在 DEGRADED 时用于冻结 map->odom
            # 使用更严格的阈值（healthy_error_threshold）确保快照质量
            if (self.latest_ndt_error < self.healthy_error_threshold and
                odom_body is not None):
                self.last_healthy_map_odom = ndt_map_odom.copy()
                self.last_healthy_odom_body = odom_body.copy()
                self.last_healthy_time = time.monotonic()

        # ── /pcl_pose 帧间跳变兜底检测 ──
        # 即使 NDT status reason 未报 pose_jump，如果 /pcl_pose (map->odom)
        # 在两帧之间跳变超过阈值，说明 NDT 静默改变了位姿估计。
        # 几何混叠场景下 NDT 可能 fitness 极低但不报 pose_jump。
        if self.pose_jump_degraded_from_pcl:
            pcl = self.latest_pcl_map_odom
            prev = self.prev_pcl_map_odom
            if pcl is not None and prev is not None:
                pcl_jump = math.hypot(pcl['x'] - prev['x'], pcl['y'] - prev['y'])
                if pcl_jump > self.pose_jump_pcl_threshold_m:
                    self.get_logger().warn(
                        f'[HEALTHY] /pcl_pose 帧间跳变检测: '
                        f'jump={pcl_jump:.3f}m > {self.pose_jump_pcl_threshold_m}m, '
                        f'prev=({prev["x"]:.3f},{prev["y"]:.3f}) '
                        f'curr=({pcl["x"]:.3f},{pcl["y"]:.3f})')
                    self._pcl_pose_jump_detected = True
                else:
                    self._pcl_pose_jump_detected = False
            if pcl is not None:
                self.prev_pcl_map_odom = pcl.copy()

        # ── 退化检测 ──
        if self._is_degraded():
            self.consecutive_degraded += 1
            if (self.verbose_logging and
                self.consecutive_degraded == 1):
                degrade_triggers = []
                if self.latest_ndt_error > self.degraded_error_threshold:
                    degrade_triggers.append(f'error={self.latest_ndt_error:.4f}>{self.degraded_error_threshold}')
                if self.latest_ndt_reason in ('pose_jump_candidate', 'confirmed_pose_jump'):
                    degrade_triggers.append(f'ndt_reason={self.latest_ndt_reason}')
                if self.latest_ndt_correction_translation > self.pose_jump_correction_threshold_m:
                    degrade_triggers.append(
                        f'correction={self.latest_ndt_correction_translation:.3f}m'
                        f'>{self.pose_jump_correction_threshold_m}m')
                if getattr(self, '_pcl_pose_jump_detected', False):
                    degrade_triggers.append('pcl_pose_jump')
                if not self.latest_ndt_converged and self.latest_ndt_error > 0.1:
                    degrade_triggers.append('not_converged')
                self.get_logger().warn(
                    f'检测到 NDT 退化: {", ".join(degrade_triggers)}，'
                    f'需连续{self.degraded_consecutive_frames}帧确认')
        else:
            self.consecutive_degraded = 0

        # 连续退化确认后进入 DEGRADED
        if self.consecutive_degraded >= self.degraded_consecutive_frames:
            self._enter_degraded()

    def _update_degraded(self, ndt_map_odom: dict, odom_body: dict):
        """
        DEGRADED 状态：NDT 定位不可靠，使用冻结 map->odom + 里程计传播

        行为:
        - 发布 frozen_map_odom 作为 map->odom TF（覆盖 NDT 的漂移值）
        - 锁定期内拒绝 NDT 恢复信号，防止假恢复循环
        - 锁定期满后使用更严格的恢复条件验证 NDT
        - 检查超时/位移是否过大

        Args:
            ndt_map_odom: 当前 NDT 的 map->odom（可能已经漂移）
            odom_body: 当前 Fast-LIO 的 camera_init->body
        """
        # 更新最新的 NDT map->odom（用于恢复时的过渡目标）
        if ndt_map_odom is not None:
            self.latest_ndt_map_odom = ndt_map_odom

        # ── 发布冻结的 map->odom（覆盖 NDT 的漂移值）──
        if self.frozen_map_odom is not None:
            self._publish_map_odom_tf(self.frozen_map_odom)

        # ── 锁定期内 NDT 帧统计 ──
        in_lock = time.monotonic() < getattr(self, 'degraded_lock_until', 0)
        if in_lock:
            self._lock_ndt_total_frames += 1
            if self._is_degraded():
                self._lock_ndt_rejected_frames += 1

        # ── 检查恢复条件 (锁定期内跳过) ──
        if not in_lock:
            if self._is_healthy_strict():
                self.consecutive_healthy += 1
                if self.verbose_logging and self.consecutive_healthy == 1:
                    self.get_logger().info(
                        f'[锁定期后] NDT 开始恢复: error={self.latest_ndt_error:.4f}'
                        f' reason={self.latest_ndt_reason}'
                        f' corr={self.latest_ndt_correction_translation:.3f}m'
                        f' 需连续{self.lock_recovery_healthy_consecutive_frames}帧确认')
            else:
                if self.consecutive_healthy > 0:
                    # 给出不健康的具体原因
                    reasons = []
                    if self.latest_ndt_error > 0.15:
                        reasons.append(f'error={self.latest_ndt_error:.4f}>0.15')
                    if not self.latest_ndt_converged:
                        reasons.append('未收敛')
                    if self.latest_ndt_reason != 'ok':
                        reasons.append(f'reason={self.latest_ndt_reason}')
                    if self.latest_ndt_correction_translation > self.lock_recovery_max_correction_m:
                        reasons.append(f'corr={self.latest_ndt_correction_translation:.3f}m'
                                       f'>{self.lock_recovery_max_correction_m}m')
                    self.get_logger().info(
                        f'[锁定期后] NDT 恢复中断: {"; ".join(reasons)}')
                self.consecutive_healthy = 0

            if self.consecutive_healthy >= self.lock_recovery_healthy_consecutive_frames:
                self._enter_transitioning()
                return

        # ── 检查超时条件 ──
        elapsed = time.monotonic() - self.degraded_start_time
        odom_displacement = self._compute_odom_displacement(odom_body)

        # ★ 累计位移追踪（不重置，用于检测跨路点累积漂移）
        if odom_displacement > self.total_odom_displacement:
            self.total_odom_displacement = odom_displacement

        # ── LOST 触发条件 (按优先级排列) ──

        # 条件1: 锁定期内 NDT 持续拒绝 → 提前 LOST
        if (in_lock and
            self._lock_ndt_total_frames >= self.lock_early_lost_min_frames and
            self._lock_ndt_rejected_frames / self._lock_ndt_total_frames
                >= self.lock_early_lost_rejection_rate):
            self.get_logger().error(
                f'[DEGRADED→LOST] 锁定期内 NDT 持续拒绝: '
                f'{self._lock_ndt_rejected_frames}/{self._lock_ndt_total_frames}'
                f' ({100*self._lock_ndt_rejected_frames/self._lock_ndt_total_frames:.0f}%)'
                f' >= {100*self.lock_early_lost_rejection_rate:.0f}%'
                f' → NDT 明显无法工作，提前触发 SC 全局重定位')
            self._enter_lost()
            return

        # 条件1.5: NDT inlier=0 + 极低 error → 虚假健康 (P0-2)
        # 长廊几何混叠的强特征: fitness 极低但 inlier 始终为 0
        # 此时 NDT 已收敛到错误位置且不再调整，必须加速触发 LOST
        if (not in_lock and
            self.latest_ndt_inlier <= 0.0 and
            self.latest_ndt_error < self.inlier_zero_error_ceiling):
            if self._inlier_zero_start <= 0.0:
                self._inlier_zero_start = time.monotonic()
            self._inlier_zero_elapsed = time.monotonic() - self._inlier_zero_start
            if self._inlier_zero_elapsed > self.inlier_zero_degraded_early_lost_sec:
                self.get_logger().error(
                    f'[DEGRADED→LOST] NDT inlier=0 虚假健康检测: '
                    f'inlier={self.latest_ndt_inlier}, '
                    f'error={self.latest_ndt_error:.4f}<{self.inlier_zero_error_ceiling}, '
                    f'持续{self._inlier_zero_elapsed:.0f}s>{self.inlier_zero_degraded_early_lost_sec:.0f}s '
                    f'→ NDT 已收敛到错误位置，触发 SC 全局重定位')
                self._enter_lost()
                return
        else:
            self._inlier_zero_start = 0.0
            self._inlier_zero_elapsed = 0.0

        # ── DEGRADED 静止检测: 里程计长时间不动 → 重置定时器 (P0-3) ──
        # 比 nav_state 变更更可靠，不依赖导航状态消息的精确时序
        if odom_body is not None and not in_lock:
            if self._last_odom_body_for_stationary is not None:
                dx = odom_body['x'] - self._last_odom_body_for_stationary['x']
                dz = odom_body['z'] - self._last_odom_body_for_stationary['z']
                moved = math.hypot(dx, dz)
                if moved < self._odom_stationary_threshold_m:
                    if self._odom_stationary_start <= 0.0:
                        self._odom_stationary_start = time.monotonic()
                    elif (time.monotonic() - self._odom_stationary_start >
                          self._odom_stationary_duration_sec):
                        old_elapsed = elapsed
                        self.degraded_start_time = time.monotonic()
                        self._odom_stationary_start = 0.0
                        self.get_logger().info(
                            f'[DEGRADED] 静止检测: odom 位移 <{self._odom_stationary_threshold_m}m '
                            f'持续{self._odom_stationary_duration_sec}s，'
                            f'重置 DEGRADED 计时器 (原已用{old_elapsed:.0f}s → 归零)')
                else:
                    self._odom_stationary_start = 0.0
            self._last_odom_body_for_stationary = odom_body.copy()
        else:
            self._odom_stationary_start = 0.0

        # 条件2: DEGRADED 总超时
        if elapsed > self.max_degraded_lock_sec:
            self.get_logger().warn(
                f'[DEGRADED→LOST] 总超时: {elapsed:.1f}s > '
                f'{self.max_degraded_lock_sec:.0f}s '
                f'(锁定期={self.min_degraded_lock_sec}s)')
            self._enter_lost()
            return

        # 条件3: 单段 odom 位移过大
        if odom_displacement > self.max_odom_displacement_m:
            self.get_logger().warn(
                f'[DEGRADED→LOST] 单段 odom 位移过大: {odom_displacement:.2f}m > '
                f'{self.max_odom_displacement_m}m')
            self._enter_lost()
            return

        # 条件4: 累计位移过大（永不重置，防止跨路点累积漂移）
        if self.total_odom_displacement > self.max_total_odom_displacement_m:
            self.get_logger().warn(
                f'[DEGRADED→LOST] 累计 odom 位移过大: '
                f'{self.total_odom_displacement:.1f}m > '
                f'{self.max_total_odom_displacement_m:.0f}m '
                f'(从进入DEGRADED算起，已跨多个路点)')
            self._enter_lost()
            return

    def _update_transitioning(self, ndt_map_odom: dict, odom_body: dict):
        """
        TRANSITIONING 状态：从 DEGRADED 平滑过渡回 HEALTHY

        行为:
        - 在 transition_duration_sec 时间内从 frozen_map_odom 平滑插值到 NDT 当前值
        - 使用 smoothstep 函数避免机器人在 map 中"瞬移"
        - 过渡完成后进入 HEALTHY

        为什么需要平滑过渡:
        - NDT 恢复后，当前的 map->odom 可能与 frozen_map_odom 有微小差异
        - 直接跳变会导致 robot_map 瞬移，navigation 和 costmap 会崩溃
        - 平滑过渡让下游有足够时间适应新的位姿

        Args:
            ndt_map_odom: 当前 NDT 的 map->odom
            odom_body: 当前 Fast-LIO 的 camera_init->body
        """
        if ndt_map_odom is not None:
            self.latest_ndt_map_odom = ndt_map_odom

        # 如果过渡目标未设置，尝试用当前 NDT 值
        if self.transition_to is None and ndt_map_odom is not None:
            self.transition_to = ndt_map_odom.copy()

        if self.transition_from is None or self.transition_to is None:
            # 还没准备好过渡数据，继续发布冻结值
            if self.frozen_map_odom is not None:
                self._publish_map_odom_tf(self.frozen_map_odom)
            return

        # ── 计算过渡进度 ──
        elapsed = time.monotonic() - self.transition_start_time
        alpha = smoothstep(elapsed / self.transition_duration_sec)

        # ── 插值 map->odom ──
        # 平移: 线性插值
        interp = {
            'x': self.transition_from['x'] + alpha * (self.transition_to['x'] - self.transition_from['x']),
            'y': self.transition_from['y'] + alpha * (self.transition_to['y'] - self.transition_from['y']),
            'z': self.transition_from['z'] + alpha * (self.transition_to['z'] - self.transition_from['z']),
        }

        # 旋转: slerp（球面线性插值）
        q_from = (self.transition_from['qx'], self.transition_from['qy'],
                   self.transition_from['qz'], self.transition_from['qw'])
        q_to = (self.transition_to['qx'], self.transition_to['qy'],
                self.transition_to['qz'], self.transition_to['qw'])
        q_interp = quat_slerp(q_from, q_to, alpha)
        interp['qx'], interp['qy'], interp['qz'], interp['qw'] = q_interp

        self._publish_map_odom_tf(interp)

        # ── 检查是否完成 ──
        if elapsed >= self.transition_duration_sec:
            self.get_logger().info(
                f'[TRANSITIONING→HEALTHY] 平滑过渡完成 '
                f'(耗时 {elapsed:.2f}s)，切回 HEALTHY')
            # 过渡完成后，用 NDT 的最新值更新快照
            if ndt_map_odom is not None:
                self.last_healthy_map_odom = ndt_map_odom.copy()
                self.last_healthy_odom_body = odom_body.copy() if odom_body else None
                self.last_healthy_time = time.monotonic()
            self.state = FusionState.HEALTHY
            self.consecutive_healthy = 0
            self.consecutive_degraded = 0
            self.transition_from = None
            self.transition_to = None
            self.degraded_lock_until = 0.0       # 平滑恢复成功后清理锁定期状态
            self._lock_ndt_total_frames = 0
            self._lock_ndt_rejected_frames = 0
            # 清理 inlier=0 虚假健康检测 (P0-2)
            self._inlier_zero_start = 0.0
            self._inlier_zero_elapsed = 0.0
            # 清理 DEGRADED 静止检测 (P0-3)
            self._odom_stationary_start = 0.0
            self._last_odom_body_for_stationary = None
            self._publish_fusion_status()

    def _update_lost(self, ndt_map_odom: dict, odom_body: dict):
        """
        LOST 状态：长时间退化，等待 recovery

        行为:
        - 不发布 map->odom（让 recovery 机制接管）
        - 定期重新请求 recovery（如果上次请求后仍处于 LOST）
        - 定期输出诊断日志
        - recovery 成功后由 _on_recovery_status 切回 HEALTHY

        Args:
            ndt_map_odom: 当前 NDT 的 map->odom
            odom_body: 当前 Fast-LIO 的 camera_init->body
        """
        if ndt_map_odom is not None:
            self.latest_ndt_map_odom = ndt_map_odom

        # 在 LOST 状态下不发布 map->odom
        # 让 recovery bridge (SC/HDL) 发布 /initialpose 后由 NDT 重新建立定位
        # recovery 会: 清除缓存 → 等待静止 → 全局重定位 → 发布 initialpose → 恢复

        now = time.monotonic()
        if now - self.last_state_log_time > 5.0:
            self.last_state_log_time = now
            self.get_logger().warn(
                f'[LOST] 等待 recovery... '
                f'(error={self.latest_ndt_error:.4f})')

        # ★ 如果上次 recovery 请求后仍处于 LOST，定期重试
        last_req = getattr(self, 'last_recovery_request_time', 0.0)
        if now - last_req > self.recovery_request_cooldown_sec:
            self._request_recovery()

    # =========================================================================
    # 状态转换辅助函数
    # =========================================================================

    def _enter_degraded(self):
        """
        进入 DEGRADED 状态

        前置条件:
        - 必须有 last_healthy_map_odom 快照（否则无法冻结）
        - 必须在 HEALTHY 状态
        """
        if self.last_healthy_map_odom is None:
            self.get_logger().warn(
                '[HEALTHY→DEGRADED] 拒绝: 没有健康快照，继续保持 HEALTHY '
                '(等待 NDT 首次收敛后记录快照)')
            self.consecutive_degraded = 0
            return

        self.state = FusionState.DEGRADED
        self.frozen_map_odom = self.last_healthy_map_odom.copy()
        # ★ 存储 camera_init 帧下的 body 3D 位置 (x=左右, y=垂直, z=前后)
        # 后续位移计算使用 (x, z) 作为 2D 水平面
        if self.last_healthy_odom_body:
            bod = self.last_healthy_odom_body
            self.frozen_odom_body = (bod['x'], bod['y'], bod['z'])
        else:
            self.frozen_odom_body = None
        self.degraded_start_time = time.monotonic()
        self.total_odom_displacement = 0.0  # ★ 累计位移，永不重置
        self.consecutive_healthy = 0

        # ── 锁定期初始化 ──
        # 进入 DEGRADED 后进入锁定期，在此期间拒绝 NDT 的恢复信号
        # 防止 NDT 快速 pose_jump→错误收敛→report ok 的假恢复
        self.degraded_lock_until = time.monotonic() + self.min_degraded_lock_sec
        # 锁定期内 NDT 帧统计（用于检测 NDT 持续不可用）
        self._lock_ndt_total_frames = 0
        self._lock_ndt_rejected_frames = 0

        # ── inlier=0 虚假健康检测初始化 (P0-2) ──
        self._inlier_zero_start = 0.0
        self._inlier_zero_elapsed = 0.0

        # ── DEGRADED 静止检测初始化 (P0-3) ──
        self._odom_stationary_start = 0.0
        self._last_odom_body_for_stationary = None

        frozen_yaw = quat_to_yaw(
            self.frozen_map_odom['qx'],
            self.frozen_map_odom['qy'],
            self.frozen_map_odom['qz'],
            self.frozen_map_odom['qw'],
        )

        self.get_logger().warn(
            '========== [HEALTHY→DEGRADED] 冻结 map->odom ==========\n'
            f'  冻结值: ({self.frozen_map_odom["x"]:.3f}, '
            f'{self.frozen_map_odom["y"]:.3f}, '
            f'yaw={frozen_yaw:.2f}rad)\n'
            f'  NDT error: {self.latest_ndt_error:.4f}\n'
            f'  此后机器人运动由 Fast-LIO 里程计驱动')

        self._publish_fusion_status()

    def _enter_transitioning(self):
        """
        进入 TRANSITIONING 状态（DEGRADED→HEALTHY 的中间状态）

        使用当前 NDT 的 map->odom 作为过渡目标。
        ★ 使用 /pcl_pose topic (latest_pcl_map_odom)，避免 TF 污染。
        如果 NDT 值不可用，保持 DEGRADED。
        """
        pcl_odom = self.latest_pcl_map_odom
        if pcl_odom is None:
            self.get_logger().warn(
                '[DEGRADED→TRANSITIONING] 拒绝: 无法获取 NDT map->odom (/pcl_pose)，保持 DEGRADED')
            self.consecutive_healthy = 0
            return

        self.state = FusionState.TRANSITIONING
        self.transition_start_time = time.monotonic()
        self.transition_from = self.frozen_map_odom.copy()
        self.transition_to = pcl_odom.copy()

        # 计算跳变距离（用于诊断）
        dx = self.transition_to['x'] - self.transition_from['x']
        dy = self.transition_to['y'] - self.transition_from['y']
        jump = math.hypot(dx, dy)

        # ── 恢复位置跳变验证 ──
        # NDT 跳变过大 → 可能收敛到错误位置 → 拒绝恢复
        if jump > self.recovery_pose_jump_max_m:
            self.get_logger().error(
                f'[DEGRADED→TRANSITIONING] 拒绝: NDT 恢复位置跳变过大 '
                f'({jump:.2f}m > {self.recovery_pose_jump_max_m}m), '
                f'疑似错误收敛 → 升级为 LOST 并请求 SC 全局重定位')
            self.state = FusionState.DEGRADED
            self.consecutive_healthy = 0
            self._enter_lost()
            return

        self.get_logger().info(
            '========== [DEGRADED→TRANSITIONING] 开始平滑过渡 ==========\n'
            f'  起始: ({self.transition_from["x"]:.3f}, {self.transition_from["y"]:.3f})\n'
            f'  目标: ({self.transition_to["x"]:.3f}, {self.transition_to["y"]:.3f})\n'
            f'  跳变: {jump:.3f}m\n'
            f'  过渡时间: {self.transition_duration_sec}s\n'
            f'  连续健康帧: {self.consecutive_healthy}/{self.healthy_consecutive_frames}')

        self._publish_fusion_status()

    def _enter_lost(self):
        """
        进入 LOST 状态：里程计偏移过大或时间过长

        停止发布 map->odom，主动请求 HDL 全局重定位。
        ★ 关键: 通过 /localization/recovery_requests 触发 hdl_bootstrap 的 recovery
        """
        self.state = FusionState.LOST
        elapsed = time.monotonic() - self.degraded_start_time

        self.get_logger().error(
            '========== [DEGRADED→LOST] 触发 recovery ==========\n'
            f'  冻结持续时间: {elapsed:.1f}s\n'
            f'  NDT error: {self.latest_ndt_error:.4f}\n'
            f'  最后健康 map->odom: ({self.frozen_map_odom["x"]:.3f}, '
            f'{self.frozen_map_odom["y"]:.3f})')

        # ★ 主动请求 HDL 全局重定位
        self._request_recovery()

        self._publish_fusion_status()

    def _request_recovery(self):
        """
        向 recovery 桥接节点发送请求 (通过 /localization/recovery_requests)

        兼容 SC (scancontext_to_initialpose) 和 HDL (hdl_bootstrap_to_initialpose):
        1. recovery bridge 收到请求后开始全局重定位
        2. 发布 /initialpose → NDT 重新初始化
        3. NDT 验证 → 发布 /localization/recovery_status → fusion 恢复
        """
        now = time.monotonic()

        # 冷却时间检查
        if (hasattr(self, 'last_recovery_request_time') and
            now - self.last_recovery_request_time < self.recovery_request_cooldown_sec):
            self.get_logger().info(
                f'[LOST] recovery 请求冷却中 '
                f'({now - self.last_recovery_request_time:.1f}s < '
                f'{self.recovery_request_cooldown_sec}s)')
            return

        # 如果 recovery 已经在进行中，不重复请求
        if getattr(self, '_recovery_in_progress', False):
            self.get_logger().info(
                '[LOST] recovery 已在执行中，跳过重复请求')
            return

        state_label = 'INITIALIZING' if self.state == FusionState.INITIALIZING else 'LOST'
        self.last_recovery_request_time = now

        import json as _json
        request_msg = String()
        request_msg.data = _json.dumps({
            'reason': 'fusion odom fallback exhausted (timeout or displacement limit)',
            'source': 'localization_odom_fusion',
            'event_type': 'fusion_lost',
            'search_radius_m': 5.0,
        })
        self.recovery_request_pub.publish(request_msg)
        self.get_logger().warn(
            f'[{state_label}] 已发送 recovery 请求到 /localization/recovery_requests，'
            '等待 SC/hdl_bootstrap 执行全局重定位...')

    def _reset_state(self):
        """重置所有内部状态变量"""
        self.consecutive_healthy = 0
        self.consecutive_degraded = 0
        self.frozen_map_odom = None
        self.frozen_odom_body = None
        self.degraded_start_time = 0.0
        self.transition_from = None
        self.transition_to = None
        self._recovery_in_progress = False
        self._pcl_pose_jump_detected = False
        self.prev_pcl_map_odom = None
        # 清理锁定期状态
        self.degraded_lock_until = 0.0
        self._lock_ndt_total_frames = 0
        self._lock_ndt_rejected_frames = 0
        # 清理 inlier=0 虚假健康检测 (P0-2)
        self._inlier_zero_start = 0.0
        self._inlier_zero_elapsed = 0.0
        # 清理 DEGRADED 静止检测 (P0-3)
        self._odom_stationary_start = 0.0
        self._last_odom_body_for_stationary = None

    # =========================================================================
    # 状态判断辅助函数
    # =========================================================================

    def _is_degraded(self) -> bool:
        """
        判断当前 NDT 是否退化

        退化条件（满足任一即退化）:
        0. NDT status reason 检测到位姿跳变 (pose_jump_candidate / confirmed_pose_jump)
        0b. NDT correction_translation 超过阈值 (默认 0.5m, 比 NDT 的 0.8m 更敏感)
        1. matching_error > degraded_error_threshold (默认 0.5)
        2. NDT 未收敛

        判据 0/0b 是 2026-05-26 新增: 几何混叠场景下 NDT 可能 fitness 极低
        (0.003) 但收敛到错误位置, 仅靠 fitness_score 无法检测。NDT status JSON
        已包含 reason/correction_translation 字段, 融合借此感知位姿跳变。

        Returns:
            True 如果 NDT 当前退化
        """
        # NDT 状态数据过期检查（超过 3 秒没有新数据认为通信异常）
        status_age = time.monotonic() - self.latest_ndt_status_time
        if status_age > 3.0:
            return True

        # 判据 0: NDT status reason 检测到位姿跳变
        # "pose_jump_candidate" — NDT 正在确认跳变, 仍在重发布旧 pose
        #   此时进入 DEGRADED 可在 NDT 接受跳变前冻结正确位姿
        # "confirmed_pose_jump" — NDT 已接受跳变, map->odom 已更新
        if self.pose_jump_degraded_from_status and self.latest_ndt_reason in (
            'pose_jump_candidate', 'confirmed_pose_jump'):
            return True

        # 判据 0b: NDT correction_translation 直接超标
        # NDT 内部 threshold 是 0.8m, 这里用更低的 0.5m 提前拦截
        if (self.pose_jump_degraded_from_status and
            self.latest_ndt_correction_translation > self.pose_jump_correction_threshold_m):
            return True

        # 判据 0c: /pcl_pose 帧间跳变兜底
        # NDT status 未报异常但 map->odom 静默跳变 → 几何混叠伪健康
        if self.pose_jump_degraded_from_pcl and getattr(self, '_pcl_pose_jump_detected', False):
            return True

        # 主要判据: matching_error 是否超标
        if self.latest_ndt_error > self.degraded_error_threshold:
            return True

        # 辅助判据: NDT 未收敛
        if not self.latest_ndt_converged and self.latest_ndt_error > 0.1:
            return True

        return False

    def _is_healthy(self) -> bool:
        """
        判断当前 NDT 是否已恢复健康

        健康条件（全部满足）:
        1. matching_error < healthy_error_threshold (默认 0.15)
        2. NDT 已收敛
        3. (DEGRADED 恢复时) NDT 当前 map→odom 与冻结值偏差 < 0.8m
           防止几何混叠导致的"低 error 但错位置"伪恢复

        frozen_map_odom 和 latest_ndt_map_odom 都来自 TF 查询
        map→odom，为标准 ROS 坐标系，可直接比较无需坐标转换。

        使用比 _is_degraded 更严格的阈值（0.15 vs 0.5），
        形成滞回区间，防止频繁振荡

        Returns:
            True 如果 NDT 当前健康
        """
        if not (self.latest_ndt_error < self.healthy_error_threshold and
                self.latest_ndt_converged):
            return False

        # DEGRADED 恢复时: 检查 NDT 新 map->odom 是否与冻结值一致
        # ★ 使用 /pcl_pose topic 数据 (latest_pcl_map_odom)，而非 TF 查询
        #    因为 DEGRADED 期间融合自己也发布 map->odom，TF 查询会被污染
        pcl_odom = self.latest_pcl_map_odom
        if self.frozen_map_odom is not None and pcl_odom is not None:
            ndt_dx = pcl_odom['x'] - self.frozen_map_odom['x']
            ndt_dy = pcl_odom['y'] - self.frozen_map_odom['y']
            pose_jump = math.hypot(ndt_dx, ndt_dy)
            if pose_jump > 0.8:
                return False

        return True

    def _is_healthy_strict(self) -> bool:
        """
        锁定期满后使用的严格恢复条件。

        比 _is_healthy() 更严格:
        1. NDT error < 0.15 (与 _is_healthy 相同)
        2. NDT 已收敛
        3. NDT reason 必须为 "ok"（拒绝 pose_jump/confirming 等过渡状态）
        4. NDT correction_translation < lock_recovery_max_correction_m (默认 0.3m)
           防止 NDT 微小跳变被忽略
        5. frozen→NDT map→odom 偏差 < 0.8m (与 _is_healthy 相同)

        Returns:
            True 如果 NDT 满足严格恢复条件
        """
        # 基础条件: 误差低且已收敛
        if not (self.latest_ndt_error < 0.15 and self.latest_ndt_converged):
            return False

        # 状态必须是 ok（不能是 pose_jump_candidate / confirming 等）
        if self.latest_ndt_reason != 'ok':
            return False

        # correction 必须足够小（NDT 不再剧烈调整）
        if self.latest_ndt_correction_translation > self.lock_recovery_max_correction_m:
            return False

        # DEGRADED 恢复时: 检查 NDT 新 map→odom 与冻结值一致
        pcl_odom = self.latest_pcl_map_odom
        if self.frozen_map_odom is not None and pcl_odom is not None:
            ndt_dx = pcl_odom['x'] - self.frozen_map_odom['x']
            ndt_dy = pcl_odom['y'] - self.frozen_map_odom['y']
            pose_jump = math.hypot(ndt_dx, ndt_dy)
            if pose_jump > 0.8:
                return False

        return True

    def _validate_lost_recovery_soft(self) -> bool:
        """
        LOST recovery 的宽松最终验收。

        目的不是用 odom 重新裁决 SC/NDT，而是只拦截明显错误的恢复结果：
        - NDT 明显仍不健康时拒绝；
        - 如果 odom/冻结位姿不完整或 odom 已累计较远，则跳过 odom 位置校验；
        - 只检查恢复后的 map->odom 相对冻结点的距离是否超过 odom 位移 + 宽松余量；
        - 不做 yaw 硬拒绝，避免走廊/角度 wrap/odom 漂移误伤正确重定位。
        """
        if not self.recovery_pose_soft_gate_enabled:
            self.get_logger().warn('[LOST] recovery 软验收已关闭，直接接受 recovery 结果')
            return True

        now = time.monotonic()

        if self.latest_ndt_status_time > 0.0:
            status_age = now - self.latest_ndt_status_time
            if status_age <= self.recovery_pose_max_status_age_sec:
                ndt_rejected = self.latest_ndt_state == 'rejected'
                ndt_bad = (
                    ndt_rejected or
                    not self.latest_ndt_converged or
                    self.latest_ndt_error > self.degraded_error_threshold
                )
                if ndt_bad:
                    self.get_logger().warn(
                        '[LOST] recovery 软验收拒绝: NDT 状态仍不健康 '
                        f'(state={self.latest_ndt_state}, '
                        f'converged={self.latest_ndt_converged}, '
                        f'error={self.latest_ndt_error:.4f})')
                    return False
            else:
                self.get_logger().warn(
                    '[LOST] recovery 软验收跳过 NDT 状态时效检查: '
                    f'status_age={status_age:.2f}s')
        else:
            self.get_logger().warn(
                '[LOST] recovery 软验收跳过 NDT 状态检查: 尚未收到 /localization/ndt_status')

        pcl_odom = self.latest_pcl_map_odom
        if pcl_odom is None:
            self.get_logger().warn(
                '[LOST] recovery 软验收放行: 无 /pcl_pose，使用 SC bridge 的 NDT 连续 accepted 结果')
            return True

        pcl_age = now - self.latest_pcl_pose_time
        if self.latest_pcl_pose_time > 0.0 and pcl_age > self.recovery_pose_max_pcl_age_sec:
            self.get_logger().warn(
                '[LOST] recovery 软验收放行: /pcl_pose 较旧 '
                f'({pcl_age:.2f}s)，不使用旧 pose 做硬拒绝')
            return True

        odom_body = self._lookup_odom_body()
        if self.frozen_map_odom is None or self.frozen_odom_body is None or odom_body is None:
            self.get_logger().warn(
                '[LOST] recovery 软验收放行: frozen/odom 信息不完整，跳过 odom 位置校验')
            return True

        odom_displacement = self._compute_odom_displacement(odom_body)
        if odom_displacement > self.recovery_pose_skip_odom_after_displacement_m:
            self.get_logger().warn(
                '[LOST] recovery 软验收放行: odom 位移已较大 '
                f'({odom_displacement:.2f}m > '
                f'{self.recovery_pose_skip_odom_after_displacement_m:.2f}m)，'
                '不再用 odom 约束全局重定位')
            return True

        recovery_delta = math.hypot(
            pcl_odom['x'] - self.frozen_map_odom['x'],
            pcl_odom['y'] - self.frozen_map_odom['y'])
        allowed_delta = odom_displacement + self.recovery_pose_max_xy_error_m
        if recovery_delta <= allowed_delta:
            self.get_logger().info(
                '[LOST] recovery 软验收通过: '
                f'recovery_delta={recovery_delta:.2f}m <= '
                f'odom_displacement({odom_displacement:.2f}m)+'
                f'margin({self.recovery_pose_max_xy_error_m:.2f}m), '
                f'NDT error={self.latest_ndt_error:.4f}')
            return True

        if self.latest_ndt_error < self.recovery_pose_accept_if_ndt_error_below:
            self.get_logger().warn(
                '[LOST] recovery 软验收放行但报警: recovery_delta='
                f'{recovery_delta:.2f}m > allowed={allowed_delta:.2f}m, '
                f'但 NDT error={self.latest_ndt_error:.4f} < '
                f'{self.recovery_pose_accept_if_ndt_error_below:.4f}')
            return True

        self.get_logger().warn(
            '[LOST] recovery 软验收拒绝: recovery_delta='
            f'{recovery_delta:.2f}m > allowed={allowed_delta:.2f}m, '
            f'odom_displacement={odom_displacement:.2f}m, '
            f'NDT error={self.latest_ndt_error:.4f}')
        return False

    def _compute_odom_displacement(self, odom_body: dict) -> float:
        """
        计算从冻结点到当前位置的里程计累积位移（2D 水平面）

        camera_init 坐标系 (非标准):
          x = 左右 (left+),  y = 垂直 (down+),  z = 前后 (back+)
        2D 水平面对应 (x, z) 平面, y 是高度不参与位移计算

        Args:
            odom_body: 当前 camera_init->body

        Returns:
            2D 水平位移距离（米），如果 frozen_odom_body 为空则返回 0
        """
        if self.frozen_odom_body is None or odom_body is None:
            return 0.0

        dx = odom_body['x'] - self.frozen_odom_body[0]   # 左右
        dz = odom_body['z'] - self.frozen_odom_body[2]   # 前后 (back+)
        return math.hypot(dx, dz)

    # =========================================================================
    # TF 发布
    # =========================================================================

    def _publish_map_odom_tf(self, map_odom: dict):
        """
        发布 map->odom TF 变换

        这是融合节点的核心输出。下游 TF 树会自动计算:
        map_T_base = map_T_odom × odom_T_camera_init × camera_init_T_body × body_T_base

        只需要发布正确的 map->odom，整个定位系统就正确了。

        Args:
            map_odom: 要发布的 map->odom 值 dict
        """
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = FRAME_MAP
        t.child_frame_id = FRAME_ODOM

        t.transform.translation.x = map_odom['x']
        t.transform.translation.y = map_odom['y']
        t.transform.translation.z = map_odom['z']
        t.transform.rotation.x = map_odom['qx']
        t.transform.rotation.y = map_odom['qy']
        t.transform.rotation.z = map_odom['qz']
        t.transform.rotation.w = map_odom['qw']

        self.tf_broadcaster.sendTransform(t)

    # =========================================================================
    # 诊断发布
    # =========================================================================

    def _publish_fusion_status(self):
        """发布融合状态诊断消息"""
        msg = String()
        now = time.strftime('%Y-%m-%d %H:%M:%S')

        status = {
            'state': self.state,
            'ndt_error': self.latest_ndt_error,
            'ndt_inlier': self.latest_ndt_inlier,
            'time': now,
        }

        if self.state == FusionState.DEGRADED:
            elapsed = time.monotonic() - self.degraded_start_time
            status['degraded_elapsed_sec'] = elapsed
            if self.frozen_map_odom:
                status['frozen_map_odom'] = (
                    f'{self.frozen_map_odom["x"]:.3f},'
                    f'{self.frozen_map_odom["y"]:.3f}')
        elif self.state == FusionState.TRANSITIONING:
            elapsed = time.monotonic() - self.transition_start_time
            status['transition_progress'] = min(
                elapsed / self.transition_duration_sec, 1.0)

        import json as _json
        msg.data = _json.dumps(status, ensure_ascii=False)
        self.fusion_status_pub.publish(msg)

    def _publish_diagnostics(self):
        """发布诊断数据"""
        # 发布里程计累积位移
        odom_body = self._lookup_odom_body()
        if odom_body is not None:
            disp = self._compute_odom_displacement(odom_body)
            self.odom_displacement_pub.publish(Float64(data=disp))

        # 发布融合状态
        self._publish_fusion_status()

        # 状态变化日志（限流）
        now = time.monotonic()
        if (self.verbose_logging and
            now - self.last_state_log_time > self.state_log_interval_sec):
            self.last_state_log_time = now
            if self.state != FusionState.HEALTHY:
                self.get_logger().info(
                    f'状态: {self.state} | '
                    f'NDT error={self.latest_ndt_error:.4f} '
                    f'inlier={self.latest_ndt_inlier:.3f} '
                    f'converged={self.latest_ndt_converged}',
                    throttle_duration_sec=2.0)


# =============================================================================
# 入口函数
# =============================================================================

def main(args=None):
    """融合节点主入口"""
    rclpy.init(args=args)
    node = LocalizationOdomFusion()

    # 使用 MultiThreadedExecutor 确保 TF 回调和定时器回调不会互相阻塞
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
