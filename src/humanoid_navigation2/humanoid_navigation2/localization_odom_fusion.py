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

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String, Float64
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from hdl_localization.msg import ScanMatchingStatus

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

        # 最大 odom 位移（米），从冻结点算起，超过进入 LOST
        # Fast-LIO 漂移率约 0.5cm/m，30m 位移 ≈ 15cm 漂移，可接受
        self.max_odom_displacement_m = float(
            self.declare_parameter('max_odom_displacement_m', 30.0).value)

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

        # ── 平滑过渡参数 ──
        # DEGRADED→HEALTHY 平滑过渡时间（秒）
        # 在此时长内从 frozen_map_odom 插值到 ndt_current_map_odom
        self.transition_duration_sec = float(
            self.declare_parameter('transition_duration_sec', 2.0).value)

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
        # 消息类型: hdl_localization/msg/ScanMatchingStatus
        # 关键字段: has_converged(bool), matching_error(float32), inlier_fraction(float32)
        self.status_sub = self.create_subscription(
            ScanMatchingStatus,
            '/localization/ndt_status',
            self._on_ndt_status,
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

        # =====================================================================
        # 内部状态初始化
        # =====================================================================

        # 当前状态
        self.state = FusionState.HEALTHY

        # 最新的 NDT 状态数据
        self.latest_ndt_error = float('inf')
        self.latest_ndt_inlier = 0.0
        self.latest_ndt_converged = False
        self.latest_ndt_status_time = 0.0

        # 最新的 NDT map->odom（从 TF 获取）
        self.latest_ndt_map_odom = None  # dict with keys: x, y, z, qx, qy, qz, qw

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
            f'  状态转换: HEALTHY ←→ DEGRADED (error>{self.degraded_error_threshold})\n'
            f'  恢复阈值: error<{self.healthy_error_threshold} 连续{self.healthy_consecutive_frames}帧\n'
            f'  超时限制: {self.max_degraded_duration_sec}s / {self.max_odom_displacement_m}m\n'
            f'  过渡时间: {self.transition_duration_sec}s\n'
            f'  发布频率: {self.publish_rate_hz}Hz'
        )

    # =========================================================================
    # 回调函数
    # =========================================================================

    def _on_ndt_status(self, msg: ScanMatchingStatus):
        """
        接收 NDT 扫描匹配状态回调

        由 lidar_localization 节点在每次 NDT 匹配后发布。
        我们用它来判断 NDT 定位是否可靠。

        Args:
            msg: ScanMatchingStatus 消息
                 - has_converged: 是否收敛
                 - matching_error: 匹配误差（类似 fitness score）
                 - inlier_fraction: 内点比例
        """
        self.latest_ndt_error = msg.matching_error
        self.latest_ndt_inlier = msg.inlier_fraction
        self.latest_ndt_converged = msg.has_converged
        self.latest_ndt_status_time = time.monotonic()

    def _on_recovery_status(self, msg: String):
        """
        接收 recovery 状态回调

        当 recovery 成功完成时（navigation_localization_recovered），
        从 LOST 状态恢复到 HEALTHY，接受新的 map->odom。

        Args:
            msg: 恢复状态 JSON 字符串
        """
        if self.state != FusionState.LOST:
            return

        # 检查是否是 recovery 成功事件
        if 'localization_recovered' in msg.data or \
           'localization_initialpose_published' in msg.data:
            self.get_logger().info(
                f'[LOST→HEALTHY] 检测到 recovery 成功，切回 HEALTHY 并接受新 map->odom')
            self._reset_state()
            self.state = FusionState.HEALTHY
            self._publish_fusion_status()

    def _on_nav_status(self, msg: String):
        """
        接收导航状态回调（★ 用于区分导航中 vs 已到达静止）

        navigation_state_manager 发布 /navigation_status topic，
        格式: {"state": "EXECUTING"|"IDLE"|"COMPLETED"|"PAUSED"|...}

        作用: 当机器人到达目标后静止播报时，NDT 即使漂移也不需要触发 LOST，
              因为此时不需要定位精度。延长静止时的 LOST 超时到 600s。

        Args:
            msg: 导航状态 JSON 字符串
        """
        try:
            import json
            data = json.loads(msg.data)
            new_state = data.get('state', 'IDLE')
            if new_state != self.nav_state:
                self.get_logger().info(
                    f'[NAV] 导航状态变更: {self.nav_state} → {new_state}',
                    throttle_duration_sec=2.0)
            self.nav_state = new_state
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
            if self.state == FusionState.HEALTHY:
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

        # ── 退化检测 ──
        if self._is_degraded():
            self.consecutive_degraded += 1
            if (self.verbose_logging and
                self.consecutive_degraded == 1):
                self.get_logger().warn(
                    f'检测到 NDT 退化: error={self.latest_ndt_error:.4f} '
                    f'> {self.degraded_error_threshold}，'
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
        - 检查 NDT 是否恢复
        - 检查超时/位移是否过大

        关键设计决策:
        - 我们不修改 map->odom（保持冻结值），只由 odom 提供运动
        - TF 树: map_T_base = frozen_map_T_odom × 当前odom_T_body × body_T_base
        - 这样做的好处: NDT 的 initial guess 仍然基于正确的 map->odom
        - 当 NDT 恢复时，匹配结果接近正确值，不需要全局重定位

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

        # ── 检查恢复条件 ──
        if self._is_healthy():
            self.consecutive_healthy += 1
            if self.verbose_logging and self.consecutive_healthy == 1:
                self.get_logger().info(
                    f'NDT 开始恢复: error={self.latest_ndt_error:.4f} '
                    f'< {self.healthy_error_threshold}，'
                    f'需连续{self.healthy_consecutive_frames}帧确认')
        else:
            self.consecutive_healthy = 0

        if self.consecutive_healthy >= self.healthy_consecutive_frames:
            self._enter_transitioning()
            return

        # ── 检查超时条件（★ 导航感知: 导航中 vs 静���不同超时）──
        elapsed = time.monotonic() - self.degraded_start_time
        odom_displacement = self._compute_odom_displacement(odom_body)

        # ★ 根据导航状态选择不同的 LOST 超时
        if self._is_robot_navigating():
            lost_timeout = self.nav_active_lost_timeout_sec
            timeout_label = 'nav_active'
        else:
            lost_timeout = self.nav_idle_lost_timeout_sec
            timeout_label = 'nav_idle'
            # 静止时额外检查: NDT error 极端高 (>5.0) 且持续超过超时一半
            # 说明 NDT 彻底挂了，不是几何混叠，该恢复了
            if self.latest_ndt_error > self.nav_idle_extreme_error:
                lost_timeout = min(lost_timeout, self.nav_idle_lost_timeout_sec / 2.0)
                timeout_label = 'nav_idle_extreme_error'

        if elapsed > lost_timeout:
            self.get_logger().warn(
                f'[DEGRADED→LOST] 超时 ({timeout_label}): {elapsed:.1f}s > '
                f'{lost_timeout:.0f}s (nav_state={self.nav_state})')
            self._enter_lost()
            return

        if odom_displacement > self.max_odom_displacement_m:
            self.get_logger().warn(
                f'[DEGRADED→LOST] odom 位移过大: {odom_displacement:.2f}m > '
                f'{self.max_odom_displacement_m}m')
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
            self._publish_fusion_status()

    def _update_lost(self, ndt_map_odom: dict, odom_body: dict):
        """
        LOST 状态：长时间退化，等待 recovery

        行为:
        - 不发布 map->odom（让 recovery 机制接管）
        - 定期输出诊断日志
        - recovery 成功后由 _on_recovery_status 切回 HEALTHY

        Args:
            ndt_map_odom: 当前 NDT 的 map->odom
            odom_body: 当前 Fast-LIO 的 camera_init->body
        """
        if ndt_map_odom is not None:
            self.latest_ndt_map_odom = ndt_map_odom

        # 在 LOST 状态下不发布 map->odom
        # 让 hdl_bootstrap_to_initialpose 的 recovery 机制接管
        # recovery 会: 清除缓存 → 等待静止 → 全局重定位 → 发布 initialpose → 恢复

        now = time.monotonic()
        if now - self.last_state_log_time > 5.0:
            self.last_state_log_time = now
            self.get_logger().warn(
                f'[LOST] 等待 recovery... '
                f'(error={self.latest_ndt_error:.4f})')

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
        self.frozen_odom_body = (self.last_healthy_odom_body.copy()
                                  if self.last_healthy_odom_body else None)
        self.degraded_start_time = time.monotonic()
        self.consecutive_healthy = 0
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
        如果 NDT 值不可用，保持 DEGRADED。
        """
        if self.latest_ndt_map_odom is None:
            self.get_logger().warn(
                '[DEGRADED→TRANSITIONING] 拒绝: 无法获取 NDT map->odom，保持 DEGRADED')
            self.consecutive_healthy = 0
            return

        self.state = FusionState.TRANSITIONING
        self.transition_start_time = time.monotonic()
        self.transition_from = self.frozen_map_odom.copy()
        self.transition_to = self.latest_ndt_map_odom.copy()

        # 计算跳变距离（用于诊断）
        dx = self.transition_to['x'] - self.transition_from['x']
        dy = self.transition_to['y'] - self.transition_from['y']
        jump = math.hypot(dx, dy)

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

        停止发布 map->odom，等待 recovery 机制接管。
        """
        self.state = FusionState.LOST
        elapsed = time.monotonic() - self.degraded_start_time

        self.get_logger().error(
            '========== [DEGRADED→LOST] 等待 recovery ==========\n'
            f'  冻结持续时间: {elapsed:.1f}s\n'
            f'  NDT error: {self.latest_ndt_error:.4f}\n'
            f'  最后健康 map->odom: ({self.frozen_map_odom["x"]:.3f}, '
            f'{self.frozen_map_odom["y"]:.3f})')

        self._publish_fusion_status()

    def _reset_state(self):
        """重置所有内部状态变量"""
        self.consecutive_healthy = 0
        self.consecutive_degraded = 0
        self.frozen_map_odom = None
        self.frozen_odom_body = None
        self.degraded_start_time = 0.0
        self.transition_from = None
        self.transition_to = None

    # =========================================================================
    # 状态判断辅助函数
    # =========================================================================

    def _is_degraded(self) -> bool:
        """
        判断当前 NDT 是否退化

        退化条件（满足任一即退化）:
        1. matching_error > degraded_error_threshold (默认 0.5)
        2. NDT 未收敛

        注意: 不使用 inlier_fraction 判断，因为在某些环境下 inlier
        可能偏低但定位仍然正确

        Returns:
            True 如果 NDT 当前退化
        """
        # NDT 状态数据过期检查（超过 3 秒没有新数据认为通信异常）
        status_age = time.monotonic() - self.latest_ndt_status_time
        if status_age > 3.0:
            # 状态数据过期，保守起见认为退化
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

        使用比 _is_degraded 更严格的阈值（0.15 vs 0.5），
        形成滞回区间，防止频繁振荡

        Returns:
            True 如果 NDT 当前健康
        """
        return (self.latest_ndt_error < self.healthy_error_threshold and
                self.latest_ndt_converged)

    def _compute_odom_displacement(self, odom_body: dict) -> float:
        """
        计算从冻结点到当前位置的里程计累积位移（2D 水平面）

        Args:
            odom_body: 当前 camera_init->body

        Returns:
            2D 位移距离（米），如果 frozen_odom_body 为空则返回 0
        """
        if self.frozen_odom_body is None or odom_body is None:
            return 0.0

        dx = odom_body['x'] - self.frozen_odom_body['x']
        dy = odom_body['y'] - self.frozen_odom_body['y']
        return math.hypot(dx, dy)

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

        msg.data = str(status)
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
