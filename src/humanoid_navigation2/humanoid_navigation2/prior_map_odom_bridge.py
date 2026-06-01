#!/usr/bin/env python3
"""
prior_map_odom_bridge.py

这个节点的职责很单一：

1. 接收“外部先验地图定位节点”给出的全局位姿候选。
   例如 FAST_LIO_LOCALIZATION_HUMANOID / FAST-LIO prior-map localization
   输出的机器人在 map 下的位姿。

2. 不让外部定位节点直接发布 map->odom。
   外部节点只负责回答“机器人现在大概在 map 的哪里”；
   本节点负责判断这个结果能不能信，然后维护 Nav2 需要的 map->odom。

3. 通过当前 TF 树中的 odom->base_footprint，计算 map->odom：

      T_map_odom = T_map_localized_frame * inverse(T_odom_localized_frame)

   默认 localized_frame 是 base_footprint。如果外部定位输出的是 map->body，
   可以把参数 localized_frame 改成 body。

4. 对 map->odom 更新做门控：
   - 首帧可以直接接收，用来建立初始 TF；
   - 小修正可以直接接收；
   - 大修正必须连续多帧一致；
   - 超过最大跳变阈值的候选直接拒绝；
   - 候选位姿太旧、frame_id 不对、TF 查不到时拒绝。
   - 可选启用第一版大跳保护：导航中大于阈值的修正先冻结，
     继续发布 last good map->odom，等待候选自己恢复或交给状态管理器处理。

注意：
这个节点只解决“谁来维护 map->odom”的问题，不做点云配准。
外部定位节点的可靠性仍然需要靠它自己的 fitness/overlap/重定位逻辑保证。
"""

import json
import math
from collections import deque
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]，用于比较 yaw 差值。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float, float, float, float]:
    """归一化四元数；如果输入坏了，就退回单位四元数。"""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def quaternion_to_matrix(q) -> np.ndarray:
    """把 ROS 四元数转换成 3x3 旋转矩阵。"""
    x, y, z, w = normalize_quaternion(q.x, q.y, q.z, q.w)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    """把 3x3 旋转矩阵转换成 ROS 四元数 (x, y, z, w)。"""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion(x, y, z, w)


def quaternion_tuple_from_pose(pose) -> Tuple[float, float, float, float]:
    """从 ROS Pose 中取出并归一化四元数，返回 (x, y, z, w)。"""
    return normalize_quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


def quaternion_slerp(
    qa: Tuple[float, float, float, float],
    qb: Tuple[float, float, float, float],
    ratio: float,
) -> Tuple[float, float, float, float]:
    """
    两个四元数之间做球面插值。

    bridge 的 odom cache 用它在两个 odom 采样之间恢复“同一时间戳”的
    odom->localized_frame。这里不做坐标轴变换，只在 axis_adapter 已经输出的
    标准 ROS 坐标系内插值。
    """
    ax, ay, az, aw = normalize_quaternion(*qa)
    bx, by, bz, bw = normalize_quaternion(*qb)
    dot = ax * bx + ay * by + az * bz + aw * bw

    # 四元数 q 和 -q 表示同一个旋转。取夹角较小的一侧，避免插值绕远路。
    if dot < 0.0:
        bx, by, bz, bw = -bx, -by, -bz, -bw
        dot = -dot

    ratio = max(0.0, min(1.0, ratio))
    if dot > 0.9995:
        x = ax + ratio * (bx - ax)
        y = ay + ratio * (by - ay)
        z = az + ratio * (bz - az)
        w = aw + ratio * (bw - aw)
        return normalize_quaternion(x, y, z, w)

    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * ratio
    sin_theta = math.sin(theta)
    scale_a = math.cos(theta) - dot * sin_theta / sin_theta_0
    scale_b = sin_theta / sin_theta_0
    return normalize_quaternion(
        scale_a * ax + scale_b * bx,
        scale_a * ay + scale_b * by,
        scale_a * az + scale_b * bz,
        scale_a * aw + scale_b * bw,
    )


def yaw_from_matrix(transform: np.ndarray) -> float:
    """从 4x4 位姿矩阵中提取平面 yaw。"""
    return math.atan2(transform[1, 0], transform[0, 0])


def pose_to_matrix(pose) -> np.ndarray:
    """把 geometry_msgs/Pose 转成 4x4 齐次变换矩阵。"""
    transform = np.eye(4)
    transform[:3, :3] = quaternion_to_matrix(pose.orientation)
    transform[0, 3] = pose.position.x
    transform[1, 3] = pose.position.y
    transform[2, 3] = pose.position.z
    return transform


def odom_msg_to_matrix(msg: Odometry) -> np.ndarray:
    """把 Odometry.pose.pose 转成 4x4 齐次矩阵。"""
    return pose_to_matrix(msg.pose.pose)


def transform_to_matrix(transform_stamped: TransformStamped) -> np.ndarray:
    """把 geometry_msgs/TransformStamped 转成 4x4 齐次变换矩阵。"""
    transform = np.eye(4)
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    transform[:3, :3] = quaternion_to_matrix(q)
    transform[0, 3] = t.x
    transform[1, 3] = t.y
    transform[2, 3] = t.z
    return transform


def correction_delta(a: np.ndarray, b: np.ndarray) -> Tuple[float, float]:
    """
    计算两个 map->odom 候选之间的平面差异。

    返回:
      translation: XY 平移差，单位米
      yaw: yaw 差，单位弧度
    """
    dx = float(a[0, 3] - b[0, 3])
    dy = float(a[1, 3] - b[1, 3])
    dyaw = normalize_angle(yaw_from_matrix(a) - yaw_from_matrix(b))
    return math.hypot(dx, dy), abs(dyaw)


class PriorMapOdomBridge(Node):
    """
    外部先验地图定位 -> map->odom 的桥接节点。

    它是 TF owner：
      - 本节点发布 map->odom；
      - Fast-LIO 继续发布 odom/camera_init 下的连续局部运动；
      - 外部 prior-map 定位节点不应该再发布 map->odom。
    """

    def __init__(self):
        super().__init__("prior_map_odom_bridge")

        # ==============================
        # 坐标系参数
        # ==============================
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.localized_frame = self.declare_parameter("localized_frame", "base_footprint").value

        # 外部定位节点输出的话题。
        # PoseStamped: /prior_localization/pose
        # PoseWithCovarianceStamped: /prior_localization/pose_with_covariance
        self.prior_pose_topic = self.declare_parameter(
            "prior_pose_topic", "/prior_localization/pose").value
        self.prior_pose_with_covariance_topic = self.declare_parameter(
            "prior_pose_with_covariance_topic", "/prior_localization/pose_with_covariance").value
        self.prior_odom_topic = self.declare_parameter(
            "prior_odom_topic", "/prior_localization/odom").value

        # 有些外部定位节点会把匹配置信度单独发布出来。
        # 例如 open3d_loc 的 /localization_3d_confidence 是 overlap/fitness，
        # 只有初始化并完成配准后才会发布。启用这个门控可以避免初始化前的默认位姿进入 TF。
        self.confidence_topic = self.declare_parameter(
            "confidence_topic", "/prior_localization/confidence").value
        self.require_confidence = bool(self.declare_parameter("require_confidence", False).value)
        self.min_confidence = float(self.declare_parameter("min_confidence", 0.5).value)
        self.confidence_timeout_sec = float(
            self.declare_parameter("confidence_timeout_sec", 1.5).value)

        # ==============================
        # 时间和发布参数
        # ==============================
        self.publish_rate = float(self.declare_parameter("publish_rate", 30.0).value)
        self.tf_lookup_timeout_sec = float(self.declare_parameter("tf_lookup_timeout_sec", 0.08).value)
        self.pose_timeout_sec = float(self.declare_parameter("pose_timeout_sec", 0.8).value)
        self.accept_zero_stamp = bool(self.declare_parameter("accept_zero_stamp", True).value)
        self.allow_initial_pose = bool(self.declare_parameter("allow_initial_pose", True).value)

        # ==============================
        # odom cache 时间同步参数
        # ==============================
        # 方案 4：bridge 直接订阅 axis_adapter 输出的标准轴 odom，并按 prior pose
        # 的 stamp 插值出同一时刻的 odom->localized_frame。
        #
        # 注意：这里订阅的不是原始 /odom，而是 fastlio_open3d_axis_adapter
        # 输出的 /prior_localization/open3d_input_odom。它已经把本系统 raw Fast-LIO
        # 的 x左/y下/z后坐标轴转换成 ROS 标准导航坐标轴，并且 child_frame_id
        # 应该等于 localized_frame。
        self.use_odom_cache = bool(self.declare_parameter("use_odom_cache", True).value)
        self.odom_cache_topic = self.declare_parameter(
            "odom_cache_topic", "/prior_localization/open3d_input_odom").value
        self.odom_cache_duration_sec = float(
            self.declare_parameter("odom_cache_duration_sec", 5.0).value)
        self.odom_interpolation_max_gap_sec = float(
            self.declare_parameter("odom_interpolation_max_gap_sec", 0.25).value)
        self.odom_lookup_tolerance_sec = float(
            self.declare_parameter("odom_lookup_tolerance_sec", 0.03).value)
        self.odom_future_wait_sec = float(
            self.declare_parameter("odom_future_wait_sec", 0.20).value)
        self.fallback_to_tf_lookup = bool(
            self.declare_parameter("fallback_to_tf_lookup", True).value)

        # ==============================
        # 修正门控参数
        # ==============================
        # 小修正：认为是正常地图约束漂移修正，直接接受。
        self.max_small_correction_translation = float(
            self.declare_parameter("max_small_correction_translation", 0.25).value)
        self.max_small_correction_yaw = float(
            self.declare_parameter("max_small_correction_yaw", 0.12).value)

        # 大修正：可能是重定位结果。不能单帧接受，必须连续一致。
        self.max_large_correction_translation = float(
            self.declare_parameter("max_large_correction_translation", 3.0).value)
        self.max_large_correction_yaw = float(
            self.declare_parameter("max_large_correction_yaw", 1.2).value)
        self.required_consistent_frames = int(
            self.declare_parameter("required_consistent_frames", 5).value)
        self.consistency_translation_tolerance = float(
            self.declare_parameter("consistency_translation_tolerance", 0.25).value)
        self.consistency_yaw_tolerance = float(
            self.declare_parameter("consistency_yaw_tolerance", 0.10).value)

        # ==============================
        # 第一版大跳保护参数
        # ==============================
        # jump_protection_mode:
        #   off     : 完全沿用原来的“连续多帧一致就接受”逻辑；
        #   monitor : 不改变实际 TF，只额外发布 WOULD_* 状态，方便离线/实机观察；
        #   protect : 真正启用导航中大跳冻结保护。
        self.jump_protection_mode = str(
            self.declare_parameter("jump_protection_mode", "off").value).lower()
        if self.jump_protection_mode not in ("off", "monitor", "protect"):
            self.get_logger().warn(
                f"unknown jump_protection_mode={self.jump_protection_mode}, fallback to off"
            )
            self.jump_protection_mode = "off"

        # 导航中 0.25m~0.50m 的中等修正不立即放行，连续稳定后接受。
        self.nav_medium_correction_translation = float(
            self.declare_parameter("nav_medium_correction_translation", 0.50).value)
        self.nav_medium_correction_yaw = float(
            self.declare_parameter("nav_medium_correction_yaw", 0.20).value)
        self.nav_medium_required_frames = int(
            self.declare_parameter("nav_medium_required_frames", 5).value)

        # 导航中超过该阈值的修正认为是大跳。protect 模式下不直接接受，
        # 而是保持 last good map->odom，让机器人暂时只靠 odom 连续性走。
        self.nav_large_correction_translation = float(
            self.declare_parameter("nav_large_correction_translation", 0.50).value)
        self.nav_large_correction_yaw = float(
            self.declare_parameter("nav_large_correction_yaw", 0.20).value)
        self.allow_nav_large_jump = bool(
            self.declare_parameter("allow_nav_large_jump", False).value)

        # 非导航/讲解/空闲阶段允许更大的回正，但仍需要连续稳定。
        self.idle_large_correction_translation = float(
            self.declare_parameter("idle_large_correction_translation", 1.00).value)
        self.idle_large_correction_yaw = float(
            self.declare_parameter("idle_large_correction_yaw", 0.35).value)
        self.idle_large_required_frames = int(
            self.declare_parameter("idle_large_required_frames", 5).value)
        self.allow_idle_large_jump = bool(
            self.declare_parameter("allow_idle_large_jump", True).value)

        # 过大的候选即使在空闲阶段也不自动接受，避免一次错误局部最优把全局位姿拉飞。
        self.hard_reject_translation = float(
            self.declare_parameter("hard_reject_translation", 1.00).value)
        self.hard_reject_yaw = float(
            self.declare_parameter("hard_reject_yaw", 0.50).value)

        # 大跳冻结超过该时间后发布 DEGRADED 状态。bridge 自身仍持续发布 last good TF；
        # 后续可由 navigation_state_manager 根据该状态选择暂停/等待/重定位。
        self.large_jump_degraded_after_sec = float(
            self.declare_parameter("large_jump_degraded_after_sec", 3.0).value)

        # ==============================
        # SpinToPose 旋转保护参数
        # ==============================
        # 这里对齐 lidar_localization 的 rotation_guard 逻辑：
        #   - 只在导航状态进入 TURNING，也就是 BT 执行 SpinToPose 阶段时冻结；
        #   - 冻结期间不接受任何外部定位候选，只持续发布 last good map->odom；
        #   - 退出 TURNING 后继续等待 settle_sec，再恢复正常定位更新。
        self.enable_spin_to_pose_guard = bool(
            self.declare_parameter("enable_spin_to_pose_guard", True).value)
        self.navigation_status_topic = self.declare_parameter(
            "navigation_status_topic", "/navigation/status").value
        self.spin_to_pose_guard_settle_sec = float(
            self.declare_parameter("spin_to_pose_guard_settle_sec", 1.2).value)
        self.spin_to_pose_guard_max_duration_sec = float(
            self.declare_parameter("spin_to_pose_guard_max_duration_sec", 8.0).value)

        # 输出约束：Nav2 只需要平面定位时，建议固定 map->odom 的 z/roll/pitch。
        self.force_2d = bool(self.declare_parameter("force_2d", True).value)
        self.force_z = float(self.declare_parameter("force_z", 0.0).value)

        # ==============================
        # ROS 通信对象
        # ==============================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.status_pub = self.create_publisher(String, "/localization/prior_map_odom_bridge_status", 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, self.prior_pose_topic, self.on_pose_stamped, 10)
        self.pose_cov_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.prior_pose_with_covariance_topic,
            self.on_pose_with_covariance,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.prior_odom_topic, self.on_odometry, 10)
        self.odom_cache_sub = self.create_subscription(
            Odometry, self.odom_cache_topic, self.on_odom_cache, 100)
        self.confidence_sub = self.create_subscription(
            Float32, self.confidence_topic, self.on_confidence, 10)
        self.navigation_status_sub = self.create_subscription(
            String, self.navigation_status_topic, self.on_navigation_status, 10)

        if self.publish_rate <= 0.0:
            self.publish_rate = 30.0
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_last_tf)

        # 当前被接受的 map->odom。没有接受任何候选前，不发布 map->odom。
        self.accepted_map_to_odom: Optional[np.ndarray] = None
        self.last_accept_time = self.get_clock().now()

        # 大修正的一致性队列。这里只保存候选 map->odom，而不是原始 pose。
        self.pending_large_candidate: Optional[np.ndarray] = None
        self.pending_large_count = 0
        self.latest_confidence: Optional[float] = None
        self.latest_confidence_time: Optional[Time] = None
        self.odom_cache = deque()
        self.pending_odom_candidates = deque()
        self.last_odom_cache_failure = ""
        self.navigation_active = False
        self.large_jump_hold_start_time: Optional[Time] = None
        self.large_jump_hold_reason = ""
        self.spin_guard_active = False
        self.spin_guard_turning = False
        self.spin_guard_start_time: Optional[Time] = None
        self.spin_guard_settle_until: Optional[Time] = None
        self.spin_guard_source = ""

        self.get_logger().info(
            "prior_map_odom_bridge started: "
            f"input={self.prior_pose_topic}, {self.prior_pose_with_covariance_topic}, "
            f"or {self.prior_odom_topic}, "
            f"localized_frame={self.localized_frame}, publishing {self.map_frame}->{self.odom_frame}, "
            f"use_odom_cache={self.use_odom_cache}, odom_cache_topic={self.odom_cache_topic}"
        )

    def on_pose_stamped(self, msg: PoseStamped):
        """处理外部定位节点发布的 PoseStamped。"""
        self.handle_candidate_pose(msg.header, msg.pose)

    def on_pose_with_covariance(self, msg: PoseWithCovarianceStamped):
        """处理外部定位节点发布的 PoseWithCovarianceStamped。"""
        self.handle_candidate_pose(msg.header, msg.pose.pose)

    def on_odometry(self, msg: Odometry):
        """
        处理外部定位节点发布的 Odometry。

        这里不使用 msg.child_frame_id 参与计算，只把 msg.pose.pose 当作
        header.frame_id -> localized_frame 的候选全局位姿。

        例如 open3d_loc 的 /baselink2map：
          header.frame_id = map
          pose = map -> 当前 FAST-LIO body/base 数值位姿
        启动 launch 时把 localized_frame 设为 body 即可。
        """
        self.handle_candidate_pose(msg.header, msg.pose.pose)

    def on_odom_cache(self, msg: Odometry):
        """
        缓存 odom->localized_frame，用于按 prior pose 的 stamp 精确取同一时刻 odom。

        这里必须非常严格地检查 frame：
          - header.frame_id 必须是 odom_frame；
          - child_frame_id 必须是 localized_frame；
        否则就可能把不同坐标轴或不同机器人基准混进 map->odom 计算。
        """
        if not self.use_odom_cache:
            return

        if msg.header.frame_id and msg.header.frame_id != self.odom_frame:
            self.publish_status(
                f"REJECTED odom_cache_wrong_frame frame_id={msg.header.frame_id} expected={self.odom_frame}"
            )
            return

        if msg.child_frame_id and msg.child_frame_id != self.localized_frame:
            self.publish_status(
                "REJECTED odom_cache_wrong_child "
                f"child_frame_id={msg.child_frame_id} expected={self.localized_frame}"
            )
            return

        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if stamp <= 0.0:
            return

        entry = (stamp, odom_msg_to_matrix(msg), msg)
        if self.odom_cache and stamp < self.odom_cache[-1][0]:
            # bag 或仿真偶发乱序时保持时间有序，方便后面做插值。
            inserted = False
            for index, cached in enumerate(self.odom_cache):
                if stamp < cached[0]:
                    self.odom_cache.insert(index, entry)
                    inserted = True
                    break
            if not inserted:
                self.odom_cache.append(entry)
        else:
            self.odom_cache.append(entry)

        cutoff = stamp - self.odom_cache_duration_sec
        while self.odom_cache and self.odom_cache[0][0] < cutoff:
            self.odom_cache.popleft()

        self.process_pending_odom_candidates()

    def on_confidence(self, msg: Float32):
        """记录外部定位节点的最新匹配置信度。"""
        self.latest_confidence = float(msg.data)
        self.latest_confidence_time = self.get_clock().now()

    def on_navigation_status(self, msg: String):
        """根据导航状态识别到点后的 SpinToPose 原地旋转阶段。"""
        self.navigation_active = self.navigation_status_is_active(msg.data)

        if not self.enable_spin_to_pose_guard:
            return

        turning = self.navigation_status_is_turning(msg.data)
        now = self.get_clock().now()
        if turning:
            self.enter_spin_guard(now, "navigation_status")
            self.spin_guard_turning = True
            return

        if self.spin_guard_turning:
            self.spin_guard_turning = False
            self.spin_guard_settle_until = now + Duration(seconds=self.spin_to_pose_guard_settle_sec)
            self.publish_status(
                f"SPIN_GUARD settling sec={self.spin_to_pose_guard_settle_sec:.2f}"
            )

    def navigation_status_is_turning(self, data: str) -> bool:
        """兼容 JSON 状态和字符串状态，判断是否处于 SpinToPose/TURNING。"""
        try:
            status = json.loads(data)
            current_state = str(status.get("current_state", "executing")).lower()
            detailed_state = str(
                status.get("current_detailed_state", status.get("detailed_state", ""))
            ).upper()
            navigation_active = current_state in ("executing", "planning", "running", "active")
            return navigation_active and detailed_state == "TURNING"
        except (TypeError, ValueError):
            return "TURNING" in data and "SpinToPose" in data

    def navigation_status_is_active(self, data: str) -> bool:
        """判断当前是否处于会驱动机器人移动的导航阶段。"""
        try:
            status = json.loads(data)
            current_state = str(status.get("current_state", "")).lower()
            detailed_state = str(
                status.get("current_detailed_state", status.get("detailed_state", ""))
            ).upper()
            if current_state in ("executing", "planning", "running", "active"):
                return True
            if detailed_state in ("NAVIGATING", "PLANNING", "CONTROLLING", "TURNING"):
                return True
            return False
        except (TypeError, ValueError):
            upper = str(data).upper()
            if "COMPLETED" in upper or "SUCCEEDED" in upper or "IDLE" in upper:
                return False
            return any(token in upper for token in ("NAVIGATING", "PLANNING", "RUNNING", "ACTIVE", "TURNING"))

    def lookup_odom_to_localized_matrix(self, lookup_time: Time, is_zero_stamp: bool) -> Optional[np.ndarray]:
        """
        取得 odom->localized_frame 矩阵。

        优先使用 odom cache，因为它直接来自 axis_adapter 输出给 open3d_loc 的
        同一个标准轴 odom 话题，避免 TF buffer 中 0.1s future extrapolation。
        如果 cache 不可用，再按参数 fallback 到 TF 查询，保证兼容旧链路。
        """
        self.last_odom_cache_failure = ""
        if self.use_odom_cache:
            cached = self.lookup_odom_cache_matrix(lookup_time, is_zero_stamp)
            if cached is not None:
                return cached
            if self.last_odom_cache_failure == "future":
                return None
            if not self.fallback_to_tf_lookup:
                return None

        try:
            tf_lookup_time = Time() if is_zero_stamp else lookup_time
            odom_to_localized = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.localized_frame,
                tf_lookup_time,
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
            return transform_to_matrix(odom_to_localized)
        except TransformException as exc:
            self.publish_status(
                f"REJECTED missing_tf {self.odom_frame}->{self.localized_frame}: {exc}")
            return None

    def lookup_odom_cache_matrix(self, lookup_time: Time, is_zero_stamp: bool) -> Optional[np.ndarray]:
        """
        从缓存中按时间戳取 odom->localized_frame。

        - zero stamp：使用缓存中最新 odom，和原 TF latest 语义一致；
        - 精确命中：直接返回对应 odom；
        - 两帧之间：线性插值平移，slerp 插值姿态；
        - 小幅越界：在 tolerance 内允许使用最近一帧；
        - 大幅越界：拒绝，避免拿不同时间的 odom 硬凑导致 map->odom 误差。
        """
        if not self.odom_cache:
            self.last_odom_cache_failure = "empty"
            self.publish_status("REJECTED odom_cache_empty")
            return None

        if is_zero_stamp:
            return self.odom_cache[-1][1]

        target = lookup_time.nanoseconds * 1e-9
        first_time = self.odom_cache[0][0]
        last_time = self.odom_cache[-1][0]

        if target < first_time:
            if first_time - target <= self.odom_lookup_tolerance_sec:
                return self.odom_cache[0][1]
            self.last_odom_cache_failure = "too_old"
            self.publish_status(
                f"REJECTED odom_cache_too_old target={target:.6f} first={first_time:.6f}"
            )
            return None

        if target > last_time:
            if target - last_time <= self.odom_lookup_tolerance_sec:
                return self.odom_cache[-1][1]
            self.last_odom_cache_failure = "future"
            return None

        previous = None
        following = None
        for entry in self.odom_cache:
            if entry[0] <= target:
                previous = entry
            if entry[0] >= target:
                following = entry
                break

        if previous is None or following is None:
            self.last_odom_cache_failure = "lookup_failed"
            self.publish_status("REJECTED odom_cache_lookup_failed")
            return None

        previous_time, previous_matrix, previous_msg = previous
        following_time, following_matrix, following_msg = following

        if abs(target - previous_time) <= 1e-6:
            return previous_matrix
        if abs(following_time - target) <= 1e-6:
            return following_matrix

        gap = following_time - previous_time
        if gap <= 1e-9 or gap > self.odom_interpolation_max_gap_sec:
            self.last_odom_cache_failure = "gap"
            self.publish_status(
                f"REJECTED odom_cache_gap gap={gap:.3f}s max={self.odom_interpolation_max_gap_sec:.3f}s"
            )
            return None

        ratio = (target - previous_time) / gap
        interpolated = np.eye(4)
        interpolated[:3, 3] = (
            previous_matrix[:3, 3]
            + ratio * (following_matrix[:3, 3] - previous_matrix[:3, 3])
        )
        qa = quaternion_tuple_from_pose(previous_msg.pose.pose)
        qb = quaternion_tuple_from_pose(following_msg.pose.pose)
        qx, qy, qz, qw = quaternion_slerp(qa, qb, ratio)

        class QuaternionLike:
            pass

        q = QuaternionLike()
        q.x = qx
        q.y = qy
        q.z = qz
        q.w = qw
        interpolated[:3, :3] = quaternion_to_matrix(q)
        return interpolated

    def queue_pending_odom_candidate(self, lookup_time: Time, map_to_localized: np.ndarray) -> bool:
        """
        prior pose 已经到了，但同 stamp 的 odom 还没进 cache 时，短暂挂起候选。

        这是方案 4 的关键：不使用“最新 odom”硬凑，也不立刻丢帧，而是等
        axis_adapter 的标准轴 odom 到达后，用同一时间戳重新计算 map->odom。
        """
        if not self.use_odom_cache or not self.odom_cache:
            return False

        target = lookup_time.nanoseconds * 1e-9
        latest = self.odom_cache[-1][0]
        future_gap = target - latest
        if future_gap <= 0.0 or future_gap > self.odom_future_wait_sec:
            return False

        self.pending_odom_candidates.append(
            {
                "target": target,
                "lookup_time": lookup_time,
                "map_to_localized": map_to_localized,
                "queued_at": self.get_clock().now(),
            }
        )
        # open3d_loc 约 1Hz 输出，队列不需要很大；保留最近候选即可防止异常堆积。
        while len(self.pending_odom_candidates) > 10:
            self.pending_odom_candidates.popleft()
        self.publish_status(
            f"PENDING wait_odom_cache gap={future_gap:.3f}s max={self.odom_future_wait_sec:.3f}s"
        )
        return True

    def process_pending_odom_candidates(self):
        """每次 odom cache 更新后，尝试处理之前因 future odom 挂起的候选。"""
        if not self.pending_odom_candidates or not self.odom_cache:
            return

        now = self.get_clock().now()
        latest = self.odom_cache[-1][0]
        remaining = deque()

        while self.pending_odom_candidates:
            item = self.pending_odom_candidates.popleft()
            queued_age = (now - item["queued_at"]).nanoseconds * 1e-9
            if queued_age > self.odom_future_wait_sec + 0.10:
                self.publish_status(
                    f"REJECTED odom_cache_pending_timeout age={queued_age:.3f}s"
                )
                continue

            if latest + self.odom_lookup_tolerance_sec < item["target"]:
                remaining.append(item)
                continue

            odom_to_localized_matrix = self.lookup_odom_cache_matrix(item["lookup_time"], False)
            if odom_to_localized_matrix is None:
                # 如果 cache 已经覆盖目标时间但仍无法插值，说明数据间隔或时间范围异常。
                if self.last_odom_cache_failure == "future":
                    remaining.append(item)
                continue

            candidate = item["map_to_localized"] @ np.linalg.inv(odom_to_localized_matrix)
            candidate = self.apply_output_constraints(candidate)
            self.evaluate_candidate(candidate)

        self.pending_odom_candidates = remaining

    def enter_spin_guard(self, now: Time, source: str):
        """进入 SpinToPose 冻结窗口，并刷新旋转结束后的 settle 截止时间。"""
        if not self.spin_guard_active:
            self.spin_guard_start_time = now
            self.publish_status(f"SPIN_GUARD entered source={source}")
        self.spin_guard_active = True
        self.spin_guard_source = source
        self.spin_guard_settle_until = now + Duration(seconds=self.spin_to_pose_guard_settle_sec)

    def handle_candidate_pose(self, header, pose):
        """
        把外部定位 pose 转换为 map->odom 候选，并执行门控。

        输入 pose 的语义必须是:
          header.frame_id -> localized_frame

        默认就是:
          map -> base_footprint
        """
        if header.frame_id and header.frame_id != self.map_frame:
            self.publish_status(
                f"REJECTED wrong_frame frame_id={header.frame_id} expected={self.map_frame}")
            return

        if not self.confidence_is_acceptable():
            return

        pose_time = Time.from_msg(header.stamp)
        now = self.get_clock().now()
        is_zero_stamp = header.stamp.sec == 0 and header.stamp.nanosec == 0

        if is_zero_stamp and self.accept_zero_stamp:
            # 有些外部节点会发 stamp=0，表示“使用最新 TF”。
            # 这种情况下无法做严格时间同步，只能用当前最新 odom->localized_frame。
            lookup_time = Time()
            pose_age = 0.0
        else:
            lookup_time = pose_time
            pose_age = (now - pose_time).nanoseconds * 1e-9
            if pose_age < -0.2 or pose_age > self.pose_timeout_sec:
                self.publish_status(f"REJECTED stale_pose age={pose_age:.3f}s")
                return

        map_to_localized = pose_to_matrix(pose)
        odom_to_localized_matrix = self.lookup_odom_to_localized_matrix(lookup_time, is_zero_stamp)
        if odom_to_localized_matrix is None:
            if (
                not is_zero_stamp
                and self.last_odom_cache_failure == "future"
                and self.queue_pending_odom_candidate(lookup_time, map_to_localized)
            ):
                return
            return

        candidate = map_to_localized @ np.linalg.inv(odom_to_localized_matrix)
        candidate = self.apply_output_constraints(candidate)

        self.evaluate_candidate(candidate)

    def confidence_is_acceptable(self) -> bool:
        """
        判断外部定位结果的置信度是否足够。

        require_confidence=False 时不启用该门控。
        require_confidence=True 时，必须最近收到过 confidence，且数值超过阈值。
        """
        if not self.require_confidence:
            return True

        if self.latest_confidence is None or self.latest_confidence_time is None:
            self.publish_status("REJECTED no_confidence")
            return False

        age = (self.get_clock().now() - self.latest_confidence_time).nanoseconds * 1e-9
        if age > self.confidence_timeout_sec:
            self.publish_status(
                f"REJECTED stale_confidence age={age:.3f}s")
            return False

        if self.latest_confidence < self.min_confidence:
            self.publish_status(
                f"REJECTED low_confidence value={self.latest_confidence:.3f} min={self.min_confidence:.3f}")
            return False

        return True

    def apply_output_constraints(self, transform: np.ndarray) -> np.ndarray:
        """
        对输出 map->odom 做导航友好的约束。

        force_2d=True 时：
          - 只保留 x/y/yaw；
          - z 固定为 force_z；
          - roll/pitch 清零。

        这样可以避免外部 3D 定位把 Nav2 的平面 TF 拉出地面。
        """
        if not self.force_2d:
            return transform

        constrained = np.eye(4)
        yaw = yaw_from_matrix(transform)
        constrained[0, 0] = math.cos(yaw)
        constrained[0, 1] = -math.sin(yaw)
        constrained[1, 0] = math.sin(yaw)
        constrained[1, 1] = math.cos(yaw)
        constrained[0, 3] = transform[0, 3]
        constrained[1, 3] = transform[1, 3]
        constrained[2, 3] = self.force_z
        return constrained

    def evaluate_candidate(self, candidate: np.ndarray):
        """根据当前已接受 TF 和候选 TF 的差异，决定接受、等待确认或拒绝。"""
        if self.spin_to_pose_guard_is_active():
            self.pending_large_candidate = None
            self.pending_large_count = 0
            self.large_jump_hold_start_time = None
            self.publish_status(
                "REJECTED spin_to_pose_freeze_tf "
                f"phase={self.spin_guard_phase()}"
            )
            return

        if self.accepted_map_to_odom is None:
            if self.allow_initial_pose:
                self.accept_candidate(candidate, "initial_pose")
            else:
                self.publish_status("REJECTED no_initial_pose")
            return

        delta_xy, delta_yaw = correction_delta(candidate, self.accepted_map_to_odom)

        if self.jump_protection_mode == "protect":
            self.evaluate_candidate_protected(candidate, delta_xy, delta_yaw)
            return

        if self.jump_protection_mode == "monitor":
            self.publish_jump_protection_monitor(delta_xy, delta_yaw)

        self.evaluate_candidate_legacy(candidate, delta_xy, delta_yaw)

    def evaluate_candidate_legacy(self, candidate: np.ndarray, delta_xy: float, delta_yaw: float):
        """原始门控逻辑：小修正直接接受，大修正连续多帧一致后接受。"""
        if delta_xy <= self.max_small_correction_translation and delta_yaw <= self.max_small_correction_yaw:
            self.pending_large_candidate = None
            self.pending_large_count = 0
            self.large_jump_hold_start_time = None
            self.accept_candidate(candidate, f"small_correction dx={delta_xy:.3f} yaw={delta_yaw:.3f}")
            return

        if delta_xy > self.max_large_correction_translation or delta_yaw > self.max_large_correction_yaw:
            self.pending_large_candidate = None
            self.pending_large_count = 0
            self.large_jump_hold_start_time = None
            self.publish_status(f"REJECTED too_large dx={delta_xy:.3f} yaw={delta_yaw:.3f}")
            return

        # 走到这里说明候选是“可疑但可能正确”的大修正。
        # 只有连续多帧落在同一个位姿簇，才真正接受。
        if self.pending_large_candidate is None:
            self.pending_large_candidate = candidate
            self.pending_large_count = 1
            self.publish_status(f"PENDING large_correction count=1 dx={delta_xy:.3f} yaw={delta_yaw:.3f}")
            return

        consistent_xy, consistent_yaw = correction_delta(candidate, self.pending_large_candidate)
        if (
            consistent_xy <= self.consistency_translation_tolerance
            and consistent_yaw <= self.consistency_yaw_tolerance
        ):
            self.pending_large_count += 1
            self.pending_large_candidate = candidate
        else:
            self.pending_large_candidate = candidate
            self.pending_large_count = 1
            self.publish_status(
                "PENDING reset_large_candidate "
                f"spread_xy={consistent_xy:.3f} spread_yaw={consistent_yaw:.3f}"
            )
            return

        if self.pending_large_count >= self.required_consistent_frames:
            self.accept_candidate(candidate, f"confirmed_large_correction count={self.pending_large_count}")
            self.pending_large_candidate = None
            self.pending_large_count = 0
            self.large_jump_hold_start_time = None
        else:
            self.publish_status(
                f"PENDING large_correction count={self.pending_large_count}/"
                f"{self.required_consistent_frames}"
            )

    def evaluate_candidate_protected(self, candidate: np.ndarray, delta_xy: float, delta_yaw: float):
        """
        第一版保护逻辑。

        导航中：
          - 小修正直接接受；
          - 中等修正连续稳定后接受；
          - 大跳默认冻结，不更新 map->odom，只持续发布 last good TF。

        空闲/讲解中：
          - 允许 1m 内的大修正连续稳定后接受；
          - 超过 hard_reject 阈值不自动接受。
        """
        if delta_xy <= self.max_small_correction_translation and delta_yaw <= self.max_small_correction_yaw:
            self.pending_large_candidate = None
            self.pending_large_count = 0
            self.large_jump_hold_start_time = None
            self.accept_candidate(candidate, f"small_correction dx={delta_xy:.3f} yaw={delta_yaw:.3f}")
            return

        if delta_xy > self.max_large_correction_translation or delta_yaw > self.max_large_correction_yaw:
            self.pending_large_candidate = None
            self.pending_large_count = 0
            self.update_large_jump_hold(
                f"too_large dx={delta_xy:.3f} yaw={delta_yaw:.3f}"
            )
            return

        if self.navigation_active:
            if delta_xy <= self.nav_medium_correction_translation and delta_yaw <= self.nav_medium_correction_yaw:
                if self.update_pending_candidate(
                    candidate,
                    self.nav_medium_required_frames,
                    "nav_medium_correction",
                    delta_xy,
                    delta_yaw,
                ):
                    self.accept_candidate(
                        candidate,
                        f"confirmed_nav_medium_correction count={self.pending_large_count}",
                    )
                    self.pending_large_candidate = None
                    self.pending_large_count = 0
                    self.large_jump_hold_start_time = None
                return

            if self.allow_nav_large_jump:
                if self.update_pending_candidate(
                    candidate,
                    self.required_consistent_frames,
                    "nav_large_correction",
                    delta_xy,
                    delta_yaw,
                ):
                    self.accept_candidate(
                        candidate,
                        f"confirmed_nav_large_correction count={self.pending_large_count}",
                    )
                    self.pending_large_candidate = None
                    self.pending_large_count = 0
                    self.large_jump_hold_start_time = None
                return

            self.pending_large_candidate = None
            self.pending_large_count = 0
            self.update_large_jump_hold(
                f"nav_large_correction dx={delta_xy:.3f} yaw={delta_yaw:.3f}"
            )
            return

        if delta_xy > self.hard_reject_translation or delta_yaw > self.hard_reject_yaw:
            self.pending_large_candidate = None
            self.pending_large_count = 0
            self.update_large_jump_hold(
                f"idle_hard_reject dx={delta_xy:.3f} yaw={delta_yaw:.3f}"
            )
            return

        if (
            self.allow_idle_large_jump
            and delta_xy <= self.idle_large_correction_translation
            and delta_yaw <= self.idle_large_correction_yaw
        ):
            if self.update_pending_candidate(
                candidate,
                self.idle_large_required_frames,
                "idle_large_correction",
                delta_xy,
                delta_yaw,
            ):
                self.accept_candidate(
                    candidate,
                    f"confirmed_idle_large_correction count={self.pending_large_count}",
                )
                self.pending_large_candidate = None
                self.pending_large_count = 0
                self.large_jump_hold_start_time = None
            return

        self.pending_large_candidate = None
        self.pending_large_count = 0
        self.update_large_jump_hold(
            f"idle_large_not_allowed dx={delta_xy:.3f} yaw={delta_yaw:.3f}"
        )

    def update_pending_candidate(
        self,
        candidate: np.ndarray,
        required_frames: int,
        label: str,
        delta_xy: float,
        delta_yaw: float,
    ) -> bool:
        """更新连续一致候选簇；达到 required_frames 时返回 True。"""
        self.large_jump_hold_start_time = None
        required_frames = max(1, int(required_frames))

        if self.pending_large_candidate is None:
            self.pending_large_candidate = candidate
            self.pending_large_count = 1
            self.publish_status(
                f"PENDING {label} count=1/{required_frames} dx={delta_xy:.3f} yaw={delta_yaw:.3f}"
            )
            return self.pending_large_count >= required_frames

        consistent_xy, consistent_yaw = correction_delta(candidate, self.pending_large_candidate)
        if (
            consistent_xy <= self.consistency_translation_tolerance
            and consistent_yaw <= self.consistency_yaw_tolerance
        ):
            self.pending_large_count += 1
            self.pending_large_candidate = candidate
        else:
            self.pending_large_candidate = candidate
            self.pending_large_count = 1
            self.publish_status(
                f"PENDING reset_{label} spread_xy={consistent_xy:.3f} spread_yaw={consistent_yaw:.3f}"
            )
            return False

        if self.pending_large_count >= required_frames:
            return True

        self.publish_status(
            f"PENDING {label} count={self.pending_large_count}/{required_frames} "
            f"dx={delta_xy:.3f} yaw={delta_yaw:.3f}"
        )
        return False

    def update_large_jump_hold(self, reason: str):
        """
        冻结 map->odom 更新。

        注意这里不是停止发布 TF，而是不更新 accepted_map_to_odom；
        publish_last_tf 定时器仍会继续重发 last good map->odom，Nav2 的 TF 链不会断。
        """
        now = self.get_clock().now()
        if self.large_jump_hold_start_time is None:
            self.large_jump_hold_start_time = now
            self.large_jump_hold_reason = reason
            self.publish_status(f"HOLD large_jump {reason}")
            return

        hold_age = (now - self.large_jump_hold_start_time).nanoseconds * 1e-9
        if hold_age >= self.large_jump_degraded_after_sec:
            self.publish_status(
                f"DEGRADED large_jump_hold age={hold_age:.2f}s reason={self.large_jump_hold_reason}"
            )
        else:
            self.publish_status(
                f"HOLD large_jump age={hold_age:.2f}s reason={self.large_jump_hold_reason}"
            )

    def publish_jump_protection_monitor(self, delta_xy: float, delta_yaw: float):
        """monitor 模式只报告第一版策略会怎么处理，不改变原始 TF 接受逻辑。"""
        if delta_xy <= self.max_small_correction_translation and delta_yaw <= self.max_small_correction_yaw:
            return

        if self.navigation_active:
            if delta_xy <= self.nav_medium_correction_translation and delta_yaw <= self.nav_medium_correction_yaw:
                self.publish_status(f"WOULD_PENDING nav_medium_correction dx={delta_xy:.3f} yaw={delta_yaw:.3f}")
            else:
                self.publish_status(f"WOULD_HOLD nav_large_correction dx={delta_xy:.3f} yaw={delta_yaw:.3f}")
            return

        if delta_xy > self.hard_reject_translation or delta_yaw > self.hard_reject_yaw:
            self.publish_status(f"WOULD_HOLD idle_hard_reject dx={delta_xy:.3f} yaw={delta_yaw:.3f}")
        else:
            self.publish_status(f"WOULD_PENDING idle_large_correction dx={delta_xy:.3f} yaw={delta_yaw:.3f}")

    def spin_to_pose_guard_is_active(self) -> bool:
        """判断 SpinToPose 冻结窗口是否仍有效。"""
        if not self.enable_spin_to_pose_guard or not self.spin_guard_active:
            return False

        now = self.get_clock().now()
        if self.spin_guard_start_time is not None:
            age = (now - self.spin_guard_start_time).nanoseconds * 1e-9
            if age < 0.0 or age > self.spin_to_pose_guard_max_duration_sec:
                self.spin_guard_active = False
                self.spin_guard_turning = False
                self.spin_guard_source = ""
                self.publish_status(
                    f"SPIN_GUARD expired age={age:.2f}s"
                )
                return False

        if self.spin_guard_settle_until is not None and now > self.spin_guard_settle_until:
            self.spin_guard_active = False
            self.spin_guard_turning = False
            self.spin_guard_source = ""
            self.publish_status("SPIN_GUARD settled")
            return False

        return True

    def spin_guard_phase(self) -> str:
        """返回当前 SpinToPose 冻结阶段，便于日志和监控区分。"""
        if self.spin_guard_turning:
            return "turning"
        return "settle"

    def accept_candidate(self, candidate: np.ndarray, reason: str):
        """正式接受一个 map->odom 候选，后续定时发布该 TF。"""
        self.accepted_map_to_odom = candidate
        self.last_accept_time = self.get_clock().now()
        xy = math.hypot(float(candidate[0, 3]), float(candidate[1, 3]))
        yaw = yaw_from_matrix(candidate)
        self.publish_status(f"ACCEPTED {reason} map_odom_xy_norm={xy:.3f} yaw={yaw:.3f}")

    def publish_last_tf(self):
        """按固定频率重发最后一次接受的 map->odom，避免 TF 断档。"""
        if self.accepted_map_to_odom is None:
            self.publish_status("WAITING no_accepted_map_to_odom")
            return

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = float(self.accepted_map_to_odom[0, 3])
        msg.transform.translation.y = float(self.accepted_map_to_odom[1, 3])
        msg.transform.translation.z = float(self.accepted_map_to_odom[2, 3])
        qx, qy, qz, qw = matrix_to_quaternion(self.accepted_map_to_odom[:3, :3])
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(msg)

    def publish_status(self, text: str):
        """发布简单字符串状态，便于 ros2 topic echo 和日志排查。"""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        if text.startswith("REJECTED"):
            self.get_logger().warn(text, throttle_duration_sec=1.0)
        elif text.startswith("ACCEPTED small_correction"):
            self.get_logger().debug(text)
        elif text.startswith("ACCEPTED"):
            self.get_logger().info(text)
        else:
            self.get_logger().debug(text)


def main(args=None):
    rclpy.init(args=args)
    node = PriorMapOdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
