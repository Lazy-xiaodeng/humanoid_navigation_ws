#!/usr/bin/env python3
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json


R_FASTLIO_TO_ROS = np.array(
    [
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=float,
)


def normalize_quaternion(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def quaternion_to_matrix(q):
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


def matrix_to_quaternion(rotation):
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


def yaw_from_quaternion(q):
    x, y, z, w = normalize_quaternion(q.x, q.y, q.z, q.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ScanContextToInitialPose(Node):
    def __init__(self):
        super().__init__("scancontext_to_initialpose")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.best_pose_topic = self.declare_parameter(
            "best_pose_topic", "/scancontext_global_localization/best_pose"
        ).value
        self.localization_pose_topic = self.declare_parameter("localization_pose_topic", "/pcl_pose").value
        self.ndt_status_topic = self.declare_parameter(
            "ndt_status_topic", "/localization/ndt_status"
        ).value
        self.initialpose_topic = self.declare_parameter("initialpose_topic", "/initialpose").value
        self.recovery_status_topic = self.declare_parameter(
            "recovery_status_topic", "/localization/recovery_status"
        ).value
        self.recovery_request_topic = self.declare_parameter(
            "recovery_request_topic", "/localization/recovery_requests"
        ).value
        self.trigger_service = self.declare_parameter(
            "trigger_service", "/scancontext_global_localization/trigger"
        ).value
        self.global_trigger_service = self.declare_parameter(
            "global_trigger_service", "/scancontext_global_localization/trigger_global"
        ).value
        self.global_recovery_after_attempts = int(
            self.declare_parameter("global_recovery_after_attempts", 3).value
        )
        self.startup_trigger_period_sec = float(
            self.declare_parameter("startup_trigger_period_sec", 2.0).value
        )
        self.runtime_trigger_period_sec = float(
            self.declare_parameter("runtime_trigger_period_sec", 8.0).value
        )
        self.startup_duration_sec = float(self.declare_parameter("startup_duration_sec", 30.0).value)
        self.monitor_localization = bool(self.declare_parameter("monitor_localization", True).value)
        self.localization_pose_stale_sec = float(
            self.declare_parameter("localization_pose_stale_sec", 2.5).value
        )
        self.recovery_settle_sec = float(self.declare_parameter("recovery_settle_sec", 6.0).value)
        self.require_ndt_stable_status_for_recovery = bool(
            self.declare_parameter("require_ndt_stable_status_for_recovery", True).value
        )
        self.ndt_recovery_required_stable_status_count = int(
            self.declare_parameter("ndt_recovery_required_stable_status_count", 3).value
        )
        self.ndt_rejected_recovery_count = int(
            self.declare_parameter("ndt_rejected_recovery_count", 2).value
        )
        self.external_recovery_request_duration_sec = float(
            self.declare_parameter("external_recovery_request_duration_sec", 12.0).value
        )
        self.publish_repetitions = int(self.declare_parameter("publish_repetitions", 8).value)
        self.publish_period_sec = float(self.declare_parameter("publish_period_sec", 0.25).value)
        self.min_publish_interval_sec = float(
            self.declare_parameter("min_publish_interval_sec", 12.0).value
        )
        self.xy_covariance = float(self.declare_parameter("xy_covariance", 0.25).value)
        self.z_covariance = float(self.declare_parameter("z_covariance", 0.04).value)
        self.yaw_covariance = float(self.declare_parameter("yaw_covariance", 0.10).value)
        # ★ 融合模式: 运行期是否允许 /pcl_pose stale 自动触发 recovery
        #   True  = 原行为 (兼容 HDL pipeline)
        #   False = 运行期只响应 fusion 的 /localization/recovery_requests
        self.enable_runtime_auto_recovery = bool(
            self.declare_parameter("enable_runtime_auto_recovery", True).value
        )

        self.started = time.monotonic()
        self.last_trigger_time = 0.0
        self.last_publish_time = 0.0
        self.last_localization_pose_time = 0.0
        self.recovery_active_until = 0.0
        self.external_recovery_until = 0.0
        self.recovery_active = False
        self.recovery_waiting_for_ndt = False
        self.recovery_ndt_check_after = 0.0
        self.ndt_recovery_stable_status_count = 0
        self.recovery_count = 0
        self.relocalize_attempts = 0
        self.active_service_name = self.trigger_service
        self.pending_initialpose = None
        self.pending_count = 0

        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 10)
        self.recovery_status_pub = self.create_publisher(String, self.recovery_status_topic, 10)
        # ★ Phase 3: HDL fallback request publisher
        self.hdl_fallback_request_topic = self.declare_parameter(
            "hdl_fallback_request_topic", "/localization/hdl_fallback_request"
        ).value
        self.hdl_fallback_pub = self.create_publisher(String, self.hdl_fallback_request_topic, 10)
        self.hdl_fallback_triggered = False
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, self.best_pose_topic, self.best_pose_callback, 10
        )
        self.localization_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.localization_pose_topic,
            self.localization_pose_callback,
            10,
        )
        self.ndt_status_sub = self.create_subscription(
            String,
            self.ndt_status_topic,
            self.ndt_status_callback,
            10,
        )
        self.recovery_request_sub = self.create_subscription(
            String,
            self.recovery_request_topic,
            self.recovery_request_callback,
            10,
        )
        self.trigger_client = self.create_client(Trigger, self.trigger_service)
        self.global_trigger_client = self.create_client(Trigger, self.global_trigger_service)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.publish_timer = self.create_timer(self.publish_period_sec, self.publish_timer_callback)
        # ★ Phase 2: prior tracking
        self.recovery_prior = None
        self.get_logger().info(
            f"scancontext_to_initialpose started: {self.best_pose_topic} -> {self.initialpose_topic}"
        )

    def timer_callback(self):
        now = time.monotonic()
        startup = now - self.started <= self.startup_duration_sec
        localization_stale = (
            self.monitor_localization
            and self.last_localization_pose_time > 0.0
            and now - self.last_localization_pose_time > self.localization_pose_stale_sec
        )
        localization_missing = self.monitor_localization and self.last_localization_pose_time <= 0.0
        external_recovery = now < self.external_recovery_until

        # ★ 融合模式: 运行期只响应 fusion LOST 请求, /pcl_pose stale 不自行触发
        # ★ localization_missing (NDT从未发布过/pcl_pose) 和 localization_stale
        #    有本质区别:
        #    - localization_missing = 启动从未成功, 永远应允许 SC 重试
        #    - localization_stale   = 运行时 NDT 丢定位, 由 fusion 统一仲裁
        #    如果 localization_missing 也被 enable_runtime_auto_recovery=False
        #    抑制, 会导致死锁: SC bridge 等 fusion → fusion INITIALIZING 等 NDT
        #    → NDT 等 SC bridge 的 /initialpose
        if startup:
            recovery_needed = localization_missing or external_recovery
        else:
            if self.enable_runtime_auto_recovery:
                recovery_needed = (localization_stale or localization_missing
                                   or external_recovery)
            else:
                recovery_needed = localization_missing or external_recovery
        if self.recovery_waiting_for_ndt or self.pending_initialpose is not None:
            return
        if not recovery_needed and now >= self.recovery_active_until:
            return
        if recovery_needed and not self.recovery_active:
            self.start_recovery(
                "localization pose missing"
                if localization_missing
                else "localization pose stale"
                if localization_stale
                else "external localization recovery request"
            )
        period = self.startup_trigger_period_sec if startup else self.runtime_trigger_period_sec
        if period <= 0.0 or now - self.last_trigger_time < period:
            return
        # ★ 启动 vs 运行阶段区分:
        #    启动阶段: odom 在原点无参考价值, 直接用全局模式 (跳过 odom gate)
        #    运行阶段: conservative 优先 (odom gate 验证), 失败后渐进升级 global
        if startup:
            use_global = True
        else:
            use_global = (
                self.recovery_active
                and self.global_recovery_after_attempts >= 0
                and self.relocalize_attempts >= self.global_recovery_after_attempts
            )
        client = self.global_trigger_client if use_global else self.trigger_client
        service_name = self.global_trigger_service if use_global else self.trigger_service
        self.get_logger().info(
            f'[SC bridge] sending trigger #{self.relocalize_attempts + 1} via '
            f'{"GLOBAL" if use_global else "CONSERVATIVE"} ({service_name}), '
            f'startup={startup}')
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.0)
            return
        self.last_trigger_time = now
        self.relocalize_attempts += 1
        self.active_service_name = service_name
        # ★ Phase 3: SC 全局搜索耗尽 → 触发 HDL fallback
        # 启动阶段宽松 (SC 始终 global, 需更多尝试), 运行阶段收紧
        hdl_trigger_extra = 5 if startup else 2
        if (not self.hdl_fallback_triggered and use_global and
            self.relocalize_attempts > self.global_recovery_after_attempts + hdl_trigger_extra):
            self._trigger_hdl_fallback(
                f"SC global search exhausted ({self.relocalize_attempts} attempts, "
                f"no accepted candidate, startup={startup})")
        mode = "global ScanContext recovery" if use_global else "ScanContext relocalize"
        self.publish_recovery_status("localization_relocalize_requested", f"requested {mode}")
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self.trigger_done)

    def trigger_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"ScanContext trigger failed: {exc}")
            self.publish_recovery_status("localization_relocalize_failed", str(exc))
            return
        if not response.success:
            self.get_logger().warn(
                f'[SC bridge] trigger #{self.relocalize_attempts} FAILED: {response.message}')
            self.publish_recovery_status("localization_recovery_waiting", response.message)
        else:
            self.get_logger().info(
                f'[SC bridge] trigger #{self.relocalize_attempts} OK, waiting for best_pose...')

    def localization_pose_callback(self, _msg):
        self.last_localization_pose_time = time.monotonic()
        if (
            self.recovery_active
            and self.pending_initialpose is None
            and not self.require_ndt_stable_status_for_recovery
        ):
            self.publish_recovery_status(
                "localization_recovered",
                "NDT pose is updating after ScanContext initialpose",
            )
            self.complete_recovery()

    def ndt_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if not self.recovery_waiting_for_ndt:
            return

        now = time.monotonic()
        if now < self.recovery_ndt_check_after:
            return

        state = status.get("state", "")
        if state == "accepted":
            self.ndt_recovery_stable_status_count += 1
            if self.ndt_recovery_stable_status_count < max(
                1, self.ndt_recovery_required_stable_status_count
            ):
                self.publish_recovery_status(
                    "localization_recovery_waiting",
                    "waiting for stable NDT accepted status",
                )
                return
            self.get_logger().info(
                '[SC bridge] NDT accepted initialpose! localization recovered.')
            self.publish_recovery_status(
                "localization_recovered",
                "NDT accepted ScanContext initialpose for consecutive frames",
            )
            self.complete_recovery()
            return

        if state != "rejected":
            self.ndt_recovery_stable_status_count = 0
            return

        self.ndt_recovery_stable_status_count = 0
        try:
            rejected_count = int(status.get("consecutive_rejected_frames", 0))
        except (TypeError, ValueError):
            rejected_count = 0
        reason = status.get("reason", "NDT scan matching rejected")
        threshold = 1 if reason == "pose_jump" else max(1, self.ndt_rejected_recovery_count)
        if rejected_count < threshold:
            return

        self.recovery_waiting_for_ndt = False
        self.pending_initialpose = None
        self.pending_count = 0
        self.last_trigger_time = 0.0
        self.last_publish_time = 0.0
        self.get_logger().warn(
            f'[SC bridge] NDT rejected initialpose after {rejected_count} consecutive '
            f'rejected frames (reason={reason}), retrying recovery...')
        self.publish_recovery_status(
            "localization_relocalize_failed",
            f"NDT rejected ScanContext initialpose {rejected_count} consecutive frames: {reason}",
        )
        # ★ Phase 3: SC 全局搜索多次被 NDT 拒绝 → 触发 HDL fallback
        if (not self.hdl_fallback_triggered and
            self.recovery_count >= self.global_recovery_after_attempts):
            self._trigger_hdl_fallback(
                f"SC recovery #{self.recovery_count} failed NDT validation: {reason}")
        self.start_recovery(
            f"NDT rejected ScanContext initialpose {rejected_count} consecutive frames: {reason}"
        )
        self.relocalize_attempts = max(0, self.global_recovery_after_attempts)

    def recovery_request_callback(self, msg):
        self.external_recovery_until = time.monotonic() + self.external_recovery_request_duration_sec
        reason = "external localization recovery request"
        prior = None
        try:
            data = json.loads(msg.data)
            reason = data.get("reason", reason)
            # ★ Phase 2: 解析 prior 信息
            prior = data.get("prior", None)
            if prior is not None:
                self.get_logger().info(
                    f'[SC bridge] recovery_request 携带 prior: '
                    f'x={prior.get("x", "?")}, y={prior.get("y", "?")}, '
                    f'radius={prior.get("radius_m", "?")}m, source={prior.get("source", "?")}')
        except Exception:
            pass
        self.recovery_prior = prior
        if not self.recovery_active:
            self.start_recovery(reason)

    def best_pose_callback(self, msg):
        now = time.monotonic()
        if not self.recovery_active:
            self.get_logger().warn(
                "Ignoring ScanContext best_pose because recovery is not active; "
                "normal navigation must not reset NDT initialpose."
            )
            return
        if now - self.last_publish_time < self.min_publish_interval_sec:
            return
        converted = self.convert_pose(msg)
        self.pending_initialpose = converted
        self.pending_count = max(1, self.publish_repetitions)
        self.last_publish_time = now
        self.recovery_active_until = now + self.recovery_settle_sec
        self.publish_recovery_status(
            "localization_initialpose_published",
            "published ScanContext pose as NDT initial pose",
        )
        self.get_logger().info(
            "accepted ScanContext pose; publishing converted /initialpose "
            f"{self.pending_count} times: "
            f"x={converted.pose.pose.position.x:.3f}, "
            f"y={converted.pose.pose.position.y:.3f}, "
            f"yaw={math.degrees(yaw_from_quaternion(converted.pose.pose.orientation)):.1f}deg"
        )

    def publish_timer_callback(self):
        if self.pending_initialpose is None or self.pending_count <= 0:
            return
        msg = self.pending_initialpose
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.pending_count -= 1
        if self.pending_count <= 0:
            self.pending_initialpose = None
            if self.require_ndt_stable_status_for_recovery and self.recovery_active:
                now = time.monotonic()
                self.recovery_waiting_for_ndt = True
                self.recovery_ndt_check_after = now + self.recovery_settle_sec
                self.recovery_active_until = self.recovery_ndt_check_after
                self.ndt_recovery_stable_status_count = 0
                self.publish_recovery_status(
                    "localization_recovery_waiting",
                    f"waiting {self.recovery_settle_sec:.1f}s for NDT to validate ScanContext initialpose",
                )

    def start_recovery(self, reason):
        self.recovery_active = True
        self.recovery_waiting_for_ndt = False
        self.recovery_ndt_check_after = 0.0
        self.ndt_recovery_stable_status_count = 0
        self.recovery_count += 1
        self.relocalize_attempts = 0
        self.active_service_name = self.trigger_service
        self.hdl_fallback_triggered = False  # ★ Phase 3
        # ★ Phase 2: prior is already set by recovery_request_callback before start_recovery
        if self.recovery_prior is not None:
            self.get_logger().info(
                f'[SC bridge] recovery with prior: x={self.recovery_prior.get("x", "?")}, '
                f'y={self.recovery_prior.get("y", "?")}, '
                f'radius={self.recovery_prior.get("radius_m", "?")}m')
        self.get_logger().info(
            f'[SC bridge] recovery #{self.recovery_count} started: {reason}')
        self.publish_recovery_status("localization_recovery_started", reason)

    def complete_recovery(self):
        self.last_localization_pose_time = time.monotonic()
        self.recovery_active = False
        self.recovery_waiting_for_ndt = False
        self.recovery_active_until = 0.0
        self.external_recovery_until = 0.0
        self.pending_initialpose = None
        self.pending_count = 0
        self.ndt_recovery_stable_status_count = 0
        self.recovery_prior = None  # ★ Phase 2
        self.hdl_fallback_triggered = False  # ★ Phase 3

    def publish_recovery_status(self, event_type, reason):
        payload = {
            "event_type": event_type,
            "reason": reason,
            "timestamp": time.time(),
            "relocalization_mode": "scancontext_recovery",
            "recovery_count": self.recovery_count,
            "relocalize_attempts": self.relocalize_attempts,
            "active_service": self.active_service_name if self.recovery_active else None,
            "use_prior": self.recovery_prior is not None,
            "prior_source": self.recovery_prior.get("source", "") if self.recovery_prior else "",
            "source": "scancontext_to_initialpose",
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.recovery_status_pub.publish(msg)

    # ★ Phase 3: HDL fallback 触发
    def _trigger_hdl_fallback(self, reason: str):
        """SC 全局搜索用尽仍失败 → 触发 HDL 作为 fallback 引擎。

        HDL 执行 FPFH+RANSAC 几何搜索，结果通过 /initialpose 发布后
        由 NDT 多帧验收。HDL 不直接响应 /localization/recovery_requests，
        只由 SC bridge 显式触发。
        """
        if self.hdl_fallback_triggered:
            return
        self.hdl_fallback_triggered = True
        prior_info = {}
        if self.recovery_prior is not None:
            prior_info = {
                'prior_x': self.recovery_prior.get('x'),
                'prior_y': self.recovery_prior.get('y'),
                'prior_radius_m': self.recovery_prior.get('radius_m', 5.0),
                'prior_source': self.recovery_prior.get('source', ''),
            }
        payload = {
            'event_type': 'hdl_fallback_request',
            'reason': reason,
            'timestamp': time.time(),
            'recovery_count': self.recovery_count,
            'source': 'scancontext_to_initialpose',
            **prior_info,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.hdl_fallback_pub.publish(msg)
        self.get_logger().error(
            f'[SC bridge] ★ 触发 HDL fallback: {reason} '
            f'(recovery#{self.recovery_count}, prior={bool(self.recovery_prior)})')
        self.publish_recovery_status("hdl_fallback_triggered", reason)

    def convert_pose(self, msg):
        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame

        p = msg.pose.pose.position
        position_fastlio = np.array([p.x, p.y, p.z], dtype=float)
        position_ros = R_FASTLIO_TO_ROS @ position_fastlio
        out.pose.pose.position.x = float(position_ros[0])
        out.pose.pose.position.y = float(position_ros[1])
        out.pose.pose.position.z = 0.0

        rotation_fastlio = quaternion_to_matrix(msg.pose.pose.orientation)
        rotation_ros = R_FASTLIO_TO_ROS @ rotation_fastlio @ R_FASTLIO_TO_ROS.T
        qx, qy, qz, qw = matrix_to_quaternion(rotation_ros)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        out.pose.covariance[0] = self.xy_covariance
        out.pose.covariance[7] = self.xy_covariance
        out.pose.covariance[14] = self.z_covariance
        out.pose.covariance[35] = self.yaw_covariance
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ScanContextToInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
