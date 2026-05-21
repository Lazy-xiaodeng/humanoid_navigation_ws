import math
import time
import json
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hdl_localization.msg import ScanMatchingStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_quaternion(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def quaternion_to_matrix(x, y, z, w):
    x, y, z, w = normalize_quaternion(x, y, z, w)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])


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


def pose_to_matrix(pose):
    transform = np.eye(4)
    q = pose.orientation
    transform[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    transform[0, 3] = pose.position.x
    transform[1, 3] = pose.position.y
    transform[2, 3] = pose.position.z
    return transform


def transform_to_matrix(transform_stamped):
    transform = np.eye(4)
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    transform[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    transform[0, 3] = t.x
    transform[1, 3] = t.y
    transform[2, 3] = t.z
    return transform


def fill_pose_from_matrix(pose, transform):
    pose.position.x = float(transform[0, 3])
    pose.position.y = float(transform[1, 3])
    pose.position.z = float(transform[2, 3])
    qx, qy, qz, qw = matrix_to_quaternion(transform[:3, :3])
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw


def yaw_from_pose(pose):
    return yaw_from_quaternion(pose.orientation)


def yaw_from_quaternion(q):
    x, y, z, w = normalize_quaternion(q.x, q.y, q.z, q.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class HdlBootstrapToInitialPose(Node):
    def __init__(self):
        super().__init__('hdl_bootstrap_to_initialpose')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.hdl_odom_topic = self.declare_parameter('hdl_odom_topic', '/hdl_bootstrap/odom').value
        self.initialpose_topic = self.declare_parameter('initialpose_topic', '/initialpose').value
        self.relocalize_service = self.declare_parameter('relocalize_service', '/relocalize').value
        self.relocalize_with_prior_service = self.declare_parameter(
            'relocalize_with_prior_service', '/relocalize_with_prior').value
        self.hdl_standby_service = self.declare_parameter(
            'hdl_standby_service', '/hdl_bootstrap/standby').value
        self.external_relocalize_prior_topic = self.declare_parameter(
            'external_relocalize_prior_topic', '/hdl_relocalize_prior').value
        self.recovery_request_topic = self.declare_parameter(
            'recovery_request_topic', '/localization/recovery_requests').value
        self.localization_pose_topic = self.declare_parameter('localization_pose_topic', '/pcl_pose').value
        self.ndt_status_topic = self.declare_parameter(
            'ndt_status_topic', '/localization/ndt_status').value
        self.hdl_status_topic = self.declare_parameter('hdl_status_topic', '/status').value
        self.recovery_status_topic = self.declare_parameter(
            'recovery_status_topic', '/localization/recovery_status').value

        self.startup_delay_sec = float(self.declare_parameter('startup_delay_sec', 2.0).value)
        self.relocalize_retry_sec = float(self.declare_parameter('relocalize_retry_sec', 10.0).value)
        self.max_relocalize_attempts = int(self.declare_parameter('max_relocalize_attempts', 0).value)
        self.max_runtime_relocalize_attempts = int(
            self.declare_parameter('max_runtime_relocalize_attempts', 5).value)
        self.runtime_recovery_failure_cooldown_sec = float(
            self.declare_parameter('runtime_recovery_failure_cooldown_sec', 30.0).value)
        self.required_stable_samples = int(self.declare_parameter('required_stable_samples', 3).value)
        self.stable_xy_tolerance = float(self.declare_parameter('stable_xy_tolerance', 0.20).value)
        self.stable_yaw_tolerance = float(self.declare_parameter('stable_yaw_tolerance', 0.12).value)
        self.tf_lookup_timeout_sec = float(self.declare_parameter('tf_lookup_timeout_sec', 0.3).value)
        self.publish_repetitions = int(self.declare_parameter('publish_repetitions', 8).value)
        self.publish_period_sec = float(self.declare_parameter('publish_period_sec', 0.25).value)
        self.xy_covariance = float(self.declare_parameter('xy_covariance', 0.25).value)
        self.z_covariance = float(self.declare_parameter('z_covariance', 0.04).value)
        self.yaw_covariance = float(self.declare_parameter('yaw_covariance', 0.25).value)
        self.exit_after_publish = bool(self.declare_parameter('exit_after_publish', True).value)
        self.monitor_localization = bool(self.declare_parameter('monitor_localization', False).value)
        self.ndt_failure_triggers_recovery = bool(
            self.declare_parameter('ndt_failure_triggers_recovery', True).value)
        self.ndt_rejected_recovery_count = int(
            self.declare_parameter('ndt_rejected_recovery_count', 2).value)
        self.localization_pose_stale_sec = float(
            self.declare_parameter('localization_pose_stale_sec', 2.5).value)
        self.map_to_odom_tf_stale_sec = float(
            self.declare_parameter('map_to_odom_tf_stale_sec', 3.0).value)
        self.recovery_settle_sec = float(self.declare_parameter('recovery_settle_sec', 6.0).value)
        self.min_recovery_interval_sec = float(
            self.declare_parameter('min_recovery_interval_sec', 12.0).value)
        self.require_hdl_status = bool(self.declare_parameter('require_hdl_status', True).value)
        self.hdl_status_stale_sec = float(self.declare_parameter('hdl_status_stale_sec', 2.0).value)
        self.hdl_max_matching_error = float(self.declare_parameter('hdl_max_matching_error', 0.20).value)
        self.hdl_min_inlier_fraction = float(self.declare_parameter('hdl_min_inlier_fraction', 0.78).value)
        self.use_prior_relocalize_on_recovery = bool(
            self.declare_parameter('use_prior_relocalize_on_recovery', True).value)
        self.allow_full_global_recovery_without_prior = bool(
            self.declare_parameter('allow_full_global_recovery_without_prior', False).value)
        self.compare_with_hdl = bool(self.declare_parameter('compare_with_hdl', True).value)
        self.hdl_divergence_triggers_recovery = bool(
            self.declare_parameter('hdl_divergence_triggers_recovery', True).value)
        self.hdl_pose_stale_sec = float(self.declare_parameter('hdl_pose_stale_sec', 2.0).value)
        self.a_hdl_max_xy_delta = float(self.declare_parameter('a_hdl_max_xy_delta', 0.80).value)
        self.a_hdl_max_yaw_delta = float(self.declare_parameter('a_hdl_max_yaw_delta', 0.50).value)
        self.prior_requires_recent_agreement_sec = float(
            self.declare_parameter('prior_requires_recent_agreement_sec', 8.0).value)
        self.trusted_pose_max_age_sec = float(
            self.declare_parameter('trusted_pose_max_age_sec', 20.0).value)
        self.trusted_pose_update_min_interval_sec = float(
            self.declare_parameter('trusted_pose_update_min_interval_sec', 0.5).value)
        self.trusted_pose_requires_hdl_status = bool(
            self.declare_parameter('trusted_pose_requires_hdl_status', self.require_hdl_status).value)
        self.trusted_pose_log_interval_sec = float(
            self.declare_parameter('trusted_pose_log_interval_sec', 30.0).value)
        self.manual_initialpose_recovery_lockout_sec = float(
            self.declare_parameter('manual_initialpose_recovery_lockout_sec', 300.0).value)
        self.external_prior_ready_delay_sec = float(
            self.declare_parameter('external_prior_ready_delay_sec', 0.2).value)
        self.external_recovery_request_cooldown_sec = float(
            self.declare_parameter('external_recovery_request_cooldown_sec', 10.0).value)
        self.recovery_healthy_stable_count = int(
            self.declare_parameter('recovery_healthy_stable_count', 3).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.relocalize_client = self.create_client(Empty, self.relocalize_service)
        self.relocalize_with_prior_client = self.create_client(Empty, self.relocalize_with_prior_service)
        self.hdl_standby_client = self.create_client(Empty, self.hdl_standby_service)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 10)
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self.initialpose_callback,
            10,
        )
        prior_qos = QoSProfile(depth=1)
        prior_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.external_relocalize_prior_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.external_relocalize_prior_topic,
            prior_qos,
        )
        self.recovery_status_pub = self.create_publisher(String, self.recovery_status_topic, 10)
        self.hdl_odom_sub = self.create_subscription(
            Odometry,
            self.hdl_odom_topic,
            self.hdl_odom_callback,
            10,
        )
        self.localization_pose_sub = self.create_subscription(
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
        self.hdl_status_sub = self.create_subscription(
            ScanMatchingStatus,
            self.hdl_status_topic,
            self.hdl_status_callback,
            10,
        )
        self.recovery_request_sub = self.create_subscription(
            String,
            self.recovery_request_topic,
            self.recovery_request_callback,
            10,
        )

        self.start_time = time.monotonic()
        self.last_relocalize_request_time = 0.0
        self.relocalize_attempts = 0
        self.relocalize_future = None
        self.active_relocalize_service = None
        self.use_prior_for_next_relocalize = False
        self.accept_hdl_samples = False
        self.need_relocalize = True
        self.bootstrap_done = False
        self.samples = deque(maxlen=max(1, self.required_stable_samples))
        self.pending_initialpose = None
        self.pending_publish_count = 0
        self.last_publish_time = 0.0
        self.last_localization_pose_time = None
        self.last_ndt_status = None
        self.last_ndt_status_time = 0.0
        self.last_hdl_odom = None
        self.last_hdl_odom_time = None
        self.last_hdl_status = None
        self.last_hdl_status_time = None
        self.monitor_suppressed_until = 0.0
        self.recovery_ndt_check_after = 0.0
        self.last_recovery_start_time = 0.0
        self.last_a_hdl_agreement_time = 0.0
        self.last_trusted_map_to_base = None
        self.last_trusted_odom_to_base = None
        self.last_trusted_time = 0.0
        self.last_trusted_update_log_time = 0.0
        self.last_auto_initialpose_publish_time = 0.0
        self.last_manual_initialpose_time = 0.0
        self.pending_external_prior = None
        self.pending_external_prior_reason = ''
        self.last_external_prior_publish_time = 0.0
        self.last_external_recovery_request_time = 0.0
        self.last_hdl_standby_request_time = 0.0
        self.recovery_waiting_for_ndt = False
        self.recovery_healthy_count = 0
        self.recovery_count = 0

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info(
            f'waiting for HDL relocalization from {self.hdl_odom_topic}, then publishing {self.initialpose_topic}'
        )
        if self.monitor_localization:
            self.get_logger().info(
                f'runtime recovery enabled: monitor {self.localization_pose_topic} and '
                f'{self.map_frame}->{self.odom_frame}, NDT status {self.ndt_status_topic}, '
                f'recover through {self.relocalize_with_prior_service}'
            )

    def hdl_odom_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f'ignore HDL odom in frame {msg.header.frame_id}; expected {self.map_frame}',
                throttle_duration_sec=2.0,
            )
            return
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        self.last_hdl_odom = msg
        self.last_hdl_odom_time = stamp
        if self.accept_hdl_samples:
            self.samples.append(msg)

    def initialpose_callback(self, msg):
        now = time.monotonic()
        if now - self.last_auto_initialpose_publish_time < 0.75:
            return

        self.last_manual_initialpose_time = now
        self.need_relocalize = False
        self.bootstrap_done = True
        self.accept_hdl_samples = False
        self.samples.clear()
        self.pending_initialpose = None
        self.pending_publish_count = 0
        self.relocalize_future = None
        self.active_relocalize_service = None
        self.use_prior_for_next_relocalize = False
        self.last_hdl_status = None
        self.last_hdl_status_time = None
        self.last_external_prior_publish_time = 0.0
        self.recovery_waiting_for_ndt = True
        self.recovery_healthy_count = 0

        lockout = max(0.0, self.manual_initialpose_recovery_lockout_sec)
        self.monitor_suppressed_until = max(self.monitor_suppressed_until, now + lockout)
        self.recovery_ndt_check_after = now + max(0.0, self.recovery_settle_sec)
        self.last_recovery_start_time = now

        pose = msg.pose.pose.position
        reason = (
            f'manual initial pose received on {self.initialpose_topic}; '
            f'auto HDL recovery suppressed for {lockout:.1f}s'
        )
        self.get_logger().info(
            f'{reason}: frame={msg.header.frame_id or "unknown"} '
            f'pose=({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})'
        )
        self.publish_recovery_status(
            'localization_manual_initialpose_override',
            reason=reason,
            manual_lockout_sec=lockout,
            frame_id=msg.header.frame_id,
            pose_x=float(pose.x),
            pose_y=float(pose.y),
            pose_z=float(pose.z),
        )
        self.request_hdl_standby('manual /initialpose override')

    def recovery_request_callback(self, msg):
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'ignore malformed localization recovery request: {exc}')
            return

        now = time.monotonic()
        if (
            self.need_relocalize and
            now - self.last_external_recovery_request_time < self.external_recovery_request_cooldown_sec
        ):
            self.get_logger().warn(
                'ignore duplicate localization recovery request while relocalization is already pending',
                throttle_duration_sec=2.0,
            )
            return

        prior_ok, prior_reason = self.prepare_external_prior_from_request(request)
        if not prior_ok and not self.allow_full_global_recovery_without_prior:
            reason = request.get('reason', 'external localization recovery request')
            self.get_logger().error(
                f'localization recovery request blocked: no usable prior ({prior_reason}); reason={reason}'
            )
            self.publish_recovery_status(
                'localization_relocalize_failed',
                reason=f'{reason}; no usable prior ({prior_reason})',
                recovery_count=self.recovery_count,
                use_prior=False,
                prior_reason=prior_reason,
                request_source=request.get('source', ''),
            )
            return

        failure_reason = request.get('reason', 'external localization recovery request')
        self.last_external_recovery_request_time = now
        self.start_recovery(
            failure_reason,
            use_prior=prior_ok,
            prior_reason=prior_reason,
            request_source=request.get('source', ''),
            search_radius_m=request.get('search_radius_m', 0.0),
            trigger_event=request.get('event_type', ''),
        )

    def prepare_external_prior_from_request(self, request):
        self.pending_external_prior = None
        self.pending_external_prior_reason = ''

        pose_data = request.get('prior_pose')
        if not isinstance(pose_data, dict):
            return False, 'request has no prior_pose'

        position = pose_data.get('position') or {}
        orientation = pose_data.get('orientation') or {}
        try:
            x = float(position['x'])
            y = float(position['y'])
            z = float(position.get('z', 0.0))
            qx = float(orientation.get('x', 0.0))
            qy = float(orientation.get('y', 0.0))
            qz = float(orientation.get('z', 0.0))
            qw = float(orientation.get('w', 1.0))
        except (KeyError, TypeError, ValueError) as exc:
            return False, f'invalid prior_pose: {exc}'

        qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = (
            request.get('prior_frame_id')
            or request.get('frame_id')
            or self.map_frame
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = self.xy_covariance
        msg.pose.covariance[7] = self.xy_covariance
        msg.pose.covariance[14] = self.z_covariance
        msg.pose.covariance[35] = self.yaw_covariance

        source = request.get('prior_source', request.get('source', 'external request'))
        try:
            search_radius = float(request.get('search_radius_m', 0.0))
        except (TypeError, ValueError):
            search_radius = 0.0
        self.pending_external_prior = msg
        self.pending_external_prior_reason = (
            f'{source} prior: frame={msg.header.frame_id} '
            f'pose=({x:.3f}, {y:.3f}, yaw={yaw_from_pose(msg.pose.pose):.3f}), '
            f'preferred_search_radius={search_radius:.1f}m'
        )
        return True, self.pending_external_prior_reason

    def request_hdl_standby(self, reason):
        now = time.monotonic()
        if now - self.last_hdl_standby_request_time < 1.0:
            return

        if not self.hdl_standby_client.service_is_ready():
            self.hdl_standby_client.wait_for_service(timeout_sec=0.0)
            self.get_logger().info(
                f'waiting for HDL standby service {self.hdl_standby_service}',
                throttle_duration_sec=3.0,
            )
            return

        self.last_hdl_standby_request_time = now
        self.hdl_standby_client.call_async(Empty.Request())
        self.get_logger().info(f'requested HDL standby: {reason}')
        self.publish_recovery_status(
            'localization_hdl_standby_requested',
            reason=reason,
        )

    def localization_pose_callback(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        self.last_localization_pose_time = stamp

    def ndt_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'ignore malformed NDT localization status: {exc}')
            return

        self.last_ndt_status = status
        self.last_ndt_status_time = time.monotonic()

        if (
            not self.monitor_localization or
            not self.ndt_failure_triggers_recovery or
            not self.bootstrap_done
        ):
            return

        if (
            self.need_relocalize or
            self.accept_hdl_samples or
            self.pending_initialpose is not None or
            self.recovery_waiting_for_ndt
        ):
            return

        now_mono = time.monotonic()
        if now_mono < self.monitor_suppressed_until or now_mono < self.recovery_ndt_check_after:
            return

        if status.get('state', '') != 'rejected':
            return

        try:
            rejected_count = int(status.get('consecutive_rejected_frames', 0))
        except (TypeError, ValueError):
            rejected_count = 0

        reason = status.get('reason', 'NDT scan matching rejected')
        threshold = 1 if reason == 'pose_jump' else max(1, self.ndt_rejected_recovery_count)
        if rejected_count < threshold:
            return

        fitness = status.get('fitness_score', None)
        if isinstance(fitness, (int, float)):
            failure_reason = (
                f'NDT rejected {rejected_count} consecutive frames: {reason}, '
                f'fitness={float(fitness):.3f}'
            )
        else:
            failure_reason = f'NDT rejected {rejected_count} consecutive frames: {reason}'

        self.trigger_recovery_for_failure(
            failure_reason,
            source='ndt_status',
            ndt_status=status,
        )

    def hdl_status_callback(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        self.last_hdl_status = msg
        self.last_hdl_status_time = stamp

    def timer_callback(self):
        if self.publish_pending_initialpose():
            return
        if self.bootstrap_done:
            if self.recovery_waiting_for_ndt:
                self.check_recovery_completion()
            self.monitor_localization_health()
            return
        if time.monotonic() - self.start_time < self.startup_delay_sec:
            return

        if self.relocalize_future is not None:
            if not self.relocalize_future.done():
                return
            try:
                self.relocalize_future.result()
                service_name = self.active_relocalize_service or self.relocalize_service
                self.get_logger().info(f'HDL {service_name} service call completed')
                self.publish_recovery_status(
                    'localization_relocalize_completed',
                    service=service_name,
                )
                self.need_relocalize = False
                self.accept_hdl_samples = True
                self.samples.clear()
                self.last_hdl_status = None
                self.last_hdl_status_time = None
            except Exception as exc:
                service_name = self.active_relocalize_service or self.relocalize_service
                self.get_logger().warn(f'HDL {service_name} service call failed: {exc}')
                self.publish_recovery_status(
                    'localization_relocalize_failed',
                    reason=str(exc),
                    service=service_name,
                )
            self.relocalize_future = None
            self.active_relocalize_service = None

        if self.need_relocalize:
            self.request_relocalize_if_needed()
            return

        if self.samples_are_stable():
            if self.prepare_initialpose_from_latest_sample():
                self.bootstrap_done = True
                return

        self.request_relocalize_if_needed()

    def publish_pending_initialpose(self):
        if self.pending_initialpose is None or self.pending_publish_count <= 0:
            return False
        now = time.monotonic()
        if now - self.last_publish_time < self.publish_period_sec:
            return True
        self.pending_initialpose.header.stamp = self.get_clock().now().to_msg()
        self.last_auto_initialpose_publish_time = now
        self.initialpose_pub.publish(self.pending_initialpose)
        self.pending_publish_count -= 1
        self.last_publish_time = now
        if self.pending_publish_count == 0:
            self.get_logger().info('finished publishing bootstrap /initialpose')
            settle_until = time.monotonic() + max(0.0, self.recovery_settle_sec)
            self.monitor_suppressed_until = settle_until
            self.recovery_ndt_check_after = settle_until
            self.recovery_waiting_for_ndt = True
            self.recovery_healthy_count = 0
            self.request_hdl_standby('verified HDL pose has been handed off to NDT')
            self.publish_recovery_status(
                'localization_initialpose_published',
                reason='published verified HDL pose as NDT initial pose',
                ndt_settle_sec=max(0.0, self.recovery_settle_sec),
            )
            if self.exit_after_publish:
                self.get_logger().info('bootstrap initial pose published; exiting helper node')
                rclpy.try_shutdown()
        return True

    def request_relocalize_if_needed(self):
        now = time.monotonic()
        if now - self.last_relocalize_request_time < self.relocalize_retry_sec:
            return
        attempt_limit = self.max_relocalize_attempts
        if self.recovery_count > 0 and self.max_runtime_relocalize_attempts > 0:
            attempt_limit = self.max_runtime_relocalize_attempts
        if attempt_limit > 0 and self.relocalize_attempts >= attempt_limit:
            reason = (
                f'max HDL relocalization attempts reached: '
                f'{self.relocalize_attempts}/{attempt_limit}'
            )
            if self.recovery_count > 0:
                self.fail_current_recovery(reason)
            else:
                self.get_logger().error(f'{reason}; bootstrap failed')
            return

        client = self.relocalize_client
        service_name = self.relocalize_service
        if self.use_prior_for_next_relocalize and self.use_prior_relocalize_on_recovery:
            client = self.relocalize_with_prior_client
            service_name = self.relocalize_with_prior_service

        if service_name == self.relocalize_with_prior_service and self.pending_external_prior is not None:
            self.pending_external_prior.header.stamp = self.get_clock().now().to_msg()
            self.external_relocalize_prior_pub.publish(self.pending_external_prior)
            if self.last_external_prior_publish_time <= 0.0:
                self.last_external_prior_publish_time = now
                self.get_logger().info(
                    f'published external HDL recovery prior on {self.external_relocalize_prior_topic}: '
                    f'{self.pending_external_prior_reason}'
                )
                self.publish_recovery_status(
                    'localization_recovery_prior_published',
                    reason=self.pending_external_prior_reason,
                    service=service_name,
                    use_external_prior=True,
                )
                return
            if now - self.last_external_prior_publish_time < self.external_prior_ready_delay_sec:
                return

        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.0)
            self.get_logger().info(
                f'waiting for HDL relocalize service {service_name}',
                throttle_duration_sec=3.0,
            )
            return

        self.samples.clear()
        self.last_hdl_status = None
        self.last_hdl_status_time = None
        self.accept_hdl_samples = False
        self.relocalize_attempts += 1
        self.last_relocalize_request_time = now
        self.active_relocalize_service = service_name
        self.relocalize_future = client.call_async(Empty.Request())
        self.get_logger().info(f'calling HDL {service_name} attempt {self.relocalize_attempts}')
        self.publish_recovery_status(
            'localization_relocalize_requested',
            service=service_name,
            use_prior=service_name == self.relocalize_with_prior_service,
            use_external_prior=self.pending_external_prior is not None,
        )

    def fail_current_recovery(self, reason):
        self.need_relocalize = False
        self.bootstrap_done = True
        self.accept_hdl_samples = False
        self.samples.clear()
        self.pending_initialpose = None
        self.pending_publish_count = 0
        self.relocalize_future = None
        self.active_relocalize_service = None
        self.recovery_waiting_for_ndt = False
        self.recovery_healthy_count = 0
        self.recovery_ndt_check_after = 0.0
        self.monitor_suppressed_until = max(
            self.monitor_suppressed_until,
            time.monotonic() + max(0.0, self.runtime_recovery_failure_cooldown_sec),
        )
        self.get_logger().error(
            f'automatic HDL recovery failed: {reason}; '
            f'cooldown={self.runtime_recovery_failure_cooldown_sec:.1f}s'
        )
        self.request_hdl_standby('automatic HDL recovery failed')
        self.publish_recovery_status(
            'localization_relocalize_failed',
            reason=reason,
            relocalize_attempts=self.relocalize_attempts,
            max_attempts=self.max_runtime_relocalize_attempts,
            cooldown_sec=self.runtime_recovery_failure_cooldown_sec,
            use_prior=self.use_prior_for_next_relocalize,
        )

    def samples_are_stable(self):
        if len(self.samples) < self.required_stable_samples:
            return False
        if self.require_hdl_status and not self.hdl_status_is_good():
            return False
        first = self.samples[0].pose.pose
        first_yaw = yaw_from_pose(first)
        for sample in list(self.samples)[1:]:
            pose = sample.pose.pose
            dx = pose.position.x - first.position.x
            dy = pose.position.y - first.position.y
            if math.hypot(dx, dy) > self.stable_xy_tolerance:
                return False
            if abs(normalize_angle(yaw_from_pose(pose) - first_yaw)) > self.stable_yaw_tolerance:
                return False
        return True

    def hdl_status_is_good(self, log=True):
        if self.last_hdl_status is None or self.last_hdl_status_time is None:
            if log:
                self.get_logger().info('waiting for HDL scan matching status', throttle_duration_sec=3.0)
            return False

        status_age = (self.get_clock().now() - self.last_hdl_status_time).nanoseconds / 1e9
        if status_age < -0.5 or status_age > self.hdl_status_stale_sec:
            if log:
                self.get_logger().warn(
                    f'HDL status is stale: age={status_age:.2f}s > {self.hdl_status_stale_sec:.2f}s',
                    throttle_duration_sec=3.0,
                )
            return False

        if not self.last_hdl_status.has_converged:
            if log:
                self.get_logger().warn('HDL scan matching has not converged yet', throttle_duration_sec=3.0)
            return False

        if self.last_hdl_status.matching_error > self.hdl_max_matching_error:
            if log:
                self.get_logger().warn(
                    f'HDL matching error too high: {self.last_hdl_status.matching_error:.3f} '
                    f'> {self.hdl_max_matching_error:.3f}',
                    throttle_duration_sec=3.0,
                )
            return False

        if self.last_hdl_status.inlier_fraction < self.hdl_min_inlier_fraction:
            if log:
                self.get_logger().warn(
                    f'HDL inlier fraction too low: {self.last_hdl_status.inlier_fraction:.3f} '
                    f'< {self.hdl_min_inlier_fraction:.3f}',
                    throttle_duration_sec=3.0,
                )
            return False

        return True

    def monitor_localization_health(self):
        if not self.monitor_localization:
            return

        now_mono = time.monotonic()
        if now_mono < self.monitor_suppressed_until:
            return

        failure_reason = self.localization_failure_reason()
        if failure_reason is None:
            self.update_trusted_pose_if_healthy()
            return

        self.trigger_recovery_for_failure(failure_reason, source='health_monitor')

    def trigger_recovery_for_failure(self, failure_reason, source='health_monitor', **extra):
        now_mono = time.monotonic()
        if now_mono - self.last_recovery_start_time < self.min_recovery_interval_sec:
            self.get_logger().warn(
                f'localization unhealthy but recovery is cooling down: {failure_reason}',
                throttle_duration_sec=3.0,
            )
            return False

        use_prior, prior_reason = self.should_use_prior_for_recovery(failure_reason, now_mono)
        if not use_prior and not self.allow_full_global_recovery_without_prior:
            self.get_logger().error(
                f'localization unhealthy: {failure_reason}; automatic HDL recovery blocked because '
                f'no trusted prior is available ({prior_reason})'
            )
            self.publish_recovery_status(
                'localization_relocalize_failed',
                reason=(
                    f'{failure_reason}; blocked automatic full-global relocalization because '
                    f'no trusted prior is available ({prior_reason})'
                ),
                recovery_count=self.recovery_count,
                use_prior=False,
                prior_reason=prior_reason,
                source=source,
            )
            return False

        self.start_recovery(
            failure_reason,
            use_prior=use_prior,
            prior_reason=prior_reason,
            source=source,
            **extra,
        )
        return True

    def start_recovery(self, failure_reason, use_prior, prior_reason, **extra):
        now_mono = time.monotonic()
        self.recovery_count += 1
        self.last_recovery_start_time = now_mono
        self.monitor_suppressed_until = 0.0
        self.recovery_ndt_check_after = 0.0
        self.relocalize_attempts = 0
        self.last_relocalize_request_time = 0.0
        self.relocalize_future = None
        self.active_relocalize_service = None
        self.use_prior_for_next_relocalize = use_prior
        self.accept_hdl_samples = False
        self.need_relocalize = True
        self.samples.clear()
        self.last_hdl_status = None
        self.last_hdl_status_time = None
        self.last_external_prior_publish_time = 0.0
        self.pending_initialpose = None
        self.pending_publish_count = 0
        self.bootstrap_done = False
        self.recovery_waiting_for_ndt = False
        self.recovery_healthy_count = 0
        self.get_logger().warn(
            f'localization unhealthy: {failure_reason}; starting HDL recovery #{self.recovery_count}; '
            f'prior={self.use_prior_for_next_relocalize} ({prior_reason})'
        )
        status_extra = {
            'use_prior': self.use_prior_for_next_relocalize,
            'prior_reason': prior_reason,
        }
        status_extra.update(extra)
        self.publish_recovery_status('localization_recovery_started', reason=failure_reason, **status_extra)

    def check_recovery_completion(self):
        now_mono = time.monotonic()
        if now_mono < self.recovery_ndt_check_after:
            self.recovery_healthy_count = 0
            remaining = self.recovery_ndt_check_after - now_mono
            self.publish_recovery_status(
                'localization_recovery_waiting',
                reason=f'waiting {remaining:.1f}s for NDT to settle after /initialpose',
                throttle_key='settling',
                throttle_sec=1.0,
                ndt_settle_remaining_sec=remaining,
            )
            return

        failure_reason = self.localization_failure_reason()
        if failure_reason is not None:
            self.recovery_healthy_count = 0
            self.publish_recovery_status(
                'localization_recovery_waiting',
                reason=failure_reason,
                throttle_key='waiting',
                throttle_sec=2.0,
            )
            return

        self.recovery_healthy_count += 1
        if self.recovery_healthy_count < max(1, self.recovery_healthy_stable_count):
            return

        self.recovery_waiting_for_ndt = False
        self.recovery_healthy_count = 0
        self.recovery_ndt_check_after = 0.0
        self.publish_recovery_status(
            'localization_recovered',
            reason='NDT pose and map->odom TF are healthy after relocalization',
        )

    def update_trusted_pose_if_healthy(self):
        now_mono = time.monotonic()
        if now_mono - self.last_trusted_time < self.trusted_pose_update_min_interval_sec:
            return
        if self.trusted_pose_requires_hdl_status:
            if self.require_hdl_status and not self.hdl_status_is_good(log=False):
                return
            if self.last_hdl_odom is None or self.last_hdl_odom_time is None:
                return

            now_ros = self.get_clock().now()
            hdl_age = (now_ros - self.last_hdl_odom_time).nanoseconds / 1e9
            if hdl_age < -0.5 or hdl_age > self.hdl_pose_stale_sec:
                return

        try:
            map_to_base = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().debug(
                f'cannot update trusted localization pose yet: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        self.last_trusted_map_to_base = transform_to_matrix(map_to_base)
        self.last_trusted_odom_to_base = transform_to_matrix(odom_to_base)
        self.last_trusted_time = now_mono

        if now_mono - self.last_trusted_update_log_time > self.trusted_pose_log_interval_sec:
            self.last_trusted_update_log_time = now_mono
            pose = map_to_base.transform.translation
            self.get_logger().info(
                'updated trusted pose for recovery prior: '
                f'map->base=({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})'
            )

    def prepare_external_recovery_prior(self, now_mono):
        self.pending_external_prior = None
        self.pending_external_prior_reason = ''

        if self.last_trusted_map_to_base is None or self.last_trusted_odom_to_base is None:
            return False, 'no trusted pose has been recorded'

        trusted_age = now_mono - self.last_trusted_time
        if trusted_age > self.trusted_pose_max_age_sec:
            return (
                False,
                f'trusted pose is stale: {trusted_age:.2f}s > {self.trusted_pose_max_age_sec:.2f}s',
            )

        try:
            current_odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            return False, f'cannot compute odom delta for trusted prior: {exc}'

        current_odom_to_base_matrix = transform_to_matrix(current_odom_to_base)
        odom_delta = np.linalg.inv(self.last_trusted_odom_to_base) @ current_odom_to_base_matrix
        predicted_map_to_base = self.last_trusted_map_to_base @ odom_delta

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        fill_pose_from_matrix(msg.pose.pose, predicted_map_to_base)
        msg.pose.covariance[0] = self.xy_covariance
        msg.pose.covariance[7] = self.xy_covariance
        msg.pose.covariance[14] = self.z_covariance
        msg.pose.covariance[35] = self.yaw_covariance

        dx = float(odom_delta[0, 3])
        dy = float(odom_delta[1, 3])
        moved_xy = math.hypot(dx, dy)
        yaw_delta = abs(normalize_angle(math.atan2(odom_delta[1, 0], odom_delta[0, 0])))
        reason = (
            f'external trusted prior: trusted_age={trusted_age:.2f}s, '
            f'odom_delta_xy={moved_xy:.2f}m, odom_delta_yaw={yaw_delta:.2f}rad; '
            f'computed from TF {self.odom_frame}->{self.base_frame} so Fast-LIO raw axes are not used directly'
        )
        self.pending_external_prior = msg
        self.pending_external_prior_reason = reason
        return True, reason

    def should_use_prior_for_recovery(self, failure_reason, now_mono):
        self.pending_external_prior = None
        self.pending_external_prior_reason = ''
        if not self.use_prior_relocalize_on_recovery:
            return False, 'disabled'
        external_prior_ok, external_prior_reason = self.prepare_external_recovery_prior(now_mono)
        if external_prior_ok:
            return True, external_prior_reason
        if failure_reason.startswith('A/HDL pose divergence'):
            return False, 'A and HDL disagree; full global relocalization is safer'
        if self.last_a_hdl_agreement_time <= 0.0:
            return False, 'no recent A/HDL agreement'

        agreement_age = now_mono - self.last_a_hdl_agreement_time
        if agreement_age > self.prior_requires_recent_agreement_sec:
            return (
                False,
                f'last A/HDL agreement is stale: {agreement_age:.2f}s '
                f'> {self.prior_requires_recent_agreement_sec:.2f}s',
            )

        return True, f'recent A/HDL agreement {agreement_age:.2f}s ago'

    def localization_failure_reason(self):
        now_ros = self.get_clock().now()

        if self.last_localization_pose_time is None:
            return f'no accepted localization pose on {self.localization_pose_topic}'

        pose_age = (now_ros - self.last_localization_pose_time).nanoseconds / 1e9
        if pose_age < -0.5:
            return f'{self.localization_pose_topic} timestamp is in the future by {-pose_age:.2f}s'
        if pose_age > self.localization_pose_stale_sec:
            return (
                f'{self.localization_pose_topic} stale for {pose_age:.2f}s '
                f'> {self.localization_pose_stale_sec:.2f}s'
            )

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            return f'{self.map_frame}->{self.odom_frame} TF unavailable: {exc}'

        tf_stamp = Time.from_msg(transform.header.stamp)
        if tf_stamp.nanoseconds == 0:
            return f'{self.map_frame}->{self.odom_frame} TF has zero timestamp'

        tf_age = (now_ros - tf_stamp).nanoseconds / 1e9
        if tf_age < -0.5:
            return f'{self.map_frame}->{self.odom_frame} TF timestamp is in the future by {-tf_age:.2f}s'
        if tf_age > self.map_to_odom_tf_stale_sec:
            return (
                f'{self.map_frame}->{self.odom_frame} TF stale for {tf_age:.2f}s '
                f'> {self.map_to_odom_tf_stale_sec:.2f}s'
            )

        divergence_reason = self.a_hdl_divergence_reason()
        if divergence_reason is not None:
            return divergence_reason

        return None

    def a_hdl_divergence_reason(self):
        if not self.compare_with_hdl:
            return None
        if self.last_hdl_odom is None or self.last_hdl_odom_time is None:
            return None
        if not self.hdl_status_is_good(log=False):
            return None

        now_ros = self.get_clock().now()
        hdl_age = (now_ros - self.last_hdl_odom_time).nanoseconds / 1e9
        if hdl_age < -0.5 or hdl_age > self.hdl_pose_stale_sec:
            return None

        try:
            a_map_to_base = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException:
            return None

        a_t = a_map_to_base.transform.translation
        h_pose = self.last_hdl_odom.pose.pose
        dx = float(a_t.x - h_pose.position.x)
        dy = float(a_t.y - h_pose.position.y)
        xy_delta = math.hypot(dx, dy)
        yaw_delta = abs(normalize_angle(
            yaw_from_quaternion(a_map_to_base.transform.rotation) - yaw_from_pose(h_pose)))

        if xy_delta > self.a_hdl_max_xy_delta or yaw_delta > self.a_hdl_max_yaw_delta:
            reason = (
                f'A/HDL pose divergence: xy={xy_delta:.2f}m '
                f'(limit {self.a_hdl_max_xy_delta:.2f}), yaw={yaw_delta:.2f}rad '
                f'(limit {self.a_hdl_max_yaw_delta:.2f})'
            )
            if not self.hdl_divergence_triggers_recovery:
                self.get_logger().warn(
                    f'{reason}; treated as HDL diagnostic only, not triggering NDT recovery',
                    throttle_duration_sec=3.0,
                )
                return None
            return reason

        self.last_a_hdl_agreement_time = time.monotonic()
        return None

    def prepare_initialpose_from_latest_sample(self):
        latest = self.samples[-1]
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except TransformException as exc:
            self.get_logger().warn(f'cannot compute map->odom from HDL odom yet: {exc}', throttle_duration_sec=2.0)
            return False

        map_to_base = pose_to_matrix(latest.pose.pose)
        odom_to_base_matrix = transform_to_matrix(odom_to_base)
        map_to_odom = map_to_base @ np.linalg.inv(odom_to_base_matrix)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(map_to_odom[0, 3])
        msg.pose.pose.position.y = float(map_to_odom[1, 3])
        msg.pose.pose.position.z = float(map_to_odom[2, 3])
        qx, qy, qz, qw = matrix_to_quaternion(map_to_odom[:3, :3])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = self.xy_covariance
        msg.pose.covariance[7] = self.xy_covariance
        msg.pose.covariance[14] = self.z_covariance
        msg.pose.covariance[35] = self.yaw_covariance

        self.pending_initialpose = msg
        self.pending_publish_count = max(1, self.publish_repetitions)
        self.last_publish_time = 0.0
        self.use_prior_for_next_relocalize = False
        self.get_logger().info(
            'prepared A initial pose from HDL bootstrap/recovery: '
            f'map->odom=({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}, '
            f'yaw={yaw_from_pose(msg.pose.pose):.3f})'
        )
        return True

    def publish_recovery_status(self, event_type, reason='', throttle_key=None, throttle_sec=0.0, **extra):
        now = time.monotonic()
        if throttle_key:
            attr = f'_last_status_{throttle_key}'
            last_time = getattr(self, attr, 0.0)
            if now - last_time < throttle_sec:
                return
            setattr(self, attr, now)

        payload = {
            'event_type': event_type,
            'reason': reason,
            'timestamp': time.time(),
            'recovery_count': self.recovery_count,
            'relocalize_attempts': self.relocalize_attempts,
            'active_service': self.active_relocalize_service,
            'use_prior': self.use_prior_for_next_relocalize,
            'waiting_for_ndt': self.recovery_waiting_for_ndt,
            'healthy_count': self.recovery_healthy_count,
        }
        payload.update(extra)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.recovery_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HdlBootstrapToInitialPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
