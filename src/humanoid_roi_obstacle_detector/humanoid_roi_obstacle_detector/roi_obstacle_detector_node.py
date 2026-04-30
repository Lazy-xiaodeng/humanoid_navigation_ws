#!/usr/bin/env python3
"""Direct LiDAR front-ROI obstacle detector for humanoid robots.

The node avoids depending on odometry. In the recommended `transform_mode=imu`
mode it levels the raw LiDAR cloud with IMU roll/pitch, applies a fixed
LiDAR-to-base transform, normalizes vertical body bounce with a point-cloud
ground-z estimate, and finally runs a simple front ROI occupancy test.
"""

import json
import math
import time
from collections import deque
from typing import Iterable, Optional, Tuple

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool, Header, String
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float32)


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float32)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float32)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    return rz @ ry @ rx


def quaternion_to_roll_pitch(x: float, y: float, z: float, w: float) -> Tuple[float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch


def transform_points(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return points @ rotation.T + translation


def normalize_vector(vector: np.ndarray) -> Optional[np.ndarray]:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-6:
        return None
    return (vector / norm).astype(np.float32)


def rotation_between_vectors(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source_norm = normalize_vector(source)
    target_norm = normalize_vector(target)
    if source_norm is None or target_norm is None:
        return np.eye(3, dtype=np.float32)

    dot = float(np.clip(np.dot(source_norm, target_norm), -1.0, 1.0))
    if dot > 0.9999:
        return np.eye(3, dtype=np.float32)

    if dot < -0.9999:
        axis = np.cross(source_norm, np.array([1.0, 0.0, 0.0], dtype=np.float32))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(source_norm, np.array([0.0, 1.0, 0.0], dtype=np.float32))
        axis = normalize_vector(axis)
        return axis_angle_to_matrix(axis, math.pi)

    axis = normalize_vector(np.cross(source_norm, target_norm))
    angle = math.acos(dot)
    return axis_angle_to_matrix(axis, angle)


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis.tolist()
    c = math.cos(angle)
    s = math.sin(angle)
    one_c = 1.0 - c
    return np.array([
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ], dtype=np.float32)


class RoiObstacleDetector(Node):
    """ROS 2 node that publishes a debounced front obstacle boolean.

    The processing pipeline is intentionally small and deterministic:

    1. Read raw xyz from `sensor_msgs/msg/PointCloud2`.
    2. Transform into the configured target frame.
    3. Optionally shift z by the estimated ground height.
    4. Crop the front ROI.
    5. Voxelize, cluster, and apply trigger/clear hysteresis.
    """

    def __init__(self) -> None:
        super().__init__('roi_obstacle_detector')

        self.declare_parameter('input_topic', '/airy_points')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('source_frame_override', '')
        self.declare_parameter('transform_mode', 'tf')
        self.declare_parameter('fallback_to_manual', False)
        self.declare_parameter('tf_timeout_sec', 0.05)
        self.declare_parameter('manual_translation_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('manual_rotation_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('manual_rotation_matrix', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('enable_imu_leveling', False)
        self.declare_parameter('imu_topic', '/airy_imu')
        self.declare_parameter('imu_mode', 'accel')
        self.declare_parameter('imu_filter_mode', 'lowpass')
        self.declare_parameter('source_to_imu_rotation_matrix', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('imu_timeout_sec', 0.3)
        self.declare_parameter('imu_accel_alpha', 0.05)
        self.declare_parameter('imu_accel_is_up', True)
        self.declare_parameter('imu_expected_accel_norm', 9.80665)
        self.declare_parameter('imu_accel_gate_sigma', 1.5)
        self.declare_parameter('imu_max_gyro_dt_sec', 0.05)
        self.declare_parameter('imu_gyro_bias_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('estimate_gyro_bias', False)
        self.declare_parameter('gyro_bias_estimation_sec', 2.0)
        self.declare_parameter('gyro_bias_min_samples', 100)
        self.declare_parameter('level_up_axis', [0.0, 0.0, 1.0])
        self.declare_parameter('level_reference_axis_source', Parameter.Type.DOUBLE_ARRAY)

        self.declare_parameter('enable_ground_z_compensation', True)
        self.declare_parameter('ground_roi_min_x', 0.25)
        self.declare_parameter('ground_roi_max_x', 3.0)
        self.declare_parameter('ground_roi_min_y', -1.2)
        self.declare_parameter('ground_roi_max_y', 1.2)
        self.declare_parameter('ground_roi_min_z', -0.45)
        self.declare_parameter('ground_roi_max_z', 0.35)
        self.declare_parameter('ground_z_percentile', 35.0)
        self.declare_parameter('ground_min_points', 200)
        self.declare_parameter('ground_filter_alpha', 0.35)
        self.declare_parameter('ground_max_step', 0.08)
        self.declare_parameter('ground_z_offset', 0.0)

        self.declare_parameter('roi_min_x', 0.25)
        self.declare_parameter('roi_max_x', 2.0)
        self.declare_parameter('roi_min_y', -0.45)
        self.declare_parameter('roi_max_y', 0.45)
        self.declare_parameter('roi_min_z', 0.05)
        self.declare_parameter('roi_max_z', 1.2)

        self.declare_parameter('voxel_leaf_size', 0.05)
        self.declare_parameter('min_points', 10)
        self.declare_parameter('use_clustering', True)
        self.declare_parameter('cluster_min_size', 10)
        self.declare_parameter('cluster_connectivity', 26)
        self.declare_parameter('trigger_frames', 2)
        self.declare_parameter('clear_frames', 4)

        self.declare_parameter('has_obstacle_topic', '/front_obstacle/has_obstacle')
        self.declare_parameter('debug_topic', '/front_obstacle/debug')
        self.declare_parameter('roi_cloud_topic', '/front_obstacle/roi_cloud')
        self.declare_parameter('publish_roi_cloud', True)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_publish_period_sec', 0.5)
        self.declare_parameter('log_input_stats', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame_override = self.get_parameter('source_frame_override').value
        self.transform_mode = self.get_parameter('transform_mode').value.lower()
        self.fallback_to_manual = bool(self.get_parameter('fallback_to_manual').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.manual_translation = np.asarray(
            self.get_parameter('manual_translation_xyz').value, dtype=np.float32)
        self.manual_rotation = self.load_manual_rotation()
        self.enable_imu_leveling = bool(self.get_parameter('enable_imu_leveling').value)
        self.imu_mode = self.get_parameter('imu_mode').value.lower()
        self.imu_filter_mode = self.get_parameter('imu_filter_mode').value.lower()
        self.source_to_imu_rotation = self.load_source_to_imu_rotation()
        self.imu_to_source_rotation = self.source_to_imu_rotation.T
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)
        self.imu_accel_alpha = float(self.get_parameter('imu_accel_alpha').value)
        self.imu_accel_is_up = bool(self.get_parameter('imu_accel_is_up').value)
        self.imu_expected_accel_norm = float(self.get_parameter('imu_expected_accel_norm').value)
        self.imu_accel_gate_sigma = float(self.get_parameter('imu_accel_gate_sigma').value)
        self.imu_max_gyro_dt_sec = float(self.get_parameter('imu_max_gyro_dt_sec').value)
        self.configured_gyro_bias = np.asarray(
            self.get_parameter('imu_gyro_bias_xyz').value, dtype=np.float32)
        self.estimate_gyro_bias = bool(self.get_parameter('estimate_gyro_bias').value)
        self.gyro_bias_estimation_sec = float(self.get_parameter('gyro_bias_estimation_sec').value)
        self.gyro_bias_min_samples = int(self.get_parameter('gyro_bias_min_samples').value)
        self.level_up_axis = np.asarray(self.get_parameter('level_up_axis').value, dtype=np.float32)
        normalized_up = normalize_vector(self.level_up_axis)
        self.level_up_axis = normalized_up if normalized_up is not None else np.array([0.0, 0.0, 1.0], dtype=np.float32)
        self.level_reference_axis_source = self.load_level_reference_axis_source()

        self.enable_ground_z_compensation = bool(self.get_parameter('enable_ground_z_compensation').value)
        self.ground_roi = {
            'min_x': float(self.get_parameter('ground_roi_min_x').value),
            'max_x': float(self.get_parameter('ground_roi_max_x').value),
            'min_y': float(self.get_parameter('ground_roi_min_y').value),
            'max_y': float(self.get_parameter('ground_roi_max_y').value),
            'min_z': float(self.get_parameter('ground_roi_min_z').value),
            'max_z': float(self.get_parameter('ground_roi_max_z').value),
        }
        self.ground_z_percentile = float(self.get_parameter('ground_z_percentile').value)
        self.ground_min_points = int(self.get_parameter('ground_min_points').value)
        self.ground_filter_alpha = float(self.get_parameter('ground_filter_alpha').value)
        self.ground_max_step = float(self.get_parameter('ground_max_step').value)
        self.ground_z_offset = float(self.get_parameter('ground_z_offset').value)

        self.roi = {
            'min_x': float(self.get_parameter('roi_min_x').value),
            'max_x': float(self.get_parameter('roi_max_x').value),
            'min_y': float(self.get_parameter('roi_min_y').value),
            'max_y': float(self.get_parameter('roi_max_y').value),
            'min_z': float(self.get_parameter('roi_min_z').value),
            'max_z': float(self.get_parameter('roi_max_z').value),
        }
        self.voxel_leaf_size = float(self.get_parameter('voxel_leaf_size').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.use_clustering = bool(self.get_parameter('use_clustering').value)
        self.cluster_min_size = int(self.get_parameter('cluster_min_size').value)
        self.cluster_connectivity = int(self.get_parameter('cluster_connectivity').value)
        self.trigger_frames = max(1, int(self.get_parameter('trigger_frames').value))
        self.clear_frames = max(1, int(self.get_parameter('clear_frames').value))
        self.publish_roi_cloud = bool(self.get_parameter('publish_roi_cloud').value)
        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.debug_publish_period_sec = float(self.get_parameter('debug_publish_period_sec').value)
        self.log_input_stats = bool(self.get_parameter('log_input_stats').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.filtered_accel_up: Optional[np.ndarray] = None
        self.last_imu_stamp_sec: Optional[float] = None
        self.gyro_bias = self.configured_gyro_bias.copy()
        self.gyro_bias_start_sec: Optional[float] = None
        self.gyro_bias_samples = []
        self.gyro_bias_ready = not self.estimate_gyro_bias
        self.imu_samples = deque(maxlen=400)
        self.filtered_ground_z: Optional[float] = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, sensor_qos)

        if self.enable_imu_leveling or self.transform_mode == 'imu':
            imu_topic = self.get_parameter('imu_topic').value
            self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, sensor_qos)
        else:
            self.imu_sub = None

        self.has_obstacle_pub = self.create_publisher(
            Bool, self.get_parameter('has_obstacle_topic').value, 10)
        self.debug_pub = self.create_publisher(
            String, self.get_parameter('debug_topic').value, 10)
        self.roi_cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter('roi_cloud_topic').value, 10)

        self.obstacle_state = False
        self.hit_streak = 0
        self.clear_streak = 0
        self.last_debug_publish = 0.0
        self.last_input_stats_log = 0.0

        self.get_logger().info(
            'ROI obstacle detector started: '
            f'input={self.input_topic}, target_frame={self.target_frame}, '
            f'transform_mode={self.transform_mode}, imu_mode={self.imu_mode}, '
            f'imu_filter_mode={self.imu_filter_mode}, '
            f'ground_z_compensation={self.enable_ground_z_compensation}, roi={self.roi}')
        if self.estimate_gyro_bias and self.imu_filter_mode == 'complementary':
            self.get_logger().info(
                'Estimating IMU gyro bias from startup samples: '
                f'duration={self.gyro_bias_estimation_sec:.2f}s, '
                f'min_samples={self.gyro_bias_min_samples}')

    def load_manual_rotation(self) -> np.ndarray:
        matrix_param = self.get_optional_double_array_parameter('manual_rotation_matrix')
        if len(matrix_param) == 9:
            matrix = np.asarray(matrix_param, dtype=np.float32).reshape((3, 3))
            self.warn_if_unclean_rotation(matrix, 'manual_rotation_matrix')
            return matrix

        return rpy_to_matrix(*[
            float(v) for v in self.get_parameter('manual_rotation_rpy').value
        ])

    def load_source_to_imu_rotation(self) -> np.ndarray:
        matrix_param = self.get_optional_double_array_parameter('source_to_imu_rotation_matrix')
        if len(matrix_param) == 9:
            matrix = np.asarray(matrix_param, dtype=np.float32).reshape((3, 3))
            self.warn_if_unclean_rotation(matrix, 'source_to_imu_rotation_matrix')
            return matrix
        return np.eye(3, dtype=np.float32)

    def load_level_reference_axis_source(self) -> np.ndarray:
        axis_param = self.get_optional_double_array_parameter('level_reference_axis_source')
        if len(axis_param) == 3:
            axis = normalize_vector(np.asarray(axis_param, dtype=np.float32))
            if axis is not None:
                return axis
            self.get_logger().warn(
                'level_reference_axis_source is invalid; using manual_rotation derived up axis.')

        axis = normalize_vector(self.manual_rotation.T @ self.level_up_axis)
        if axis is None:
            return np.array([0.0, 0.0, 1.0], dtype=np.float32)
        return axis

    def get_optional_double_array_parameter(self, name: str) -> list:
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            return []
        return value or []

    def warn_if_unclean_rotation(self, matrix: np.ndarray, name: str) -> None:
        det = float(np.linalg.det(matrix))
        orth_err = float(np.linalg.norm(matrix.T @ matrix - np.eye(3, dtype=np.float32)))
        if abs(det - 1.0) > 0.02 or orth_err > 0.02:
            self.get_logger().warn(
                f'{name} is not a clean rotation '
                f'(det={det:.6f}, orth_err={orth_err:.6f}).')

    def imu_callback(self, msg: Imu) -> None:
        """Store the latest IMU-derived leveling rotation.

        The recorded Airy IMU orientation field is constant in the available
        bags, so the default path estimates the local up vector from
        acceleration and uses gyro integration only inside the complementary
        filter.
        """
        level_rotation = self.compute_level_rotation_from_imu(msg)
        if level_rotation is None:
            return

        stamp_sec = self.stamp_to_sec(msg.header.stamp)
        self.imu_samples.append((stamp_sec, level_rotation))

    def cloud_callback(self, msg: PointCloud2) -> None:
        """Run one full obstacle-detection pass for an incoming cloud."""
        start = time.time()
        source_frame = self.source_frame_override or msg.header.frame_id
        if not source_frame:
            self.get_logger().warn('PointCloud2 header.frame_id is empty; skipping.', throttle_duration_sec=2.0)
            return

        points = self.read_xyz(msg)
        if points.size == 0:
            self.publish_state(False)
            return

        input_bounds = self.compute_bounds(points)
        self.log_input_stats_if_needed(source_frame, len(points), input_bounds)

        transform = self.get_source_to_target_transform(msg, source_frame)
        if transform is None:
            return

        rotation, translation, output_frame = transform
        points_target = transform_points(points, rotation, translation)
        pre_ground_bounds = self.compute_bounds(points_target)
        points_target, ground_z, ground_candidate_count = self.apply_ground_z_compensation(points_target)
        target_bounds = self.compute_bounds(points_target)

        roi_points = self.crop_roi(points_target)
        raw_roi_count = int(len(roi_points))
        roi_points = self.voxel_downsample(roi_points, self.voxel_leaf_size)
        voxel_count = int(len(roi_points))

        max_cluster_size = 0
        if voxel_count >= self.min_points:
            if self.use_clustering:
                max_cluster_size = self.find_max_cluster_size(roi_points, self.voxel_leaf_size)
                detected_now = max_cluster_size >= self.cluster_min_size
            else:
                detected_now = True
        else:
            detected_now = False

        self.update_hysteresis(detected_now)
        self.publish_state(self.obstacle_state)

        if self.publish_roi_cloud:
            self.publish_cloud(roi_points, msg.header.stamp, output_frame)

        elapsed_ms = (time.time() - start) * 1000.0
        self.publish_debug_message(
            input_count=len(points),
            raw_roi_count=raw_roi_count,
            voxel_count=voxel_count,
            max_cluster_size=max_cluster_size,
            detected_now=detected_now,
            output_frame=output_frame,
            source_frame=source_frame,
            input_bounds=input_bounds,
            pre_ground_bounds=pre_ground_bounds,
            target_bounds=target_bounds,
            ground_z=ground_z,
            ground_candidate_count=ground_candidate_count,
            elapsed_ms=elapsed_ms,
        )

    def read_xyz(self, msg: PointCloud2) -> np.ndarray:
        try:
            points = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.asarray(points, dtype=np.float32)
            if points.ndim == 1:
                points = points.reshape((-1, 3))
            return points[:, :3]
        except Exception as exc:
            self.get_logger().warn(f'Failed to read PointCloud2 xyz fields: {exc}', throttle_duration_sec=2.0)
            return np.empty((0, 3), dtype=np.float32)

    def get_source_to_target_transform(
        self,
        msg: PointCloud2,
        source_frame: str,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, str]]:
        """Return the rotation/translation used for this cloud.

        `imu` mode composes the fixed level-source-to-base transform with the
        latest roll/pitch leveling rotation. It deliberately does not consume
        odometry or Fast-LIO pose.
        """
        if self.transform_mode == 'none' or source_frame == self.target_frame:
            return np.eye(3, dtype=np.float32), np.zeros(3, dtype=np.float32), source_frame

        if self.transform_mode == 'manual':
            return self.manual_rotation, self.manual_translation, self.target_frame

        if self.transform_mode == 'imu':
            level_rotation = self.lookup_level_rotation(self.stamp_to_sec(msg.header.stamp))
            if level_rotation is None:
                self.get_logger().warn(
                    'No recent IMU leveling sample; skipping cloud. '
                    'Check /airy_imu or set transform_mode:=manual.',
                    throttle_duration_sec=2.0)
                return None

            combined_rotation = self.manual_rotation @ level_rotation
            return combined_rotation, self.manual_translation, self.target_frame

        if self.transform_mode != 'tf':
            self.get_logger().warn(
                f"Unknown transform_mode '{self.transform_mode}', expected tf/manual/imu/none.",
                throttle_duration_sec=2.0)
            return None

        try:
            stamp = Time.from_msg(msg.header.stamp)
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            rotation = quaternion_to_matrix(q.x, q.y, q.z, q.w)
            translation = np.array([t.x, t.y, t.z], dtype=np.float32)
            return rotation, translation, self.target_frame
        except TransformException as exc:
            if self.fallback_to_manual:
                self.get_logger().warn(
                    f'TF {source_frame}->{self.target_frame} failed, using manual transform: {exc}',
                    throttle_duration_sec=2.0)
                return self.manual_rotation, self.manual_translation, self.target_frame

            self.get_logger().warn(
                f'TF {source_frame}->{self.target_frame} failed; skipping cloud: {exc}',
                throttle_duration_sec=2.0)
            return None

    def compute_level_rotation_from_imu(self, msg: Imu) -> Optional[np.ndarray]:
        """Estimate the LiDAR roll/pitch correction from one IMU sample."""
        if self.imu_mode == 'orientation':
            q = msg.orientation
            norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if norm < 1e-6:
                return None
            sensor_to_world = quaternion_to_matrix(q.x, q.y, q.z, q.w)
            current_up_imu = sensor_to_world.T @ np.array([0.0, 0.0, 1.0], dtype=np.float32)
            accel_norm = None
        elif self.imu_mode == 'accel':
            current_up_imu = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ], dtype=np.float32)
            if not np.all(np.isfinite(current_up_imu)):
                return None
            accel_norm = float(np.linalg.norm(current_up_imu))
            if not self.imu_accel_is_up:
                current_up_imu = -current_up_imu
        else:
            self.get_logger().warn(
                f"Unknown imu_mode '{self.imu_mode}', expected accel/orientation.",
                throttle_duration_sec=2.0)
            return None

        current_up_imu = normalize_vector(current_up_imu)
        if current_up_imu is None:
            return None

        current_up = normalize_vector(self.imu_to_source_rotation @ current_up_imu)
        if current_up is None:
            return None

        if self.imu_mode == 'orientation':
            self.filtered_accel_up = current_up
            return rotation_between_vectors(self.filtered_accel_up, self.level_reference_axis_source)

        if self.imu_filter_mode == 'complementary':
            filtered_up = self.update_complementary_up(msg, current_up, accel_norm)
        elif self.imu_filter_mode in ('lowpass', 'accel_lowpass'):
            filtered_up = self.update_lowpass_up(current_up)
        else:
            self.get_logger().warn(
                f"Unknown imu_filter_mode '{self.imu_filter_mode}', expected lowpass/complementary.",
                throttle_duration_sec=2.0)
            filtered_up = self.update_lowpass_up(current_up)

        if filtered_up is None:
            return None

        return rotation_between_vectors(filtered_up, self.level_reference_axis_source)

    def update_lowpass_up(self, measured_up: np.ndarray) -> Optional[np.ndarray]:
        alpha = float(np.clip(self.imu_accel_alpha, 0.0, 1.0))
        if self.filtered_accel_up is None or alpha >= 1.0:
            self.filtered_accel_up = measured_up
        else:
            blended = (1.0 - alpha) * self.filtered_accel_up + alpha * measured_up
            normalized = normalize_vector(blended)
            if normalized is not None:
                self.filtered_accel_up = normalized

        return self.filtered_accel_up

    def update_complementary_up(
        self,
        msg: Imu,
        measured_up: np.ndarray,
        accel_norm: Optional[float],
    ) -> Optional[np.ndarray]:
        """Fuse gyro prediction with acceleration-based up-vector correction."""
        stamp_sec = self.stamp_to_sec(msg.header.stamp)
        predicted_up = self.filtered_accel_up

        if predicted_up is not None and self.last_imu_stamp_sec is not None:
            dt = stamp_sec - self.last_imu_stamp_sec
            if 0.0 < dt <= self.imu_max_gyro_dt_sec:
                raw_gyro_imu = np.array([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ], dtype=np.float32)
                if np.all(np.isfinite(raw_gyro_imu)):
                    self.update_gyro_bias(raw_gyro_imu, stamp_sec)
                    gyro_imu = raw_gyro_imu - self.gyro_bias
                    gyro_source = self.imu_to_source_rotation @ gyro_imu
                    gyro_norm = float(np.linalg.norm(gyro_source))
                    if gyro_norm > 1e-6:
                        axis = gyro_source / gyro_norm
                        predicted_up = axis_angle_to_matrix(axis, gyro_norm * dt) @ predicted_up
                        predicted_up = normalize_vector(predicted_up)

        if predicted_up is None:
            predicted_up = measured_up

        correction_alpha = float(np.clip(self.imu_accel_alpha, 0.0, 1.0))
        if accel_norm is not None and self.imu_accel_gate_sigma > 1e-6:
            accel_error = (accel_norm - self.imu_expected_accel_norm) / self.imu_accel_gate_sigma
            correction_alpha *= math.exp(-0.5 * accel_error * accel_error)

        fused_up = normalize_vector((1.0 - correction_alpha) * predicted_up + correction_alpha * measured_up)
        if fused_up is not None:
            self.filtered_accel_up = fused_up
        self.last_imu_stamp_sec = stamp_sec
        return self.filtered_accel_up

    def update_gyro_bias(self, gyro_imu: np.ndarray, stamp_sec: float) -> None:
        if self.gyro_bias_ready or not self.estimate_gyro_bias:
            return

        if self.gyro_bias_start_sec is None:
            self.gyro_bias_start_sec = stamp_sec

        elapsed = stamp_sec - self.gyro_bias_start_sec
        if elapsed <= self.gyro_bias_estimation_sec:
            self.gyro_bias_samples.append(gyro_imu.copy())
            self.gyro_bias = np.mean(np.asarray(self.gyro_bias_samples, dtype=np.float32), axis=0)
            return

        if len(self.gyro_bias_samples) >= self.gyro_bias_min_samples:
            self.gyro_bias = np.mean(np.asarray(self.gyro_bias_samples, dtype=np.float32), axis=0)
            self.get_logger().info(
                'IMU gyro bias estimated: '
                f'[{self.gyro_bias[0]:.6f}, {self.gyro_bias[1]:.6f}, {self.gyro_bias[2]:.6f}] rad/s')
        else:
            self.gyro_bias = self.configured_gyro_bias.copy()
            self.get_logger().warn(
                'Not enough IMU samples for gyro bias estimation; using configured bias '
                f'[{self.gyro_bias[0]:.6f}, {self.gyro_bias[1]:.6f}, {self.gyro_bias[2]:.6f}] rad/s.')

        self.gyro_bias_ready = True
        self.gyro_bias_samples.clear()

    def lookup_level_rotation(self, cloud_stamp_sec: float) -> Optional[np.ndarray]:
        if not self.imu_samples:
            return None

        best_sample = None
        best_dt = float('inf')
        for sample_stamp, level_rotation in reversed(self.imu_samples):
            dt = abs(sample_stamp - cloud_stamp_sec)
            if dt < best_dt:
                best_dt = dt
                best_sample = level_rotation
            if sample_stamp <= cloud_stamp_sec:
                break

        if best_sample is None or best_dt > self.imu_timeout_sec:
            return None
        return best_sample

    def stamp_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def crop_roi(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points
        mask = (
            (points[:, 0] >= self.roi['min_x']) & (points[:, 0] <= self.roi['max_x']) &
            (points[:, 1] >= self.roi['min_y']) & (points[:, 1] <= self.roi['max_y']) &
            (points[:, 2] >= self.roi['min_z']) & (points[:, 2] <= self.roi['max_z'])
        )
        return points[mask]

    def apply_ground_z_compensation(self, points: np.ndarray) -> Tuple[np.ndarray, Optional[float], int]:
        """Shift the cloud so the estimated floor remains near z=0.

        The vertical bounce of a humanoid-mounted LiDAR is a translation, not a
        rotation. IMU double integration is avoided here because it drifts
        quickly; using the point cloud floor estimate keeps ROI heights
        relative to the observed ground.
        """
        if not self.enable_ground_z_compensation or points.size == 0:
            return points, None, 0

        measured_ground_z, candidate_count = self.estimate_ground_z(points)
        if measured_ground_z is not None:
            target_ground_z = measured_ground_z + self.ground_z_offset
            if self.filtered_ground_z is None:
                self.filtered_ground_z = target_ground_z
            else:
                if self.ground_max_step > 0.0:
                    delta = target_ground_z - self.filtered_ground_z
                    target_ground_z = self.filtered_ground_z + float(
                        np.clip(delta, -self.ground_max_step, self.ground_max_step))
                alpha = float(np.clip(self.ground_filter_alpha, 0.0, 1.0))
                self.filtered_ground_z = (
                    (1.0 - alpha) * self.filtered_ground_z +
                    alpha * target_ground_z
                )

        if self.filtered_ground_z is None:
            return points, None, candidate_count

        compensated = points.copy()
        compensated[:, 2] -= float(self.filtered_ground_z)
        return compensated, float(self.filtered_ground_z), candidate_count

    def estimate_ground_z(self, points: np.ndarray) -> Tuple[Optional[float], int]:
        """Estimate ground height from a configured near-floor search window."""
        mask = (
            (points[:, 0] >= self.ground_roi['min_x']) &
            (points[:, 0] <= self.ground_roi['max_x']) &
            (points[:, 1] >= self.ground_roi['min_y']) &
            (points[:, 1] <= self.ground_roi['max_y']) &
            (points[:, 2] >= self.ground_roi['min_z']) &
            (points[:, 2] <= self.ground_roi['max_z'])
        )
        candidates = points[mask, 2]
        candidate_count = int(len(candidates))
        if candidate_count < self.ground_min_points:
            return None, candidate_count
        percentile = float(np.clip(self.ground_z_percentile, 0.0, 100.0))
        return float(np.percentile(candidates, percentile)), candidate_count

    def compute_bounds(self, points: np.ndarray) -> dict:
        if points.size == 0:
            return {}
        mins = np.min(points, axis=0)
        maxs = np.max(points, axis=0)
        return {
            'min_x': float(mins[0]),
            'max_x': float(maxs[0]),
            'min_y': float(mins[1]),
            'max_y': float(maxs[1]),
            'min_z': float(mins[2]),
            'max_z': float(maxs[2]),
        }

    def log_input_stats_if_needed(self, source_frame: str, point_count: int, bounds: dict) -> None:
        if not self.log_input_stats:
            return
        now = time.time()
        if now - self.last_input_stats_log < 2.0:
            return
        self.last_input_stats_log = now
        self.get_logger().info(
            f'Input cloud: frame={source_frame}, points={point_count}, bounds={bounds}')

    def voxel_downsample(self, points: np.ndarray, leaf_size: float) -> np.ndarray:
        if points.size == 0 or leaf_size <= 0.0:
            return points
        voxel_indices = np.floor(points / leaf_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        return points[np.sort(unique_indices)]

    def find_max_cluster_size(self, points: np.ndarray, leaf_size: float) -> int:
        """Return the largest connected component size in voxel coordinates."""
        if points.size == 0:
            return 0
        leaf = leaf_size if leaf_size > 0.0 else 0.05
        voxels = np.floor(points / leaf).astype(np.int32)
        unique_voxels, counts = np.unique(voxels, axis=0, return_counts=True)

        coord_to_index = {tuple(coord): idx for idx, coord in enumerate(unique_voxels)}
        unvisited = set(range(len(unique_voxels)))
        offsets = self.neighbor_offsets(self.cluster_connectivity)
        max_size = 0

        while unvisited:
            start_idx = unvisited.pop()
            stack = [start_idx]
            cluster_size = int(counts[start_idx])

            while stack:
                idx = stack.pop()
                base = unique_voxels[idx]
                for offset in offsets:
                    neighbor = tuple((base + offset).tolist())
                    neighbor_idx = coord_to_index.get(neighbor)
                    if neighbor_idx is not None and neighbor_idx in unvisited:
                        unvisited.remove(neighbor_idx)
                        stack.append(neighbor_idx)
                        cluster_size += int(counts[neighbor_idx])

            max_size = max(max_size, cluster_size)

        return max_size

    def neighbor_offsets(self, connectivity: int) -> Iterable[np.ndarray]:
        offsets = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    if connectivity == 6 and abs(dx) + abs(dy) + abs(dz) != 1:
                        continue
                    offsets.append(np.array([dx, dy, dz], dtype=np.int32))
        return offsets

    def update_hysteresis(self, detected_now: bool) -> None:
        if detected_now:
            self.hit_streak += 1
            self.clear_streak = 0
            if self.hit_streak >= self.trigger_frames:
                self.obstacle_state = True
        else:
            self.clear_streak += 1
            self.hit_streak = 0
            if self.clear_streak >= self.clear_frames:
                self.obstacle_state = False

    def publish_state(self, state: bool) -> None:
        self.has_obstacle_pub.publish(Bool(data=bool(state)))

    def publish_cloud(self, points: np.ndarray, stamp, frame_id: str) -> None:
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        cloud = pc2.create_cloud_xyz32(header, points.astype(np.float32).tolist())
        self.roi_cloud_pub.publish(cloud)

    def publish_debug_message(self, **kwargs) -> None:
        if not self.publish_debug:
            return
        now = time.time()
        if now - self.last_debug_publish < self.debug_publish_period_sec:
            return
        self.last_debug_publish = now

        payload = {
            **kwargs,
            'has_obstacle': self.obstacle_state,
            'hit_streak': self.hit_streak,
            'clear_streak': self.clear_streak,
            'roi': self.roi,
            'transform_mode': self.transform_mode,
        }
        self.debug_pub.publish(String(data=json.dumps(payload, ensure_ascii=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoiObstacleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
