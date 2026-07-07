#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_accumulated_scan_pcd.py

文件作用：
  1. 这是验证辅助脚本，不属于功能源码。
  2. 从 rosbag2 中选定一个目标 /fast_lio/cloud_registered 帧。
  3. 把目标帧附近若干帧 world 点云统一反变换到目标时刻 raw body，再转成 base_footprint 标准轴。
  4. 输出累积 scan PCD 和可直接运行的 single_scan_pcd_path YAML，用于验证多帧累积是否能提升 hard frame 重定位稳定性。

重要说明：
  - 该脚本用于离线验证“多帧局部子图”方向，不会改变功能源码。
  - 输出 PCD 已经是 base_footprint 标准轴，因此生成的 YAML 使用 input_mode=registered_world，
    让 C++ single PCD 入口直接把它作为 base scan 参与匹配。
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import sensor_msgs_py.point_cloud2 as pc2


@dataclass
class OdomSample:
    stamp_sec: float
    msg: object


@dataclass
class ReferenceSample:
    bag_time_sec: float
    msg: object


@dataclass
class CloudSample:
    index: int
    stamp_sec: float
    bag_time_sec: float
    msg: object


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_to_rotation_matrix(q_msg) -> np.ndarray:
    x = float(q_msg.x)
    y = float(q_msg.y)
    z = float(q_msg.z)
    w = float(q_msg.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_from_quaternion(q_msg) -> float:
    rotation = quaternion_to_rotation_matrix(q_msg)
    return math.atan2(rotation[1, 0], rotation[0, 0])


def nearest_odom(odoms: list[OdomSample], stamp_sec: float, tolerance_sec: float) -> OdomSample:
    best = min(odoms, key=lambda odom: abs(odom.stamp_sec - stamp_sec), default=None)
    if best is None or abs(best.stamp_sec - stamp_sec) > tolerance_sec:
        raise RuntimeError(f"no odom within {tolerance_sec}s for stamp {stamp_sec}")
    return best


def nearest_reference(references: list[ReferenceSample], bag_time_sec: float, tolerance_sec: float) -> ReferenceSample:
    best = min(references, key=lambda ref: abs(ref.bag_time_sec - bag_time_sec), default=None)
    if best is None or abs(best.bag_time_sec - bag_time_sec) > tolerance_sec:
        raise RuntimeError(f"no reference pose within {tolerance_sec}s for bag time {bag_time_sec}")
    return best


def raw_body_to_base(points_raw: np.ndarray) -> np.ndarray:
    """与 C++ convert_raw_body_to_base_cloud 保持一致。"""
    points_base = np.empty_like(points_raw)
    points_base[:, 0] = -points_raw[:, 2] + 0.072
    points_base[:, 1] = points_raw[:, 0] - 0.004
    points_base[:, 2] = -points_raw[:, 1] + 1.215
    return points_base


def write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    points = points.astype(np.float32)
    with path.open("w", encoding="ascii") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def write_eval_yaml(path: Path, pcd_path: Path, output_dir: Path, reference_xyz_yaw: tuple[float, float, float, float]) -> None:
    x, y, z, yaw = reference_xyz_yaw
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"""# 自动生成的多帧累积 scan 单帧验证配置。
# 由 make_accumulated_scan_pcd.py 生成，仅用于 hard frame 离线验证。

global_relocalization_eval:
  ros__parameters:
    input_mode: "registered_world"
    registered_world_topic: "/fast_lio/cloud_registered"
    body_topic: "/cloud_registered_body"
    odom_topic: "/odom"
    single_scan_pcd_path: "{pcd_path}"
    bag_paths: []
    map_path: ""
    map_dir: "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/pcd"
    map_file_name: "hall_open3d_grounded.pcd"
    map_candidates:
      - "hall_open3d_grounded.pcd"
    output_dir: "{output_dir}"

    convert_raw_body_to_base: true
    raw_body_to_base_xyz: [0.072, -0.004, 1.215]
    force_2d_output: true

    map_leaf_size: 0.25
    scan_leaf_size: 0.30
    min_scan_range: 0.80
    max_scan_range: 20.0
    enable_map_z_crop: true
    map_min_z: 0.2
    map_max_z: 2.5
    enable_scan_z_crop: true
    scan_min_z: 0.2
    scan_max_z: 2.5

    bbs_min_level_res: 0.75
    bbs_max_level: 5
    bbs_voxel_expansion_rate: 2.0
    bbs_num_threads: 8
    bbs_score_threshold_percentage: 0.12
    bbs_timeout_msec: 0
    search_min_xyz: [-35.0, -35.0, -1.0]
    search_max_xyz: [ 35.0,  35.0,  2.0]
    search_min_rpy: [-0.05, -0.05, 0.0]
    search_max_rpy: [0.05, 0.05, 6.283185307]
    top_k: 30

    refine_method: "gicp"
    refine_methods_for_sweep: ["gicp"]
    refine_top_k: true
    max_refine_candidates: 20
    refine_max_iterations: 25
    refine_max_correspondence_distance: 1.5
    refine_transformation_epsilon: 0.001
    refine_euclidean_fitness_epsilon: 0.001
    refine_with_gicp: true
    gicp_max_iterations: 25
    gicp_max_correspondence_distance: 1.5
    gicp_transformation_epsilon: 0.001

    reference_pose_xyzrpy: [{x:.9f}, {y:.9f}, {z:.9f}, 0.0, 0.0, {yaw:.9f}]
    use_bag_reference_pose: false
    reference_pose_topic: "/robot_realpose"
    reference_time_tolerance_sec: 0.05
    enable_reference_sanity_check: true
    reference_max_abs_xy_m: 100.0
    reference_max_abs_z_m: 10.0
    success_translation_thresh: 0.80
    success_yaw_thresh_deg: 15.0

    save_aligned_cloud: false
    metrics_csv_name: "global_relocalization_metrics.csv"
    candidates_csv_name: "global_relocalization_candidates.csv"
    summary_csv_name: "global_relocalization_summary.csv"

    simulated_relocalization_cases:
      - name: "arbitrary_start_no_prior"
        offset_xyzrpy: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
""",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate accumulated base-frame scan PCD and eval YAML.")
    parser.add_argument("--bag", required=True)
    parser.add_argument("--target-index", type=int, required=True)
    parser.add_argument("--before", type=int, default=3)
    parser.add_argument("--after", type=int, default=3)
    parser.add_argument("--odom-tolerance", type=float, default=0.05)
    parser.add_argument("--reference-tolerance", type=float, default=0.05)
    parser.add_argument("--pcd-output", required=True)
    parser.add_argument("--yaml-output", required=True)
    parser.add_argument("--eval-output-dir", required=True)
    args = parser.parse_args()

    rclpy.init()
    point_cloud_type = get_message("sensor_msgs/msg/PointCloud2")
    odom_type = get_message("nav_msgs/msg/Odometry")
    reference_type = get_message("geometry_msgs/msg/PoseWithCovarianceStamped")

    wanted_indices = set(range(args.target_index - args.before, args.target_index + args.after + 1))
    odoms: list[OdomSample] = []
    references: list[ReferenceSample] = []
    clouds: list[CloudSample] = []

    reader = open_reader(args.bag)
    cloud_index = -1
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        bag_time_sec = float(bag_time_ns) * 1e-9
        if topic == "/odom":
            msg = deserialize_message(data, odom_type)
            odoms.append(OdomSample(stamp_to_sec(msg.header.stamp), msg))
            continue
        if topic == "/robot_realpose":
            msg = deserialize_message(data, reference_type)
            references.append(ReferenceSample(bag_time_sec, msg))
            continue
        if topic != "/fast_lio/cloud_registered":
            continue
        cloud_index += 1
        if cloud_index in wanted_indices:
            msg = deserialize_message(data, point_cloud_type)
            clouds.append(CloudSample(cloud_index, stamp_to_sec(msg.header.stamp), bag_time_sec, msg))

    if not clouds:
        raise RuntimeError("no cloud samples collected")
    target = next((cloud for cloud in clouds if cloud.index == args.target_index), None)
    if target is None:
        raise RuntimeError(f"target index {args.target_index} was not collected")

    target_odom = nearest_odom(odoms, target.stamp_sec, args.odom_tolerance)
    target_reference = nearest_reference(references, target.bag_time_sec, args.reference_tolerance)
    rotation_world_target = quaternion_to_rotation_matrix(target_odom.msg.pose.pose.orientation)
    translation_world_target = np.array(
        [
            float(target_odom.msg.pose.pose.position.x),
            float(target_odom.msg.pose.pose.position.y),
            float(target_odom.msg.pose.pose.position.z),
        ],
        dtype=np.float64,
    )

    accumulated_raw_body = []
    for cloud in clouds:
        points_world = pc2.read_points_numpy(cloud.msg, field_names=["x", "y", "z"], skip_nans=True).astype(np.float64)
        points_target_body = (points_world - translation_world_target) @ rotation_world_target
        accumulated_raw_body.append(points_target_body)
    points_base = raw_body_to_base(np.vstack(accumulated_raw_body))

    pcd_path = Path(args.pcd_output)
    yaml_path = Path(args.yaml_output)
    write_ascii_pcd(pcd_path, points_base)

    ref = target_reference.msg.pose.pose
    reference_xyz_yaw = (
        float(ref.position.x),
        float(ref.position.y),
        float(ref.position.z),
        yaw_from_quaternion(ref.orientation),
    )
    write_eval_yaml(yaml_path, pcd_path.resolve(), Path(args.eval_output_dir).resolve(), reference_xyz_yaw)

    rclpy.shutdown()
    print(
        f"accumulated_scan target_index={args.target_index} frames={len(clouds)} "
        f"points={len(points_base)} pcd={pcd_path} yaml={yaml_path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
