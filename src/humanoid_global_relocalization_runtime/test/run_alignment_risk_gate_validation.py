#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_alignment_risk_gate_validation.py

文件作用：
  1. 离线验证“可定位性 / 配准不确定性 / 空闲空间一致性 / 主方向一致性”组合门控。
  2. 输入现有 C++ evaluator 产生的 global_relocalization_metrics.csv，不重新做全局召回和 GICP。
  3. 从真实 bag 重新读取每个候选对应的点云，把 scan 投到候选 map 位姿下，计算不依赖真值的几何可信度指标。
  4. 输出候选级明细和阈值 sweep 汇总，用来判断这些门控能否压低重复走廊里的误接受。

设计说明：
  - 真值只用于最后统计 0.2m/3deg、0.3m/5deg、0.5m/10deg 成功率，不参与任何门控评分。
  - 该脚本验证的是“是否应该自动注入/接受这个候选”，不是重新寻找位姿。
  - 当前默认处理 registered_world 输入链路：/fast_lio/cloud_registered + /odom -> base_footprint。
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
from dataclasses import dataclass
from pathlib import Path
import resource
import time
from typing import Any

import numpy as np
import open3d as o3d
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree

from run_scan_context_keyframe_recall import (
    CloudSample,
    OdomSample,
    nearest_odom,
    open_reader,
    registered_world_cloud_to_body,
    stamp_to_sec,
)


@dataclass(frozen=True)
class MetricSample:
    """一条 evaluator 输出的最终候选，用作门控验证输入。"""

    source_id: str
    bag_path: Path
    bag_frame_index: int
    stamp_sec: float
    final_x: float
    final_y: float
    final_yaw_deg: float
    refine_fitness: float
    candidate_rank: int
    candidate_count: int
    score_ratio: float
    has_reference: bool
    reference_x: float
    reference_y: float
    reference_yaw_deg: float
    translation_error_m: float
    yaw_error_deg: float


@dataclass(frozen=True)
class OccupancyGrid:
    """地图二维占据栅格，用于空闲空间和禁穿墙验证。"""

    origin_x: float
    origin_y: float
    resolution: float
    occupied: np.ndarray


@dataclass(frozen=True)
class GateMetrics:
    """一个候选对应的一组无真值门控指标。"""

    source_id: str
    bag_frame_index: int
    stamp_sec: float
    endpoint_near_ratio: float
    residual_mean_m: float
    residual_p90_m: float
    residual_p95_m: float
    inlier_ratio_0p30: float
    inlier_ratio_0p50: float
    nn_cov_trace: float
    nn_cov_major_m: float
    nn_cov_minor_m: float
    nn_cov_anisotropy: float
    ray_wall_cross_ratio: float
    origin_clear: bool
    principal_orientation_delta_deg: float
    scan_anisotropy: float
    sector_coverage: float
    sector_entropy: float
    front_back_symmetry: float
    localizability_score: float
    alignment_risk_score: float
    translation_error_m: float
    yaw_error_deg: float
    refine_fitness: float


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 根目录。"""

    return Path(__file__).resolve().parents[3]


def normalize_angle_deg(angle: float) -> float:
    """把角度规整到 [-180, 180]。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def current_cpu_seconds() -> float:
    """读取当前进程累计 CPU 秒。"""

    usage = resource.getrusage(resource.RUSAGE_SELF)
    return float(usage.ru_utime + usage.ru_stime)


def current_rss_mb() -> float:
    """读取当前进程最大 RSS，Linux 下 ru_maxrss 单位是 KB。"""

    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def pose_matrix_2d(x: float, y: float, yaw_deg: float) -> np.ndarray:
    """构造 base -> map 的二维刚体变换矩阵。"""

    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    matrix = np.eye(4, dtype=np.float64)
    matrix[0, 0] = c
    matrix[0, 1] = -s
    matrix[1, 0] = s
    matrix[1, 1] = c
    matrix[0, 3] = x
    matrix[1, 3] = y
    return matrix


def transform_points(points: np.ndarray, x: float, y: float, yaw_deg: float) -> np.ndarray:
    """把 base 坐标 scan 点变换到 map 坐标。"""

    pose = pose_matrix_2d(x, y, yaw_deg)
    homogeneous = np.ones((len(points), 4), dtype=np.float64)
    homogeneous[:, :3] = points[:, :3]
    return (homogeneous @ pose.T)[:, :3]


def read_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """从 CSV 行安全读取浮点数。"""

    try:
        return float(row.get(key, "") or default)
    except ValueError:
        return default


def read_int(row: dict[str, str], key: str, default: int = 0) -> int:
    """从 CSV 行安全读取整数。"""

    try:
        return int(float(row.get(key, "") or default))
    except ValueError:
        return default


def load_metric_samples(metrics_globs: list[str], scenario_name: str, max_samples: int) -> list[MetricSample]:
    """读取一个或多个 evaluator metrics CSV，并抽取指定场景的最终候选。"""

    samples: list[MetricSample] = []
    for pattern in metrics_globs:
        for path_text in sorted(glob.glob(pattern)):
            path = Path(path_text)
            source_id = path.parent.name
            with path.open(newline="", encoding="utf-8") as f:
                for row in csv.DictReader(f):
                    if row.get("scenario_name") != scenario_name:
                        continue
                    if row.get("localized") != "1":
                        continue
                    samples.append(
                        MetricSample(
                            source_id=source_id,
                            bag_path=Path(row["bag_path"].strip('"')),
                            bag_frame_index=read_int(row, "bag_frame_index", -1),
                            stamp_sec=read_float(row, "stamp_sec", 0.0),
                            final_x=read_float(row, "final_x_m", 0.0),
                            final_y=read_float(row, "final_y_m", 0.0),
                            final_yaw_deg=read_float(row, "final_yaw_deg", 0.0),
                            refine_fitness=read_float(row, "refine_fitness_score", 999.0),
                            candidate_rank=read_int(row, "refined_candidate_rank", 0),
                            candidate_count=read_int(row, "candidate_count", 0),
                            score_ratio=read_float(row, "score_ratio", 0.0),
                            has_reference=row.get("has_reference") == "1",
                            reference_x=read_float(row, "reference_x_m", 0.0),
                            reference_y=read_float(row, "reference_y_m", 0.0),
                            reference_yaw_deg=read_float(row, "reference_yaw_deg", 0.0),
                            translation_error_m=read_float(row, "translation_error_m", -1.0),
                            yaw_error_deg=read_float(row, "yaw_error_deg", -1.0),
                        )
                    )
                    if max_samples > 0 and len(samples) >= max_samples:
                        return samples
    return samples


def read_bag_clouds(bag_path: Path, target_indices: set[int]) -> tuple[list[OdomSample], dict[int, CloudSample]]:
    """从 bag 读取 /odom 和目标 /fast_lio/cloud_registered 点云。"""

    point_cloud_type = get_message("sensor_msgs/msg/PointCloud2")
    odom_type = get_message("nav_msgs/msg/Odometry")

    odoms: list[OdomSample] = []
    clouds: dict[int, CloudSample] = {}
    reader = open_reader(bag_path)
    cloud_index = -1
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        bag_time_sec = float(bag_time_ns) * 1e-9
        if topic == "/odom":
            msg = deserialize_message(data, odom_type)
            odoms.append(OdomSample(stamp_to_sec(msg.header.stamp), msg))
            continue
        if topic != "/fast_lio/cloud_registered":
            continue
        cloud_index += 1
        if cloud_index not in target_indices:
            continue
        msg = deserialize_message(data, point_cloud_type)
        clouds[cloud_index] = CloudSample(cloud_index, stamp_to_sec(msg.header.stamp), bag_time_sec, msg)
        if len(clouds) == len(target_indices):
            # 仍然保留已经读到的 odom；目标点都齐了后可以提前退出，避免把整个 bag 再扫一遍。
            break
    return odoms, clouds


def voxel_downsample_numpy(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """使用 Open3D 对 NumPy 点云做体素降采样。"""

    if voxel_size <= 0.0 or len(points) == 0:
        return points
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points[:, :3])
    return np.asarray(cloud.voxel_down_sample(voxel_size).points, dtype=np.float64)


def preprocess_scan(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """按门控验证需要裁剪 scan 范围、高度并降采样。"""

    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    radius = np.hypot(points[:, 0], points[:, 1])
    valid = (
        (radius >= args.min_range)
        & (radius <= args.max_range)
        & (points[:, 2] >= args.scan_min_z)
        & (points[:, 2] <= args.scan_max_z)
    )
    points = points[valid]
    points = voxel_downsample_numpy(points, args.scan_voxel_size)
    if args.max_scan_points > 0 and len(points) > args.max_scan_points:
        # 固定步长抽样比随机抽样更容易复现，同时保留整个视野的点。
        step = int(math.ceil(len(points) / args.max_scan_points))
        points = points[::step][: args.max_scan_points]
    return points


def load_map_points(map_path: Path, args: argparse.Namespace) -> np.ndarray:
    """读取地图点云，按墙体/障碍高度裁剪并降采样。"""

    cloud = o3d.io.read_point_cloud(str(map_path))
    if args.map_voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(args.map_voxel_size)
    points = np.asarray(cloud.points, dtype=np.float64)
    valid = (
        np.isfinite(points).all(axis=1)
        & (points[:, 2] >= args.map_min_z)
        & (points[:, 2] <= args.map_max_z)
    )
    return points[valid]


def build_occupancy_grid(points: np.ndarray, args: argparse.Namespace) -> OccupancyGrid:
    """把地图障碍点投影成二维占据栅格。"""

    margin = args.max_range + 2.0
    min_xy = points[:, :2].min(axis=0) - margin
    max_xy = points[:, :2].max(axis=0) + margin
    size = np.ceil((max_xy - min_xy) / args.occupancy_resolution).astype(np.int32) + 1
    occupied = np.zeros((int(size[1]), int(size[0])), dtype=bool)
    ix = np.floor((points[:, 0] - min_xy[0]) / args.occupancy_resolution).astype(np.int32)
    iy = np.floor((points[:, 1] - min_xy[1]) / args.occupancy_resolution).astype(np.int32)
    valid = (ix >= 0) & (iy >= 0) & (ix < size[0]) & (iy < size[1])
    occupied[iy[valid], ix[valid]] = True
    return OccupancyGrid(float(min_xy[0]), float(min_xy[1]), args.occupancy_resolution, occupied)


def grid_indices(grid: OccupancyGrid, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """把 map 坐标批量转换成栅格索引。"""

    ix = np.floor((x - grid.origin_x) / grid.resolution).astype(np.int32)
    iy = np.floor((y - grid.origin_y) / grid.resolution).astype(np.int32)
    inside = (ix >= 0) & (iy >= 0) & (ix < grid.occupied.shape[1]) & (iy < grid.occupied.shape[0])
    return ix, iy, inside


def has_origin_clearance(grid: OccupancyGrid, x: float, y: float, radius_m: float) -> bool:
    """检查候选 base 站位附近是否被地图障碍占据。"""

    cx = int(math.floor((x - grid.origin_x) / grid.resolution))
    cy = int(math.floor((y - grid.origin_y) / grid.resolution))
    radius_cells = int(math.ceil(radius_m / grid.resolution))
    if cx < radius_cells or cy < radius_cells:
        return False
    if cx + radius_cells >= grid.occupied.shape[1] or cy + radius_cells >= grid.occupied.shape[0]:
        return False
    patch = grid.occupied[cy - radius_cells : cy + radius_cells + 1, cx - radius_cells : cx + radius_cells + 1]
    return not bool(patch.any())


def ray_wall_cross_ratio(
    scan_points: np.ndarray,
    scan_in_map: np.ndarray,
    grid: OccupancyGrid,
    x: float,
    y: float,
    args: argparse.Namespace,
) -> float:
    """统计候选位姿下 scan 射线是否在端点之前提前穿过地图障碍。"""

    if len(scan_points) == 0:
        return 1.0
    if len(scan_points) > args.ray_sample_points:
        step = int(math.ceil(len(scan_points) / args.ray_sample_points))
        indices = np.arange(0, len(scan_points), step, dtype=np.int32)[: args.ray_sample_points]
    else:
        indices = np.arange(len(scan_points), dtype=np.int32)

    crossed = 0
    valid_rays = 0
    origin = np.array([x, y], dtype=np.float64)
    for index in indices:
        endpoint = scan_in_map[index, :2]
        length = float(np.linalg.norm(endpoint - origin))
        if length <= args.min_range + args.wall_before_end_tolerance:
            continue
        valid_rays += 1
        check_length = max(args.min_range, length - args.wall_before_end_tolerance)
        steps = int(check_length / args.ray_step)
        if steps <= 0:
            continue
        ratios = np.linspace(args.min_range / length, check_length / length, steps, dtype=np.float64)
        sample_x = x + ratios * (endpoint[0] - x)
        sample_y = y + ratios * (endpoint[1] - y)
        ix, iy, inside = grid_indices(grid, sample_x, sample_y)
        hit = np.zeros_like(inside, dtype=bool)
        hit[inside] = grid.occupied[iy[inside], ix[inside]]
        if bool(hit.any()):
            crossed += 1
    if valid_rays == 0:
        return 1.0
    return float(crossed) / float(valid_rays)


def scan_localizability(points: np.ndarray, sectors: int) -> tuple[float, float, float, float, float]:
    """根据单帧 scan 的几何形状估计可定位性，输出分数和组成项。"""

    if len(points) < 10:
        return 0.0, 1.0, 0.0, 0.0, 1.0

    xy = points[:, :2] - points[:, :2].mean(axis=0)
    cov = np.cov(xy.T)
    eig = np.sort(np.linalg.eigvalsh(cov))[::-1]
    anisotropy = float((eig[0] - eig[1]) / max(eig[0] + eig[1], 1e-9))

    theta = (np.arctan2(points[:, 1], points[:, 0]) + math.pi) / (2.0 * math.pi)
    sector_index = np.minimum(sectors - 1, np.floor(theta * sectors).astype(np.int32))
    counts = np.bincount(sector_index, minlength=sectors).astype(np.float64)
    occupied = counts > 0
    coverage = float(occupied.mean())
    prob = counts / max(float(counts.sum()), 1.0)
    entropy = float(-(prob[prob > 0.0] * np.log(prob[prob > 0.0])).sum() / math.log(float(sectors)))

    half = sectors // 2
    if half > 0 and sectors % 2 == 0:
        left = counts[:half]
        right = counts[half:]
        denominator = np.linalg.norm(left) * np.linalg.norm(right)
        symmetry = float(np.dot(left, right) / denominator) if denominator > 1e-9 else 1.0
    else:
        symmetry = 1.0

    score = (
        0.40 * entropy
        + 0.25 * coverage
        + 0.20 * (1.0 - anisotropy)
        + 0.15 * (1.0 - max(0.0, min(1.0, symmetry)))
    )
    return float(max(0.0, min(1.0, score))), anisotropy, coverage, entropy, symmetry


def principal_orientation(points_xy: np.ndarray) -> tuple[float, float]:
    """用二维 PCA 估计点集主方向和各向异性。"""

    if len(points_xy) < 10:
        return 0.0, 0.0
    centered = points_xy - points_xy.mean(axis=0)
    cov = np.cov(centered.T)
    values, vectors = np.linalg.eigh(cov)
    order = np.argsort(values)[::-1]
    major = vectors[:, order[0]]
    angle = math.degrees(math.atan2(float(major[1]), float(major[0])))
    eig = values[order]
    anisotropy = float((eig[0] - eig[1]) / max(eig[0] + eig[1], 1e-9))
    return angle, anisotropy


def orientation_delta_mod_180(a_deg: float, b_deg: float) -> float:
    """计算两条无向主轴的角度差，范围 [0, 90]。"""

    delta = abs(normalize_angle_deg(a_deg - b_deg))
    if delta > 90.0:
        delta = 180.0 - delta
    return abs(delta)


def residual_covariance_metrics(scan_in_map: np.ndarray, map_points: np.ndarray, nn_indices: np.ndarray) -> tuple[float, float, float, float]:
    """根据最近邻残差估计类似 GICP 不确定性的二维协方差指标。"""

    if len(scan_in_map) < 5 or len(nn_indices) < 5:
        return 999.0, 999.0, 999.0, 1.0
    residual_xy = scan_in_map[:, :2] - map_points[nn_indices, :2]
    cov = np.cov(residual_xy.T)
    values = np.sort(np.maximum(np.linalg.eigvalsh(cov), 0.0))[::-1]
    major = math.sqrt(float(values[0]))
    minor = math.sqrt(float(values[1]))
    trace = float(values.sum())
    anisotropy = float((values[0] - values[1]) / max(values.sum(), 1e-9))
    return trace, major, minor, anisotropy


def evaluate_sample(
    sample: MetricSample,
    scan_points: np.ndarray,
    map_points: np.ndarray,
    map_tree: cKDTree,
    map_xy_tree: cKDTree,
    grid: OccupancyGrid,
    args: argparse.Namespace,
) -> GateMetrics:
    """对一个最终候选计算完整门控指标。"""

    scan = preprocess_scan(scan_points, args)
    scan_in_map = transform_points(scan, sample.final_x, sample.final_y, sample.final_yaw_deg)
    distances, indices = map_tree.query(scan_in_map[:, :3], k=1, workers=-1)

    endpoint_near = float(np.mean(distances <= args.endpoint_near_distance)) if len(distances) else 0.0
    inlier_030 = float(np.mean(distances <= 0.30)) if len(distances) else 0.0
    inlier_050 = float(np.mean(distances <= 0.50)) if len(distances) else 0.0
    residual_mean = float(np.mean(distances)) if len(distances) else 999.0
    residual_p90 = float(np.percentile(distances, 90)) if len(distances) else 999.0
    residual_p95 = float(np.percentile(distances, 95)) if len(distances) else 999.0
    cov_trace, cov_major, cov_minor, cov_anisotropy = residual_covariance_metrics(scan_in_map, map_points, indices)

    wall_ratio = ray_wall_cross_ratio(scan, scan_in_map, grid, sample.final_x, sample.final_y, args)
    origin_clear = has_origin_clearance(grid, sample.final_x, sample.final_y, args.origin_clearance_radius)

    nearby = map_xy_tree.query_ball_point([sample.final_x, sample.final_y], args.orientation_radius)
    map_local = map_points[np.asarray(nearby, dtype=np.int64), :2] if nearby else np.empty((0, 2), dtype=np.float64)
    scan_local = scan_in_map[:, :2]
    map_angle, map_axis_strength = principal_orientation(map_local)
    scan_angle, scan_axis_strength = principal_orientation(scan_local)
    if map_axis_strength < args.min_orientation_anisotropy or scan_axis_strength < args.min_orientation_anisotropy:
        orientation_delta = 0.0
    else:
        orientation_delta = orientation_delta_mod_180(map_angle, scan_angle)

    local_score, scan_anisotropy, sector_coverage, sector_entropy, symmetry = scan_localizability(scan, args.localizability_sectors)
    residual_risk = min(1.0, residual_p95 / max(args.risk_residual_p95_scale, 1e-6))
    wall_risk = min(1.0, wall_ratio / max(args.risk_wall_scale, 1e-6))
    endpoint_risk = max(0.0, 1.0 - endpoint_near)
    local_risk = max(0.0, 1.0 - local_score)
    orientation_risk = min(1.0, orientation_delta / 45.0)
    alignment_risk = (
        0.30 * residual_risk
        + 0.25 * wall_risk
        + 0.20 * endpoint_risk
        + 0.15 * local_risk
        + 0.10 * orientation_risk
    )

    return GateMetrics(
        source_id=sample.source_id,
        bag_frame_index=sample.bag_frame_index,
        stamp_sec=sample.stamp_sec,
        endpoint_near_ratio=endpoint_near,
        residual_mean_m=residual_mean,
        residual_p90_m=residual_p90,
        residual_p95_m=residual_p95,
        inlier_ratio_0p30=inlier_030,
        inlier_ratio_0p50=inlier_050,
        nn_cov_trace=cov_trace,
        nn_cov_major_m=cov_major,
        nn_cov_minor_m=cov_minor,
        nn_cov_anisotropy=cov_anisotropy,
        ray_wall_cross_ratio=wall_ratio,
        origin_clear=origin_clear,
        principal_orientation_delta_deg=orientation_delta,
        scan_anisotropy=scan_anisotropy,
        sector_coverage=sector_coverage,
        sector_entropy=sector_entropy,
        front_back_symmetry=symmetry,
        localizability_score=local_score,
        alignment_risk_score=float(max(0.0, min(1.0, alignment_risk))),
        translation_error_m=sample.translation_error_m,
        yaw_error_deg=sample.yaw_error_deg,
        refine_fitness=sample.refine_fitness,
    )


def is_success(metric: GateMetrics, xy_thresh: float, yaw_thresh: float) -> bool:
    """按指定 xy/yaw 阈值判断真值成功。"""

    return metric.translation_error_m >= 0.0 and metric.translation_error_m <= xy_thresh and metric.yaw_error_deg <= yaw_thresh


def pass_gate(metric: GateMetrics, params: dict[str, float]) -> bool:
    """按一组门控阈值判断是否允许自动接受候选。"""

    if params.get("require_origin_clear", 0.0) > 0.5 and not metric.origin_clear:
        return False
    return (
        metric.endpoint_near_ratio >= params["min_endpoint_near"]
        and metric.inlier_ratio_0p50 >= params["min_inlier_0p50"]
        and metric.residual_p95_m <= params["max_residual_p95"]
        and metric.ray_wall_cross_ratio <= params["max_wall_cross"]
        and metric.localizability_score >= params["min_localizability"]
        and metric.alignment_risk_score <= params["max_alignment_risk"]
        and metric.refine_fitness <= params["max_refine_fitness"]
    )


def default_gate_sweep() -> list[dict[str, float]]:
    """给出从宽到严的完整门控组合，用同一组指标做对照。"""

    return [
        {
            "name": "risk_lenient",
            "min_endpoint_near": 0.55,
            "min_inlier_0p50": 0.55,
            "max_residual_p95": 1.45,
            "max_wall_cross": 1.00,
            "min_localizability": 0.60,
            "max_alignment_risk": 0.85,
            "max_refine_fitness": 0.07,
            "require_origin_clear": 0.0,
        },
        {
            "name": "risk_balanced",
            "min_endpoint_near": 0.55,
            "min_inlier_0p50": 0.55,
            "max_residual_p95": 1.45,
            "max_wall_cross": 1.00,
            "min_localizability": 0.60,
            "max_alignment_risk": 0.80,
            "max_refine_fitness": 0.045,
            "require_origin_clear": 0.0,
        },
        {
            "name": "risk_strict",
            "min_endpoint_near": 0.55,
            "min_inlier_0p50": 0.55,
            "max_residual_p95": 1.45,
            "max_wall_cross": 1.00,
            "min_localizability": 0.60,
            "max_alignment_risk": 0.80,
            "max_refine_fitness": 0.035,
            "require_origin_clear": 0.0,
        },
        {
            "name": "risk_origin_strict",
            "min_endpoint_near": 0.60,
            "min_inlier_0p50": 0.60,
            "max_residual_p95": 1.30,
            "max_wall_cross": 1.00,
            "min_localizability": 0.65,
            "max_alignment_risk": 0.75,
            "max_refine_fitness": 0.045,
            "require_origin_clear": 1.0,
        },
    ]


def write_details(path: Path, rows: list[dict[str, str]]) -> None:
    """写出候选级门控指标 CSV。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [])
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, rows: list[dict[str, str]]) -> None:
    """写出门控组合汇总 CSV。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [])
        writer.writeheader()
        writer.writerows(rows)


def summarize(metrics: list[GateMetrics], output: Path) -> list[dict[str, str]]:
    """统计 baseline 和各门控组合在三档阈值下的成功/误接受情况。"""

    rows: list[dict[str, str]] = []
    threshold_sets = [
        ("strict_0.2m_3deg", 0.2, 3.0),
        ("practical_0.3m_5deg", 0.3, 5.0),
        ("recoverable_0.5m_10deg", 0.5, 10.0),
    ]

    gate_sets = [{"name": "baseline_accept_all"}] + default_gate_sweep()
    for gate in gate_sets:
        accepted = metrics if gate["name"] == "baseline_accept_all" else [m for m in metrics if pass_gate(m, gate)]
        rejected = len(metrics) - len(accepted)
        for label, xy, yaw in threshold_sets:
            success = sum(is_success(m, xy, yaw) for m in accepted)
            false_accept = len(accepted) - success
            rejected_success = sum(is_success(m, xy, yaw) for m in metrics if m not in accepted)
            rows.append(
                {
                    "gate_name": gate["name"],
                    "threshold": label,
                    "total": str(len(metrics)),
                    "accepted": str(len(accepted)),
                    "rejected": str(rejected),
                    "accepted_success": str(success),
                    "accepted_success_rate": f"{success / max(len(accepted), 1):.6f}",
                    "false_accept": str(false_accept),
                    "false_accept_rate": f"{false_accept / max(len(accepted), 1):.6f}",
                    "rejected_true_success": str(rejected_success),
                    "auto_success_rate_over_total": f"{success / max(len(metrics), 1):.6f}",
                }
            )

    write_summary(output, rows)
    return rows


def run(args: argparse.Namespace) -> int:
    """脚本主流程：读取 metrics、读 bag、计算指标、写出明细和汇总。"""

    start_wall = time.time()
    start_cpu = current_cpu_seconds()
    workspace = args.workspace.resolve()
    map_path = args.map if args.map.is_absolute() else workspace / args.map

    samples = load_metric_samples(args.metrics_glob, args.scenario_name, args.max_samples)
    if not samples:
        raise RuntimeError("没有从 metrics 中读取到可验证样本")

    bag_paths = sorted({sample.bag_path for sample in samples})
    if len(bag_paths) != 1:
        raise RuntimeError(f"当前脚本一次只处理一个 bag，实际={bag_paths}")
    bag_path = bag_paths[0]
    target_indices = {sample.bag_frame_index for sample in samples}

    print(f"[alignment_risk_gate] samples={len(samples)} unique_frames={len(target_indices)} bag={bag_path}", flush=True)
    odoms, clouds = read_bag_clouds(bag_path, target_indices)
    print(f"[alignment_risk_gate] loaded_clouds={len(clouds)} odoms={len(odoms)}", flush=True)

    map_points = load_map_points(map_path, args)
    map_tree = cKDTree(map_points[:, :3])
    map_xy_tree = cKDTree(map_points[:, :2])
    grid = build_occupancy_grid(map_points, args)
    print(f"[alignment_risk_gate] map_points={len(map_points)} rss_mb={current_rss_mb():.1f}", flush=True)

    details: list[dict[str, str]] = []
    gate_metrics: list[GateMetrics] = []
    for index, sample in enumerate(samples):
        cloud = clouds.get(sample.bag_frame_index)
        if cloud is None:
            continue
        odom = nearest_odom(odoms, cloud.stamp_sec)
        scan_points = registered_world_cloud_to_body(cloud, odom)
        metric = evaluate_sample(sample, scan_points, map_points, map_tree, map_xy_tree, grid, args)
        gate_metrics.append(metric)
        details.append(
            {
                "source_id": metric.source_id,
                "bag_frame_index": str(metric.bag_frame_index),
                "stamp_sec": f"{metric.stamp_sec:.6f}",
                "endpoint_near_ratio": f"{metric.endpoint_near_ratio:.6f}",
                "residual_mean_m": f"{metric.residual_mean_m:.6f}",
                "residual_p90_m": f"{metric.residual_p90_m:.6f}",
                "residual_p95_m": f"{metric.residual_p95_m:.6f}",
                "inlier_ratio_0p30": f"{metric.inlier_ratio_0p30:.6f}",
                "inlier_ratio_0p50": f"{metric.inlier_ratio_0p50:.6f}",
                "nn_cov_trace": f"{metric.nn_cov_trace:.6f}",
                "nn_cov_major_m": f"{metric.nn_cov_major_m:.6f}",
                "nn_cov_minor_m": f"{metric.nn_cov_minor_m:.6f}",
                "nn_cov_anisotropy": f"{metric.nn_cov_anisotropy:.6f}",
                "ray_wall_cross_ratio": f"{metric.ray_wall_cross_ratio:.6f}",
                "origin_clear": "1" if metric.origin_clear else "0",
                "principal_orientation_delta_deg": f"{metric.principal_orientation_delta_deg:.6f}",
                "scan_anisotropy": f"{metric.scan_anisotropy:.6f}",
                "sector_coverage": f"{metric.sector_coverage:.6f}",
                "sector_entropy": f"{metric.sector_entropy:.6f}",
                "front_back_symmetry": f"{metric.front_back_symmetry:.6f}",
                "localizability_score": f"{metric.localizability_score:.6f}",
                "alignment_risk_score": f"{metric.alignment_risk_score:.6f}",
                "refine_fitness": f"{metric.refine_fitness:.6f}",
                "translation_error_m": f"{metric.translation_error_m:.6f}",
                "yaw_error_deg": f"{metric.yaw_error_deg:.6f}",
                "success_0p2m_3deg": "1" if is_success(metric, 0.2, 3.0) else "0",
                "success_0p3m_5deg": "1" if is_success(metric, 0.3, 5.0) else "0",
                "success_0p5m_10deg": "1" if is_success(metric, 0.5, 10.0) else "0",
            }
        )
        if (index + 1) % max(1, args.progress_interval) == 0:
            print(f"[alignment_risk_gate] progress {index + 1}/{len(samples)}", flush=True)

    detail_output = args.output if args.output.is_absolute() else workspace / args.output
    summary_output = args.summary_output if args.summary_output.is_absolute() else workspace / args.summary_output
    write_details(detail_output, details)
    summary_rows = summarize(gate_metrics, summary_output)

    elapsed = time.time() - start_wall
    cpu_cores = (current_cpu_seconds() - start_cpu) / max(elapsed, 1e-6)
    print(f"[alignment_risk_gate] details={detail_output}", flush=True)
    print(f"[alignment_risk_gate] summary={summary_output}", flush=True)
    print(f"[alignment_risk_gate] elapsed_sec={elapsed:.3f} cpu_core_equiv={cpu_cores:.2f} rss_mb={current_rss_mb():.1f}", flush=True)
    for row in summary_rows:
        if row["threshold"] == "practical_0.3m_5deg":
            print(
                "[alignment_risk_gate] "
                f"gate={row['gate_name']} accepted={row['accepted']}/{row['total']} "
                f"ok={row['accepted_success']} false={row['false_accept']} "
                f"reject_good={row['rejected_true_success']}",
                flush=True,
            )
    return 0


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""

    root = repo_root_from_script()
    parser = argparse.ArgumentParser(description="验证 alignment risk / localizability 候选门控。")
    parser.add_argument("--workspace", type=Path, default=root, help="humanoid_ws 工作空间")
    parser.add_argument(
        "--metrics-glob",
        action="append",
        default=[],
        help="global_relocalization_metrics.csv 路径或 glob，可重复传入",
    )
    parser.add_argument("--scenario-name", default="arbitrary_start_no_prior", help="只验证该离线场景")
    parser.add_argument("--max-samples", type=int, default=0, help="最多验证多少条 metrics；0 表示全量")
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"), help="全局地图 PCD")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/alignment_risk_gate/details.csv"), help="候选指标明细 CSV")
    parser.add_argument("--summary-output", type=Path, default=Path(".codex_tmp/alignment_risk_gate/summary.csv"), help="门控汇总 CSV")
    parser.add_argument("--progress-interval", type=int, default=25, help="每处理多少条样本打印一次进度")

    parser.add_argument("--map-voxel-size", type=float, default=0.12, help="地图障碍点降采样尺寸")
    parser.add_argument("--map-min-z", type=float, default=0.15, help="地图障碍最低高度")
    parser.add_argument("--map-max-z", type=float, default=2.20, help="地图障碍最高高度")
    parser.add_argument("--occupancy-resolution", type=float, default=0.18, help="二维占据栅格分辨率")

    parser.add_argument("--scan-voxel-size", type=float, default=0.12, help="scan 门控计算降采样尺寸")
    parser.add_argument("--max-scan-points", type=int, default=2200, help="每帧最多参与门控的 scan 点数")
    parser.add_argument("--min-range", type=float, default=0.35, help="忽略过近点")
    parser.add_argument("--max-range", type=float, default=18.0, help="门控计算最大半径")
    parser.add_argument("--scan-min-z", type=float, default=-1.0, help="scan 最低高度")
    parser.add_argument("--scan-max-z", type=float, default=2.20, help="scan 最高高度")

    parser.add_argument("--endpoint-near-distance", type=float, default=0.45, help="端点贴近地图表面的距离阈值")
    parser.add_argument("--ray-sample-points", type=int, default=240, help="每帧禁穿墙射线采样数量")
    parser.add_argument("--ray-step", type=float, default=0.18, help="禁穿墙射线采样步长")
    parser.add_argument("--wall-before-end-tolerance", type=float, default=0.70, help="端点前该距离内命中不算穿墙")
    parser.add_argument("--origin-clearance-radius", type=float, default=0.25, help="候选站位清空半径")

    parser.add_argument("--orientation-radius", type=float, default=8.0, help="主方向一致性使用的地图局部半径")
    parser.add_argument("--min-orientation-anisotropy", type=float, default=0.20, help="主方向强度低于该值时跳过方向差惩罚")
    parser.add_argument("--localizability-sectors", type=int, default=72, help="可定位性角度扇区数")
    parser.add_argument("--risk-residual-p95-scale", type=float, default=1.20, help="alignment risk 中 residual p95 归一化尺度")
    parser.add_argument("--risk-wall-scale", type=float, default=0.40, help="alignment risk 中穿墙比例归一化尺度")
    if not parser.parse_known_args()[0].metrics_glob:
        parser.set_defaults(
            metrics_glob=[
                str(root / ".codex_tmp/bbs2d_integrated_random100/rand100_seed*/global_relocalization_metrics.csv")
            ]
        )
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
