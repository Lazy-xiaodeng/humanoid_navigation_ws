#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_virtual_lidar_relocalization_validation.py

文件作用：
  1. 离线验证“地图虚拟 LiDAR scan 数据库 + 描述子检索 + MCL 多假设验证 + 可见性/禁穿墙门控 + GICP 精修”的冷启动重定位效果。
  2. 从先验 PCD 地图中提取障碍占据栅格，在可站立候选位置模拟 360 度 LiDAR 首次命中距离，生成虚拟 scan descriptor 数据库。
  3. 在线查询阶段把真实目标 scan 转成同样的 range/scan-context 描述子，检索 top-K 虚拟候选，并用粒子滤波式多假设验证重排候选。
  4. 对重排后的少量候选执行 Open3D Generalized ICP，输出严格阈值成功率、资源消耗、候选明细和失败位置，用来判断是否值得移植进 C++ 运行态。

重要说明：
  - 该脚本是完整离线验证工具，不发布 TF，不注入 initialpose，也不改变线上节点。
  - 虚拟 LiDAR 库只由地图生成，不使用目标帧附近真实 keyframe，因此用于验证静止冷启动更接近上线场景。
  - MCL 这里用于静止恢复的多假设重排：没有运动输入时，粒子通过可见性似然和描述子似然迭代收敛；恢复场景接入 odom 后可扩展 motion model。
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import resource
import time
from dataclasses import dataclass

import numpy as np
import open3d as o3d


@dataclass(frozen=True)
class TargetSample:
    """一个待验证的目标 scan 及其真实 map 位姿。"""

    target_id: str
    cloud_index: int
    reference_x: float
    reference_y: float
    reference_yaw_deg: float
    pcd_path: Path


@dataclass(frozen=True)
class OccupancyGrid:
    """由地图障碍点投影得到的二维占据栅格。"""

    origin_x: float
    origin_y: float
    resolution: float
    occupied: np.ndarray


@dataclass(frozen=True)
class VirtualDatabase:
    """离线虚拟 LiDAR scan 数据库。"""

    candidates_xy: np.ndarray
    descriptors: np.ndarray
    ranges: np.ndarray
    hit_counts: np.ndarray


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""

    return Path(__file__).resolve().parents[3]


def normalize_angle_deg(angle: float) -> float:
    """把角度规整到 [-180, 180]，方便误差和 yaw 候选去重。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def yaw_from_matrix(matrix: np.ndarray) -> float:
    """从 4x4 变换矩阵读取 yaw，单位 deg。"""

    return math.degrees(math.atan2(float(matrix[1, 0]), float(matrix[0, 0])))


def pose_matrix(x: float, y: float, yaw_deg: float) -> np.ndarray:
    """构造 source(scan/base) 到 target(map) 的 map->base 初始位姿矩阵。"""

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


def current_rss_mb() -> float:
    """读取当前进程最大 RSS；Linux ru_maxrss 单位为 KB。"""

    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def read_targets(targets_csv: Path) -> list[TargetSample]:
    """读取已导出的目标 PCD CSV，统一 hard52/rand100/edge100 的输入口径。"""

    targets: list[TargetSample] = []
    with targets_csv.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            targets.append(
                TargetSample(
                    target_id=row.get("target_id", f"metric_{len(targets) + 1:03d}"),
                    cloud_index=int(row["cloud_index"]),
                    reference_x=float(row["x"]),
                    reference_y=float(row["y"]),
                    reference_yaw_deg=float(row["yaw_deg"]),
                    pcd_path=Path(row["pcd"]),
                )
            )
    return targets


def limit_targets(targets: list[TargetSample], max_targets: int) -> list[TargetSample]:
    """按需限制目标数量，只用于脚本 smoke；正式统计默认 max_targets=0 表示全量。"""

    if max_targets <= 0:
        return targets
    return targets[:max_targets]


def load_map_points(map_path: Path, args: argparse.Namespace) -> np.ndarray:
    """读取全局 PCD 地图，并按障碍高度范围过滤供虚拟 LiDAR 和 GICP 使用。"""

    cloud = o3d.io.read_point_cloud(str(map_path))
    points = np.asarray(cloud.points, dtype=np.float64)
    valid = (
        np.isfinite(points).all(axis=1)
        & (points[:, 2] >= args.map_min_z)
        & (points[:, 2] <= args.map_max_z)
    )
    return points[valid]


def build_occupancy_grid(points: np.ndarray, args: argparse.Namespace) -> OccupancyGrid:
    """把地图障碍点压成二维占据栅格，用于候选站位筛选、射线模拟和禁穿墙检查。"""

    margin = args.max_radius + 2.0
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
    """把 map 坐标批量转换为占据栅格索引，并返回是否在地图内。"""

    ix = np.floor((x - grid.origin_x) / grid.resolution).astype(np.int32)
    iy = np.floor((y - grid.origin_y) / grid.resolution).astype(np.int32)
    inside = (ix >= 0) & (iy >= 0) & (ix < grid.occupied.shape[1]) & (iy < grid.occupied.shape[0])
    return ix, iy, inside


def has_clearance(grid: OccupancyGrid, x: float, y: float, radius_m: float) -> bool:
    """检查候选站位周围是否有足够机器人半径，避免把粒子放进墙体或障碍物内。"""

    cx = int(math.floor((x - grid.origin_x) / grid.resolution))
    cy = int(math.floor((y - grid.origin_y) / grid.resolution))
    radius_cells = int(math.ceil(radius_m / grid.resolution))
    if cx < radius_cells or cy < radius_cells:
        return False
    if cx + radius_cells >= grid.occupied.shape[1] or cy + radius_cells >= grid.occupied.shape[0]:
        return False
    patch = grid.occupied[cy - radius_cells : cy + radius_cells + 1, cx - radius_cells : cx + radius_cells + 1]
    return not bool(patch.any())


def precompute_ray_offsets(args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """预计算所有 beam 在各距离步长处的局部坐标偏移，避免每个候选重复三角函数。"""

    angles = np.linspace(-math.pi, math.pi, args.sectors, endpoint=False, dtype=np.float64)
    distances = np.arange(args.min_range, args.max_radius + 1e-9, args.ray_step, dtype=np.float64)
    cos_v = np.cos(angles)[:, None]
    sin_v = np.sin(angles)[:, None]
    return angles, distances, cos_v * distances[None, :], sin_v * distances[None, :]


def raycast_ranges(
    grid: OccupancyGrid,
    x: float,
    y: float,
    offsets_x: np.ndarray,
    offsets_y: np.ndarray,
    distances: np.ndarray,
) -> np.ndarray:
    """从 map 坐标中的候选位置做 360 度首次占据命中射线模拟。"""

    sample_x = x + offsets_x
    sample_y = y + offsets_y
    ix, iy, inside = grid_indices(grid, sample_x, sample_y)
    hit = np.zeros_like(inside, dtype=bool)
    hit[inside] = grid.occupied[iy[inside], ix[inside]]
    any_hit = hit.any(axis=1)
    first = np.argmax(hit, axis=1)
    ranges = np.full(hit.shape[0], np.nan, dtype=np.float32)
    ranges[any_hit] = distances[first[any_hit]].astype(np.float32)
    return ranges


def descriptor_from_ranges(ranges: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """把每个 sector 的首次命中距离编码成 ring/sector 描述子。"""

    descriptor = np.zeros((args.rings, args.sectors), dtype=np.float32)
    valid = np.isfinite(ranges)
    if not valid.any():
      return descriptor
    ring_index = np.minimum(args.rings - 1, np.floor(ranges[valid] / args.max_radius * args.rings).astype(np.int32))
    sector_index = np.nonzero(valid)[0]
    descriptor[ring_index, sector_index] = 1.0
    # 给相邻 ring 一个弱响应，降低真实 scan 与虚拟栅格之间的小范围量化误差。
    lower = np.maximum(0, ring_index - 1)
    upper = np.minimum(args.rings - 1, ring_index + 1)
    descriptor[lower, sector_index] = np.maximum(descriptor[lower, sector_index], 0.35)
    descriptor[upper, sector_index] = np.maximum(descriptor[upper, sector_index], 0.35)
    return descriptor


def query_ranges_from_points(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """把真实 scan 点云压成与虚拟 LiDAR 相同的每扇区最近距离观测。"""

    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    radius = np.hypot(points[:, 0], points[:, 1])
    valid = (
        (radius >= args.min_range)
        & (radius <= args.max_radius)
        & (points[:, 2] >= args.scan_min_z)
        & (points[:, 2] <= args.scan_max_z)
    )
    points = points[valid]
    radius = radius[valid]
    ranges = np.full(args.sectors, np.inf, dtype=np.float32)
    if len(points) == 0:
        ranges[:] = np.nan
        return ranges
    theta = (np.arctan2(points[:, 1], points[:, 0]) + math.pi) / (2.0 * math.pi)
    sector_index = np.minimum(args.sectors - 1, np.floor(theta * args.sectors).astype(np.int32))
    np.minimum.at(ranges, sector_index, radius.astype(np.float32))
    ranges[~np.isfinite(ranges)] = np.nan
    return ranges


def descriptor_distance(query: np.ndarray, candidate: np.ndarray) -> tuple[float, int]:
    """对两个描述子做循环 yaw shift 匹配，返回最小距离和最佳 shift。"""

    sectors = query.shape[1]
    best_distance = float("inf")
    best_shift = 0
    for shift in range(sectors):
        shifted = np.roll(candidate, shift, axis=1)
        occupied = (query > 0.0) | (shifted > 0.0)
        if not occupied.any():
            continue
        intersection = np.minimum(query, shifted)[occupied].sum()
        union = np.maximum(query, shifted)[occupied].sum()
        distance = 1.0 - float(intersection) / float(max(1e-6, union))
        if distance < best_distance:
            best_distance = distance
            best_shift = shift
    return best_distance, best_shift


def generate_candidate_xy(points: np.ndarray, grid: OccupancyGrid, args: argparse.Namespace) -> np.ndarray:
    """从地图包围盒生成可站立候选点，并用机器人半径和可见命中数量过滤。"""

    min_xy = points[:, :2].min(axis=0) - args.candidate_margin
    max_xy = points[:, :2].max(axis=0) + args.candidate_margin
    xs = np.arange(min_xy[0], max_xy[0] + 1e-9, args.candidate_grid_size)
    ys = np.arange(min_xy[1], max_xy[1] + 1e-9, args.candidate_grid_size)
    _, distances, offsets_x, offsets_y = precompute_ray_offsets(args)
    candidates: list[tuple[float, float]] = []
    for x in xs:
        for y in ys:
            if not has_clearance(grid, float(x), float(y), args.robot_clearance_radius):
                continue
            ranges = raycast_ranges(grid, float(x), float(y), offsets_x, offsets_y, distances)
            if int(np.isfinite(ranges).sum()) >= args.min_virtual_hits:
                candidates.append((float(x), float(y)))
    return np.asarray(candidates, dtype=np.float32)


def build_virtual_database(points: np.ndarray, grid: OccupancyGrid, args: argparse.Namespace) -> VirtualDatabase:
    """离线生成虚拟 LiDAR scan descriptor 数据库。"""

    cache = args.database_cache
    if cache is not None and cache.exists() and not args.rebuild_database:
        data = np.load(cache)
        return VirtualDatabase(data["candidates_xy"], data["descriptors"], data["ranges"], data["hit_counts"])

    start = time.perf_counter()
    _, distances, offsets_x, offsets_y = precompute_ray_offsets(args)
    candidates_xy = generate_candidate_xy(points, grid, args)
    descriptors = np.zeros((len(candidates_xy), args.rings, args.sectors), dtype=np.float32)
    ranges = np.zeros((len(candidates_xy), args.sectors), dtype=np.float32)
    hit_counts = np.zeros(len(candidates_xy), dtype=np.int32)
    for index, (x, y) in enumerate(candidates_xy):
        candidate_ranges = raycast_ranges(grid, float(x), float(y), offsets_x, offsets_y, distances)
        ranges[index] = np.nan_to_num(candidate_ranges, nan=0.0).astype(np.float32)
        hit_counts[index] = int(np.isfinite(candidate_ranges).sum())
        descriptors[index] = descriptor_from_ranges(candidate_ranges, args)
        if index % max(1, len(candidates_xy) // 10) == 0:
            print(f"[virtual_lidar] database progress {index}/{len(candidates_xy)}", flush=True)
    if cache is not None:
        cache.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            cache,
            candidates_xy=candidates_xy,
            descriptors=descriptors,
            ranges=ranges,
            hit_counts=hit_counts,
        )
    print(
        f"[virtual_lidar] database candidates={len(candidates_xy)} "
        f"build_sec={time.perf_counter() - start:.2f} rss_mb={current_rss_mb():.1f}",
        flush=True,
    )
    return VirtualDatabase(candidates_xy, descriptors, ranges, hit_counts)


def retrieve_virtual_candidates(
    query_descriptor: np.ndarray,
    database: VirtualDatabase,
    args: argparse.Namespace,
) -> list[dict[str, float | int]]:
    """用描述子检索 top-K 虚拟位置，并把 shift 转成粗 yaw 初值。"""

    scored: list[dict[str, float | int]] = []
    for index in range(len(database.candidates_xy)):
        distance, shift = descriptor_distance(query_descriptor, database.descriptors[index])
        # candidate 描述子在 map 坐标 yaw=0 下生成；query 描述子在 base 坐标下生成。
        # descriptor_distance 里 roll(candidate, shift) 对齐 query，因此 map->base yaw 取负 shift 角。
        yaw_deg = normalize_angle_deg(-float(shift) * 360.0 / float(args.sectors))
        scored.append(
            {
                "db_index": index,
                "descriptor_distance": distance,
                "shift": shift,
                "x": float(database.candidates_xy[index, 0]),
                "y": float(database.candidates_xy[index, 1]),
                "yaw_deg": yaw_deg,
            }
        )
    scored.sort(key=lambda item: float(item["descriptor_distance"]))
    return scored[: max(1, args.descriptor_top_k)]


def ray_likelihood(query_ranges: np.ndarray, expected_ranges: np.ndarray, args: argparse.Namespace) -> tuple[float, float, float]:
    """计算真实 scan 与虚拟 scan 的可见性似然、平均距离误差和穿墙比例。"""

    expected = expected_ranges.astype(np.float32)
    expected = np.where(expected > 0.0, expected, np.nan)
    valid = np.isfinite(query_ranges) & np.isfinite(expected)
    if not valid.any():
        return 0.0, float("inf"), 1.0
    diff = np.abs(query_ranges[valid] - expected[valid])
    mean_error = float(np.mean(diff))
    overlap = float(valid.sum()) / float(max(1, np.isfinite(query_ranges).sum()))
    behind_wall = float(np.mean((query_ranges[valid] - expected[valid]) > args.wall_penetration_tolerance))
    likelihood = math.exp(-mean_error / max(1e-6, args.range_sigma)) * overlap * max(0.0, 1.0 - behind_wall)
    return likelihood, mean_error, behind_wall


def rotate_ranges(ranges: np.ndarray, shift: int) -> np.ndarray:
    """按 yaw shift 旋转虚拟 range image，使其与 query base 扇区对齐。"""

    return np.roll(ranges, int(shift))


def mcl_verify_candidates(
    query_ranges: np.ndarray,
    retrieved: list[dict[str, float | int]],
    database: VirtualDatabase,
    args: argparse.Namespace,
) -> list[dict[str, float | int]]:
    """静止恢复专用 MCL：围绕检索候选采样粒子，通过可见性似然重排多假设。"""

    rng = np.random.default_rng(args.random_seed)
    particles: list[dict[str, float | int]] = []
    for item in retrieved[: args.mcl_seed_top_k]:
        db_index = int(item["db_index"])
        base_shift = int(item["shift"])
        for _ in range(args.particles_per_seed):
            yaw_jitter = float(rng.normal(0.0, args.particle_yaw_std_deg))
            xy_jitter = rng.normal(0.0, args.particle_xy_std_m, size=2)
            shift_jitter = int(round(yaw_jitter / (360.0 / float(args.sectors))))
            shift = (base_shift + shift_jitter) % args.sectors
            likelihood, mean_error, wall_ratio = ray_likelihood(
                query_ranges,
                rotate_ranges(database.ranges[db_index], shift),
                args,
            )
            particles.append(
                {
                    "db_index": db_index,
                    "x": float(item["x"]) + float(xy_jitter[0]),
                    "y": float(item["y"]) + float(xy_jitter[1]),
                    "yaw_deg": normalize_angle_deg(float(item["yaw_deg"]) + yaw_jitter),
                    "shift": shift,
                    "descriptor_distance": float(item["descriptor_distance"]),
                    "ray_likelihood": likelihood,
                    "ray_mean_error": mean_error,
                    "wall_ratio": wall_ratio,
                    "weight": likelihood * math.exp(-float(item["descriptor_distance"]) / max(1e-6, args.descriptor_sigma)),
                }
            )

    for _ in range(max(0, args.mcl_iterations - 1)):
        weights = np.asarray([float(p["weight"]) for p in particles], dtype=np.float64)
        if weights.sum() <= 0.0:
            break
        weights /= weights.sum()
        chosen = rng.choice(len(particles), size=len(particles), replace=True, p=weights)
        new_particles: list[dict[str, float | int]] = []
        for index in chosen:
            parent = particles[int(index)]
            db_index = int(parent["db_index"])
            yaw_jitter = float(rng.normal(0.0, args.particle_yaw_std_deg * 0.5))
            shift_jitter = int(round(yaw_jitter / (360.0 / float(args.sectors))))
            shift = (int(parent["shift"]) + shift_jitter) % args.sectors
            xy_jitter = rng.normal(0.0, args.particle_xy_std_m * 0.5, size=2)
            likelihood, mean_error, wall_ratio = ray_likelihood(
                query_ranges,
                rotate_ranges(database.ranges[db_index], shift),
                args,
            )
            new_particles.append(
                {
                    "db_index": db_index,
                    "x": float(parent["x"]) + float(xy_jitter[0]),
                    "y": float(parent["y"]) + float(xy_jitter[1]),
                    "yaw_deg": normalize_angle_deg(float(parent["yaw_deg"]) + yaw_jitter),
                    "shift": shift,
                    "descriptor_distance": float(parent["descriptor_distance"]),
                    "ray_likelihood": likelihood,
                    "ray_mean_error": mean_error,
                    "wall_ratio": wall_ratio,
                    "weight": likelihood * math.exp(-float(parent["descriptor_distance"]) / max(1e-6, args.descriptor_sigma)),
                }
            )
        particles = new_particles

    particles.sort(key=lambda item: (-float(item["weight"]), float(item["ray_mean_error"]), float(item["descriptor_distance"])))
    # 对非常接近的粒子做去重，保留真正不同的多假设。
    selected: list[dict[str, float | int]] = []
    for item in particles:
        duplicate = False
        for kept in selected:
            if (
                math.hypot(float(item["x"]) - float(kept["x"]), float(item["y"]) - float(kept["y"])) <= args.duplicate_xy_m
                and abs(normalize_angle_deg(float(item["yaw_deg"]) - float(kept["yaw_deg"]))) <= args.duplicate_yaw_deg
            ):
                duplicate = True
                break
        if not duplicate:
            selected.append(item)
        if len(selected) >= args.refine_top_k:
            break
    return selected


def make_o3d_cloud(points: np.ndarray, voxel_size: float) -> o3d.geometry.PointCloud:
    """把 numpy 点集转成 Open3D 点云并可选降采样。"""

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size)
    return cloud


def load_scan_points(path: Path, args: argparse.Namespace) -> np.ndarray:
    """读取目标 scan PCD，并按当前恢复算法的观测范围裁剪。"""

    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points, dtype=np.float64)
    radius = np.linalg.norm(points[:, :3], axis=1)
    valid = (
        np.isfinite(points).all(axis=1)
        & (radius >= args.min_range)
        & (radius <= args.max_radius)
        & (points[:, 2] >= args.scan_min_z)
        & (points[:, 2] <= args.scan_max_z)
    )
    return points[valid]


def crop_map_cloud_for_pose(map_points: np.ndarray, x: float, y: float, args: argparse.Namespace) -> o3d.geometry.PointCloud:
    """按候选位置裁剪局部地图给 GICP，避免全图重复结构和计算量同时过大。"""

    valid = (
        (map_points[:, 0] >= x - args.gicp_crop_radius)
        & (map_points[:, 0] <= x + args.gicp_crop_radius)
        & (map_points[:, 1] >= y - args.gicp_crop_radius)
        & (map_points[:, 1] <= y + args.gicp_crop_radius)
    )
    return make_o3d_cloud(map_points[valid], args.map_voxel_size)


def run_gicp(
    scan_cloud: o3d.geometry.PointCloud,
    map_cloud: o3d.geometry.PointCloud,
    init: np.ndarray,
    args: argparse.Namespace,
) -> o3d.pipelines.registration.RegistrationResult:
    """执行 Open3D Generalized ICP 精修。"""

    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-6,
        relative_rmse=1e-6,
        max_iteration=args.gicp_iterations,
    )
    return o3d.pipelines.registration.registration_generalized_icp(
        scan_cloud,
        map_cloud,
        args.max_correspondence_distance,
        init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria,
    )


def evaluate_target(
    target: TargetSample,
    map_points: np.ndarray,
    database: VirtualDatabase,
    args: argparse.Namespace,
) -> tuple[dict[str, str], list[dict[str, str]]]:
    """对单个目标执行虚拟 LiDAR 召回、MCL 重排和 GICP 精修。"""

    start = time.perf_counter()
    scan_points = load_scan_points(target.pcd_path, args)
    query_ranges = query_ranges_from_points(scan_points, args)
    query_descriptor = descriptor_from_ranges(query_ranges, args)
    retrieved = retrieve_virtual_candidates(query_descriptor, database, args)
    verified = mcl_verify_candidates(query_ranges, retrieved, database, args)
    scan_cloud = make_o3d_cloud(scan_points, args.scan_voxel_size)

    candidate_rows: list[dict[str, str]] = []
    refined: list[dict[str, float | int]] = []
    for rank, item in enumerate(verified, start=1):
        for yaw_offset in args.yaw_offsets:
            init_yaw = normalize_angle_deg(float(item["yaw_deg"]) + float(yaw_offset))
            init = pose_matrix(float(item["x"]), float(item["y"]), init_yaw)
            map_cloud = crop_map_cloud_for_pose(map_points, float(item["x"]), float(item["y"]), args)
            result = run_gicp(scan_cloud, map_cloud, init, args)
            pose = result.transformation
            final_x = float(pose[0, 3])
            final_y = float(pose[1, 3])
            final_yaw = yaw_from_matrix(pose)
            trans_error = math.hypot(final_x - target.reference_x, final_y - target.reference_y)
            yaw_error = abs(normalize_angle_deg(final_yaw - target.reference_yaw_deg))
            seed_drift_xy = math.hypot(final_x - float(item["x"]), final_y - float(item["y"]))
            seed_drift_yaw = abs(normalize_angle_deg(final_yaw - init_yaw))
            seed_drift_penalty = 0.0
            if seed_drift_xy > args.max_gicp_seed_drift_m:
                seed_drift_penalty += args.seed_drift_weight * (seed_drift_xy - args.max_gicp_seed_drift_m)
            if seed_drift_yaw > args.max_gicp_seed_drift_yaw_deg:
                seed_drift_penalty += args.seed_drift_weight * (
                    (seed_drift_yaw - args.max_gicp_seed_drift_yaw_deg) / max(1.0, args.max_gicp_seed_drift_yaw_deg)
                )
            score = (
                -float(result.fitness)
                + args.rmse_weight * float(result.inlier_rmse)
                + args.ray_error_weight * float(item["ray_mean_error"])
                + args.wall_weight * float(item["wall_ratio"])
                + args.descriptor_weight * float(item["descriptor_distance"])
                + seed_drift_penalty
            )
            refined.append(
                {
                    "rank": rank,
                    "score": score,
                    "fitness": float(result.fitness),
                    "inlier_rmse": float(result.inlier_rmse),
                    "final_x": final_x,
                    "final_y": final_y,
                    "final_yaw": final_yaw,
                    "trans_error": trans_error,
                    "yaw_error": yaw_error,
                    "seed_drift_xy": seed_drift_xy,
                    "seed_drift_yaw": seed_drift_yaw,
                    "yaw_offset": float(yaw_offset),
                    **item,
                }
            )
            candidate_rows.append(
                {
                    "target_id": target.target_id,
                    "cloud_index": str(target.cloud_index),
                    "mcl_rank": str(rank),
                    "db_index": str(int(item["db_index"])),
                    "seed_x_m": f"{float(item['x']):.6f}",
                    "seed_y_m": f"{float(item['y']):.6f}",
                    "seed_yaw_deg": f"{float(item['yaw_deg']):.6f}",
                    "yaw_offset_deg": f"{float(yaw_offset):.3f}",
                    "descriptor_distance": f"{float(item['descriptor_distance']):.6f}",
                    "ray_likelihood": f"{float(item['ray_likelihood']):.6f}",
                    "ray_mean_error_m": f"{float(item['ray_mean_error']):.6f}",
                    "wall_ratio": f"{float(item['wall_ratio']):.6f}",
                    "gicp_fitness": f"{float(result.fitness):.6f}",
                    "gicp_rmse": f"{float(result.inlier_rmse):.6f}",
                    "final_x_m": f"{final_x:.6f}",
                    "final_y_m": f"{final_y:.6f}",
                    "final_yaw_deg": f"{final_yaw:.6f}",
                    "translation_error_m": f"{trans_error:.6f}",
                    "yaw_error_deg": f"{yaw_error:.6f}",
                    "seed_drift_xy_m": f"{seed_drift_xy:.6f}",
                    "seed_drift_yaw_deg": f"{seed_drift_yaw:.6f}",
                    "score": f"{score:.6f}",
                }
            )

    refined.sort(key=lambda item: (float(item["score"]), -float(item["fitness"]), float(item["inlier_rmse"])))
    best = refined[0] if refined else {}
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    row = {
        "target_id": target.target_id,
        "cloud_index": str(target.cloud_index),
        "reference_x_m": f"{target.reference_x:.6f}",
        "reference_y_m": f"{target.reference_y:.6f}",
        "reference_yaw_deg": f"{target.reference_yaw_deg:.6f}",
        "selected_rank": str(int(best.get("rank", 0))),
        "selected_x_m": f"{float(best.get('final_x', float('nan'))):.6f}",
        "selected_y_m": f"{float(best.get('final_y', float('nan'))):.6f}",
        "selected_yaw_deg": f"{float(best.get('final_yaw', float('nan'))):.6f}",
        "translation_error_m": f"{float(best.get('trans_error', float('inf'))):.6f}",
        "yaw_error_deg": f"{float(best.get('yaw_error', float('inf'))):.6f}",
        "gicp_fitness": f"{float(best.get('fitness', 0.0)):.6f}",
        "gicp_rmse": f"{float(best.get('inlier_rmse', float('inf'))):.6f}",
        "ray_mean_error_m": f"{float(best.get('ray_mean_error', float('inf'))):.6f}",
        "wall_ratio": f"{float(best.get('wall_ratio', 1.0)):.6f}",
        "seed_drift_xy_m": f"{float(best.get('seed_drift_xy', float('inf'))):.6f}",
        "seed_drift_yaw_deg": f"{float(best.get('seed_drift_yaw', float('inf'))):.6f}",
        "success_0p2_3deg": "1" if float(best.get("trans_error", float("inf"))) <= 0.2 and float(best.get("yaw_error", float("inf"))) <= 3.0 else "0",
        "success_0p3_5deg": "1" if float(best.get("trans_error", float("inf"))) <= 0.3 and float(best.get("yaw_error", float("inf"))) <= 5.0 else "0",
        "success_0p5_10deg": "1" if float(best.get("trans_error", float("inf"))) <= 0.5 and float(best.get("yaw_error", float("inf"))) <= 10.0 else "0",
        "elapsed_ms": f"{elapsed_ms:.3f}",
    }
    return row, candidate_rows


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """写出 CSV；空结果时仍创建父目录，便于脚本化检查。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    """解析命令行参数，默认值按 bag46 当前地图和严格离线验证设置。"""

    parser = argparse.ArgumentParser(description="Validate virtual LiDAR database relocalization.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--targets-csv", type=Path, required=True, help="目标 PCD CSV，包含 x/y/yaw/pcd")
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"), help="全局地图 PCD")
    parser.add_argument("--output", type=Path, required=True, help="目标级验证结果 CSV")
    parser.add_argument("--candidate-output", type=Path, default=None, help="候选级验证明细 CSV")
    parser.add_argument("--max-targets", type=int, default=0, help="只验证前 N 个目标；0 表示全量")
    parser.add_argument("--database-cache", type=Path, default=Path(".codex_tmp/virtual_lidar/virtual_db_grid1p0_sec120.npz"), help="虚拟 scan 数据库缓存")
    parser.add_argument("--rebuild-database", action="store_true", help="忽略缓存并重建虚拟 scan 数据库")
    parser.add_argument("--candidate-grid-size", type=float, default=1.0, help="虚拟站位 XY 栅格尺寸")
    parser.add_argument("--candidate-margin", type=float, default=0.0, help="地图包围盒额外候选边界")
    parser.add_argument("--occupancy-resolution", type=float, default=0.18, help="禁穿墙/射线模拟占据栅格分辨率")
    parser.add_argument("--robot-clearance-radius", type=float, default=0.28, help="候选站位最小清障半径")
    parser.add_argument("--map-min-z", type=float, default=0.25, help="用于障碍地图的最低高度")
    parser.add_argument("--map-max-z", type=float, default=2.2, help="用于障碍地图的最高高度")
    parser.add_argument("--scan-min-z", type=float, default=0.2, help="用于真实 scan 描述子的最低高度")
    parser.add_argument("--scan-max-z", type=float, default=2.5, help="用于真实 scan 描述子的最高高度")
    parser.add_argument("--min-range", type=float, default=0.6, help="忽略机器人近场点")
    parser.add_argument("--max-radius", type=float, default=18.0, help="虚拟 LiDAR 和 descriptor 最大半径")
    parser.add_argument("--ray-step", type=float, default=0.18, help="虚拟 LiDAR 射线步长")
    parser.add_argument("--sectors", type=int, default=120, help="水平射线/描述子扇区数")
    parser.add_argument("--rings", type=int, default=24, help="range descriptor 环数")
    parser.add_argument("--min-virtual-hits", type=int, default=24, help="候选位置最少可见射线命中数")
    parser.add_argument("--descriptor-top-k", type=int, default=80, help="描述子召回候选数量")
    parser.add_argument("--descriptor-sigma", type=float, default=0.35, help="描述子距离转粒子权重的尺度")
    parser.add_argument("--mcl-seed-top-k", type=int, default=30, help="进入 MCL 的描述子候选数量")
    parser.add_argument("--particles-per-seed", type=int, default=8, help="每个描述子候选生成的粒子数")
    parser.add_argument("--mcl-iterations", type=int, default=3, help="静止 MCL 重采样迭代次数")
    parser.add_argument("--particle-xy-std-m", type=float, default=0.18, help="粒子 XY 抖动标准差")
    parser.add_argument("--particle-yaw-std-deg", type=float, default=4.0, help="粒子 yaw 抖动标准差")
    parser.add_argument("--range-sigma", type=float, default=1.0, help="可见性 range 误差转似然尺度")
    parser.add_argument("--wall-penetration-tolerance", type=float, default=0.8, help="真实观测比虚拟墙面更远多少视为穿墙")
    parser.add_argument("--duplicate-xy-m", type=float, default=0.35, help="MCL 输出候选 XY 去重阈值")
    parser.add_argument("--duplicate-yaw-deg", type=float, default=8.0, help="MCL 输出候选 yaw 去重阈值")
    parser.add_argument("--refine-top-k", type=int, default=8, help="GICP 精修的 MCL 候选数量")
    parser.add_argument("--yaw-offsets", nargs="*", type=float, default=[-15.0, -9.0, -3.0, 0.0, 3.0, 9.0, 15.0], help="每个 MCL 候选额外 yaw 微调")
    parser.add_argument("--scan-voxel-size", type=float, default=0.25, help="GICP source scan 降采样")
    parser.add_argument("--map-voxel-size", type=float, default=0.30, help="GICP target map patch 降采样")
    parser.add_argument("--gicp-crop-radius", type=float, default=12.0, help="每个候选 GICP 局部地图裁剪半径")
    parser.add_argument("--max-correspondence-distance", type=float, default=1.5, help="GICP 最大对应距离")
    parser.add_argument("--gicp-iterations", type=int, default=30, help="GICP 最大迭代次数")
    parser.add_argument("--rmse-weight", type=float, default=0.4, help="最终候选排序中 RMSE 权重")
    parser.add_argument("--ray-error-weight", type=float, default=0.15, help="最终候选排序中可见性 range 误差权重")
    parser.add_argument("--wall-weight", type=float, default=1.0, help="最终候选排序中穿墙比例权重")
    parser.add_argument("--descriptor-weight", type=float, default=0.25, help="最终候选排序中描述子距离权重")
    parser.add_argument("--max-gicp-seed-drift-m", type=float, default=2.0, help="GICP 精修允许偏离 seed 的最大 XY 距离，超过后加惩罚")
    parser.add_argument("--max-gicp-seed-drift-yaw-deg", type=float, default=35.0, help="GICP 精修允许偏离 seed 的最大 yaw，超过后加惩罚")
    parser.add_argument("--seed-drift-weight", type=float, default=2.0, help="GICP seed 漂移惩罚权重")
    parser.add_argument("--random-seed", type=int, default=20260706, help="MCL 粒子随机种子")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    workspace = args.workspace.resolve()
    args.targets_csv = args.targets_csv if args.targets_csv.is_absolute() else workspace / args.targets_csv
    args.map = args.map if args.map.is_absolute() else workspace / args.map
    args.output = args.output if args.output.is_absolute() else workspace / args.output
    if args.candidate_output is None:
        args.candidate_output = args.output.with_name(args.output.stem + "_candidates.csv")
    else:
        args.candidate_output = args.candidate_output if args.candidate_output.is_absolute() else workspace / args.candidate_output
    args.database_cache = args.database_cache if args.database_cache.is_absolute() else workspace / args.database_cache

    start = time.perf_counter()
    targets = limit_targets(read_targets(args.targets_csv), args.max_targets)
    map_points = load_map_points(args.map, args)
    grid = build_occupancy_grid(map_points, args)
    database = build_virtual_database(map_points, grid, args)
    print(
        f"[virtual_lidar] loaded targets={len(targets)} map_points={len(map_points)} "
        f"db_candidates={len(database.candidates_xy)} rss_mb={current_rss_mb():.1f}",
        flush=True,
    )

    rows: list[dict[str, str]] = []
    candidate_rows: list[dict[str, str]] = []
    for index, target in enumerate(targets, start=1):
        row, candidates = evaluate_target(target, map_points, database, args)
        rows.append(row)
        candidate_rows.extend(candidates)
        print(
            f"[virtual_lidar] {index}/{len(targets)} idx={target.cloud_index} "
            f"err={row['translation_error_m']}m/{row['yaw_error_deg']}deg "
            f"fitness={row['gicp_fitness']} wall={row['wall_ratio']} "
            f"success03={row['success_0p3_5deg']}",
            flush=True,
        )

    write_csv(args.output, rows)
    write_csv(args.candidate_output, candidate_rows)
    elapsed = time.perf_counter() - start
    for key in ["success_0p2_3deg", "success_0p3_5deg", "success_0p5_10deg"]:
        count = sum(int(row[key]) for row in rows)
        print(f"[virtual_lidar] {key}={count}/{len(rows)} ({count / max(1, len(rows)) * 100.0:.1f}%)")
    print(f"[virtual_lidar] wrote {args.output}")
    print(f"[virtual_lidar] wrote {args.candidate_output}")
    print(f"[virtual_lidar] elapsed_sec={elapsed:.2f} peak_rss_mb={current_rss_mb():.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
