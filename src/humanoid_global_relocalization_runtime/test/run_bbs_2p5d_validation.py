#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_bbs_2p5d_validation.py

文件作用：
  1. 离线验证“2D/2.5D Branch-and-Bound Correlative Scan Matching + GICP 精修”是否适合当前室内全局重定位。
  2. 从全局 PCD 地图生成多高度层二维占据距离场，把单帧 3D scan 投影到相同高度层，在整个地图 x/y/yaw 空间做多分辨率全局搜索。
  3. 搜索得到 top-K map->base 候选后，再用 Open3D Generalized ICP 做 3D scan-to-map 精修，统计严格阈值成功率和资源占用。
  4. 输出每个 target 的最终结果、BBS 候选明细和参数 sweep 证据，用于判断是否值得移植到 C++ 运行态。

重要说明：
  - 该脚本是验证工具，不发布 TF，不注入 initialpose，也不修改线上导航系统。
  - 搜索不使用真值；真值只用于最后统计 0.2m/3deg、0.3m/5deg、0.5m/10deg 成功率。
  - 这里的 BBS 是面向上线实现的多分辨率分支搜索：先在粗栅格全图遍历，再逐层展开高分候选到细栅格。
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
import resource
import time

import numpy as np
import open3d as o3d
from scipy import ndimage


@dataclass(frozen=True)
class TargetSample:
    """一个待验证 scan 的真值位姿和 PCD 路径。"""

    target_id: str
    cloud_index: int
    reference_x: float
    reference_y: float
    reference_yaw_deg: float
    pcd_path: Path


@dataclass(frozen=True)
class HeightBand:
    """一个 2.5D 高度层范围。"""

    min_z: float
    max_z: float


@dataclass(frozen=True)
class BbsCandidate:
    """BBS 搜索中的一个 x/y/yaw 候选。"""

    ix: int
    iy: int
    yaw_deg: float
    score: float


@dataclass
class ScorePyramid:
    """一个高度层的多分辨率距离场金字塔。"""

    levels: dict[int, np.ndarray]


@dataclass
class BbsMap:
    """全局地图转换得到的 2.5D BBS 搜索结构。"""

    origin_x: float
    origin_y: float
    base_resolution: float
    factors: list[int]
    pyramids: list[ScorePyramid]
    occupied: np.ndarray
    map_cloud: o3d.geometry.PointCloud


def repo_root_from_script() -> Path:
    """根据脚本位置推断 humanoid_ws 根目录。"""

    return Path(__file__).resolve().parents[3]


def normalize_angle_deg(angle: float) -> float:
    """把角度规整到 [-180, 180]，方便 yaw 误差统计。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def current_rss_mb() -> float:
    """读取当前进程最大 RSS；Linux ru_maxrss 单位为 KB。"""

    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def current_cpu_seconds() -> float:
    """读取当前进程累计 user+system CPU 秒，用于换算平均核心占用。"""

    usage = resource.getrusage(resource.RUSAGE_SELF)
    return float(usage.ru_utime + usage.ru_stime)


def read_targets(path: Path, max_targets: int) -> list[TargetSample]:
    """读取 target CSV；max_targets 只用于 smoke，正式验证默认 0 表示全量。"""

    targets: list[TargetSample] = []
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if 0 < max_targets <= len(targets):
                break
            targets.append(
                TargetSample(
                    target_id=row.get("target_id", f"target_{row['cloud_index']}"),
                    cloud_index=int(row["cloud_index"]),
                    reference_x=float(row["x"]),
                    reference_y=float(row["y"]),
                    reference_yaw_deg=float(row["yaw_deg"]),
                    pcd_path=Path(row["pcd"]),
                )
            )
    return targets


def parse_height_bands(text: str) -> list[HeightBand]:
    """把形如 0.2:0.8,0.8:1.5 的参数解析为高度层列表。"""

    bands: list[HeightBand] = []
    for item in text.split(","):
        if not item.strip():
            continue
        begin, end = item.split(":", maxsplit=1)
        bands.append(HeightBand(float(begin), float(end)))
    if not bands:
        raise ValueError("height bands cannot be empty")
    return bands


def make_o3d_cloud(points: np.ndarray, voxel_size: float) -> o3d.geometry.PointCloud:
    """把 numpy 点集转换为 Open3D 点云并按需降采样。"""

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size)
    return cloud


def load_map_points(map_path: Path, args: argparse.Namespace) -> np.ndarray:
    """读取全局 PCD 地图，并按整体高度范围过滤。"""

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


def max_pool_grid(grid: np.ndarray, factor: int) -> np.ndarray:
    """对 score grid 做 factor x factor 最大池化，得到粗分辨率上界图。"""

    if factor == 1:
        return grid.astype(np.float32, copy=True)
    pad_y = (-grid.shape[0]) % factor
    pad_x = (-grid.shape[1]) % factor
    padded = np.pad(grid, ((0, pad_y), (0, pad_x)), mode="constant", constant_values=0.0)
    reshaped = padded.reshape(padded.shape[0] // factor, factor, padded.shape[1] // factor, factor)
    return reshaped.max(axis=(1, 3)).astype(np.float32)


def build_score_grid(
    points: np.ndarray,
    band: HeightBand,
    origin_x: float,
    origin_y: float,
    width: int,
    height: int,
    args: argparse.Namespace,
) -> np.ndarray:
    """为一个高度层构建二维占据距离得分图，越靠近地图障碍表面分数越高。"""

    band_points = points[(points[:, 2] >= band.min_z) & (points[:, 2] < band.max_z)]
    occupied = np.zeros((height, width), dtype=bool)
    if len(band_points) > 0:
        ix = np.floor((band_points[:, 0] - origin_x) / args.base_resolution).astype(np.int32)
        iy = np.floor((band_points[:, 1] - origin_y) / args.base_resolution).astype(np.int32)
        valid = (ix >= 0) & (iy >= 0) & (ix < width) & (iy < height)
        occupied[iy[valid], ix[valid]] = True
    distance_cells = ndimage.distance_transform_edt(~occupied).astype(np.float32)
    distance_m = distance_cells * float(args.base_resolution)
    score = np.exp(-(distance_m * distance_m) / (2.0 * args.distance_sigma * args.distance_sigma))
    score[occupied] = 1.0
    return score.astype(np.float32)


def build_bbs_map(map_path: Path, bands: list[HeightBand], args: argparse.Namespace) -> BbsMap:
    """从 PCD 地图构建 2.5D score 金字塔和 GICP 地图点云。"""

    points = load_map_points(map_path, args)
    margin = args.map_margin_m
    min_xy = points[:, :2].min(axis=0) - margin
    max_xy = points[:, :2].max(axis=0) + margin
    width = int(math.ceil((max_xy[0] - min_xy[0]) / args.base_resolution)) + 1
    height = int(math.ceil((max_xy[1] - min_xy[1]) / args.base_resolution)) + 1
    factors = [int(item) for item in args.pyramid_factors.split(",") if item.strip()]
    factors = sorted(set(factors), reverse=True)
    if factors[-1] != 1:
        factors.append(1)

    pyramids: list[ScorePyramid] = []
    occupied_any = np.zeros((height, width), dtype=bool)
    for band in bands:
        base_grid = build_score_grid(points, band, float(min_xy[0]), float(min_xy[1]), width, height, args)
        occupied_any |= base_grid >= 0.999
        pyramids.append(ScorePyramid({factor: max_pool_grid(base_grid, factor) for factor in factors}))

    map_cloud = make_o3d_cloud(points, args.gicp_map_voxel_size)
    return BbsMap(float(min_xy[0]), float(min_xy[1]), args.base_resolution, factors, pyramids, occupied_any, map_cloud)


def load_scan_points(path: Path, args: argparse.Namespace) -> np.ndarray:
    """读取目标 scan PCD，并按 BBS/GICP 共同使用的范围裁剪。"""

    cloud = o3d.io.read_point_cloud(str(path))
    if args.scan_voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(args.scan_voxel_size)
    points = np.asarray(cloud.points, dtype=np.float64)
    radius = np.linalg.norm(points[:, :2], axis=1)
    valid = (
        np.isfinite(points).all(axis=1)
        & (radius >= args.min_range)
        & (radius <= args.max_range)
        & (points[:, 2] >= args.scan_min_z)
        & (points[:, 2] <= args.scan_max_z)
    )
    return points[valid]


def split_scan_by_band(points: np.ndarray, bands: list[HeightBand], args: argparse.Namespace) -> list[np.ndarray]:
    """把 scan 点按高度层拆分，并限制每层最大点数，避免搜索过慢。"""

    result: list[np.ndarray] = []
    for index, band in enumerate(bands):
        band_points = points[(points[:, 2] >= band.min_z) & (points[:, 2] < band.max_z)]
        if len(band_points) > args.max_scan_points_per_band:
            rng = np.random.default_rng(args.random_seed + index + len(points))
            chosen = rng.choice(len(band_points), args.max_scan_points_per_band, replace=False)
            band_points = band_points[chosen]
        result.append(band_points[:, :2].astype(np.float32))
    return result


def grid_indices(bbs_map: BbsMap, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """把 map 坐标批量转换为 base occupancy grid 索引。"""

    ix = np.floor((x - bbs_map.origin_x) / bbs_map.base_resolution).astype(np.int32)
    iy = np.floor((y - bbs_map.origin_y) / bbs_map.base_resolution).astype(np.int32)
    inside = (ix >= 0) & (iy >= 0) & (ix < bbs_map.occupied.shape[1]) & (iy < bbs_map.occupied.shape[0])
    return ix, iy, inside


def has_origin_clearance(bbs_map: BbsMap, x: float, y: float, radius_m: float) -> bool:
    """检查候选站位周围是否被地图障碍占据。"""

    cx = int(math.floor((x - bbs_map.origin_x) / bbs_map.base_resolution))
    cy = int(math.floor((y - bbs_map.origin_y) / bbs_map.base_resolution))
    radius_cells = int(math.ceil(radius_m / bbs_map.base_resolution))
    if cx < radius_cells or cy < radius_cells:
        return False
    if cx + radius_cells >= bbs_map.occupied.shape[1] or cy + radius_cells >= bbs_map.occupied.shape[0]:
        return False
    patch = bbs_map.occupied[cy - radius_cells : cy + radius_cells + 1, cx - radius_cells : cx + radius_cells + 1]
    return not bool(patch.any())


def transform_scan_xy(points_xy: np.ndarray, x: float, y: float, yaw_deg: float) -> np.ndarray:
    """把 scan XY 点按候选位姿转换到 map XY。"""

    rotated = rotate_points(points_xy, yaw_deg)
    transformed = rotated.copy()
    transformed[:, 0] += x
    transformed[:, 1] += y
    return transformed


def ray_wall_cross_ratio(
    bbs_map: BbsMap,
    scan_points: np.ndarray,
    x: float,
    y: float,
    yaw_deg: float,
    args: argparse.Namespace,
) -> float:
    """统计候选位姿解释当前 scan 时，有多少射线会在端点前提前撞到地图障碍。"""

    if len(scan_points) == 0:
        return 1.0
    points_xy = scan_points[:, :2]
    if len(points_xy) > args.visibility_ray_sample_points:
        rng = np.random.default_rng(args.random_seed + int(abs(x * 1000.0)) + int(abs(y * 1000.0)))
        indexes = rng.choice(len(points_xy), args.visibility_ray_sample_points, replace=False)
        points_xy = points_xy[indexes]
    endpoints = transform_scan_xy(points_xy.astype(np.float32), x, y, yaw_deg)
    crossed = 0
    valid_rays = 0
    origin = np.array([x, y], dtype=np.float64)
    for endpoint in endpoints:
        vector = endpoint.astype(np.float64) - origin
        length = float(np.linalg.norm(vector))
        if length <= args.min_range + args.wall_before_end_tolerance:
            continue
        valid_rays += 1
        check_length = max(args.min_range, length - args.wall_before_end_tolerance)
        steps = int(check_length / args.visibility_ray_step)
        if steps <= 0:
            continue
        ratios = np.linspace(args.min_range / length, check_length / length, steps, dtype=np.float64)
        sample_x = x + ratios * vector[0]
        sample_y = y + ratios * vector[1]
        ix, iy, inside = grid_indices(bbs_map, sample_x, sample_y)
        hit = np.zeros_like(inside, dtype=bool)
        hit[inside] = bbs_map.occupied[iy[inside], ix[inside]]
        if bool(hit.any()):
            crossed += 1
    if valid_rays == 0:
        return 1.0
    return float(crossed) / float(valid_rays)


def rotate_points(points_xy: np.ndarray, yaw_deg: float) -> np.ndarray:
    """把 base 坐标 scan 点按候选 yaw 旋转到 map 方向。"""

    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    rotated = np.empty_like(points_xy, dtype=np.float32)
    rotated[:, 0] = c * points_xy[:, 0] - s * points_xy[:, 1]
    rotated[:, 1] = s * points_xy[:, 0] + c * points_xy[:, 1]
    return rotated


def score_candidate_cells(
    bbs_map: BbsMap,
    scan_by_band: list[np.ndarray],
    yaw_deg: float,
    factor: int,
    candidate_ix: np.ndarray,
    candidate_iy: np.ndarray,
    args: argparse.Namespace,
) -> np.ndarray:
    """在某个分辨率层批量计算候选 x/y/yaw 的 correlative score。"""

    level_resolution = bbs_map.base_resolution * float(factor)
    total_scores = np.zeros(len(candidate_ix), dtype=np.float32)
    total_points = 0
    for band_index, points_xy in enumerate(scan_by_band):
        if len(points_xy) == 0:
            continue
        rotated = rotate_points(points_xy, yaw_deg)
        offsets_x = np.floor(rotated[:, 0] / level_resolution).astype(np.int32)
        offsets_y = np.floor(rotated[:, 1] / level_resolution).astype(np.int32)
        grid = bbs_map.pyramids[band_index].levels[factor]
        band_score_sum = np.zeros(len(candidate_ix), dtype=np.float32)
        for begin in range(0, len(candidate_ix), args.score_batch_size):
            end = min(begin + args.score_batch_size, len(candidate_ix))
            xs = candidate_ix[begin:end, None] + offsets_x[None, :]
            ys = candidate_iy[begin:end, None] + offsets_y[None, :]
            inside = (xs >= 0) & (ys >= 0) & (xs < grid.shape[1]) & (ys < grid.shape[0])
            values = np.zeros(xs.shape, dtype=np.float32)
            values[inside] = grid[ys[inside], xs[inside]]
            band_score_sum[begin:end] = values.sum(axis=1)
        total_scores += band_score_sum
        total_points += len(points_xy)
    if total_points <= 0:
        return total_scores
    return total_scores / float(total_points)


def keep_top_candidates(
    candidates: list[BbsCandidate],
    keep: int,
    nms_xy_cells: float = 0.0,
    nms_yaw_deg: float = 0.0,
) -> list[BbsCandidate]:
    """按 score 保留 top-K 候选；可选启用空间/角度 NMS 保持候选多样性。"""

    if len(candidates) <= keep:
        return sorted(candidates, key=lambda item: item.score, reverse=True)
    candidates.sort(key=lambda item: item.score, reverse=True)
    unique: list[BbsCandidate] = []
    seen: set[tuple[int, int, int]] = set()
    for item in candidates:
        yaw_key = int(round(item.yaw_deg * 100.0))
        key = (item.ix, item.iy, yaw_key)
        if key in seen:
            continue
        if nms_xy_cells > 0.0 and nms_yaw_deg > 0.0:
            too_close = False
            for selected in unique:
                xy_cells = math.hypot(float(item.ix - selected.ix), float(item.iy - selected.iy))
                yaw_gap = abs(normalize_angle_deg(item.yaw_deg - selected.yaw_deg))
                if xy_cells < nms_xy_cells and yaw_gap < nms_yaw_deg:
                    too_close = True
                    break
            if too_close:
                continue
        seen.add(key)
        unique.append(item)
        if len(unique) >= keep:
            break
    return unique


def initial_search(
    bbs_map: BbsMap,
    scan_by_band: list[np.ndarray],
    factor: int,
    yaw_values: list[float],
    args: argparse.Namespace,
) -> list[BbsCandidate]:
    """在最粗层对全图 x/y/yaw 做完整遍历。"""

    grid = bbs_map.pyramids[0].levels[factor]
    iy, ix = np.indices(grid.shape, dtype=np.int32)
    flat_ix = ix.reshape(-1)
    flat_iy = iy.reshape(-1)
    all_candidates: list[BbsCandidate] = []
    for yaw in yaw_values:
        scores = score_candidate_cells(bbs_map, scan_by_band, yaw, factor, flat_ix, flat_iy, args)
        if len(scores) == 0:
            continue
        keep = min(args.level_keep_candidates, len(scores))
        top_indices = np.argpartition(scores, -keep)[-keep:]
        for index in top_indices:
            all_candidates.append(BbsCandidate(int(flat_ix[index]), int(flat_iy[index]), float(yaw), float(scores[index])))
    return keep_top_candidates(all_candidates, args.level_keep_candidates)


def refine_level(
    bbs_map: BbsMap,
    scan_by_band: list[np.ndarray],
    parent_factor: int,
    child_factor: int,
    parents: list[BbsCandidate],
    args: argparse.Namespace,
) -> list[BbsCandidate]:
    """把上一层候选展开到下一层子栅格，并重新评分保留 top-K。"""

    scale = parent_factor // child_factor
    child_grid = bbs_map.pyramids[0].levels[child_factor]
    child_candidates: list[tuple[int, int, float]] = []
    for parent in parents:
        base_ix = parent.ix * scale
        base_iy = parent.iy * scale
        for dx in range(scale):
            for dy in range(scale):
                ix = base_ix + dx
                iy = base_iy + dy
                if 0 <= ix < child_grid.shape[1] and 0 <= iy < child_grid.shape[0]:
                    child_candidates.append((ix, iy, parent.yaw_deg))
    if not child_candidates:
        return []

    grouped: dict[float, tuple[list[int], list[int]]] = {}
    for ix, iy, yaw in child_candidates:
        if yaw not in grouped:
            grouped[yaw] = ([], [])
        grouped[yaw][0].append(ix)
        grouped[yaw][1].append(iy)

    scored: list[BbsCandidate] = []
    for yaw, (ixs, iys) in grouped.items():
        ix_arr = np.asarray(ixs, dtype=np.int32)
        iy_arr = np.asarray(iys, dtype=np.int32)
        scores = score_candidate_cells(bbs_map, scan_by_band, yaw, child_factor, ix_arr, iy_arr, args)
        for index, score in enumerate(scores):
            scored.append(BbsCandidate(int(ix_arr[index]), int(iy_arr[index]), float(yaw), float(score)))
    return keep_top_candidates(scored, args.level_keep_candidates)


def final_yaw_refinement(
    bbs_map: BbsMap,
    scan_by_band: list[np.ndarray],
    candidates: list[BbsCandidate],
    args: argparse.Namespace,
) -> list[BbsCandidate]:
    """在最终细栅格上对 top 候选做局部 yaw 加密评分。"""

    if args.final_yaw_step_deg <= 0.0 or args.final_yaw_radius_deg <= 0.0:
        return keep_top_candidates(candidates, args.bbs_top_candidates)
    variants: list[tuple[int, int, float]] = []
    offsets = np.arange(-args.final_yaw_radius_deg, args.final_yaw_radius_deg + 1e-6, args.final_yaw_step_deg)
    for item in candidates[: args.final_yaw_seed_candidates]:
        for offset in offsets:
            variants.append((item.ix, item.iy, normalize_angle_deg(item.yaw_deg + float(offset))))

    grouped: dict[float, tuple[list[int], list[int]]] = {}
    for ix, iy, yaw in variants:
        if yaw not in grouped:
            grouped[yaw] = ([], [])
        grouped[yaw][0].append(ix)
        grouped[yaw][1].append(iy)

    scored: list[BbsCandidate] = []
    for yaw, (ixs, iys) in grouped.items():
        ix_arr = np.asarray(ixs, dtype=np.int32)
        iy_arr = np.asarray(iys, dtype=np.int32)
        scores = score_candidate_cells(bbs_map, scan_by_band, yaw, 1, ix_arr, iy_arr, args)
        for index, score in enumerate(scores):
            scored.append(BbsCandidate(int(ix_arr[index]), int(iy_arr[index]), float(yaw), float(score)))
    nms_xy_cells = args.final_nms_xy_m / max(bbs_map.base_resolution, 1e-6)
    return keep_top_candidates(scored, args.bbs_top_candidates, nms_xy_cells, args.final_nms_yaw_deg)


def run_bbs_search(bbs_map: BbsMap, scan_by_band: list[np.ndarray], args: argparse.Namespace) -> list[BbsCandidate]:
    """执行完整多层 BBS 搜索，返回最终 top 候选。"""

    yaw_values = list(np.arange(-180.0, 180.0, args.yaw_step_deg, dtype=np.float64))
    factors = bbs_map.factors
    candidates = initial_search(bbs_map, scan_by_band, factors[0], yaw_values, args)
    for parent_factor, child_factor in zip(factors[:-1], factors[1:]):
        candidates = refine_level(bbs_map, scan_by_band, parent_factor, child_factor, candidates, args)
    return final_yaw_refinement(bbs_map, scan_by_band, candidates, args)


def pose_matrix(x: float, y: float, yaw_deg: float) -> np.ndarray:
    """根据 x/y/yaw 构造 scan(base)->map 初始变换矩阵。"""

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


def yaw_from_matrix(matrix: np.ndarray) -> float:
    """从 4x4 变换矩阵读取 yaw，单位 deg。"""

    return math.degrees(math.atan2(float(matrix[1, 0]), float(matrix[0, 0])))


def run_gicp(
    scan_cloud: o3d.geometry.PointCloud,
    map_cloud: o3d.geometry.PointCloud,
    init: np.ndarray,
    args: argparse.Namespace,
) -> o3d.pipelines.registration.RegistrationResult:
    """执行一次 Open3D Generalized ICP 精修。"""

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


def candidate_to_pose(bbs_map: BbsMap, candidate: BbsCandidate) -> tuple[float, float, float]:
    """把最终细栅格候选转换成 map 坐标。"""

    x = bbs_map.origin_x + candidate.ix * bbs_map.base_resolution
    y = bbs_map.origin_y + candidate.iy * bbs_map.base_resolution
    return float(x), float(y), float(candidate.yaw_deg)


def evaluate_target(
    target: TargetSample,
    bbs_map: BbsMap,
    bands: list[HeightBand],
    args: argparse.Namespace,
) -> tuple[dict[str, str], list[dict[str, str]]]:
    """对一个 target 执行 BBS 搜索、GICP 精修和误差统计。"""

    begin = time.perf_counter()
    scan_points = load_scan_points(target.pcd_path, args)
    scan_by_band = split_scan_by_band(scan_points, bands, args)
    bbs_candidates = run_bbs_search(bbs_map, scan_by_band, args)
    bbs_ms = (time.perf_counter() - begin) * 1000.0

    scan_cloud = make_o3d_cloud(scan_points, args.gicp_scan_voxel_size)
    refined_rows: list[dict[str, str]] = []
    best_row: dict[str, str] | None = None
    best_score = -1.0
    for rank, candidate in enumerate(bbs_candidates[: args.gicp_refine_candidates], start=1):
        seed_x, seed_y, seed_yaw = candidate_to_pose(bbs_map, candidate)
        origin_clear = has_origin_clearance(bbs_map, seed_x, seed_y, args.origin_clearance_radius)
        wall_ratio = ray_wall_cross_ratio(bbs_map, scan_points, seed_x, seed_y, seed_yaw, args)
        result = run_gicp(scan_cloud, bbs_map.map_cloud, pose_matrix(seed_x, seed_y, seed_yaw), args)
        pose = result.transformation
        final_x = float(pose[0, 3])
        final_y = float(pose[1, 3])
        final_yaw = yaw_from_matrix(pose)
        xy_error = math.hypot(final_x - target.reference_x, final_y - target.reference_y)
        yaw_error = abs(normalize_angle_deg(final_yaw - target.reference_yaw_deg))
        seed_xy_error = math.hypot(seed_x - target.reference_x, seed_y - target.reference_y)
        seed_yaw_error = abs(normalize_angle_deg(seed_yaw - target.reference_yaw_deg))
        accept_score = (
            candidate.score
            - args.wall_cross_penalty_weight * wall_ratio
            + (args.origin_clear_bonus if origin_clear else 0.0)
            + args.gicp_fitness_weight * float(result.fitness)
            - args.gicp_rmse_weight * float(result.inlier_rmse)
        )
        row = {
            "target_id": target.target_id,
            "target_index": str(target.cloud_index),
            "candidate_rank": str(rank),
            "bbs_score": f"{candidate.score:.9f}",
            "seed_x_m": f"{seed_x:.6f}",
            "seed_y_m": f"{seed_y:.6f}",
            "seed_yaw_deg": f"{seed_yaw:.6f}",
            "seed_translation_error_m": f"{seed_xy_error:.6f}",
            "seed_yaw_error_deg": f"{seed_yaw_error:.6f}",
            "origin_clear": "1" if origin_clear else "0",
            "ray_wall_cross_ratio": f"{wall_ratio:.9f}",
            "gicp_fitness": f"{float(result.fitness):.9f}",
            "gicp_rmse": f"{float(result.inlier_rmse):.9f}",
            "accept_score": f"{accept_score:.9f}",
            "final_x_m": f"{final_x:.6f}",
            "final_y_m": f"{final_y:.6f}",
            "final_yaw_deg": f"{final_yaw:.6f}",
            "reference_x_m": f"{target.reference_x:.6f}",
            "reference_y_m": f"{target.reference_y:.6f}",
            "reference_yaw_deg": f"{target.reference_yaw_deg:.6f}",
            "translation_error_m": f"{xy_error:.6f}",
            "yaw_error_deg": f"{yaw_error:.6f}",
            "success_0p2m_3deg": "1" if xy_error <= 0.2 and yaw_error <= 3.0 else "0",
            "success_0p3m_5deg": "1" if xy_error <= 0.3 and yaw_error <= 5.0 else "0",
            "success_0p5m_10deg": "1" if xy_error <= 0.5 and yaw_error <= 10.0 else "0",
        }
        refined_rows.append(row)
        if best_row is None or accept_score > best_score:
            best_score = accept_score
            best_row = row

    if best_row is None:
        raise RuntimeError(f"target {target.cloud_index} has no refined BBS candidates")

    total_ms = (time.perf_counter() - begin) * 1000.0
    summary = dict(best_row)
    summary.update(
        {
            "bbs_candidates": str(len(bbs_candidates)),
            "refined_candidates": str(len(refined_rows)),
            "scan_points": str(len(scan_points)),
            "band_points": ";".join(str(len(item)) for item in scan_by_band),
            "bbs_ms": f"{bbs_ms:.3f}",
            "total_ms": f"{total_ms:.3f}",
            "rss_mb": f"{current_rss_mb():.3f}",
        }
    )
    return summary, refined_rows


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """写出 CSV；字段顺序沿用第一行。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    """解析命令行参数，默认跑 bag46 hard52 全量。"""

    root = repo_root_from_script()
    parser = argparse.ArgumentParser(description="验证 2.5D BBS 全局重定位效果。")
    parser.add_argument("--workspace", type=Path, default=root, help="humanoid_ws 工作空间路径")
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"), help="全局 PCD 地图")
    parser.add_argument("--targets", type=Path, default=Path(".codex_tmp/official_std_standalone/nav46_hard52_dataset_acc10_fullz/targets.csv"), help="target PCD CSV")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/bbs_2p5d/hard52_summary.csv"), help="summary CSV 输出")
    parser.add_argument("--candidate-output", type=Path, default=Path(".codex_tmp/bbs_2p5d/hard52_candidates.csv"), help="候选明细 CSV 输出")
    parser.add_argument("--max-targets", type=int, default=0, help="只验证前 N 个 target；0 表示全量")
    parser.add_argument("--random-seed", type=int, default=17, help="确定性采样随机种子")

    parser.add_argument("--height-bands", default="0.2:0.8,0.8:1.5,1.5:2.2", help="2.5D 高度层，格式 min:max,min:max")
    parser.add_argument("--base-resolution", type=float, default=0.20, help="最终 BBS 栅格分辨率")
    parser.add_argument("--pyramid-factors", default="8,4,2,1", help="从粗到细的分辨率因子")
    parser.add_argument("--distance-sigma", type=float, default=0.35, help="距离场得分的高斯 sigma")
    parser.add_argument("--map-margin-m", type=float, default=1.0, help="地图搜索边界外扩距离")
    parser.add_argument("--map-voxel-size", type=float, default=0.12, help="构建 2.5D 地图前的 PCD 降采样尺寸")
    parser.add_argument("--map-min-z", type=float, default=0.15, help="地图整体最低高度")
    parser.add_argument("--map-max-z", type=float, default=2.3, help="地图整体最高高度")

    parser.add_argument("--scan-voxel-size", type=float, default=0.18, help="BBS 输入 scan 降采样尺寸")
    parser.add_argument("--min-range", type=float, default=0.35, help="忽略近场点")
    parser.add_argument("--max-range", type=float, default=18.0, help="scan 最大半径")
    parser.add_argument("--scan-min-z", type=float, default=-1.0, help="scan 最低高度")
    parser.add_argument("--scan-max-z", type=float, default=2.3, help="scan 最高高度")
    parser.add_argument("--max-scan-points-per-band", type=int, default=220, help="每个高度层最多参与 BBS 的点数")

    parser.add_argument("--yaw-step-deg", type=float, default=5.0, help="全局 yaw 搜索步长")
    parser.add_argument("--level-keep-candidates", type=int, default=12000, help="每个金字塔层保留的候选数")
    parser.add_argument("--bbs-top-candidates", type=int, default=80, help="BBS 最终保留候选数")
    parser.add_argument("--final-yaw-seed-candidates", type=int, default=1200, help="最终 yaw 加密使用的候选数")
    parser.add_argument("--final-yaw-radius-deg", type=float, default=5.0, help="最终 yaw 加密半径")
    parser.add_argument("--final-yaw-step-deg", type=float, default=2.5, help="最终 yaw 加密步长")
    parser.add_argument("--final-nms-xy-m", type=float, default=2.0, help="最终候选空间 NMS 半径")
    parser.add_argument("--final-nms-yaw-deg", type=float, default=30.0, help="最终候选 yaw NMS 半径")
    parser.add_argument("--score-batch-size", type=int, default=512, help="BBS 批量评分候选数")

    parser.add_argument("--origin-clearance-radius", type=float, default=0.25, help="最终候选站位清空检查半径")
    parser.add_argument("--visibility-ray-sample-points", type=int, default=160, help="最终候选禁穿墙射线采样点数")
    parser.add_argument("--visibility-ray-step", type=float, default=0.22, help="最终候选禁穿墙射线采样步长")
    parser.add_argument("--wall-before-end-tolerance", type=float, default=0.70, help="端点前该距离内命中地图不算提前撞墙")
    parser.add_argument("--wall-cross-penalty-weight", type=float, default=0.70, help="最终接受分数中的穿墙比例惩罚")
    parser.add_argument("--origin-clear-bonus", type=float, default=0.05, help="最终接受分数中的站位清空奖励")

    parser.add_argument("--gicp-refine-candidates", type=int, default=16, help="参与 GICP 精修的 BBS top 候选数")
    parser.add_argument("--gicp-map-voxel-size", type=float, default=0.30, help="GICP 地图体素尺寸")
    parser.add_argument("--gicp-scan-voxel-size", type=float, default=0.25, help="GICP scan 体素尺寸")
    parser.add_argument("--max-correspondence-distance", type=float, default=1.5, help="GICP 最大对应距离")
    parser.add_argument("--gicp-iterations", type=int, default=25, help="GICP 最大迭代次数")
    parser.add_argument("--gicp-fitness-weight", type=float, default=0.0, help="最终接受分数中的 GICP fitness 权重")
    parser.add_argument("--gicp-rmse-weight", type=float, default=0.0, help="最终接受分数中的 GICP rmse 惩罚权重")
    return parser.parse_args()


def main() -> int:
    """脚本入口：构建地图、逐 target 验证、写 CSV 和打印摘要。"""

    args = parse_args()
    workspace = args.workspace.resolve()
    map_path = args.map if args.map.is_absolute() else workspace / args.map
    targets_path = args.targets if args.targets.is_absolute() else workspace / args.targets
    output = args.output if args.output.is_absolute() else workspace / args.output
    candidate_output = args.candidate_output if args.candidate_output.is_absolute() else workspace / args.candidate_output

    start_time = time.time()
    start_cpu = current_cpu_seconds()
    bands = parse_height_bands(args.height_bands)
    bbs_map = build_bbs_map(map_path, bands, args)
    targets = read_targets(targets_path, args.max_targets)

    summary_rows: list[dict[str, str]] = []
    candidate_rows: list[dict[str, str]] = []
    for index, target in enumerate(targets, start=1):
        summary, candidates = evaluate_target(target, bbs_map, bands, args)
        summary_rows.append(summary)
        candidate_rows.extend(candidates)
        print(
            f"[bbs_2p5d] {index}/{len(targets)} idx={target.cloud_index} "
            f"err={summary['translation_error_m']}m/{summary['yaw_error_deg']}deg "
            f"score={summary['accept_score']} total_ms={summary['total_ms']}",
            flush=True,
        )

    write_csv(output, summary_rows)
    write_csv(candidate_output, candidate_rows)
    total = len(summary_rows)
    strict = sum(row["success_0p2m_3deg"] == "1" for row in summary_rows)
    practical = sum(row["success_0p3m_5deg"] == "1" for row in summary_rows)
    recoverable = sum(row["success_0p5m_10deg"] == "1" for row in summary_rows)
    elapsed = time.time() - start_time
    cpu_core_equiv = (current_cpu_seconds() - start_cpu) / max(elapsed, 1e-6)
    print(f"[bbs_2p5d] summary={output}")
    print(f"[bbs_2p5d] candidates={candidate_output}")
    print(f"[bbs_2p5d] success_0p2m_3deg={strict}/{total}")
    print(f"[bbs_2p5d] success_0p3m_5deg={practical}/{total}")
    print(f"[bbs_2p5d] success_0p5m_10deg={recoverable}/{total}")
    print(f"[bbs_2p5d] elapsed_sec={elapsed:.3f} cpu_core_equiv={cpu_core_equiv:.2f} rss_mb={current_rss_mb():.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
