#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""文件作用：完整离线复现两种面向有限视场 LiDAR 的全局重定位算法。

方法 synthetic：
  Offline-Online Hierarchical 3D Global Relocalization With Synthetic LiDAR
  Sensing and Descriptor-Space Retrieval 的地面机器人特化。离线从先验 PCD
  建可行站位、虚拟 LiDAR 首次命中扫描和描述子库；在线使用同构描述子检索，
  再按候选顺序执行 GN-ICP/GICP 验证。

方法 gprobe：
  G-PROBE 的完整前后端：虚拟视场分解、跨 FOV 分支检索、FOV-aware
  height/BKL 评分、gamma-SGRT、certainty map 和两阶段 CG-GICP。

本脚本只写测试产物，不发布 TF，也不改变导航状态。当前机器人强制 2D 导航，
因此候选采样限制在可行 XY 平面；描述子、certainty 和配准仍使用 3D 点云。
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
from scipy.ndimage import gaussian_filter1d


@dataclass(frozen=True)
class Target:
    target_id: str
    cloud_index: int
    x: float
    y: float
    yaw_deg: float
    pcd: Path


@dataclass(frozen=True)
class Keyframe:
    frame_id: int
    x: float
    y: float
    yaw_deg: float
    pcd: Path


@dataclass(frozen=True)
class MapGrid:
    origin_x: float
    origin_y: float
    resolution: float
    occupied: np.ndarray
    max_height: np.ndarray


@dataclass(frozen=True)
class VirtualDatabase:
    positions_xy: np.ndarray
    ranges: np.ndarray
    heights: np.ndarray


@dataclass(frozen=True)
class Bev:
    occupancy: np.ndarray
    uncertainty: np.ndarray
    max_height: np.ndarray
    fov_mask: np.ndarray


@dataclass(frozen=True)
class SensorPair:
    center_deg: float
    mask: np.ndarray


def repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def norm_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def angle_error_deg(a: float, b: float) -> float:
    return abs(norm_deg(a - b))


def pose_matrix(x: float, y: float, yaw_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    result = np.eye(4, dtype=np.float64)
    result[0, 0], result[0, 1] = c, -s
    result[1, 0], result[1, 1] = s, c
    result[0, 3], result[1, 3] = x, y
    return result


def pose_from_matrix(matrix: np.ndarray) -> tuple[float, float, float]:
    return (
        float(matrix[0, 3]),
        float(matrix[1, 3]),
        math.degrees(math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))),
    )


def rss_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def read_targets(path: Path, limit: int) -> list[Target]:
    rows: list[Target] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            rows.append(
                Target(
                    target_id=row.get("target_id", f"target_{len(rows):04d}"),
                    cloud_index=int(row["cloud_index"]),
                    x=float(row["x"]),
                    y=float(row["y"]),
                    yaw_deg=float(row["yaw_deg"]),
                    pcd=Path(row["pcd"]),
                )
            )
    return rows if limit <= 0 else rows[:limit]


def read_keyframes(path: Path) -> list[Keyframe]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [
            Keyframe(int(row["frame_id"]), float(row["x"]), float(row["y"]), float(row["yaw_deg"]), Path(row["pcd"]))
            for row in csv.DictReader(stream)
        ]


def load_points(path: Path) -> np.ndarray:
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points, dtype=np.float64)
    return points[np.isfinite(points).all(axis=1)]


def voxel_downsample(points: np.ndarray, size: float) -> np.ndarray:
    if len(points) == 0 or size <= 0.0:
        return points
    keys = np.floor(points / size).astype(np.int64)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(keep)]


def build_map_grid(points: np.ndarray, args: argparse.Namespace) -> MapGrid:
    selected = points[
        (points[:, 2] >= args.map_min_z) & (points[:, 2] <= args.map_max_z)
    ]
    margin = args.max_range + 1.0
    minimum = selected[:, :2].min(axis=0) - margin
    maximum = selected[:, :2].max(axis=0) + margin
    size = np.ceil((maximum - minimum) / args.map_resolution).astype(np.int32) + 1
    occupied = np.zeros((int(size[1]), int(size[0])), dtype=bool)
    heights = np.full_like(occupied, -np.inf, dtype=np.float32)
    ix = np.floor((selected[:, 0] - minimum[0]) / args.map_resolution).astype(np.int32)
    iy = np.floor((selected[:, 1] - minimum[1]) / args.map_resolution).astype(np.int32)
    valid = (ix >= 0) & (iy >= 0) & (ix < size[0]) & (iy < size[1])
    occupied[iy[valid], ix[valid]] = True
    np.maximum.at(heights, (iy[valid], ix[valid]), selected[valid, 2].astype(np.float32))
    heights[~np.isfinite(heights)] = 0.0
    return MapGrid(float(minimum[0]), float(minimum[1]), args.map_resolution, occupied, heights)


def grid_lookup(grid: MapGrid, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ix = np.floor((x - grid.origin_x) / grid.resolution).astype(np.int32)
    iy = np.floor((y - grid.origin_y) / grid.resolution).astype(np.int32)
    inside = (ix >= 0) & (iy >= 0) & (ix < grid.occupied.shape[1]) & (iy < grid.occupied.shape[0])
    return ix, iy, inside


def has_clearance(grid: MapGrid, x: float, y: float, radius: float) -> bool:
    cx = int(math.floor((x - grid.origin_x) / grid.resolution))
    cy = int(math.floor((y - grid.origin_y) / grid.resolution))
    cells = int(math.ceil(radius / grid.resolution))
    if cx - cells < 0 or cy - cells < 0 or cx + cells >= grid.occupied.shape[1] or cy + cells >= grid.occupied.shape[0]:
        return False
    return not grid.occupied[cy - cells : cy + cells + 1, cx - cells : cx + cells + 1].any()


def raycast_panorama(grid: MapGrid, x: float, y: float, args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray]:
    angles = np.linspace(-math.pi, math.pi, args.sectors, endpoint=False)
    distances = np.arange(args.min_range, args.max_range + args.ray_step * 0.5, args.ray_step)
    sx = x + np.cos(angles)[:, None] * distances[None, :]
    sy = y + np.sin(angles)[:, None] * distances[None, :]
    ix, iy, inside = grid_lookup(grid, sx, sy)
    hit = np.zeros_like(inside)
    hit[inside] = grid.occupied[iy[inside], ix[inside]]
    any_hit = hit.any(axis=1)
    first = np.argmax(hit, axis=1)
    ranges = np.full(args.sectors, np.nan, dtype=np.float32)
    heights = np.zeros(args.sectors, dtype=np.float32)
    rows = np.flatnonzero(any_hit)
    ranges[rows] = distances[first[rows]].astype(np.float32)
    heights[rows] = grid.max_height[iy[rows, first[rows]], ix[rows, first[rows]]]
    return ranges, heights


def candidate_positions(points: np.ndarray, grid: MapGrid, args: argparse.Namespace) -> np.ndarray:
    selected = points[(points[:, 2] >= args.map_min_z) & (points[:, 2] <= args.map_max_z)]
    lo, hi = selected[:, :2].min(axis=0), selected[:, :2].max(axis=0)
    positions: list[tuple[float, float]] = []
    for x in np.arange(lo[0], hi[0] + 1e-9, args.candidate_spacing):
        for y in np.arange(lo[1], hi[1] + 1e-9, args.candidate_spacing):
            if not has_clearance(grid, float(x), float(y), args.robot_clearance):
                continue
            ranges, _ = raycast_panorama(grid, float(x), float(y), args)
            if np.isfinite(ranges).sum() >= args.min_virtual_hits:
                positions.append((float(x), float(y)))
    return np.asarray(positions, dtype=np.float32)


def build_database(map_points: np.ndarray, grid: MapGrid, args: argparse.Namespace) -> VirtualDatabase:
    if args.database_cache.exists() and not args.rebuild_database:
        data = np.load(args.database_cache)
        if int(data["sectors"]) == args.sectors:
            return VirtualDatabase(data["positions_xy"], data["ranges"], data["heights"])
    start = time.perf_counter()
    positions = candidate_positions(map_points, grid, args)
    ranges = np.full((len(positions), args.sectors), np.nan, dtype=np.float32)
    heights = np.zeros((len(positions), args.sectors), dtype=np.float32)
    for index, (x, y) in enumerate(positions):
        ranges[index], heights[index] = raycast_panorama(grid, float(x), float(y), args)
        if index % max(1, len(positions) // 10) == 0:
            print(f"[cross_fov] database {index}/{len(positions)}", flush=True)
    args.database_cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.database_cache,
        positions_xy=positions,
        ranges=ranges,
        heights=heights,
        sectors=np.asarray(args.sectors),
    )
    print(f"[cross_fov] database candidates={len(positions)} build_sec={time.perf_counter()-start:.1f}")
    return VirtualDatabase(positions, ranges, heights)


def points_for_query(path: Path, args: argparse.Namespace) -> np.ndarray:
    points = load_points(path)
    radius = np.linalg.norm(points[:, :2], axis=1)
    angle = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    valid = (
        (radius >= args.min_range)
        & (radius <= args.max_range)
        & (points[:, 2] >= args.scan_min_z)
        & (points[:, 2] <= args.scan_max_z)
        & (np.abs(angle) <= args.query_fov_deg * 0.5)
    )
    return voxel_downsample(points[valid], args.query_voxel)


def bev_from_points(points: np.ndarray, fov_deg: float, args: argparse.Namespace) -> Bev:
    rings, sectors = args.rings, args.sectors
    radius = np.linalg.norm(points[:, :2], axis=1)
    angle = np.arctan2(points[:, 1], points[:, 0])
    rindex = np.clip(np.floor(radius / args.max_range * rings).astype(np.int32), 0, rings - 1)
    sindex = np.floor((angle + math.pi) / (2.0 * math.pi) * sectors).astype(np.int32) % sectors
    raw = np.zeros((rings, sectors), dtype=np.float64)
    height = np.zeros((rings, sectors), dtype=np.float64)
    raw[rindex, sindex] = 1.0
    np.maximum.at(height, (rindex, sindex), points[:, 2])
    mu = raw.copy()
    delta_theta = 2.0 * math.pi / sectors
    delta_range = args.max_range / rings
    radial_sigma = max(int(round(args.translation_sigma / delta_range)), 1)
    mu = gaussian_filter1d(mu, radial_sigma, axis=0, mode="nearest")
    for ring in range(rings):
        center_range = max((ring + 0.5) * delta_range, delta_range * 0.5)
        density = float(raw[ring].mean())
        angular_sigma = args.translation_sigma * math.sqrt(max(density, 1e-6)) / (center_range * delta_theta)
        mu[ring] = gaussian_filter1d(mu[ring], max(angular_sigma, 0.25), mode="wrap")
    # 已实际观测到的占据单元必须保持概率 1；Gaussian 只用于把平移不确定性
    # 扩散到邻域。若直接使用归一化卷积，稀疏 LiDAR 的峰值会被压成 0.2~0.6，
    # 随后被论文固定的 sigma_thr=0.4 全部误判为高不确定。
    mu = np.maximum(raw, np.clip(mu, 0.0, 1.0))
    uncertainty = np.sqrt(mu * (1.0 - mu))
    sector_angles = np.linspace(-180.0, 180.0, sectors, endpoint=False)
    mask = np.abs(sector_angles) <= fov_deg * 0.5 + 1e-6
    return Bev(mu, uncertainty, height, mask)


def bev_from_virtual(ranges: np.ndarray, heights: np.ndarray, args: argparse.Namespace) -> Bev:
    valid = np.isfinite(ranges)
    angles = np.linspace(-math.pi, math.pi, args.sectors, endpoint=False)
    points = np.column_stack(
        [
            ranges[valid] * np.cos(angles[valid]),
            ranges[valid] * np.sin(angles[valid]),
            heights[valid],
        ]
    )
    return bev_from_points(points, 360.0, args)


def masked_ring_key(bev: Bev, mask: np.ndarray) -> np.ndarray:
    mask2 = mask[None, :]
    count = max(1, int(mask.sum()))
    key = np.concatenate(
        [
            (bev.occupancy * mask2).sum(axis=1) / count,
            (bev.max_height * mask2).sum(axis=1) / count,
        ]
    )
    norm = np.linalg.norm(key)
    return key if norm <= 1e-12 else key / norm


def roll_bev(bev: Bev, shift: int) -> Bev:
    return Bev(
        np.roll(bev.occupancy, shift, axis=1),
        np.roll(bev.uncertainty, shift, axis=1),
        np.roll(bev.max_height, shift, axis=1),
        np.roll(bev.fov_mask, shift),
    )


def virtual_pairs(fov_deg: float, center_deg: float, sectors: int) -> list[SensorPair]:
    n = max(1, int(round(fov_deg / 90.0)))
    half = fov_deg / (2.0 * n)
    centers = [center_deg + (2 * (i + 1) - n - 1) * half for i in range(n)]
    angles = np.linspace(-180.0, 180.0, sectors, endpoint=False)
    sensor_masks = []
    for center in centers:
        sensor_masks.append(np.abs(np.vectorize(norm_deg)(angles - center)) <= half + 1e-6)
    if n == 1:
        return [SensorPair(norm_deg(centers[0]), sensor_masks[0])]
    pairs: list[SensorPair] = []
    for i in range(n):
        for j in range(i + 1, n):
            sx, sy = math.cos(math.radians(centers[i])) + math.cos(math.radians(centers[j])), math.sin(math.radians(centers[i])) + math.sin(math.radians(centers[j]))
            center = centers[i] if math.hypot(sx, sy) < 1e-9 else math.degrees(math.atan2(sy, sx))
            pairs.append(SensorPair(norm_deg(center), sensor_masks[i] | sensor_masks[j]))
    return pairs


def overlap_masks(query: SensorPair, database: SensorPair, hint_deg: float, sectors: int) -> tuple[np.ndarray, np.ndarray, float]:
    shift = int(round(hint_deg * sectors / 360.0))
    db_in_query = np.roll(database.mask, shift)
    qmask = query.mask & db_in_query
    query_in_db = np.roll(query.mask, -shift)
    dmask = database.mask & query_in_db
    denominator = max(1, min(int(query.mask.sum()), int(database.mask.sum())))
    return qmask, dmask, float(qmask.sum()) / denominator


def score_branch(query: Bev, database: Bev, qpair: SensorPair, dpair: SensorPair, hint_deg: float, args: argparse.Namespace) -> dict[str, object]:
    qmask_hint, _, fov_weight = overlap_masks(qpair, dpair, hint_deg, args.sectors)
    if fov_weight < args.min_fov_overlap:
        return {"score": 0.0}
    hint_shift = int(round(hint_deg * args.sectors / 360.0))
    window = int(round(args.heading_window_deg * args.sectors / 360.0))
    best: dict[str, object] = {"score": 0.0}
    for shift in range(hint_shift - window, hint_shift + window + 1):
        aligned = roll_bev(database, shift)
        overlap = qpair.mask & aligned.fov_mask
        if overlap.sum() < 3:
            continue
        mask2 = overlap[None, :]
        uq, ud = query.uncertainty, aligned.uncertainty
        pq = np.clip(query.occupancy * (1.0 - uq) + 0.5 * uq, 1e-6, 1.0 - 1e-6)
        pd = np.clip(aligned.occupancy * (1.0 - ud) + 0.5 * ud, 1e-6, 1.0 - 1e-6)
        union = mask2 & ((query.occupancy + aligned.occupancy) > 0.1)
        if union.sum() < 5:
            continue
        dsym = 0.5 * (
            pq * np.log(pq / pd) + (1.0 - pq) * np.log((1.0 - pq) / (1.0 - pd))
            + pd * np.log(pd / pq) + (1.0 - pd) * np.log((1.0 - pd) / (1.0 - pq))
        )
        columns: list[tuple[float, int]] = []
        for sector in np.flatnonzero(union.any(axis=0)):
            selected = union[:, sector]
            columns.append((float(dsym[selected, sector].mean()), int(selected.sum())))
        trim = min(len(columns) - 1, int(round(len(columns) * args.column_trim)))
        columns.sort(key=lambda item: item[0])
        kept = columns[: len(columns) - trim] if trim > 0 else columns
        bkl = math.exp(-sum(value * weight for value, weight in kept) / max(1, sum(weight for _, weight in kept)))
        observed = mask2 & (query.max_height > 0.0) & (aligned.max_height > 0.0)
        if observed.sum() < 5:
            ccz = 0.0
        else:
            weights = pq * pd * observed
            weight_sum = float(weights.sum())
            qmean = float((weights * query.max_height).sum() / max(weight_sum, 1e-9))
            dmean = float((weights * aligned.max_height).sum() / max(weight_sum, 1e-9))
            qz, dz = query.max_height - qmean, aligned.max_height - dmean
            denominator = math.sqrt(float((weights * qz * qz).sum()) * float((weights * dz * dz).sum()))
            ccz = 1.0 if denominator <= 1e-12 else max(0.0, float((weights * qz * dz).sum()) / denominator)
        score = fov_weight * ccz * bkl
        if score > float(best["score"]):
            raw_certainty = (1.0 - uq) * (1.0 - ud) * np.minimum(query.occupancy, aligned.occupancy) * mask2
            certainty = raw_certainty.copy()
            certainty[(uq >= args.certainty_sigma_max) | (ud >= args.certainty_sigma_max)] = 0.0
            certainty[(query.occupancy < args.certainty_mu_min) | (aligned.occupancy < args.certainty_mu_min)] = 0.0
            best = {
                "score": score,
                "shift": shift % args.sectors,
                "yaw_deg": norm_deg(shift * 360.0 / args.sectors),
                "bkl": bkl,
                "ccz": ccz,
                "certainty": certainty,
                "raw_certainty_max": float(raw_certainty.max()),
                "query_mu_max": float(query.occupancy[:, overlap].max()),
                "database_mu_max": float(aligned.occupancy[:, overlap].max()),
                "certainty_bins": int((certainty >= args.certainty_threshold).sum()),
            }
    return best


def make_cloud(points: np.ndarray, voxel: float) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    return cloud.voxel_down_sample(voxel) if voxel > 0.0 else cloud


def crop_map(map_points: np.ndarray, x: float, y: float, radius: float) -> np.ndarray:
    mask = (
        (map_points[:, 0] >= x - radius)
        & (map_points[:, 0] <= x + radius)
        & (map_points[:, 1] >= y - radius)
        & (map_points[:, 1] <= y + radius)
    )
    return map_points[mask]


def gicp(source: np.ndarray, target: np.ndarray, init: np.ndarray, args: argparse.Namespace) -> o3d.pipelines.registration.RegistrationResult:
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-6,
        relative_rmse=1e-6,
        max_iteration=args.gicp_iterations,
    )
    return o3d.pipelines.registration.registration_generalized_icp(
        make_cloud(source, args.registration_voxel),
        make_cloud(target, args.registration_voxel),
        args.max_correspondence,
        init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria,
    )


def certainty_filter(points: np.ndarray, certainty: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    radius = np.linalg.norm(points[:, :2], axis=1)
    angle = np.arctan2(points[:, 1], points[:, 0])
    ri = np.clip(np.floor(radius / args.max_range * args.rings).astype(np.int32), 0, args.rings - 1)
    si = np.floor((angle + math.pi) / (2.0 * math.pi) * args.sectors).astype(np.int32) % args.sectors
    keep = certainty[ri, si] >= args.certainty_threshold
    return points[keep]


def evaluate_synthetic(target: Target, query_points: np.ndarray, map_points: np.ndarray, database: VirtualDatabase, args: argparse.Namespace) -> tuple[dict[str, object], list[dict[str, object]]]:
    start = time.perf_counter()
    qbev = bev_from_points(query_points, args.query_fov_deg, args)
    mask = qbev.fov_mask
    candidates: list[dict[str, object]] = []
    for db_index in range(len(database.positions_xy)):
        dbev = bev_from_virtual(database.ranges[db_index], database.heights[db_index], args)
        for shift in range(args.sectors):
            aligned = roll_bev(dbev, shift)
            observed = mask[None, :] & ((qbev.occupancy + aligned.occupancy) > 0.1)
            if observed.sum() < 5:
                continue
            occupancy_error = float(np.abs(qbev.occupancy - aligned.occupancy)[observed].mean())
            height_observed = mask[None, :] & (qbev.max_height > 0.0) & (aligned.max_height > 0.0)
            height_error = float(np.abs(qbev.max_height - aligned.max_height)[height_observed].mean()) if height_observed.any() else args.max_range
            score = occupancy_error + args.synthetic_height_weight * height_error
            candidates.append({"db_index": db_index, "shift": shift, "retrieval_score": score})
    candidates.sort(key=lambda row: float(row["retrieval_score"]))
    refined: list[dict[str, object]] = []
    details: list[dict[str, object]] = []
    for rank, candidate in enumerate(candidates[: args.top_k], start=1):
        db_index, shift = int(candidate["db_index"]), int(candidate["shift"])
        x, y = map(float, database.positions_xy[db_index])
        yaw = norm_deg(shift * 360.0 / args.sectors)
        local_map = crop_map(map_points, x, y, args.map_crop_radius)
        result = gicp(query_points, local_map, pose_matrix(x, y, yaw), args)
        fx, fy, fyaw = pose_from_matrix(result.transformation)
        trans_error, yaw_error = math.hypot(fx - target.x, fy - target.y), angle_error_deg(fyaw, target.yaw_deg)
        selection = float(candidate["retrieval_score"]) + args.rmse_weight * float(result.inlier_rmse)
        row = {"rank": rank, "x": fx, "y": fy, "yaw_deg": fyaw, "translation_error_m": trans_error, "yaw_error_deg": yaw_error, "fitness": float(result.fitness), "rmse": float(result.inlier_rmse), "front_score": float(candidate["retrieval_score"]), "selection": selection}
        refined.append(row)
        details.append({"target_id": target.target_id, **row})
    refined.sort(key=lambda row: float(row["selection"]))
    return result_row("synthetic", target, refined[0] if refined else None, start), details


def evaluate_gprobe(target: Target, query_points: np.ndarray, map_points: np.ndarray, database: VirtualDatabase, db_bevs: list[Bev], args: argparse.Namespace, keyframes: list[Keyframe] | None = None, keyframe_points: list[np.ndarray] | None = None) -> tuple[dict[str, object], list[dict[str, object]]]:
    start = time.perf_counter()
    qbev = bev_from_points(query_points, args.query_fov_deg, args)
    qpairs = virtual_pairs(args.query_fov_deg, 0.0, args.sectors)
    dpairs = virtual_pairs(360.0, 0.0, args.sectors)
    nominations: dict[int, dict[str, float]] = {}
    branches: list[tuple[SensorPair, SensorPair, float, np.ndarray, np.ndarray, float]] = []
    for qpair in qpairs:
        for dpair in dpairs:
            hint = norm_deg(qpair.center_deg - dpair.center_deg)
            qmask, dmask, fov_weight = overlap_masks(qpair, dpair, hint, args.sectors)
            if fov_weight < args.min_fov_overlap:
                continue
            qkey = masked_ring_key(qbev, qmask)
            similarities = np.asarray([float(qkey @ masked_ring_key(dbev, dmask)) for dbev in db_bevs])
            top = np.argsort(similarities)[-args.branch_top_k :][::-1]
            for index in top:
                item = nominations.setdefault(int(index), {"support": 0.0, "similarity": 0.0})
                item["support"] += fov_weight
                item["similarity"] += float(similarities[index])
            branches.append((qpair, dpair, hint, qmask, dmask, fov_weight))
    shortlist = sorted(nominations, key=lambda i: (nominations[i]["support"], nominations[i]["similarity"]), reverse=True)[: args.top_k]
    front_rows: list[dict[str, object]] = []
    for db_index in shortlist:
        branch_scores: list[dict[str, object]] = []
        for qpair, dpair, hint, _, _, _ in branches:
            scored = score_branch(qbev, db_bevs[db_index], qpair, dpair, hint, args)
            if float(scored.get("score", 0.0)) > 0.0:
                scored["hint_deg"] = hint
                branch_scores.append(scored)
        if not branch_scores:
            continue
        hypothesis: dict[int, list[float]] = {}
        for branch in branch_scores:
            key = int(round(float(branch["hint_deg"]) / 90.0)) % 4
            hypothesis.setdefault(key, []).append(float(branch["score"]))
        axis_scores: dict[int, float] = {}
        for key, values in hypothesis.items():
            axis = key % 2
            axis_scores[axis] = max(axis_scores.get(axis, 0.0), float(np.mean(values)))
        best_axis = max(axis_scores, key=axis_scores.get)
        best_axis_score = axis_scores[best_axis]
        pdom = 1.0 / (1.0 + sum(math.exp(args.sgrt_kappa * (score / max(best_axis_score, 1e-12) - 1.0)) for axis, score in axis_scores.items() if axis != best_axis))
        best_branch = max(branch_scores, key=lambda row: float(row["score"]))
        gamma = min(args.query_fov_deg, 360.0) / 360.0
        final_score = float(best_branch["score"]) * pdom ** (1.0 - gamma)
        front_rows.append({"db_index": db_index, "front_score": final_score, "pdom": pdom, **best_branch})
    front_rows.sort(key=lambda row: float(row["front_score"]), reverse=True)
    details: list[dict[str, object]] = []
    refined: list[dict[str, object]] = []
    for rank, front in enumerate(front_rows[: args.gprobe_refine_top_k], start=1):
        db_index = int(front["db_index"])
        relative_yaw = float(front["yaw_deg"])
        if keyframes is not None and keyframe_points is not None:
            keyframe = keyframes[db_index]
            registration_target = keyframe_points[db_index]
            initial = pose_matrix(0.0, 0.0, relative_yaw)
        else:
            x, y = map(float, database.positions_xy[db_index])
            registration_target = crop_map(map_points, x, y, args.map_crop_radius)
            initial = pose_matrix(x, y, relative_yaw)
        coarse = gicp(query_points, registration_target, initial, args)
        filtered = certainty_filter(query_points, np.asarray(front["certainty"]), args)
        fine = coarse if len(filtered) < args.min_certainty_points else gicp(filtered, registration_target, coarse.transformation, args)
        final_transform = pose_matrix(keyframe.x, keyframe.y, keyframe.yaw_deg) @ fine.transformation if keyframes is not None else fine.transformation
        fx, fy, fyaw = pose_from_matrix(final_transform)
        trans_error, yaw_error = math.hypot(fx - target.x, fy - target.y), angle_error_deg(fyaw, target.yaw_deg)
        selection = -float(front["front_score"]) + args.rmse_weight * float(fine.inlier_rmse)
        row = {"rank": rank, "x": fx, "y": fy, "yaw_deg": fyaw, "translation_error_m": trans_error, "yaw_error_deg": yaw_error, "fitness": float(fine.fitness), "rmse": float(fine.inlier_rmse), "front_score": float(front["front_score"]), "pdom": float(front["pdom"]), "certainty_points": len(filtered), "certainty_bins": int(front["certainty_bins"]), "raw_certainty_max": float(front["raw_certainty_max"]), "query_mu_max": float(front["query_mu_max"]), "database_mu_max": float(front["database_mu_max"]), "selection": selection}
        refined.append(row)
        details.append({"target_id": target.target_id, **row})
    refined.sort(key=lambda row: float(row["selection"]))
    return result_row("gprobe", target, refined[0] if refined else None, start), details


def result_row(method: str, target: Target, best: dict[str, object] | None, start: float) -> dict[str, object]:
    best = best or {}
    trans = float(best.get("translation_error_m", math.inf))
    yaw = float(best.get("yaw_error_deg", math.inf))
    return {
        "method": method,
        "target_id": target.target_id,
        "cloud_index": target.cloud_index,
        "reference_x_m": target.x,
        "reference_y_m": target.y,
        "reference_yaw_deg": target.yaw_deg,
        "selected_x_m": best.get("x", math.nan),
        "selected_y_m": best.get("y", math.nan),
        "selected_yaw_deg": best.get("yaw_deg", math.nan),
        "translation_error_m": trans,
        "yaw_error_deg": yaw,
        "fitness": best.get("fitness", 0.0),
        "rmse": best.get("rmse", math.inf),
        "front_score": best.get("front_score", 0.0),
        "pdom": best.get("pdom", 1.0),
        "success_0p2_5deg": int(trans <= 0.2 and yaw <= 5.0),
        "success_0p3_5deg": int(trans <= 0.3 and yaw <= 5.0),
        "elapsed_ms": (time.perf_counter() - start) * 1000.0,
        "peak_rss_mb": rss_mb(),
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate complete synthetic-LiDAR and G-PROBE relocalization pipelines.")
    parser.add_argument("--workspace", type=Path, default=repo_root())
    parser.add_argument("--targets-csv", type=Path, required=True)
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--method", choices=["synthetic", "gprobe", "both"], default="both")
    parser.add_argument("--max-targets", type=int, default=0)
    parser.add_argument("--database-cache", type=Path, default=Path(".codex_tmp/cross_fov_full/virtual_db.npz"))
    parser.add_argument("--gprobe-keyframes-csv", type=Path, help="Real scan database used by the G-PROBE pipeline.")
    parser.add_argument("--rebuild-database", action="store_true")
    parser.add_argument("--map-resolution", type=float, default=0.18)
    parser.add_argument("--candidate-spacing", type=float, default=0.75)
    parser.add_argument("--robot-clearance", type=float, default=0.28)
    parser.add_argument("--map-min-z", type=float, default=0.2)
    parser.add_argument("--map-max-z", type=float, default=2.5)
    parser.add_argument("--scan-min-z", type=float, default=0.2)
    parser.add_argument("--scan-max-z", type=float, default=2.5)
    parser.add_argument("--min-range", type=float, default=0.8)
    parser.add_argument("--max-range", type=float, default=20.0)
    parser.add_argument("--ray-step", type=float, default=0.18)
    parser.add_argument("--min-virtual-hits", type=int, default=20)
    parser.add_argument("--rings", type=int, default=40)
    parser.add_argument("--sectors", type=int, default=60)
    parser.add_argument("--query-fov-deg", type=float, default=120.0)
    parser.add_argument("--database-fov-deg", type=float, default=120.0)
    parser.add_argument("--translation-sigma", type=float, default=2.0)
    parser.add_argument("--query-voxel", type=float, default=0.2)
    parser.add_argument("--top-k", type=int, default=20)
    parser.add_argument("--branch-top-k", type=int, default=5)
    parser.add_argument("--gprobe-refine-top-k", type=int, default=5)
    parser.add_argument("--min-fov-overlap", type=float, default=0.3)
    parser.add_argument("--heading-window-deg", type=float, default=50.0)
    parser.add_argument("--column-trim", type=float, default=0.1)
    parser.add_argument("--sgrt-kappa", type=float, default=10.0)
    parser.add_argument("--certainty-threshold", type=float, default=0.01)
    parser.add_argument("--certainty-sigma-max", type=float, default=0.4)
    parser.add_argument("--certainty-mu-min", type=float, default=0.15)
    parser.add_argument("--min-certainty-points", type=int, default=40)
    parser.add_argument("--registration-voxel", type=float, default=0.2)
    parser.add_argument("--max-correspondence", type=float, default=2.0)
    parser.add_argument("--gicp-iterations", type=int, default=30)
    parser.add_argument("--map-crop-radius", type=float, default=22.0)
    parser.add_argument("--rmse-weight", type=float, default=0.25)
    parser.add_argument("--synthetic-height-weight", type=float, default=0.2)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    workspace = args.workspace.resolve()
    for name in ["targets_csv", "map", "output_dir", "database_cache", "gprobe_keyframes_csv"]:
        value = getattr(args, name)
        if value is None:
            continue
        setattr(args, name, value if value.is_absolute() else workspace / value)
    targets = read_targets(args.targets_csv, args.max_targets)
    map_points = load_points(args.map)
    grid = build_map_grid(map_points, args)
    database = build_database(map_points, grid, args)
    db_bevs = [bev_from_virtual(database.ranges[i], database.heights[i], args) for i in range(len(database.positions_xy))]
    keyframes: list[Keyframe] | None = None
    keyframe_points: list[np.ndarray] | None = None
    if args.gprobe_keyframes_csv:
        keyframes = read_keyframes(args.gprobe_keyframes_csv)
        keyframe_points = [points_for_query(row.pcd, args) for row in keyframes]
        db_bevs = [bev_from_points(points, args.database_fov_deg, args) for points in keyframe_points]
        print(f"[cross_fov] G-PROBE real keyframes={len(keyframes)}", flush=True)
    methods = ["synthetic", "gprobe"] if args.method == "both" else [args.method]
    results: list[dict[str, object]] = []
    details: list[dict[str, object]] = []
    for index, target in enumerate(targets, start=1):
        query = points_for_query(target.pcd, args)
        for method in methods:
            if method == "synthetic":
                row, candidate_rows = evaluate_synthetic(target, query, map_points, database, args)
            else:
                row, candidate_rows = evaluate_gprobe(target, query, map_points, database, db_bevs, args, keyframes, keyframe_points)
            results.append(row)
            details.extend({"method": method, **candidate} for candidate in candidate_rows)
            print(
                f"[cross_fov] {index}/{len(targets)} method={method} idx={target.cloud_index} "
                f"err={float(row['translation_error_m']):.3f}m/{float(row['yaw_error_deg']):.3f}deg "
                f"success03={row['success_0p3_5deg']} elapsed={float(row['elapsed_ms']):.1f}ms",
                flush=True,
            )
    write_csv(args.output_dir / "results.csv", results)
    write_csv(args.output_dir / "candidates.csv", details)
    summary: list[dict[str, object]] = []
    for method in methods:
        rows = [row for row in results if row["method"] == method]
        summary.append(
            {
                "method": method,
                "targets": len(rows),
                "success_0p2_5deg": sum(int(row["success_0p2_5deg"]) for row in rows),
                "success_0p3_5deg": sum(int(row["success_0p3_5deg"]) for row in rows),
                "median_translation_error_m": float(np.median([float(row["translation_error_m"]) for row in rows])),
                "median_yaw_error_deg": float(np.median([float(row["yaw_error_deg"]) for row in rows])),
                "median_elapsed_ms": float(np.median([float(row["elapsed_ms"]) for row in rows])),
                "peak_rss_mb": max(float(row["peak_rss_mb"]) for row in rows),
            }
        )
    write_csv(args.output_dir / "summary.csv", summary)
    print(f"[cross_fov] wrote {args.output_dir / 'summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
