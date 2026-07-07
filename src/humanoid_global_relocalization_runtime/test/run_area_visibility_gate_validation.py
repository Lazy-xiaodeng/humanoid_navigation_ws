#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_area_visibility_gate_validation.py

文件作用：
  1. 离线验证“区域可见性 / 禁穿墙 / 站位可行性”门控能否提升全局重定位候选的准确性。
  2. 输入已经由 Scan Context、三角描述子、3D-BBS、GICP 等算法生成的候选 CSV，不重新做召回和精配准。
  3. 对每个候选位姿，把当前 scan 投到地图坐标系，检查 scan 端点是否贴近地图障碍表面、视线是否提前撞墙、机器人站位是否落入障碍。
  4. 输出候选级明细和阈值 sweep 汇总，用来判断这类拓扑/物理约束是否值得接进 C++ 运行态。

重要说明：
  - 该脚本只做离线评估，不发布 TF，不注入 initialpose，也不修改导航系统。
  - 评分和门控不使用真值；真值只用于最后统计 0.2m/3deg、0.3m/5deg、0.5m/10deg 成功率。
  - 这里的“区域”是从 PCD 地图投影出来的几何可见区域，不依赖人工房间语义标注。
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
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class TargetSample:
    """一个待验证 scan 的真值位姿和 PCD 文件路径。"""

    waypoint_id: str
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
class CandidateGateResult:
    """一个候选在可见性门控下的计算结果。"""

    row: dict[str, str]
    final_x: float
    final_y: float
    final_yaw_deg: float
    endpoint_near_ratio: float
    ray_wall_cross_ratio: float
    origin_clear: bool
    base_score: float
    visibility_score: float
    translation_error_m: float
    yaw_error_deg: float


def repo_root_from_script() -> Path:
    """根据脚本位置推断 humanoid_ws 根目录，保证从任意 cwd 执行都能找到默认文件。"""

    return Path(__file__).resolve().parents[3]


def normalize_angle_deg(angle: float) -> float:
    """把角度规整到 [-180, 180]，便于统计 yaw 误差。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def current_rss_mb() -> float:
    """读取当前 Python 进程的最大 RSS，Linux 下 ru_maxrss 单位是 KB。"""

    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def current_cpu_seconds() -> float:
    """读取当前进程累计 user+system CPU 秒，用于换算平均占用核心数。"""

    usage = resource.getrusage(resource.RUSAGE_SELF)
    return float(usage.ru_utime + usage.ru_stime)


def read_targets(targets_csv: Path) -> dict[int, TargetSample]:
    """读取目标 scan CSV，并用 cloud_index 建索引，方便候选 CSV 按 target_index 关联。"""

    targets: dict[int, TargetSample] = {}
    with targets_csv.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            cloud_index = int(row["cloud_index"])
            targets[cloud_index] = TargetSample(
                waypoint_id=row.get("target_id", f"target_{cloud_index}"),
                cloud_index=cloud_index,
                reference_x=float(row["x"]),
                reference_y=float(row["y"]),
                reference_yaw_deg=float(row["yaw_deg"]),
                pcd_path=Path(row["pcd"]),
            )
    return targets


def read_candidate_rows(candidate_csv: Path, max_targets: int) -> dict[int, list[dict[str, str]]]:
    """读取候选 CSV，并按 target_index 分组；max_targets 只用于快速 smoke。"""

    grouped: dict[int, list[dict[str, str]]] = {}
    with candidate_csv.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            target_index = int(row.get("target_index", row.get("cloud_index", "0")))
            if target_index not in grouped and 0 < max_targets <= len(grouped):
                continue
            grouped.setdefault(target_index, []).append(row)
    return grouped


def load_map_points(map_path: Path, args: argparse.Namespace) -> np.ndarray:
    """读取全局 PCD 地图，并按高度范围保留可用于墙体/障碍判断的点。"""

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
    """把地图障碍点压成二维栅格，后续用于站位可行性和射线禁穿墙检查。"""

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
    """把 map 坐标批量转换成栅格索引，并返回每个点是否落在栅格范围内。"""

    ix = np.floor((x - grid.origin_x) / grid.resolution).astype(np.int32)
    iy = np.floor((y - grid.origin_y) / grid.resolution).astype(np.int32)
    inside = (ix >= 0) & (iy >= 0) & (ix < grid.occupied.shape[1]) & (iy < grid.occupied.shape[0])
    return ix, iy, inside


def has_origin_clearance(grid: OccupancyGrid, x: float, y: float, radius_m: float) -> bool:
    """检查候选 base 位置周围是否被障碍占据，避免把机器人放进墙体或大障碍内部。"""

    cx = int(math.floor((x - grid.origin_x) / grid.resolution))
    cy = int(math.floor((y - grid.origin_y) / grid.resolution))
    radius_cells = int(math.ceil(radius_m / grid.resolution))
    if cx < radius_cells or cy < radius_cells:
        return False
    if cx + radius_cells >= grid.occupied.shape[1] or cy + radius_cells >= grid.occupied.shape[0]:
        return False
    patch = grid.occupied[cy - radius_cells : cy + radius_cells + 1, cx - radius_cells : cx + radius_cells + 1]
    return not bool(patch.any())


def load_scan_points(pcd_path: Path, args: argparse.Namespace) -> np.ndarray:
    """读取目标 scan PCD，并按运行态近似规则裁剪距离和高度。"""

    cloud = o3d.io.read_point_cloud(str(pcd_path))
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
    points = points[valid]
    if len(points) > args.max_scan_points:
        rng = np.random.default_rng(args.random_seed + int(len(points)))
        points = points[rng.choice(len(points), args.max_scan_points, replace=False)]
    return points


def transform_scan_to_map(scan_points: np.ndarray, x: float, y: float, yaw_deg: float) -> np.ndarray:
    """把 base/body 坐标系下的 scan 点按候选位姿转换到 map 坐标系。"""

    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    transformed = scan_points.copy()
    transformed[:, 0] = c * scan_points[:, 0] - s * scan_points[:, 1] + x
    transformed[:, 1] = s * scan_points[:, 0] + c * scan_points[:, 1] + y
    return transformed


def endpoint_near_ratio(scan_in_map: np.ndarray, map_kdtree: cKDTree, args: argparse.Namespace) -> float:
    """统计 scan 端点落到候选位姿后有多少比例能贴近地图表面。"""

    if len(scan_in_map) == 0:
        return 0.0
    distances, _ = map_kdtree.query(scan_in_map[:, :3], k=1, workers=-1)
    return float(np.mean(distances <= args.endpoint_near_distance))


def ray_wall_cross_ratio(
    scan_points: np.ndarray,
    scan_in_map: np.ndarray,
    grid: OccupancyGrid,
    x: float,
    y: float,
    args: argparse.Namespace,
) -> float:
    """统计候选位姿解释当前 scan 时，有多少射线会在端点之前提前撞到地图障碍。"""

    if len(scan_points) == 0:
        return 1.0
    if len(scan_points) > args.ray_sample_points:
        rng = np.random.default_rng(args.random_seed + int(abs(x * 1000.0)) + int(abs(y * 1000.0)))
        indexes = rng.choice(len(scan_points), args.ray_sample_points, replace=False)
    else:
        indexes = np.arange(len(scan_points))

    crossed = 0
    valid_rays = 0
    for index in indexes:
        endpoint = scan_in_map[index, :2]
        length = float(np.linalg.norm(endpoint - np.array([x, y], dtype=np.float64)))
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


def candidate_pose(row: dict[str, str]) -> tuple[float, float, float]:
    """从不同候选 CSV 字段中读取最终位姿，优先使用 GICP/精配准后的 final_* 字段。"""

    pose_field_sets = [
        ("final_x_m", "final_y_m", "final_yaw_deg"),
        ("x", "y", "yaw_deg"),
        ("selected_x_m", "selected_y_m", "selected_yaw_deg"),
        ("candidate_x_m", "candidate_y_m", "candidate_yaw_deg"),
    ]
    for x_name, y_name, yaw_name in pose_field_sets:
        if x_name in row and y_name in row and yaw_name in row:
            return float(row[x_name]), float(row[y_name]), float(row[yaw_name])
    raise KeyError(f"候选 CSV 缺少可识别位姿字段: {sorted(row.keys())}")


def candidate_truth_error(row: dict[str, str], target: TargetSample, x: float, y: float, yaw_deg: float) -> tuple[float, float]:
    """读取或计算候选相对真值的误差；真值只用于输出统计，不参与候选评分。"""

    if "translation_error_m" in row and "yaw_error_deg" in row:
        return float(row["translation_error_m"]), float(row["yaw_error_deg"])
    xy_error = math.hypot(x - target.reference_x, y - target.reference_y)
    yaw_error = abs(normalize_angle_deg(yaw_deg - target.reference_yaw_deg))
    return xy_error, yaw_error


def base_candidate_score(row: dict[str, str]) -> float:
    """复用候选自身可用的无真值质量信息，构造一个基础排序分数，数值越小越好。"""

    rmse = float(row.get("inlier_rmse", row.get("gicp_rmse", "1.0")))
    rank = float(row.get("descriptor_rank", row.get("rank", "1")))
    similarity = float(row.get("triangle_similarity", "0.0"))
    seed_xy = float(row.get("seed_drift_xy_m", "0.0"))
    return rmse + 0.015 * rank + 0.02 * seed_xy - 0.08 * similarity


def evaluate_candidate(
    row: dict[str, str],
    target: TargetSample,
    scan_points: np.ndarray,
    map_kdtree: cKDTree,
    grid: OccupancyGrid,
    args: argparse.Namespace,
) -> CandidateGateResult:
    """计算单个候选的可见性指标和无真值排序分数。"""

    x, y, yaw_deg = candidate_pose(row)
    scan_in_map = transform_scan_to_map(scan_points, x, y, yaw_deg)
    near_ratio = endpoint_near_ratio(scan_in_map, map_kdtree, args)
    wall_ratio = ray_wall_cross_ratio(scan_points, scan_in_map, grid, x, y, args)
    origin_clear = has_origin_clearance(grid, x, y, args.origin_clearance_radius)
    xy_error, yaw_error = candidate_truth_error(row, target, x, y, yaw_deg)
    base_score = base_candidate_score(row)
    visibility_score = (
        base_score
        + args.wall_penalty_weight * wall_ratio
        + args.endpoint_penalty_weight * max(0.0, args.endpoint_near_reference - near_ratio)
        - args.endpoint_reward_weight * near_ratio
        + (0.0 if origin_clear else args.origin_blocked_penalty)
    )
    return CandidateGateResult(
        row=row,
        final_x=x,
        final_y=y,
        final_yaw_deg=yaw_deg,
        endpoint_near_ratio=near_ratio,
        ray_wall_cross_ratio=wall_ratio,
        origin_clear=origin_clear,
        base_score=base_score,
        visibility_score=visibility_score,
        translation_error_m=xy_error,
        yaw_error_deg=yaw_error,
    )


def pass_gate(result: CandidateGateResult, min_near: float, max_wall: float, require_clear: bool) -> bool:
    """按指定阈值判断候选是否通过物理可见性门控。"""

    if require_clear and not result.origin_clear:
        return False
    return result.endpoint_near_ratio >= min_near and result.ray_wall_cross_ratio <= max_wall


def success_flags(result: CandidateGateResult) -> tuple[bool, bool, bool]:
    """返回严格、实用、可救回三档成功口径。"""

    strict = result.translation_error_m <= 0.2 and result.yaw_error_deg <= 3.0
    practical = result.translation_error_m <= 0.3 and result.yaw_error_deg <= 5.0
    recoverable = result.translation_error_m <= 0.5 and result.yaw_error_deg <= 10.0
    return strict, practical, recoverable


def write_candidate_details(path: Path, rows: list[dict[str, str]]) -> None:
    """把候选门控明细写成 CSV，便于后续定位具体失败区域。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, rows: list[dict[str, str]]) -> None:
    """把阈值 sweep 汇总写成 CSV，方便和其它算法验证结果横向比较。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def evaluate_all(args: argparse.Namespace) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    """完整执行候选可见性评估、阈值 sweep 和结果统计。"""

    workspace = args.workspace.resolve()
    map_path = args.map if args.map.is_absolute() else workspace / args.map
    targets_csv = args.targets if args.targets.is_absolute() else workspace / args.targets
    candidate_csv = args.candidates if args.candidates.is_absolute() else workspace / args.candidates

    map_points = load_map_points(map_path, args)
    map_kdtree = cKDTree(map_points[:, :3])
    grid = build_occupancy_grid(map_points, args)
    targets = read_targets(targets_csv)
    grouped_candidates = read_candidate_rows(candidate_csv, args.max_targets)

    all_details: list[dict[str, str]] = []
    per_target_results: dict[int, list[CandidateGateResult]] = {}
    for target_index, rows in grouped_candidates.items():
        if target_index not in targets:
            continue
        target = targets[target_index]
        scan_points = load_scan_points(target.pcd_path, args)
        evaluated = [
            evaluate_candidate(row, target, scan_points, map_kdtree, grid, args)
            for row in rows
        ]
        per_target_results[target_index] = evaluated
        for result in evaluated:
            strict, practical, recoverable = success_flags(result)
            detail = dict(result.row)
            detail.update(
                {
                    "gate_final_x_m": f"{result.final_x:.6f}",
                    "gate_final_y_m": f"{result.final_y:.6f}",
                    "gate_final_yaw_deg": f"{result.final_yaw_deg:.6f}",
                    "endpoint_near_ratio": f"{result.endpoint_near_ratio:.6f}",
                    "ray_wall_cross_ratio": f"{result.ray_wall_cross_ratio:.6f}",
                    "origin_clear": "1" if result.origin_clear else "0",
                    "base_score": f"{result.base_score:.6f}",
                    "visibility_score": f"{result.visibility_score:.6f}",
                    "truth_translation_error_m": f"{result.translation_error_m:.6f}",
                    "truth_yaw_error_deg": f"{result.yaw_error_deg:.6f}",
                    "truth_success_0p2m_3deg": "1" if strict else "0",
                    "truth_success_0p3m_5deg": "1" if practical else "0",
                    "truth_success_0p5m_10deg": "1" if recoverable else "0",
                }
            )
            all_details.append(detail)

    summary_rows: list[dict[str, str]] = []
    min_near_values = [float(item) for item in args.min_near_sweep.split(",") if item.strip()]
    max_wall_values = [float(item) for item in args.max_wall_sweep.split(",") if item.strip()]
    for min_near in min_near_values:
        for max_wall in max_wall_values:
            accepted = 0
            rejected = 0
            strict_count = 0
            practical_count = 0
            recoverable_count = 0
            selected_errors: list[tuple[float, float]] = []
            for results in per_target_results.values():
                passed = [item for item in results if pass_gate(item, min_near, max_wall, args.require_origin_clear)]
                if not passed:
                    rejected += 1
                    continue
                selected = min(passed, key=lambda item: item.visibility_score)
                accepted += 1
                strict, practical, recoverable = success_flags(selected)
                strict_count += int(strict)
                practical_count += int(practical)
                recoverable_count += int(recoverable)
                selected_errors.append((selected.translation_error_m, selected.yaw_error_deg))

            total = len(per_target_results)
            xy_values = [item[0] for item in selected_errors]
            yaw_values = [item[1] for item in selected_errors]
            summary_rows.append(
                {
                    "min_endpoint_near_ratio": f"{min_near:.3f}",
                    "max_ray_wall_cross_ratio": f"{max_wall:.3f}",
                    "require_origin_clear": "1" if args.require_origin_clear else "0",
                    "targets": str(total),
                    "accepted": str(accepted),
                    "rejected": str(rejected),
                    "success_0p2m_3deg": str(strict_count),
                    "success_0p3m_5deg": str(practical_count),
                    "success_0p5m_10deg": str(recoverable_count),
                    "median_xy_error_m": f"{float(np.median(xy_values)):.6f}" if xy_values else "nan",
                    "median_yaw_error_deg": f"{float(np.median(yaw_values)):.6f}" if yaw_values else "nan",
                    "max_xy_error_m": f"{float(np.max(xy_values)):.6f}" if xy_values else "nan",
                    "max_yaw_error_deg": f"{float(np.max(yaw_values)):.6f}" if yaw_values else "nan",
                }
            )

    accepted = 0
    strict_count = 0
    practical_count = 0
    recoverable_count = 0
    selected_errors: list[tuple[float, float]] = []
    for results in per_target_results.values():
        selected = min(results, key=lambda item: item.visibility_score)
        accepted += 1
        strict, practical, recoverable = success_flags(selected)
        strict_count += int(strict)
        practical_count += int(practical)
        recoverable_count += int(recoverable)
        selected_errors.append((selected.translation_error_m, selected.yaw_error_deg))

    xy_values = [item[0] for item in selected_errors]
    yaw_values = [item[1] for item in selected_errors]
    summary_rows.append(
        {
            "min_endpoint_near_ratio": "soft_rerank",
            "max_ray_wall_cross_ratio": "soft_rerank",
            "require_origin_clear": "0",
            "targets": str(len(per_target_results)),
            "accepted": str(accepted),
            "rejected": "0",
            "success_0p2m_3deg": str(strict_count),
            "success_0p3m_5deg": str(practical_count),
            "success_0p5m_10deg": str(recoverable_count),
            "median_xy_error_m": f"{float(np.median(xy_values)):.6f}" if xy_values else "nan",
            "median_yaw_error_deg": f"{float(np.median(yaw_values)):.6f}" if yaw_values else "nan",
            "max_xy_error_m": f"{float(np.max(xy_values)):.6f}" if xy_values else "nan",
            "max_yaw_error_deg": f"{float(np.max(yaw_values)):.6f}" if yaw_values else "nan",
        }
    )

    return all_details, summary_rows


def parse_args() -> argparse.Namespace:
    """解析命令行参数；默认路径指向 bag46 hard52 三角描述子候选验证数据。"""

    root = repo_root_from_script()
    parser = argparse.ArgumentParser(description="验证区域可见性/禁穿墙候选门控效果。")
    parser.add_argument("--workspace", type=Path, default=root, help="humanoid_ws 工作空间路径")
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"), help="全局 PCD 地图")
    parser.add_argument("--targets", type=Path, default=Path(".codex_tmp/official_std_standalone/nav46_hard52_dataset_acc10_fullz/targets.csv"), help="目标 scan CSV")
    parser.add_argument("--candidates", type=Path, default=Path(".codex_tmp/triangle_descriptor/hard52_top20_candidates.csv"), help="候选明细 CSV")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/area_visibility_gate/hard52_candidate_details.csv"), help="候选门控明细输出")
    parser.add_argument("--summary-output", type=Path, default=Path(".codex_tmp/area_visibility_gate/hard52_summary.csv"), help="阈值 sweep 汇总输出")
    parser.add_argument("--max-targets", type=int, default=0, help="只验证前 N 个 target；0 表示全量")
    parser.add_argument("--random-seed", type=int, default=7, help="固定随机采样种子，保证结果可复现")

    parser.add_argument("--map-voxel-size", type=float, default=0.12, help="地图降采样体素尺寸，越小越精细但 KDTree 和栅格更慢")
    parser.add_argument("--map-min-z", type=float, default=0.15, help="参与障碍判断的地图最低高度，过滤地面点")
    parser.add_argument("--map-max-z", type=float, default=2.20, help="参与障碍判断的地图最高高度，过滤天花板/高噪声")
    parser.add_argument("--occupancy-resolution", type=float, default=0.18, help="禁穿墙二维占据栅格分辨率")

    parser.add_argument("--scan-voxel-size", type=float, default=0.18, help="目标 scan 降采样体素尺寸")
    parser.add_argument("--max-scan-points", type=int, default=650, help="端点贴图计算最多使用的 scan 点数")
    parser.add_argument("--ray-sample-points", type=int, default=120, help="禁穿墙射线抽样点数，越大越严格但越慢")
    parser.add_argument("--min-range", type=float, default=0.35, help="忽略过近点，减少机器人自身和近场噪声影响")
    parser.add_argument("--max-range", type=float, default=18.0, help="参与可见性判断的最大 scan 半径")
    parser.add_argument("--scan-min-z", type=float, default=-1.0, help="目标 scan 最低高度")
    parser.add_argument("--scan-max-z", type=float, default=2.2, help="目标 scan 最高高度")

    parser.add_argument("--endpoint-near-distance", type=float, default=0.45, help="scan 端点距离地图表面小于该值认为贴图成功")
    parser.add_argument("--endpoint-near-reference", type=float, default=0.45, help="可见性排序中期望的端点贴图比例")
    parser.add_argument("--ray-step", type=float, default=0.22, help="禁穿墙射线采样步长")
    parser.add_argument("--wall-before-end-tolerance", type=float, default=0.70, help="端点前该距离内的占据不算提前撞墙，避免把真实墙面端点误判为穿墙")
    parser.add_argument("--origin-clearance-radius", type=float, default=0.25, help="候选站位周围障碍清空半径")
    parser.add_argument("--require-origin-clear", action="store_true", help="启用后候选站位必须通过障碍清空检查")

    parser.add_argument("--wall-penalty-weight", type=float, default=0.4, help="排序分数中穿墙比例的惩罚权重")
    parser.add_argument("--endpoint-penalty-weight", type=float, default=0.0, help="排序分数中端点贴图不足的惩罚权重")
    parser.add_argument("--endpoint-reward-weight", type=float, default=0.8, help="排序分数中端点贴图比例的奖励权重")
    parser.add_argument("--origin-blocked-penalty", type=float, default=0.0, help="排序分数中站位被占据的惩罚；当前默认只诊断不惩罚，避免稠密墙体误伤正确位姿")
    parser.add_argument("--min-near-sweep", default="0.25,0.35,0.45,0.55", help="端点贴图比例阈值 sweep，逗号分隔")
    parser.add_argument("--max-wall-sweep", default="0.05,0.10,0.20,0.30", help="最大穿墙比例阈值 sweep，逗号分隔")
    return parser.parse_args()


def main() -> int:
    """脚本入口：执行评估、写出 CSV，并在终端打印最佳几组阈值。"""

    args = parse_args()
    start_time = time.time()
    start_cpu = current_cpu_seconds()
    details, summary = evaluate_all(args)
    output = args.output if args.output.is_absolute() else args.workspace.resolve() / args.output
    summary_output = args.summary_output if args.summary_output.is_absolute() else args.workspace.resolve() / args.summary_output
    write_candidate_details(output, details)
    write_summary(summary_output, summary)

    ranked = sorted(
        summary,
        key=lambda row: (
            -int(row["success_0p3m_5deg"]),
            -int(row["success_0p2m_3deg"]),
            int(row["rejected"]),
            float(row["median_xy_error_m"]) if row["median_xy_error_m"] != "nan" else 999.0,
        ),
    )
    print(f"[area_visibility_gate] details={output}")
    print(f"[area_visibility_gate] summary={summary_output}")
    elapsed_sec = time.time() - start_time
    cpu_core_equiv = (current_cpu_seconds() - start_cpu) / max(elapsed_sec, 1e-6)
    print(
        f"[area_visibility_gate] elapsed_sec={elapsed_sec:.3f} "
        f"cpu_core_equiv={cpu_core_equiv:.2f} rss_mb={current_rss_mb():.1f}"
    )
    for row in ranked[:5]:
        print(
            "[area_visibility_gate] "
            f"min_near={row['min_endpoint_near_ratio']} max_wall={row['max_ray_wall_cross_ratio']} "
            f"accepted={row['accepted']}/{row['targets']} "
            f"0.2m3deg={row['success_0p2m_3deg']} "
            f"0.3m5deg={row['success_0p3m_5deg']} "
            f"0.5m10deg={row['success_0p5m_10deg']} "
            f"median={row['median_xy_error_m']}m/{row['median_yaw_error_deg']}deg"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
