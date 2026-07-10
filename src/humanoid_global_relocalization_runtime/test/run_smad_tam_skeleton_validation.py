#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_smad_tam_skeleton_validation.py

文件作用：
  1. 验证“Sparse feasible hypothesis + SMAD/TAM beam-level matching + structural skeleton score”的被动全局重定位效果。
  2. 从 PCD 地图投影 2D occupancy，在可通行区域生成稀疏候选位置，并为每个候选预计算 360 度虚拟 LiDAR range。
  3. 对真实 bag scan 生成 2D range observation，先用 SMAD 在全局候选/yaw 上粗排，再用 TAM 在 top seed 周围做平移/yaw 局部重评估。
  4. 从地图提取 Hough/skeleton 结构线，作为 TAM 并列候选的结构一致性加分项。
  5. 输出 top1 和 topK 成功率，用来判断它是否能作为当前 BBS2D/GICP 的补充召回/消歧层。

重要说明：
  - 真值只用于统计，不参与候选评分。
  - 该脚本不注入 initialpose，不修改线上节点。
  - 这是离线完整算法验证：候选 DB 来自地图，查询 scan 来自真实 bag。
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
import resource
import time

import cv2
import numpy as np
import open3d as o3d

from run_alignment_risk_gate_validation import (
    MetricSample,
    current_cpu_seconds,
    current_rss_mb,
    load_metric_samples,
    read_bag_clouds,
)
from run_scan_context_keyframe_recall import nearest_odom, registered_world_cloud_to_body
from run_structural_map_gate_validation import (
    StructuralMap,
    build_structural_map,
    load_map_points,
    normalize_angle_180,
    orientation_histogram,
    local_map_lines,
    cosine_similarity,
)


@dataclass(frozen=True)
class OccupancyGrid:
    """二维占据/可通行栅格。"""

    origin_x: float
    origin_y: float
    resolution: float
    occupied: np.ndarray
    feasible: np.ndarray


@dataclass(frozen=True)
class VirtualRangeDatabase:
    """地图候选位置和对应虚拟 2D LiDAR range。"""

    candidates_xy: np.ndarray
    ranges: np.ndarray
    hit_counts: np.ndarray


@dataclass(frozen=True)
class Hypothesis:
    """一个 2D map->base 候选假设。"""

    x: float
    y: float
    yaw_deg: float
    smad: float
    tam_cost: float
    structure_score: float
    final_score: float
    matched_beams: int


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


def yaw_error_deg(candidate: float, reference: float) -> float:
    """计算 yaw 绝对误差。"""

    return abs(normalize_angle_deg(candidate - reference))


def build_occupancy_grid(points: np.ndarray, args: argparse.Namespace) -> OccupancyGrid:
    """从地图点云构建二维占据图和可通行区域。"""

    margin = args.max_range + 2.0
    min_xy = points[:, :2].min(axis=0) - margin
    max_xy = points[:, :2].max(axis=0) + margin
    size = np.ceil((max_xy - min_xy) / args.occupancy_resolution).astype(np.int32) + 1
    occupied = np.zeros((int(size[1]), int(size[0])), dtype=np.uint8)
    ix = np.floor((points[:, 0] - min_xy[0]) / args.occupancy_resolution).astype(np.int32)
    iy = np.floor((points[:, 1] - min_xy[1]) / args.occupancy_resolution).astype(np.int32)
    valid = (ix >= 0) & (iy >= 0) & (ix < size[0]) & (iy < size[1])
    occupied[iy[valid], ix[valid]] = 1

    inflated = cv2.dilate(
        occupied,
        np.ones((max(1, int(math.ceil(args.robot_clearance_radius / args.occupancy_resolution))),) * 2, dtype=np.uint8),
        iterations=1,
    )
    free = (inflated == 0).astype(np.uint8)
    # 距离障碍太远的外部空白也不认为是可通行区域；要求周围能看到一定数量障碍。
    obstacle_distance = cv2.distanceTransform(free, cv2.DIST_L2, 3) * args.occupancy_resolution
    near_structure = obstacle_distance <= args.max_feasible_obstacle_distance
    feasible = (free.astype(bool) & near_structure).astype(np.uint8)
    return OccupancyGrid(float(min_xy[0]), float(min_xy[1]), args.occupancy_resolution, occupied.astype(bool), feasible.astype(bool))


def world_to_grid(grid: OccupancyGrid, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """map 坐标转栅格索引。"""

    ix = np.floor((x - grid.origin_x) / grid.resolution).astype(np.int32)
    iy = np.floor((y - grid.origin_y) / grid.resolution).astype(np.int32)
    inside = (ix >= 0) & (iy >= 0) & (ix < grid.occupied.shape[1]) & (iy < grid.occupied.shape[0])
    return ix, iy, inside


def is_feasible(grid: OccupancyGrid, x: float, y: float) -> bool:
    """判断世界坐标是否落在可通行区域。"""

    ix, iy, inside = world_to_grid(grid, np.asarray([x]), np.asarray([y]))
    return bool(inside[0] and grid.feasible[iy[0], ix[0]])


def precompute_ray_offsets(args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """预计算 2D beam 方向和距离采样。"""

    angles = np.linspace(-math.pi, math.pi, args.sectors, endpoint=False, dtype=np.float64)
    distances = np.arange(args.min_range, args.max_range + 1e-9, args.ray_step, dtype=np.float64)
    return angles, np.cos(angles)[:, None] * distances[None, :], np.sin(angles)[:, None] * distances[None, :]


def raycast_ranges(grid: OccupancyGrid, x: float, y: float, offsets_x: np.ndarray, offsets_y: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """在 occupancy grid 上模拟 360 度首次命中 range。"""

    distances = np.arange(args.min_range, args.max_range + 1e-9, args.ray_step, dtype=np.float64)
    sample_x = x + offsets_x
    sample_y = y + offsets_y
    ix, iy, inside = world_to_grid(grid, sample_x, sample_y)
    hit = np.zeros_like(inside, dtype=bool)
    hit[inside] = grid.occupied[iy[inside], ix[inside]]
    any_hit = hit.any(axis=1)
    first = np.argmax(hit, axis=1)
    ranges = np.full(hit.shape[0], np.nan, dtype=np.float32)
    ranges[any_hit] = distances[first[any_hit]].astype(np.float32)
    return ranges


def generate_feasible_candidates(grid: OccupancyGrid, args: argparse.Namespace) -> np.ndarray:
    """在可通行栅格上按固定间距生成稀疏候选位置。"""

    ys, xs = np.nonzero(grid.feasible)
    if len(xs) == 0:
        return np.empty((0, 2), dtype=np.float32)
    stride = max(1, int(round(args.candidate_grid_size / grid.resolution)))
    selected = ((xs % stride) == 0) & ((ys % stride) == 0)
    xs = xs[selected]
    ys = ys[selected]
    xy = np.column_stack((grid.origin_x + xs.astype(np.float64) * grid.resolution, grid.origin_y + ys.astype(np.float64) * grid.resolution))
    return xy.astype(np.float32)


def build_virtual_database(map_points: np.ndarray, grid: OccupancyGrid, args: argparse.Namespace) -> VirtualRangeDatabase:
    """构建或加载虚拟 range 数据库。"""

    if args.database_cache.exists() and not args.rebuild_database:
        data = np.load(args.database_cache)
        return VirtualRangeDatabase(data["candidates_xy"], data["ranges"], data["hit_counts"])

    _, offsets_x, offsets_y = precompute_ray_offsets(args)
    candidates_xy = generate_feasible_candidates(grid, args)
    ranges = np.zeros((len(candidates_xy), args.sectors), dtype=np.float32)
    hit_counts = np.zeros(len(candidates_xy), dtype=np.int32)
    for i, (x, y) in enumerate(candidates_xy):
        r = raycast_ranges(grid, float(x), float(y), offsets_x, offsets_y, args)
        ranges[i] = np.nan_to_num(r, nan=0.0).astype(np.float32)
        hit_counts[i] = int(np.isfinite(r).sum())
        if i % max(1, len(candidates_xy) // 10) == 0:
            print(f"[smad_tam] db progress {i}/{len(candidates_xy)}", flush=True)
    keep = hit_counts >= args.min_virtual_hits
    candidates_xy = candidates_xy[keep]
    ranges = ranges[keep]
    hit_counts = hit_counts[keep]
    args.database_cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.database_cache, candidates_xy=candidates_xy, ranges=ranges, hit_counts=hit_counts)
    return VirtualRangeDatabase(candidates_xy, ranges, hit_counts)


def query_ranges_from_points(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """把真实 scan 压成每个 sector 的最近 range。"""

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
    radius = radius[valid]
    ranges = np.full(args.sectors, np.nan, dtype=np.float32)
    if len(points) == 0:
        return ranges
    theta = (np.arctan2(points[:, 1], points[:, 0]) + math.pi) / (2.0 * math.pi)
    sector = np.minimum(args.sectors - 1, np.floor(theta * args.sectors).astype(np.int32))
    tmp = np.full(args.sectors, np.inf, dtype=np.float32)
    np.minimum.at(tmp, sector, radius.astype(np.float32))
    tmp[~np.isfinite(tmp)] = np.nan
    return tmp


def smad_scores(query: np.ndarray, db: VirtualRangeDatabase, args: argparse.Namespace) -> list[Hypothesis]:
    """SMAD：对全局候选位置和离散 yaw shift 计算 clipped mean absolute range difference。"""

    hypotheses: list[Hypothesis] = []
    query_valid = np.isfinite(query)
    if query_valid.sum() < args.min_query_hits:
        return hypotheses
    db_ranges = np.where(db.ranges > 0.0, db.ranges, np.nan)
    for shift in range(args.sectors):
        expected = np.roll(db_ranges, shift, axis=1)
        valid = query_valid[None, :] & np.isfinite(expected)
        matched = valid.sum(axis=1)
        diff = np.abs(expected - query[None, :])
        diff = np.where(valid, np.minimum(diff, args.smad_clip_m), 0.0)
        cost = diff.sum(axis=1) / np.maximum(matched, 1)
        coverage_penalty = args.coverage_penalty * (1.0 - matched / max(1, query_valid.sum()))
        score = cost + coverage_penalty
        top_n = min(args.smad_keep_per_yaw, len(score))
        if top_n <= 0:
            continue
        idx = np.argpartition(score, top_n - 1)[:top_n]
        for i in idx:
            if matched[i] < args.min_matched_beams:
                continue
            yaw = normalize_angle_deg(-float(shift) * 360.0 / float(args.sectors))
            hypotheses.append(
                Hypothesis(
                    x=float(db.candidates_xy[i, 0]),
                    y=float(db.candidates_xy[i, 1]),
                    yaw_deg=yaw,
                    smad=float(score[i]),
                    tam_cost=float(score[i]),
                    structure_score=0.0,
                    final_score=float(score[i]),
                    matched_beams=int(matched[i]),
                )
            )
    hypotheses.sort(key=lambda h: h.smad)
    return deduplicate_hypotheses(hypotheses, args)[: args.smad_top_k]


def deduplicate_hypotheses(items: list[Hypothesis], args: argparse.Namespace) -> list[Hypothesis]:
    """去除几乎相同的候选。"""

    selected: list[Hypothesis] = []
    for item in items:
        duplicate = False
        for kept in selected:
            if (
                math.hypot(item.x - kept.x, item.y - kept.y) <= args.duplicate_xy_m
                and yaw_error_deg(item.yaw_deg, kept.yaw_deg) <= args.duplicate_yaw_deg
            ):
                duplicate = True
                break
        if not duplicate:
            selected.append(item)
    return selected


def tam_cost(query: np.ndarray, expected: np.ndarray, args: argparse.Namespace) -> tuple[float, int]:
    """TAM：使用截断距离、覆盖率和穿墙/过远不一致惩罚评价 beam-level 似然。"""

    expected = np.where(expected > 0.0, expected, np.nan)
    valid = np.isfinite(query) & np.isfinite(expected)
    if valid.sum() < args.min_matched_beams:
        return 999.0, int(valid.sum())
    diff = np.abs(query[valid] - expected[valid])
    clipped = np.minimum(diff, args.tam_clip_m)
    robust = float(np.mean(clipped))
    coverage = float(valid.sum()) / float(max(1, np.isfinite(query).sum()))
    wall = float(np.mean((query[valid] - expected[valid]) > args.wall_penetration_tolerance))
    free = float(np.mean((expected[valid] - query[valid]) > args.free_space_tolerance))
    return robust + args.tam_coverage_weight * (1.0 - coverage) + args.tam_wall_weight * wall + args.tam_free_weight * free, int(valid.sum())


def candidate_structure_score(h: Hypothesis, structural_map: StructuralMap, args: argparse.Namespace) -> float:
    """根据候选附近地图骨架方向丰富度给并列候选加分，避免空白/弱结构区过度自信。"""

    lines = local_map_lines(structural_map, h.x, h.y, args.structure_radius_m)
    if not lines:
        return 0.0
    hist = orientation_histogram(lines, args.structure_orientation_bins)
    nonzero = np.count_nonzero(hist > args.structure_bin_min_weight)
    length_sum = sum(line.length_m for line in lines)
    diversity = min(1.0, float(nonzero) / 4.0)
    strength = min(1.0, length_sum / max(args.structure_length_scale_m, 1e-6))
    return 0.6 * diversity + 0.4 * strength


def tam_refine(
    query: np.ndarray,
    seeds: list[Hypothesis],
    grid: OccupancyGrid,
    structural_map: StructuralMap,
    args: argparse.Namespace,
) -> list[Hypothesis]:
    """对 SMAD top seed 周围做局部平移/yaw TAM 重评估。"""

    _, offsets_x, offsets_y = precompute_ray_offsets(args)
    refined: list[Hypothesis] = []
    xy_offsets = np.arange(-args.tam_xy_radius_m, args.tam_xy_radius_m + 1e-9, args.tam_xy_step_m)
    yaw_offsets = np.arange(-args.tam_yaw_radius_deg, args.tam_yaw_radius_deg + 1e-9, args.tam_yaw_step_deg)
    for seed in seeds[: args.tam_seed_top_k]:
        for dx in xy_offsets:
            for dy in xy_offsets:
                x = seed.x + float(dx)
                y = seed.y + float(dy)
                if not is_feasible(grid, x, y):
                    continue
                base_ranges = raycast_ranges(grid, x, y, offsets_x, offsets_y, args)
                for dyaw in yaw_offsets:
                    yaw = normalize_angle_deg(seed.yaw_deg + float(dyaw))
                    shift = int(round(-yaw / (360.0 / float(args.sectors)))) % args.sectors
                    expected = np.roll(np.nan_to_num(base_ranges, nan=0.0), shift)
                    cost, matched = tam_cost(query, expected, args)
                    if cost >= 900.0:
                        continue
                    h = Hypothesis(
                        x=x,
                        y=y,
                        yaw_deg=yaw,
                        smad=seed.smad,
                        tam_cost=cost,
                        structure_score=0.0,
                        final_score=cost,
                        matched_beams=matched,
                    )
                    structure = candidate_structure_score(h, structural_map, args)
                    final_score = cost - args.structure_weight * structure + args.smad_weight * seed.smad
                    refined.append(
                        Hypothesis(
                            x=x,
                            y=y,
                            yaw_deg=yaw,
                            smad=seed.smad,
                            tam_cost=cost,
                            structure_score=structure,
                            final_score=final_score,
                            matched_beams=matched,
                        )
                    )
    refined.sort(key=lambda h: h.final_score)
    return deduplicate_hypotheses(refined, args)[: args.output_top_k]


def pose_errors(h: Hypothesis, sample: MetricSample) -> tuple[float, float]:
    """计算候选相对真值的 xy/yaw 误差。"""

    return math.hypot(h.x - sample.reference_x, h.y - sample.reference_y), yaw_error_deg(h.yaw_deg, sample.reference_yaw_deg)


def run(args: argparse.Namespace) -> int:
    """主流程：构建 DB、读取 bag、逐帧 SMAD/TAM/骨架评分。"""

    start_wall = time.time()
    start_cpu = current_cpu_seconds()
    workspace = args.workspace.resolve()
    map_path = args.map if args.map.is_absolute() else workspace / args.map
    if not args.database_cache.is_absolute():
        args.database_cache = workspace / args.database_cache

    samples = load_metric_samples(args.metrics_glob, args.scenario_name, args.max_samples)
    if not samples:
        raise RuntimeError("没有读取到 metrics 样本")
    bags = sorted({s.bag_path for s in samples})
    if len(bags) != 1:
        raise RuntimeError(f"当前只支持单 bag，实际={bags}")
    target_indices = {s.bag_frame_index for s in samples}
    print(f"[smad_tam] samples={len(samples)} unique_frames={len(target_indices)}", flush=True)

    map_points = load_map_points(map_path, args)
    grid = build_occupancy_grid(map_points, args)
    structural_map = build_structural_map(map_points, args)
    db = build_virtual_database(map_points, grid, args)
    print(
        f"[smad_tam] map_points={len(map_points)} feasible_candidates={len(db.candidates_xy)} "
        f"structure_lines={len(structural_map.lines)} rss_mb={current_rss_mb():.1f}",
        flush=True,
    )

    odoms, clouds = read_bag_clouds(bags[0], target_indices)
    print(f"[smad_tam] loaded_clouds={len(clouds)} odoms={len(odoms)}", flush=True)

    details: list[dict[str, str]] = []
    summary_counts = {
        "top1_0p2m_3deg": 0,
        "top1_0p3m_5deg": 0,
        "top1_0p5m_10deg": 0,
        "topk_0p2m_3deg": 0,
        "topk_0p3m_5deg": 0,
        "topk_0p5m_10deg": 0,
        "processed": 0,
    }
    for idx, sample in enumerate(samples):
        cloud = clouds.get(sample.bag_frame_index)
        if cloud is None:
            continue
        odom = nearest_odom(odoms, cloud.stamp_sec)
        points = registered_world_cloud_to_body(cloud, odom)
        query = query_ranges_from_points(points, args)
        smad = smad_scores(query, db, args)
        refined = tam_refine(query, smad, grid, structural_map, args)
        if not refined:
            continue
        summary_counts["processed"] += 1
        top1 = refined[0]
        top1_xy, top1_yaw = pose_errors(top1, sample)
        topk_errors = [pose_errors(h, sample) for h in refined[: args.output_top_k]]
        summary_counts["top1_0p2m_3deg"] += int(top1_xy <= 0.2 and top1_yaw <= 3.0)
        summary_counts["top1_0p3m_5deg"] += int(top1_xy <= 0.3 and top1_yaw <= 5.0)
        summary_counts["top1_0p5m_10deg"] += int(top1_xy <= 0.5 and top1_yaw <= 10.0)
        summary_counts["topk_0p2m_3deg"] += int(any(xy <= 0.2 and yaw <= 3.0 for xy, yaw in topk_errors))
        summary_counts["topk_0p3m_5deg"] += int(any(xy <= 0.3 and yaw <= 5.0 for xy, yaw in topk_errors))
        summary_counts["topk_0p5m_10deg"] += int(any(xy <= 0.5 and yaw <= 10.0 for xy, yaw in topk_errors))
        for rank, h in enumerate(refined[: args.output_top_k], start=1):
            xy, yaw = pose_errors(h, sample)
            details.append(
                {
                    "source_id": sample.source_id,
                    "bag_frame_index": str(sample.bag_frame_index),
                    "rank": str(rank),
                    "x": f"{h.x:.6f}",
                    "y": f"{h.y:.6f}",
                    "yaw_deg": f"{h.yaw_deg:.6f}",
                    "smad": f"{h.smad:.6f}",
                    "tam_cost": f"{h.tam_cost:.6f}",
                    "structure_score": f"{h.structure_score:.6f}",
                    "final_score": f"{h.final_score:.6f}",
                    "matched_beams": str(h.matched_beams),
                    "reference_x": f"{sample.reference_x:.6f}",
                    "reference_y": f"{sample.reference_y:.6f}",
                    "reference_yaw_deg": f"{sample.reference_yaw_deg:.6f}",
                    "translation_error_m": f"{xy:.6f}",
                    "yaw_error_deg": f"{yaw:.6f}",
                    "success_0p2m_3deg": "1" if xy <= 0.2 and yaw <= 3.0 else "0",
                    "success_0p3m_5deg": "1" if xy <= 0.3 and yaw <= 5.0 else "0",
                    "success_0p5m_10deg": "1" if xy <= 0.5 and yaw <= 10.0 else "0",
                }
            )
        if (idx + 1) % max(1, args.progress_interval) == 0:
            print(f"[smad_tam] progress {idx + 1}/{len(samples)}", flush=True)

    output = args.output if args.output.is_absolute() else workspace / args.output
    summary_output = args.summary_output if args.summary_output.is_absolute() else workspace / args.summary_output
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(details[0].keys()) if details else [])
        writer.writeheader()
        writer.writerows(details)

    processed = max(1, summary_counts["processed"])
    summary_rows = []
    for key in [
        "top1_0p2m_3deg",
        "top1_0p3m_5deg",
        "top1_0p5m_10deg",
        "topk_0p2m_3deg",
        "topk_0p3m_5deg",
        "topk_0p5m_10deg",
    ]:
        summary_rows.append({"metric": key, "success": str(summary_counts[key]), "total": str(summary_counts["processed"]), "rate": f"{summary_counts[key] / processed:.6f}"})
    summary_output.parent.mkdir(parents=True, exist_ok=True)
    with summary_output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["metric", "success", "total", "rate"])
        writer.writeheader()
        writer.writerows(summary_rows)

    elapsed = time.time() - start_wall
    cpu_cores = (current_cpu_seconds() - start_cpu) / max(elapsed, 1e-6)
    print(f"[smad_tam] details={output}", flush=True)
    print(f"[smad_tam] summary={summary_output}", flush=True)
    print(f"[smad_tam] elapsed_sec={elapsed:.3f} cpu_core_equiv={cpu_cores:.2f} rss_mb={current_rss_mb():.1f}", flush=True)
    for row in summary_rows:
        print(f"[smad_tam] {row['metric']}={row['success']}/{row['total']} rate={row['rate']}", flush=True)
    return 0


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""

    root = repo_root_from_script()
    parser = argparse.ArgumentParser(description="验证 SMAD/TAM + skeleton 被动全局重定位。")
    parser.add_argument("--workspace", type=Path, default=root, help="humanoid_ws 工作空间")
    parser.add_argument("--metrics-glob", action="append", default=[], help="输入 metrics CSV glob")
    parser.add_argument("--scenario-name", default="arbitrary_start_no_prior", help="验证场景")
    parser.add_argument("--max-samples", type=int, default=0, help="最多验证样本数")
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"), help="地图 PCD")
    parser.add_argument("--database-cache", type=Path, default=Path(".codex_tmp/smad_tam_skeleton/virtual_range_db.npz"), help="虚拟 range DB 缓存")
    parser.add_argument("--rebuild-database", action="store_true", help="强制重建虚拟 DB")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/smad_tam_skeleton/details.csv"), help="候选明细")
    parser.add_argument("--summary-output", type=Path, default=Path(".codex_tmp/smad_tam_skeleton/summary.csv"), help="汇总输出")
    parser.add_argument("--progress-interval", type=int, default=10, help="进度间隔")

    parser.add_argument("--map-voxel-size", type=float, default=0.10, help="地图降采样")
    parser.add_argument("--map-min-z", type=float, default=0.15, help="地图最低高度")
    parser.add_argument("--map-max-z", type=float, default=2.20, help="地图最高高度")
    parser.add_argument("--occupancy-resolution", type=float, default=0.12, help="occupancy 分辨率")
    parser.add_argument("--candidate-grid-size", type=float, default=0.45, help="候选位置间距")
    parser.add_argument("--robot-clearance-radius", type=float, default=0.28, help="机器人清障半径")
    parser.add_argument("--max-feasible-obstacle-distance", type=float, default=9.0, help="距离障碍太远的外部空白剔除")

    parser.add_argument("--sectors", type=int, default=72, help="beam 数")
    parser.add_argument("--min-range", type=float, default=0.35, help="最小 range")
    parser.add_argument("--max-range", type=float, default=18.0, help="最大 range")
    parser.add_argument("--ray-step", type=float, default=0.12, help="raycast 步长")
    parser.add_argument("--scan-min-z", type=float, default=-1.0, help="scan 最低 z")
    parser.add_argument("--scan-max-z", type=float, default=2.20, help="scan 最高 z")
    parser.add_argument("--min-virtual-hits", type=int, default=20, help="虚拟候选最少命中 beam")
    parser.add_argument("--min-query-hits", type=int, default=20, help="真实查询最少命中 beam")
    parser.add_argument("--min-matched-beams", type=int, default=18, help="候选最少匹配 beam")

    parser.add_argument("--smad-clip-m", type=float, default=4.0, help="SMAD 截断距离")
    parser.add_argument("--coverage-penalty", type=float, default=1.0, help="SMAD 覆盖率惩罚")
    parser.add_argument("--smad-keep-per-yaw", type=int, default=20, help="每个 yaw 保留候选数")
    parser.add_argument("--smad-top-k", type=int, default=120, help="SMAD 全局保留候选数")
    parser.add_argument("--tam-seed-top-k", type=int, default=40, help="TAM 输入 seed 数")
    parser.add_argument("--tam-xy-radius-m", type=float, default=0.45, help="TAM 平移搜索半径")
    parser.add_argument("--tam-xy-step-m", type=float, default=0.225, help="TAM 平移搜索步长")
    parser.add_argument("--tam-yaw-radius-deg", type=float, default=7.5, help="TAM yaw 搜索半径")
    parser.add_argument("--tam-yaw-step-deg", type=float, default=2.5, help="TAM yaw 搜索步长")
    parser.add_argument("--tam-clip-m", type=float, default=3.0, help="TAM 截断距离")
    parser.add_argument("--wall-penetration-tolerance", type=float, default=0.8, help="穿墙/提前命中容差")
    parser.add_argument("--free-space-tolerance", type=float, default=1.2, help="空闲空间过远容差")
    parser.add_argument("--tam-coverage-weight", type=float, default=1.0, help="TAM 覆盖惩罚")
    parser.add_argument("--tam-wall-weight", type=float, default=0.8, help="TAM 穿墙惩罚")
    parser.add_argument("--tam-free-weight", type=float, default=0.3, help="TAM 空闲空间惩罚")
    parser.add_argument("--smad-weight", type=float, default=0.15, help="最终分数中的 SMAD 权重")

    parser.add_argument("--structure-radius-m", type=float, default=9.0, help="候选附近 skeleton 半径")
    parser.add_argument("--structure-weight", type=float, default=0.15, help="最终分数中的结构加分权重")
    parser.add_argument("--structure-orientation-bins", type=int, default=18, help="结构方向直方图 bins")
    parser.add_argument("--structure-bin-min-weight", type=float, default=0.05, help="有效方向 bin 阈值")
    parser.add_argument("--structure-length-scale-m", type=float, default=40.0, help="结构长度归一化尺度")
    parser.add_argument("--structural-resolution", type=float, default=0.10, help="skeleton 投影分辨率")
    parser.add_argument("--map-margin-m", type=float, default=2.0, help="skeleton 地图外扩边界")
    parser.add_argument("--map-close-kernel", type=int, default=3, help="skeleton 地图闭运算核")
    parser.add_argument("--map-dilate-kernel", type=int, default=2, help="skeleton 地图膨胀核")
    parser.add_argument("--hough-threshold", type=int, default=18, help="HoughLinesP 阈值")
    parser.add_argument("--hough-max-gap-m", type=float, default=0.45, help="Hough 最大连接间隙")
    parser.add_argument("--map-min-line-length-m", type=float, default=1.2, help="地图最短结构线")
    parser.add_argument("--merge-angle-deg", type=float, default=6.0, help="结构线合并角度")
    parser.add_argument("--merge-distance-m", type=float, default=0.25, help="结构线合并垂距")
    parser.add_argument("--merge-midpoint-m", type=float, default=1.0, help="结构线合并中点距离")

    parser.add_argument("--duplicate-xy-m", type=float, default=0.35, help="候选去重 xy 阈值")
    parser.add_argument("--duplicate-yaw-deg", type=float, default=5.0, help="候选去重 yaw 阈值")
    parser.add_argument("--output-top-k", type=int, default=20, help="输出 top-K")

    if not parser.parse_known_args()[0].metrics_glob:
        parser.set_defaults(metrics_glob=[str(root / ".codex_tmp/bbs2d_integrated_random100/rand100_seed*/global_relocalization_metrics.csv")])
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
