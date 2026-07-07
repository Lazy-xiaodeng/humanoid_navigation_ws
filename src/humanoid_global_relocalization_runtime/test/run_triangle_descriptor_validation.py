#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_triangle_descriptor_validation.py

文件作用：
  1. 离线验证“STD/BTC 思路启发的三角几何描述子召回 + GICP 精配准”是否能补强冷启动全局重定位。
  2. 从 bag 中抽样历史 keyframe scan，提取稳定空间关键点，再把关键点三元组的边长量化成旋转/平移不敏感的三角指纹。
  3. 对目标帧计算同样的三角指纹，召回 top-K keyframe 位姿作为 seed，并通过 yaw 多假设 + GICP 做最终几何配准。
  4. 输出严格阈值下的误差、资源消耗和候选明细，用来判断后续是否值得把该方法移植进 C++ 运行态第三阶段。

重要说明：
  - 这是 STD 风格的工程验证原型，不是 HKU-MARS STD/BTC 代码的逐行移植。
  - 三角描述子只负责“候选召回”，最终是否可自动恢复仍要看 GICP 后的 0.2m/3deg、0.3m/5deg、0.5m/10deg 等门控。
  - 默认读取 /fast_lio/cloud_registered，并复用现有 adapter 规则转成 base 坐标，保证和前面 Scan Context 验证口径一致。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_triangle_descriptor_validation.py \
    --metrics-csv .codex_tmp/nav46_random_strict_validation/scan_context_db_loaded_hard52/global_relocalization_metrics.csv \
    --output .codex_tmp/triangle_descriptor/hard52.csv
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import math
from dataclasses import dataclass
from itertools import combinations
from pathlib import Path
import resource
import time

import numpy as np
import open3d as o3d

from run_descriptor_fusion_validation import (
    crop_scan_points,
    load_map_cloud,
    make_o3d_cloud,
    normalize_angle_deg,
    pose_matrix,
    run_gicp,
    yaw_from_matrix,
)
from run_scan_context_keyframe_recall import (
    CloudSample,
    OdomSample,
    ReferenceSample,
    Target,
    nearest_odom,
    nearest_reference,
    read_bag_samples,
    read_targets,
    registered_world_cloud_to_body,
    repo_root_from_script,
)


Histogram = dict[str, int]


@dataclass(frozen=True)
class TriangleEntry:
    """三角描述子数据库中的一个 keyframe 条目。"""

    cloud_index: int
    reference_x: float
    reference_y: float
    reference_yaw_deg: float
    histogram: Histogram
    keypoint_count: int
    triangle_count: int


def encode_triangle_key(length_bins: tuple[int, int, int]) -> str:
    """把三条边的量化长度编码成稳定字符串，便于 JSON/CSV 缓存。"""

    return f"{length_bins[0]}:{length_bins[1]}:{length_bins[2]}"


def current_rss_mb() -> float:
    """读取当前进程最大 RSS；Linux ru_maxrss 单位是 KB。"""

    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def read_targets_from_failure_csv(failure_csv: Path, indices_path: Path) -> list[Target]:
    """把 hard failure CSV 和对应 cloud index 列表合成 Target，避免人工维护中间 matches 文件。"""

    indices = [int(line.strip()) for line in indices_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    targets: list[Target] = []
    with failure_csv.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != len(indices):
        raise ValueError(f"failure rows={len(rows)} 与 indices={len(indices)} 数量不一致")
    for index, (row, cloud_index) in enumerate(zip(rows, indices), start=1):
        targets.append(
            Target(
                waypoint_id=f"hard_{index:03d}",
                name=f"{row['set']}_{row['sample_order']}",
                cloud_index=cloud_index,
                reference_x=float(row["reference_x_m"]),
                reference_y=float(row["reference_y_m"]),
                reference_yaw_deg=float(row["reference_yaw_deg"]),
            )
        )
    return targets


def read_targets_from_edge_selection(path: Path) -> list[Target]:
    """读取 edge/corner 选点 CSV，作为静止冷启动边角样本。"""

    targets: list[Target] = []
    with path.open(newline="", encoding="utf-8") as f:
        for index, row in enumerate(csv.DictReader(f), start=1):
            cloud_index = int(row["cloud_index"])
            targets.append(
                Target(
                    waypoint_id=f"edge_{index:03d}",
                    name=f"edge_corner_{cloud_index}",
                    cloud_index=cloud_index,
                    reference_x=float(row["ref_x"]),
                    reference_y=float(row["ref_y"]),
                    reference_yaw_deg=float(row["ref_yaw_deg"]),
                )
            )
    return targets


def read_targets_from_metrics_csv(path: Path) -> list[Target]:
    """读取 evaluator 输出的 metrics CSV；该文件自带 bag_frame_index 和真值，最不容易串样本。"""

    targets: list[Target] = []
    with path.open(newline="", encoding="utf-8") as f:
        for index, row in enumerate(csv.DictReader(f), start=1):
            cloud_index = int(row["bag_frame_index"])
            targets.append(
                Target(
                    waypoint_id=f"metric_{index:03d}",
                    name=f"{row.get('scenario_name', 'scenario')}_{cloud_index}",
                    cloud_index=cloud_index,
                    reference_x=float(row["reference_x_m"]),
                    reference_y=float(row["reference_y_m"]),
                    reference_yaw_deg=float(row["reference_yaw_deg"]),
                )
            )
    return targets


def load_targets(args: argparse.Namespace) -> list[Target]:
    """根据命令行选择目标来源，统一转换成现有验证工具使用的 Target。"""

    selected_ids = set(args.ids)
    targets: list[Target]
    if args.matches is not None:
        targets = read_targets(args.matches, selected_ids)
    elif args.metrics_csv is not None:
        targets = read_targets_from_metrics_csv(args.metrics_csv)
    elif args.failure_csv is not None and args.indices is not None:
        targets = read_targets_from_failure_csv(args.failure_csv, args.indices)
    elif args.edge_selection is not None:
        targets = read_targets_from_edge_selection(args.edge_selection)
    else:
        raise ValueError("必须提供 --matches，或 --metrics-csv，或 --failure-csv + --indices，或 --edge-selection")
    if selected_ids and args.matches is None:
        targets = [target for target in targets if target.waypoint_id in selected_ids]
    return targets


def crop_descriptor_points(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """按描述子专用规则裁剪点云，减少地面、近场和远场稀疏点对三角指纹的干扰。"""

    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    radius = np.hypot(points[:, 0], points[:, 1])
    valid = (
        (radius >= args.min_range)
        & (radius <= args.max_radius)
        & (points[:, 2] >= args.min_z)
        & (points[:, 2] <= args.max_z)
    )
    return points[valid]


def extract_keypoints(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """从点云中提取关键点：先 XY 栅格聚合，再按点数和空间离散度做简单稳定筛选。"""

    points = crop_descriptor_points(points, args)
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float64)

    # 使用 XY 体素聚类可以把墙面/边角的密集点压成少量代表点，降低三角组合爆炸。
    xy_bins = np.floor(points[:, :2] / args.keypoint_voxel_size).astype(np.int32)
    unique_bins, inverse = np.unique(xy_bins, axis=0, return_inverse=True)
    centroids: list[np.ndarray] = []
    scores: list[float] = []
    for bin_index in range(len(unique_bins)):
        mask = inverse == bin_index
        if int(mask.sum()) < args.min_keypoint_points:
            continue
        cluster = points[mask]
        centroid = cluster.mean(axis=0)
        z_span = float(cluster[:, 2].max() - cluster[:, 2].min())
        radial = float(np.hypot(centroid[0], centroid[1]))
        # 点数代表稳定性，z_span 对墙角/立面略加分，radial 轻微鼓励视野外圈结构。
        score = math.log1p(float(mask.sum())) + 0.25 * z_span + 0.02 * radial
        centroids.append(centroid)
        scores.append(score)
    if not centroids:
        return np.empty((0, 3), dtype=np.float64)

    candidates = np.asarray(centroids, dtype=np.float64)
    order = np.argsort(np.asarray(scores, dtype=np.float64))[::-1]
    selected: list[np.ndarray] = []
    for candidate_index in order:
        candidate = candidates[candidate_index]
        if selected:
            distances = np.linalg.norm(np.asarray(selected)[:, :2] - candidate[:2], axis=1)
            if float(distances.min()) < args.keypoint_nms_radius:
                continue
        selected.append(candidate)
        if len(selected) >= args.max_keypoints:
            break
    return np.asarray(selected, dtype=np.float64)


def triangle_histogram_from_keypoints(keypoints: np.ndarray, args: argparse.Namespace) -> Histogram:
    """把关键点三元组转成边长量化直方图；该表示对整体平移和旋转天然不敏感。"""

    if len(keypoints) < 3:
        return {}
    histogram: Counter[str] = Counter()
    triangle_limit = max(1, args.max_triangles)
    triangle_seen = 0
    for ia, ib, ic in combinations(range(len(keypoints)), 3):
        a = keypoints[ia]
        b = keypoints[ib]
        c = keypoints[ic]
        sides = sorted(
            [
                float(np.linalg.norm(a - b)),
                float(np.linalg.norm(a - c)),
                float(np.linalg.norm(b - c)),
            ]
        )
        if sides[0] < args.min_triangle_side or sides[2] > args.max_triangle_side:
            continue
        area_vector = np.cross(b - a, c - a)
        area = 0.5 * float(np.linalg.norm(area_vector))
        if area < args.min_triangle_area:
            continue
        bins = tuple(int(round(side / args.triangle_bin_size)) for side in sides)
        histogram[encode_triangle_key(bins)] += 1
        triangle_seen += 1
        if triangle_seen >= triangle_limit:
            break
    return dict(histogram)


def triangle_descriptor(points: np.ndarray, args: argparse.Namespace) -> tuple[Histogram, int, int]:
    """完整计算一个 scan 的三角描述子，并返回关键点数量和有效三角数量。"""

    keypoints = extract_keypoints(points, args)
    histogram = triangle_histogram_from_keypoints(keypoints, args)
    return histogram, int(len(keypoints)), int(sum(histogram.values()))


def histogram_similarity(query: Histogram, candidate: Histogram) -> float:
    """计算两个三角直方图的交并式相似度，越大表示结构越接近。"""

    if not query or not candidate:
        return 0.0
    shared = set(query).intersection(candidate)
    intersection = sum(min(query[key], candidate[key]) for key in shared)
    total = sum(query.values()) + sum(candidate.values()) - intersection
    return float(intersection) / float(max(1, total))


def save_database(path: Path, database: list[TriangleEntry]) -> None:
    """把三角描述子库保存为 JSON，便于参数扫验时复用。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    payload = [
        {
            "cloud_index": entry.cloud_index,
            "reference_x": entry.reference_x,
            "reference_y": entry.reference_y,
            "reference_yaw_deg": entry.reference_yaw_deg,
            "histogram": entry.histogram,
            "keypoint_count": entry.keypoint_count,
            "triangle_count": entry.triangle_count,
        }
        for entry in database
    ]
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


def load_database(path: Path) -> list[TriangleEntry]:
    """从 JSON 缓存读取三角描述子库。"""

    payload = json.loads(path.read_text(encoding="utf-8"))
    return [
        TriangleEntry(
            cloud_index=int(item["cloud_index"]),
            reference_x=float(item["reference_x"]),
            reference_y=float(item["reference_y"]),
            reference_yaw_deg=float(item["reference_yaw_deg"]),
            histogram={str(key): int(value) for key, value in item["histogram"].items()},
            keypoint_count=int(item["keypoint_count"]),
            triangle_count=int(item["triangle_count"]),
        )
        for item in payload
    ]


def build_triangle_database(
    keyframe_clouds: list[CloudSample],
    odoms: list[OdomSample],
    references: list[ReferenceSample],
    args: argparse.Namespace,
) -> list[TriangleEntry]:
    """把抽样 keyframe 点云转换成三角描述子数据库。"""

    entries: list[TriangleEntry] = []
    for sample in keyframe_clouds:
        odom = nearest_odom(odoms, sample.stamp_sec)
        reference = nearest_reference(references, sample.bag_time_sec)
        points = registered_world_cloud_to_body(sample, odom)
        histogram, keypoint_count, triangle_count = triangle_descriptor(points, args)
        entries.append(
            TriangleEntry(
                cloud_index=sample.cloud_index,
                reference_x=reference.x,
                reference_y=reference.y,
                reference_yaw_deg=reference.yaw_deg,
                histogram=histogram,
                keypoint_count=keypoint_count,
                triangle_count=triangle_count,
            )
        )
    return entries


def query_triangle_candidates(
    target: Target,
    query_histogram: Histogram,
    database: list[TriangleEntry],
    args: argparse.Namespace,
) -> list[tuple[float, TriangleEntry]]:
    """根据三角直方图召回 top-K keyframe 候选，并排除目标附近帧避免数据泄漏。"""

    scored: list[tuple[float, TriangleEntry]] = []
    for entry in database:
        if abs(entry.cloud_index - target.cloud_index) <= args.exclude_index_radius:
            continue
        similarity = histogram_similarity(query_histogram, entry.histogram)
        scored.append((similarity, entry))
    scored.sort(key=lambda item: item[0], reverse=True)
    return scored[: max(1, args.descriptor_top_k)]


def evaluate_target(
    target: Target,
    target_cloud: CloudSample,
    odoms: list[OdomSample],
    database: list[TriangleEntry],
    map_cloud: o3d.geometry.PointCloud,
    args: argparse.Namespace,
) -> tuple[dict[str, str], list[dict[str, str]]]:
    """对单个目标执行三角描述子召回、多 yaw 初值和 GICP 精配准。"""

    begin = time.perf_counter()
    odom = nearest_odom(odoms, target_cloud.stamp_sec)
    points = registered_world_cloud_to_body(target_cloud, odom)
    query_histogram, query_keypoints, query_triangles = triangle_descriptor(points, args)
    candidates = query_triangle_candidates(target, query_histogram, database, args)
    scan_points = crop_scan_points(points, args)
    scan_cloud = make_o3d_cloud(scan_points, args.scan_voxel_size)

    refined: list[dict[str, float | int]] = []
    candidate_rows: list[dict[str, str]] = []
    for descriptor_rank, (similarity, entry) in enumerate(candidates, start=1):
        for yaw_offset in args.yaw_offsets:
            init = pose_matrix(entry.reference_x, entry.reference_y, entry.reference_yaw_deg + yaw_offset)
            result = run_gicp(scan_cloud, map_cloud, init, args)
            pose = result.transformation
            final_yaw = yaw_from_matrix(pose)
            xy_error = math.hypot(float(pose[0, 3]) - target.reference_x, float(pose[1, 3]) - target.reference_y)
            yaw_error = abs(normalize_angle_deg(final_yaw - target.reference_yaw_deg))
            seed_drift_xy = math.hypot(float(pose[0, 3]) - entry.reference_x, float(pose[1, 3]) - entry.reference_y)
            seed_yaw = entry.reference_yaw_deg + float(yaw_offset)
            seed_drift_yaw = abs(normalize_angle_deg(final_yaw - seed_yaw))
            item = {
                "descriptor_rank": descriptor_rank,
                "similarity": similarity,
                "keyframe_index": entry.cloud_index,
                "yaw_offset_deg": float(yaw_offset),
                "fitness": float(result.fitness),
                "inlier_rmse": float(result.inlier_rmse),
                "x": float(pose[0, 3]),
                "y": float(pose[1, 3]),
                "yaw_deg": final_yaw,
                "xy_error": xy_error,
                "yaw_error": yaw_error,
                "seed_drift_xy": seed_drift_xy,
                "seed_drift_yaw": seed_drift_yaw,
            }
            refined.append(item)
            candidate_rows.append(
                {
                    "waypoint_id": target.waypoint_id,
                    "target_index": str(target.cloud_index),
                    "descriptor_rank": str(descriptor_rank),
                    "triangle_similarity": f"{similarity:.6f}",
                    "keyframe_index": str(entry.cloud_index),
                    "keyframe_x": f"{entry.reference_x:.6f}",
                    "keyframe_y": f"{entry.reference_y:.6f}",
                    "keyframe_yaw_deg": f"{entry.reference_yaw_deg:.6f}",
                    "yaw_offset_deg": f"{float(yaw_offset):.3f}",
                    "fitness": f"{float(result.fitness):.6f}",
                    "inlier_rmse": f"{float(result.inlier_rmse):.6f}",
                    "seed_drift_xy_m": f"{seed_drift_xy:.6f}",
                    "seed_drift_yaw_deg": f"{seed_drift_yaw:.6f}",
                    "final_x_m": f"{float(pose[0, 3]):.6f}",
                    "final_y_m": f"{float(pose[1, 3]):.6f}",
                    "final_yaw_deg": f"{final_yaw:.6f}",
                    "translation_error_m": f"{xy_error:.6f}",
                    "yaw_error_deg": f"{yaw_error:.6f}",
                }
            )

    gated = [
        item
        for item in refined
        if float(item["seed_drift_xy"]) <= args.max_seed_drift_m
        and float(item["seed_drift_yaw"]) <= args.max_seed_drift_yaw_deg
    ]
    candidate_pool = gated if gated else refined
    if not candidate_pool:
        raise RuntimeError(f"target {target.cloud_index} 没有可评估候选")

    # 排序策略：先信三角召回相似度和 rank，再用 GICP rmse/fitness 做几何细分。
    def fusion_score(item: dict[str, float | int]) -> float:
        return (
            -args.triangle_similarity_weight * float(item["similarity"])
            + args.descriptor_rank_weight * float(item["descriptor_rank"])
            + float(item["inlier_rmse"])
            - args.fitness_weight * float(item["fitness"])
        )

    candidate_pool.sort(key=lambda item: (fusion_score(item), float(item["inlier_rmse"])))
    best = candidate_pool[0]
    similarity_sorted = sorted(candidate_pool, key=lambda item: float(item["similarity"]), reverse=True)
    second_similarity = float(similarity_sorted[1]["similarity"]) if len(similarity_sorted) > 1 else 0.0
    similarity_margin = float(best["similarity"]) - second_similarity
    total_ms = (time.perf_counter() - begin) * 1000.0

    summary = {
        "waypoint_id": target.waypoint_id,
        "name": target.name,
        "target_index": str(target.cloud_index),
        "descriptor_top_k": str(args.descriptor_top_k),
        "yaw_offsets": ";".join(str(v) for v in args.yaw_offsets),
        "query_keypoints": str(query_keypoints),
        "query_triangles": str(query_triangles),
        "scan_points": str(len(scan_cloud.points)),
        "candidate_evaluations": str(len(refined)),
        "gated_candidate_evaluations": str(len(gated)),
        "selected_descriptor_rank": str(int(best["descriptor_rank"])),
        "selected_keyframe_index": str(int(best["keyframe_index"])),
        "selected_yaw_offset_deg": f"{float(best['yaw_offset_deg']):.3f}",
        "triangle_similarity": f"{float(best['similarity']):.6f}",
        "similarity_margin": f"{similarity_margin:.6f}",
        "fitness": f"{float(best['fitness']):.6f}",
        "inlier_rmse": f"{float(best['inlier_rmse']):.6f}",
        "seed_drift_xy_m": f"{float(best['seed_drift_xy']):.6f}",
        "seed_drift_yaw_deg": f"{float(best['seed_drift_yaw']):.6f}",
        "fusion_score": f"{fusion_score(best):.6f}",
        "final_x_m": f"{float(best['x']):.6f}",
        "final_y_m": f"{float(best['y']):.6f}",
        "final_yaw_deg": f"{float(best['yaw_deg']):.6f}",
        "reference_x_m": f"{target.reference_x:.6f}",
        "reference_y_m": f"{target.reference_y:.6f}",
        "reference_yaw_deg": f"{target.reference_yaw_deg:.6f}",
        "translation_error_m": f"{float(best['xy_error']):.6f}",
        "yaw_error_deg": f"{float(best['yaw_error']):.6f}",
        "success_0p2m_3deg": "1" if float(best["xy_error"]) <= 0.2 and float(best["yaw_error"]) <= 3.0 else "0",
        "success_0p3m_5deg": "1" if float(best["xy_error"]) <= 0.3 and float(best["yaw_error"]) <= 5.0 else "0",
        "success_0p5m_10deg": "1" if float(best["xy_error"]) <= 0.5 and float(best["yaw_error"]) <= 10.0 else "0",
        "total_ms": f"{total_ms:.3f}",
        "rss_mb": f"{current_rss_mb():.3f}",
    }
    return summary, candidate_rows


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """写出 CSV；字段顺序沿用第一行，方便后续人工查看。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def print_success_summary(rows: list[dict[str, str]]) -> None:
    """按用户关心的三个阈值打印成功率摘要。"""

    total = len(rows)
    for key in ["success_0p2m_3deg", "success_0p3m_5deg", "success_0p5m_10deg"]:
        count = sum(row[key] == "1" for row in rows)
        print(f"[triangle_descriptor] {key}={count}/{total}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate STD-style triangle descriptor recall fused with GICP.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--bag", type=Path, default=Path("/home/ubuntu/nav_drift_test/nav_drift_test46"), help="验证 bag")
    parser.add_argument("--matches", type=Path, default=None, help="普通点位匹配 CSV")
    parser.add_argument("--metrics-csv", type=Path, default=None, help="evaluator 输出的 metrics CSV")
    parser.add_argument("--failure-csv", type=Path, default=None, help="hard failure CSV")
    parser.add_argument("--indices", type=Path, default=None, help="hard failure 对应 cloud index 列表")
    parser.add_argument("--edge-selection", type=Path, default=None, help="edge/corner 选点 CSV")
    parser.add_argument("--ids", nargs="*", default=[], help="只验证指定 waypoint_id")
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"), help="全局地图 PCD")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/triangle_descriptor_validation.csv"), help="输出 CSV")
    parser.add_argument("--candidate-output", type=Path, default=None, help="可选：输出每个候选 GICP 明细 CSV")
    parser.add_argument("--database-cache", type=Path, default=None, help="三角描述子数据库 JSON 缓存路径")
    parser.add_argument("--keyframe-stride", type=int, default=80, help="描述子库抽样步长")
    parser.add_argument("--exclude-index-radius", type=int, default=240, help="排除目标前后 cloud index 范围，避免泄漏")
    parser.add_argument("--descriptor-top-k", type=int, default=20, help="参与 GICP 的三角描述子候选数量")
    parser.add_argument("--yaw-offsets", nargs="*", type=float, default=[-90, -60, -30, 0, 30, 60, 90, 120, 150, 180, -150, -120], help="每个 keyframe seed 额外尝试的 yaw 偏移")
    parser.add_argument("--max-radius", type=float, default=20.0, help="描述子和 scan 裁剪最大半径")
    parser.add_argument("--min-range", type=float, default=0.6, help="忽略机器人近场点")
    parser.add_argument("--min-z", type=float, default=0.2, help="最低高度")
    parser.add_argument("--max-z", type=float, default=2.5, help="最高高度")
    parser.add_argument("--keypoint-voxel-size", type=float, default=0.60, help="三角关键点 XY 聚类体素")
    parser.add_argument("--min-keypoint-points", type=int, default=4, help="成为关键点所需的最少原始点数")
    parser.add_argument("--keypoint-nms-radius", type=float, default=0.75, help="关键点非极大值抑制半径")
    parser.add_argument("--max-keypoints", type=int, default=36, help="每帧最多关键点数量")
    parser.add_argument("--max-triangles", type=int, default=5000, help="每帧最多三角数量")
    parser.add_argument("--triangle-bin-size", type=float, default=0.35, help="三角边长量化分辨率")
    parser.add_argument("--min-triangle-side", type=float, default=1.0, help="三角最短边下限")
    parser.add_argument("--max-triangle-side", type=float, default=18.0, help="三角最长边上限")
    parser.add_argument("--min-triangle-area", type=float, default=0.8, help="三角面积下限，过滤近似共线点")
    parser.add_argument("--map-voxel-size", type=float, default=0.30, help="地图降采样体素")
    parser.add_argument("--scan-voxel-size", type=float, default=0.25, help="scan 降采样体素")
    parser.add_argument("--max-correspondence-distance", type=float, default=1.5, help="GICP 最大对应距离")
    parser.add_argument("--gicp-iterations", type=int, default=30, help="GICP 最大迭代次数")
    parser.add_argument("--max-seed-drift-m", type=float, default=1.5, help="GICP 结果相对 descriptor seed 允许的最大平移漂移")
    parser.add_argument("--max-seed-drift-yaw-deg", type=float, default=55.0, help="GICP 结果相对 descriptor seed 允许的最大 yaw 漂移")
    parser.add_argument("--triangle-similarity-weight", type=float, default=1.2, help="融合排序中三角相似度权重")
    parser.add_argument("--descriptor-rank-weight", type=float, default=0.02, help="融合排序中 descriptor rank 权重")
    parser.add_argument("--fitness-weight", type=float, default=0.0, help="融合排序中 Open3D fitness 奖励权重")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    bag = args.bag if args.bag.is_absolute() else workspace / args.bag
    map_path = args.map if args.map.is_absolute() else workspace / args.map
    output = args.output if args.output.is_absolute() else workspace / args.output
    if args.matches is not None and not args.matches.is_absolute():
        args.matches = workspace / args.matches
    if args.metrics_csv is not None and not args.metrics_csv.is_absolute():
        args.metrics_csv = workspace / args.metrics_csv
    if args.failure_csv is not None and not args.failure_csv.is_absolute():
        args.failure_csv = workspace / args.failure_csv
    if args.indices is not None and not args.indices.is_absolute():
        args.indices = workspace / args.indices
    if args.edge_selection is not None and not args.edge_selection.is_absolute():
        args.edge_selection = workspace / args.edge_selection
    if args.database_cache is not None and not args.database_cache.is_absolute():
        args.database_cache = workspace / args.database_cache

    targets = load_targets(args)
    if not targets:
        print("[triangle_descriptor] no targets selected")
        return 2

    begin = time.perf_counter()
    cache_ready = args.database_cache is not None and args.database_cache.exists()
    # 如果已有描述子库缓存，本轮只需要目标点云/odom/reference，不再反复反序列化 keyframe 点云。
    read_stride = 10**12 if cache_ready else max(1, args.keyframe_stride)
    odoms, references, target_clouds, keyframe_clouds = read_bag_samples(bag, targets, read_stride)
    if args.database_cache is not None and args.database_cache.exists():
        database = load_database(args.database_cache)
        print(f"[triangle_descriptor] loaded database entries={len(database)} cache={args.database_cache}")
    else:
        database = build_triangle_database(keyframe_clouds, odoms, references, args)
        if args.database_cache is not None:
            save_database(args.database_cache, database)
            print(f"[triangle_descriptor] saved database entries={len(database)} cache={args.database_cache}")
    map_cloud = load_map_cloud(map_path, args)
    prepare_ms = (time.perf_counter() - begin) * 1000.0
    print(
        "[triangle_descriptor] "
        f"targets={len(targets)} database={len(database)} map_points={len(map_cloud.points)} "
        f"prepare_ms={prepare_ms:.1f} rss_mb={current_rss_mb():.1f}"
    )

    rows: list[dict[str, str]] = []
    candidate_rows: list[dict[str, str]] = []
    for target in targets:
        sample = target_clouds.get(target.cloud_index)
        if sample is None:
            print(f"[triangle_descriptor] missing target cloud index={target.cloud_index}")
            continue
        row, target_candidate_rows = evaluate_target(target, sample, odoms, database, map_cloud, args)
        rows.append(row)
        candidate_rows.extend(target_candidate_rows)
        print(
            "[triangle_descriptor] "
            f"id={row['waypoint_id']} idx={row['target_index']} "
            f"0.3/5={row['success_0p3m_5deg']} "
            f"err={row['translation_error_m']}m/{row['yaw_error_deg']}deg "
            f"sim={row['triangle_similarity']} rank={row['selected_descriptor_rank']} "
            f"rmse={row['inlier_rmse']} ms={row['total_ms']}"
        )

    write_csv(output, rows)
    if args.candidate_output is not None:
        candidate_output = args.candidate_output if args.candidate_output.is_absolute() else workspace / args.candidate_output
        write_csv(candidate_output, candidate_rows)
        print(f"[triangle_descriptor] wrote candidates {candidate_output}")
    print_success_summary(rows)
    print(f"[triangle_descriptor] wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
