#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_structural_map_gate_validation.py

文件作用：
  1. 离线验证“结构化地图 / 墙线 / 局部拓扑一致性”是否能压低全局重定位误接受。
  2. 从 PCD 地图投影生成二维占据图，用 HoughLinesP 提取地图墙线结构。
  3. 对 C++ evaluator 的最终候选位姿，把真实 scan 投到 map 下并提取 scan 线段。
  4. 用线段匹配率、方向直方图相似度、未匹配长线比例等无真值指标判断候选是否结构一致。

设计说明：
  - 该脚本验证“候选是否允许自动注入”，不重新做 BBS/GICP 搜索。
  - 真值只用于最后统计成功率和误接受，不参与任何结构评分。
  - 与 run_alignment_risk_gate_validation.py 不同，这里不再依赖点到点残差，而是显式比较墙线/线段结构。
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
    is_success,
    load_metric_samples,
    preprocess_scan,
    read_bag_clouds,
    transform_points,
)
from run_scan_context_keyframe_recall import nearest_odom, registered_world_cloud_to_body


@dataclass(frozen=True)
class LineSegment:
    """二维线段，保存端点、长度、角度和中点，便于快速匹配。"""

    x1: float
    y1: float
    x2: float
    y2: float
    angle_deg: float
    length_m: float
    mid_x: float
    mid_y: float


@dataclass(frozen=True)
class StructuralMap:
    """从 PCD 提取出来的二维结构地图。"""

    origin_x: float
    origin_y: float
    resolution: float
    image: np.ndarray
    lines: list[LineSegment]


@dataclass(frozen=True)
class StructuralMetrics:
    """一个候选的结构一致性指标。"""

    source_id: str
    bag_frame_index: int
    stamp_sec: float
    scan_line_count: int
    map_line_count: int
    matched_line_count: int
    matched_length_ratio: float
    unmatched_long_ratio: float
    orientation_similarity: float
    orientation_bin_count: int
    structural_score: float
    translation_error_m: float
    yaw_error_deg: float
    refine_fitness: float


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间。"""

    return Path(__file__).resolve().parents[3]


def normalize_angle_180(angle_deg: float) -> float:
    """把无向线段角度规整到 [0, 180)。"""

    angle = angle_deg % 180.0
    if angle < 0.0:
        angle += 180.0
    return angle


def angle_diff_180(a_deg: float, b_deg: float) -> float:
    """计算两条无向线段的夹角差，范围 [0, 90]。"""

    diff = abs(normalize_angle_180(a_deg) - normalize_angle_180(b_deg))
    return min(diff, 180.0 - diff)


def line_from_points(x1: float, y1: float, x2: float, y2: float) -> LineSegment | None:
    """由端点构造线段；太短线段返回 None。"""

    length = math.hypot(x2 - x1, y2 - y1)
    if length <= 1e-6:
        return None
    angle = normalize_angle_180(math.degrees(math.atan2(y2 - y1, x2 - x1)))
    return LineSegment(x1, y1, x2, y2, angle, length, 0.5 * (x1 + x2), 0.5 * (y1 + y2))


def load_map_points(map_path: Path, args: argparse.Namespace) -> np.ndarray:
    """读取地图点云并保留墙体/障碍高度范围。"""

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


def points_to_image(
    points_xy: np.ndarray,
    resolution: float,
    margin_m: float,
) -> tuple[np.ndarray, float, float]:
    """把二维点投影成二值占据图，并返回图像原点。"""

    min_xy = points_xy.min(axis=0) - margin_m
    max_xy = points_xy.max(axis=0) + margin_m
    size = np.ceil((max_xy - min_xy) / resolution).astype(np.int32) + 1
    image = np.zeros((int(size[1]), int(size[0])), dtype=np.uint8)
    ix = np.floor((points_xy[:, 0] - min_xy[0]) / resolution).astype(np.int32)
    iy = np.floor((points_xy[:, 1] - min_xy[1]) / resolution).astype(np.int32)
    valid = (ix >= 0) & (iy >= 0) & (ix < size[0]) & (iy < size[1])
    image[iy[valid], ix[valid]] = 255
    return image, float(min_xy[0]), float(min_xy[1])


def clean_occupancy_image(image: np.ndarray, close_kernel: int, dilate_kernel: int) -> np.ndarray:
    """对点云投影图做闭运算/膨胀，让墙线更连续。"""

    cleaned = image.copy()
    if close_kernel > 1:
        kernel = np.ones((close_kernel, close_kernel), dtype=np.uint8)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
    if dilate_kernel > 1:
        kernel = np.ones((dilate_kernel, dilate_kernel), dtype=np.uint8)
        cleaned = cv2.dilate(cleaned, kernel, iterations=1)
    return cleaned


def hough_lines_to_segments(
    image: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    args: argparse.Namespace,
    min_length_m: float,
) -> list[LineSegment]:
    """从二值图提取 Hough 线段并转换回 map 坐标。"""

    lines = cv2.HoughLinesP(
        image,
        rho=1,
        theta=np.pi / 180.0,
        threshold=args.hough_threshold,
        minLineLength=max(1, int(min_length_m / resolution)),
        maxLineGap=max(1, int(args.hough_max_gap_m / resolution)),
    )
    if lines is None:
        return []

    segments: list[LineSegment] = []
    height = image.shape[0]
    for raw in lines[:, 0, :]:
        px1, py1, px2, py2 = (int(v) for v in raw)
        # 图像 y 轴向下，但 map y 轴按栅格 origin 向上累加；这里沿用同一栅格坐标，不翻转，
        # 因为地图和 scan patch 都使用相同转换，线段角度/距离比较保持一致。
        x1 = origin_x + float(px1) * resolution
        y1 = origin_y + float(py1) * resolution
        x2 = origin_x + float(px2) * resolution
        y2 = origin_y + float(py2) * resolution
        segment = line_from_points(x1, y1, x2, y2)
        if segment and segment.length_m >= min_length_m:
            segments.append(segment)
    return merge_similar_lines(segments, args)


def merge_similar_lines(lines: list[LineSegment], args: argparse.Namespace) -> list[LineSegment]:
    """合并 Hough 输出里几乎重叠的短碎线，降低重复计数。"""

    merged: list[LineSegment] = []
    for line in sorted(lines, key=lambda item: item.length_m, reverse=True):
        duplicate = False
        for existing in merged:
            if angle_diff_180(line.angle_deg, existing.angle_deg) > args.merge_angle_deg:
                continue
            if point_to_line_distance(line.mid_x, line.mid_y, existing) > args.merge_distance_m:
                continue
            if math.hypot(line.mid_x - existing.mid_x, line.mid_y - existing.mid_y) > args.merge_midpoint_m:
                continue
            duplicate = True
            break
        if not duplicate:
            merged.append(line)
    return merged


def build_structural_map(map_points: np.ndarray, args: argparse.Namespace) -> StructuralMap:
    """从地图 PCD 构建结构地图。"""

    image, origin_x, origin_y = points_to_image(map_points[:, :2], args.structural_resolution, args.map_margin_m)
    cleaned = clean_occupancy_image(image, args.map_close_kernel, args.map_dilate_kernel)
    lines = hough_lines_to_segments(
        cleaned,
        origin_x,
        origin_y,
        args.structural_resolution,
        args,
        args.map_min_line_length_m,
    )
    return StructuralMap(origin_x, origin_y, args.structural_resolution, cleaned, lines)


def point_to_line_distance(x: float, y: float, line: LineSegment) -> float:
    """计算点到有限线段所在直线的垂直距离。"""

    vx = line.x2 - line.x1
    vy = line.y2 - line.y1
    denom = max(math.hypot(vx, vy), 1e-9)
    return abs(vy * x - vx * y + line.x2 * line.y1 - line.y2 * line.x1) / denom


def projection_interval(line: LineSegment, angle_deg: float) -> tuple[float, float]:
    """把线段端点投影到给定方向上，返回一维区间。"""

    theta = math.radians(angle_deg)
    ux = math.cos(theta)
    uy = math.sin(theta)
    a = line.x1 * ux + line.y1 * uy
    b = line.x2 * ux + line.y2 * uy
    return min(a, b), max(a, b)


def interval_overlap_ratio(scan_line: LineSegment, map_line: LineSegment) -> float:
    """计算两个近似平行线段沿 scan 主方向的一维重叠比例。"""

    a1, a2 = projection_interval(scan_line, scan_line.angle_deg)
    b1, b2 = projection_interval(map_line, scan_line.angle_deg)
    overlap = max(0.0, min(a2, b2) - max(a1, b1))
    return overlap / max(scan_line.length_m, 1e-6)


def local_map_lines(structural_map: StructuralMap, x: float, y: float, radius_m: float) -> list[LineSegment]:
    """筛出候选附近的地图线段。"""

    radius2 = radius_m * radius_m
    return [
        line for line in structural_map.lines
        if (line.mid_x - x) * (line.mid_x - x) + (line.mid_y - y) * (line.mid_y - y) <= radius2
    ]


def scan_lines_from_points(scan_in_map: np.ndarray, sample: MetricSample, args: argparse.Namespace) -> list[LineSegment]:
    """把候选位姿下的 scan 点投影到局部图像并提取线段。"""

    if len(scan_in_map) == 0:
        return []
    half = args.scan_patch_radius_m
    mask = (
        (scan_in_map[:, 0] >= sample.final_x - half)
        & (scan_in_map[:, 0] <= sample.final_x + half)
        & (scan_in_map[:, 1] >= sample.final_y - half)
        & (scan_in_map[:, 1] <= sample.final_y + half)
    )
    points = scan_in_map[mask, :2]
    if len(points) < 20:
        return []

    image, origin_x, origin_y = points_to_image(points, args.structural_resolution, args.scan_patch_margin_m)
    cleaned = clean_occupancy_image(image, args.scan_close_kernel, args.scan_dilate_kernel)
    return hough_lines_to_segments(
        cleaned,
        origin_x,
        origin_y,
        args.structural_resolution,
        args,
        args.scan_min_line_length_m,
    )


def line_matches(scan_line: LineSegment, map_line: LineSegment, args: argparse.Namespace) -> bool:
    """判断一条 scan 线段是否能由地图墙线解释。"""

    if angle_diff_180(scan_line.angle_deg, map_line.angle_deg) > args.match_angle_deg:
        return False
    if point_to_line_distance(scan_line.mid_x, scan_line.mid_y, map_line) > args.match_distance_m:
        return False
    if interval_overlap_ratio(scan_line, map_line) < args.match_overlap_ratio:
        return False
    return True


def orientation_histogram(lines: list[LineSegment], bins: int) -> np.ndarray:
    """按线段长度加权计算无向方向直方图。"""

    hist = np.zeros(bins, dtype=np.float64)
    for line in lines:
        index = min(bins - 1, int(normalize_angle_180(line.angle_deg) / 180.0 * bins))
        hist[index] += line.length_m
    norm = np.linalg.norm(hist)
    return hist / norm if norm > 1e-9 else hist


def cosine_similarity(lhs: np.ndarray, rhs: np.ndarray) -> float:
    """计算两个归一化直方图的余弦相似度。"""

    denom = np.linalg.norm(lhs) * np.linalg.norm(rhs)
    if denom <= 1e-9:
        return 0.0
    return float(np.dot(lhs, rhs) / denom)


def evaluate_structural_sample(
    sample: MetricSample,
    scan_points: np.ndarray,
    structural_map: StructuralMap,
    args: argparse.Namespace,
) -> StructuralMetrics:
    """计算单个候选的结构一致性指标。"""

    scan = preprocess_scan(scan_points, args)
    scan_in_map = transform_points(scan, sample.final_x, sample.final_y, sample.final_yaw_deg)
    scan_lines = scan_lines_from_points(scan_in_map, sample, args)
    map_lines = local_map_lines(structural_map, sample.final_x, sample.final_y, args.scan_patch_radius_m + 2.0)

    matched_count = 0
    matched_length = 0.0
    unmatched_long = 0
    total_length = sum(line.length_m for line in scan_lines)
    for scan_line in scan_lines:
        matched = any(line_matches(scan_line, map_line, args) for map_line in map_lines)
        if matched:
            matched_count += 1
            matched_length += scan_line.length_m
        elif scan_line.length_m >= args.unmatched_long_line_m:
            unmatched_long += 1

    matched_ratio = matched_length / max(total_length, 1e-6)
    unmatched_ratio = unmatched_long / max(len(scan_lines), 1)
    scan_hist = orientation_histogram(scan_lines, args.orientation_bins)
    map_hist = orientation_histogram(map_lines, args.orientation_bins)
    orientation_similarity = cosine_similarity(scan_hist, map_hist)
    orientation_bin_count = int(np.count_nonzero(scan_hist > args.orientation_bin_min_weight))
    structural_score = (
        0.50 * matched_ratio
        + 0.35 * orientation_similarity
        + 0.15 * max(0.0, 1.0 - unmatched_ratio)
    )

    return StructuralMetrics(
        source_id=sample.source_id,
        bag_frame_index=sample.bag_frame_index,
        stamp_sec=sample.stamp_sec,
        scan_line_count=len(scan_lines),
        map_line_count=len(map_lines),
        matched_line_count=matched_count,
        matched_length_ratio=matched_ratio,
        unmatched_long_ratio=unmatched_ratio,
        orientation_similarity=orientation_similarity,
        orientation_bin_count=orientation_bin_count,
        structural_score=float(max(0.0, min(1.0, structural_score))),
        translation_error_m=sample.translation_error_m,
        yaw_error_deg=sample.yaw_error_deg,
        refine_fitness=sample.refine_fitness,
    )


def pass_gate(metric: StructuralMetrics, gate: dict[str, float]) -> bool:
    """按结构门控参数判断是否接受候选。"""

    return (
        metric.scan_line_count >= gate["min_scan_lines"]
        and metric.matched_length_ratio >= gate["min_matched_ratio"]
        and metric.orientation_similarity >= gate["min_orientation_similarity"]
        and metric.unmatched_long_ratio <= gate["max_unmatched_long_ratio"]
        and metric.structural_score >= gate["min_structural_score"]
        and metric.refine_fitness <= gate["max_refine_fitness"]
    )


def gate_sweep() -> list[dict[str, float]]:
    """结构门控组合：从宽松诊断到较严格自动接受。"""

    return [
        {
            "name": "structural_lenient",
            "min_scan_lines": 1,
            "min_matched_ratio": 0.10,
            "min_orientation_similarity": 0.05,
            "max_unmatched_long_ratio": 0.75,
            "min_structural_score": 0.15,
            "max_refine_fitness": 0.08,
        },
        {
            "name": "structural_balanced",
            "min_scan_lines": 2,
            "min_matched_ratio": 0.15,
            "min_orientation_similarity": 0.15,
            "max_unmatched_long_ratio": 0.70,
            "min_structural_score": 0.20,
            "max_refine_fitness": 0.06,
        },
        {
            "name": "structural_strict",
            "min_scan_lines": 2,
            "min_matched_ratio": 0.20,
            "min_orientation_similarity": 0.25,
            "max_unmatched_long_ratio": 0.62,
            "min_structural_score": 0.25,
            "max_refine_fitness": 0.045,
        },
        {
            "name": "structural_fitness_only_reference",
            "min_scan_lines": 0,
            "min_matched_ratio": 0.0,
            "min_orientation_similarity": 0.0,
            "max_unmatched_long_ratio": 1.0,
            "min_structural_score": 0.0,
            "max_refine_fitness": 0.055,
        },
    ]


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """写 CSV 工具函数。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else [])
        writer.writeheader()
        writer.writerows(rows)


def summarize(metrics: list[StructuralMetrics], output: Path) -> list[dict[str, str]]:
    """统计各结构门控在三档阈值下的效果。"""

    thresholds = [
        ("strict_0.2m_3deg", 0.2, 3.0),
        ("practical_0.3m_5deg", 0.3, 5.0),
        ("recoverable_0.5m_10deg", 0.5, 10.0),
    ]
    rows: list[dict[str, str]] = []
    for gate in [{"name": "baseline_accept_all"}] + gate_sweep():
        accepted = metrics if gate["name"] == "baseline_accept_all" else [m for m in metrics if pass_gate(m, gate)]
        rejected = [m for m in metrics if m not in accepted]
        for label, xy, yaw in thresholds:
            ok = sum(is_success(m, xy, yaw) for m in accepted)
            false = len(accepted) - ok
            rejected_ok = sum(is_success(m, xy, yaw) for m in rejected)
            rows.append(
                {
                    "gate_name": gate["name"],
                    "threshold": label,
                    "total": str(len(metrics)),
                    "accepted": str(len(accepted)),
                    "rejected": str(len(rejected)),
                    "accepted_success": str(ok),
                    "accepted_success_rate": f"{ok / max(len(accepted), 1):.6f}",
                    "false_accept": str(false),
                    "false_accept_rate": f"{false / max(len(accepted), 1):.6f}",
                    "rejected_true_success": str(rejected_ok),
                    "auto_success_rate_over_total": f"{ok / max(len(metrics), 1):.6f}",
                }
            )
    write_csv(output, rows)
    return rows


def run(args: argparse.Namespace) -> int:
    """脚本主流程。"""

    start_wall = time.time()
    start_cpu = current_cpu_seconds()
    workspace = args.workspace.resolve()
    map_path = args.map if args.map.is_absolute() else workspace / args.map

    samples = load_metric_samples(args.metrics_glob, args.scenario_name, args.max_samples)
    if not samples:
        raise RuntimeError("没有读取到 metrics 样本")
    bag_paths = sorted({sample.bag_path for sample in samples})
    if len(bag_paths) != 1:
        raise RuntimeError(f"当前脚本一次只处理一个 bag，实际={bag_paths}")
    target_indices = {sample.bag_frame_index for sample in samples}
    print(f"[structural_map_gate] samples={len(samples)} unique_frames={len(target_indices)}", flush=True)

    odoms, clouds = read_bag_clouds(bag_paths[0], target_indices)
    print(f"[structural_map_gate] loaded_clouds={len(clouds)} odoms={len(odoms)}", flush=True)

    map_points = load_map_points(map_path, args)
    structural_map = build_structural_map(map_points, args)
    print(
        f"[structural_map_gate] map_points={len(map_points)} map_lines={len(structural_map.lines)} "
        f"rss_mb={current_rss_mb():.1f}",
        flush=True,
    )

    details: list[dict[str, str]] = []
    metrics: list[StructuralMetrics] = []
    for index, sample in enumerate(samples):
        cloud = clouds.get(sample.bag_frame_index)
        if cloud is None:
            continue
        odom = nearest_odom(odoms, cloud.stamp_sec)
        scan_points = registered_world_cloud_to_body(cloud, odom)
        metric = evaluate_structural_sample(sample, scan_points, structural_map, args)
        metrics.append(metric)
        details.append(
            {
                "source_id": metric.source_id,
                "bag_frame_index": str(metric.bag_frame_index),
                "stamp_sec": f"{metric.stamp_sec:.6f}",
                "scan_line_count": str(metric.scan_line_count),
                "map_line_count": str(metric.map_line_count),
                "matched_line_count": str(metric.matched_line_count),
                "matched_length_ratio": f"{metric.matched_length_ratio:.6f}",
                "unmatched_long_ratio": f"{metric.unmatched_long_ratio:.6f}",
                "orientation_similarity": f"{metric.orientation_similarity:.6f}",
                "orientation_bin_count": str(metric.orientation_bin_count),
                "structural_score": f"{metric.structural_score:.6f}",
                "refine_fitness": f"{metric.refine_fitness:.6f}",
                "translation_error_m": f"{metric.translation_error_m:.6f}",
                "yaw_error_deg": f"{metric.yaw_error_deg:.6f}",
                "success_0p2m_3deg": "1" if is_success(metric, 0.2, 3.0) else "0",
                "success_0p3m_5deg": "1" if is_success(metric, 0.3, 5.0) else "0",
                "success_0p5m_10deg": "1" if is_success(metric, 0.5, 10.0) else "0",
            }
        )
        if (index + 1) % max(1, args.progress_interval) == 0:
            print(f"[structural_map_gate] progress {index + 1}/{len(samples)}", flush=True)

    output = args.output if args.output.is_absolute() else workspace / args.output
    summary_output = args.summary_output if args.summary_output.is_absolute() else workspace / args.summary_output
    write_csv(output, details)
    summary = summarize(metrics, summary_output)

    elapsed = time.time() - start_wall
    cpu_cores = (current_cpu_seconds() - start_cpu) / max(elapsed, 1e-6)
    print(f"[structural_map_gate] details={output}", flush=True)
    print(f"[structural_map_gate] summary={summary_output}", flush=True)
    print(f"[structural_map_gate] elapsed_sec={elapsed:.3f} cpu_core_equiv={cpu_cores:.2f} rss_mb={current_rss_mb():.1f}", flush=True)
    for row in summary:
        if row["threshold"] == "practical_0.3m_5deg":
            print(
                "[structural_map_gate] "
                f"gate={row['gate_name']} accepted={row['accepted']}/{row['total']} "
                f"ok={row['accepted_success']} false={row['false_accept']} "
                f"reject_good={row['rejected_true_success']}",
                flush=True,
            )
    return 0


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""

    root = repo_root_from_script()
    parser = argparse.ArgumentParser(description="验证结构化墙线地图候选门控。")
    parser.add_argument("--workspace", type=Path, default=root, help="humanoid_ws 工作空间")
    parser.add_argument(
        "--metrics-glob",
        action="append",
        default=[],
        help="global_relocalization_metrics.csv 路径或 glob，可重复传入",
    )
    parser.add_argument("--scenario-name", default="arbitrary_start_no_prior", help="只验证该场景")
    parser.add_argument("--max-samples", type=int, default=0, help="最多验证样本数；0 表示全量")
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"), help="全局地图 PCD")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/structural_map_gate/details.csv"), help="结构明细 CSV")
    parser.add_argument("--summary-output", type=Path, default=Path(".codex_tmp/structural_map_gate/summary.csv"), help="结构汇总 CSV")
    parser.add_argument("--progress-interval", type=int, default=25, help="进度打印间隔")

    parser.add_argument("--map-voxel-size", type=float, default=0.08, help="地图点云降采样尺寸")
    parser.add_argument("--map-min-z", type=float, default=0.15, help="地图墙体最低高度")
    parser.add_argument("--map-max-z", type=float, default=2.20, help="地图墙体最高高度")
    parser.add_argument("--structural-resolution", type=float, default=0.08, help="结构图分辨率")
    parser.add_argument("--map-margin-m", type=float, default=2.0, help="地图投影外扩边界")
    parser.add_argument("--map-close-kernel", type=int, default=3, help="地图占据图闭运算核")
    parser.add_argument("--map-dilate-kernel", type=int, default=2, help="地图占据图膨胀核")

    parser.add_argument("--scan-voxel-size", type=float, default=0.08, help="scan 结构提取降采样尺寸")
    parser.add_argument("--max-scan-points", type=int, default=3500, help="最多参与结构提取的 scan 点数")
    parser.add_argument("--min-range", type=float, default=0.35, help="scan 最小距离")
    parser.add_argument("--max-range", type=float, default=18.0, help="scan 最大距离")
    parser.add_argument("--scan-min-z", type=float, default=-1.0, help="scan 最低高度")
    parser.add_argument("--scan-max-z", type=float, default=2.20, help="scan 最高高度")
    parser.add_argument("--scan-patch-radius-m", type=float, default=18.0, help="候选局部结构 patch 半径")
    parser.add_argument("--scan-patch-margin-m", type=float, default=0.5, help="scan patch 投影边界")
    parser.add_argument("--scan-close-kernel", type=int, default=3, help="scan 图闭运算核")
    parser.add_argument("--scan-dilate-kernel", type=int, default=2, help="scan 图膨胀核")

    parser.add_argument("--hough-threshold", type=int, default=18, help="HoughLinesP 累计阈值")
    parser.add_argument("--hough-max-gap-m", type=float, default=0.45, help="Hough 最大断裂连接距离")
    parser.add_argument("--map-min-line-length-m", type=float, default=1.2, help="地图最短墙线")
    parser.add_argument("--scan-min-line-length-m", type=float, default=0.8, help="scan 最短线段")
    parser.add_argument("--merge-angle-deg", type=float, default=6.0, help="线段合并角度阈值")
    parser.add_argument("--merge-distance-m", type=float, default=0.25, help="线段合并垂直距离")
    parser.add_argument("--merge-midpoint-m", type=float, default=1.0, help="线段合并中点距离")

    parser.add_argument("--match-angle-deg", type=float, default=10.0, help="scan-map 线段匹配角度阈值")
    parser.add_argument("--match-distance-m", type=float, default=0.45, help="scan-map 线段匹配垂直距离阈值")
    parser.add_argument("--match-overlap-ratio", type=float, default=0.20, help="scan-map 线段投影重叠比例阈值")
    parser.add_argument("--unmatched-long-line-m", type=float, default=1.5, help="未匹配长线判定长度")
    parser.add_argument("--orientation-bins", type=int, default=18, help="方向直方图 bins")
    parser.add_argument("--orientation-bin-min-weight", type=float, default=0.05, help="方向直方图有效 bin 最小权重")

    if not parser.parse_known_args()[0].metrics_glob:
        parser.set_defaults(
            metrics_glob=[
                str(root / ".codex_tmp/bbs2d_integrated_random100/rand100_seed*/global_relocalization_metrics.csv")
            ]
        )
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
