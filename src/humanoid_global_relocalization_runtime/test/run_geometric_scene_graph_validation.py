#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_geometric_scene_graph_validation.py

文件作用：
  1. 离线验证 Outram/scene-graph 思路在当前机器人室内 PCD 地图与真实 bag 点云上的可行性。
  2. 官方 Outram 需要 ROS1/catkin、逐点语义 label 和 semantic cluster map；当前 bag 没有语义 label，
     因此本脚本保留完整“场景图三角描述子 + 全局候选 + 几何一致性共识 + ICP 精修”的算法链路，
     但用墙线端点/交点等几何结构节点替代语义实例节点。
  3. 输入真实 /fast_lio/cloud_registered + /odom，沿用 C++ evaluator 的 raw body -> base_footprint 轴转换。
  4. 输出 top1/topK 成功率、误差、耗时和内存，用于判断该路线是否值得继续工程化。

重要说明：
  - 真值只用于统计，不参与搜索、排序和精修。
  - 该脚本不发布 TF、不注入 initialpose、不修改线上配置。
  - 这是完整算法验证脚本，不是门控脚本：它会自行从地图和当前 scan 生成全局位姿候选。
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import itertools
import math
from pathlib import Path
import resource
import time

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from run_alignment_risk_gate_validation import (
    current_cpu_seconds,
    current_rss_mb,
    load_metric_samples,
    read_bag_clouds,
)
from run_scan_context_keyframe_recall import nearest_odom, registered_world_cloud_to_body
from run_structural_map_gate_validation import (
    LineSegment,
    angle_diff_180,
    build_structural_map,
    load_map_points,
    normalize_angle_180,
)


@dataclass(frozen=True)
class GraphNode:
    """场景图节点：来自墙线端点、墙线交点或强结构点。"""

    x: float
    y: float
    kind: int
    orientation_deg: float
    strength: float


@dataclass(frozen=True)
class TriangleDesc:
    """三角子结构描述子，用边长、节点类型和局部方向关系做匹配。"""

    indices: tuple[int, int, int]
    side_lengths: tuple[float, float, float]
    kind_hist: tuple[int, int, int]
    orientation_signature: tuple[float, float, float]


@dataclass(frozen=True)
class PoseCandidate:
    """一个 map->base 候选位姿及其无真值评分。"""

    x: float
    y: float
    yaw_deg: float
    consensus: int
    node_rmse: float
    line_score: float
    icp_fitness: float
    icp_rmse: float
    final_score: float
    source: str


@dataclass
class MapGraphDatabase:
    """地图侧结构图和三角描述子数据库。"""

    nodes: list[GraphNode]
    lines: list[LineSegment]
    descriptors: list[TriangleDesc]
    descriptor_bins: dict[tuple[int, int, int, int, int, int, int, int, int], list[int]]
    node_tree: cKDTree
    map_points: np.ndarray
    map_cloud: o3d.geometry.PointCloud


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 根目录。"""

    return Path(__file__).resolve().parents[3]


def normalize_angle_deg(angle: float) -> float:
    """把角度规整到 [-180, 180]，用于 yaw 误差和位姿输出。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def yaw_error_deg(candidate: float, reference: float) -> float:
    """计算 yaw 绝对误差，单位 deg。"""

    return abs(normalize_angle_deg(candidate - reference))


def make_o3d_cloud(points: np.ndarray, voxel_size: float) -> o3d.geometry.PointCloud:
    """把 numpy 点集转换为 Open3D 点云，并按参数做体素降采样。"""

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size)
    return cloud


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


def yaw_from_matrix(matrix: np.ndarray) -> float:
    """从 4x4 刚体矩阵中读取 yaw，单位 deg。"""

    return normalize_angle_deg(math.degrees(math.atan2(matrix[1, 0], matrix[0, 0])))


def transform_xy(points_xy: np.ndarray, x: float, y: float, yaw_deg: float) -> np.ndarray:
    """把 base 坐标二维点按候选位姿变换到 map 坐标。"""

    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    out = np.empty_like(points_xy, dtype=np.float64)
    out[:, 0] = c * points_xy[:, 0] - s * points_xy[:, 1] + x
    out[:, 1] = s * points_xy[:, 0] + c * points_xy[:, 1] + y
    return out


def preprocess_scan_points(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """按当前算法需要裁剪 scan 范围、高度并降采样。"""

    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    radius = np.linalg.norm(points[:, :3], axis=1)
    valid = (
        (radius >= args.min_scan_range)
        & (radius <= args.max_scan_range)
        & (points[:, 2] >= args.scan_min_z)
        & (points[:, 2] <= args.scan_max_z)
    )
    points = points[valid]
    cloud = make_o3d_cloud(points, args.scan_voxel_size)
    points = np.asarray(cloud.points, dtype=np.float64)
    if args.max_scan_points > 0 and len(points) > args.max_scan_points:
        step = int(math.ceil(len(points) / args.max_scan_points))
        points = points[::step][: args.max_scan_points]
    return points


def scan_lines_from_base_points(points: np.ndarray, args: argparse.Namespace) -> list[LineSegment]:
    """从当前 scan 的 base 坐标点云提取二维墙线/结构线。"""

    if len(points) < args.scan_min_points_for_lines:
        return []
    points_xy = points[:, :2]
    min_xy = points_xy.min(axis=0) - args.scan_patch_margin_m
    max_xy = points_xy.max(axis=0) + args.scan_patch_margin_m
    size = np.ceil((max_xy - min_xy) / args.scan_line_resolution).astype(np.int32) + 1
    if np.any(size <= 2):
        return []

    image = np.zeros((int(size[1]), int(size[0])), dtype=np.uint8)
    ix = np.floor((points_xy[:, 0] - min_xy[0]) / args.scan_line_resolution).astype(np.int32)
    iy = np.floor((points_xy[:, 1] - min_xy[1]) / args.scan_line_resolution).astype(np.int32)
    valid = (ix >= 0) & (iy >= 0) & (ix < size[0]) & (iy < size[1])
    image[iy[valid], ix[valid]] = 255

    if args.scan_close_kernel > 1:
        kernel = np.ones((args.scan_close_kernel, args.scan_close_kernel), dtype=np.uint8)
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
    if args.scan_dilate_kernel > 1:
        kernel = np.ones((args.scan_dilate_kernel, args.scan_dilate_kernel), dtype=np.uint8)
        image = cv2.dilate(image, kernel, iterations=1)

    raw_lines = cv2.HoughLinesP(
        image,
        rho=1,
        theta=np.pi / 180.0,
        threshold=args.scan_hough_threshold,
        minLineLength=max(1, int(args.scan_min_line_length_m / args.scan_line_resolution)),
        maxLineGap=max(1, int(args.scan_hough_max_gap_m / args.scan_line_resolution)),
    )
    if raw_lines is None:
        return []

    lines: list[LineSegment] = []
    for raw in raw_lines[:, 0, :]:
        px1, py1, px2, py2 = (int(v) for v in raw)
        x1 = float(min_xy[0] + px1 * args.scan_line_resolution)
        y1 = float(min_xy[1] + py1 * args.scan_line_resolution)
        x2 = float(min_xy[0] + px2 * args.scan_line_resolution)
        y2 = float(min_xy[1] + py2 * args.scan_line_resolution)
        length = math.hypot(x2 - x1, y2 - y1)
        if length < args.scan_min_line_length_m:
            continue
        angle = normalize_angle_180(math.degrees(math.atan2(y2 - y1, x2 - x1)))
        lines.append(LineSegment(x1, y1, x2, y2, angle, length, 0.5 * (x1 + x2), 0.5 * (y1 + y2)))
    return merge_lines_for_graph(lines, args.line_merge_angle_deg, args.line_merge_distance_m, args.line_merge_midpoint_m)


def point_line_distance(x: float, y: float, line: LineSegment) -> float:
    """计算点到线段所在直线的距离。"""

    vx = line.x2 - line.x1
    vy = line.y2 - line.y1
    denom = max(math.hypot(vx, vy), 1e-9)
    return abs(vy * x - vx * y + line.x2 * line.y1 - line.y2 * line.x1) / denom


def merge_lines_for_graph(
    lines: list[LineSegment],
    angle_thresh_deg: float,
    distance_thresh_m: float,
    midpoint_thresh_m: float,
) -> list[LineSegment]:
    """合并近似重叠的 Hough 线，避免同一面墙贡献大量重复节点。"""

    merged: list[LineSegment] = []
    for line in sorted(lines, key=lambda item: item.length_m, reverse=True):
        duplicate = False
        for kept in merged:
            if angle_diff_180(line.angle_deg, kept.angle_deg) > angle_thresh_deg:
                continue
            if point_line_distance(line.mid_x, line.mid_y, kept) > distance_thresh_m:
                continue
            if math.hypot(line.mid_x - kept.mid_x, line.mid_y - kept.mid_y) > midpoint_thresh_m:
                continue
            duplicate = True
            break
        if not duplicate:
            merged.append(line)
    return merged


def segment_intersection(a: LineSegment, b: LineSegment, max_distance_m: float) -> tuple[float, float] | None:
    """计算两条近似非平行线段所在直线交点，并要求交点离两线段中点不要过远。"""

    if angle_diff_180(a.angle_deg, b.angle_deg) < 20.0:
        return None
    x1, y1, x2, y2 = a.x1, a.y1, a.x2, a.y2
    x3, y3, x4, y4 = b.x1, b.y1, b.x2, b.y2
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-9:
        return None
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
    if math.hypot(px - a.mid_x, py - a.mid_y) > max_distance_m:
        return None
    if math.hypot(px - b.mid_x, py - b.mid_y) > max_distance_m:
        return None
    return float(px), float(py)


def nodes_from_lines(lines: list[LineSegment], args: argparse.Namespace, max_nodes: int) -> list[GraphNode]:
    """从线段端点、交点和中点生成场景图节点。"""

    nodes: list[GraphNode] = []
    for line in sorted(lines, key=lambda item: item.length_m, reverse=True)[: args.max_lines_for_nodes]:
        nodes.append(GraphNode(line.x1, line.y1, 1, line.angle_deg, line.length_m))
        nodes.append(GraphNode(line.x2, line.y2, 1, line.angle_deg, line.length_m))
        if line.length_m >= args.min_line_midpoint_node_length_m:
            nodes.append(GraphNode(line.mid_x, line.mid_y, 3, line.angle_deg, line.length_m))

    long_lines = sorted(lines, key=lambda item: item.length_m, reverse=True)[: args.max_intersection_lines]
    for a, b in itertools.combinations(long_lines, 2):
        point = segment_intersection(a, b, args.max_intersection_midpoint_distance_m)
        if point is None:
            continue
        orientation = normalize_angle_180(0.5 * (a.angle_deg + b.angle_deg))
        nodes.append(GraphNode(point[0], point[1], 2, orientation, min(a.length_m, b.length_m)))

    return deduplicate_nodes(nodes, args.node_dedup_radius_m, max_nodes)


def iss_nodes_from_points(points: np.ndarray, args: argparse.Namespace, max_nodes: int, voxel_size: float) -> list[GraphNode]:
    """从三维点云提取 ISS keypoints，并投影成 2D 场景图节点。

    官方 Outram 使用语义实例中心点作为图节点；当前 bag 没有逐点语义标签。
    这里使用 ISS 关键点作为几何实例点，并按高度分成低/中/高三类，保留完整三角图匹配流程。
    """

    if len(points) < 50:
        return []
    cloud = make_o3d_cloud(points, voxel_size)
    if len(cloud.points) < 50:
        return []
    keypoints = o3d.geometry.keypoint.compute_iss_keypoints(
        cloud,
        salient_radius=args.iss_salient_radius_m,
        non_max_radius=args.iss_non_max_radius_m,
        gamma_21=args.iss_gamma_21,
        gamma_32=args.iss_gamma_32,
        min_neighbors=args.iss_min_neighbors,
    )
    kp = np.asarray(keypoints.points, dtype=np.float64)
    if len(kp) == 0:
        return []
    nodes: list[GraphNode] = []
    for point in kp:
        if point[2] < args.iss_low_z_m:
            kind = 1
        elif point[2] < args.iss_high_z_m:
            kind = 2
        else:
            kind = 3
        # 没有语义实例法向时，用 keypoint 相对原点方位作为弱方向签名；三角描述子里只使用方向差。
        orientation = normalize_angle_180(math.degrees(math.atan2(point[1], point[0])))
        strength = float(np.linalg.norm(point[:2]))
        nodes.append(GraphNode(float(point[0]), float(point[1]), kind, orientation, strength))
    return deduplicate_nodes(nodes, args.node_dedup_radius_m, max_nodes)


def deduplicate_nodes(nodes: list[GraphNode], radius_m: float, max_nodes: int) -> list[GraphNode]:
    """按空间距离去重节点，保留结构强度更高的节点。"""

    selected: list[GraphNode] = []
    for node in sorted(nodes, key=lambda item: item.strength, reverse=True):
        if any(math.hypot(node.x - kept.x, node.y - kept.y) <= radius_m for kept in selected):
            continue
        selected.append(node)
        if len(selected) >= max_nodes:
            break
    return selected


def triangle_descriptor(nodes: list[GraphNode], indices: tuple[int, int, int], args: argparse.Namespace) -> TriangleDesc | None:
    """为三个节点构造三角描述子，过滤太小/太大的不稳定三角形。"""

    pts = np.asarray([[nodes[i].x, nodes[i].y] for i in indices], dtype=np.float64)
    d01 = float(np.linalg.norm(pts[0] - pts[1]))
    d02 = float(np.linalg.norm(pts[0] - pts[2]))
    d12 = float(np.linalg.norm(pts[1] - pts[2]))
    lengths = sorted([d01, d02, d12])
    if lengths[0] < args.triangle_min_side_m or lengths[2] > args.triangle_max_side_m:
        return None
    area = abs(np.cross(pts[1] - pts[0], pts[2] - pts[0])) * 0.5
    if area < args.triangle_min_area_m2:
        return None
    kinds = [nodes[i].kind for i in indices]
    kind_hist = (kinds.count(1), kinds.count(2), kinds.count(3))
    # 全局重定位时 yaw 未知，因此不能把节点的绝对方向直接放进 key。
    # 这里使用三条节点主方向之间的无向夹角差，作为旋转不变量保留局部结构形状。
    orientations = [normalize_angle_180(nodes[i].orientation_deg) for i in indices]
    orientation_diffs = sorted(
        [
            min(abs(orientations[0] - orientations[1]), 180.0 - abs(orientations[0] - orientations[1])),
            min(abs(orientations[0] - orientations[2]), 180.0 - abs(orientations[0] - orientations[2])),
            min(abs(orientations[1] - orientations[2]), 180.0 - abs(orientations[1] - orientations[2])),
        ]
    )
    return TriangleDesc(indices, (lengths[0], lengths[1], lengths[2]), kind_hist, (orientation_diffs[0], orientation_diffs[1], orientation_diffs[2]))


def descriptor_key(desc: TriangleDesc, args: argparse.Namespace) -> tuple[int, int, int, int, int, int, int, int, int]:
    """把连续描述子量化到哈希桶，加速全局候选匹配。"""

    q_len = [int(round(value / args.triangle_length_bin_m)) for value in desc.side_lengths]
    q_ori = [int(round(value / args.triangle_orientation_bin_deg)) for value in desc.orientation_signature]
    return (
        q_len[0],
        q_len[1],
        q_len[2],
        desc.kind_hist[0],
        desc.kind_hist[1],
        desc.kind_hist[2],
        q_ori[0],
        q_ori[1],
        q_ori[2],
    )


def build_triangle_descriptors(nodes: list[GraphNode], args: argparse.Namespace, for_map: bool) -> list[TriangleDesc]:
    """按近邻组合生成稳定三角子结构描述子。"""

    if len(nodes) < 3:
        return []
    xy = np.asarray([[node.x, node.y] for node in nodes], dtype=np.float64)
    tree = cKDTree(xy)
    descriptors: list[TriangleDesc] = []
    near_k = args.map_triangle_neighbor_k if for_map else args.scan_triangle_neighbor_k
    for i, point in enumerate(xy):
        _, idx = tree.query(point, k=min(len(nodes), near_k + 1))
        idx = [int(v) for v in np.atleast_1d(idx) if int(v) != i]
        for j, k in itertools.combinations(idx, 2):
            desc = triangle_descriptor(nodes, (i, j, k), args)
            if desc is not None:
                descriptors.append(desc)
                if len(descriptors) >= (args.max_map_descriptors if for_map else args.max_scan_descriptors):
                    return descriptors
    return descriptors


def build_descriptor_bins(descriptors: list[TriangleDesc], args: argparse.Namespace) -> dict[tuple[int, int, int, int, int, int, int, int, int], list[int]]:
    """建立地图三角描述子哈希桶。"""

    bins: dict[tuple[int, int, int, int, int, int, int, int, int], list[int]] = {}
    for index, desc in enumerate(descriptors):
        bins.setdefault(descriptor_key(desc, args), []).append(index)
    return bins


def build_map_database(args: argparse.Namespace) -> MapGraphDatabase:
    """构建地图结构图、三角描述子和 Open3D 地图点云。"""

    map_path = args.map if args.map.is_absolute() else args.workspace / args.map
    map_points = load_map_points(map_path, args)
    structural_map = build_structural_map(map_points, args)
    map_lines = merge_lines_for_graph(
        structural_map.lines,
        args.line_merge_angle_deg,
        args.line_merge_distance_m,
        args.line_merge_midpoint_m,
    )
    if args.node_source == "iss":
        nodes = iss_nodes_from_points(map_points, args, args.max_map_nodes, args.iss_map_voxel_size)
    else:
        nodes = nodes_from_lines(map_lines, args, args.max_map_nodes)
    descriptors = build_triangle_descriptors(nodes, args, for_map=True)
    bins = build_descriptor_bins(descriptors, args)
    node_xy = np.asarray([[node.x, node.y] for node in nodes], dtype=np.float64)
    node_tree = cKDTree(node_xy) if len(node_xy) else cKDTree(np.zeros((1, 2), dtype=np.float64))
    map_cloud = make_o3d_cloud(map_points, args.icp_map_voxel_size)
    print(
        f"[scene_graph] map_points={len(map_points)} lines={len(map_lines)} nodes={len(nodes)} "
        f"triangles={len(descriptors)} bins={len(bins)} rss_mb={current_rss_mb():.1f}",
        flush=True,
    )
    return MapGraphDatabase(nodes, map_lines, descriptors, bins, node_tree, map_points, map_cloud)


def descriptor_distance(a: TriangleDesc, b: TriangleDesc) -> float:
    """计算两个三角描述子的连续距离，用于同桶内排序。"""

    len_dist = sum(abs(x - y) for x, y in zip(a.side_lengths, b.side_lengths))
    kind_dist = sum(abs(x - y) for x, y in zip(a.kind_hist, b.kind_hist))
    ori_dist = sum(min(abs(x - y), 180.0 - abs(x - y)) for x, y in zip(a.orientation_signature, b.orientation_signature)) / 180.0
    return len_dist + 0.5 * kind_dist + ori_dist


def estimate_pose_from_triangles(
    scan_nodes: list[GraphNode],
    scan_desc: TriangleDesc,
    map_nodes: list[GraphNode],
    map_desc: TriangleDesc,
) -> tuple[float, float, float] | None:
    """从一对三角形估计二维刚体变换，并尝试所有顶点排列取残差最小者。"""

    src = np.asarray([[scan_nodes[i].x, scan_nodes[i].y] for i in scan_desc.indices], dtype=np.float64)
    tgt_raw = np.asarray([[map_nodes[i].x, map_nodes[i].y] for i in map_desc.indices], dtype=np.float64)
    best: tuple[float, float, float, float] | None = None
    for perm in itertools.permutations(range(3)):
        tgt = tgt_raw[list(perm)]
        src_center = src.mean(axis=0)
        tgt_center = tgt.mean(axis=0)
        src0 = src - src_center
        tgt0 = tgt - tgt_center
        h = src0.T @ tgt0
        u, _, vt = np.linalg.svd(h)
        r = vt.T @ u.T
        if np.linalg.det(r) < 0.0:
            vt[-1, :] *= -1.0
            r = vt.T @ u.T
        transformed = (src0 @ r.T) + tgt_center
        rmse = float(np.sqrt(np.mean(np.sum((transformed - tgt) ** 2, axis=1))))
        yaw = normalize_angle_deg(math.degrees(math.atan2(r[1, 0], r[0, 0])))
        t = tgt_center - (r @ src_center)
        candidate = (rmse, float(t[0]), float(t[1]), yaw)
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None or best[0] > 0.5:
        return None
    return best[1], best[2], best[3]


def line_alignment_score(lines: list[LineSegment], map_lines: list[LineSegment], x: float, y: float, yaw_deg: float, args: argparse.Namespace) -> float:
    """用 scan 线段变换后与地图线段的方向/位置一致性给候选打分。"""

    if not lines or not map_lines:
        return 0.0
    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    score = 0.0
    total = 0.0
    for line in sorted(lines, key=lambda item: item.length_m, reverse=True)[: args.line_score_top_n]:
        mx = c * line.mid_x - s * line.mid_y + x
        my = s * line.mid_x + c * line.mid_y + y
        angle = normalize_angle_180(line.angle_deg + yaw_deg)
        total += line.length_m
        best = 0.0
        for map_line in map_lines:
            if math.hypot(mx - map_line.mid_x, my - map_line.mid_y) > args.line_score_radius_m:
                continue
            angle_gap = angle_diff_180(angle, map_line.angle_deg)
            if angle_gap > args.line_score_angle_deg:
                continue
            dist = point_line_distance(mx, my, map_line)
            if dist > args.line_score_distance_m:
                continue
            value = math.exp(-dist / max(args.line_score_distance_m, 1e-6)) * (1.0 - angle_gap / args.line_score_angle_deg)
            best = max(best, value * line.length_m)
        score += best
    return score / max(total, 1e-6)


def score_pose_by_nodes(
    scan_nodes: list[GraphNode],
    map_db: MapGraphDatabase,
    lines: list[LineSegment],
    x: float,
    y: float,
    yaw_deg: float,
    args: argparse.Namespace,
) -> tuple[int, float, float, float]:
    """对一个候选做节点一致性和线段一致性评分。"""

    if not scan_nodes:
        return 0, 999.0, 0.0, 999.0
    scan_xy = np.asarray([[node.x, node.y] for node in scan_nodes], dtype=np.float64)
    transformed = transform_xy(scan_xy, x, y, yaw_deg)
    dists, _ = map_db.node_tree.query(transformed, k=1)
    inlier = dists <= args.node_match_radius_m
    consensus = int(inlier.sum())
    rmse = float(np.sqrt(np.mean(np.minimum(dists, args.node_match_radius_m * 2.0) ** 2)))
    line_score = line_alignment_score(lines, map_db.lines, x, y, yaw_deg, args)
    final_score = -float(consensus) + args.node_rmse_weight * rmse - args.line_score_weight * line_score
    return consensus, rmse, line_score, final_score


def generate_pose_candidates(
    scan_nodes: list[GraphNode],
    scan_lines: list[LineSegment],
    scan_descs: list[TriangleDesc],
    map_db: MapGraphDatabase,
    args: argparse.Namespace,
) -> list[PoseCandidate]:
    """三角描述子匹配生成全局位姿候选，并用节点/线段共识排序。"""

    raw_candidates: list[PoseCandidate] = []
    for scan_desc in scan_descs:
        key = descriptor_key(scan_desc, args)
        map_indices = map_db.descriptor_bins.get(key, [])
        if not map_indices:
            continue
        ranked = sorted(map_indices, key=lambda idx: descriptor_distance(scan_desc, map_db.descriptors[idx]))[: args.max_map_matches_per_scan_triangle]
        for map_index in ranked:
            pose = estimate_pose_from_triangles(scan_nodes, scan_desc, map_db.nodes, map_db.descriptors[map_index])
            if pose is None:
                continue
            x, y, yaw = pose
            consensus, rmse, line_score, final_score = score_pose_by_nodes(scan_nodes, map_db, scan_lines, x, y, yaw, args)
            if consensus < args.min_node_consensus:
                continue
            raw_candidates.append(
                PoseCandidate(
                    x=x,
                    y=y,
                    yaw_deg=yaw,
                    consensus=consensus,
                    node_rmse=rmse,
                    line_score=line_score,
                    icp_fitness=0.0,
                    icp_rmse=999.0,
                    final_score=final_score,
                    source="triangle_graph",
                )
            )
            if len(raw_candidates) >= args.max_raw_pose_candidates:
                break
        if len(raw_candidates) >= args.max_raw_pose_candidates:
            break
    raw_candidates.sort(key=lambda item: item.final_score)
    return deduplicate_candidates(raw_candidates, args)[: args.pose_top_k_before_icp]


def deduplicate_candidates(candidates: list[PoseCandidate], args: argparse.Namespace) -> list[PoseCandidate]:
    """对接近的候选做 NMS，避免同一解占满 top-K。"""

    selected: list[PoseCandidate] = []
    for candidate in candidates:
        duplicate = False
        for kept in selected:
            if math.hypot(candidate.x - kept.x, candidate.y - kept.y) <= args.pose_nms_xy_m and yaw_error_deg(candidate.yaw_deg, kept.yaw_deg) <= args.pose_nms_yaw_deg:
                duplicate = True
                break
        if not duplicate:
            selected.append(candidate)
    return selected


def crop_map_points(map_points: np.ndarray, x: float, y: float, radius_m: float) -> np.ndarray:
    """按候选位置裁剪局部地图点云，减少 ICP 计算量。"""

    dx = map_points[:, 0] - x
    dy = map_points[:, 1] - y
    keep = (dx * dx + dy * dy) <= radius_m * radius_m
    return map_points[keep]


def refine_candidate_with_icp(scan_points: np.ndarray, map_db: MapGraphDatabase, candidate: PoseCandidate, args: argparse.Namespace) -> PoseCandidate:
    """用 Open3D ICP/GICP 对候选做局部精修。"""

    local_map = crop_map_points(map_db.map_points, candidate.x, candidate.y, args.icp_map_crop_radius_m)
    if len(local_map) < args.icp_min_map_points or len(scan_points) < args.icp_min_scan_points:
        return candidate
    scan_cloud = make_o3d_cloud(scan_points, args.icp_scan_voxel_size)
    map_cloud = make_o3d_cloud(local_map, args.icp_map_voxel_size)
    init = pose_matrix_2d(candidate.x, candidate.y, candidate.yaw_deg)
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=args.icp_iterations)
    if args.icp_method == "gicp":
        result = o3d.pipelines.registration.registration_generalized_icp(
            scan_cloud,
            map_cloud,
            args.icp_max_correspondence_m,
            init,
            o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
            criteria,
        )
    else:
        result = o3d.pipelines.registration.registration_icp(
            scan_cloud,
            map_cloud,
            args.icp_max_correspondence_m,
            init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria,
        )
    pose = result.transformation
    final_x = float(pose[0, 3])
    final_y = float(pose[1, 3])
    final_yaw = yaw_from_matrix(pose)
    final_score = candidate.final_score - args.icp_fitness_weight * float(result.fitness) + args.icp_rmse_weight * float(result.inlier_rmse)
    return PoseCandidate(
        x=final_x,
        y=final_y,
        yaw_deg=final_yaw,
        consensus=candidate.consensus,
        node_rmse=candidate.node_rmse,
        line_score=candidate.line_score,
        icp_fitness=float(result.fitness),
        icp_rmse=float(result.inlier_rmse),
        final_score=final_score,
        source="triangle_graph_icp",
    )


def evaluate_sample(sample, cloud, odoms, map_db: MapGraphDatabase, args: argparse.Namespace) -> tuple[list[PoseCandidate], dict[str, float | int]]:
    """对单帧执行完整 scene graph relocalization。"""

    odom = nearest_odom(odoms, cloud.stamp_sec)
    raw_points = registered_world_cloud_to_body(cloud, odom)
    scan_points = preprocess_scan_points(raw_points, args)
    scan_lines = scan_lines_from_base_points(scan_points, args)
    if args.node_source == "iss":
        scan_nodes = iss_nodes_from_points(scan_points, args, args.max_scan_nodes, args.iss_scan_voxel_size)
    else:
        scan_nodes = nodes_from_lines(scan_lines, args, args.max_scan_nodes)
    scan_descs = build_triangle_descriptors(scan_nodes, args, for_map=False)
    candidates = generate_pose_candidates(scan_nodes, scan_lines, scan_descs, map_db, args)
    refined = [refine_candidate_with_icp(scan_points, map_db, candidate, args) for candidate in candidates[: args.icp_refine_top_k]]
    refined.extend(candidates[args.icp_refine_top_k:])
    refined.sort(key=lambda item: item.final_score)
    stats = {
        "scan_points": len(scan_points),
        "scan_lines": len(scan_lines),
        "scan_nodes": len(scan_nodes),
        "scan_triangles": len(scan_descs),
        "raw_candidates": len(candidates),
    }
    return refined[: args.output_top_k], stats


def pose_errors(candidate: PoseCandidate, sample) -> tuple[float, float]:
    """计算候选相对参考位姿的平移/yaw 误差。"""

    xy = math.hypot(candidate.x - sample.reference_x, candidate.y - sample.reference_y)
    yaw = yaw_error_deg(candidate.yaw_deg, sample.reference_yaw_deg)
    return xy, yaw


def run(args: argparse.Namespace) -> int:
    """主流程：构建地图 scene graph，逐帧执行全局重定位并输出统计。"""

    start_wall = time.time()
    start_cpu = current_cpu_seconds()
    args.workspace = args.workspace.resolve()
    if not args.output.is_absolute():
        args.output = args.workspace / args.output
    if not args.summary_output.is_absolute():
        args.summary_output = args.workspace / args.summary_output

    samples = load_metric_samples(args.metrics_glob, args.scenario_name, args.max_samples)
    if not samples:
        raise RuntimeError("没有读取到 metrics 样本")
    bags = sorted({sample.bag_path for sample in samples})
    if len(bags) != 1:
        raise RuntimeError(f"当前验证脚本只支持单 bag，实际={bags}")
    target_indices = {sample.bag_frame_index for sample in samples}

    map_db = build_map_database(args)
    odoms, clouds = read_bag_clouds(bags[0], target_indices)
    print(f"[scene_graph] samples={len(samples)} clouds={len(clouds)} odoms={len(odoms)}", flush=True)

    detail_rows: list[dict[str, str]] = []
    summary_counts = {
        "processed": 0,
        "top1_0p2m_3deg": 0,
        "top1_0p3m_5deg": 0,
        "top1_0p5m_10deg": 0,
        "topk_0p2m_3deg": 0,
        "topk_0p3m_5deg": 0,
        "topk_0p5m_10deg": 0,
        "no_candidate": 0,
    }
    for index, sample in enumerate(samples, start=1):
        frame_start = time.time()
        cloud = clouds.get(sample.bag_frame_index)
        if cloud is None:
            summary_counts["no_candidate"] += 1
            continue
        candidates, stats = evaluate_sample(sample, cloud, odoms, map_db, args)
        if not candidates:
            summary_counts["no_candidate"] += 1
            continue
        summary_counts["processed"] += 1
        top1_xy, top1_yaw = pose_errors(candidates[0], sample)
        topk_errors = [pose_errors(candidate, sample) for candidate in candidates]
        summary_counts["top1_0p2m_3deg"] += int(top1_xy <= 0.2 and top1_yaw <= 3.0)
        summary_counts["top1_0p3m_5deg"] += int(top1_xy <= 0.3 and top1_yaw <= 5.0)
        summary_counts["top1_0p5m_10deg"] += int(top1_xy <= 0.5 and top1_yaw <= 10.0)
        summary_counts["topk_0p2m_3deg"] += int(any(xy <= 0.2 and yaw <= 3.0 for xy, yaw in topk_errors))
        summary_counts["topk_0p3m_5deg"] += int(any(xy <= 0.3 and yaw <= 5.0 for xy, yaw in topk_errors))
        summary_counts["topk_0p5m_10deg"] += int(any(xy <= 0.5 and yaw <= 10.0 for xy, yaw in topk_errors))
        elapsed_ms = (time.time() - frame_start) * 1000.0
        for rank, candidate in enumerate(candidates, start=1):
            xy, yaw = pose_errors(candidate, sample)
            detail_rows.append(
                {
                    "source_id": sample.source_id,
                    "bag_frame_index": str(sample.bag_frame_index),
                    "rank": str(rank),
                    "x": f"{candidate.x:.6f}",
                    "y": f"{candidate.y:.6f}",
                    "yaw_deg": f"{candidate.yaw_deg:.6f}",
                    "consensus": str(candidate.consensus),
                    "node_rmse": f"{candidate.node_rmse:.6f}",
                    "line_score": f"{candidate.line_score:.6f}",
                    "icp_fitness": f"{candidate.icp_fitness:.6f}",
                    "icp_rmse": f"{candidate.icp_rmse:.6f}",
                    "final_score": f"{candidate.final_score:.6f}",
                    "source": candidate.source,
                    "reference_x": f"{sample.reference_x:.6f}",
                    "reference_y": f"{sample.reference_y:.6f}",
                    "reference_yaw_deg": f"{sample.reference_yaw_deg:.6f}",
                    "translation_error_m": f"{xy:.6f}",
                    "yaw_error_deg": f"{yaw:.6f}",
                    "success_0p2m_3deg": "1" if xy <= 0.2 and yaw <= 3.0 else "0",
                    "success_0p3m_5deg": "1" if xy <= 0.3 and yaw <= 5.0 else "0",
                    "success_0p5m_10deg": "1" if xy <= 0.5 and yaw <= 10.0 else "0",
                    "scan_points": str(stats["scan_points"]),
                    "scan_lines": str(stats["scan_lines"]),
                    "scan_nodes": str(stats["scan_nodes"]),
                    "scan_triangles": str(stats["scan_triangles"]),
                    "raw_candidates": str(stats["raw_candidates"]),
                    "elapsed_ms": f"{elapsed_ms:.3f}",
                }
            )
        if index % max(1, args.progress_interval) == 0:
            print(f"[scene_graph] progress {index}/{len(samples)}", flush=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if detail_rows:
        with args.output.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(detail_rows[0].keys()))
            writer.writeheader()
            writer.writerows(detail_rows)
    else:
        args.output.write_text("", encoding="utf-8")

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
    summary_rows.append({"metric": "no_candidate", "success": str(summary_counts["no_candidate"]), "total": str(len(samples)), "rate": f"{summary_counts['no_candidate'] / max(1, len(samples)):.6f}"})
    args.summary_output.parent.mkdir(parents=True, exist_ok=True)
    with args.summary_output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["metric", "success", "total", "rate"])
        writer.writeheader()
        writer.writerows(summary_rows)

    elapsed = time.time() - start_wall
    cpu_core = (current_cpu_seconds() - start_cpu) / max(elapsed, 1e-6)
    print(f"[scene_graph] details={args.output}", flush=True)
    print(f"[scene_graph] summary={args.summary_output}", flush=True)
    print(f"[scene_graph] elapsed_sec={elapsed:.3f} cpu_core_equiv={cpu_core:.2f} rss_mb={current_rss_mb():.1f}", flush=True)
    for row in summary_rows:
        print(f"[scene_graph] {row['metric']}={row['success']}/{row['total']} rate={row['rate']}", flush=True)
    return 0


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""

    root = repo_root_from_script()
    parser = argparse.ArgumentParser(description="验证几何 scene graph / Outram-style 全局重定位。")
    parser.add_argument("--workspace", type=Path, default=root, help="humanoid_ws 工作空间")
    parser.add_argument("--metrics-glob", action="append", default=[], help="输入 evaluator metrics CSV glob")
    parser.add_argument("--scenario-name", default="arbitrary_start_no_prior", help="验证场景")
    parser.add_argument("--max-samples", type=int, default=0, help="最多验证多少帧；0 表示全量")
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"), help="地图 PCD")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/geometric_scene_graph/details.csv"), help="候选明细 CSV")
    parser.add_argument("--summary-output", type=Path, default=Path(".codex_tmp/geometric_scene_graph/summary.csv"), help="汇总 CSV")
    parser.add_argument("--progress-interval", type=int, default=5, help="进度打印间隔")
    parser.add_argument("--node-source", choices=["lines", "iss"], default="lines", help="场景图节点来源：lines=结构线端点/交点，iss=3D ISS keypoints")

    parser.add_argument("--map-voxel-size", type=float, default=0.10, help="结构图地图降采样")
    parser.add_argument("--map-min-z", type=float, default=0.15, help="地图最低高度")
    parser.add_argument("--map-max-z", type=float, default=2.20, help="地图最高高度")
    parser.add_argument("--structural-resolution", type=float, default=0.10, help="地图结构线投影分辨率")
    parser.add_argument("--map-margin-m", type=float, default=2.0, help="地图结构图外扩")
    parser.add_argument("--map-close-kernel", type=int, default=3, help="地图结构图闭运算核")
    parser.add_argument("--map-dilate-kernel", type=int, default=2, help="地图结构图膨胀核")
    parser.add_argument("--hough-threshold", type=int, default=18, help="地图 Hough 阈值")
    parser.add_argument("--hough-max-gap-m", type=float, default=0.45, help="地图 Hough 最大连接间隙")
    parser.add_argument("--map-min-line-length-m", type=float, default=1.2, help="地图最短线段")
    parser.add_argument("--merge-angle-deg", type=float, default=6.0, help="兼容结构图工具的地图线合并角度")
    parser.add_argument("--merge-distance-m", type=float, default=0.25, help="兼容结构图工具的地图线合并距离")
    parser.add_argument("--merge-midpoint-m", type=float, default=1.0, help="兼容结构图工具的地图线合并中点距离")

    parser.add_argument("--min-scan-range", type=float, default=0.35, help="scan 最小距离")
    parser.add_argument("--max-scan-range", type=float, default=18.0, help="scan 最大距离")
    parser.add_argument("--scan-min-z", type=float, default=0.20, help="scan 最低高度")
    parser.add_argument("--scan-max-z", type=float, default=2.50, help="scan 最高高度")
    parser.add_argument("--scan-voxel-size", type=float, default=0.12, help="scan 结构提取前降采样")
    parser.add_argument("--max-scan-points", type=int, default=12000, help="scan 最多参与 ICP 的点数")
    parser.add_argument("--scan-min-points-for-lines", type=int, default=200, help="提线最少点数")
    parser.add_argument("--scan-line-resolution", type=float, default=0.08, help="scan Hough 投影分辨率")
    parser.add_argument("--scan-patch-margin-m", type=float, default=1.0, help="scan patch 外扩")
    parser.add_argument("--scan-close-kernel", type=int, default=3, help="scan 投影闭运算核")
    parser.add_argument("--scan-dilate-kernel", type=int, default=2, help="scan 投影膨胀核")
    parser.add_argument("--scan-hough-threshold", type=int, default=14, help="scan Hough 阈值")
    parser.add_argument("--scan-hough-max-gap-m", type=float, default=0.35, help="scan Hough 最大连接间隙")
    parser.add_argument("--scan-min-line-length-m", type=float, default=0.9, help="scan 最短结构线")

    parser.add_argument("--line-merge-angle-deg", type=float, default=6.0, help="线段去重角度")
    parser.add_argument("--line-merge-distance-m", type=float, default=0.25, help="线段去重垂距")
    parser.add_argument("--line-merge-midpoint-m", type=float, default=1.0, help="线段去重中点距离")
    parser.add_argument("--max-lines-for-nodes", type=int, default=120, help="最多从多少线段生成节点")
    parser.add_argument("--max-intersection-lines", type=int, default=45, help="最多用多少长线求交点")
    parser.add_argument("--max-intersection-midpoint-distance-m", type=float, default=8.0, help="交点离线段中点最大距离")
    parser.add_argument("--min-line-midpoint-node-length-m", type=float, default=2.0, help="长线中点节点最小线长")
    parser.add_argument("--node-dedup-radius-m", type=float, default=0.35, help="节点去重半径")
    parser.add_argument("--max-map-nodes", type=int, default=420, help="地图最大节点数")
    parser.add_argument("--max-scan-nodes", type=int, default=90, help="scan 最大节点数")
    parser.add_argument("--iss-map-voxel-size", type=float, default=0.15, help="ISS 地图预降采样")
    parser.add_argument("--iss-scan-voxel-size", type=float, default=0.10, help="ISS scan 预降采样")
    parser.add_argument("--iss-salient-radius-m", type=float, default=0.8, help="ISS salient radius")
    parser.add_argument("--iss-non-max-radius-m", type=float, default=0.5, help="ISS non-max radius")
    parser.add_argument("--iss-gamma-21", type=float, default=0.975, help="ISS gamma21")
    parser.add_argument("--iss-gamma-32", type=float, default=0.975, help="ISS gamma32")
    parser.add_argument("--iss-min-neighbors", type=int, default=5, help="ISS 最少邻居")
    parser.add_argument("--iss-low-z-m", type=float, default=0.8, help="ISS 低高度类型阈值")
    parser.add_argument("--iss-high-z-m", type=float, default=1.6, help="ISS 高高度类型阈值")

    parser.add_argument("--triangle-min-side-m", type=float, default=1.0, help="三角形最短边")
    parser.add_argument("--triangle-max-side-m", type=float, default=16.0, help="三角形最长边")
    parser.add_argument("--triangle-min-area-m2", type=float, default=0.8, help="三角形最小面积")
    parser.add_argument("--triangle-length-bin-m", type=float, default=0.35, help="三角边长量化")
    parser.add_argument("--triangle-orientation-bin-deg", type=float, default=15.0, help="三角节点方向量化")
    parser.add_argument("--map-triangle-neighbor-k", type=int, default=12, help="地图每个节点近邻三角组合数")
    parser.add_argument("--scan-triangle-neighbor-k", type=int, default=12, help="scan 每个节点近邻三角组合数")
    parser.add_argument("--max-map-descriptors", type=int, default=50000, help="最多地图三角描述子")
    parser.add_argument("--max-scan-descriptors", type=int, default=8000, help="最多 scan 三角描述子")
    parser.add_argument("--max-map-matches-per-scan-triangle", type=int, default=8, help="每个 scan 三角最多匹配地图三角")
    parser.add_argument("--max-raw-pose-candidates", type=int, default=4000, help="最多原始姿态候选")

    parser.add_argument("--node-match-radius-m", type=float, default=0.55, help="节点共识匹配半径")
    parser.add_argument("--min-node-consensus", type=int, default=4, help="最少节点共识数量")
    parser.add_argument("--node-rmse-weight", type=float, default=2.0, help="节点 RMSE 评分权重")
    parser.add_argument("--line-score-weight", type=float, default=3.0, help="线段一致性评分权重")
    parser.add_argument("--line-score-top-n", type=int, default=20, help="用于线段评分的 scan 线数")
    parser.add_argument("--line-score-radius-m", type=float, default=3.0, help="线段匹配中点半径")
    parser.add_argument("--line-score-angle-deg", type=float, default=12.0, help="线段匹配角度阈值")
    parser.add_argument("--line-score-distance-m", type=float, default=0.5, help="线段匹配垂距阈值")

    parser.add_argument("--pose-top-k-before-icp", type=int, default=40, help="ICP 前保留候选数")
    parser.add_argument("--pose-nms-xy-m", type=float, default=0.7, help="候选 NMS xy 阈值")
    parser.add_argument("--pose-nms-yaw-deg", type=float, default=8.0, help="候选 NMS yaw 阈值")
    parser.add_argument("--output-top-k", type=int, default=20, help="输出 top-K")

    parser.add_argument("--icp-method", choices=["icp", "gicp"], default="gicp", help="精修方法")
    parser.add_argument("--icp-refine-top-k", type=int, default=8, help="精修前几个候选")
    parser.add_argument("--icp-map-crop-radius-m", type=float, default=18.0, help="局部地图裁剪半径")
    parser.add_argument("--icp-min-map-points", type=int, default=500, help="ICP 最少地图点")
    parser.add_argument("--icp-min-scan-points", type=int, default=200, help="ICP 最少 scan 点")
    parser.add_argument("--icp-scan-voxel-size", type=float, default=0.25, help="ICP scan 体素")
    parser.add_argument("--icp-map-voxel-size", type=float, default=0.25, help="ICP map 体素")
    parser.add_argument("--icp-max-correspondence-m", type=float, default=0.8, help="ICP 最大对应距离")
    parser.add_argument("--icp-iterations", type=int, default=25, help="ICP 迭代次数")
    parser.add_argument("--icp-fitness-weight", type=float, default=3.0, help="ICP fitness 加分权重")
    parser.add_argument("--icp-rmse-weight", type=float, default=1.0, help="ICP RMSE 惩罚权重")

    if not parser.parse_known_args()[0].metrics_glob:
        parser.set_defaults(metrics_glob=[str(root / ".codex_tmp/bbs2d_integrated_random100/rand100_seed20260707/global_relocalization_metrics.csv")])
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
