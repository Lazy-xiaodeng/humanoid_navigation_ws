#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_descriptor_fusion_validation.py

文件作用：
  1. 验证“Scan Context keyframe 召回 + 局部 yaw 搜索 + GICP 精配准”的静止冷启动融合效果。
  2. 复用 run_scan_context_keyframe_recall.py 中的真实 keyframe scan 描述子库，先召回 top-N 候选位置。
  3. 对每个候选位置在若干 yaw 偏移上构造初值，用 Open3D Generalized ICP 对当前静止 scan 和全局地图做精配准。
  4. 输出每个点位最终误差、fitness、inlier_rmse、候选 margin，作为后续移植进 C++ evaluator 前的离线证据。

重要说明：
  - 该脚本仍是验证工具，不发布 TF，不注入 initialpose，也不改变线上节点。
  - 描述子候选只作为 seed 来源，最终是否成功仍由地图配准误差和真值评估判断。
  - 当前使用 bag 中真实 keyframe scan 建库，用来模拟“建图/巡航阶段离线生成的 keyframe DB”。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_descriptor_fusion_validation.py \
    --matches .codex_tmp/waypoint_pose_validation_bag46_25pts_single_fallback/dynamic_waypoints_matches.csv \
    --ids 8 \
    --output .codex_tmp/descriptor_fusion_wp8.csv
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import open3d as o3d

from run_scan_context_keyframe_recall import (
    DescriptorEntry,
    Target,
    build_descriptor_database,
    descriptor_distance,
    nearest_odom,
    read_bag_samples,
    read_targets,
    registered_world_cloud_to_body,
    repo_root_from_script,
    scan_context_descriptor,
)


def normalize_angle_deg(angle: float) -> float:
    """把角度规整到 [-180, 180]。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def pose_matrix(x: float, y: float, yaw_deg: float) -> np.ndarray:
    """根据 x/y/yaw 构造 map->base 初值矩阵。"""

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
    """从 4x4 位姿矩阵读取 yaw，单位 deg。"""

    return math.degrees(math.atan2(float(matrix[1, 0]), float(matrix[0, 0])))


def crop_scan_points(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """按现有 evaluator 的 scan 规则做近似裁剪，减少 GICP 负担。"""

    radius = np.linalg.norm(points[:, :3], axis=1)
    valid = (
        np.isfinite(points).all(axis=1) &
        (radius >= args.min_range) &
        (radius <= args.max_radius) &
        (points[:, 2] >= args.min_z) &
        (points[:, 2] <= args.max_z)
    )
    return points[valid]


def make_o3d_cloud(points: np.ndarray, voxel_size: float) -> o3d.geometry.PointCloud:
    """把 numpy 点集转成 Open3D 点云并降采样。"""

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size)
    return cloud


def load_map_cloud(map_path: Path, args: argparse.Namespace) -> o3d.geometry.PointCloud:
    """读取全局地图并做与当前 C++ evaluator 接近的裁剪/降采样。"""

    cloud = o3d.io.read_point_cloud(str(map_path))
    if args.map_voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(args.map_voxel_size)
    points = np.asarray(cloud.points)
    valid = (
        np.isfinite(points).all(axis=1) &
        (points[:, 2] >= args.min_z) &
        (points[:, 2] <= args.max_z)
    )
    return make_o3d_cloud(points[valid], 0.0)


def query_descriptor_candidates(
    target: Target,
    target_descriptor: np.ndarray,
    database: list[DescriptorEntry],
    args: argparse.Namespace,
) -> list[tuple[float, int, DescriptorEntry]]:
    """检索一个目标的 top-N 描述子候选。"""

    scored: list[tuple[float, int, DescriptorEntry]] = []
    for entry in database:
        if abs(entry.cloud_index - target.cloud_index) <= args.exclude_index_radius:
            continue
        distance, shift = descriptor_distance(target_descriptor, entry.descriptor)
        scored.append((distance, shift, entry))
    scored.sort(key=lambda item: item[0])
    return scored[:max(1, args.descriptor_top_k)]


def run_gicp(
    scan_cloud: o3d.geometry.PointCloud,
    map_cloud: o3d.geometry.PointCloud,
    init: np.ndarray,
    args: argparse.Namespace,
) -> o3d.pipelines.registration.RegistrationResult:
    """执行一次 Open3D Generalized ICP。"""

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
    target: Target,
    target_cloud,
    odoms,
    database: list[DescriptorEntry],
    map_cloud: o3d.geometry.PointCloud,
    args: argparse.Namespace,
) -> tuple[dict[str, str], list[dict[str, str]]]:
    """对一个点位执行描述子召回融合验证。"""

    odom = nearest_odom(odoms, target_cloud.stamp_sec)
    points = registered_world_cloud_to_body(target_cloud, odom)
    descriptor = scan_context_descriptor(
        points,
        args.rings,
        args.sectors,
        args.max_radius,
        args.min_range,
        args.min_z,
        args.max_z,
    )
    scan_points = crop_scan_points(points, args)
    scan_cloud = make_o3d_cloud(scan_points, args.scan_voxel_size)
    candidates = query_descriptor_candidates(target, descriptor, database, args)

    refined: list[dict[str, float | int]] = []
    candidate_rows: list[dict[str, str]] = []
    for descriptor_rank, (descriptor_distance_value, shift, entry) in enumerate(candidates, start=1):
        for yaw_offset in args.yaw_offsets:
            init = pose_matrix(entry.reference_x, entry.reference_y, entry.reference_yaw_deg + yaw_offset)
            result = run_gicp(scan_cloud, map_cloud, init, args)
            pose = result.transformation
            xy_error = math.hypot(float(pose[0, 3]) - target.reference_x, float(pose[1, 3]) - target.reference_y)
            yaw_error = abs(normalize_angle_deg(yaw_from_matrix(pose) - target.reference_yaw_deg))
            refined.append(
                {
                    "descriptor_rank": descriptor_rank,
                    "descriptor_distance": float(descriptor_distance_value),
                    "descriptor_shift": int(shift),
                    "keyframe_index": entry.cloud_index,
                    "yaw_offset_deg": float(yaw_offset),
                    "fitness": float(result.fitness),
                    "inlier_rmse": float(result.inlier_rmse),
                    "x": float(pose[0, 3]),
                    "y": float(pose[1, 3]),
                    "yaw_deg": yaw_from_matrix(pose),
                    "xy_error": xy_error,
                    "yaw_error": yaw_error,
                }
            )
            candidate_rows.append(
                {
                    "waypoint_id": target.waypoint_id,
                    "target_index": str(target.cloud_index),
                    "descriptor_rank": str(descriptor_rank),
                    "descriptor_distance": f"{float(descriptor_distance_value):.6f}",
                    "descriptor_shift": str(shift),
                    "keyframe_index": str(entry.cloud_index),
                    "keyframe_x": f"{entry.reference_x:.6f}",
                    "keyframe_y": f"{entry.reference_y:.6f}",
                    "keyframe_yaw_deg": f"{entry.reference_yaw_deg:.6f}",
                    "yaw_offset_deg": f"{float(yaw_offset):.3f}",
                    "fitness": f"{float(result.fitness):.6f}",
                    "inlier_rmse": f"{float(result.inlier_rmse):.6f}",
                    "final_x_m": f"{float(pose[0, 3]):.6f}",
                    "final_y_m": f"{float(pose[1, 3]):.6f}",
                    "final_yaw_deg": f"{yaw_from_matrix(pose):.6f}",
                    "translation_error_m": f"{xy_error:.6f}",
                    "yaw_error_deg": f"{yaw_error:.6f}",
                    "success": "1" if xy_error <= args.success_translation_thresh and yaw_error <= args.success_yaw_thresh_deg else "0",
                }
            )

    def seed_drift(item: dict[str, float | int]) -> tuple[float, float]:
        """计算 GICP 结果相对 descriptor seed 的漂移量。"""

        keyframe = next(
            entry for entry in database
            if entry.cloud_index == int(item["keyframe_index"])
        )
        xy = math.hypot(float(item["x"]) - keyframe.reference_x, float(item["y"]) - keyframe.reference_y)
        seed_yaw = keyframe.reference_yaw_deg + float(item["yaw_offset_deg"])
        yaw = abs(normalize_angle_deg(float(item["yaw_deg"]) - seed_yaw))
        return xy, yaw

    # GICP 在重复走廊里可能被吸到远处的低 rmse 错误结构，因此先用 seed drift gate 限制它只能小范围修正。
    # 在 gate 内再优先信 descriptor 排名/距离，rmse 只作为同类候选的辅助排序。
    gated: list[dict[str, float | int]] = []
    for item in refined:
        drift_xy, drift_yaw = seed_drift(item)
        item["seed_drift_xy"] = drift_xy
        item["seed_drift_yaw"] = drift_yaw
        if drift_xy <= args.max_seed_drift_m and drift_yaw <= args.max_seed_drift_yaw_deg:
            gated.append(item)
    candidate_pool = gated if gated else refined
    def fusion_score(item: dict[str, float | int]) -> float:
        """融合 descriptor 召回质量和 GICP 几何质量的排序分数，越小越好。"""

        return (
            float(item["inlier_rmse"]) +
            args.descriptor_distance_weight * float(item["descriptor_distance"]) +
            args.descriptor_rank_weight * float(item["descriptor_rank"]) -
            args.fitness_weight * float(item["fitness"])
        )

    candidate_pool.sort(key=lambda item: (fusion_score(item), float(item["inlier_rmse"])))
    best = candidate_pool[0]
    rmse_sorted = sorted(candidate_pool, key=lambda item: float(item["inlier_rmse"]))
    second_rmse = float(rmse_sorted[1]["inlier_rmse"]) if len(rmse_sorted) > 1 else float("inf")
    rmse_margin = second_rmse - float(best["inlier_rmse"])
    success = float(best["xy_error"]) <= args.success_translation_thresh and float(best["yaw_error"]) <= args.success_yaw_thresh_deg

    summary = {
        "waypoint_id": target.waypoint_id,
        "name": target.name,
        "target_index": str(target.cloud_index),
        "descriptor_top_k": str(args.descriptor_top_k),
        "yaw_offsets": ";".join(str(v) for v in args.yaw_offsets),
        "scan_points": str(len(scan_cloud.points)),
        "candidate_evaluations": str(len(refined)),
        "gated_candidate_evaluations": str(len(gated)),
        "selected_descriptor_rank": str(int(best["descriptor_rank"])),
        "selected_keyframe_index": str(int(best["keyframe_index"])),
        "selected_yaw_offset_deg": f"{float(best['yaw_offset_deg']):.3f}",
        "selected_descriptor_distance": f"{float(best['descriptor_distance']):.6f}",
        "fitness": f"{float(best['fitness']):.6f}",
        "inlier_rmse": f"{float(best['inlier_rmse']):.6f}",
        "rmse_margin": f"{rmse_margin:.6f}",
        "seed_drift_xy_m": f"{float(best.get('seed_drift_xy', 0.0)):.6f}",
        "seed_drift_yaw_deg": f"{float(best.get('seed_drift_yaw', 0.0)):.6f}",
        "fusion_score": f"{fusion_score(best):.6f}",
        "final_x_m": f"{float(best['x']):.6f}",
        "final_y_m": f"{float(best['y']):.6f}",
        "final_yaw_deg": f"{float(best['yaw_deg']):.6f}",
        "translation_error_m": f"{float(best['xy_error']):.6f}",
        "yaw_error_deg": f"{float(best['yaw_error']):.6f}",
        "success": "1" if success else "0",
    }
    return summary, candidate_rows


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """写出融合验证 CSV。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate descriptor recall fused with local GICP.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--bag", type=Path, default=Path("/home/ubuntu/nav_drift_test/nav_drift_test46"), help="验证 bag")
    parser.add_argument("--matches", type=Path, required=True, help="点位匹配 CSV")
    parser.add_argument("--ids", nargs="*", default=[], help="只验证指定 waypoint_id；不传则验证全部")
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"), help="全局地图 PCD")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/descriptor_fusion_validation.csv"), help="输出 CSV")
    parser.add_argument("--candidate-output", type=Path, default=None, help="可选：输出每个 descriptor/yaw seed 的 GICP 明细 CSV")
    parser.add_argument("--keyframe-stride", type=int, default=40, help="描述子库抽样步长")
    parser.add_argument("--exclude-index-radius", type=int, default=240, help="排除目标前后 cloud index 范围，避免泄漏")
    parser.add_argument("--descriptor-top-k", type=int, default=20, help="参与 GICP 的描述子候选数量")
    parser.add_argument("--yaw-offsets", nargs="*", type=float, default=[-30, -20, -10, 0, 10, 20, 30], help="每个 keyframe seed 额外尝试的 yaw 偏移")
    parser.add_argument("--rings", type=int, default=20, help="Scan Context 环数")
    parser.add_argument("--sectors", type=int, default=60, help="Scan Context 扇区数")
    parser.add_argument("--max-radius", type=float, default=20.0, help="描述子和 scan 裁剪最大半径")
    parser.add_argument("--min-range", type=float, default=0.6, help="忽略机器人近场点")
    parser.add_argument("--min-z", type=float, default=0.2, help="最低高度")
    parser.add_argument("--max-z", type=float, default=2.5, help="最高高度")
    parser.add_argument("--map-voxel-size", type=float, default=0.30, help="地图降采样体素")
    parser.add_argument("--scan-voxel-size", type=float, default=0.30, help="scan 降采样体素")
    parser.add_argument("--max-correspondence-distance", type=float, default=1.5, help="GICP 最大对应距离")
    parser.add_argument("--gicp-iterations", type=int, default=30, help="GICP 最大迭代次数")
    parser.add_argument("--max-seed-drift-m", type=float, default=1.5, help="GICP 结果相对 descriptor seed 允许的最大平移漂移")
    parser.add_argument("--max-seed-drift-yaw-deg", type=float, default=35.0, help="GICP 结果相对 descriptor seed 允许的最大 yaw 漂移")
    parser.add_argument("--descriptor-distance-weight", type=float, default=0.5, help="融合排序中 descriptor distance 的权重")
    parser.add_argument("--descriptor-rank-weight", type=float, default=0.03, help="融合排序中 descriptor rank 的权重")
    parser.add_argument("--fitness-weight", type=float, default=0.0, help="融合排序中 Open3D fitness 的奖励权重")
    parser.add_argument("--success-translation-thresh", type=float, default=0.80, help="成功平移阈值")
    parser.add_argument("--success-yaw-thresh-deg", type=float, default=15.0, help="成功 yaw 阈值")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    bag = args.bag if args.bag.is_absolute() else workspace / args.bag
    matches = args.matches if args.matches.is_absolute() else workspace / args.matches
    map_path = args.map if args.map.is_absolute() else workspace / args.map
    output = args.output if args.output.is_absolute() else workspace / args.output

    targets = read_targets(matches, set(args.ids))
    if not targets:
        print("[descriptor_fusion] no targets selected")
        return 2

    odoms, references, target_clouds, keyframe_clouds = read_bag_samples(
        bag,
        targets,
        max(1, args.keyframe_stride),
    )
    database = build_descriptor_database(keyframe_clouds, odoms, references, args)
    map_cloud = load_map_cloud(map_path, args)

    rows: list[dict[str, str]] = []
    candidate_rows: list[dict[str, str]] = []
    for target in targets:
        sample = target_clouds.get(target.cloud_index)
        if sample is None:
            print(f"[descriptor_fusion] missing target cloud index={target.cloud_index}")
            continue
        row, target_candidate_rows = evaluate_target(target, sample, odoms, database, map_cloud, args)
        rows.append(row)
        candidate_rows.extend(target_candidate_rows)
        print(
            "[descriptor_fusion] "
            f"wp={row['waypoint_id']} success={row['success']} "
            f"err={row['translation_error_m']}m/{row['yaw_error_deg']}deg "
            f"rank={row['selected_descriptor_rank']} rmse={row['inlier_rmse']} "
            f"fitness={row['fitness']}"
        )

    write_csv(output, rows)
    if args.candidate_output is not None:
        candidate_output = args.candidate_output if args.candidate_output.is_absolute() else workspace / args.candidate_output
        write_csv(candidate_output, candidate_rows)
        print(f"[descriptor_fusion] wrote candidates {candidate_output}")
    success_count = sum(row["success"] == "1" for row in rows)
    print(f"[descriptor_fusion] success={success_count}/{len(rows)} wrote {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
