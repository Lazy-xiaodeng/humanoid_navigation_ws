#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_spectral_gv_validation.py

文件作用：
  1. 离线验证 SpectralGV 几何一致性重排序是否能提升当前全局重定位候选的准确率。
  2. 读取我们已有的 target/keyframe PCD 和三角描述子 + GICP 候选 CSV，不重新跑 bag，不发布 TF。
  3. 对每个 target-keyframe 唯一组合提取 FPFH 局部特征，使用 SpectralGV 官方核心思想构造空间一致性图并计算谱分数。
  4. 把 SpectralGV 分数回填到每个 yaw/GICP 候选，做无真值权重 sweep，输出严格阈值成功率、资源消耗和候选明细。

重要说明：
  - SpectralGV 官方算法需要“全局检索候选 + 局部特征”。官方 demo 使用已有网络局部特征 pickle；
    当前脚本为了完整跑我们自己的 bag，使用 Open3D FPFH 作为局部特征来源，谱图/幂迭代重排序逻辑保持完整。
  - 真值只用于统计最终误差，不参与 SpectralGV 分数计算或候选排序。
  - 该脚本是验证工具，不改变线上节点，不接入 bridge/nav，不注入 initialpose。
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
import torch


@dataclass(frozen=True)
class ScanFeature:
    """一个 scan 的关键点坐标和局部描述子。"""

    keypoints: np.ndarray
    features: np.ndarray
    raw_points: int
    used_points: int


@dataclass(frozen=True)
class TargetSample:
    """待验证 target scan 的真值和 PCD 路径。"""

    target_id: str
    cloud_index: int
    reference_x: float
    reference_y: float
    reference_yaw_deg: float
    pcd_path: Path


@dataclass(frozen=True)
class KeyframeSample:
    """描述子数据库中的 keyframe scan 及其参考位姿。"""

    cloud_index: int
    reference_x: float
    reference_y: float
    reference_yaw_deg: float
    pcd_path: Path


def repo_root_from_script() -> Path:
    """根据脚本位置推断 humanoid_ws 根目录。"""

    return Path(__file__).resolve().parents[3]


def normalize_angle_deg(angle: float) -> float:
    """把 yaw 误差规整到 [-180, 180]。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def current_rss_mb() -> float:
    """读取当前进程最大 RSS；Linux ru_maxrss 单位为 KB。"""

    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def current_cpu_seconds() -> float:
    """读取当前进程累计 user+system CPU 秒。"""

    usage = resource.getrusage(resource.RUSAGE_SELF)
    return float(usage.ru_utime + usage.ru_stime)


def read_targets(path: Path) -> dict[int, TargetSample]:
    """读取 target CSV，并按 cloud_index 建索引。"""

    targets: dict[int, TargetSample] = {}
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            cloud_index = int(row["cloud_index"])
            targets[cloud_index] = TargetSample(
                target_id=row.get("target_id", f"target_{cloud_index}"),
                cloud_index=cloud_index,
                reference_x=float(row["x"]),
                reference_y=float(row["y"]),
                reference_yaw_deg=float(row["yaw_deg"]),
                pcd_path=Path(row["pcd"]),
            )
    return targets


def read_keyframes(path: Path) -> dict[int, KeyframeSample]:
    """读取 keyframe CSV，并按 cloud_index 建索引。"""

    keyframes: dict[int, KeyframeSample] = {}
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            cloud_index = int(row["cloud_index"])
            keyframes[cloud_index] = KeyframeSample(
                cloud_index=cloud_index,
                reference_x=float(row["x"]),
                reference_y=float(row["y"]),
                reference_yaw_deg=float(row["yaw_deg"]),
                pcd_path=Path(row["pcd"]),
            )
    return keyframes


def read_candidate_rows(path: Path, max_targets: int) -> dict[int, list[dict[str, str]]]:
    """读取候选 CSV，并按 target_index 分组；max_targets 只用于快速 smoke。"""

    grouped: dict[int, list[dict[str, str]]] = {}
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            target_index = int(row["target_index"])
            if target_index not in grouped and 0 < max_targets <= len(grouped):
                continue
            grouped.setdefault(target_index, []).append(row)
    return grouped


def load_scan_points(path: Path, args: argparse.Namespace) -> np.ndarray:
    """读取 PCD 并按距离/高度过滤，得到用于局部特征的 scan 点。"""

    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points, dtype=np.float64)
    radius = np.linalg.norm(points[:, :2], axis=1)
    valid = (
        np.isfinite(points).all(axis=1)
        & (radius >= args.min_range)
        & (radius <= args.max_range)
        & (points[:, 2] >= args.min_z)
        & (points[:, 2] <= args.max_z)
    )
    return points[valid]


def make_o3d_cloud(points: np.ndarray, voxel_size: float) -> o3d.geometry.PointCloud:
    """把 numpy 点集转换为 Open3D 点云，并按需降采样。"""

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(voxel_size)
    return cloud


def deterministic_sample(points: np.ndarray, max_points: int, seed: int) -> np.ndarray:
    """对关键点做确定性抽样，控制谱图 N^2 内存和耗时。"""

    if max_points <= 0 or len(points) <= max_points:
        return points
    rng = np.random.default_rng(seed)
    return points[rng.choice(len(points), max_points, replace=False)]


def extract_scan_feature(path: Path, args: argparse.Namespace, seed: int) -> ScanFeature:
    """提取一个 scan 的 FPFH 局部特征，作为 SpectralGV 的局部特征输入。"""

    raw_points = load_scan_points(path, args)
    cloud = make_o3d_cloud(raw_points, args.feature_voxel_size)
    if len(cloud.points) == 0:
        empty = np.zeros((0, 3), dtype=np.float32)
        return ScanFeature(empty, np.zeros((0, 33), dtype=np.float32), len(raw_points), 0)

    cloud.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=args.normal_radius,
            max_nn=args.normal_max_nn,
        )
    )
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        cloud,
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=args.fpfh_radius,
            max_nn=args.fpfh_max_nn,
        ),
    )
    keypoints = np.asarray(cloud.points, dtype=np.float32)
    features = np.asarray(fpfh.data, dtype=np.float32).T
    if len(keypoints) > args.max_keypoints:
        indexes = deterministic_sample(np.arange(len(keypoints)), args.max_keypoints, seed)
        keypoints = keypoints[indexes]
        features = features[indexes]
    return ScanFeature(keypoints, features, len(raw_points), len(keypoints))


def torch_device(args: argparse.Namespace) -> torch.device:
    """根据参数和环境选择 CPU/GPU；当前机器人验证机通常是 CPU-only。"""

    if args.device == "cuda" and torch.cuda.is_available():
        return torch.device("cuda")
    if args.device == "auto" and torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def spectral_gv_score(query: ScanFeature, candidate: ScanFeature, args: argparse.Namespace, device: torch.device) -> float:
    """执行 SpectralGV 空间一致性评分，返回越大越可信的谱分数。"""

    count = min(len(query.keypoints), len(candidate.keypoints), args.max_sgv_points)
    if count < args.min_sgv_points:
        return 0.0

    q_keypoints = query.keypoints[:count]
    q_features = query.features[:count]
    c_keypoints = candidate.keypoints[:count]
    c_features = candidate.features[:count]

    src_keypts = torch.as_tensor(q_keypoints, dtype=torch.float32, device=device).unsqueeze(0)
    tgt_keypts = torch.as_tensor(c_keypoints, dtype=torch.float32, device=device).unsqueeze(0)
    src_features = torch.nn.functional.normalize(
        torch.as_tensor(q_features, dtype=torch.float32, device=device).unsqueeze(0),
        p=2.0,
        dim=2,
    )
    tgt_features = torch.nn.functional.normalize(
        torch.as_tensor(c_features, dtype=torch.float32, device=device).unsqueeze(0),
        p=2.0,
        dim=2,
    )

    distance = torch.cdist(src_features, tgt_features)
    min_ids = torch.argmin(distance, dim=2)
    min_ids = min_ids.unsqueeze(-1).expand(-1, -1, 3)
    tgt_corr = torch.gather(tgt_keypts, 1, min_ids)
    src_corr = src_keypts

    src_dist = torch.norm(src_corr[:, :, None, :] - src_corr[:, None, :, :], dim=-1)
    tgt_dist = torch.norm(tgt_corr[:, :, None, :] - tgt_corr[:, None, :, :], dim=-1)
    cross_dist = torch.abs(src_dist - tgt_dist)
    adjacency = torch.clamp(1.0 - cross_dist.square() / (args.sgv_distance_threshold ** 2), min=0.0)

    leading = torch.ones_like(adjacency[:, :, 0:1])
    for _ in range(args.power_iterations):
        leading_next = torch.bmm(adjacency, leading)
        leading_next = leading_next / (torch.norm(leading_next, dim=1, keepdim=True) + 1e-6)
        if torch.allclose(leading_next, leading):
            leading = leading_next
            break
        leading = leading_next

    leading_vec = leading.squeeze(-1)
    spatial_consistency = leading_vec[:, None, :] @ adjacency @ leading_vec[:, :, None]
    score = spatial_consistency.squeeze() / float(count)
    return float(score.detach().cpu().item())


def truth_error(row: dict[str, str], target: TargetSample) -> tuple[float, float]:
    """读取候选误差；如果 CSV 没有误差字段则按 target 真值计算。"""

    if "translation_error_m" in row and "yaw_error_deg" in row:
        return float(row["translation_error_m"]), float(row["yaw_error_deg"])
    x = float(row["final_x_m"])
    y = float(row["final_y_m"])
    yaw = float(row["final_yaw_deg"])
    return math.hypot(x - target.reference_x, y - target.reference_y), abs(
        normalize_angle_deg(yaw - target.reference_yaw_deg)
    )


def base_score(row: dict[str, str]) -> float:
    """构造不使用真值的基础候选分数，越小越好。"""

    rmse = float(row.get("inlier_rmse", "1.0"))
    rank = float(row.get("descriptor_rank", "1"))
    similarity = float(row.get("triangle_similarity", "0.0"))
    return rmse + 0.02 * rank - 1.2 * similarity


def success_flags(xy_error: float, yaw_error: float) -> tuple[bool, bool, bool]:
    """返回严格、实用、可救回三档成功口径。"""

    return (
        xy_error <= 0.2 and yaw_error <= 3.0,
        xy_error <= 0.3 and yaw_error <= 5.0,
        xy_error <= 0.5 and yaw_error <= 10.0,
    )


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """把验证结果写成 CSV。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def evaluate(args: argparse.Namespace) -> tuple[list[dict[str, str]], list[dict[str, str]]]:
    """执行 SpectralGV 特征缓存、候选评分和权重 sweep。"""

    workspace = args.workspace.resolve()
    targets_csv = args.targets if args.targets.is_absolute() else workspace / args.targets
    keyframes_csv = args.keyframes if args.keyframes.is_absolute() else workspace / args.keyframes
    candidates_csv = args.candidates if args.candidates.is_absolute() else workspace / args.candidates

    targets = read_targets(targets_csv)
    keyframes = read_keyframes(keyframes_csv)
    grouped = read_candidate_rows(candidates_csv, args.max_targets)
    device = torch_device(args)

    feature_cache: dict[Path, ScanFeature] = {}

    def feature_for(path: Path, seed: int) -> ScanFeature:
        resolved = path.resolve()
        if resolved not in feature_cache:
            feature_cache[resolved] = extract_scan_feature(resolved, args, seed)
        return feature_cache[resolved]

    sgv_cache: dict[tuple[int, int], float] = {}
    detail_rows: list[dict[str, str]] = []
    target_candidate_rows: dict[int, list[dict[str, str]]] = {}

    for target_index, rows in grouped.items():
        if target_index not in targets:
            continue
        target = targets[target_index]
        target_feature = feature_for(target.pcd_path, args.random_seed + target_index)
        enriched: list[dict[str, str]] = []
        for row in rows:
            keyframe_index = int(row["keyframe_index"])
            if keyframe_index not in keyframes:
                continue
            key = (target_index, keyframe_index)
            if key not in sgv_cache:
                keyframe = keyframes[keyframe_index]
                keyframe_feature = feature_for(keyframe.pcd_path, args.random_seed + keyframe_index)
                sgv_cache[key] = spectral_gv_score(target_feature, keyframe_feature, args, device)
            xy_error, yaw_error = truth_error(row, target)
            strict, practical, recoverable = success_flags(xy_error, yaw_error)
            enriched_row = dict(row)
            enriched_row.update(
                {
                    "spectral_gv_score": f"{sgv_cache[key]:.9f}",
                    "spectral_base_score": f"{base_score(row):.9f}",
                    "truth_translation_error_m": f"{xy_error:.6f}",
                    "truth_yaw_error_deg": f"{yaw_error:.6f}",
                    "truth_success_0p2m_3deg": "1" if strict else "0",
                    "truth_success_0p3m_5deg": "1" if practical else "0",
                    "truth_success_0p5m_10deg": "1" if recoverable else "0",
                    "target_feature_points": str(target_feature.used_points),
                }
            )
            enriched.append(enriched_row)
            detail_rows.append(enriched_row)
        target_candidate_rows[target_index] = enriched

    weights = [float(item) for item in args.sgv_weight_sweep.split(",") if item.strip()]
    summary_rows: list[dict[str, str]] = []
    for weight in weights:
        selected_rows: list[dict[str, str]] = []
        for rows in target_candidate_rows.values():
            if not rows:
                continue
            gated_rows = [
                row
                for row in rows
                if float(row.get("seed_drift_xy_m", "0.0")) <= args.max_seed_drift_m
                and float(row.get("seed_drift_yaw_deg", "0.0")) <= args.max_seed_drift_yaw_deg
            ]
            candidate_pool = gated_rows if gated_rows else rows
            selected_rows.append(
                min(
                    candidate_pool,
                    key=lambda row: (
                        float(row["spectral_base_score"]) - weight * float(row["spectral_gv_score"]),
                        float(row.get("inlier_rmse", "1.0")),
                    ),
                )
            )
        strict_count = 0
        practical_count = 0
        recoverable_count = 0
        xy_values: list[float] = []
        yaw_values: list[float] = []
        for row in selected_rows:
            xy_error = float(row["truth_translation_error_m"])
            yaw_error = float(row["truth_yaw_error_deg"])
            strict, practical, recoverable = success_flags(xy_error, yaw_error)
            strict_count += int(strict)
            practical_count += int(practical)
            recoverable_count += int(recoverable)
            xy_values.append(xy_error)
            yaw_values.append(yaw_error)
        summary_rows.append(
            {
                "sgv_weight": f"{weight:.6f}",
                "targets": str(len(selected_rows)),
                "success_0p2m_3deg": str(strict_count),
                "success_0p3m_5deg": str(practical_count),
                "success_0p5m_10deg": str(recoverable_count),
                "median_xy_error_m": f"{float(np.median(xy_values)):.6f}" if xy_values else "nan",
                "median_yaw_error_deg": f"{float(np.median(yaw_values)):.6f}" if yaw_values else "nan",
                "max_xy_error_m": f"{float(np.max(xy_values)):.6f}" if xy_values else "nan",
                "max_yaw_error_deg": f"{float(np.max(yaw_values)):.6f}" if yaw_values else "nan",
                "unique_sgv_pairs": str(len(sgv_cache)),
                "feature_cache_size": str(len(feature_cache)),
            }
        )

    return detail_rows, summary_rows


def parse_args() -> argparse.Namespace:
    """解析命令行参数，默认跑 bag46 hard52 候选。"""

    root = repo_root_from_script()
    parser = argparse.ArgumentParser(description="验证 SpectralGV 对全局重定位候选重排序的作用。")
    parser.add_argument("--workspace", type=Path, default=root, help="humanoid_ws 工作空间路径")
    parser.add_argument("--targets", type=Path, default=Path(".codex_tmp/official_std_standalone/nav46_hard52_dataset_acc10_fullz/targets.csv"), help="target PCD CSV")
    parser.add_argument("--keyframes", type=Path, default=Path(".codex_tmp/spectral_gv/candidate_keyframes.csv"), help="keyframe PCD CSV")
    parser.add_argument("--candidates", type=Path, default=Path(".codex_tmp/triangle_descriptor/hard52_top20_candidates.csv"), help="候选明细 CSV")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/spectral_gv/hard52_candidate_details.csv"), help="候选明细输出")
    parser.add_argument("--summary-output", type=Path, default=Path(".codex_tmp/spectral_gv/hard52_summary.csv"), help="汇总输出")
    parser.add_argument("--max-targets", type=int, default=0, help="只验证前 N 个 target；0 表示全量")
    parser.add_argument("--random-seed", type=int, default=13, help="确定性采样随机种子")
    parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="auto", help="SpectralGV 计算设备")

    parser.add_argument("--feature-voxel-size", type=float, default=0.28, help="FPFH 特征点云体素尺寸")
    parser.add_argument("--max-keypoints", type=int, default=900, help="每个 scan 最多保留的局部特征点数")
    parser.add_argument("--max-sgv-points", type=int, default=512, help="每个候选对进入谱图的最大对应点数")
    parser.add_argument("--min-sgv-points", type=int, default=40, help="少于该点数时认为 SpectralGV 分数无效")
    parser.add_argument("--min-range", type=float, default=0.35, help="忽略过近点")
    parser.add_argument("--max-range", type=float, default=18.0, help="scan 最大使用半径")
    parser.add_argument("--min-z", type=float, default=-1.0, help="scan 最低高度")
    parser.add_argument("--max-z", type=float, default=2.2, help="scan 最高高度")
    parser.add_argument("--normal-radius", type=float, default=0.70, help="法向估计半径")
    parser.add_argument("--normal-max-nn", type=int, default=40, help="法向估计最大邻居数")
    parser.add_argument("--fpfh-radius", type=float, default=1.40, help="FPFH 特征半径")
    parser.add_argument("--fpfh-max-nn", type=int, default=80, help="FPFH 最大邻居数")
    parser.add_argument("--sgv-distance-threshold", type=float, default=0.55, help="SpectralGV 空间一致性距离阈值")
    parser.add_argument("--power-iterations", type=int, default=8, help="谱图主特征向量幂迭代次数")
    parser.add_argument("--max-seed-drift-m", type=float, default=3.0, help="复用当前 hard52 候选产物中的最大 seed 平移漂移门控")
    parser.add_argument("--max-seed-drift-yaw-deg", type=float, default=55.0, help="复用三角描述子验证中的最大 seed yaw 漂移门控")
    parser.add_argument("--sgv-weight-sweep", default="0,0.05,0.1,0.2,0.4,0.8,1.2,1.6,2.0,3.0", help="SpectralGV 分数权重 sweep")
    return parser.parse_args()


def main() -> int:
    """脚本入口：执行验证、写 CSV，并打印最佳权重。"""

    args = parse_args()
    start_time = time.time()
    start_cpu = current_cpu_seconds()
    details, summary = evaluate(args)
    output = args.output if args.output.is_absolute() else args.workspace.resolve() / args.output
    summary_output = args.summary_output if args.summary_output.is_absolute() else args.workspace.resolve() / args.summary_output
    write_csv(output, details)
    write_csv(summary_output, summary)

    elapsed = time.time() - start_time
    cpu_core_equiv = (current_cpu_seconds() - start_cpu) / max(elapsed, 1e-6)
    ranked = sorted(
        summary,
        key=lambda row: (
            -int(row["success_0p3m_5deg"]),
            -int(row["success_0p2m_3deg"]),
            float(row["median_xy_error_m"]) if row["median_xy_error_m"] != "nan" else 999.0,
        ),
    )
    print(f"[spectral_gv] details={output}")
    print(f"[spectral_gv] summary={summary_output}")
    print(f"[spectral_gv] elapsed_sec={elapsed:.3f} cpu_core_equiv={cpu_core_equiv:.2f} rss_mb={current_rss_mb():.1f}")
    for row in ranked[:5]:
        print(
            "[spectral_gv] "
            f"weight={row['sgv_weight']} targets={row['targets']} "
            f"0.2m3deg={row['success_0p2m_3deg']} "
            f"0.3m5deg={row['success_0p3m_5deg']} "
            f"0.5m10deg={row['success_0p5m_10deg']} "
            f"median={row['median_xy_error_m']}m/{row['median_yaw_error_deg']}deg "
            f"pairs={row['unique_sgv_pairs']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
