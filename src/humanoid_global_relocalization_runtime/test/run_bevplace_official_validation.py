#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_bevplace_official_validation.py

文件作用：
  1. 用 BEVPlace++ 官方仓库的完整模型链路验证室内全局重定位效果。
  2. 将我们的 PCD 地图候选位置和 bag 导出的目标 scan 转成 BEVPlace++ 论文/源码使用的 201x201 BEV 图像。
  3. 运行官方 REIN 网络提取全局描述子做地点召回，再用官方 local feature + rigid RANSAC 估计 3DoF 位姿。
  4. 输出每个目标点的最终误差、召回候选、RANSAC 内点数、CPU/内存/耗时，判断该学习式方案是否适合作为 Scan Context 的替代或增强。

重要说明：
  - 该脚本只在离线验证目录使用，不接 ROS topic，不发布 TF，也不修改线上节点。
  - “完整算法”指模型、BEV 生成、REIN、NetVLAD、local feature、RANSAC 均来自 BEVPlace++ 官方实现；本脚本只做数据格式适配和 CPU 兼容包装。
  - 官方 checkpoint 来自上游仓库，若未针对我们的室内地图训练，验证结果只能说明“直接迁移”的可用性，不代表重新训练后的上限。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import resource
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import torch


@dataclass(frozen=True)
class TargetSample:
    """一个待验证目标 scan 与其真实 map 位姿。"""

    target_id: str
    cloud_index: int
    reference_x: float
    reference_y: float
    reference_yaw_deg: float
    pcd_path: Path


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""

    return Path(__file__).resolve().parents[3]


def normalize_angle_deg(angle: float) -> float:
    """把角度规整到 [-180, 180]，用于 yaw 误差统计。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def current_rss_mb() -> float:
    """读取当前进程最大 RSS；Linux ru_maxrss 单位为 KB。"""

    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def read_targets(targets_csv: Path) -> list[TargetSample]:
    """读取 hard52/rand100/edge100 等统一格式的目标 CSV。"""

    targets: list[TargetSample] = []
    with targets_csv.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            targets.append(
                TargetSample(
                    target_id=row.get("target_id", f"target_{len(targets) + 1:03d}"),
                    cloud_index=int(row["cloud_index"]),
                    reference_x=float(row["x"]),
                    reference_y=float(row["y"]),
                    reference_yaw_deg=float(row["yaw_deg"]),
                    pcd_path=Path(row["pcd"]),
                )
            )
    return targets


def limit_targets(targets: list[TargetSample], max_targets: int) -> list[TargetSample]:
    """按需限制目标数量；正式统计默认 max_targets=0 表示全量。"""

    if max_targets <= 0:
        return targets
    return targets[:max_targets]


def load_pcd_points(path: Path) -> np.ndarray:
    """读取 PCD 点云并过滤非有限点。"""

    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points, dtype=np.float32)
    return points[np.isfinite(points).all(axis=1)]


def transform_map_to_candidate_local(points: np.ndarray, x: float, y: float, yaw_deg: float) -> np.ndarray:
    """把 map 点云变换到某个候选位姿的局部坐标系，作为 BEVPlace++ 数据库图像。"""

    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    shifted = points.copy()
    shifted[:, 0] -= float(x)
    shifted[:, 1] -= float(y)
    local = shifted.copy()
    local[:, 0] = c * shifted[:, 0] + s * shifted[:, 1]
    local[:, 1] = -s * shifted[:, 0] + c * shifted[:, 1]
    return local


def filter_bev_window(points: np.ndarray, max_radius: float) -> np.ndarray:
    """按官方 BEVPlace++ 的 80m x 80m 视野裁剪点云。"""

    valid = (
        (np.abs(points[:, 0]) < max_radius)
        & (np.abs(points[:, 1]) < max_radius)
        & (np.abs(points[:, 2]) < max_radius)
    )
    return points[valid].astype(np.float32)


def official_get_bev_compatible(all_points: np.ndarray) -> tuple[np.ndarray, int, int]:
    """按 BEVPlace++ 官方 getBEV() 的坐标映射生成 BEV，并用 NumPy 计数加速离线验证。"""

    # 官方实现先做 0.4m voxel_down_sample，再按 0.4m 像素累计每格最多 10 次命中。
    # 对验证而言，最终输入只关心 201x201 像素强度；这里直接栅格计数并裁剪到 10，避免每个候选重复 Open3D 降采样。
    x_min = -40
    y_min = -40
    x_max = 40
    y_max = 40
    x_min_ind = np.floor(x_min / 0.4).astype(int)
    x_max_ind = np.floor(x_max / 0.4).astype(int)
    y_min_ind = np.floor(y_min / 0.4).astype(int)
    y_max_ind = np.floor(y_max / 0.4).astype(int)
    x_num = x_max_ind - x_min_ind + 1
    y_num = y_max_ind - y_min_ind + 1
    x_ind = x_max_ind - np.floor(all_points[:, 1] / 0.4).astype(np.int32)
    y_ind = y_max_ind - np.floor(all_points[:, 0] / 0.4).astype(np.int32)
    valid = (x_ind >= 0) & (y_ind >= 0) & (x_ind < x_num) & (y_ind < y_num)
    linear = y_ind[valid] * x_num + x_ind[valid]
    counts = np.bincount(linear, minlength=x_num * y_num).reshape(y_num, x_num)
    mat_global_image = np.minimum(counts, 10).astype(np.float32) * 10.0
    max_pixel = float(np.max(mat_global_image))
    if max_pixel > 0.0:
        mat_global_image = mat_global_image / max_pixel * 255.0
    return mat_global_image, x_max_ind, y_max_ind


def make_bev_image(points: np.ndarray, official_get_bev, max_radius: float) -> np.ndarray:
    """调用官方 getBEV() 生成单通道 BEV 图像，并修正空图像的数值异常。"""

    cropped = filter_bev_window(points, max_radius)
    if len(cropped) == 0:
        return np.zeros((201, 201), dtype=np.uint8)
    image, _, _ = official_get_bev(cropped)
    image = np.nan_to_num(image, nan=0.0, posinf=255.0, neginf=0.0)
    return np.clip(image, 0, 255).astype(np.uint8)


def pose_2d_matrix(x: float, y: float, yaw_deg: float) -> np.ndarray:
    """构造 2D 齐次位姿矩阵。"""

    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]], dtype=np.float64)


def yaw_from_pose_2d(matrix: np.ndarray) -> float:
    """从 2D 齐次位姿矩阵中读取 yaw，单位 deg。"""

    return math.degrees(math.atan2(float(matrix[1, 0]), float(matrix[0, 0])))


def patch_torch_cuda_for_cpu() -> None:
    """让官方源码里写死的 .cuda() 在 CPU-only 机器上退化成 no-op。"""

    torch.Tensor.cuda = lambda self, *args, **kwargs: self  # type: ignore[assignment]
    torch.nn.Module.cuda = lambda self, *args, **kwargs: self  # type: ignore[assignment]


def load_official_modules(upstream_root: Path):
    """加载 BEVPlace++ 官方 REIN、RANSAC 和 BEV 生成函数。"""

    sys.path.insert(0, str(upstream_root))
    patch_torch_cuda_for_cpu()
    from REIN import REIN  # pylint: disable=import-error,import-outside-toplevel
    from RANSAC import rigidRansac  # pylint: disable=import-error,import-outside-toplevel

    return REIN, rigidRansac, official_get_bev_compatible


def load_official_model(upstream_root: Path, checkpoint: Path):
    """加载 BEVPlace++ 官方 REIN 模型和 checkpoint。"""

    REIN, rigid_ransac, official_get_bev = load_official_modules(upstream_root)
    model = REIN().eval()
    state = torch.load(checkpoint, map_location="cpu", weights_only=False)
    model.load_state_dict(state["state_dict"], strict=True)
    model.eval()
    return model, rigid_ransac, official_get_bev


def images_to_tensor(images: np.ndarray) -> torch.Tensor:
    """把 N,H,W uint8 图像转成官方 InferDataset 相同的 N,3,H,W float 输入。"""

    tensor = torch.from_numpy(images.astype(np.float32) / 256.0)
    tensor = tensor[:, None, :, :].repeat(1, 3, 1, 1)
    return tensor


def infer_global_descriptors(model, images: np.ndarray, batch_size: int) -> np.ndarray:
    """批量提取 BEVPlace++ 全局描述子，用于地点召回。"""

    descs: list[np.ndarray] = []
    with torch.no_grad():
        for start in range(0, len(images), batch_size):
            batch = images_to_tensor(images[start : start + batch_size])
            _, _, global_desc = model(batch)
            descs.append(global_desc.detach().cpu().numpy().astype(np.float32))
    return np.concatenate(descs, axis=0)


def infer_local_features(model, images: np.ndarray) -> np.ndarray:
    """提取少量 query/topK 图像的 dense local feature，用于官方 RANSAC 位姿估计。"""

    with torch.no_grad():
        batch = images_to_tensor(images)
        _, local_feat, _ = model(batch)
    return local_feat.detach().cpu().numpy().transpose(0, 2, 3, 1)


def estimate_relative_pose(query_image: np.ndarray, db_image: np.ndarray, query_feat: np.ndarray, db_feat: np.ndarray, rigid_ransac):
    """复用官方 evaluateResults() 的 FAST+local feature+rigidRansac 相对位姿估计。"""

    fast = cv2.FastFeatureDetector_create()
    query_rgb = np.repeat(query_image[:, :, None], 3, axis=2)
    db_rgb = np.repeat(db_image[:, :, None], 3, axis=2)
    query_kps = fast.detect(query_rgb, None)
    db_kps = fast.detect(db_rgb, None)
    if len(query_kps) < 3 or len(db_kps) < 3:
        return None, 0

    query_des = np.asarray([query_feat[int(kp.pt[1]), int(kp.pt[0])] for kp in query_kps], dtype=np.float32)
    db_des = np.asarray([db_feat[int(kp.pt[1]), int(kp.pt[0])] for kp in db_kps], dtype=np.float32)
    matcher = cv2.BFMatcher()
    matches = matcher.knnMatch(query_des, db_des, k=2)
    if len(matches) < 3:
        return None, 0

    all_match = [m[0] for m in matches if len(m) > 0]
    if len(all_match) < 3:
        return None, 0
    points1 = np.float32([query_kps[m.queryIdx].pt for m in all_match])
    points2 = np.float32([db_kps[m.trainIdx].pt for m in all_match])
    image_side = query_image.shape[0]
    relative, mask, consensus = rigid_ransac(
        (np.array([[image_side // 2, image_side // 2]]) - points1) * 0.4,
        (np.array([[image_side // 2, image_side // 2]]) - points2) * 0.4,
    )
    if relative is None or np.asarray(relative).shape != (2, 3):
        return None, int(consensus)
    return np.vstack((relative, np.array([[0.0, 0.0, 1.0]]))), int(consensus)


def build_database_images(args: argparse.Namespace, map_points: np.ndarray, candidate_xy: np.ndarray, official_get_bev) -> np.ndarray:
    """为所有地图候选位置生成或读取 BEVPlace++ 数据库 BEV 图像。"""

    if args.db_image_cache.exists() and not args.rebuild_cache:
        return np.load(args.db_image_cache)["images"]

    images = np.zeros((len(candidate_xy), 201, 201), dtype=np.uint8)
    for index, (x, y) in enumerate(candidate_xy):
        # 数据库候选使用 map 坐标 yaw=0 的 BEV 图像，局部窗口裁剪与官方 80m x 80m 视野等价。
        # 先裁剪再变换可以避免每个候选重复处理完整地图点云，算法输入保持不变。
        window = (
            (np.abs(map_points[:, 0] - float(x)) < args.bev_max_radius)
            & (np.abs(map_points[:, 1] - float(y)) < args.bev_max_radius)
            & (np.abs(map_points[:, 2]) < args.bev_max_radius)
        )
        local_points = transform_map_to_candidate_local(map_points[window], float(x), float(y), 0.0)
        images[index] = make_bev_image(local_points, official_get_bev, args.bev_max_radius)
        if index % max(1, len(candidate_xy) // 10) == 0:
            print(f"[bevplace] build db image {index}/{len(candidate_xy)}", flush=True)
    args.db_image_cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.db_image_cache, images=images, candidate_xy=candidate_xy)
    return images


def load_or_build_db_descriptors(args: argparse.Namespace, model, db_images: np.ndarray) -> np.ndarray:
    """为数据库 BEV 图像提取或读取全局描述子。"""

    if args.db_descriptor_cache.exists() and not args.rebuild_cache:
        return np.load(args.db_descriptor_cache)["descriptors"]
    descriptors = infer_global_descriptors(model, db_images, args.batch_size)
    args.db_descriptor_cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.db_descriptor_cache, descriptors=descriptors)
    return descriptors


def evaluate_target(
    target: TargetSample,
    args: argparse.Namespace,
    model,
    rigid_ransac,
    official_get_bev,
    db_images: np.ndarray,
    db_descriptors: np.ndarray,
    candidate_xy: np.ndarray,
) -> dict[str, str]:
    """对单个目标 scan 做官方 BEVPlace++ 召回和 RANSAC 位姿估计。"""

    query_points = load_pcd_points(target.pcd_path)
    query_image = make_bev_image(query_points, official_get_bev, args.bev_max_radius)
    query_descriptor = infer_global_descriptors(model, query_image[None, :, :], args.batch_size)[0]
    distances = np.sum((db_descriptors - query_descriptor[None, :]) ** 2, axis=1)
    top_indices = np.argsort(distances)[: args.top_k]

    # 对 query + topK 数据库图像一次性提取 local feature，保留官方局部匹配/RANSAC 链路。
    local_images = np.concatenate((query_image[None, :, :], db_images[top_indices]), axis=0)
    local_features = infer_local_features(model, local_images)
    query_feat = local_features[0]

    best: dict[str, float | int] | None = None
    for rank, db_index in enumerate(top_indices, start=1):
        relative, consensus = estimate_relative_pose(query_image, db_images[db_index], query_feat, local_features[rank], rigid_ransac)
        if relative is None:
            continue
        db_pose = pose_2d_matrix(float(candidate_xy[db_index, 0]), float(candidate_xy[db_index, 1]), 0.0)
        predicted = db_pose @ relative
        pred_x = float(predicted[0, 2])
        pred_y = float(predicted[1, 2])
        pred_yaw = normalize_angle_deg(yaw_from_pose_2d(predicted))
        trans_error = math.hypot(pred_x - target.reference_x, pred_y - target.reference_y)
        yaw_error = abs(normalize_angle_deg(pred_yaw - target.reference_yaw_deg))
        candidate = {
            "rank": rank,
            "db_index": int(db_index),
            "global_distance": float(distances[db_index]),
            "consensus": int(consensus),
            "pred_x": pred_x,
            "pred_y": pred_y,
            "pred_yaw_deg": pred_yaw,
            "trans_error_m": trans_error,
            "yaw_error_deg": yaw_error,
        }
        if best is None:
            best = candidate
            continue
        # 官方局部位姿估计更依赖 RANSAC 共识数量；共识相同再比较全局描述子距离。
        if int(candidate["consensus"]) > int(best["consensus"]) or (
            int(candidate["consensus"]) == int(best["consensus"])
            and float(candidate["global_distance"]) < float(best["global_distance"])
        ):
            best = candidate

    if best is None:
        best = {
            "rank": -1,
            "db_index": -1,
            "global_distance": float(distances[top_indices[0]]),
            "consensus": 0,
            "pred_x": float("nan"),
            "pred_y": float("nan"),
            "pred_yaw_deg": float("nan"),
            "trans_error_m": float("inf"),
            "yaw_error_deg": 180.0,
        }

    return {
        "target_id": target.target_id,
        "cloud_index": str(target.cloud_index),
        "reference_x": f"{target.reference_x:.6f}",
        "reference_y": f"{target.reference_y:.6f}",
        "reference_yaw_deg": f"{target.reference_yaw_deg:.6f}",
        "rank": str(int(best["rank"])),
        "db_index": str(int(best["db_index"])),
        "global_distance": f"{float(best['global_distance']):.9f}",
        "ransac_consensus": str(int(best["consensus"])),
        "pred_x": f"{float(best['pred_x']):.6f}",
        "pred_y": f"{float(best['pred_y']):.6f}",
        "pred_yaw_deg": f"{float(best['pred_yaw_deg']):.6f}",
        "trans_error_m": f"{float(best['trans_error_m']):.6f}",
        "yaw_error_deg": f"{float(best['yaw_error_deg']):.6f}",
        "success_0p2_3deg": "1" if float(best["trans_error_m"]) <= 0.2 and float(best["yaw_error_deg"]) <= 3.0 else "0",
        "success_0p3_5deg": "1" if float(best["trans_error_m"]) <= 0.3 and float(best["yaw_error_deg"]) <= 5.0 else "0",
        "success_0p5_10deg": "1" if float(best["trans_error_m"]) <= 0.5 and float(best["yaw_error_deg"]) <= 10.0 else "0",
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    """写出验证结果 CSV。"""

    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    """解析离线验证参数。"""

    root = repo_root_from_script()
    parser = argparse.ArgumentParser(description="BEVPlace++ 官方完整链路离线验证")
    parser.add_argument("--targets-csv", type=Path, required=True, help="目标 scan CSV，包含 pcd/x/y/yaw_deg")
    parser.add_argument("--output", type=Path, required=True, help="输出结果 CSV")
    parser.add_argument("--upstream-root", type=Path, default=root / ".codex_tmp/upstream_virtual_bev/BEVPlace2", help="BEVPlace++ 官方仓库目录")
    parser.add_argument("--checkpoint", type=Path, default=root / ".codex_tmp/upstream_virtual_bev/BEVPlace2/runs/Aug08_10-17-29/model_best.pth.tar", help="官方 checkpoint")
    parser.add_argument("--map-path", type=Path, default=root / "src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd", help="全局 PCD 地图")
    parser.add_argument("--candidate-db", type=Path, default=root / ".codex_tmp/virtual_lidar/virtual_db_grid1p0_sec120.npz", help="候选 XY 数据库")
    parser.add_argument("--db-image-cache", type=Path, default=root / ".codex_tmp/bevplace_validation/db_images_grid1p0.npz", help="数据库 BEV 图像缓存")
    parser.add_argument("--db-descriptor-cache", type=Path, default=root / ".codex_tmp/bevplace_validation/db_desc_grid1p0.npz", help="数据库全局描述子缓存")
    parser.add_argument("--rebuild-cache", action="store_true", help="重新生成 BEV 图像和描述子缓存")
    parser.add_argument("--bev-max-radius", type=float, default=40.0, help="BEVPlace++ 官方视野半径，单位 m")
    parser.add_argument("--top-k", type=int, default=10, help="全局描述子召回后进入 local feature RANSAC 的候选数量")
    parser.add_argument("--batch-size", type=int, default=4, help="CPU 推理 batch size；越大越快但更占内存")
    parser.add_argument("--max-targets", type=int, default=0, help="只验证前 N 个目标；0 表示全量")
    return parser.parse_args()


def main() -> int:
    """主流程：加载官方模型、构建数据库、逐目标验证并统计成功率。"""

    args = parse_args()
    os.environ.setdefault("CUDA_VISIBLE_DEVICES", "")
    start = time.perf_counter()
    targets = limit_targets(read_targets(args.targets_csv), args.max_targets)
    model, rigid_ransac, official_get_bev = load_official_model(args.upstream_root, args.checkpoint)
    map_points = load_pcd_points(args.map_path)
    candidate_data = np.load(args.candidate_db)
    candidate_xy = candidate_data["candidates_xy"].astype(np.float32)
    db_images = build_database_images(args, map_points, candidate_xy, official_get_bev)
    db_descriptors = load_or_build_db_descriptors(args, model, db_images)
    print(
        f"[bevplace] targets={len(targets)} db_candidates={len(candidate_xy)} "
        f"rss_mb={current_rss_mb():.1f}",
        flush=True,
    )

    rows: list[dict[str, str]] = []
    for index, target in enumerate(targets, start=1):
        row = evaluate_target(target, args, model, rigid_ransac, official_get_bev, db_images, db_descriptors, candidate_xy)
        rows.append(row)
        print(
            f"[bevplace] {index}/{len(targets)} idx={target.cloud_index} "
            f"err={row['trans_error_m']}m/{row['yaw_error_deg']}deg "
            f"rank={row['rank']} consensus={row['ransac_consensus']} success03={row['success_0p3_5deg']}",
            flush=True,
        )

    write_csv(args.output, rows)
    n = max(1, len(rows))
    success02 = sum(int(row["success_0p2_3deg"]) for row in rows)
    success03 = sum(int(row["success_0p3_5deg"]) for row in rows)
    success05 = sum(int(row["success_0p5_10deg"]) for row in rows)
    print(f"[bevplace] success_0p2_3deg={success02}/{n} ({success02 / n * 100.0:.1f}%)", flush=True)
    print(f"[bevplace] success_0p3_5deg={success03}/{n} ({success03 / n * 100.0:.1f}%)", flush=True)
    print(f"[bevplace] success_0p5_10deg={success05}/{n} ({success05 / n * 100.0:.1f}%)", flush=True)
    print(f"[bevplace] wrote {args.output}", flush=True)
    print(f"[bevplace] elapsed_sec={time.perf_counter() - start:.2f} peak_rss_mb={current_rss_mb():.1f}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
