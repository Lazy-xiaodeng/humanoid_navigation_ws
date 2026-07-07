#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_ring_official_validation.py

文件作用：
  1. 用 RING/RING++ 官方仓库中的 RING occupancy 完整 CPU 分支验证真实 keyframe scan 数据库的全局重定位能力。
  2. 读取离线导出的 keyframes.csv 和 targets.csv，对每个 keyframe/target 生成官方 BEV、RING 和 TIRING 描述子。
  3. 使用官方 fast_corr 做 yaw/地点召回，并使用官方 solve_translation 做 query->keyframe 的 2D 平移估计。
  4. 输出 top1/topK 位置误差、yaw 误差、资源消耗和依赖状态，用于判断 RING 是否值得继续融合进 C++ 恢复链路。

重要说明：
  - 当前机器没有 CUDA/nvcc，RING++ 的 point-feature Cython 分支不能完整运行；本脚本跑的是官方 RING occupancy CPU 分支。
  - 官方 fast_gicp Python 扩展因 pybind11/Python3.12/PCL1.14 兼容问题未能安装，因此本脚本不伪装执行 refinement，只统计 RING 初始位姿。
  - 脚本会排除目标 cloud_index 附近的 keyframe，避免目标帧邻域泄漏造成过高结果。
"""

from __future__ import annotations

import argparse
import csv
import math
import resource
import sys
import time
import types
import zlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d
import torch


@dataclass(frozen=True)
class PoseCloud:
    """一个带 map 位姿和 PCD 路径的 keyframe/target。"""

    sample_id: str
    cloud_index: int
    x: float
    y: float
    yaw_deg: float
    pcd_path: Path


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""

    return Path(__file__).resolve().parents[3]


def current_rss_mb() -> float:
    """读取当前进程最大 RSS；Linux ru_maxrss 单位为 KB。"""

    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def normalize_angle_deg(angle: float) -> float:
    """把角度规整到 [-180, 180]，方便 yaw 误差统计。"""

    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def pose_matrix_2d(x: float, y: float, yaw_deg: float) -> np.ndarray:
    """构造 2D 齐次位姿矩阵。"""

    yaw = math.radians(yaw_deg)
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]], dtype=np.float64)


def yaw_from_matrix_2d(matrix: np.ndarray) -> float:
    """从 2D 齐次矩阵读取 yaw，单位 deg。"""

    return math.degrees(math.atan2(float(matrix[1, 0]), float(matrix[0, 0])))


def read_keyframes(path: Path) -> list[PoseCloud]:
    """读取 keyframes.csv。"""

    rows: list[PoseCloud] = []
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows.append(
                PoseCloud(
                    sample_id=row.get("frame_id", f"keyframe_{len(rows):06d}"),
                    cloud_index=int(row["cloud_index"]),
                    x=float(row["x"]),
                    y=float(row["y"]),
                    yaw_deg=float(row["yaw_deg"]),
                    pcd_path=Path(row["pcd"]),
                )
            )
    return rows


def read_targets(path: Path) -> list[PoseCloud]:
    """读取 targets.csv。"""

    rows: list[PoseCloud] = []
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows.append(
                PoseCloud(
                    sample_id=row.get("target_id", f"target_{len(rows):06d}"),
                    cloud_index=int(row["cloud_index"]),
                    x=float(row["x"]),
                    y=float(row["y"]),
                    yaw_deg=float(row["yaw_deg"]),
                    pcd_path=Path(row["pcd"]),
                )
            )
    return rows


def install_ring_import_shims(upstream_root: Path) -> None:
    """给官方 RING CPU 分支安装 import shim，避开当前机器缺失的 CUDA-only 模块。"""

    # 官方 core.py 顶层 import voxelocc/voxelfeat/torch_radon；CPU occupancy 路径不会调用这些模块。
    # 这里放入空模块，只是让 Python 能加载官方 CPU 函数，不改变实际参与验证的 RING 算法函数。
    for module_name in ["voxelocc", "voxelfeat"]:
        if module_name not in sys.modules:
            sys.modules[module_name] = types.ModuleType(module_name)
    if "torch_radon" not in sys.modules:
        fake = types.ModuleType("torch_radon")
        fake.Radon = object
        fake.ParallelBeam = object
        fake.RadonFanbeam = object
        sys.modules["torch_radon"] = fake

    sys.path.insert(0, str(upstream_root))
    # 官方 generate_bev() 的 CPU 分支里仍对 scene_centroid 调 .cuda()；CPU-only 环境下把它退化成 no-op。
    torch.Tensor.cuda = lambda self, *args, **kwargs: self  # type: ignore[assignment]


def load_ring_core(upstream_root: Path):
    """加载 RING 官方核心函数和配置。"""

    install_ring_import_shims(upstream_root)
    import utils.config as cfg  # pylint: disable=import-error,import-outside-toplevel
    from utils.core import (  # pylint: disable=import-error,import-outside-toplevel
        fast_corr,
        generate_RING_cpu,
        generate_bev,
        rotate_bev,
        solve_translation,
    )

    return cfg, generate_bev, generate_RING_cpu, fast_corr, rotate_bev, solve_translation


def load_points(path: Path, max_points: int, random_seed: int) -> np.ndarray:
    """读取 PCD 并按需确定性下采样，避免 RING/BEV 在超大单帧上耗时过高。"""

    points = np.asarray(o3d.io.read_point_cloud(str(path)).points, dtype=np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    if max_points > 0 and len(points) > max_points:
        stable_path_hash = zlib.crc32(str(path).encode("utf-8")) % 1000003
        rng = np.random.default_rng(random_seed + stable_path_hash)
        keep = rng.choice(len(points), size=max_points, replace=False)
        points = points[keep]
    return points[:, :3].astype(np.float32)


def build_descriptor(sample: PoseCloud, args: argparse.Namespace, generate_bev, generate_RING_cpu) -> dict[str, object]:
    """对一个 PCD 运行官方 BEV -> RING/TIRING 描述子生成。"""

    points = load_points(sample.pcd_path, args.max_points, args.random_seed)
    bev = generate_bev(points)
    ring, tiring = generate_RING_cpu(bev)
    return {"sample": sample, "bev": bev, "ring": ring, "tiring": tiring}


def estimate_pose_from_candidate(
    target_entry: dict[str, object],
    key_entry: dict[str, object],
    cfg,
    fast_corr,
    rotate_bev,
    solve_translation,
) -> dict[str, float]:
    """使用官方 fast_corr + solve_translation 从 target 到 keyframe 估计相对 2D 位姿。"""

    target = target_entry["sample"]
    key = key_entry["sample"]
    dist, angle_matched = fast_corr(target_entry["tiring"], key_entry["tiring"])
    angle_matched = int(angle_matched)
    angle_extra = angle_matched - cfg.num_ring // 2
    angle_res = 2.0 * math.pi / float(cfg.num_ring)
    angle_rad = float(angle_matched) * angle_res
    angle_extra_rad = float(angle_extra) * angle_res

    bev_rotated = rotate_bev(target_entry["bev"], angle_rad)
    bev_rotated_extra = rotate_bev(target_entry["bev"], angle_extra_rad)
    x_shift, y_shift, corr_error = solve_translation(bev_rotated, key_entry["bev"])
    x_shift_extra, y_shift_extra, corr_error_extra = solve_translation(bev_rotated_extra, key_entry["bev"])
    if float(corr_error) < float(corr_error_extra):
        shift_x = float(x_shift)
        shift_y = float(y_shift)
        rel_yaw_rad = angle_rad
        translation_error_score = float(corr_error)
    else:
        shift_x = float(x_shift_extra)
        shift_y = float(y_shift_extra)
        rel_yaw_rad = angle_extra_rad
        translation_error_score = float(corr_error_extra)

    trans_x = shift_x / float(cfg.num_sector) * float(cfg.point_cloud["x_bound"][1] - cfg.point_cloud["x_bound"][0])
    trans_y = shift_y / float(cfg.num_ring) * float(cfg.point_cloud["y_bound"][1] - cfg.point_cloud["y_bound"][0])
    relative = pose_matrix_2d(trans_x, trans_y, math.degrees(rel_yaw_rad))
    predicted = pose_matrix_2d(key.x, key.y, key.yaw_deg) @ relative
    pred_x = float(predicted[0, 2])
    pred_y = float(predicted[1, 2])
    pred_yaw = normalize_angle_deg(yaw_from_matrix_2d(predicted))
    trans_error = math.hypot(pred_x - target.x, pred_y - target.y)
    yaw_error = abs(normalize_angle_deg(pred_yaw - target.yaw_deg))
    return {
        "descriptor_distance": float(dist),
        "translation_error_score": translation_error_score,
        "pred_x": pred_x,
        "pred_y": pred_y,
        "pred_yaw_deg": pred_yaw,
        "trans_error_m": trans_error,
        "yaw_error_deg": yaw_error,
        "relative_x": trans_x,
        "relative_y": trans_y,
        "relative_yaw_deg": normalize_angle_deg(math.degrees(rel_yaw_rad)),
    }


def evaluate_target(
    target_entry: dict[str, object],
    key_entries: list[dict[str, object]],
    args: argparse.Namespace,
    cfg,
    fast_corr,
    rotate_bev,
    solve_translation,
) -> dict[str, str]:
    """对单个目标计算 top-K RING 候选并统计最佳候选误差。"""

    target = target_entry["sample"]
    scored: list[tuple[float, int, dict[str, float]]] = []
    for index, key_entry in enumerate(key_entries):
        key = key_entry["sample"]
        if abs(key.cloud_index - target.cloud_index) <= args.exclude_cloud_index_window:
            continue
        estimate = estimate_pose_from_candidate(target_entry, key_entry, cfg, fast_corr, rotate_bev, solve_translation)
        scored.append((estimate["descriptor_distance"], index, estimate))

    scored.sort(key=lambda item: item[0])
    top = scored[: max(1, args.top_k)]
    # RING 的 descriptor 距离负责召回；最终 pose 口径用 topK 中平移+yaw 最接近的 oracle 和 top1 分开统计。
    top1_distance, top1_index, top1_estimate = top[0]
    oracle_distance, oracle_index, oracle_estimate = min(
        top,
        key=lambda item: item[2]["trans_error_m"] + 0.05 * item[2]["yaw_error_deg"],
    )
    key_top1 = key_entries[top1_index]["sample"]
    key_oracle = key_entries[oracle_index]["sample"]
    return {
        "target_id": target.sample_id,
        "cloud_index": str(target.cloud_index),
        "reference_x": f"{target.x:.6f}",
        "reference_y": f"{target.y:.6f}",
        "reference_yaw_deg": f"{target.yaw_deg:.6f}",
        "top1_key_cloud_index": str(key_top1.cloud_index),
        "top1_descriptor_distance": f"{top1_distance:.9f}",
        "top1_trans_error_m": f"{top1_estimate['trans_error_m']:.6f}",
        "top1_yaw_error_deg": f"{top1_estimate['yaw_error_deg']:.6f}",
        "topk_oracle_key_cloud_index": str(key_oracle.cloud_index),
        "topk_oracle_descriptor_distance": f"{oracle_distance:.9f}",
        "topk_oracle_trans_error_m": f"{oracle_estimate['trans_error_m']:.6f}",
        "topk_oracle_yaw_error_deg": f"{oracle_estimate['yaw_error_deg']:.6f}",
        "success_top1_0p2_3deg": "1" if top1_estimate["trans_error_m"] <= 0.2 and top1_estimate["yaw_error_deg"] <= 3.0 else "0",
        "success_top1_0p3_5deg": "1" if top1_estimate["trans_error_m"] <= 0.3 and top1_estimate["yaw_error_deg"] <= 5.0 else "0",
        "success_top1_0p5_10deg": "1" if top1_estimate["trans_error_m"] <= 0.5 and top1_estimate["yaw_error_deg"] <= 10.0 else "0",
        "success_topk_oracle_0p2_3deg": "1" if oracle_estimate["trans_error_m"] <= 0.2 and oracle_estimate["yaw_error_deg"] <= 3.0 else "0",
        "success_topk_oracle_0p3_5deg": "1" if oracle_estimate["trans_error_m"] <= 0.3 and oracle_estimate["yaw_error_deg"] <= 5.0 else "0",
        "success_topk_oracle_0p5_10deg": "1" if oracle_estimate["trans_error_m"] <= 0.5 and oracle_estimate["yaw_error_deg"] <= 10.0 else "0",
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
    """解析命令行参数。"""

    root = repo_root_from_script()
    parser = argparse.ArgumentParser(description="RING 官方 CPU occupancy 分支离线验证")
    parser.add_argument("--upstream-root", type=Path, default=root / ".codex_tmp/upstream_ring_ndtmc/RING", help="RING 官方仓库目录")
    parser.add_argument("--keyframes-csv", type=Path, required=True, help="keyframes.csv")
    parser.add_argument("--targets-csv", type=Path, required=True, help="targets.csv")
    parser.add_argument("--output", type=Path, required=True, help="输出 CSV")
    parser.add_argument("--top-k", type=int, default=20, help="统计 topK oracle 的候选数量")
    parser.add_argument("--exclude-cloud-index-window", type=int, default=240, help="排除目标 cloud_index 前后窗口，避免泄漏")
    parser.add_argument("--max-targets", type=int, default=0, help="只验证前 N 个目标；0 表示全量")
    parser.add_argument("--max-points", type=int, default=60000, help="每帧最多参与 RING 的点数；0 表示不采样")
    parser.add_argument("--random-seed", type=int, default=20260706, help="确定性点采样随机种子")
    return parser.parse_args()


def main() -> int:
    """主流程：生成 keyframe RING 数据库，逐目标验证召回和位姿误差。"""

    args = parse_args()
    start = time.perf_counter()
    cfg, generate_bev, generate_RING_cpu, fast_corr, rotate_bev, solve_translation = load_ring_core(args.upstream_root)
    keyframes = read_keyframes(args.keyframes_csv)
    targets = read_targets(args.targets_csv)
    if args.max_targets > 0:
        targets = targets[: args.max_targets]

    print(f"[ring] build keyframe descriptors keyframes={len(keyframes)}", flush=True)
    key_entries = []
    for index, keyframe in enumerate(keyframes, start=1):
        key_entries.append(build_descriptor(keyframe, args, generate_bev, generate_RING_cpu))
        if index % max(1, len(keyframes) // 10) == 0:
            print(f"[ring] keyframe {index}/{len(keyframes)} rss_mb={current_rss_mb():.1f}", flush=True)

    rows: list[dict[str, str]] = []
    for index, target in enumerate(targets, start=1):
        target_entry = build_descriptor(target, args, generate_bev, generate_RING_cpu)
        row = evaluate_target(target_entry, key_entries, args, cfg, fast_corr, rotate_bev, solve_translation)
        rows.append(row)
        print(
            f"[ring] {index}/{len(targets)} idx={target.cloud_index} "
            f"top1={row['top1_trans_error_m']}m/{row['top1_yaw_error_deg']}deg "
            f"oracle={row['topk_oracle_trans_error_m']}m/{row['topk_oracle_yaw_error_deg']}deg "
            f"success03={row['success_top1_0p3_5deg']}",
            flush=True,
        )

    write_csv(args.output, rows)
    n = max(1, len(rows))
    for prefix in ["top1", "topk_oracle"]:
        success02 = sum(int(row[f"success_{prefix}_0p2_3deg"]) for row in rows)
        success03 = sum(int(row[f"success_{prefix}_0p3_5deg"]) for row in rows)
        success05 = sum(int(row[f"success_{prefix}_0p5_10deg"]) for row in rows)
        print(f"[ring] {prefix}_success_0p2_3deg={success02}/{n} ({success02 / n * 100.0:.1f}%)", flush=True)
        print(f"[ring] {prefix}_success_0p3_5deg={success03}/{n} ({success03 / n * 100.0:.1f}%)", flush=True)
        print(f"[ring] {prefix}_success_0p5_10deg={success05}/{n} ({success05 / n * 100.0:.1f}%)", flush=True)
    print(f"[ring] wrote {args.output}", flush=True)
    print(f"[ring] elapsed_sec={time.perf_counter() - start:.2f} peak_rss_mb={current_rss_mb():.1f}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
