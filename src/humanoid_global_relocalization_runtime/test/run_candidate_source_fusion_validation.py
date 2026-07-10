#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_candidate_source_fusion_validation.py

文件作用：
  1. 离线验证“多候选源召回 + 可见性/禁穿墙软重排序”的完整恢复链路。
  2. 输入已经由完整算法生成的候选 CSV，例如 Scan Context+3D-BBS+GICP 的最终输出、
     2.5D BBS+GICP 的 topK 候选明细；本脚本只负责融合、去重和二次物理一致性排序。
  3. 对每个候选计算 scan 端点贴图比例、射线提前撞墙比例、站位可行性，再按无真值分数选择最终位姿。
  4. 输出候选明细和每 target 的最终选择，用于判断这条低成本路线是否值得移植进 C++ 运行态。
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import resource
import time

import numpy as np
from scipy.spatial import cKDTree

from run_area_visibility_gate_validation import (
    CandidateGateResult,
    TargetSample,
    build_occupancy_grid,
    candidate_pose,
    evaluate_candidate,
    load_map_points,
    load_scan_points,
    normalize_angle_deg,
    read_targets,
)


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[3]


def current_rss_mb() -> float:
    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


def current_cpu_seconds() -> float:
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return float(usage.ru_utime + usage.ru_stime)


def to_float(row: dict[str, str], names: list[str], default: float) -> float:
    for name in names:
      value = row.get(name)
      if value not in (None, ""):
        try:
          return float(value)
        except ValueError:
          continue
    return default


def normalize_source_row(source: str, row: dict[str, str], ordinal: int) -> dict[str, str] | None:
    """把不同算法输出的 CSV 行统一成 area visibility 可识别的候选字段。"""

    target_index = row.get("target_index") or row.get("cloud_index") or row.get("bag_frame_index")
    if target_index in (None, "", "-1"):
        return None

    if "final_x_m" in row and "final_y_m" in row and "final_yaw_deg" in row:
        x_name, y_name, yaw_name = "final_x_m", "final_y_m", "final_yaw_deg"
    elif "candidate_x_m" in row and "candidate_y_m" in row and "candidate_yaw_deg" in row:
        x_name, y_name, yaw_name = "candidate_x_m", "candidate_y_m", "candidate_yaw_deg"
    else:
        return None

    rank = int(to_float(row, ["candidate_rank", "rank", "descriptor_rank", "refined_candidate_rank"], float(ordinal)))
    quality = to_float(row, ["gicp_rmse", "inlier_rmse", "refine_fitness_score", "gicp_fitness"], 1.0)
    score = to_float(row, ["bbs_score", "score_ratio", "fitness"], 0.0)

    normalized = dict(row)
    normalized.update(
        {
            "source": source,
            "target_index": str(int(float(target_index))),
            "rank": str(rank),
            "final_x_m": row[x_name],
            "final_y_m": row[y_name],
            "final_yaw_deg": row[yaw_name],
            "inlier_rmse": f"{quality:.9f}",
            "source_quality": f"{quality:.9f}",
            "source_score": f"{score:.9f}",
        }
    )
    return normalized


def read_source_rows(spec: str, max_rank: int) -> dict[int, list[dict[str, str]]]:
    """读取 label:path 格式的候选源。"""

    if ":" not in spec:
        raise ValueError(f"source spec must be label:path, got {spec}")
    label, path_text = spec.split(":", maxsplit=1)
    path = Path(path_text)
    grouped: dict[int, list[dict[str, str]]] = {}
    with path.open(newline="", encoding="utf-8") as f:
        for ordinal, row in enumerate(csv.DictReader(f), start=1):
            normalized = normalize_source_row(label, row, ordinal)
            if normalized is None:
                continue
            rank = int(normalized["rank"])
            if max_rank > 0 and rank > max_rank and label != "scan_context":
                continue
            target_index = int(normalized["target_index"])
            grouped.setdefault(target_index, []).append(normalized)
    return grouped


def merge_sources(source_specs: list[str], max_rank: int) -> dict[int, list[dict[str, str]]]:
    grouped: dict[int, list[dict[str, str]]] = {}
    for spec in source_specs:
        source_grouped = read_source_rows(spec, max_rank)
        for target_index, rows in source_grouped.items():
            grouped.setdefault(target_index, []).extend(rows)
    return grouped


def deduplicate_candidates(
    rows: list[dict[str, str]],
    xy_gate_m: float,
    yaw_gate_deg: float,
    args: argparse.Namespace,
) -> list[dict[str, str]]:
    """按最终位姿去重，保留同一小簇里质量分最低的候选。"""

    selected: list[dict[str, str]] = []
    for row in sorted(rows, key=lambda item: source_base_score(item, args)):
        x, y, yaw = candidate_pose(row)
        duplicate = False
        for kept in selected:
            kx, ky, kyaw = candidate_pose(kept)
            if math.hypot(x - kx, y - ky) <= xy_gate_m and abs(normalize_angle_deg(yaw - kyaw)) <= yaw_gate_deg:
                duplicate = True
                break
        if not duplicate:
            selected.append(row)
    return selected


def source_base_score(row: dict[str, str], args: argparse.Namespace) -> float:
    """候选源自身质量分，越小越好，不使用真值。"""

    source = row.get("source", "")
    rank = to_float(row, ["rank"], 99.0)
    quality = to_float(row, ["source_quality", "inlier_rmse"], 1.0)
    source_prior = {
        "scan_context": args.scan_context_prior,
        "bbs2p5d": args.bbs2p5d_prior,
        "triangle": 0.04,
    }.get(source, 0.02)
    return source_prior + args.quality_weight * quality + args.rank_weight * rank


def fusion_score(result: CandidateGateResult, args: argparse.Namespace) -> float:
    """最终融合分数，越小越好；只使用算法质量和物理一致性，不使用真值。"""

    row = result.row
    return (
        source_base_score(row, args)
        + args.wall_penalty_weight * result.ray_wall_cross_ratio
        + args.endpoint_penalty_weight * max(0.0, args.endpoint_near_reference - result.endpoint_near_ratio)
        - args.endpoint_reward_weight * result.endpoint_near_ratio
        + (0.0 if result.origin_clear else args.origin_blocked_penalty)
    )


def success_flags(result: CandidateGateResult) -> tuple[bool, bool, bool, bool]:
    return (
        result.translation_error_m <= 0.2 and result.yaw_error_deg <= 3.0,
        result.translation_error_m <= 0.3 and result.yaw_error_deg <= 5.0,
        result.translation_error_m <= 0.5 and result.yaw_error_deg <= 10.0,
        result.translation_error_m <= 0.8 and result.yaw_error_deg <= 15.0,
    )


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row.keys():
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate source-level candidate fusion with visibility reranking.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script())
    parser.add_argument("--map", type=Path, default=Path("src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd"))
    parser.add_argument("--targets", type=Path, default=Path(".codex_tmp/official_std_standalone/nav46_hard52_dataset/targets.csv"))
    parser.add_argument("--source", action="append", required=True, help="候选源，格式 label:path")
    parser.add_argument("--output", type=Path, default=Path(".codex_tmp/source_fusion/hard52_selected.csv"))
    parser.add_argument("--details-output", type=Path, default=Path(".codex_tmp/source_fusion/hard52_details.csv"))
    parser.add_argument("--max-rank", type=int, default=16)
    parser.add_argument("--dedup-xy-m", type=float, default=0.35)
    parser.add_argument("--dedup-yaw-deg", type=float, default=8.0)
    parser.add_argument("--scan-context-prior", type=float, default=-0.18)
    parser.add_argument("--bbs2p5d-prior", type=float, default=0.0)
    parser.add_argument("--quality-weight", type=float, default=0.65)
    parser.add_argument("--rank-weight", type=float, default=0.012)

    parser.add_argument("--map-voxel-size", type=float, default=0.12)
    parser.add_argument("--map-min-z", type=float, default=0.15)
    parser.add_argument("--map-max-z", type=float, default=2.20)
    parser.add_argument("--occupancy-resolution", type=float, default=0.18)
    parser.add_argument("--scan-voxel-size", type=float, default=0.18)
    parser.add_argument("--max-scan-points", type=int, default=650)
    parser.add_argument("--ray-sample-points", type=int, default=120)
    parser.add_argument("--min-range", type=float, default=0.35)
    parser.add_argument("--max-range", type=float, default=18.0)
    parser.add_argument("--scan-min-z", type=float, default=-1.0)
    parser.add_argument("--scan-max-z", type=float, default=2.2)
    parser.add_argument("--endpoint-near-distance", type=float, default=0.45)
    parser.add_argument("--endpoint-near-reference", type=float, default=0.45)
    parser.add_argument("--ray-step", type=float, default=0.22)
    parser.add_argument("--wall-before-end-tolerance", type=float, default=0.70)
    parser.add_argument("--origin-clearance-radius", type=float, default=0.25)
    parser.add_argument("--wall-penalty-weight", type=float, default=0.35)
    parser.add_argument("--endpoint-penalty-weight", type=float, default=0.0)
    parser.add_argument("--endpoint-reward-weight", type=float, default=0.75)
    parser.add_argument("--origin-blocked-penalty", type=float, default=0.0)
    parser.add_argument("--random-seed", type=int, default=7)
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    map_path = args.map if args.map.is_absolute() else workspace / args.map
    targets_path = args.targets if args.targets.is_absolute() else workspace / args.targets
    output_path = args.output if args.output.is_absolute() else workspace / args.output
    details_path = args.details_output if args.details_output.is_absolute() else workspace / args.details_output
    source_specs = []
    for spec in args.source:
        label, path_text = spec.split(":", maxsplit=1)
        path = Path(path_text)
        if not path.is_absolute():
            path = workspace / path
        source_specs.append(f"{label}:{path}")

    start_time = time.time()
    start_cpu = current_cpu_seconds()
    map_points = load_map_points(map_path, args)
    map_kdtree = cKDTree(map_points[:, :3])
    grid = build_occupancy_grid(map_points, args)
    targets = read_targets(targets_path)
    grouped = merge_sources(source_specs, args.max_rank)

    selected_rows: list[dict[str, str]] = []
    detail_rows: list[dict[str, str]] = []
    for target_index, target in sorted(targets.items()):
        raw_rows = grouped.get(target_index, [])
        rows = deduplicate_candidates(raw_rows, args.dedup_xy_m, args.dedup_yaw_deg, args)
        if not rows:
            continue
        scan_points = load_scan_points(target.pcd_path, args)
        evaluated = [evaluate_candidate(row, target, scan_points, map_kdtree, grid, args) for row in rows]
        selected = min(evaluated, key=lambda item: fusion_score(item, args))
        strict, practical, recoverable, guard = success_flags(selected)
        selected_row = {
            "target_index": str(target_index),
            "target_id": target.waypoint_id,
            "candidate_count_raw": str(len(raw_rows)),
            "candidate_count_dedup": str(len(rows)),
            "selected_source": selected.row.get("source", ""),
            "selected_rank": selected.row.get("rank", ""),
            "fusion_score": f"{fusion_score(selected, args):.9f}",
            "endpoint_near_ratio": f"{selected.endpoint_near_ratio:.9f}",
            "ray_wall_cross_ratio": f"{selected.ray_wall_cross_ratio:.9f}",
            "origin_clear": "1" if selected.origin_clear else "0",
            "final_x_m": f"{selected.final_x:.6f}",
            "final_y_m": f"{selected.final_y:.6f}",
            "final_yaw_deg": f"{selected.final_yaw_deg:.6f}",
            "translation_error_m": f"{selected.translation_error_m:.6f}",
            "yaw_error_deg": f"{selected.yaw_error_deg:.6f}",
            "success_0p2m_3deg": "1" if strict else "0",
            "success_0p3m_5deg": "1" if practical else "0",
            "success_0p5m_10deg": "1" if recoverable else "0",
            "success_0p8m_15deg": "1" if guard else "0",
        }
        selected_rows.append(selected_row)
        for result in evaluated:
            s0, s1, s2, s3 = success_flags(result)
            detail = dict(result.row)
            detail.update(
                {
                    "fusion_score": f"{fusion_score(result, args):.9f}",
                    "endpoint_near_ratio": f"{result.endpoint_near_ratio:.9f}",
                    "ray_wall_cross_ratio": f"{result.ray_wall_cross_ratio:.9f}",
                    "origin_clear": "1" if result.origin_clear else "0",
                    "truth_translation_error_m": f"{result.translation_error_m:.6f}",
                    "truth_yaw_error_deg": f"{result.yaw_error_deg:.6f}",
                    "truth_success_0p2m_3deg": "1" if s0 else "0",
                    "truth_success_0p3m_5deg": "1" if s1 else "0",
                    "truth_success_0p5m_10deg": "1" if s2 else "0",
                    "truth_success_0p8m_15deg": "1" if s3 else "0",
                }
            )
            detail_rows.append(detail)

    write_csv(output_path, selected_rows)
    write_csv(details_path, detail_rows)
    total = len(selected_rows)
    counts = {
        "0.2m/3deg": sum(row["success_0p2m_3deg"] == "1" for row in selected_rows),
        "0.3m/5deg": sum(row["success_0p3m_5deg"] == "1" for row in selected_rows),
        "0.5m/10deg": sum(row["success_0p5m_10deg"] == "1" for row in selected_rows),
        "0.8m/15deg": sum(row["success_0p8m_15deg"] == "1" for row in selected_rows),
    }
    source_counts: dict[str, int] = {}
    for row in selected_rows:
        source_counts[row["selected_source"]] = source_counts.get(row["selected_source"], 0) + 1
    elapsed = time.time() - start_time
    cpu = (current_cpu_seconds() - start_cpu) / max(elapsed, 1e-6)
    print(f"[source_fusion] selected={output_path}")
    print(f"[source_fusion] details={details_path}")
    print(f"[source_fusion] targets={total} counts={counts} selected_sources={source_counts}")
    print(f"[source_fusion] elapsed_sec={elapsed:.3f} cpu_core_equiv={cpu:.2f} rss_mb={current_rss_mb():.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
