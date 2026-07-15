#!/usr/bin/env python3
"""文件作用：验证重定位初值上的保守厘米级 scan-to-map 二次精配准。"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import resource
import time

import numpy as np

from run_cross_fov_full_validation import (
    angle_error_deg,
    crop_map,
    gicp,
    load_points,
    norm_deg,
    points_for_query,
    pose_from_matrix,
    pose_matrix,
    write_csv,
)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def pose_delta(lhs: tuple[float, float, float], rhs: tuple[float, float, float]) -> tuple[float, float]:
    return math.hypot(lhs[0] - rhs[0], lhs[1] - rhs[1]), angle_error_deg(lhs[2], rhs[2])


def errors(pose: tuple[float, float, float], truth: tuple[float, float, float]) -> tuple[float, float]:
    return math.hypot(pose[0] - truth[0], pose[1] - truth[1]), angle_error_deg(pose[2], truth[2])


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values), q)) if values else math.nan


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--current-root", type=Path, required=True)
    parser.add_argument("--solid-results", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--sample-stride", type=int, default=1)
    parser.add_argument("--coarse-voxel", type=float, default=0.10)
    parser.add_argument("--fine-voxel", type=float, default=0.05)
    parser.add_argument("--coarse-correspondence", type=float, default=0.60)
    parser.add_argument("--fine-correspondence", type=float, default=0.25)
    parser.add_argument("--max-pose-change-m", type=float, default=0.25)
    parser.add_argument("--max-pose-change-deg", type=float, default=4.0)
    parser.add_argument("--min-fitness", type=float, default=0.65)
    parser.add_argument("--max-rmse", type=float, default=0.20)
    args = parser.parse_args()

    targets = read_rows(args.targets)[:: max(1, args.sample_stride)]
    if args.limit > 0:
        targets = targets[: args.limit]
    solid = {row["target_id"]: row for row in read_rows(args.solid_results)}
    current: dict[str, dict[str, str]] = {}
    groups = sorted({row["group"] for row in targets})
    for group in groups:
        path = args.current_root / group / "global_relocalization_metrics.csv"
        rows = [row for row in read_rows(path) if row["scenario_name"] == "arbitrary_start_no_prior"]
        for index, row in enumerate(rows, start=1):
            current[f"{group}__metric_{index:03d}"] = row

    map_points = load_points(args.map)
    query_args = argparse.Namespace(
        query_voxel=args.fine_voxel,
        query_fov_deg=120.0,
        scan_min_z=0.2,
        scan_max_z=2.5,
        min_range=0.8,
        max_range=20.0,
    )
    rows_out: list[dict[str, object]] = []
    for index, target in enumerate(targets, start=1):
        target_id = target["target_id"]
        if target_id not in current or target_id not in solid:
            continue
        truth = float(target["x"]), float(target["y"]), float(target["yaw_deg"])
        source_poses = {
            "current": (
                float(current[target_id]["final_x_m"]),
                float(current[target_id]["final_y_m"]),
                float(current[target_id]["final_yaw_deg"]),
            ),
            "solid": (float(solid[target_id]["x"]), float(solid[target_id]["y"]), float(solid[target_id]["yaw_deg"])),
        }
        query = points_for_query(Path(target["pcd"]), query_args)
        for source, initial in source_poses.items():
            begin = time.perf_counter()
            local_map = crop_map(map_points, initial[0], initial[1], 22.0)
            coarse_args = argparse.Namespace(
                registration_voxel=args.coarse_voxel,
                max_correspondence=args.coarse_correspondence,
                gicp_iterations=40,
            )
            fine_args = argparse.Namespace(
                registration_voxel=args.fine_voxel,
                max_correspondence=args.fine_correspondence,
                gicp_iterations=50,
            )
            coarse = gicp(query, local_map, pose_matrix(*initial), coarse_args)
            fine = gicp(query, local_map, coarse.transformation, fine_args)
            refined = pose_from_matrix(fine.transformation)
            change_m, change_deg = pose_delta(refined, initial)
            accepted = (
                fine.fitness >= args.min_fitness
                and fine.inlier_rmse <= args.max_rmse
                and change_m <= args.max_pose_change_m
                and change_deg <= args.max_pose_change_deg
            )
            final = refined if accepted else initial
            initial_te, initial_ye = errors(initial, truth)
            final_te, final_ye = errors(final, truth)
            rows_out.append(
                {
                    "target_id": target_id,
                    "source": source,
                    "accepted": int(accepted),
                    "initial_translation_error_m": initial_te,
                    "initial_yaw_error_deg": initial_ye,
                    "final_translation_error_m": final_te,
                    "final_yaw_error_deg": final_ye,
                    "pose_change_m": change_m,
                    "pose_change_deg": change_deg,
                    "fitness": float(fine.fitness),
                    "rmse": float(fine.inlier_rmse),
                    "elapsed_ms": (time.perf_counter() - begin) * 1000.0,
                }
            )
        print(f"[centimeter] {index}/{len(targets)} {target_id}", flush=True)

    summary: list[dict[str, object]] = []
    for source in ("current", "solid"):
        selected = [row for row in rows_out if row["source"] == source]
        te = [float(row["final_translation_error_m"]) for row in selected]
        ye = [float(row["final_yaw_error_deg"]) for row in selected]
        summary.append(
            {
                "source": source,
                "targets": len(selected),
                "refinement_accepted": sum(int(row["accepted"]) for row in selected),
                "success_0p05_2deg": sum(e <= 0.05 and y <= 2.0 for e, y in zip(te, ye)),
                "success_0p10_3deg": sum(e <= 0.10 and y <= 3.0 for e, y in zip(te, ye)),
                "success_0p20_5deg": sum(e <= 0.20 and y <= 5.0 for e, y in zip(te, ye)),
                "success_0p30_5deg": sum(e <= 0.30 and y <= 5.0 for e, y in zip(te, ye)),
                "translation_min_m": min(te),
                "translation_median_m": percentile(te, 50),
                "translation_p95_m": percentile(te, 95),
                "translation_max_m": max(te),
                "yaw_min_deg": min(ye),
                "yaw_median_deg": percentile(ye, 50),
                "yaw_p95_deg": percentile(ye, 95),
                "yaw_max_deg": max(ye),
                "median_elapsed_ms": percentile([float(row["elapsed_ms"]) for row in selected], 50),
                "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
            }
        )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "results.csv", rows_out)
    write_csv(args.output_dir / "summary.csv", summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
