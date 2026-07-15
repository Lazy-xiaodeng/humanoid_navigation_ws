#!/usr/bin/env python3
"""文件作用：运行官方 SOLiD 召回、航向估计与 GICP 完整验证链路。"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import resource
import subprocess
import tempfile
import time

import numpy as np

from run_cross_fov_full_validation import gicp, load_points, pose_from_matrix, pose_matrix, points_for_query, norm_deg, angle_error_deg, write_csv


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def descriptors(rows: list[dict[str, str]], binary: Path, args: argparse.Namespace) -> np.ndarray:
    with tempfile.TemporaryDirectory(prefix="solid_") as directory:
        manifest, output = Path(directory) / "manifest.txt", Path(directory) / "descriptors.csv"
        manifest.write_text("".join(f"{index}\t{row['pcd']}\n" for index, row in enumerate(rows)), encoding="utf-8")
        subprocess.run([str(binary), str(manifest), str(output), str(args.max_range), str(args.solid_voxel), str(args.fov_down), str(args.fov_up), str(args.sensor_height), str(args.min_range)], check=True)
        data = []
        with output.open(newline="", encoding="utf-8") as stream:
            for row in csv.reader(stream): data.append([float(value) for value in row[1:]])
        return np.asarray(data, dtype=np.float64)


def similarity(query: np.ndarray, database: np.ndarray) -> np.ndarray:
    q = query[:40]
    d = database[:, :40]
    return d @ q / np.maximum(np.linalg.norm(d, axis=1) * np.linalg.norm(q), 1e-12)


def heading(query: np.ndarray, candidate: np.ndarray) -> float:
    q, c = query[40:], candidate[40:]
    errors = [np.abs(c - np.roll(q, shift)).sum() for shift in range(60)]
    return norm_deg((int(np.argmin(errors)) + 1) * 6.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--keyframes", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--top-k", type=int, default=20)
    parser.add_argument("--refine-top-k", type=int, default=8)
    parser.add_argument("--max-range", type=float, default=20.0)
    parser.add_argument("--min-range", type=float, default=0.8)
    parser.add_argument("--solid-voxel", type=float, default=0.4)
    parser.add_argument("--fov-down", type=float, default=-35.0)
    parser.add_argument("--fov-up", type=float, default=35.0)
    parser.add_argument("--sensor-height", type=float, default=1.215)
    parser.add_argument("--selection-rmse-weight", type=float, default=2.0)
    args = parser.parse_args()
    targets, keys = load_rows(args.targets), load_rows(args.keyframes)
    start_all = time.perf_counter()
    key_desc = descriptors(keys, args.binary, args)
    query_desc = descriptors(targets, args.binary, args)
    results, candidates = [], []
    gargs = argparse.Namespace(query_voxel=0.2, query_fov_deg=120.0, scan_min_z=0.2, scan_max_z=2.5, min_range=0.8, max_range=20.0, registration_voxel=0.2, max_correspondence=2.0, gicp_iterations=30)
    key_clouds = [points_for_query(Path(row["pcd"]), gargs) for row in keys]
    for ti, target in enumerate(targets):
        begin = time.perf_counter()
        scores = similarity(query_desc[ti], key_desc)
        order = np.argsort(scores)[::-1][: args.top_k]
        query = points_for_query(Path(target["pcd"]), gargs)
        refined = []
        for rank, ki in enumerate(order[: args.refine_top_k], start=1):
            relative_yaw = heading(query_desc[ti], key_desc[ki])
            result = gicp(query, key_clouds[ki], pose_matrix(0.0, 0.0, relative_yaw), gargs)
            world = pose_matrix(float(keys[ki]["x"]), float(keys[ki]["y"]), float(keys[ki]["yaw_deg"])) @ result.transformation
            x, y, yaw = pose_from_matrix(world)
            te = math.hypot(x - float(target["x"]), y - float(target["y"]))
            ye = angle_error_deg(yaw, float(target["yaw_deg"]))
            item = {"target_id": target["target_id"], "cloud_index": target["cloud_index"], "rank": rank, "keyframe_index": int(ki), "similarity": float(scores[ki]), "x": x, "y": y, "yaw_deg": yaw, "translation_error_m": te, "yaw_error_deg": ye, "fitness": float(result.fitness), "rmse": float(result.inlier_rmse)}
            item["selection"] = -item["similarity"] + args.selection_rmse_weight * item["rmse"]
            refined.append(item); candidates.append(item)
        best = min(refined, key=lambda row: row["selection"])
        results.append({"method": "solid", **best, "success_0p2_5deg": int(best["translation_error_m"] <= 0.2 and best["yaw_error_deg"] <= 5), "success_0p3_5deg": int(best["translation_error_m"] <= 0.3 and best["yaw_error_deg"] <= 5), "elapsed_ms": (time.perf_counter() - begin) * 1000.0, "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0})
        print(f"[solid] {ti+1}/{len(targets)} idx={target['cloud_index']} err={best['translation_error_m']:.3f}m/{best['yaw_error_deg']:.3f}deg", flush=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "results.csv", results); write_csv(args.output_dir / "candidates.csv", candidates)
    summary = [{"targets": len(results), "success_0p2_5deg": sum(r["success_0p2_5deg"] for r in results), "success_0p3_5deg": sum(r["success_0p3_5deg"] for r in results), "median_translation_error_m": float(np.median([r["translation_error_m"] for r in results])), "median_yaw_error_deg": float(np.median([r["yaw_error_deg"] for r in results])), "median_elapsed_ms": float(np.median([r["elapsed_ms"] for r in results])), "total_elapsed_sec": time.perf_counter() - start_all, "peak_rss_mb": max(r["peak_rss_mb"] for r in results)}]
    write_csv(args.output_dir / "summary.csv", summary)
    return 0


if __name__ == "__main__": raise SystemExit(main())
