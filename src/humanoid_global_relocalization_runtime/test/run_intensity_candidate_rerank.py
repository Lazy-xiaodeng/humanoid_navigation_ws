#!/usr/bin/env python3
"""Rerank geometric keyframe candidates using aligned LiDAR intensity."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def normalize_intensity(values: np.ndarray) -> np.ndarray:
    finite = values[np.isfinite(values)]
    low, high = np.percentile(finite, (5.0, 95.0))
    return np.clip((values - low) / max(high - low, 1e-6), 0.0, 1.0)


def voxel_indices(points: np.ndarray, voxel: float) -> np.ndarray:
    keys = np.floor(points / voxel).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return np.sort(indices)


def pose(x: float, y: float, yaw_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:2, :2] = ((c, -s), (s, c))
    result[0, 3], result[1, 3] = x, y
    return result


def load_sidecars(paths: list[Path]) -> dict[tuple[str, str], Path]:
    result = {}
    for path in paths:
        for row in read(path):
            result[(row["source_bag"], row["item_id"])] = Path(row["sidecar"])
    return result


def aligned_intensity_score(query_path: Path, key_path: Path, relative: np.ndarray) -> tuple[float, float, float, int]:
    query_data, key_data = np.load(query_path), np.load(key_path)
    query_xyz, key_xyz = query_data["xyz"].astype(np.float64), key_data["xyz"].astype(np.float64)
    query_i, key_i = normalize_intensity(query_data["intensity"]), normalize_intensity(key_data["intensity"])
    query_keep, key_keep = voxel_indices(query_xyz, 0.10), voxel_indices(key_xyz, 0.10)
    query_xyz, query_i = query_xyz[query_keep], query_i[query_keep]
    key_xyz, key_i = key_xyz[key_keep], key_i[key_keep]
    transformed = query_xyz @ relative[:3, :3].T + relative[:3, 3]
    distances, indices = cKDTree(key_xyz).query(transformed, k=1, distance_upper_bound=0.20, workers=1)
    valid = np.isfinite(distances) & (indices < len(key_xyz))
    if valid.sum() < 100:
        return 1.0, 0.0, float(valid.mean()), int(valid.sum())
    lhs, rhs = query_i[valid], key_i[indices[valid]]
    difference = np.abs(lhs - rhs)
    correlation = float(np.corrcoef(lhs, rhs)[0, 1]) if np.std(lhs) > 1e-6 and np.std(rhs) > 1e-6 else 0.0
    return float(np.median(difference)), correlation, float(valid.mean()), int(valid.sum())


def angle_error(lhs: float, rhs: float) -> float:
    return abs((lhs - rhs + 180.0) % 360.0 - 180.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--keyframes", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--sidecar-manifests", nargs="+", type=Path, required=True)
    parser.add_argument("--query-source", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    targets = {row["target_id"]: row for row in read(args.targets)}
    keys = {int(row["frame_id"]): row for row in read(args.keyframes)}
    sidecars = load_sidecars(args.sidecar_manifests)
    candidates = read(args.candidates)
    cache: dict[tuple[str, int], tuple[float, float, float, int]] = {}
    output = []
    start = time.perf_counter()
    for number, item in enumerate(candidates, start=1):
        target, key = targets[item["target_id"]], keys[int(item["keyframe_index"])]
        query_path = sidecars[(args.query_source, target["target_id"])]
        key_path = sidecars[(key["source_bag"], key["frame_id"])]
        relative = np.linalg.inv(pose(float(key["x"]), float(key["y"]), float(key["yaw_deg"]))) @ pose(float(item["x"]), float(item["y"]), float(item["yaw_deg"]))
        metrics = aligned_intensity_score(query_path, key_path, relative)
        output.append({**item, "intensity_median_abs_diff": metrics[0], "intensity_correlation": metrics[1], "intensity_overlap": metrics[2], "intensity_pairs": metrics[3]})
        if number % 100 == 0:
            print(f"[intensity_rerank] {number}/{len(candidates)}", flush=True)
    write(args.output_dir / "candidates.csv", output)
    groups: dict[str, list[dict[str, object]]] = {}
    for row in output:
        groups.setdefault(str(row["target_id"]), []).append(row)
    summaries = []
    results = []
    for weight in (0.0, 0.25, 0.5, 1.0, 2.0):
        selected = []
        for target_id, rows in groups.items():
            best = min(rows, key=lambda row: float(row["selection"]) + weight * float(row["intensity_median_abs_diff"]) - 0.1 * weight * float(row["intensity_correlation"]))
            target = targets[target_id]
            translation = math.hypot(float(best["x"]) - float(target["x"]), float(best["y"]) - float(target["y"]))
            yaw = angle_error(float(best["yaw_deg"]), float(target["yaw_deg"]))
            selected.append((translation, yaw))
            results.append({"weight": weight, "target_id": target_id, "cloud_index": target["cloud_index"], "rank": best["rank"], "translation_error_m": translation, "yaw_error_deg": yaw})
        summaries.append({
            "weight": weight,
            "targets": len(selected),
            "success_0p05_2deg": sum(t <= 0.05 and y <= 2.0 for t, y in selected),
            "success_0p10_3deg": sum(t <= 0.10 and y <= 3.0 for t, y in selected),
            "success_0p20_5deg": sum(t <= 0.20 and y <= 5.0 for t, y in selected),
            "median_translation_error_m": float(np.median([t for t, _ in selected])),
            "median_yaw_error_deg": float(np.median([y for _, y in selected])),
            "elapsed_sec": time.perf_counter() - start,
        })
    write(args.output_dir / "results.csv", results)
    write(args.output_dir / "summary.csv", summaries)
    for row in summaries:
        print(row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
