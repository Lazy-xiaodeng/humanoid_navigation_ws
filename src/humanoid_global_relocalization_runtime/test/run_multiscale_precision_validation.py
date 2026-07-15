#!/usr/bin/env python3
"""Validate conservative multi-frame, multi-scale point-to-plane refinement."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import resource
import time

import numpy as np
import open3d as o3d


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def angle_delta(lhs: float, rhs: float) -> float:
    return abs((lhs - rhs + 180.0) % 360.0 - 180.0)


def pose(x: float, y: float, yaw_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:2, :2] = ((c, -s), (s, c))
    result[0, 3], result[1, 3] = x, y
    return result


def pose_values(transform: np.ndarray) -> tuple[float, float, float]:
    return (
        float(transform[0, 3]),
        float(transform[1, 3]),
        math.degrees(math.atan2(float(transform[1, 0]), float(transform[0, 0]))),
    )


def project_2d(transform: np.ndarray) -> np.ndarray:
    x, y, yaw = pose_values(transform)
    return pose(x, y, yaw)


def cloud(path: str, voxel: float = 0.0) -> o3d.geometry.PointCloud:
    result = o3d.io.read_point_cloud(path)
    if voxel > 0.0:
        result = result.voxel_down_sample(voxel)
    return result


def crop_map(points: np.ndarray, x: float, y: float, radius: float) -> o3d.geometry.PointCloud:
    selected = points[
        (np.abs(points[:, 0] - x) <= radius)
        & (np.abs(points[:, 1] - y) <= radius)
        & (points[:, 2] >= 0.2)
        & (points[:, 2] <= 2.5)
    ]
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(selected)
    return result


def candidate_gate(row: dict[str, str]) -> bool:
    return (
        float(row["similarity"]) >= 0.90
        and float(row["fitness"]) >= 0.70
        and float(row["rmse"]) <= 0.30
    )


def best_cluster(items: list[tuple[dict[str, str], dict[str, str]]]) -> list[tuple[dict[str, str], dict[str, str]]]:
    eligible = [item for item in items if candidate_gate(item[1])]
    best: list[tuple[dict[str, str], dict[str, str]]] = []
    for _, anchor in eligible:
        cluster = [
            item
            for item in eligible
            if math.hypot(float(item[1]["x"]) - float(anchor["x"]), float(item[1]["y"]) - float(anchor["y"])) <= 0.15
            and angle_delta(float(item[1]["yaw_deg"]), float(anchor["yaw_deg"])) <= 3.0
        ]
        if len(cluster) > len(best):
            best = cluster
    return best


def medoid(items: list[tuple[dict[str, str], dict[str, str]]]) -> tuple[dict[str, str], dict[str, str]]:
    def cost(item: tuple[dict[str, str], dict[str, str]]) -> float:
        row = item[1]
        return sum(
            math.hypot(float(row["x"]) - float(other[1]["x"]), float(row["y"]) - float(other[1]["y"]))
            + 0.02 * angle_delta(float(row["yaw_deg"]), float(other[1]["yaw_deg"]))
            for other in items
        )
    return min(items, key=cost)


def accumulated_cloud(
    cluster: list[tuple[dict[str, str], dict[str, str]]], center_pose: np.ndarray
) -> o3d.geometry.PointCloud:
    merged = o3d.geometry.PointCloud()
    center_inverse = np.linalg.inv(center_pose)
    for target, result in cluster:
        scan = cloud(target["pcd"], 0.03)
        scan.transform(center_inverse @ pose(float(result["x"]), float(result["y"]), float(result["yaw_deg"])))
        merged += scan
    return merged.voxel_down_sample(0.03)


def refine(
    source: o3d.geometry.PointCloud,
    map_points: np.ndarray,
    initial: np.ndarray,
    radius: float,
) -> tuple[np.ndarray, float, float, float, int]:
    x, y, _ = pose_values(initial)
    local = crop_map(map_points, x, y, radius)
    transform = initial.copy()
    result = None
    stages = ((0.15, 0.50, 25), (0.10, 0.25, 30), (0.05, 0.12, 40))
    for voxel, correspondence, iterations in stages:
        source_level = source.voxel_down_sample(voxel)
        target_level = local.voxel_down_sample(voxel)
        target_level.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 4.0, max_nn=40))
        loss = o3d.pipelines.registration.TukeyLoss(k=correspondence * 0.5)
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
        result = o3d.pipelines.registration.registration_icp(
            source_level,
            target_level,
            correspondence,
            transform,
            estimation,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iterations),
        )
        transform = project_2d(result.transformation)
    assert result is not None
    source_fine = source.voxel_down_sample(0.05)
    target_fine = local.voxel_down_sample(0.05)
    information = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source_fine, target_fine, 0.12, transform
    )
    eigenvalues = np.linalg.eigvalsh(np.asarray(information))
    positive = eigenvalues[eigenvalues > 1e-9]
    condition = float(positive[-1] / positive[0]) if len(positive) == 6 else math.inf
    evaluation = o3d.pipelines.registration.evaluate_registration(source_fine, target_fine, 0.12, transform)
    return transform, float(evaluation.fitness), float(evaluation.inlier_rmse), condition, len(source_fine.points)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--min-support", type=int, default=3)
    parser.add_argument("--local-map-radius", type=float, default=20.0)
    parser.add_argument("--max-pose-change", type=float, default=0.20)
    parser.add_argument("--max-yaw-change", type=float, default=3.0)
    parser.add_argument("--min-fitness", type=float, default=0.65)
    parser.add_argument("--max-rmse", type=float, default=0.08)
    parser.add_argument("--max-condition", type=float, default=1.0e6)
    args = parser.parse_args()

    targets = read_csv(args.targets)
    candidates = {int(row["cloud_index"]): row for row in read_csv(args.candidates)}
    groups: dict[int, list[tuple[dict[str, str], dict[str, str]]]] = {}
    for target in targets:
        index = int(target["cloud_index"])
        if index in candidates:
            groups.setdefault(int(target["center_index"]), []).append((target, candidates[index]))
    map_points = np.asarray(o3d.io.read_point_cloud(str(args.map)).points)

    rows: list[dict[str, object]] = []
    for number, (center, items) in enumerate(sorted(groups.items()), start=1):
        begin = time.perf_counter()
        cluster = best_cluster(items)
        if len(cluster) < args.min_support:
            rows.append({"center_index": center, "support": len(cluster), "accepted": 0, "reason": "support"})
            continue
        center_target = next(target for target, _ in items if int(target["cloud_index"]) == center)
        center_items = [item for item in cluster if int(item[0]["cloud_index"]) == center]
        if not center_items:
            rows.append({"center_index": center, "support": len(cluster), "accepted": 0, "reason": "center_not_supported"})
            continue
        _, seed = center_items[0]
        initial = pose(float(seed["x"]), float(seed["y"]), float(seed["yaw_deg"]))
        source = accumulated_cloud(cluster, initial)
        refined, fitness, rmse, condition, points = refine(source, map_points, initial, args.local_map_radius)
        final_x, final_y, final_yaw = pose_values(refined)
        initial_x, initial_y, initial_yaw = pose_values(initial)
        change = math.hypot(final_x - initial_x, final_y - initial_y)
        yaw_change = angle_delta(final_yaw, initial_yaw)
        error = math.hypot(final_x - float(center_target["x"]), final_y - float(center_target["y"]))
        yaw_error = angle_delta(final_yaw, float(center_target["yaw_deg"]))
        initial_error = math.hypot(initial_x - float(center_target["x"]), initial_y - float(center_target["y"]))
        initial_yaw_error = angle_delta(initial_yaw, float(center_target["yaw_deg"]))
        accepted = (
            fitness >= args.min_fitness
            and rmse <= args.max_rmse
            and condition <= args.max_condition
            and change <= args.max_pose_change
            and yaw_change <= args.max_yaw_change
        )
        rows.append({
            "center_index": center,
            "support": len(cluster),
            "accepted": int(accepted),
            "reason": "accepted" if accepted else "precision_gate",
            "translation_error_m": error,
            "yaw_error_deg": yaw_error,
            "initial_translation_error_m": initial_error,
            "initial_yaw_error_deg": initial_yaw_error,
            "success_0p05_2deg": int(error <= 0.05 and yaw_error <= 2.0),
            "success_0p10_3deg": int(error <= 0.10 and yaw_error <= 3.0),
            "success_0p20_5deg": int(error <= 0.20 and yaw_error <= 5.0),
            "pose_change_m": change,
            "yaw_change_deg": yaw_change,
            "fitness": fitness,
            "rmse": rmse,
            "information_condition": condition,
            "source_points": points,
            "elapsed_ms": (time.perf_counter() - begin) * 1000.0,
            "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
        })
        print(f"[precision] {number}/{len(groups)} center={center} support={len(cluster)} accepted={int(accepted)} err={error:.3f}m/{yaw_error:.2f}deg", flush=True)

    accepted = [row for row in rows if int(row["accepted"])]
    summary = [{
        "groups": len(groups),
        "refined": sum("translation_error_m" in row for row in rows),
        "accepted": len(accepted),
        "success_0p05_2deg": sum(int(row["success_0p05_2deg"]) for row in accepted),
        "success_0p10_3deg": sum(int(row["success_0p10_3deg"]) for row in accepted),
        "success_0p20_5deg": sum(int(row["success_0p20_5deg"]) for row in accepted),
        "false_accept_0p20_5deg": sum(not int(row["success_0p20_5deg"]) for row in accepted),
        "median_translation_error_m": float(np.median([row["translation_error_m"] for row in accepted])) if accepted else math.nan,
        "max_translation_error_m": max((float(row["translation_error_m"]) for row in accepted), default=math.nan),
        "median_elapsed_ms": float(np.median([row["elapsed_ms"] for row in accepted])) if accepted else math.nan,
        "peak_rss_mb": max((float(row.get("peak_rss_mb", 0.0)) for row in rows), default=0.0),
    }]
    write_csv(args.output_dir / "results.csv", rows)
    write_csv(args.output_dir / "summary.csv", summary)
    print(summary[0])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
