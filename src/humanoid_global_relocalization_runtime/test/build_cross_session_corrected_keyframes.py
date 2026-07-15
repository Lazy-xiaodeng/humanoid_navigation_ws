#!/usr/bin/env python3
"""Build locally corrected keyframe poses from held-in cross-session constraints."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import tempfile
import subprocess

import numpy as np

from run_cross_fov_full_validation import gicp, points_for_query, pose_from_matrix, pose_matrix
from run_solid_official_validation import descriptors, similarity, heading


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def angle(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def delta_pose(estimated: np.ndarray, original: np.ndarray) -> tuple[float, float, float]:
    delta = estimated @ np.linalg.inv(original)
    return pose_from_matrix(delta)


def apply_delta(row: dict[str, str], correction: tuple[float, float, float], scale: float) -> dict[str, object]:
    transform = pose_matrix(correction[0] * scale, correction[1] * scale, correction[2] * scale) @ pose_matrix(float(row["x"]), float(row["y"]), float(row["yaw_deg"]))
    x, y, yaw = pose_from_matrix(transform)
    return {**row, "x": x, "y": y, "yaw_deg": yaw}


def correction_field(rows: list[dict[str, str]], observations: list[tuple[float, float, float, float, float]], neighbors: int) -> dict[int, tuple[float, float, float]]:
    result = {}
    for row in rows:
        x, y = float(row["x"]), float(row["y"])
        local = sorted(observations, key=lambda item: math.hypot(item[0] - x, item[1] - y))[:neighbors]
        result[int(row["frame_id"])] = (
            float(np.median([item[2] for item in local])),
            float(np.median([item[3] for item in local])),
            float(np.median([item[4] for item in local])),
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--keyframes", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument("--neighbors", type=int, default=5)
    parser.add_argument("--max-prior-xy", type=float, default=1.0)
    parser.add_argument("--max-prior-yaw", type=float, default=15.0)
    args = parser.parse_args()
    rows = read(args.keyframes)
    session44 = [row for row in rows if row["source_bag"] == "44"]
    session45 = [row for row in rows if row["source_bag"] == "45"]
    descriptor_args = argparse.Namespace(max_range=20.0, solid_voxel=0.4, fov_down=-35.0, fov_up=35.0, sensor_height=1.215, min_range=0.8)
    desc44, desc45 = descriptors(session44, args.binary, descriptor_args), descriptors(session45, args.binary, descriptor_args)
    registration_args = argparse.Namespace(query_voxel=0.2, query_fov_deg=120.0, scan_min_z=0.2, scan_max_z=2.5, min_range=0.8, max_range=20.0, registration_voxel=0.2, max_correspondence=2.0, gicp_iterations=30)
    cache: dict[str, np.ndarray] = {}
    def points(row):
        if row["pcd"] not in cache:
            cache[row["pcd"]] = points_for_query(Path(row["pcd"]), registration_args)
        return cache[row["pcd"]]
    edges = []
    obs44, obs45 = [], []
    for index, source in enumerate(session44):
        scores = similarity(desc44[index], desc45)
        order = np.argsort(scores)[::-1][: args.top_k]
        source_pose = pose_matrix(float(source["x"]), float(source["y"]), float(source["yaw_deg"]))
        for target_index in order:
            target = session45[int(target_index)]
            target_pose = pose_matrix(float(target["x"]), float(target["y"]), float(target["yaw_deg"]))
            relative_yaw = heading(desc44[index], desc45[int(target_index)])
            registration = gicp(points(source), points(target), pose_matrix(0.0, 0.0, relative_yaw), registration_args)
            estimated_source = target_pose @ registration.transformation
            estimated_target = source_pose @ np.linalg.inv(registration.transformation)
            sx, sy, syaw = pose_from_matrix(estimated_source)
            prior_xy = math.hypot(sx - float(source["x"]), sy - float(source["y"]))
            prior_yaw = abs(angle(syaw - float(source["yaw_deg"])))
            accepted = float(scores[target_index]) >= 0.90 and registration.fitness >= 0.70 and registration.inlier_rmse <= 0.30 and prior_xy <= args.max_prior_xy and prior_yaw <= args.max_prior_yaw
            edge = {"source_frame": source["frame_id"], "target_frame": target["frame_id"], "similarity": float(scores[target_index]), "fitness": float(registration.fitness), "rmse": float(registration.inlier_rmse), "prior_xy": prior_xy, "prior_yaw": prior_yaw, "accepted": int(accepted)}
            edges.append(edge)
            if accepted:
                d44 = delta_pose(estimated_source, source_pose)
                d45 = delta_pose(estimated_target, target_pose)
                obs44.append((float(source["x"]), float(source["y"]), *d44))
                obs45.append((float(target["x"]), float(target["y"]), *d45))
        if (index + 1) % 100 == 0:
            print(f"[cross_session] {index + 1}/{len(session44)} accepted={len(obs44)}", flush=True)
    write(args.output_dir / "edges.csv", edges)
    field44, field45 = correction_field(session44, obs44, args.neighbors), correction_field(session45, obs45, args.neighbors)
    variants = {
        "fix45": [apply_delta(row, field44[int(row["frame_id"])], 1.0) if row["source_bag"] == "44" else row for row in rows],
        "fix44": [apply_delta(row, field45[int(row["frame_id"])], 1.0) if row["source_bag"] == "45" else row for row in rows],
        "half": [apply_delta(row, field44[int(row["frame_id"])], 0.5) if row["source_bag"] == "44" else apply_delta(row, field45[int(row["frame_id"])], 0.5) for row in rows],
    }
    for name, variant in variants.items():
        write(args.output_dir / f"keyframes_{name}.csv", variant)
    summary = [{"edges": len(edges), "accepted_edges": len(obs44), "median_44_xy_correction_m": float(np.median([math.hypot(item[2], item[3]) for item in obs44])), "median_44_yaw_correction_deg": float(np.median([abs(item[4]) for item in obs44]))}]
    write(args.output_dir / "summary.csv", summary)
    print(summary[0])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
