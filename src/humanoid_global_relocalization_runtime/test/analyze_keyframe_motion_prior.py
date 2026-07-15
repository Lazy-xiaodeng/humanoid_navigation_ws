#!/usr/bin/env python3
"""Compare no-motion, inertial-yaw and Fast-LIO motion priors for keyframe candidates."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def read(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def angle(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def pose(x: float, y: float, yaw_deg: float) -> np.ndarray:
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    result = np.eye(3)
    result[:2, :2] = ((c, -s), (s, c))
    result[:2, 2] = (x, y)
    return result


def values(transform: np.ndarray) -> tuple[float, float, float]:
    return float(transform[0, 2]), float(transform[1, 2]), math.degrees(math.atan2(transform[1, 0], transform[0, 0]))


def candidate_ok(row: dict[str, str], max_rmse: float) -> bool:
    return float(row["similarity"]) >= 0.90 and float(row["fitness"]) >= 0.70 and float(row["rmse"]) <= max_rmse


def residual(expected: np.ndarray, row: dict[str, str]) -> tuple[float, float]:
    x, y, yaw = values(expected)
    return math.hypot(float(row["x"]) - x, float(row["y"]) - y), abs(angle(float(row["yaw_deg"]) - yaw))


def fuse_local_candidates(
    selected: dict[str, str], candidates: list[dict[str, str]], max_rmse: float
) -> tuple[float, float, float, int]:
    local = []
    for item in candidates:
        if not candidate_ok(item, max_rmse):
            continue
        if math.hypot(float(item["x"]) - float(selected["x"]), float(item["y"]) - float(selected["y"])) > 0.15:
            continue
        if abs(angle(float(item["yaw_deg"]) - float(selected["yaw_deg"]))) > 5.0:
            continue
        weight = math.exp(8.0 * float(item["similarity"]) - 4.0 * float(item["rmse"]))
        local.append((item, weight))
    total = sum(weight for _, weight in local)
    x = sum(float(item["x"]) * weight for item, weight in local) / total
    y = sum(float(item["y"]) * weight for item, weight in local) / total
    sin_yaw = sum(math.sin(math.radians(float(item["yaw_deg"]))) * weight for item, weight in local)
    cos_yaw = sum(math.cos(math.radians(float(item["yaw_deg"]))) * weight for item, weight in local)
    return x, y, math.degrees(math.atan2(sin_yaw, cos_yaw)), len(local)


def expected_pose(
    mode: str,
    center_candidate: dict[str, str],
    center_target: dict[str, str],
    neighbor_target: dict[str, str],
) -> np.ndarray:
    center_world = pose(float(center_candidate["x"]), float(center_candidate["y"]), float(center_candidate["yaw_deg"]))
    if mode == "none":
        return center_world
    yaw_delta = angle(float(neighbor_target["odom_base_yaw_deg"]) - float(center_target["odom_base_yaw_deg"]))
    if mode == "imu_yaw":
        x, y, yaw = values(center_world)
        return pose(x, y, yaw + yaw_delta)
    center_odom = pose(float(center_target["odom_base_x"]), float(center_target["odom_base_y"]), float(center_target["odom_base_yaw_deg"]))
    neighbor_odom = pose(float(neighbor_target["odom_base_x"]), float(neighbor_target["odom_base_y"]), float(neighbor_target["odom_base_yaw_deg"]))
    return center_world @ np.linalg.inv(center_odom) @ neighbor_odom


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--support", type=int, default=3)
    parser.add_argument("--xy-threshold", type=float, default=0.15)
    parser.add_argument("--yaw-threshold", type=float, default=3.0)
    parser.add_argument("--max-candidate-rmse", type=float, default=0.30)
    parser.add_argument("--descriptor-anchor-margin", type=float, default=0.01)
    args = parser.parse_args()

    targets = read(args.targets)
    target_by_index = {int(row["cloud_index"]): row for row in targets}
    groups: dict[int, list[dict[str, str]]] = {}
    for row in targets:
        groups.setdefault(int(row["center_index"]), []).append(row)
    candidates: dict[int, list[dict[str, str]]] = {}
    for row in read(args.candidates):
        candidates.setdefault(int(row["cloud_index"]), []).append(row)

    decisions: list[dict[str, object]] = []
    modes = ("none", "imu_yaw", "fastlio_odom", "hybrid_imu_yaw", "hybrid_fastlio_odom")
    for mode in modes:
        for center, window in sorted(groups.items()):
            center_target = target_by_index[center]
            ranked: list[tuple[int, float, dict[str, str]]] = []
            eligible_center = [
                candidate for candidate in candidates.get(center, [])
                if candidate_ok(candidate, args.max_candidate_rmse)
            ]
            best_similarity = max((float(candidate["similarity"]) for candidate in eligible_center), default=-math.inf)
            for candidate in candidates.get(center, []):
                if not candidate_ok(candidate, args.max_candidate_rmse):
                    continue
                if float(candidate["similarity"]) < best_similarity - args.descriptor_anchor_margin:
                    continue
                supporting_frames = 1
                total_residual = 0.0
                absolute_support = 1
                for neighbor in window:
                    neighbor_index = int(neighbor["cloud_index"])
                    if neighbor_index == center:
                        continue
                    prior_mode = mode.removeprefix("hybrid_")
                    expected = expected_pose(prior_mode, candidate, center_target, neighbor)
                    matches = []
                    absolute_matches = []
                    for item in candidates.get(neighbor_index, []):
                        if not candidate_ok(item, args.max_candidate_rmse):
                            continue
                        xy, yaw = residual(expected, item)
                        if xy <= args.xy_threshold and yaw <= args.yaw_threshold:
                            matches.append((xy + 0.02 * yaw, item))
                        absolute_xy, absolute_yaw = residual(
                            expected_pose("none", candidate, center_target, neighbor), item
                        )
                        if absolute_xy <= args.xy_threshold and absolute_yaw <= args.yaw_threshold:
                            absolute_matches.append((absolute_xy + 0.02 * absolute_yaw, item))
                    if matches:
                        supporting_frames += 1
                        total_residual += min(matches, key=lambda item: item[0])[0]
                    if absolute_matches:
                        absolute_support += 1
                if mode.startswith("hybrid_") and absolute_support < args.support:
                    continue
                selection = total_residual - float(candidate["similarity"]) + 0.2 * float(candidate["rmse"])
                ranked.append((supporting_frames, selection, candidate))
            if not ranked:
                decisions.append({"mode": mode, "center_index": center, "accepted": 0, "support": 0, "reason": "no_candidate"})
                continue
            support, _, selected = min(ranked, key=lambda item: (-item[0], item[1]))
            final_x, final_y, final_yaw, fused_candidates = fuse_local_candidates(
                selected, candidates.get(center, []), args.max_candidate_rmse
            )
            translation = math.hypot(final_x - float(center_target["x"]), final_y - float(center_target["y"]))
            yaw_error = abs(angle(final_yaw - float(center_target["yaw_deg"])))
            accepted = support >= args.support
            decisions.append({
                "mode": mode,
                "center_index": center,
                "accepted": int(accepted),
                "support": support,
                "reason": "accepted" if accepted else "support",
                "rank": selected["rank"],
                "keyframe_index": selected["keyframe_index"],
                "fused_candidates": fused_candidates,
                "translation_error_m": translation,
                "yaw_error_deg": yaw_error,
                "success_0p05_2deg": int(translation <= 0.05 and yaw_error <= 2.0),
                "success_0p10_3deg": int(translation <= 0.10 and yaw_error <= 3.0),
                "success_0p20_5deg": int(translation <= 0.20 and yaw_error <= 5.0),
            })

    summary: list[dict[str, object]] = []
    for mode in modes:
        selected = [row for row in decisions if row["mode"] == mode and int(row["accepted"])]
        summary.append({
            "mode": mode,
            "groups": len(groups),
            "accepted": len(selected),
            "success_0p05_2deg": sum(int(row["success_0p05_2deg"]) for row in selected),
            "success_0p10_3deg": sum(int(row["success_0p10_3deg"]) for row in selected),
            "success_0p20_5deg": sum(int(row["success_0p20_5deg"]) for row in selected),
            "false_accept_0p20_5deg": sum(not int(row["success_0p20_5deg"]) for row in selected),
            "median_translation_error_m": float(np.median([row["translation_error_m"] for row in selected])) if selected else math.nan,
            "max_translation_error_m": max((float(row["translation_error_m"]) for row in selected), default=math.nan),
        })
    write(args.output_dir / "decisions.csv", decisions)
    write(args.output_dir / "summary.csv", summary)
    for row in summary:
        print(row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
