#!/usr/bin/env python3
"""Evaluate full keyframe-window relocalization with a propagated recovery prior."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

from analyze_keyframe_motion_prior import (
    angle,
    candidate_ok,
    expected_pose,
    fuse_local_candidates,
    pose,
    read,
    residual,
    write,
)


def prior_for(target: dict[str, str], distance: float, yaw_offset: float, direction_deg: float) -> np.ndarray:
    direction = math.radians(direction_deg)
    return pose(
        float(target["x"]) + distance * math.cos(direction),
        float(target["y"]) + distance * math.sin(direction),
        float(target["yaw_deg"]) + yaw_offset,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-candidate-rmse", type=float, default=0.30)
    parser.add_argument("--descriptor-anchor-margin", type=float, default=0.01)
    parser.add_argument("--support", type=int, default=3)
    parser.add_argument("--absolute-support", type=int, default=2)
    parser.add_argument("--cluster-xy", type=float, default=0.15)
    parser.add_argument("--cluster-yaw", type=float, default=3.0)
    parser.add_argument("--prior-radius", type=float, default=2.0)
    parser.add_argument("--prior-yaw", type=float, default=15.0)
    args = parser.parse_args()

    targets = read(args.targets)
    by_index = {int(row["cloud_index"]): row for row in targets}
    groups: dict[int, list[dict[str, str]]] = {}
    for row in targets:
        groups.setdefault(int(row["center_index"]), []).append(row)
    candidates: dict[int, list[dict[str, str]]] = {}
    for row in read(args.candidates):
        candidates.setdefault(int(row["cloud_index"]), []).append(row)

    cases = (
        ("recovery_0p2m_5deg", 0.2, 5.0),
        ("recovery_0p5m_5deg", 0.5, 5.0),
        ("recovery_0p5m_10deg", 0.5, 10.0),
        ("recovery_1p0m_10deg", 1.0, 10.0),
    )
    decisions: list[dict[str, object]] = []
    for case, prior_error, prior_yaw_error in cases:
        for direction in (0.0, 90.0, 180.0, 270.0):
            for center, window in sorted(groups.items()):
                center_target = by_index[center]
                recovery_prior = prior_for(center_target, prior_error, prior_yaw_error, direction)
                eligible = []
                for candidate in candidates.get(center, []):
                    if not candidate_ok(candidate, args.max_candidate_rmse):
                        continue
                    prior_xy, prior_yaw = residual(recovery_prior, candidate)
                    if prior_xy <= args.prior_radius and prior_yaw <= args.prior_yaw:
                        eligible.append(candidate)
                best_similarity = max((float(item["similarity"]) for item in eligible), default=-math.inf)
                anchors = [
                    item for item in eligible
                    if float(item["similarity"]) >= best_similarity - args.descriptor_anchor_margin
                ]
                ranked: list[tuple[int, float, dict[str, str]]] = []
                for anchor in anchors:
                    support = 1
                    absolute_support = 1
                    cost = 0.0
                    for neighbor in window:
                        neighbor_index = int(neighbor["cloud_index"])
                        if neighbor_index == center:
                            continue
                        expected = expected_pose("fastlio_odom", anchor, center_target, neighbor)
                        matches = []
                        absolute_matches = []
                        for item in candidates.get(neighbor_index, []):
                            if not candidate_ok(item, args.max_candidate_rmse):
                                continue
                            xy, yaw = residual(expected, item)
                            if xy <= args.cluster_xy and yaw <= args.cluster_yaw:
                                matches.append((xy + 0.02 * yaw, item))
                            absolute_xy, absolute_yaw = residual(
                                expected_pose("none", anchor, center_target, neighbor), item
                            )
                            if absolute_xy <= args.cluster_xy and absolute_yaw <= args.cluster_yaw:
                                absolute_matches.append((absolute_xy + 0.02 * absolute_yaw, item))
                        if matches:
                            support += 1
                            cost += min(matches, key=lambda pair: pair[0])[0]
                        if absolute_matches:
                            absolute_support += 1
                    if absolute_support >= args.absolute_support:
                        ranked.append((support, cost - float(anchor["similarity"]), anchor))
                if not ranked:
                    decisions.append({"case": case, "direction_deg": direction, "center_index": center, "accepted": 0, "support": 0})
                    continue
                support, _, selected = min(ranked, key=lambda item: (-item[0], item[1]))
                x, y, yaw, fused = fuse_local_candidates(selected, candidates[center], args.max_candidate_rmse)
                translation = math.hypot(x - float(center_target["x"]), y - float(center_target["y"]))
                yaw_error = abs(angle(yaw - float(center_target["yaw_deg"])))
                accepted = support >= args.support
                decisions.append({
                    "case": case,
                    "direction_deg": direction,
                    "center_index": center,
                    "accepted": int(accepted),
                    "support": support,
                    "fused_candidates": fused,
                    "translation_error_m": translation,
                    "yaw_error_deg": yaw_error,
                    "success_0p05_2deg": int(translation <= 0.05 and yaw_error <= 2.0),
                    "success_0p10_3deg": int(translation <= 0.10 and yaw_error <= 3.0),
                    "success_0p20_5deg": int(translation <= 0.20 and yaw_error <= 5.0),
                })

    summary = []
    for case, _, _ in cases:
        selected = [row for row in decisions if row["case"] == case and int(row["accepted"])]
        trials = len(groups) * 4
        summary.append({
            "case": case,
            "trials": trials,
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
