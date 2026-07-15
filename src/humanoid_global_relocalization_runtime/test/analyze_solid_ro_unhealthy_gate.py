#!/usr/bin/env python3
"""文件作用：验证 RO 不健康时 SOLiD/G-PROBE 候选的连续帧运动一致性门控。"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import resource
import time

import numpy as np

from run_cross_fov_full_validation import gicp, norm_deg, points_for_query, pose_matrix, write_csv


def load(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def yaw(matrix: np.ndarray) -> float:
    return math.degrees(math.atan2(float(matrix[1, 0]), float(matrix[0, 0])))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--centers", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--method", choices=["solid", "gprobe"], default="solid")
    parser.add_argument("--motion-xy-m", type=float, default=0.20)
    parser.add_argument("--motion-yaw-deg", type=float, default=3.0)
    args = parser.parse_args()

    metadata = {int(row["cloud_index"]): row for row in load(args.targets)}
    results = {int(row["cloud_index"]): row for row in load(args.results) if row["method"] == args.method}
    def gate(row: dict[str, str]) -> bool:
        if args.method == "solid":
            return float(row["similarity"]) >= 0.90 and float(row["rmse"]) <= 0.30 and float(row["fitness"]) >= 0.70
        return (
            float(row["front_score"]) >= 0.68
            and float(row["pdom"]) >= 0.999
            and 0.03 <= float(row["rmse"]) <= 0.25
            and float(row["fitness"]) >= 0.10
        )
    centers = [
        int(row["cloud_index"])
        for row in load(args.centers)
        if row["method"] == args.method and gate(row)
    ]
    cfg = argparse.Namespace(query_voxel=0.2, query_fov_deg=120.0, scan_min_z=0.2, scan_max_z=2.5, min_range=0.8, max_range=20.0, registration_voxel=0.2, max_correspondence=1.0, gicp_iterations=30)
    clouds = {index: points_for_query(Path(row["pcd"]), cfg) for index, row in metadata.items()}
    rows = []
    start = time.perf_counter()
    for center in centers:
        if center not in results or center not in clouds:
            continue
        center_result = results[center]
        x_key, y_key, yaw_key = (("x", "y", "yaw_deg") if args.method == "solid" else ("selected_x_m", "selected_y_m", "selected_yaw_deg"))
        center_pose = pose_matrix(float(center_result[x_key]), float(center_result[y_key]), float(center_result[yaw_key]))
        support = 0
        worst_xy = 0.0
        worst_yaw = 0.0
        for neighbor in (center - 1, center + 1):
            if neighbor not in results or neighbor not in clouds:
                continue
            item = results[neighbor]
            if not gate(item):
                continue
            neighbor_pose = pose_matrix(float(item[x_key]), float(item[y_key]), float(item[yaw_key]))
            candidate_relative = np.linalg.inv(center_pose) @ neighbor_pose
            registration = gicp(clouds[neighbor], clouds[center], np.eye(4), cfg)
            residual = np.linalg.inv(registration.transformation) @ candidate_relative
            residual_xy = math.hypot(float(residual[0, 3]), float(residual[1, 3]))
            residual_yaw = abs(norm_deg(yaw(residual)))
            if residual_xy <= args.motion_xy_m and residual_yaw <= args.motion_yaw_deg:
                support += 1
            worst_xy = max(worst_xy, residual_xy)
            worst_yaw = max(worst_yaw, residual_yaw)
        translation = float(center_result["translation_error_m"])
        yaw_error = float(center_result["yaw_error_deg"])
        rows.append({
            "cloud_index": center,
            "supporting_neighbors": support,
            "accept_2_frames": int(support >= 1),
            "accept_3_frames": int(support >= 2),
            "translation_error_m": translation,
            "yaw_error_deg": yaw_error,
            "success_0p2_5deg": int(translation <= 0.2 and yaw_error <= 5.0),
            "success_0p3_5deg": int(translation <= 0.3 and yaw_error <= 5.0),
            "worst_motion_residual_xy_m": worst_xy,
            "worst_motion_residual_yaw_deg": worst_yaw,
        })
    summary = []
    for frames, field in ((2, "accept_2_frames"), (3, "accept_3_frames")):
        accepted = [row for row in rows if row[field]]
        summary.append({
            "support_frames": frames,
            "centers": len(rows),
            "accepted": len(accepted),
            "accepted_success_0p2_5deg": sum(row["success_0p2_5deg"] for row in accepted),
            "accepted_success_0p3_5deg": sum(row["success_0p3_5deg"] for row in accepted),
            "false_accept_0p3_5deg": sum(not row["success_0p3_5deg"] for row in accepted),
            "elapsed_ms": (time.perf_counter() - start) * 1000.0,
            "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
        })
    write_csv(args.output_dir / "decisions.csv", rows)
    write_csv(args.output_dir / "summary.csv", summary)
    print(f"[solid_ro_unhealthy] wrote {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
