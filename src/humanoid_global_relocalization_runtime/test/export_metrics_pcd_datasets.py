#!/usr/bin/env python3
"""文件作用：把评估器选中的 bag 帧导出为 base 坐标系 PCD 数据集。"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import open3d as o3d

from run_scan_context_keyframe_recall import (
    Target,
    nearest_odom,
    nearest_reference,
    quaternion_to_rotation_matrix,
    read_bag_samples,
    registered_world_cloud_to_body,
    yaw_from_quaternion,
)


def fastlio_odom_to_base_pose(msg: object) -> tuple[float, float, float]:
    """Match fastlio_open3d_axis_adapter's raw camera_init/body conversion."""
    raw_rotation = quaternion_to_rotation_matrix(msg.pose.pose.orientation)
    raw_translation = np.array(
        [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
        dtype=np.float64,
    )
    raw_to_std = np.array(
        [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64
    )
    body_to_base_rotation_raw = np.array(
        [[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]], dtype=np.float64
    )
    body_to_base_translation_raw = np.array([0.004, 1.215, 0.072], dtype=np.float64)
    base_translation_raw = raw_translation + raw_rotation @ body_to_base_translation_raw
    base_translation_std = raw_to_std @ (base_translation_raw - body_to_base_translation_raw)
    base_rotation_std = raw_to_std @ raw_rotation @ body_to_base_rotation_raw
    return (
        float(base_translation_std[0]),
        float(base_translation_std[1]),
        math.degrees(math.atan2(float(base_rotation_std[1, 0]), float(base_rotation_std[0, 0]))),
    )


def read_group(path: Path, scenario: str) -> list[Target]:
    targets: list[Target] = []
    seen: set[int] = set()
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row["scenario_name"] != scenario:
                continue
            cloud_index = int(row["bag_frame_index"])
            if cloud_index in seen:
                continue
            seen.add(cloud_index)
            targets.append(
                Target(
                    waypoint_id=f"metric_{len(targets) + 1:03d}",
                    name=f"{path.parent.name}_{cloud_index}",
                    cloud_index=cloud_index,
                    reference_x=float(row["reference_x_m"]),
                    reference_y=float(row["reference_y_m"]),
                    reference_yaw_deg=float(row["reference_yaw_deg"]),
                )
            )
    return targets


def read_matches(path: Path) -> list[Target]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [
            Target(
                row["waypoint_id"],
                row["name"],
                int(row["cloud_index"]),
                float(row["reference_x"]),
                float(row["reference_y"]),
                float(row["reference_yaw_deg"]),
            )
            for row in csv.DictReader(stream)
        ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scenario", default="arbitrary_start_no_prior")
    parser.add_argument("--window-results", nargs="*", type=Path, default=[])
    parser.add_argument("--solid-window-results", nargs="*", type=Path, default=[])
    parser.add_argument("--window-radius", type=int, default=1)
    parser.add_argument("--matches", nargs="*", type=Path, default=[])
    parser.add_argument(
        "--matches-window-radius",
        type=int,
        default=0,
        help="also export neighboring cloud indices around each matched waypoint",
    )
    parser.add_argument("--keyframe-stride", type=int, default=0)
    parser.add_argument("metrics", nargs="*", type=Path)
    args = parser.parse_args()

    groups = {path.parent.name: read_group(path, args.scenario) for path in args.metrics}
    centers: dict[str, dict[int, int]] = {}
    for path in args.matches:
        label = path.parent.name
        matches = read_matches(path)
        if args.matches_window_radius <= 0:
            groups[label] = matches
            continue
        center_map: dict[int, int] = {}
        for match in matches:
            for offset in range(-args.matches_window_radius, args.matches_window_radius + 1):
                center_map.setdefault(match.cloud_index + offset, match.cloud_index)
        centers[label] = center_map
        groups[label] = [
            Target(f"window_{index:04d}", f"center_{center}", cloud_index, 0.0, 0.0, 0.0)
            for index, (cloud_index, center) in enumerate(sorted(center_map.items()), start=1)
        ]
    for path in args.window_results:
        label = path.parent.name
        center_map: dict[int, int] = {}
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                if row["method"] != "gprobe":
                    continue
                if not (
                    float(row["front_score"]) >= 0.68
                    and float(row["pdom"]) >= 0.995
                    and float(row["rmse"]) <= 0.30
                ):
                    continue
                center = int(row["cloud_index"])
                for offset in range(-args.window_radius, args.window_radius + 1):
                    center_map.setdefault(center + offset, center)
        centers[label] = center_map
        groups[label] = [
            Target(f"window_{index:04d}", f"center_{center}", cloud_index, 0.0, 0.0, 0.0)
            for index, (cloud_index, center) in enumerate(sorted(center_map.items()), start=1)
        ]
    for path in args.solid_window_results:
        label = path.parent.name
        center_map: dict[int, int] = {}
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                if row["method"] != "solid":
                    continue
                if not (
                    float(row["similarity"]) >= 0.90
                    and float(row["rmse"]) <= 0.30
                    and float(row["fitness"]) >= 0.70
                ):
                    continue
                center = int(row["cloud_index"])
                for offset in range(-args.window_radius, args.window_radius + 1):
                    center_map.setdefault(center + offset, center)
        centers[label] = center_map
        groups[label] = [
            Target(f"window_{index:04d}", f"center_{center}", cloud_index, 0.0, 0.0, 0.0)
            for index, (cloud_index, center) in enumerate(sorted(center_map.items()), start=1)
        ]
    if not groups and args.keyframe_stride <= 0:
        parser.error("provide metrics paths or --window-results")
    all_targets = {target.cloud_index: target for targets in groups.values() for target in targets}
    stride = args.keyframe_stride if args.keyframe_stride > 0 else 10**9
    odoms, references, clouds, keyframes = read_bag_samples(args.bag, list(all_targets.values()), stride)
    if set(clouds) != set(all_targets):
        raise RuntimeError(f"missing clouds: {sorted(set(all_targets) - set(clouds))}")

    for label, targets in groups.items():
        group_dir = args.output_dir / label
        cloud_dir = group_dir / "targets"
        cloud_dir.mkdir(parents=True, exist_ok=True)
        rows: list[dict[str, object]] = []
        for target in targets:
            sample = clouds[target.cloud_index]
            odom = nearest_odom(odoms, sample.stamp_sec)
            reference = nearest_reference(references, sample.bag_time_sec)
            odom_base_x, odom_base_y, odom_base_yaw_deg = fastlio_odom_to_base_pose(odom.msg)
            points = registered_world_cloud_to_body(sample, odom)
            pcd = cloud_dir / f"{target.waypoint_id}_{target.cloud_index}.pcd"
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            if not o3d.io.write_point_cloud(str(pcd), cloud, write_ascii=False, compressed=True):
                raise RuntimeError(f"failed to write {pcd}")
            rows.append({
                "target_id": target.waypoint_id,
                "cloud_index": target.cloud_index,
                "x": reference.x if label in centers else target.reference_x,
                "y": reference.y if label in centers else target.reference_y,
                "z": 0.0,
                "yaw_deg": reference.yaw_deg if label in centers else target.reference_yaw_deg,
                "pcd": pcd.resolve(),
                "center_index": centers.get(label, {}).get(target.cloud_index, target.cloud_index),
                "offset": target.cloud_index - centers.get(label, {}).get(target.cloud_index, target.cloud_index),
                "stamp_sec": sample.stamp_sec,
                "odom_x": float(odom.msg.pose.pose.position.x),
                "odom_y": float(odom.msg.pose.pose.position.y),
                "odom_yaw_deg": math.degrees(yaw_from_quaternion(odom.msg.pose.pose.orientation)),
                "odom_base_x": odom_base_x,
                "odom_base_y": odom_base_y,
                "odom_base_yaw_deg": odom_base_yaw_deg,
            })
        with (group_dir / "targets.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"[export_metrics_pcd] {label}: {len(rows)} targets")
    if args.keyframe_stride > 0:
        keyframe_dir = args.output_dir / "keyframes"
        keyframe_dir.mkdir(parents=True, exist_ok=True)
        rows = []
        for frame_id, sample in enumerate(keyframes):
            odom = nearest_odom(odoms, sample.stamp_sec)
            reference = nearest_reference(references, sample.bag_time_sec)
            points = registered_world_cloud_to_body(sample, odom)
            pcd = keyframe_dir / f"{frame_id:06d}.pcd"
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            if not o3d.io.write_point_cloud(str(pcd), cloud, write_ascii=False, compressed=True):
                raise RuntimeError(f"failed to write {pcd}")
            rows.append({
                "frame_id": frame_id,
                "cloud_index": sample.cloud_index,
                "x": reference.x,
                "y": reference.y,
                "z": 0.0,
                "yaw_deg": reference.yaw_deg,
                "pcd": pcd.resolve(),
            })
        with (args.output_dir / "keyframes.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"[export_metrics_pcd] keyframes: {len(rows)} stride={args.keyframe_stride}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
