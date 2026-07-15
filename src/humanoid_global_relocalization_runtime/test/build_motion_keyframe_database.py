#!/usr/bin/env python3
"""Build a multi-session keyframe database using spatial and heading change."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path

import open3d as o3d
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from run_scan_context_keyframe_recall import (
    CloudSample,
    OdomSample,
    ReferenceSample,
    open_reader,
    registered_world_cloud_to_body,
    stamp_to_sec,
    yaw_from_quaternion,
)


def angle_delta(lhs: float, rhs: float) -> float:
    return abs((lhs - rhs + 180.0) % 360.0 - 180.0)


def nearest(items: list, keys: list[float], value: float):
    index = bisect.bisect_left(keys, value)
    choices: list[int] = []
    if index < len(items):
        choices.append(index)
    if index > 0:
        choices.append(index - 1)
    return items[min(choices, key=lambda item_index: abs(keys[item_index] - value))]


def index_bag(bag: Path) -> tuple[list[OdomSample], list[ReferenceSample], list[tuple[int, float, float]]]:
    odom_type = get_message("nav_msgs/msg/Odometry")
    reference_type = get_message("geometry_msgs/msg/PoseWithCovarianceStamped")
    odoms, references, clouds = [], [], []
    reader = open_reader(bag)
    cloud_index = -1
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        bag_time = bag_time_ns * 1e-9
        if topic == "/odom":
            msg = deserialize_message(data, odom_type)
            odoms.append(OdomSample(stamp_to_sec(msg.header.stamp), msg))
        elif topic == "/robot_realpose":
            msg = deserialize_message(data, reference_type)
            p = msg.pose.pose
            references.append(ReferenceSample(bag_time, float(p.position.x), float(p.position.y), math.degrees(yaw_from_quaternion(p.orientation))))
        elif topic == "/fast_lio/cloud_registered":
            cloud_index += 1
            msg = deserialize_message(data, get_message("sensor_msgs/msg/PointCloud2"))
            clouds.append((cloud_index, stamp_to_sec(msg.header.stamp), bag_time))
    return odoms, references, clouds


def select_indices(
    references: list[ReferenceSample],
    clouds: list[tuple[int, float, float]],
    distance: float,
    yaw_deg: float,
    min_index_gap: int,
) -> dict[int, ReferenceSample]:
    ref_times = [item.bag_time_sec for item in references]
    selected: dict[int, ReferenceSample] = {}
    last = None
    last_index = -min_index_gap
    for index, _, bag_time in clouds:
        ref = nearest(references, ref_times, bag_time)
        if abs(ref.bag_time_sec - bag_time) > 0.05:
            continue
        changed = last is None or math.hypot(ref.x - last.x, ref.y - last.y) >= distance or angle_delta(ref.yaw_deg, last.yaw_deg) >= yaw_deg
        if changed and index - last_index >= min_index_gap:
            selected[index] = ref
            last, last_index = ref, index
    return selected


def export_selected(
    bag: Path,
    session: str,
    output: Path,
    odoms: list[OdomSample],
    selected: dict[int, ReferenceSample],
    frame_offset: int,
) -> list[dict[str, object]]:
    point_type = get_message("sensor_msgs/msg/PointCloud2")
    odom_stamps = [item.stamp_sec for item in odoms]
    rows = []
    reader = open_reader(bag)
    cloud_index = -1
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        if topic != "/fast_lio/cloud_registered":
            continue
        cloud_index += 1
        if cloud_index not in selected:
            continue
        msg = deserialize_message(data, point_type)
        stamp = stamp_to_sec(msg.header.stamp)
        odom = nearest(odoms, odom_stamps, stamp)
        sample = CloudSample(cloud_index, stamp, bag_time_ns * 1e-9, msg)
        points = registered_world_cloud_to_body(sample, odom)
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        pcd = output / "keyframes" / f"{frame_offset + len(rows):06d}.pcd"
        pcd.parent.mkdir(parents=True, exist_ok=True)
        if not o3d.io.write_point_cloud(str(pcd), cloud, compressed=True):
            raise RuntimeError(f"failed to write {pcd}")
        ref = selected[cloud_index]
        rows.append({
            "frame_id": frame_offset + len(rows),
            "cloud_index": cloud_index,
            "x": ref.x,
            "y": ref.y,
            "z": 0.0,
            "yaw_deg": ref.yaw_deg,
            "pcd": pcd.resolve(),
            "source_bag": session,
        })
        if len(rows) % 100 == 0:
            print(f"[motion_db] {session} exported={len(rows)}/{len(selected)}", flush=True)
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bags", nargs="+", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--distance", type=float, default=0.40)
    parser.add_argument("--yaw-deg", type=float, default=12.0)
    parser.add_argument("--min-index-gap", type=int, default=3)
    args = parser.parse_args()
    all_rows = []
    for bag in args.bags:
        session = bag.name.removeprefix("nav_drift_test")
        odoms, references, clouds = index_bag(bag)
        selected = select_indices(references, clouds, args.distance, args.yaw_deg, args.min_index_gap)
        print(f"[motion_db] {session} clouds={len(clouds)} selected={len(selected)}", flush=True)
        all_rows.extend(export_selected(bag, session, args.output_dir, odoms, selected, len(all_rows)))
    manifest = args.output_dir / "keyframes.csv"
    with manifest.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    print(f"[motion_db] wrote {manifest} keyframes={len(all_rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
