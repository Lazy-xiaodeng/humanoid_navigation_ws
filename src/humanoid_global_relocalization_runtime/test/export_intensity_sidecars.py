#!/usr/bin/env python3
"""Export transformed XYZI arrays for selected bag frames without changing PCDs."""

from __future__ import annotations

import argparse
import bisect
import csv
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sensor_msgs_py.point_cloud2 as pc2

from run_scan_context_keyframe_recall import open_reader, quaternion_to_rotation_matrix, stamp_to_sec


def read_manifest(path: Path, source_bag: str) -> dict[int, str]:
    result = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("source_bag") and row["source_bag"] != source_bag:
                continue
            result[int(row["cloud_index"])] = row.get("frame_id", row.get("target_id", row["cloud_index"]))
    return result


def load_odoms(bag: Path):
    odom_type = get_message("nav_msgs/msg/Odometry")
    result = []
    reader = open_reader(bag)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/odom":
            msg = deserialize_message(data, odom_type)
            result.append((stamp_to_sec(msg.header.stamp), msg.pose.pose))
    return result


def nearest_odom(odoms, stamps, stamp):
    index = bisect.bisect_left(stamps, stamp)
    choices = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(odoms)]
    return odoms[min(choices, key=lambda candidate: abs(stamps[candidate] - stamp))]


def transform_xyzi(msg, odom_pose) -> tuple[np.ndarray, np.ndarray]:
    values = pc2.read_points_numpy(msg, field_names=["x", "y", "z", "intensity"], skip_nans=True).astype(np.float64)
    world = values[:, :3]
    translation = np.array([odom_pose.position.x, odom_pose.position.y, odom_pose.position.z], dtype=np.float64)
    rotation = quaternion_to_rotation_matrix(odom_pose.orientation)
    raw = (world - translation) @ rotation
    base = np.empty_like(raw, dtype=np.float32)
    base[:, 0] = -raw[:, 2] + 0.072
    base[:, 1] = raw[:, 0] - 0.004
    base[:, 2] = -raw[:, 1] + 1.215
    return base, values[:, 3].astype(np.float32)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--source-bag", required=True)
    args = parser.parse_args()
    selected = read_manifest(args.manifest, args.source_bag)
    odoms = load_odoms(args.bag)
    stamps = [item[0] for item in odoms]
    point_type = get_message("sensor_msgs/msg/PointCloud2")
    output_rows = []
    reader = open_reader(args.bag)
    cloud_index = -1
    for_count = len(selected)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/fast_lio/cloud_registered":
            continue
        cloud_index += 1
        if cloud_index not in selected:
            continue
        msg = deserialize_message(data, point_type)
        stamp = stamp_to_sec(msg.header.stamp)
        _, odom_pose = nearest_odom(odoms, stamps, stamp)
        xyz, intensity = transform_xyzi(msg, odom_pose)
        sidecar = args.output_dir / args.source_bag / f"{selected[cloud_index]}.npz"
        sidecar.parent.mkdir(parents=True, exist_ok=True)
        np.savez(sidecar, xyz=xyz, intensity=intensity)
        output_rows.append({"cloud_index": cloud_index, "item_id": selected[cloud_index], "source_bag": args.source_bag, "sidecar": sidecar.resolve()})
        if len(output_rows) % 100 == 0:
            print(f"[intensity_export] {args.source_bag} {len(output_rows)}/{for_count}", flush=True)
    manifest = args.output_dir / f"manifest_{args.source_bag}.csv"
    with manifest.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(output_rows[0]))
        writer.writeheader()
        writer.writerows(output_rows)
    print(f"[intensity_export] wrote {manifest} rows={len(output_rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
