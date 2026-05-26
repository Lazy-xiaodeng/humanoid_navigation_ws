#!/usr/bin/env python3
"""Build a Scan Context sidecar database from a ROS 2 bag.

The output format intentionally matches the C++ sidecar node and the older
humanoid_relocalization Scan Context database format.
"""

import argparse
import math
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


@dataclass
class Config:
    num_sectors: int = 90
    num_rings: int = 24
    max_range: float = 60.0
    min_range: float = 0.8
    horizontal_axis_1: int = 0
    horizontal_axis_2: int = 2
    vertical_axis: int = 1
    vertical_sign: float = -1.0
    cloud_frame_mode: str = "registered"


def storage_id_for_bag(bag_path: Path) -> str:
    metadata = bag_path / "metadata.yaml"
    if metadata.exists():
        with metadata.open("r") as f:
            info = yaml.safe_load(f)
        return info.get("rosbag2_bagfile_information", {}).get("storage_identifier", "sqlite3")
    if list(bag_path.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def stamp_sec(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def pointcloud_to_array(msg: PointCloud2):
    fields = [field.name for field in msg.fields]
    names = ("x", "y", "z", "intensity") if "intensity" in fields else ("x", "y", "z")
    points = list(pc2.read_points(msg, field_names=names, skip_nans=True))
    if not points:
        return None
    arr = np.zeros((len(points), 4), dtype=np.float32)
    if hasattr(points[0], "__getitem__") and not isinstance(points[0], tuple):
        arr[:, 0] = [p["x"] for p in points]
        arr[:, 1] = [p["y"] for p in points]
        arr[:, 2] = [p["z"] for p in points]
        if "intensity" in names:
            arr[:, 3] = [p["intensity"] for p in points]
    else:
        raw = np.asarray(points, dtype=np.float32)
        arr[:, : raw.shape[1]] = raw[:, :4]
    return arr


def odom_to_matrix(msg: Odometry) -> np.ndarray:
    pos = msg.pose.pose.position
    q = msg.pose.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    rot = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )
    mat = np.eye(4, dtype=np.float32)
    mat[:3, :3] = rot
    mat[:3, 3] = [pos.x, pos.y, pos.z]
    return mat


def registered_cloud_to_local(cloud: np.ndarray, pose: np.ndarray) -> np.ndarray:
    """Convert /fast_lio/cloud_registered from camera_init/world into body-local points."""
    local = np.array(cloud, copy=True)
    rotation = pose[:3, :3]
    translation = pose[:3, 3]
    local[:, :3] = (rotation.T @ (local[:, :3] - translation).T).T
    return local


def compute_descriptor(cloud: np.ndarray, cfg: Config) -> np.ndarray:
    descriptor = np.zeros((cfg.num_rings, cfg.num_sectors), dtype=np.float32)
    if cloud is None or len(cloud) == 0:
        return descriptor

    h1 = cloud[:, cfg.horizontal_axis_1]
    h2 = cloud[:, cfg.horizontal_axis_2]
    vertical = cloud[:, cfg.vertical_axis] * cfg.vertical_sign
    xy_range = np.sqrt(h1**2 + h2**2)
    valid = (xy_range >= cfg.min_range) & (xy_range <= cfg.max_range)
    if not np.any(valid):
        return descriptor

    ranges = xy_range[valid]
    valid_h1 = h1[valid]
    valid_h2 = h2[valid]
    valid_vertical = vertical[valid]
    min_z = float(np.min(valid_vertical))
    heights = valid_vertical - min_z
    angles = np.arctan2(valid_h2, valid_h1)
    angles = np.where(angles < 0.0, angles + 2.0 * math.pi, angles)

    ring_step = cfg.max_range / cfg.num_rings
    sector_step = 2.0 * math.pi / cfg.num_sectors
    ring_indices = np.clip((ranges / ring_step).astype(np.int32), 0, cfg.num_rings - 1)
    sector_indices = np.clip((angles / sector_step).astype(np.int32), 0, cfg.num_sectors - 1)

    for ring, sector, height in zip(ring_indices, sector_indices, heights):
        if height > descriptor[ring, sector]:
            descriptor[ring, sector] = height
    return descriptor


def nearest_odom(odoms, t):
    if not odoms:
        return None
    best = min(odoms, key=lambda item: abs(item[0] - t))
    return best if abs(best[0] - t) < 0.25 else None


def read_bag(args):
    bag_path = Path(args.bag)
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id=storage_id_for_bag(bag_path)),
        ConverterOptions("", ""),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if args.cloud_topic not in type_map:
        raise RuntimeError(f"cloud topic {args.cloud_topic} not found. Available: {sorted(type_map)}")
    if args.odom_topic not in type_map:
        raise RuntimeError(f"odom topic {args.odom_topic} not found. Available: {sorted(type_map)}")

    odoms = []
    clouds = []
    last_cloud_time = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        t = timestamp * 1e-9
        if topic == args.odom_topic:
            msg = deserialize_message(data, Odometry)
            odoms.append((stamp_sec(msg) or t, odom_to_matrix(msg)))
        elif topic == args.cloud_topic:
            if last_cloud_time is not None and t - last_cloud_time < args.interval:
                continue
            msg = deserialize_message(data, PointCloud2)
            points = pointcloud_to_array(msg)
            if points is None or len(points) < args.min_points:
                continue
            clouds.append((stamp_sec(msg) or t, points))
            last_cloud_time = t
            print(f"cloud keyframe candidate {len(clouds)}: t={t:.2f}, points={len(points)}")
    return clouds, odoms


def write_database(path: Path, keyframes, cfg: Config):
    with path.open("wb") as f:
        f.write(struct.pack("i", cfg.num_sectors))
        f.write(struct.pack("i", cfg.num_rings))
        f.write(struct.pack("d", cfg.max_range))
        f.write(struct.pack("d", 0.0))
        f.write(struct.pack("Q", len(keyframes)))
        for keyframe_id, pose, descriptor in keyframes:
            ring_key = descriptor.mean(axis=1).astype(np.float32)
            f.write(struct.pack("i", keyframe_id))
            f.write(np.asarray(pose, dtype=np.float32).T.tobytes())
            f.write(struct.pack("i", descriptor.shape[0]))
            f.write(struct.pack("i", descriptor.shape[1]))
            f.write(np.asarray(descriptor, dtype=np.float32).T.tobytes())
            f.write(struct.pack("i", ring_key.shape[0]))
            f.write(ring_key.astype(np.float32).tobytes())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cloud-topic", default="/fast_lio/cloud_registered")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument(
        "--cloud-frame-mode",
        choices=("registered", "local"),
        default="registered",
        help="Use registered for /fast_lio/cloud_registered, local for body/sensor-frame clouds.",
    )
    parser.add_argument("--interval", type=float, default=1.5)
    parser.add_argument("--min-points", type=int, default=1000)
    parser.add_argument("--num-sectors", type=int, default=90)
    parser.add_argument("--num-rings", type=int, default=24)
    parser.add_argument("--max-range", type=float, default=60.0)
    parser.add_argument("--min-range", type=float, default=0.8)
    parser.add_argument("--horizontal-axis-1", type=int, default=0)
    parser.add_argument("--horizontal-axis-2", type=int, default=2)
    parser.add_argument("--vertical-axis", type=int, default=1)
    parser.add_argument("--vertical-sign", type=float, default=-1.0)
    args = parser.parse_args()

    cfg = Config(
        args.num_sectors,
        args.num_rings,
        args.max_range,
        args.min_range,
        args.horizontal_axis_1,
        args.horizontal_axis_2,
        args.vertical_axis,
        args.vertical_sign,
        args.cloud_frame_mode,
    )
    clouds, odoms = read_bag(args)
    keyframes = []
    for t, cloud in clouds:
        odom = nearest_odom(odoms, t)
        if odom is None:
            continue
        descriptor_cloud = cloud
        if cfg.cloud_frame_mode == "registered":
            descriptor_cloud = registered_cloud_to_local(cloud, odom[1])
        descriptor = compute_descriptor(descriptor_cloud, cfg)
        if np.count_nonzero(descriptor) < cfg.num_rings:
            continue
        keyframes.append((len(keyframes), odom[1], descriptor))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    write_database(output, keyframes, cfg)
    print(f"wrote {len(keyframes)} keyframes to {output}")


if __name__ == "__main__":
    main()
