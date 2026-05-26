#!/usr/bin/env python3
"""Validate ScanContext relocalization and odometry route consistency from a bag."""

from __future__ import annotations

import argparse
import csv
import json
import math
import multiprocessing as mp
import re
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


R_FASTLIO_TO_ROS = np.array(
    [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
    dtype=np.float32,
)

_WORKER_CFG: Config | None = None
_WORKER_ARGS: argparse.Namespace | None = None
_WORKER_IDS: np.ndarray | None = None
_WORKER_POSES: np.ndarray | None = None
_WORKER_DESCRIPTORS: np.ndarray | None = None
_WORKER_RING_KEYS: np.ndarray | None = None


@dataclass
class Config:
    num_sectors: int
    num_rings: int
    max_range: float
    min_range: float = 0.8
    lateral_search_ratio: float = 0.1
    horizontal_axis_1: int = 0
    horizontal_axis_2: int = 2
    vertical_axis: int = 1
    vertical_sign: float = -1.0


@dataclass
class KeyFrame:
    keyframe_id: int
    pose: np.ndarray
    descriptor: np.ndarray
    ring_key: np.ndarray


def storage_id_for_bag(bag_path: Path) -> str:
    metadata = bag_path / "metadata.yaml"
    if metadata.exists():
        with metadata.open("r", encoding="utf-8") as f:
            info = yaml.safe_load(f)
        return info.get("rosbag2_bagfile_information", {}).get("storage_identifier", "sqlite3")
    if list(bag_path.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def stamp_sec(msg, fallback_ns: int) -> float:
    stamp = getattr(msg, "header", None).stamp if hasattr(msg, "header") else None
    if stamp is None:
        return fallback_ns * 1e-9
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return value if value > 0.0 else fallback_ns * 1e-9


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


def pointcloud_to_array(msg: PointCloud2) -> np.ndarray | None:
    fields = [field.name for field in msg.fields]
    names = ("x", "y", "z", "intensity") if "intensity" in fields else ("x", "y", "z")
    try:
        raw = pc2.read_points_numpy(msg, field_names=names, skip_nans=True)
        if raw.size == 0:
            return None
        raw = np.asarray(raw, dtype=np.float32).reshape(-1, len(names))
        arr = np.zeros((raw.shape[0], 4), dtype=np.float32)
        arr[:, : raw.shape[1]] = raw[:, :4]
        return arr
    except Exception:
        pass

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


def registered_cloud_to_local(cloud: np.ndarray, pose: np.ndarray) -> np.ndarray:
    local = np.array(cloud, copy=True)
    local[:, :3] = (pose[:3, :3].T @ (local[:, :3] - pose[:3, 3]).T).T
    return local


def compute_descriptor(cloud: np.ndarray, cfg: Config) -> np.ndarray:
    descriptor = np.zeros((cfg.num_rings, cfg.num_sectors), dtype=np.float32)
    if cloud is None or len(cloud) == 0:
        return descriptor
    h1 = cloud[:, cfg.horizontal_axis_1]
    h2 = cloud[:, cfg.horizontal_axis_2]
    vertical = cloud[:, cfg.vertical_axis] * cfg.vertical_sign
    ranges = np.sqrt(h1**2 + h2**2)
    valid = (ranges >= cfg.min_range) & (ranges <= cfg.max_range)
    if not np.any(valid):
        return descriptor
    ranges = ranges[valid]
    h1 = h1[valid]
    h2 = h2[valid]
    vertical = vertical[valid]
    heights = vertical - float(np.min(vertical))
    angles = np.arctan2(h2, h1)
    angles = np.where(angles < 0.0, angles + 2.0 * math.pi, angles)
    ring_step = cfg.max_range / cfg.num_rings
    sector_step = 2.0 * math.pi / cfg.num_sectors
    rings = np.clip((ranges / ring_step).astype(np.int32), 0, cfg.num_rings - 1)
    sectors = np.clip((angles / sector_step).astype(np.int32), 0, cfg.num_sectors - 1)
    for ring, sector, height in zip(rings, sectors, heights):
        if height > descriptor[ring, sector]:
            descriptor[ring, sector] = height
    return descriptor


def load_database(path: Path, args: argparse.Namespace) -> tuple[Config, list[KeyFrame]]:
    with path.open("rb") as f:
        num_sectors = struct.unpack("i", f.read(4))[0]
        num_rings = struct.unpack("i", f.read(4))[0]
        max_range = struct.unpack("d", f.read(8))[0]
        _reserved = struct.unpack("d", f.read(8))[0]
        count = struct.unpack("Q", f.read(8))[0]
        cfg = Config(
            num_sectors=num_sectors,
            num_rings=num_rings,
            max_range=max_range,
            min_range=args.min_range,
            lateral_search_ratio=args.lateral_search_ratio,
            horizontal_axis_1=args.horizontal_axis_1,
            horizontal_axis_2=args.horizontal_axis_2,
            vertical_axis=args.vertical_axis,
            vertical_sign=args.vertical_sign,
        )
        keyframes: list[KeyFrame] = []
        for _ in range(count):
            keyframe_id = struct.unpack("i", f.read(4))[0]
            pose = np.frombuffer(f.read(16 * 4), dtype=np.float32).reshape((4, 4)).T.copy()
            rows = struct.unpack("i", f.read(4))[0]
            cols = struct.unpack("i", f.read(4))[0]
            descriptor = np.frombuffer(f.read(rows * cols * 4), dtype=np.float32).reshape((cols, rows)).T.copy()
            ring_size = struct.unpack("i", f.read(4))[0]
            ring_key = np.frombuffer(f.read(ring_size * 4), dtype=np.float32).copy()
            keyframes.append(KeyFrame(keyframe_id, pose, descriptor, ring_key))
    return cfg, keyframes


def circular_shift_reference(reference: np.ndarray, shift: int) -> np.ndarray:
    return np.roll(reference, -shift, axis=1)


def column_cosine_distance(query: np.ndarray, reference: np.ndarray) -> float:
    q_norm = np.linalg.norm(query, axis=0)
    r_norm = np.linalg.norm(reference, axis=0)
    valid = (q_norm > 1e-6) & (r_norm > 1e-6)
    if not np.any(valid):
        return 1.0
    cosine = np.sum(query[:, valid] * reference[:, valid], axis=0) / (q_norm[valid] * r_norm[valid])
    return float(np.mean(1.0 - np.clip(cosine, -1.0, 1.0)))


def distance_and_yaw(query: np.ndarray, reference: np.ndarray, cfg: Config) -> tuple[float, int]:
    best_distance = float("inf")
    best_shift = 0
    lateral = max(0, round(cfg.num_sectors * cfg.lateral_search_ratio))
    for shift in range(cfg.num_sectors):
        distance = column_cosine_distance(query, circular_shift_reference(reference, shift))
        for offset in range(1, lateral + 1):
            distance = min(distance, column_cosine_distance(query, circular_shift_reference(reference, shift + offset)))
            distance = min(distance, column_cosine_distance(query, circular_shift_reference(reference, shift - offset)))
        if distance < best_distance:
            best_distance = distance
            best_shift = shift
    return best_distance, best_shift


def yaw_correction(yaw_rad: float, cfg: Config) -> np.ndarray:
    mat = np.eye(4, dtype=np.float32)
    a, b = cfg.horizontal_axis_1, cfg.horizontal_axis_2
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    mat[a, a] = c
    mat[a, b] = -s
    mat[b, a] = s
    mat[b, b] = c
    return mat


def search_scancontext(descriptor: np.ndarray, cfg: Config, keyframes: list[KeyFrame], args) -> dict[str, float | int]:
    query_ring = descriptor.mean(axis=1)
    ring_scores = sorted(
        ((float(np.linalg.norm(query_ring - keyframe.ring_key)), idx) for idx, keyframe in enumerate(keyframes)),
        key=lambda item: item[0],
    )
    candidates = []
    for _, idx in ring_scores[: max(args.top_k, args.ring_candidates)]:
        keyframe = keyframes[idx]
        distance, shift = distance_and_yaw(descriptor, keyframe.descriptor, cfg)
        yaw_rad = shift * 2.0 * math.pi / cfg.num_sectors
        pose = keyframe.pose @ yaw_correction(yaw_rad, cfg)
        candidates.append((distance, shift, keyframe.keyframe_id, pose, idx))
    candidates.sort(key=lambda item: item[0])
    best = candidates[0]
    return {
        "distance": best[0],
        "yaw_shift": best[1],
        "keyframe_id": best[2],
        "pose": best[3],
        "accepted": best[0] <= args.sc_threshold,
    }


def build_database_arrays(keyframes: list[KeyFrame]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    ids = np.asarray([keyframe.keyframe_id for keyframe in keyframes], dtype=np.int32)
    poses = np.stack([keyframe.pose for keyframe in keyframes]).astype(np.float32)
    descriptors = np.stack([keyframe.descriptor for keyframe in keyframes]).astype(np.float32)
    ring_keys = np.stack([keyframe.ring_key for keyframe in keyframes]).astype(np.float32)
    return ids, poses, descriptors, ring_keys


def init_worker(cfg: Config, args: argparse.Namespace, ids, poses, descriptors, ring_keys) -> None:
    global _WORKER_CFG, _WORKER_ARGS, _WORKER_IDS, _WORKER_POSES, _WORKER_DESCRIPTORS, _WORKER_RING_KEYS
    _WORKER_CFG = cfg
    _WORKER_ARGS = args
    _WORKER_IDS = ids
    _WORKER_POSES = poses
    _WORKER_DESCRIPTORS = descriptors
    _WORKER_RING_KEYS = ring_keys


def search_scancontext_arrays(descriptor: np.ndarray, sample_index: int, odom_pose: np.ndarray) -> dict[str, float | int]:
    cfg = _WORKER_CFG
    args = _WORKER_ARGS
    ids = _WORKER_IDS
    poses = _WORKER_POSES
    db_desc = _WORKER_DESCRIPTORS
    ring_keys = _WORKER_RING_KEYS
    assert cfg is not None and args is not None
    assert ids is not None and poses is not None and db_desc is not None and ring_keys is not None

    query_ring = descriptor.mean(axis=1)
    ring_dist = np.linalg.norm(ring_keys - query_ring.reshape(1, -1), axis=1)
    if args.exclude_keyframe_window > 0:
        excluded = np.abs(ids - sample_index) <= args.exclude_keyframe_window
        ring_dist = np.where(excluded, np.inf, ring_dist)
    candidate_count = min(len(ids), max(args.top_k, args.ring_candidates))
    candidate_indices = np.argpartition(ring_dist, candidate_count - 1)[:candidate_count]
    refs = db_desc[candidate_indices]
    q_norm = np.linalg.norm(descriptor, axis=0).reshape(1, -1)
    ref_norm = np.linalg.norm(refs, axis=1)

    best_distance = np.full(candidate_count, np.inf, dtype=np.float32)
    best_shift = np.zeros(candidate_count, dtype=np.int32)
    for shift in range(cfg.num_sectors):
        shifted = np.roll(refs, -shift, axis=2)
        dot = np.sum(shifted * descriptor.reshape(1, cfg.num_rings, cfg.num_sectors), axis=1)
        denom = ref_norm * q_norm
        valid = denom > 1e-6
        cosine = np.zeros_like(dot, dtype=np.float32)
        cosine[valid] = np.clip(dot[valid] / denom[valid], -1.0, 1.0)
        dist_cols = np.where(valid, 1.0 - cosine, np.nan)
        distances = np.nanmean(dist_cols, axis=1)
        distances = np.where(np.isfinite(distances), distances, 1.0)
        improved = distances < best_distance
        best_distance[improved] = distances[improved]
        best_shift[improved] = shift

    order = np.argsort(best_distance)
    selected = None
    selected_gate_error = float("inf")
    gate_pass_indices = []
    raw_best = int(order[0])
    odom_xy = camera_to_ros_xy(odom_pose)
    for local_idx in order:
        db_idx = int(candidate_indices[int(local_idx)])
        yaw_rad = float(best_shift[int(local_idx)]) * 2.0 * math.pi / cfg.num_sectors
        pose = poses[db_idx] @ yaw_correction(yaw_rad, cfg)
        gate_error = float(np.linalg.norm(camera_to_ros_xy(pose) - odom_xy))
        if args.odom_gate_distance <= 0.0 or gate_error <= args.odom_gate_distance:
            gate_pass_indices.append((int(local_idx), db_idx, yaw_rad, pose, gate_error))
            if selected is None:
                selected = (int(local_idx), db_idx, yaw_rad, pose, gate_error, True)

    if selected is None:
        db_idx = int(candidate_indices[raw_best])
        yaw_rad = float(best_shift[raw_best]) * 2.0 * math.pi / cfg.num_sectors
        pose = poses[db_idx] @ yaw_correction(yaw_rad, cfg)
        selected_gate_error = float(np.linalg.norm(camera_to_ros_xy(pose) - odom_xy))
        selected = (raw_best, db_idx, yaw_rad, pose, selected_gate_error, False)

    local_best, db_idx, _yaw_rad, pose, selected_gate_error, odom_gate_pass = selected
    ambiguity_gate_pass = True
    if args.enable_candidate_confidence_gate and len(gate_pass_indices) > 1:
        selected_xy = camera_to_ros_xy(pose)
        selected_distance = float(best_distance[local_best])
        for other_local_idx, _other_db_idx, _other_yaw, other_pose, _other_gate_error in gate_pass_indices:
            if other_local_idx == local_best:
                continue
            distance_gap = float(best_distance[other_local_idx]) - selected_distance
            if distance_gap >= args.min_sc_distance_gap:
                continue
            candidate_separation = float(np.linalg.norm(camera_to_ros_xy(other_pose) - selected_xy))
            if candidate_separation > args.max_ambiguous_candidate_distance:
                ambiguity_gate_pass = False
                break
    accepted = (
        float(best_distance[local_best]) <= args.sc_threshold
        and bool(odom_gate_pass)
        and bool(ambiguity_gate_pass)
    )
    return {
        "distance": float(best_distance[local_best]),
        "yaw_shift": int(best_shift[local_best]),
        "keyframe_id": int(ids[db_idx]),
        "pose": pose,
        "accepted": accepted,
        "odom_gate_error": float(selected_gate_error),
        "odom_gate_pass": bool(odom_gate_pass),
        "ambiguity_gate_pass": bool(ambiguity_gate_pass),
    }


def process_sample(task: tuple[int, float, np.ndarray, np.ndarray]) -> tuple[int, float, np.ndarray, dict]:
    idx, t, descriptor, odom_pose = task
    return idx, t, odom_pose, search_scancontext_arrays(descriptor, idx, odom_pose)


def camera_to_ros_xy(pose: np.ndarray) -> np.ndarray:
    return (R_FASTLIO_TO_ROS @ pose[:3, 3].reshape(3, 1)).reshape(3)[:2]


def load_waypoints(path: Path, group: str) -> list[dict]:
    data = json.loads(path.read_text(encoding="utf-8"))
    items = []
    for type_map in data.get("waypoints", {}).values():
        for waypoint_id, waypoint in type_map.items():
            name = str(waypoint.get("name", waypoint_id))
            number = int(re.search(r"(\d+)", name).group(1)) if re.search(r"(\d+)", name) else 99999
            created = float(waypoint.get("created_time", 0.0))
            pos = waypoint.get("position", [])
            if len(pos) < 2:
                continue
            items.append(
                {
                    "id": str(waypoint_id),
                    "name": name,
                    "number": number,
                    "created_time": created,
                    "xy": np.asarray(pos[:2], dtype=np.float32),
                }
            )
    if group == "latest":
        max_created_by_number = {}
        for item in items:
            old = max_created_by_number.get(item["number"])
            if old is None or item["created_time"] > old["created_time"]:
                max_created_by_number[item["number"]] = item
        items = list(max_created_by_number.values())
    elif group == "earliest":
        min_created_by_number = {}
        for item in items:
            old = min_created_by_number.get(item["number"])
            if old is None or item["created_time"] < old["created_time"]:
                min_created_by_number[item["number"]] = item
        items = list(min_created_by_number.values())
    elif group.startswith("ids:"):
        allowed = set(group.split(":", 1)[1].split(","))
        items = [item for item in items if item["id"] in allowed]
    items.sort(key=lambda item: (item["number"], item["id"]))
    return items


def route_points_from_waypoints(waypoints: list[dict]) -> np.ndarray:
    route = [np.asarray([0.0, 0.0], dtype=np.float32)]
    route.extend(item["xy"] for item in waypoints)
    return np.asarray(route, dtype=np.float32)


def distance_to_polyline(point: np.ndarray, route: np.ndarray) -> tuple[float, int]:
    best_distance = float("inf")
    best_segment = 0
    for idx in range(len(route) - 1):
        a = route[idx]
        b = route[idx + 1]
        ab = b - a
        denom = float(np.dot(ab, ab))
        t = 0.0 if denom < 1e-9 else float(np.clip(np.dot(point - a, ab) / denom, 0.0, 1.0))
        proj = a + t * ab
        distance = float(np.linalg.norm(point - proj))
        if distance < best_distance:
            best_distance = distance
            best_segment = idx
    return best_distance, best_segment


def nearest_waypoint(point: np.ndarray, waypoints: list[dict]) -> tuple[dict, float]:
    distances = [(float(np.linalg.norm(point - item["xy"])), item) for item in waypoints]
    distance, item = min(distances, key=lambda pair: pair[0])
    return item, distance


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def percentile(values: list[float], q: float) -> float:
    return float(np.quantile(np.asarray(values, dtype=np.float32), q)) if values else float("nan")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", default="/home/ubuntu/fast-lio-bags/hall_mapping")
    parser.add_argument("--database", default="/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin")
    parser.add_argument("--waypoints", default="/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json")
    parser.add_argument("--waypoint-group", default="latest", help="latest, earliest, or ids:id1,id2,...")
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--cloud-topic", default="/fast_lio/cloud_registered")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--sample-period", type=float, default=2.0)
    parser.add_argument(
        "--start-offset",
        type=float,
        default=0.0,
        help="Delay the first query sample by this many seconds after the first cloud.",
    )
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument(
        "--exclude-keyframe-window",
        type=int,
        default=0,
        help="Exclude database keyframe ids within +/- this window of the query sample index.",
    )
    parser.add_argument("--min-points", type=int, default=1000)
    parser.add_argument("--route-threshold", type=float, default=1.0)
    parser.add_argument("--waypoint-radius", type=float, default=1.0)
    parser.add_argument("--sc-threshold", type=float, default=0.25)
    parser.add_argument(
        "--odom-gate-distance",
        type=float,
        default=1.0,
        help="Reject SC candidates farther than this from current odom in ROS/map xy. <=0 disables.",
    )
    parser.add_argument("--enable-candidate-confidence-gate", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--min-sc-distance-gap", type=float, default=0.03)
    parser.add_argument("--max-ambiguous-candidate-distance", type=float, default=2.0)
    parser.add_argument("--rescue-error-threshold", type=float, default=1.0)
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--ring-candidates", type=int, default=30)
    parser.add_argument("--min-range", type=float, default=0.8)
    parser.add_argument("--lateral-search-ratio", type=float, default=0.1)
    parser.add_argument("--horizontal-axis-1", type=int, default=0)
    parser.add_argument("--horizontal-axis-2", type=int, default=2)
    parser.add_argument("--vertical-axis", type=int, default=1)
    parser.add_argument("--vertical-sign", type=float, default=-1.0)
    args = parser.parse_args()

    started = time.strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_dir or f"/home/ubuntu/humanoid_ws/debug_monitor/scancontext_bag_validation_{started}")
    output_dir.mkdir(parents=True, exist_ok=True)

    cfg, keyframes = load_database(Path(args.database), args)
    waypoints = load_waypoints(Path(args.waypoints), args.waypoint_group)
    route = route_points_from_waypoints(waypoints)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(Path(args.bag)), storage_id=storage_id_for_bag(Path(args.bag))),
        ConverterOptions("", ""),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if args.cloud_topic not in type_map or args.odom_topic not in type_map:
        raise RuntimeError(f"Required topics missing. Available: {sorted(type_map)}")

    odoms: list[tuple[float, np.ndarray]] = []
    samples: list[tuple[int, float, np.ndarray, np.ndarray]] = []
    last_sample_time: float | None = None
    first_cloud_time: float | None = None
    print(f"Reading bag {args.bag}")
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == args.odom_topic:
            msg = deserialize_message(data, Odometry)
            odoms.append((stamp_sec(msg, timestamp), odom_to_matrix(msg)))
        elif topic == args.cloud_topic:
            t = timestamp * 1e-9
            if first_cloud_time is None:
                first_cloud_time = t
            if t < first_cloud_time + args.start_offset:
                continue
            if last_sample_time is not None and t - last_sample_time < args.sample_period:
                continue
            if not odoms:
                continue
            msg = deserialize_message(data, PointCloud2)
            cloud = pointcloud_to_array(msg)
            if cloud is None or len(cloud) < args.min_points:
                continue
            odom_t, odom_pose = min(odoms[-20:], key=lambda item: abs(item[0] - stamp_sec(msg, timestamp)))
            if abs(odom_t - stamp_sec(msg, timestamp)) > 0.35:
                continue
            local_cloud = registered_cloud_to_local(cloud, odom_pose)
            descriptor = compute_descriptor(local_cloud, cfg)
            samples.append((len(samples), stamp_sec(msg, timestamp), descriptor, odom_pose))
            last_sample_time = t
            if len(samples) % 25 == 0:
                print(f"  sampled {len(samples)} clouds at t={t:.1f}")

    rows: list[dict] = []
    waypoint_stats = {
        item["id"]: {
            "id": item["id"],
            "name": item["name"],
            "x": float(item["xy"][0]),
            "y": float(item["xy"][1]),
            "min_odom_dist": float("inf"),
            "min_sc_dist": float("inf"),
            "first_odom_near_t": "",
            "first_sc_near_t": "",
        }
        for item in waypoints
    }

    ids, poses, descriptors, ring_keys = build_database_arrays(keyframes)
    if args.workers > 1 and len(samples) > 1:
        with mp.Pool(
            processes=args.workers,
            initializer=init_worker,
            initargs=(cfg, args, ids, poses, descriptors, ring_keys),
        ) as pool:
            results_iter = pool.imap(process_sample, samples, chunksize=max(1, len(samples) // (args.workers * 16)))
            results = []
            for count, result_item in enumerate(results_iter, start=1):
                results.append(result_item)
                if count % 25 == 0:
                    print(f"  matched {count}/{len(samples)} samples")
        results.sort(key=lambda item: item[0])
    else:
        init_worker(cfg, args, ids, poses, descriptors, ring_keys)
        results = [process_sample(sample) for sample in samples]

    for idx, t, odom_pose, result in results:
        odom_xy = camera_to_ros_xy(odom_pose)
        sc_xy = camera_to_ros_xy(result["pose"])
        odom_route_dist, odom_seg = distance_to_polyline(odom_xy, route)
        sc_route_dist, sc_seg = distance_to_polyline(sc_xy, route)
        near_odom_wp, near_odom_dist = nearest_waypoint(odom_xy, waypoints)
        near_sc_wp, near_sc_dist = nearest_waypoint(sc_xy, waypoints)
        sc_odom_error = float(np.linalg.norm(sc_xy - odom_xy))
        rescue_possible = (
            odom_route_dist > args.route_threshold
            and bool(result["accepted"])
            and sc_route_dist <= args.route_threshold
            and sc_odom_error <= args.rescue_error_threshold
        )

        for item, dist, kind in ((near_odom_wp, near_odom_dist, "odom"), (near_sc_wp, near_sc_dist, "sc")):
            stat = waypoint_stats[item["id"]]
            key = f"min_{kind}_dist"
            first_key = f"first_{kind}_near_t"
            if dist < stat[key]:
                stat[key] = dist
            if dist <= args.waypoint_radius and not stat[first_key]:
                stat[first_key] = f"{t:.3f}"

        rows.append(
            {
                "sample_index": idx,
                "stamp": f"{t:.3f}",
                "odom_x": f"{odom_xy[0]:.4f}",
                "odom_y": f"{odom_xy[1]:.4f}",
                "odom_route_dist": f"{odom_route_dist:.4f}",
                "odom_route_segment": odom_seg,
                "odom_nearest_waypoint": near_odom_wp["name"],
                "odom_nearest_waypoint_id": near_odom_wp["id"],
                "odom_nearest_waypoint_dist": f"{near_odom_dist:.4f}",
                "sc_x": f"{sc_xy[0]:.4f}",
                "sc_y": f"{sc_xy[1]:.4f}",
                "sc_route_dist": f"{sc_route_dist:.4f}",
                "sc_route_segment": sc_seg,
                "sc_nearest_waypoint": near_sc_wp["name"],
                "sc_nearest_waypoint_id": near_sc_wp["id"],
                "sc_nearest_waypoint_dist": f"{near_sc_dist:.4f}",
                "sc_odom_error": f"{sc_odom_error:.4f}",
                "sc_distance": f"{float(result['distance']):.4f}",
                "odom_gate_error": f"{float(result['odom_gate_error']):.4f}",
                "odom_gate_pass": bool(result["odom_gate_pass"]),
                "ambiguity_gate_pass": bool(result["ambiguity_gate_pass"]),
                "sc_accepted": bool(result["accepted"]),
                "sc_keyframe_id": int(result["keyframe_id"]),
                "sc_yaw_shift": int(result["yaw_shift"]),
                "rescue_possible": rescue_possible,
            }
        )

    write_csv(output_dir / "samples.csv", rows)
    write_csv(output_dir / "waypoints.csv", list(waypoint_stats.values()))

    accepted = [r for r in rows if r["sc_accepted"]]
    odom_gate_rejected = [r for r in rows if not r["odom_gate_pass"]]
    ambiguity_gate_rejected = [r for r in rows if r["odom_gate_pass"] and not r["ambiguity_gate_pass"]]
    odom_route = [float(r["odom_route_dist"]) for r in rows]
    sc_route = [float(r["sc_route_dist"]) for r in rows if r["sc_accepted"]]
    sc_odom = [float(r["sc_odom_error"]) for r in rows if r["sc_accepted"]]
    sc_dist = [float(r["sc_distance"]) for r in rows]
    first_odom_off = next((r for r in rows if float(r["odom_route_dist"]) > args.route_threshold), None)
    first_sc_bad = next((r for r in rows if (not r["sc_accepted"]) or float(r["sc_odom_error"]) > args.rescue_error_threshold), None)
    rescue_count = sum(1 for r in rows if r["rescue_possible"])

    unreached = [
        stat for stat in waypoint_stats.values()
        if not stat["first_odom_near_t"] and not stat["first_sc_near_t"]
    ]

    summary = {
        "bag": args.bag,
        "database": args.database,
        "waypoints": args.waypoints,
        "waypoint_group": args.waypoint_group,
        "samples": len(rows),
        "keyframes_in_database": len(keyframes),
        "accepted_count": len(accepted),
        "accepted_rate": len(accepted) / len(rows) if rows else 0.0,
        "odom_gate_distance": args.odom_gate_distance,
        "odom_gate_rejected_count": len(odom_gate_rejected),
        "odom_gate_rejected_rate": len(odom_gate_rejected) / len(rows) if rows else 0.0,
        "candidate_confidence_gate": args.enable_candidate_confidence_gate,
        "min_sc_distance_gap": args.min_sc_distance_gap,
        "max_ambiguous_candidate_distance": args.max_ambiguous_candidate_distance,
        "ambiguity_gate_rejected_count": len(ambiguity_gate_rejected),
        "ambiguity_gate_rejected_rate": len(ambiguity_gate_rejected) / len(rows) if rows else 0.0,
        "odom_route_dist_median": percentile(odom_route, 0.5),
        "odom_route_dist_p95": percentile(odom_route, 0.95),
        "sc_route_dist_median": percentile(sc_route, 0.5),
        "sc_route_dist_p95": percentile(sc_route, 0.95),
        "sc_odom_error_median": percentile(sc_odom, 0.5),
        "sc_odom_error_p95": percentile(sc_odom, 0.95),
        "sc_distance_median": percentile(sc_dist, 0.5),
        "sc_distance_p95": percentile(sc_dist, 0.95),
        "first_odom_off_route": first_odom_off,
        "first_sc_bad": first_sc_bad,
        "rescue_possible_samples": rescue_count,
        "unreached_waypoints": unreached,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    md = [
        "# ScanContext Bag Validation",
        "",
        f"- Bag: `{args.bag}`",
        f"- Database: `{args.database}`",
        f"- Waypoints: `{args.waypoints}` (`{args.waypoint_group}`)",
        f"- Samples: {len(rows)}",
        f"- SC accepted: {len(accepted)}/{len(rows)} ({summary['accepted_rate'] * 100:.1f}%)",
        f"- Odom gate rejected: {len(odom_gate_rejected)}/{len(rows)} ({summary['odom_gate_rejected_rate'] * 100:.1f}%), gate={args.odom_gate_distance:.2f} m",
        f"- Ambiguity gate rejected: {len(ambiguity_gate_rejected)}/{len(rows)} ({summary['ambiguity_gate_rejected_rate'] * 100:.1f}%)",
        f"- Odom route distance median/p95: {summary['odom_route_dist_median']:.3f} / {summary['odom_route_dist_p95']:.3f} m",
        f"- SC route distance median/p95: {summary['sc_route_dist_median']:.3f} / {summary['sc_route_dist_p95']:.3f} m",
        f"- SC-vs-odom error median/p95: {summary['sc_odom_error_median']:.3f} / {summary['sc_odom_error_p95']:.3f} m",
        f"- SC distance median/p95: {summary['sc_distance_median']:.3f} / {summary['sc_distance_p95']:.3f}",
        f"- Rescue-possible samples: {rescue_count}",
        "",
        "## First Events",
        "",
        f"- First odom off route > {args.route_threshold:.2f} m: {first_odom_off or 'none'}",
        f"- First SC bad: {first_sc_bad or 'none'}",
        "",
        "## Waypoint Reach Table",
        "",
        "| id | name | x | y | min odom dist | first odom near | min SC dist | first SC near |",
        "|---|---|---:|---:|---:|---|---:|---|",
    ]
    for stat in waypoint_stats.values():
        md.append(
            f"| {stat['id']} | {stat['name']} | {stat['x']:.3f} | {stat['y']:.3f} | "
            f"{stat['min_odom_dist']:.3f} | {stat['first_odom_near_t']} | "
            f"{stat['min_sc_dist']:.3f} | {stat['first_sc_near_t']} |"
        )
    (output_dir / "report.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(f"wrote validation output to {output_dir}")
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
