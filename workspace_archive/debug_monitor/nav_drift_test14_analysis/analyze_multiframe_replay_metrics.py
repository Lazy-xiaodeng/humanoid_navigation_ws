#!/usr/bin/env python3
import csv
import json
import math
import statistics
import sys
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG = "/home/ubuntu/nav_drift_test/nav_drift_test14"
OUT_DIR = Path("workspace_archive/debug_monitor/nav_drift_test14_analysis")
TZ = ZoneInfo("Asia/Shanghai")


@dataclass
class Waypoint:
    name: str
    waypoint_id: str
    start_t: float
    sequence_id: str
    reach_t: float | None = None
    failure_t: float | None = None
    failure_reason: str = ""
    next_start_t: float | None = None


def fmt_t(t):
    if t is None:
        return "-"
    return datetime.fromtimestamp(t, TZ).strftime("%H:%M:%S.%f")[:-3]


def percentile(values, pct):
    if not values:
        return 0.0
    values = sorted(values)
    idx = min(len(values) - 1, max(0, round((len(values) - 1) * pct)))
    return values[idx]


def read_waypoints():
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_type = get_message(topic_types["/navigation/status"])
    nav_msgs = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != "/navigation/status":
            continue
        msg = deserialize_message(data, msg_type)
        try:
            nav_msgs.append((t_ns / 1e9, json.loads(msg.data)))
        except Exception:
            pass

    waypoints = []
    by_seq = {}
    for t, obj in nav_msgs:
        event_type = obj.get("event_type")
        data = obj.get("event_data") or {}
        seq = obj.get("sequence_id") or ""
        if event_type == "waypoint_started":
            wp = Waypoint(data.get("waypoint_name", ""), data.get("waypoint_id", ""), t, seq)
            waypoints.append(wp)
            by_seq[seq] = wp
        elif event_type == "waypoint_reached" and seq in by_seq:
            by_seq[seq].reach_t = t
        elif event_type in {"navigation_failed", "navigation_localization_recovery_failed"} and seq in by_seq:
            by_seq[seq].failure_t = t
            by_seq[seq].failure_reason = data.get("reason", event_type)

    for idx, wp in enumerate(waypoints):
        wp.next_start_t = waypoints[idx + 1].start_t if idx + 1 < len(waypoints) else None
    return waypoints


def read_statuses(path):
    rows = []
    with Path(path).open() as f:
        for line in f:
            if not line.strip():
                continue
            row = json.loads(line)
            row["_t"] = float(row.get("stamp_sec", 0.0))
            rows.append(row)
    rows.sort(key=lambda r: r["_t"])
    return rows


def rows_for_wp(statuses, wp):
    end = wp.next_start_t if wp.next_start_t is not None else float("inf")
    return [s for s in statuses if wp.start_t <= s["_t"] < end]


def summarize_waypoints(statuses, waypoints):
    out = []
    for wp in waypoints:
        rows = rows_for_wp(statuses, wp)
        reasons = Counter(s.get("reason", "") for s in rows)
        states = Counter(s.get("state", "") for s in rows)
        corr = [float(s.get("correction_translation", 0.0) or 0.0) for s in rows]
        fitness = [float(s.get("fitness_score", 0.0) or 0.0) for s in rows]
        mean_corr = [float(s.get("mean_corr_dist", 0.0) or 0.0) for s in rows]
        frames = [int(s.get("multi_frame_source_frames", 1) or 1) for s in rows]
        points = [int(s.get("multi_frame_source_points", 0) or 0) for s in rows]
        align = [float(s.get("ndt_align_duration_sec", 0.0) or 0.0) for s in rows]
        out.append({
            "waypoint": wp.name,
            "id": wp.waypoint_id,
            "start": fmt_t(wp.start_t),
            "reach_or_fail": fmt_t(wp.reach_t or wp.failure_t),
            "result": "failed" if wp.failure_t or not wp.reach_t else "reached",
            "total": len(rows),
            "accepted": states.get("accepted", 0),
            "confirming": states.get("confirming", 0),
            "rejected": states.get("rejected", 0),
            "pose_jump": reasons.get("pose_jump", 0),
            "high_fitness": reasons.get("high_fitness", 0),
            "max_correction": max(corr or [0.0]),
            "max_fitness": max(fitness or [0.0]),
            "max_mean_corr": max(mean_corr or [0.0]),
            "avg_source_frames": statistics.fmean(frames) if frames else 0.0,
            "max_source_frames": max(frames or [0]),
            "avg_source_points": statistics.fmean(points) if points else 0.0,
            "max_source_points": max(points or [0]),
            "align_p50_ms": percentile(align, 0.50) * 1000.0,
            "align_p95_ms": percentile(align, 0.95) * 1000.0,
            "align_max_ms": max(align or [0.0]) * 1000.0,
            "fastlio_delta_used": sum(1 for s in rows if s.get("fastlio_delta_applied")),
            "failure_reason": wp.failure_reason,
        })
    return out


def write_waypoint_metrics(rows):
    path = OUT_DIR / "replayed_multiframe_waypoint_metrics.csv"
    with path.open("w", newline="") as f:
        fieldnames = list(rows[0].keys()) if rows else []
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


def write_point18_timeline(statuses, waypoints):
    wp18 = next((wp for wp in waypoints if wp.name == "点位18"), None)
    if wp18 is None:
        return None
    rows = rows_for_wp(statuses, wp18)
    interesting = [
        s for s in rows
        if s.get("state") != "accepted"
        or float(s.get("correction_translation", 0.0) or 0.0) > 0.5
        or float(s.get("fitness_score", 0.0) or 0.0) > 0.05
    ]
    path = OUT_DIR / "replayed_multiframe_point18_timeline.csv"
    with path.open("w", newline="") as f:
        fields = [
            "time", "state", "reason", "correction_m", "fitness", "mean_corr_dist",
            "source_frames", "source_points", "align_ms", "candidate_x", "candidate_y",
            "candidate_yaw_deg", "init_x", "init_y", "init_yaw_deg",
            "candidate_delta_last_good_m", "consecutive_rejected",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for s in interesting:
            writer.writerow({
                "time": fmt_t(s["_t"]),
                "state": s.get("state", ""),
                "reason": s.get("reason", ""),
                "correction_m": f"{float(s.get('correction_translation', 0.0) or 0.0):.3f}",
                "fitness": f"{float(s.get('fitness_score', 0.0) or 0.0):.3f}",
                "mean_corr_dist": f"{float(s.get('mean_corr_dist', 0.0) or 0.0):.3f}",
                "source_frames": int(s.get("multi_frame_source_frames", 1) or 1),
                "source_points": int(s.get("multi_frame_source_points", 0) or 0),
                "align_ms": f"{float(s.get('ndt_align_duration_sec', 0.0) or 0.0) * 1000.0:.1f}",
                "candidate_x": f"{float(s.get('ndt_candidate_x', 0.0) or 0.0):.3f}",
                "candidate_y": f"{float(s.get('ndt_candidate_y', 0.0) or 0.0):.3f}",
                "candidate_yaw_deg": f"{math.degrees(float(s.get('ndt_candidate_yaw', 0.0) or 0.0)):.1f}",
                "init_x": f"{float(s.get('ndt_init_guess_x', 0.0) or 0.0):.3f}",
                "init_y": f"{float(s.get('ndt_init_guess_y', 0.0) or 0.0):.3f}",
                "init_yaw_deg": f"{math.degrees(float(s.get('ndt_init_guess_yaw', 0.0) or 0.0)):.1f}",
                "candidate_delta_last_good_m": f"{float(s.get('ndt_candidate_delta_last_good_xy', 0.0) or 0.0):.3f}",
                "consecutive_rejected": int(s.get("consecutive_rejected_frames", 0) or 0),
            })
    return path


def main():
    if len(sys.argv) != 2:
        print("usage: analyze_multiframe_replay_metrics.py STATUS_JSONL", file=sys.stderr)
        raise SystemExit(2)
    statuses = read_statuses(sys.argv[1])
    waypoints = read_waypoints()
    metrics = summarize_waypoints(statuses, waypoints)
    print(f"metrics={write_waypoint_metrics(metrics)}")
    print(f"point18={write_point18_timeline(statuses, waypoints)}")


if __name__ == "__main__":
    main()
