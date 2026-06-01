#!/usr/bin/env python3
import csv
import json
import math
import sys
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
JUMP_TRANS_LIMIT = 0.50
JUMP_YAW_LIMIT = 0.40


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


@dataclass
class Waypoint:
    name: str
    waypoint_id: str
    start_t: float
    sequence_id: str
    goal: tuple
    reach_t: float | None = None
    complete_t: float | None = None
    failure_t: float | None = None
    failure_reason: str = ""
    next_start_t: float | None = None


def fmt_t(t):
    if t is None:
        return "-"
    return datetime.fromtimestamp(t, TZ).strftime("%H:%M:%S.%f")[:-3]


def fmt_pose(p):
    if p is None:
        return "-"
    return f"({p.x:.3f}, {p.y:.3f}, yaw={math.degrees(p.yaw):.1f}deg)"


def pose_from_status(prefix, status):
    if not status.get(f"{prefix}_pose_valid", True):
        return None
    keys = (f"{prefix}_x", f"{prefix}_y", f"{prefix}_yaw")
    if not all(k in status for k in keys):
        return None
    return Pose(float(status[keys[0]]), float(status[keys[1]]), float(status[keys[2]]))


def read_nav_waypoints():
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
    return parse_waypoints(nav_msgs)


def parse_waypoints(nav_msgs):
    waypoints = []
    by_seq = {}
    for t, obj in nav_msgs:
        et = obj.get("event_type")
        data = obj.get("event_data") or {}
        seq = obj.get("sequence_id") or ""
        if et == "waypoint_started":
            wp = Waypoint(
                name=data.get("waypoint_name", ""),
                waypoint_id=data.get("waypoint_id", ""),
                start_t=t,
                sequence_id=seq,
                goal=tuple(data.get("position", [None, None, None])),
            )
            waypoints.append(wp)
            by_seq[seq] = wp
        elif et == "waypoint_reached" and seq in by_seq:
            by_seq[seq].reach_t = t
        elif et == "navigation_completed" and seq in by_seq:
            by_seq[seq].complete_t = t
        elif et in {"navigation_failed", "navigation_localization_recovery_failed"}:
            wp = by_seq.get(seq)
            if wp is not None:
                wp.failure_t = t
                wp.failure_reason = data.get("reason", et)
    for i, wp in enumerate(waypoints):
        wp.next_start_t = waypoints[i + 1].start_t if i + 1 < len(waypoints) else None
    return waypoints


def waypoint_for_time(waypoints, t):
    for wp in waypoints:
        end = wp.next_start_t if wp.next_start_t is not None else float("inf")
        if wp.start_t <= t < end:
            return wp
    return None


def read_statuses(path):
    rows = []
    with Path(path).open() as f:
        for line in f:
            if not line.strip():
                continue
            obj = json.loads(line)
            obj["_bag_t"] = float(obj.get("stamp_sec", 0.0))
            rows.append(obj)
    rows.sort(key=lambda x: x["_bag_t"])
    return rows


def is_jump(status):
    return (
        float(status.get("correction_translation", 0.0)) > JUMP_TRANS_LIMIT
        or abs(float(status.get("correction_yaw", 0.0))) > JUMP_YAW_LIMIT
    )


def cluster_statuses(statuses):
    clusters = []
    current = []
    for s in statuses:
        if not is_jump(s):
            if current:
                clusters.append(current)
                current = []
            continue
        if current and s["_bag_t"] - current[-1]["_bag_t"] > 1.5:
            clusters.append(current)
            current = []
        current.append(s)
    if current:
        clusters.append(current)
    return clusters


def summarize(waypoints, statuses, clusters):
    cluster_rows = []
    for c in clusters:
        first = c[0]
        last = c[-1]
        max_s = max(c, key=lambda s: float(s.get("correction_translation", 0.0)))
        wp = waypoint_for_time(waypoints, first["_bag_t"])
        phase = "非导航"
        if wp is not None:
            if wp.reach_t and first["_bag_t"] > wp.reach_t:
                phase = "到达后/下个点前"
            elif wp.failure_t and first["_bag_t"] >= wp.failure_t:
                phase = "失败后"
            else:
                phase = "导航中"
        rejected = [s for s in c if s.get("state") == "rejected" or s.get("reason") == "pose_jump"]
        before = pose_from_status("ndt_init_guess", max_s)
        after = pose_from_status("ndt_candidate", max_s)
        cluster_rows.append(
            {
                "waypoint": wp.name if wp else "非导航窗口",
                "phase": phase,
                "start": first["_bag_t"],
                "end": last["_bag_t"],
                "frames": len(c),
                "accepted_frames": sum(1 for s in c if s.get("state") == "accepted"),
                "rejected_frames": len(rejected),
                "outcome": "被 jump 拒绝" if rejected else "跳变已发布/accepted",
                "max_translation": float(max_s.get("correction_translation", 0.0)),
                "max_yaw": float(max_s.get("correction_yaw", 0.0)),
                "max_fitness": float(max_s.get("fitness_score", 0.0)),
                "max_time": max_s["_bag_t"],
                "before": before,
                "after": after,
                "rotation_guard_frames": sum(1 for s in c if s.get("rotation_guard_active")),
                "multiframe_max": max(int(s.get("multi_frame_source_frames", 1)) for s in c),
            }
        )

    waypoint_rows = []
    for wp in waypoints:
        end = wp.next_start_t if wp.next_start_t is not None else float("inf")
        wp_statuses = [s for s in statuses if wp.start_t <= s["_bag_t"] < end]
        wp_clusters = [r for r in cluster_rows if r["waypoint"] == wp.name]
        nav_end = wp.failure_t or wp.reach_t or end
        nav_clusters = [r for r in wp_clusters if r["start"] <= nav_end]
        max_cluster = max(wp_clusters, key=lambda r: r["max_translation"], default=None)
        waypoint_rows.append(
            {
                "waypoint": wp.name,
                "id": wp.waypoint_id,
                "start": wp.start_t,
                "reach_or_fail": wp.reach_t or wp.failure_t,
                "nav_result": "失败/暂停未完成" if wp.failure_t or not wp.reach_t else "成功到达",
                "jump_clusters": len(wp_clusters),
                "jump_clusters_during_nav": len(nav_clusters),
                "jump_frames": sum(r["frames"] for r in wp_clusters),
                "jump_rejected_frames": sum(r["rejected_frames"] for r in wp_clusters),
                "ndt_rejected_frames": sum(1 for s in wp_statuses if s.get("state") != "accepted"),
                "max_translation": max_cluster["max_translation"] if max_cluster else 0.0,
                "max_fitness": max_cluster["max_fitness"] if max_cluster else 0.0,
                "max_before": max_cluster["before"] if max_cluster else None,
                "max_after": max_cluster["after"] if max_cluster else None,
                "max_time": max_cluster["max_time"] if max_cluster else None,
                "failure_reason": wp.failure_reason,
            }
        )
    return waypoint_rows, cluster_rows


def write_outputs(status_path, waypoint_rows, cluster_rows):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    summary_path = OUT_DIR / "replayed_nojump_waypoint_summary.csv"
    cluster_path = OUT_DIR / "replayed_nojump_jump_clusters.csv"
    report_path = OUT_DIR / "nav_drift_test14_replayed_nojump_report.md"

    with summary_path.open("w", newline="") as f:
        fields = [
            "waypoint", "id", "start_time", "reach_or_fail_time", "navigation_result",
            "jump_clusters", "jump_clusters_during_nav", "jump_frames",
            "jump_rejected_frames", "ndt_rejected_frames", "max_jump_translation_m",
            "max_jump_fitness", "max_jump_time", "max_jump_before_pose",
            "max_jump_after_pose", "failure_reason",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in waypoint_rows:
            writer.writerow({
                "waypoint": r["waypoint"],
                "id": r["id"],
                "start_time": fmt_t(r["start"]),
                "reach_or_fail_time": fmt_t(r["reach_or_fail"]),
                "navigation_result": r["nav_result"],
                "jump_clusters": r["jump_clusters"],
                "jump_clusters_during_nav": r["jump_clusters_during_nav"],
                "jump_frames": r["jump_frames"],
                "jump_rejected_frames": r["jump_rejected_frames"],
                "ndt_rejected_frames": r["ndt_rejected_frames"],
                "max_jump_translation_m": f"{r['max_translation']:.3f}",
                "max_jump_fitness": f"{r['max_fitness']:.3f}",
                "max_jump_time": fmt_t(r["max_time"]),
                "max_jump_before_pose": fmt_pose(r["max_before"]),
                "max_jump_after_pose": fmt_pose(r["max_after"]),
                "failure_reason": r["failure_reason"],
            })

    with cluster_path.open("w", newline="") as f:
        fields = [
            "waypoint", "phase", "start_time", "end_time", "outcome", "frames",
            "accepted_frames", "rejected_frames", "max_translation_m", "max_yaw_rad",
            "max_fitness", "max_time", "jump_before_pose", "jump_after_pose",
            "rotation_guard_frames", "max_multiframe_source_frames",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in cluster_rows:
            writer.writerow({
                "waypoint": r["waypoint"],
                "phase": r["phase"],
                "start_time": fmt_t(r["start"]),
                "end_time": fmt_t(r["end"]),
                "outcome": r["outcome"],
                "frames": r["frames"],
                "accepted_frames": r["accepted_frames"],
                "rejected_frames": r["rejected_frames"],
                "max_translation_m": f"{r['max_translation']:.3f}",
                "max_yaw_rad": f"{r['max_yaw']:.3f}",
                "max_fitness": f"{r['max_fitness']:.3f}",
                "max_time": fmt_t(r["max_time"]),
                "jump_before_pose": fmt_pose(r["before"]),
                "jump_after_pose": fmt_pose(r["after"]),
                "rotation_guard_frames": r["rotation_guard_frames"],
                "max_multiframe_source_frames": r["multiframe_max"],
            })

    total = sum(r["jump_frames"] for r in waypoint_rows)
    rejected = sum(r["jump_rejected_frames"] for r in waypoint_rows)
    with report_path.open("w") as f:
        f.write("# nav_drift_test14 replayed NDT no-jump report\n\n")
        f.write(f"- replay_status_jsonl: `{status_path}`\n")
        f.write("- config: rotation_guard=on, multi_frame=always, reject_pose_jump=false\n")
        f.write(f"- jump definition: correction_translation > {JUMP_TRANS_LIMIT:.2f}m or correction_yaw > {JUMP_YAW_LIMIT:.2f}rad\n")
        f.write(f"- total would-be jump frames: {total}\n")
        f.write(f"- jump rejected frames: {rejected}\n")
        f.write("- note: navigation result is from original bag BT events; this replay validates localization publishing behavior only.\n\n")
        f.write("## Waypoint Summary\n\n")
        f.write("| 点位 | 时间 | 导航结果 | 跳变 | 最大跳变 | 跳变前 | 跳变后 | jump拒绝 |\n")
        f.write("| --- | --- | --- | --- | --- | --- | --- | --- |\n")
        for r in waypoint_rows:
            f.write(
                f"| {r['waypoint']} | {fmt_t(r['start'])}->{fmt_t(r['reach_or_fail'])} | "
                f"{r['nav_result']} | {r['jump_clusters']}段/{r['jump_frames']}帧 | "
                f"{r['max_translation']:.3f}m @ {fmt_t(r['max_time'])} | "
                f"`{fmt_pose(r['max_before'])}` | `{fmt_pose(r['max_after'])}` | "
                f"{r['jump_rejected_frames']}帧 |\n"
            )
        f.write("\n## Jump Clusters\n\n")
        f.write("| 点位 | 阶段 | 时间 | 结果 | 帧数 | 最大平移 | fitness | 跳变前 | 跳变后 |\n")
        f.write("| --- | --- | --- | --- | ---: | ---: | ---: | --- | --- |\n")
        for r in cluster_rows:
            f.write(
                f"| {r['waypoint']} | {r['phase']} | {fmt_t(r['start'])}->{fmt_t(r['end'])} | "
                f"{r['outcome']} | {r['frames']} | {r['max_translation']:.3f} | "
                f"{r['max_fitness']:.3f} | `{fmt_pose(r['before'])}` | `{fmt_pose(r['after'])}` |\n"
            )

    print(f"summary={summary_path}")
    print(f"clusters={cluster_path}")
    print(f"report={report_path}")
    print(f"jump_frames={total} rejected_jump_frames={rejected}")


def main():
    if len(sys.argv) != 2:
        print("usage: analyze_replayed_ndt_nojump.py STATUS_JSONL", file=sys.stderr)
        raise SystemExit(2)
    statuses = read_statuses(sys.argv[1])
    waypoints = read_nav_waypoints()
    clusters = cluster_statuses(statuses)
    waypoint_rows, cluster_rows = summarize(waypoints, statuses, clusters)
    write_outputs(sys.argv[1], waypoint_rows, cluster_rows)


if __name__ == "__main__":
    main()
