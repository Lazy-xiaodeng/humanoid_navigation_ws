#!/usr/bin/env python3
import bisect
import csv
import json
import math
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG = "/home/ubuntu/nav_drift_test/nav_drift_test14"
OUT_DIR = Path("workspace_archive/debug_monitor/nav_drift_test14_analysis")
TZ = ZoneInfo("Asia/Shanghai")

TOPICS = {
    "/rosout",
    "/navigation/status",
    "/localization/ndt_status",
    "/localization/recovery_status",
    "/localization/recovery_requests",
    "/initialpose",
    "/pcl_pose",
    "/robot_realpose",
}


@dataclass
class Pose:
    x: float
    y: float
    z: float
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


@dataclass
class JumpLog:
    t: float
    action: str
    trans: float | None
    yaw: float | None
    fitness: float | None
    line: str
    ndt: dict | None = None


@dataclass
class JumpCluster:
    start_t: float
    end_t: float
    logs: list[JumpLog] = field(default_factory=list)
    waypoint: str = ""
    phase: str = ""
    before_pose: Pose | None = None
    after_pose: Pose | None = None
    candidate_pose: Pose | None = None
    candidate_before_pose: Pose | None = None
    candidate_time: float | None = None
    outcome: str = ""


def yaw_from_q(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_from_msg(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return Pose(p.x, p.y, p.z, yaw_from_q(q))


def pose_from_status(prefix, status):
    try:
        return Pose(
            float(status[f"{prefix}_x"]),
            float(status[f"{prefix}_y"]),
            0.0,
            float(status[f"{prefix}_yaw"]),
        )
    except Exception:
        return None


def fmt_t(t):
    if t is None:
        return "-"
    return datetime.fromtimestamp(t, TZ).strftime("%H:%M:%S.%f")[:-3]


def fmt_pose(p):
    if p is None:
        return "-"
    return f"({p.x:.3f}, {p.y:.3f}, yaw={math.degrees(p.yaw):.1f}deg)"


def pose_dist(a, b):
    if a is None or b is None:
        return None
    return math.hypot(a.x - b.x, a.y - b.y)


def read_bag():
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {topic: get_message(topic_types[topic]) for topic in TOPICS if topic in topic_types}

    nav_msgs = []
    rosout = []
    ndt_status = []
    recovery_status = []
    recovery_requests = []
    pcl_poses = []
    robot_poses = []
    initialposes = []

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_types:
            continue
        t = t_ns / 1e9
        msg = deserialize_message(data, msg_types[topic])

        if topic == "/navigation/status":
            try:
                nav_msgs.append((t, json.loads(msg.data)))
            except Exception:
                pass
        elif topic == "/rosout":
            rosout.append((t, msg.name, msg.msg))
        elif topic == "/localization/ndt_status":
            try:
                obj = json.loads(msg.data)
                obj["_bag_t"] = t
                ndt_status.append(obj)
            except Exception:
                pass
        elif topic == "/localization/recovery_status":
            try:
                recovery_status.append((t, json.loads(msg.data)))
            except Exception:
                recovery_status.append((t, {"raw": msg.data}))
        elif topic == "/localization/recovery_requests":
            try:
                recovery_requests.append((t, json.loads(msg.data)))
            except Exception:
                recovery_requests.append((t, {"raw": msg.data}))
        elif topic == "/pcl_pose":
            pcl_poses.append((t, pose_from_msg(msg)))
        elif topic == "/robot_realpose":
            robot_poses.append((t, pose_from_msg(msg)))
        elif topic == "/initialpose":
            initialposes.append((t, pose_from_msg(msg)))

    return {
        "nav": nav_msgs,
        "rosout": rosout,
        "ndt": ndt_status,
        "recovery_status": recovery_status,
        "recovery_requests": recovery_requests,
        "pcl": pcl_poses,
        "robot": robot_poses,
        "initialposes": initialposes,
    }


def nearest_before(series, t):
    times = [x[0] for x in series]
    idx = bisect.bisect_right(times, t) - 1
    if idx < 0:
        return None
    return series[idx][1]


def nearest_after(series, t):
    times = [x[0] for x in series]
    idx = bisect.bisect_left(times, t)
    if idx >= len(series):
        return None
    return series[idx][1]


def nearest_status(statuses, t, max_dt=0.25):
    times = [s["_bag_t"] for s in statuses]
    idx = bisect.bisect_left(times, t)
    candidates = []
    for j in (idx - 1, idx, idx + 1):
        if 0 <= j < len(statuses):
            candidates.append(statuses[j])
    if not candidates:
        return None
    best = min(candidates, key=lambda s: abs(s["_bag_t"] - t))
    return best if abs(best["_bag_t"] - t) <= max_dt else None


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


JUMP_RE = re.compile(
    r"(?P<action>Holding|Accepting|Rejecting).*?NDT pose jump.*?"
    r"translation=(?P<trans>[0-9.]+).*?yaw=(?P<yaw>[0-9.]+).*?fitness=(?P<fitness>[0-9.]+)"
)


def parse_jump_logs(rosout, ndt_status):
    logs = []
    for t, name, msg in rosout:
        if name != "lidar_localization":
            continue
        m = JUMP_RE.search(msg)
        if not m:
            continue
        action = m.group("action").lower()
        log = JumpLog(
            t=t,
            action=action,
            trans=float(m.group("trans")),
            yaw=float(m.group("yaw")),
            fitness=float(m.group("fitness")),
            line=msg,
            ndt=nearest_status(ndt_status, t),
        )
        logs.append(log)
    return logs


def cluster_logs(jump_logs):
    clusters = []
    current = None
    for log in jump_logs:
        if current is None or log.t - current.end_t > 1.5:
            current = JumpCluster(start_t=log.t, end_t=log.t, logs=[log])
            clusters.append(current)
        else:
            current.logs.append(log)
            current.end_t = log.t
    return clusters


def waypoint_for_time(waypoints, t):
    for wp in waypoints:
        end = wp.next_start_t if wp.next_start_t is not None else float("inf")
        if wp.start_t <= t < end:
            return wp
    return None


def finalize_clusters(clusters, waypoints, pcl_poses):
    for c in clusters:
        wp = waypoint_for_time(waypoints, c.start_t)
        c.waypoint = wp.name if wp else "非导航窗口"
        if wp is None:
            c.phase = "非导航"
        elif wp.reach_t and c.start_t > wp.reach_t:
            c.phase = "到达后/下个点前"
        elif wp.failure_t and c.start_t >= wp.failure_t:
            c.phase = "失败后"
        else:
            c.phase = "导航中"

        c.before_pose = nearest_before(pcl_poses, c.start_t - 0.005)
        has_accept = any(log.action == "accepting" for log in c.logs)
        has_reject = any(log.action == "rejecting" for log in c.logs)
        if has_accept:
            last_accept_t = max(log.t for log in c.logs if log.action == "accepting")
            c.after_pose = nearest_after(pcl_poses, last_accept_t)
            c.outcome = "成功接受"
        elif has_reject:
            # For rejected jumps, the published pose is intentionally frozen. Show the NDT candidate as "after".
            rejected = [log for log in c.logs if log.action == "rejecting"]
            max_reject = max(rejected, key=lambda log: log.trans or 0.0)
            status = max_reject.ndt if max_reject.ndt else None
            c.candidate_pose = pose_from_status("ndt_candidate", status or {})
            c.candidate_time = max_reject.t
            c.candidate_before_pose = nearest_before(pcl_poses, max_reject.t - 0.005)
            c.after_pose = nearest_after(pcl_poses, c.end_t)
            c.outcome = "被 jump 拒绝"
        else:
            c.after_pose = nearest_after(pcl_poses, c.end_t)
            c.outcome = "仅 hold，未形成最终接受/拒绝"


def summarize_by_waypoint(waypoints, clusters, ndt_status):
    rows = []
    for wp in waypoints:
        end = wp.next_start_t if wp.next_start_t is not None else float("inf")
        strict_end = wp.failure_t or wp.reach_t or end
        wp_clusters = [c for c in clusters if wp.start_t <= c.start_t < end]
        nav_clusters = [c for c in clusters if wp.start_t <= c.start_t <= strict_end]
        status_in_window = [s for s in ndt_status if wp.start_t <= s["_bag_t"] < end]
        rejected_status = [s for s in status_in_window if s.get("state") != "accepted"]
        accepted = sum(1 for c in wp_clusters if c.outcome == "成功接受")
        rejected_clusters = sum(1 for c in wp_clusters if c.outcome == "被 jump 拒绝")
        rejected_frames = sum(1 for c in wp_clusters for log in c.logs if log.action == "rejecting")
        max_trans = max([log.trans for c in wp_clusters for log in c.logs if log.trans is not None] or [0.0])
        max_fit = max([log.fitness for c in wp_clusters for log in c.logs if log.fitness is not None] or [0.0])
        rows.append(
            {
                "waypoint": wp.name,
                "id": wp.waypoint_id,
                "goal": wp.goal,
                "start": wp.start_t,
                "reach": wp.reach_t,
                "complete": wp.complete_t,
                "failure": wp.failure_t,
                "failure_reason": wp.failure_reason,
                "has_jump": bool(wp_clusters),
                "during_nav_jump_clusters": len(nav_clusters),
                "accepted_jump_clusters": accepted,
                "rejected_jump_clusters": rejected_clusters,
                "rejected_jump_frames": rejected_frames,
                "nonaccepted_ndt_status_frames": len(rejected_status),
                "max_jump_translation": max_trans,
                "max_jump_fitness": max_fit,
                "navigation_result": "失败/暂停未完成" if wp.failure_t or not wp.reach_t else "成功到达",
            }
        )
    return rows


def write_outputs(data):
    waypoints = parse_waypoints(data["nav"])
    jumps = parse_jump_logs(data["rosout"], data["ndt"])
    clusters = cluster_logs(jumps)
    finalize_clusters(clusters, waypoints, data["pcl"])
    summary_rows = summarize_by_waypoint(waypoints, clusters, data["ndt"])

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    with (OUT_DIR / "waypoint_ndt_summary.csv").open("w", newline="") as f:
        fieldnames = [
            "waypoint",
            "id",
            "start_time",
            "reach_time",
            "result",
            "has_jump",
            "accepted_jump_clusters",
            "rejected_jump_clusters",
            "rejected_jump_frames",
            "nonaccepted_ndt_status_frames",
            "max_jump_translation_m",
            "max_jump_fitness",
            "goal",
            "failure_reason",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in summary_rows:
            writer.writerow(
                {
                    "waypoint": r["waypoint"],
                    "id": r["id"],
                    "start_time": fmt_t(r["start"]),
                    "reach_time": fmt_t(r["reach"] or r["failure"]),
                    "result": r["navigation_result"],
                    "has_jump": "yes" if r["has_jump"] else "no",
                    "accepted_jump_clusters": r["accepted_jump_clusters"],
                    "rejected_jump_clusters": r["rejected_jump_clusters"],
                    "rejected_jump_frames": r["rejected_jump_frames"],
                    "nonaccepted_ndt_status_frames": r["nonaccepted_ndt_status_frames"],
                    "max_jump_translation_m": f"{r['max_jump_translation']:.3f}",
                    "max_jump_fitness": f"{r['max_jump_fitness']:.3f}",
                    "goal": r["goal"],
                    "failure_reason": r["failure_reason"],
                }
            )

    with (OUT_DIR / "jump_clusters.csv").open("w", newline="") as f:
        fieldnames = [
            "waypoint",
            "phase",
            "start_time",
            "end_time",
            "outcome",
            "log_count",
            "holding_count",
            "accepting_count",
            "rejecting_count",
            "max_translation_m",
            "max_yaw_rad",
            "max_fitness",
            "before_pose",
            "after_published_pose",
            "rejected_candidate_pose",
            "rejected_candidate_time",
            "pose_before_rejected_candidate",
            "published_pose_shift_m",
            "first_log",
            "last_log",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for c in clusters:
            writer.writerow(
                {
                    "waypoint": c.waypoint,
                    "phase": c.phase,
                    "start_time": fmt_t(c.start_t),
                    "end_time": fmt_t(c.end_t),
                    "outcome": c.outcome,
                    "log_count": len(c.logs),
                    "holding_count": sum(1 for log in c.logs if log.action == "holding"),
                    "accepting_count": sum(1 for log in c.logs if log.action == "accepting"),
                    "rejecting_count": sum(1 for log in c.logs if log.action == "rejecting"),
                    "max_translation_m": f"{max(log.trans for log in c.logs if log.trans is not None):.3f}",
                    "max_yaw_rad": f"{max(log.yaw for log in c.logs if log.yaw is not None):.3f}",
                    "max_fitness": f"{max(log.fitness for log in c.logs if log.fitness is not None):.3f}",
                    "before_pose": fmt_pose(c.before_pose),
                    "after_published_pose": fmt_pose(c.after_pose),
                    "rejected_candidate_pose": fmt_pose(c.candidate_pose),
                    "rejected_candidate_time": fmt_t(c.candidate_time),
                    "pose_before_rejected_candidate": fmt_pose(c.candidate_before_pose),
                    "published_pose_shift_m": "-" if pose_dist(c.before_pose, c.after_pose) is None else f"{pose_dist(c.before_pose, c.after_pose):.3f}",
                    "first_log": c.logs[0].line,
                    "last_log": c.logs[-1].line,
                }
            )

    with (OUT_DIR / "initialposes.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "pose"])
        for t, pose in data["initialposes"]:
            writer.writerow([fmt_t(t), fmt_pose(pose)])

    with (OUT_DIR / "recovery_events.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "event_type", "reason", "raw"])
        for t, obj in data["recovery_status"]:
            writer.writerow([fmt_t(t), obj.get("event_type", ""), obj.get("reason", ""), json.dumps(obj, ensure_ascii=False)])

    print("WAYPOINT SUMMARY")
    for r in summary_rows:
        reach_or_fail = r["reach"] or r["failure"]
        print(
            f"{r['waypoint']} {fmt_t(r['start'])}->{fmt_t(reach_or_fail)} "
            f"{r['navigation_result']} jump={r['has_jump']} "
            f"accept={r['accepted_jump_clusters']} reject_clusters={r['rejected_jump_clusters']} "
            f"reject_frames={r['rejected_jump_frames']} max_jump={r['max_jump_translation']:.3f}m"
        )

    print("\nJUMP CLUSTERS")
    for c in clusters:
        print(
            f"{fmt_t(c.start_t)}-{fmt_t(c.end_t)} {c.waypoint} {c.phase} {c.outcome} "
            f"logs={len(c.logs)} hold={sum(1 for log in c.logs if log.action == 'holding')} "
            f"accept={sum(1 for log in c.logs if log.action == 'accepting')} "
            f"reject={sum(1 for log in c.logs if log.action == 'rejecting')} "
            f"max_trans={max(log.trans for log in c.logs if log.trans is not None):.3f} "
            f"before={fmt_pose(c.before_pose)} after={fmt_pose(c.after_pose)} candidate={fmt_pose(c.candidate_pose)}"
        )


def main():
    data = read_bag()
    write_outputs(data)


if __name__ == "__main__":
    main()
