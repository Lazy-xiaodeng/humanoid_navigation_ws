#!/usr/bin/env python3
"""
离线回放 prior-map bridge 策略，不改生产节点。

这个脚本读取 bag 中已经录下来的:
  - /prior_localization/odom: open3d_loc 输出的 map->prior_open3d_base 候选
  - /prior_localization/open3d_input_odom: axis_adapter 输出的 odom->prior_open3d_base
  - /prior_localization/confidence: open3d_loc 置信度
  - /navigation/status: 当前导航状态，用来判断导航中 / SpinToPose

然后按 bridge 的公式重新计算:
  map->odom = map->prior_open3d_base * inverse(odom->prior_open3d_base)

最后同时模拟两套策略:
  1. current: 当前生产 bridge 的小修正/大修正确认逻辑
  2. protected: 导航中阻断大修正，空闲/讲解窗口允许稳定大修正找回

注意：这是离线策略验证，不会启动 Nav2 闭环，也不会写入真实 TF。
它能回答“哪些跳变会被拦住、哪些会在空闲窗口找回、是否会触发降级条件”，
但不能单独证明 Nav2 控制器在闭环中一定怎么走。
"""

import argparse
import json
import math
from bisect import bisect_left
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float32, String

from analyze_navtest15 import angle_diff, read_log, tf_jumps, yaw_from_quat
from humanoid_navigation2.prior_map_odom_bridge import (
    correction_delta,
    odom_msg_to_matrix,
    yaw_from_matrix,
)


def open_reader(uri: str):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, topic_types


def msg_stamp_sec(msg, fallback_ns: int) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return fallback_ns * 1e-9
    sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return sec if sec > 0.0 else fallback_ns * 1e-9


def pose_to_matrix_from_odom(msg: Odometry) -> np.ndarray:
    return odom_msg_to_matrix(msg)


def force_2d(transform: np.ndarray) -> np.ndarray:
    constrained = np.eye(4)
    yaw = yaw_from_matrix(transform)
    constrained[0, 0] = math.cos(yaw)
    constrained[0, 1] = -math.sin(yaw)
    constrained[1, 0] = math.sin(yaw)
    constrained[1, 1] = math.cos(yaw)
    constrained[0, 3] = transform[0, 3]
    constrained[1, 3] = transform[1, 3]
    constrained[2, 3] = 0.0
    return constrained


def matrix_to_xyyaw(t: float, mat: np.ndarray) -> Tuple[float, float, float, float]:
    return (t, float(mat[0, 3]), float(mat[1, 3]), yaw_from_matrix(mat))


def navigation_status(data: str) -> Tuple[bool, bool]:
    """返回 (navigation_active, turning)。"""
    try:
        status = json.loads(data)
    except (TypeError, ValueError):
        return ("TURNING" in data or "executing" in data), ("TURNING" in data and "SpinToPose" in data)

    current_state = str(status.get("current_state", "")).lower()
    detailed_state = str(status.get("current_detailed_state", status.get("detailed_state", ""))).upper()
    active = current_state in ("executing", "planning", "running", "active")
    return active, active and detailed_state == "TURNING"


def read_bag_inputs(uri: str) -> Dict[str, list]:
    wanted = {
        "/prior_localization/odom",
        "/prior_localization/open3d_input_odom",
        "/prior_localization/confidence",
        "/navigation/status",
        "/tf",
    }
    reader, topic_types = open_reader(uri)
    msg_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in wanted
    }

    events = []
    actual_tf = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        t = t_ns * 1e-9
        if topic == "/prior_localization/odom":
            events.append((t, "prior", msg_stamp_sec(msg, t_ns), pose_to_matrix_from_odom(msg)))
        elif topic == "/prior_localization/open3d_input_odom":
            events.append((t, "odom", msg_stamp_sec(msg, t_ns), pose_to_matrix_from_odom(msg)))
        elif topic == "/prior_localization/confidence":
            events.append((t, "confidence", t, float(msg.data)))
        elif topic == "/navigation/status":
            active, turning = navigation_status(msg.data)
            events.append((t, "nav_status", t, (active, turning)))
        elif topic == "/tf":
            for tr in msg.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    x = float(tr.transform.translation.x)
                    y = float(tr.transform.translation.y)
                    yaw = yaw_from_quat(tr.transform.rotation)
                    actual_tf.append((t, x, y, yaw))
    events.sort(key=lambda x: x[0])
    return {"events": events, "actual_tf": compress_xyyaw(actual_tf)}


def compress_xyyaw(samples: Iterable[Tuple[float, float, float, float]]) -> List[Tuple[float, float, float, float]]:
    out = []
    last = None
    for sample in samples:
        if last is None:
            out.append(sample)
            last = sample
            continue
        dx = math.hypot(sample[1] - last[1], sample[2] - last[2])
        dyaw = abs(angle_diff(sample[3], last[3]))
        if dx > 1e-4 or dyaw > 1e-4:
            out.append(sample)
            last = sample
    return out


class ReplayBridge:
    def __init__(self, name: str, protected: bool, args):
        self.name = name
        self.protected = protected
        self.args = args
        self.accepted: Optional[np.ndarray] = None
        self.accepted_samples = []
        self.events = []
        self.pending: Optional[np.ndarray] = None
        self.pending_count = 0
        self.pending_start_t: Optional[float] = None
        self.confidence: Optional[float] = None
        self.confidence_t: Optional[float] = None
        self.nav_active = False
        self.turning = False
        self.spin_until = 0.0

    def set_confidence(self, t: float, value: float):
        self.confidence = value
        self.confidence_t = t

    def set_navigation_status(self, t: float, active: bool, turning: bool):
        self.nav_active = active
        if turning:
            self.turning = True
            self.spin_until = max(self.spin_until, t + self.args.spin_settle)
        else:
            if self.turning:
                self.spin_until = max(self.spin_until, t + self.args.spin_settle)
            self.turning = False

    def confidence_ok(self, t: float) -> bool:
        if self.confidence is None or self.confidence_t is None:
            self.record(t, "reject", "no_confidence")
            return False
        age = t - self.confidence_t
        if age > self.args.confidence_timeout:
            self.record(t, "reject", "stale_confidence", confidence_age=age)
            return False
        if self.confidence < self.args.min_confidence:
            self.record(t, "reject", "low_confidence", confidence=self.confidence)
            return False
        return True

    def evaluate(self, t: float, candidate: np.ndarray):
        if not self.confidence_ok(t):
            return

        if t <= self.spin_until:
            self.pending = None
            self.pending_count = 0
            self.pending_start_t = None
            self.record(t, "reject", "spin_to_pose_freeze_tf")
            return

        if self.accepted is None:
            self.accept(t, candidate, "initial_pose")
            return

        dx, dyaw = correction_delta(candidate, self.accepted)
        if dx <= self.args.small_xy and dyaw <= self.args.small_yaw:
            self.accept(t, candidate, "small_correction", dx, dyaw)
            return

        if dx > self.args.max_large_xy or dyaw > self.args.max_large_yaw:
            self.reset_pending()
            self.record(t, "reject", "too_large", dx=dx, dyaw=dyaw)
            return

        if self.protected:
            self.evaluate_protected(t, candidate, dx, dyaw)
        else:
            self.evaluate_current(t, candidate, dx, dyaw)

    def evaluate_current(self, t: float, candidate: np.ndarray, dx: float, dyaw: float):
        count = self.update_pending(t, candidate)
        if count >= self.args.current_required_frames:
            self.accept(t, candidate, "confirmed_large_correction", dx, dyaw, pending_count=count)
        else:
            self.record(t, "pending", "large_correction", dx=dx, dyaw=dyaw, pending_count=count)

    def evaluate_protected(self, t: float, candidate: np.ndarray, dx: float, dyaw: float):
        if self.nav_active:
            if dx <= self.args.nav_medium_xy and dyaw <= self.args.nav_medium_yaw:
                count = self.update_pending(t, candidate)
                if count >= self.args.nav_medium_frames:
                    self.accept(t, candidate, "nav_medium_stable", dx, dyaw, pending_count=count)
                else:
                    self.record(t, "pending", "nav_medium_pending", dx=dx, dyaw=dyaw, pending_count=count)
                return

            count = self.update_pending(t, candidate)
            age = 0.0 if self.pending_start_t is None else t - self.pending_start_t
            action = "hold_degraded" if age >= self.args.degraded_after else "hold"
            self.record(t, action, "nav_large_blocked", dx=dx, dyaw=dyaw, pending_count=count, pending_age=age)
            return

        count = self.update_pending(t, candidate)
        if dx > self.args.idle_auto_accept_max_xy:
            age = 0.0 if self.pending_start_t is None else t - self.pending_start_t
            action = "hold_degraded" if age >= self.args.degraded_after else "hold"
            self.record(
                t,
                action,
                "idle_large_over_auto_accept_limit",
                dx=dx,
                dyaw=dyaw,
                pending_count=count,
                pending_age=age,
            )
            return

        if count >= self.args.idle_large_frames:
            age = 0.0 if self.pending_start_t is None else t - self.pending_start_t
            self.accept(t, candidate, "idle_large_stable", dx, dyaw, pending_count=count, pending_age=age)
        else:
            self.record(t, "pending", "idle_large_pending", dx=dx, dyaw=dyaw, pending_count=count)

    def update_pending(self, t: float, candidate: np.ndarray) -> int:
        if self.pending is None:
            self.pending = candidate
            self.pending_count = 1
            self.pending_start_t = t
            return self.pending_count

        spread_xy, spread_yaw = correction_delta(candidate, self.pending)
        if spread_xy <= self.args.stable_xy and spread_yaw <= self.args.stable_yaw:
            self.pending = candidate
            self.pending_count += 1
        else:
            self.pending = candidate
            self.pending_count = 1
            self.pending_start_t = t
        return self.pending_count

    def accept(self, t: float, candidate: np.ndarray, reason: str, dx: float = 0.0, dyaw: float = 0.0, **extra):
        self.accepted = candidate
        self.accepted_samples.append(matrix_to_xyyaw(t, candidate))
        self.reset_pending()
        self.record(t, "accept", reason, dx=dx, dyaw=dyaw, **extra)

    def reset_pending(self):
        self.pending = None
        self.pending_count = 0
        self.pending_start_t = None

    def record(self, t: float, action: str, reason: str, **extra):
        row = {"t": t, "policy": self.name, "action": action, "reason": reason, "nav_active": self.nav_active}
        row.update(extra)
        if self.accepted is not None:
            row["accepted_x"] = float(self.accepted[0, 3])
            row["accepted_y"] = float(self.accepted[1, 3])
            row["accepted_yaw"] = yaw_from_matrix(self.accepted)
        self.events.append(row)


def nearest_odom(odom_cache: List[Tuple[float, np.ndarray]], stamp: float, tolerance: float) -> Optional[np.ndarray]:
    if not odom_cache:
        return None
    times = [x[0] for x in odom_cache]
    idx = bisect_left(times, stamp)
    candidates = []
    if idx < len(odom_cache):
        candidates.append(odom_cache[idx])
    if idx > 0:
        candidates.append(odom_cache[idx - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda x: abs(x[0] - stamp))
    return best[1] if abs(best[0] - stamp) <= tolerance else None


def simulate(inputs: Dict[str, list], args) -> Dict[str, object]:
    current = ReplayBridge("current", False, args)
    protected = ReplayBridge("protected", True, args)
    bridges = [current, protected]
    odom_cache: List[Tuple[float, np.ndarray]] = []

    for bag_t, kind, stamp, payload in inputs["events"]:
        if kind == "odom":
            odom_cache.append((stamp, payload))
            cutoff = stamp - args.odom_cache_duration
            while odom_cache and odom_cache[0][0] < cutoff:
                odom_cache.pop(0)
        elif kind == "confidence":
            for bridge in bridges:
                bridge.set_confidence(bag_t, payload)
        elif kind == "nav_status":
            active, turning = payload
            for bridge in bridges:
                bridge.set_navigation_status(bag_t, active, turning)
        elif kind == "prior":
            odom = nearest_odom(odom_cache, stamp, args.odom_lookup_tolerance)
            if odom is None:
                for bridge in bridges:
                    bridge.record(bag_t, "reject", "missing_odom_cache")
                continue
            candidate = force_2d(payload @ np.linalg.inv(odom))
            for bridge in bridges:
                bridge.evaluate(bag_t, candidate)

    return {
        "current": current,
        "protected": protected,
    }


def label_for_time(t: float, segments: List[dict]) -> str:
    for seg in segments:
        if seg["start"] <= t <= seg["done"]:
            return seg["point"]
    return "idle"


def load_segments(args) -> List[dict]:
    if args.analysis_json:
        data = json.loads(Path(args.analysis_json).read_text(encoding="utf-8"))
        return [
            {
                "point": seg["point"],
                "start": seg["start"],
                "done": seg["done"],
                "failed": bool(seg.get("failed", False)),
            }
            for seg in data["segments"]
        ]

    _, segments = read_log(Path(args.log))
    return segments


def jump_stats(samples: List[Tuple[float, float, float, float]], start: float, end: float) -> Dict[str, object]:
    window = [s for s in samples if start <= s[0] <= end]
    jumps = tf_jumps(window)
    return {
        "max_jump": max([j["dx"] for j in jumps] or [0.0]),
        "big_jump_count": len([j for j in jumps if j["dx"] >= 0.5]),
    }


def interval_stats(samples: List[Tuple[float, float, float, float]], segments: List[dict]) -> List[dict]:
    rows = []
    for seg in segments:
        nav = jump_stats(samples, seg["start"] - 0.5, seg["done"])
        recovery = jump_stats(samples, seg["done"], seg["done"] + 6.0)
        whole = jump_stats(samples, seg["start"] - 0.5, seg["done"] + 6.0)
        rows.append({
            "point": seg["point"],
            "start": seg["start"],
            "done": seg["done"],
            "failed": bool(seg.get("failed", False)),
            "nav_max_jump": nav["max_jump"],
            "nav_big_jump_count": nav["big_jump_count"],
            "recovery_max_jump": recovery["max_jump"],
            "recovery_big_jump_count": recovery["big_jump_count"],
            "max_jump": whole["max_jump"],
            "big_jump_count": whole["big_jump_count"],
        })
    return rows


def summarize_policy(bridge: ReplayBridge, segments: List[dict]) -> Dict[str, object]:
    jumps = tf_jumps(bridge.accepted_samples)
    events = bridge.events
    by_point = defaultdict(list)
    for event in events:
        if event.get("dx", 0.0) >= 0.5 or event["action"] in ("hold_degraded", "reject"):
            by_point[label_for_time(event["t"], segments)].append(event)

    return {
        "accepted_samples": bridge.accepted_samples,
        "max_jump": max([j["dx"] for j in jumps] or [0.0]),
        "big_jump_count": len([j for j in jumps if j["dx"] >= 0.5]),
        "accept_count": len([e for e in events if e["action"] == "accept"]),
        "hold_count": len([e for e in events if e["action"] in ("hold", "hold_degraded")]),
        "degraded_count": len([e for e in events if e["action"] == "hold_degraded"]),
        "idle_large_accepts": [
            e for e in events
            if e["action"] == "accept" and e["reason"] == "idle_large_stable"
        ],
        "nav_large_accepts": [
            e for e in events
            if e["action"] == "accept" and e.get("nav_active") and e.get("dx", 0.0) > 0.5
        ],
        "segments": interval_stats(bridge.accepted_samples, segments),
        "notable_events_by_point": dict(by_point),
        "events": events,
    }


def render(report: Dict[str, object]) -> str:
    lines = [
        "# Experimental Bridge Replay Report",
        "",
        "## Summary",
        "",
        f"- actual recorded max jump: {report['actual']['max_jump']:.3f} m",
        f"- current simulated max jump: {report['current']['max_jump']:.3f} m",
        f"- protected simulated max jump: {report['protected']['max_jump']:.3f} m",
        f"- protected held candidates: {report['protected']['hold_count']}",
        f"- protected degraded events: {report['protected']['degraded_count']}",
        f"- protected idle large accepts: {len(report['protected']['idle_large_accepts'])}",
        f"- protected nav large accepts: {len(report['protected']['nav_large_accepts'])}",
        "",
        "## Per Point",
        "",
        "| point | actual nav max | current nav max | protected nav max | protected recovery max | protected nav >=0.5m | failed |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    actual_rows = {r["point"]: r for r in report["actual"]["segments"]}
    current_rows = {r["point"]: r for r in report["current"]["segments"]}
    protected_rows = {r["point"]: r for r in report["protected"]["segments"]}
    for point in actual_rows:
        a = actual_rows[point]
        c = current_rows.get(point, {})
        p = protected_rows.get(point, {})
        lines.append(
            f"| {point} | {a.get('nav_max_jump', 0.0):.3f} | {c.get('nav_max_jump', 0.0):.3f} | "
            f"{p.get('nav_max_jump', 0.0):.3f} | {p.get('recovery_max_jump', 0.0):.3f} | "
            f"{p.get('nav_big_jump_count', 0)} | {a.get('failed', False)} |"
        )

    lines.extend(["", "## Protected Idle Large Accepts", ""])
    for event in report["protected"]["idle_large_accepts"]:
        lines.append(
            f"- {event['t']:.3f} {label_for_time(event['t'], report['segments'])} "
            f"dx={event.get('dx', 0.0):.3f} yaw={event.get('dyaw', 0.0):.3f} "
            f"pending_age={event.get('pending_age', 0.0):.2f}s"
        )

    lines.extend(["", "## Protected Notable Events", ""])
    for point, events in report["protected"]["notable_events_by_point"].items():
        lines.append(f"### {point}")
        for event in events[:20]:
            lines.append(
                f"- {event['t']:.3f} {event['action']} {event['reason']} "
                f"dx={event.get('dx', 0.0):.3f} yaw={event.get('dyaw', 0.0):.3f} "
                f"age={event.get('pending_age', 0.0):.2f}s"
            )
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--log")
    parser.add_argument("--analysis-json")
    parser.add_argument("--out", required=True)
    parser.add_argument("--min-confidence", type=float, default=0.50)
    parser.add_argument("--confidence-timeout", type=float, default=2.0)
    parser.add_argument("--odom-cache-duration", type=float, default=5.0)
    parser.add_argument("--odom-lookup-tolerance", type=float, default=0.05)
    parser.add_argument("--spin-settle", type=float, default=3.0)
    parser.add_argument("--small-xy", type=float, default=0.25)
    parser.add_argument("--small-yaw", type=float, default=0.12)
    parser.add_argument("--max-large-xy", type=float, default=3.0)
    parser.add_argument("--max-large-yaw", type=float, default=1.2)
    parser.add_argument("--stable-xy", type=float, default=0.25)
    parser.add_argument("--stable-yaw", type=float, default=0.10)
    parser.add_argument("--current-required-frames", type=int, default=3)
    parser.add_argument("--nav-medium-xy", type=float, default=0.50)
    parser.add_argument("--nav-medium-yaw", type=float, default=0.12)
    parser.add_argument("--nav-medium-frames", type=int, default=6)
    parser.add_argument("--idle-large-frames", type=int, default=3)
    parser.add_argument("--idle-auto-accept-max-xy", type=float, default=1.0)
    parser.add_argument("--degraded-after", type=float, default=3.0)
    args = parser.parse_args()

    if not args.log and not args.analysis_json:
        parser.error("必须提供 --log 或 --analysis-json")

    segments = load_segments(args)
    inputs = read_bag_inputs(args.bag)
    sim = simulate(inputs, args)
    actual_jumps = tf_jumps(inputs["actual_tf"])
    report = {
        "bag": args.bag,
        "log": args.log,
        "params": vars(args),
        "segments": segments,
        "actual": {
            "max_jump": max([j["dx"] for j in actual_jumps] or [0.0]),
            "big_jump_count": len([j for j in actual_jumps if j["dx"] >= 0.5]),
            "segments": interval_stats(inputs["actual_tf"], segments),
        },
        "current": summarize_policy(sim["current"], segments),
        "protected": summarize_policy(sim["protected"], segments),
    }

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "experimental_bridge_replay.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (out_dir / "experimental_bridge_replay.md").write_text(render(report), encoding="utf-8")
    print(f"wrote {out_dir / 'experimental_bridge_replay.json'}")
    print(f"wrote {out_dir / 'experimental_bridge_replay.md'}")


if __name__ == "__main__":
    main()
