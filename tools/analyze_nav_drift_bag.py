#!/usr/bin/env python3
import argparse
import json
import math
import re
from collections import Counter, defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/robot_realpose",
    "/odom",
    "/prior_localization/odom",
    "/prior_localization/confidence",
    "/localization/prior_map_odom_bridge_status",
    "/navigation/status",
    "/plan",
    "/cmd_vel",
    "/tf",
}


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def pose_xy_yaw(pose):
    p = pose.position
    return p.x, p.y, yaw_from_quat(pose.orientation)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def xy(sample):
    return (sample[1], sample[2])


def safe_json(s):
    try:
        return json.loads(s)
    except Exception:
        return None


def open_reader(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)
    return reader


def load_messages(bag_path):
    reader = open_reader(bag_path)
    type_map = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
        if t.name in TOPICS
    }
    msg_types = {topic: get_message(type_name) for topic, type_name in type_map.items()}
    data = defaultdict(list)
    start = None
    end = None
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in msg_types:
            continue
        t = t_ns / 1e9
        start = t if start is None else min(start, t)
        end = t if end is None else max(end, t)
        msg = deserialize_message(raw, msg_types[topic])
        if topic == "/robot_realpose":
            x, y, yaw = pose_xy_yaw(msg.pose.pose)
            data[topic].append((t, x, y, yaw))
        elif topic in ("/odom", "/prior_localization/odom"):
            x, y, yaw = pose_xy_yaw(msg.pose.pose)
            data[topic].append((t, x, y, yaw))
        elif topic == "/prior_localization/confidence":
            data[topic].append((t, float(msg.data)))
        elif topic in ("/localization/prior_map_odom_bridge_status", "/navigation/status"):
            data[topic].append((t, msg.data, safe_json(msg.data)))
        elif topic == "/cmd_vel":
            data[topic].append((t, msg.linear.x, msg.linear.y, msg.angular.z))
        elif topic == "/plan":
            poses = []
            for ps in msg.poses:
                poses.append(pose_xy_yaw(ps.pose))
            length = sum(dist(poses[i - 1], poses[i]) for i in range(1, len(poses)))
            first = poses[0] if poses else None
            last = poses[-1] if poses else None
            data[topic].append((t, len(poses), length, first, last))
        elif topic == "/tf":
            for tr in msg.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    q = tr.transform.rotation
                    data["/tf:map_odom"].append((
                        t,
                        tr.transform.translation.x,
                        tr.transform.translation.y,
                        yaw_from_quat(q),
                    ))
    return data, start, end


def parse_log(log_path):
    segs = []
    accepted = []
    rejected = []
    jumps = []
    recover = []
    begin_re = re.compile(r"\[(\d+\.\d+)\].*开始导航到路点: (点位\d+)")
    bt_re = re.compile(
        r"\[(\d+\.\d+)\].*Begin navigating from current location "
        r"\(([-0-9.]+), ([-0-9.]+)\) to \(([-0-9.]+), ([-0-9.]+)\)"
    )
    done_re = re.compile(r"\[(\d+\.\d+)\].*Nav2确认到达路点: (点位\d+)")
    acc_re = re.compile(r"\[(\d+\.\d+)\].*ACCEPTED ([^ ]+).*map_odom_xy_norm=([-0-9.]+).*yaw=([-0-9.]+)")
    rej_re = re.compile(r"\[(\d+\.\d+)\].*REJECTED ([^ ]+)")
    current = None
    with open(log_path, "r", errors="replace") as f:
        for line in f:
            if m := begin_re.search(line):
                current = {
                    "name": m.group(2),
                    "start": float(m.group(1)),
                    "bt_start": None,
                    "from": None,
                    "goal": None,
                    "end": None,
                }
                segs.append(current)
            if m := bt_re.search(line):
                if current is not None and current["bt_start"] is None:
                    current["bt_start"] = float(m.group(1))
                    current["from"] = (float(m.group(2)), float(m.group(3)))
                    current["goal"] = (float(m.group(4)), float(m.group(5)))
            if m := done_re.search(line):
                for seg in reversed(segs):
                    if seg["name"] == m.group(2) and seg["end"] is None:
                        seg["end"] = float(m.group(1))
                        break
            if m := acc_re.search(line):
                accepted.append((float(m.group(1)), m.group(2), float(m.group(3)), float(m.group(4))))
            if m := rej_re.search(line):
                rejected.append((float(m.group(1)), m.group(2)))
            if "位姿跳变" in line:
                ts = re.search(r"\[(\d+\.\d+)\]", line)
                jumps.append((float(ts.group(1)) if ts else None, line.strip()))
            if "localization_" in line or "定位异常" in line or "hdl_fallback" in line:
                ts = re.search(r"\[(\d+\.\d+)\]", line)
                recover.append((float(ts.group(1)) if ts else None, line.strip()))
    return segs, accepted, rejected, jumps, recover


def slice_series(series, start, end):
    return [v for v in series if start <= v[0] <= end]


def jump_stats(series):
    jumps = []
    max_step = (0.0, None)
    max_yaw = (0.0, None)
    for prev, cur in zip(series, series[1:]):
        dt = cur[0] - prev[0]
        if dt <= 0:
            continue
        dxy = math.hypot(cur[1] - prev[1], cur[2] - prev[2])
        dyaw = abs(angle_diff(cur[3], prev[3]))
        speed = dxy / dt
        if dxy > max_step[0]:
            max_step = (dxy, (prev, cur, speed))
        if dyaw > max_yaw[0]:
            max_yaw = (dyaw, (prev, cur))
        if dxy > 0.35 or speed > 2.0 or dyaw > 0.8:
            jumps.append((cur[0], dt, dxy, speed, dyaw))
    return jumps, max_step, max_yaw


def plan_stats(plans):
    out = []
    max_goal_shift = (0.0, None)
    max_start_shift = (0.0, None)
    max_len_shift = (0.0, None)
    for prev, cur in zip(plans, plans[1:]):
        if prev[4] and cur[4]:
            g = dist(prev[4], cur[4])
            if g > max_goal_shift[0]:
                max_goal_shift = (g, (prev, cur))
        if prev[3] and cur[3]:
            s = dist(prev[3], cur[3])
            if s > max_start_shift[0]:
                max_start_shift = (s, (prev, cur))
        l = abs(cur[2] - prev[2])
        if l > max_len_shift[0]:
            max_len_shift = (l, (prev, cur))
        if (prev[4] and cur[4] and dist(prev[4], cur[4]) > 0.2) or l > 1.0:
            out.append((cur[0], l, dist(prev[4], cur[4]) if prev[4] and cur[4] else None))
    return out, max_goal_shift, max_start_shift, max_len_shift


def nearest(series, t):
    if not series:
        return None
    best = min(series, key=lambda v: abs(v[0] - t))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--log", required=True)
    args = ap.parse_args()

    data, start, end = load_messages(args.bag)
    segs, accepted, rejected, log_jumps, recover = parse_log(args.log)

    print(f"bag_start={start:.3f} bag_end={end:.3f} duration={end - start:.1f}s")
    print("topic_counts=" + json.dumps({k: len(v) for k, v in data.items()}, ensure_ascii=False, sort_keys=True))
    print(f"log_segments={len(segs)} accepted={len(accepted)} rejected={len(rejected)} log_pose_jumps={len(log_jumps)}")

    if accepted:
        print("accepted_events:")
        for t, reason, xy_norm, yaw in accepted:
            print(f"  {t:.3f} {reason} xy_norm={xy_norm:.3f} yaw={yaw:.3f}")
    rej_by_reason = defaultdict(int)
    for _, reason in rejected:
        rej_by_reason[reason] += 1
    print("rejected_by_reason=" + json.dumps(dict(sorted(rej_by_reason.items())), ensure_ascii=False))
    if log_jumps:
        print("log_pose_jumps:")
        for t, line in log_jumps:
            print(f"  {t}: {line}")

    bridge_status = data["/localization/prior_map_odom_bridge_status"]
    nav_status = data["/navigation/status"]
    events = Counter()
    nav_events = Counter()
    fallback_msgs = []
    for t, s, obj in bridge_status:
        if isinstance(obj, dict):
            events[str(obj.get("event") or obj.get("state") or obj.get("status"))] += 1
            text = json.dumps(obj, ensure_ascii=False)
        else:
            text = s
        if "fallback" in text or "abnormal" in text or "异常" in text:
            fallback_msgs.append((t, text[:220]))
    for t, s, obj in nav_status:
        if isinstance(obj, dict):
            nav_events[str(obj.get("event_type") or obj.get("status") or obj.get("type"))] += 1
            text = json.dumps(obj, ensure_ascii=False)
        else:
            text = s
        if "fallback" in text or "定位异常" in text or "localization" in text or "system_exception" in text:
            fallback_msgs.append((t, text[:220]))
    print("bridge_status_events=" + json.dumps(dict(events.most_common(20)), ensure_ascii=False))
    print("nav_status_events=" + json.dumps(dict(nav_events.most_common(20)), ensure_ascii=False))
    print(f"fallback_or_localization_status_msgs={len(fallback_msgs)}")
    for t, text in fallback_msgs[:20]:
        print(f"  status {t:.3f}: {text}")

    print("segments:")
    for seg in segs:
        s = seg["bt_start"] or seg["start"]
        e = seg["end"] or s
        rp = slice_series(data["/robot_realpose"], max(start, s - 0.3), min(end, e + 0.3))
        mo = slice_series(data["/tf:map_odom"], max(start, s - 0.3), min(end, e + 0.3))
        od = slice_series(data["/odom"], max(start, s - 0.3), min(end, e + 0.3))
        plans = slice_series(data["/plan"], max(start, s - 0.5), min(end, e + 0.5))
        cmd = slice_series(data["/cmd_vel"], max(start, s - 0.3), min(end, e + 0.3))
        rp_jumps, rp_max, rp_yaw = jump_stats(rp)
        mo_jumps, mo_max, mo_yaw = jump_stats(mo)
        od_jumps, od_max, _ = jump_stats(od)
        plan_jumps, pg, ps, pl = plan_stats(plans)
        acc = [a for a in accepted if s - 0.5 <= a[0] <= e + 0.5]
        rej = [r for r in rejected if s - 0.5 <= r[0] <= e + 0.5]
        cmd_max = max((math.hypot(c[1], c[2]) for c in cmd), default=0.0)
        cmd_ang = max((abs(c[3]) for c in cmd), default=0.0)
        n0 = nearest(rp, s)
        n1 = nearest(rp, e)
        actual = dist(xy(n0), xy(n1)) if n0 and n1 else 0.0
        goal_err = dist(xy(n1), seg["goal"]) if n1 and seg["goal"] else None
        print(
            f"  {seg['name']} {s:.3f}->{e:.3f} dur={e-s:.1f}s "
            f"from={seg['from']} goal={seg['goal']} actual={actual:.2f} "
            f"goal_err={goal_err if goal_err is not None else 'NA'} "
            f"realpose_max_step={rp_max[0]:.3f} realpose_jump_count={len(rp_jumps)} "
            f"map_odom_max_step={mo_max[0]:.3f} map_odom_jump_count={len(mo_jumps)} "
            f"odom_max_step={od_max[0]:.3f} plan_count={len(plans)} "
            f"plan_goal_shift_max={pg[0]:.3f} plan_start_shift_max={ps[0]:.3f} "
            f"plan_len_shift_max={pl[0]:.3f} cmd_max={cmd_max:.2f} cmd_ang_max={cmd_ang:.2f} "
            f"accepted={len(acc)} rejected={len(rej)}"
        )
        for jt in rp_jumps[:5]:
            print(f"    realpose_jump t={jt[0]:.3f} dt={jt[1]:.3f} dxy={jt[2]:.3f} speed={jt[3]:.2f} dyaw={jt[4]:.3f}")
        for jt in mo_jumps[:5]:
            print(f"    map_odom_jump t={jt[0]:.3f} dt={jt[1]:.3f} dxy={jt[2]:.3f} speed={jt[3]:.2f} dyaw={jt[4]:.3f}")
        for jt in plan_jumps[:5]:
            print(f"    plan_jump t={jt[0]:.3f} len_delta={jt[1]:.3f} goal_shift={jt[2]}")


if __name__ == "__main__":
    main()
