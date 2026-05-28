#!/usr/bin/env python3
import json
import math
import sys
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/initialpose",
    "/localization/ndt_status",
    "/localization/recovery_requests",
    "/localization/recovery_status",
    "/pcl_pose",
    "/robot_realpose",
    "/hdl_bootstrap/odom",
    "/status",
}


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_tuple(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (p.x, p.y, p.z, yaw_from_q(q))


def odom_pose_tuple(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (p.x, p.y, p.z, yaw_from_q(q))


def fmt_pose(p):
    return f"x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f} yaw={math.degrees(p[3]):.1f}deg"


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} BAG_DIR_OR_URI", file=sys.stderr)
        return 2
    uri = sys.argv[1]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {topic: get_message(type_map[topic]) for topic in TOPICS if topic in type_map}

    initialposes = []
    recovery_requests = []
    recovery_status = []
    hdl_odom = []
    hdl_status = []
    ndt_counter = Counter()
    ndt_reason_counter = Counter()
    ndt_samples = []
    ndt_bad_runs = []
    current_bad = None
    pcl_first = pcl_last = None
    robot_first = robot_last = None
    robot_samples = []
    pcl_samples = []

    start_ns = None
    end_ns = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if start_ns is None:
            start_ns = t_ns
        end_ns = t_ns
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        sec = t_ns / 1e9
        rel = (t_ns - start_ns) / 1e9

        if topic == "/initialpose":
            initialposes.append((sec, rel, pose_tuple(msg), msg.header.frame_id))
        elif topic == "/pcl_pose":
            p = pose_tuple(msg)
            if pcl_first is None:
                pcl_first = (sec, rel, p, msg.header.frame_id)
            pcl_last = (sec, rel, p, msg.header.frame_id)
            if len(pcl_samples) == 0 or rel - pcl_samples[-1][1] >= 5.0:
                pcl_samples.append((sec, rel, p, msg.header.frame_id))
        elif topic == "/robot_realpose":
            p = pose_tuple(msg)
            if robot_first is None:
                robot_first = (sec, rel, p, msg.header.frame_id)
            robot_last = (sec, rel, p, msg.header.frame_id)
            if len(robot_samples) == 0 or rel - robot_samples[-1][1] >= 5.0:
                robot_samples.append((sec, rel, p, msg.header.frame_id))
        elif topic == "/hdl_bootstrap/odom":
            hdl_odom.append((sec, rel, odom_pose_tuple(msg), msg.header.frame_id, msg.child_frame_id))
        elif topic == "/status":
            hdl_status.append((sec, rel, getattr(msg, "matching_error", None), getattr(msg, "inlier_fraction", None)))
        elif topic == "/localization/recovery_requests":
            try:
                payload = json.loads(msg.data)
            except Exception:
                payload = {"raw": msg.data}
            recovery_requests.append((sec, rel, payload))
        elif topic == "/localization/recovery_status":
            try:
                payload = json.loads(msg.data)
            except Exception:
                payload = {"raw": msg.data}
            recovery_status.append((sec, rel, payload))
        elif topic == "/localization/ndt_status":
            try:
                payload = json.loads(msg.data)
            except Exception:
                continue
            state = payload.get("state", "")
            reason = payload.get("reason", "")
            ndt_counter[state] += 1
            ndt_reason_counter[(state, reason)] += 1
            if len(ndt_samples) == 0 or rel - ndt_samples[-1][1] >= 5.0 or state != ndt_samples[-1][2].get("state"):
                ndt_samples.append((sec, rel, payload))
            bad = state != "accepted"
            if bad and current_bad is None:
                current_bad = [sec, rel, sec, rel, Counter()]
            if bad:
                current_bad[2] = sec
                current_bad[3] = rel
                current_bad[4][reason] += 1
            elif current_bad is not None:
                ndt_bad_runs.append(tuple(current_bad))
                current_bad = None
    if current_bad is not None:
        ndt_bad_runs.append(tuple(current_bad))

    print(f"bag_start={start_ns/1e9:.3f} bag_end={end_ns/1e9:.3f} duration={(end_ns-start_ns)/1e9:.1f}s")
    print("\ninitialpose:")
    for sec, rel, p, frame in initialposes:
        print(f"  t={sec:.3f} rel={rel:.1f}s frame={frame} {fmt_pose(p)}")

    print("\nNDT state counts:")
    for k, v in ndt_counter.most_common():
        print(f"  {k}: {v}")
    print("NDT reason counts:")
    for (state, reason), v in ndt_reason_counter.most_common(12):
        print(f"  {state}/{reason}: {v}")
    print("NDT bad runs >=2s:")
    for s_sec, s_rel, e_sec, e_rel, reasons in ndt_bad_runs:
        if e_rel - s_rel >= 2.0:
            print(f"  rel={s_rel:.1f}-{e_rel:.1f}s dur={e_rel-s_rel:.1f}s reasons={dict(reasons)}")

    print("\nrecovery_requests:")
    for sec, rel, payload in recovery_requests:
        print(f"  t={sec:.3f} rel={rel:.1f}s {payload}")
    print("\nrecovery_status events:")
    for sec, rel, payload in recovery_status:
        et = payload.get("event_type", payload.get("raw", ""))
        reason = payload.get("reason", "")
        src = payload.get("source", "")
        attempts = payload.get("relocalize_attempts", "")
        print(f"  rel={rel:.1f}s event={et} source={src} attempts={attempts} reason={reason}")

    print("\npose endpoints:")
    for name, item in [("pcl_first", pcl_first), ("pcl_last", pcl_last), ("robot_first", robot_first), ("robot_last", robot_last)]:
        if item:
            sec, rel, p, frame = item
            print(f"  {name}: rel={rel:.1f}s frame={frame} {fmt_pose(p)}")

    print("\nrobot_realpose samples every ~5s:")
    for sec, rel, p, frame in robot_samples[:120]:
        print(f"  rel={rel:.1f}s {fmt_pose(p)}")

    print("\npcl_pose samples every ~5s:")
    for sec, rel, p, frame in pcl_samples[:120]:
        print(f"  rel={rel:.1f}s {fmt_pose(p)}")

    print("\nHDL odom:")
    for sec, rel, p, frame, child in hdl_odom:
        print(f"  rel={rel:.1f}s {frame}->{child} {fmt_pose(p)}")
    print("\nHDL status:")
    for sec, rel, err, inlier in hdl_status:
        print(f"  rel={rel:.1f}s matching_error={err} inlier={inlier}")


if __name__ == "__main__":
    raise SystemExit(main())
