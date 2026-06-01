#!/usr/bin/env python3
import json
import math
import re
import sys
from collections import Counter, defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG = "/home/ubuntu/nav_drift_test/nav_drift_test14"


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


def yaw_from_q(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def pose_tuple(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (p.x, p.y, p.z, yaw_from_q(q))


def fmt_pose(p):
    if p is None:
        return "-"
    return f"x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}, yaw={math.degrees(p[3]):.1f}deg"


def reader_for(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {topic: get_message(topic_types[topic]) for topic in TOPICS if topic in topic_types}
    return reader, msg_types


def main():
    bag = sys.argv[1] if len(sys.argv) > 1 else BAG
    reader, msg_types = reader_for(bag)

    counts = Counter()
    samples = defaultdict(list)
    rosout_hits = []
    nav_events = []
    ndt_status_samples = []
    recovery_events = []
    initialposes = []
    pcl_poses = []
    robot_poses = []

    interesting = re.compile(
        r"Rejecting NDT pose jump|Accepting confirmed NDT pose jump|Holding NDT pose jump|"
        r"Republishing last good map->odom|fitness score is over|initialpose input|Converted initialpose|"
        r"定位异常|定位恢复|navigation_localization|开始导航到路点|Nav2确认到达路点|navigation_completed|"
        r"navigation_started|navigation_failed|Goal canceled|Failed|failed|recovery failed|relocalize|"
        r"pose jump",
        re.I,
    )

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        t = t_ns / 1e9
        counts[topic] += 1
        if len(samples[topic]) < 5:
            samples[topic].append((t, str(msg)[:800].replace("\n", "\\n")))

        if topic == "/rosout":
            line = msg.msg
            if interesting.search(line):
                rosout_hits.append((t, msg.name, line))
        elif topic == "/navigation/status":
            data_s = msg.data
            if len(samples[topic]) < 20:
                samples[topic].append((t, data_s[:800]))
            try:
                obj = json.loads(data_s)
            except Exception:
                obj = None
            nav_events.append((t, data_s, obj))
        elif topic == "/localization/ndt_status":
            if len(ndt_status_samples) < 30 or "jump" in msg.data.lower() or "reject" in msg.data.lower():
                ndt_status_samples.append((t, msg.data))
        elif topic in {"/localization/recovery_status", "/localization/recovery_requests"}:
            recovery_events.append((t, topic, msg.data))
        elif topic == "/initialpose":
            initialposes.append((t, pose_tuple(msg)))
        elif topic == "/pcl_pose":
            pcl_poses.append((t, pose_tuple(msg)))
        elif topic == "/robot_realpose":
            robot_poses.append((t, pose_tuple(msg)))

    print("COUNTS")
    for topic, count in sorted(counts.items()):
        print(f"{topic}: {count}")

    print("\nSAMPLES")
    for topic in sorted(samples):
        print(f"## {topic}")
        for t, sample in samples[topic][:5]:
            print(f"{t:.3f}: {sample}")

    print("\nINITIALPOSES")
    for t, p in initialposes:
        print(f"{t:.3f}: {fmt_pose(p)}")

    print("\nNDT_STATUS_SAMPLES")
    for t, s in ndt_status_samples[:200]:
        print(f"{t:.3f}: {s}")

    print("\nRECOVERY_EVENTS")
    for t, topic, s in recovery_events:
        print(f"{t:.3f} {topic}: {s}")

    print("\nROSOUT_HITS")
    for t, name, line in rosout_hits[:1000]:
        print(f"{t:.3f} [{name}]: {line}")

    print("\nNAV_EVENTS_FILTERED")
    for t, data_s, obj in nav_events:
        lower = data_s.lower()
        if any(k in lower for k in ["waypoint", "navigation", "failed", "success", "paused", "recovered", "localization"]):
            print(f"{t:.3f}: {data_s}")


if __name__ == "__main__":
    main()
