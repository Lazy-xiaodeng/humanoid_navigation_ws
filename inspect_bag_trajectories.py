#!/usr/bin/env python3
from collections import Counter
from math import hypot
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG = Path("/home/ubuntu/nav_drift_test/nav_drift_test10/nav_drift_test10_0.mcap")
TOPICS = ["/odom", "/pcl_pose", "/robot_realpose", "/tf"]


def xy_from_pose(msg):
    p = msg.pose.pose.position
    return float(p.x), float(p.y)


reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=str(BAG), storage_id="mcap"),
    rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
)
types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
msg_types = {topic: get_message(types[topic]) for topic in TOPICS if topic in types}

stats = {topic: {"count": 0, "minx": None, "maxx": None, "miny": None, "maxy": None, "prev": None, "maxjump": 0.0, "samples": []} for topic in TOPICS if topic != "/tf"}
frames = Counter()
tf_samples = {}

while reader.has_next():
    topic, data, stamp = reader.read_next()
    msg = deserialize_message(data, msg_types[topic])
    if topic == "/tf":
        for tf in msg.transforms:
            key = (tf.header.frame_id, tf.child_frame_id)
            frames[key] += 1
            if key not in tf_samples:
                tr = tf.transform.translation
                tf_samples[key] = (float(tr.x), float(tr.y), float(tr.z))
        continue

    x, y = xy_from_pose(msg)
    s = stats[topic]
    s["count"] += 1
    s["minx"] = x if s["minx"] is None else min(s["minx"], x)
    s["maxx"] = x if s["maxx"] is None else max(s["maxx"], x)
    s["miny"] = y if s["miny"] is None else min(s["miny"], y)
    s["maxy"] = y if s["maxy"] is None else max(s["maxy"], y)
    if len(s["samples"]) < 5:
        s["samples"].append((stamp * 1e-9, x, y))
    if s["prev"] is not None:
        s["maxjump"] = max(s["maxjump"], hypot(x - s["prev"][0], y - s["prev"][1]))
    s["prev"] = (x, y)

for topic, s in stats.items():
    print(topic)
    print(f"  count={s['count']} x=[{s['minx']:.3f}, {s['maxx']:.3f}] y=[{s['miny']:.3f}, {s['maxy']:.3f}] max_step={s['maxjump']:.3f}")
    print("  first samples=" + ", ".join(f"({x:.3f},{y:.3f})" for _, x, y in s["samples"]))

print("/tf frames")
for key, count in frames.most_common():
    sample = tf_samples[key]
    print(f"  {key[0]} -> {key[1]} count={count} first=({sample[0]:.3f},{sample[1]:.3f},{sample[2]:.3f})")
