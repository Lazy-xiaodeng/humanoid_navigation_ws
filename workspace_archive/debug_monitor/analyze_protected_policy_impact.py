#!/usr/bin/env python3
"""
分析 shadow/experimental bridge 策略对导航的影响。

它不做 Nav2 闭环重跑，只回答一个更具体的问题：
  如果第一版保护策略生效，哪些时间段会冻结 map->odom 更新？
  冻结期间原 bag 中机器人是否仍在发 /cmd_vel、/odom 是否还在动？

这可以估算“会靠 odom 连续性推进多久”，但不能替代真正闭环 replay。
"""

import argparse
import json
import math
from bisect import bisect_left, bisect_right
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from analyze_navtest15 import angle_diff, odom_to_xyyaw


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


def read_motion_topics(uri: str):
    wanted = {"/cmd_vel", "/odom"}
    reader, topic_types = open_reader(uri)
    msg_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in wanted
    }
    cmd = []
    odom = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        t = t_ns * 1e-9
        if topic == "/cmd_vel":
            cmd.append((t, float(msg.linear.x), float(msg.angular.z)))
        elif topic == "/odom":
            odom.append((t, *odom_to_xyyaw(msg)))
    return cmd, odom


def segment_for_time(t, segments):
    previous = None
    next_segment = None
    for seg in segments:
        if seg["start"] <= t <= seg["done"]:
            return seg["point"], "nav"
        if seg["done"] < t:
            previous = seg
        elif seg["start"] > t and next_segment is None:
            next_segment = seg
    if previous or next_segment:
        return f"idle prev={previous and previous['point']} next={next_segment and next_segment['point']}", "idle"
    return "unknown", "unknown"


def protected_hold_clusters(events, min_dx):
    clusters = []
    current = []
    for event in events:
        is_hold = (
            event["action"] in ("hold", "hold_degraded")
            and event.get("dx", 0.0) >= min_dx
            and event["reason"] in ("nav_large_blocked", "idle_large_over_auto_accept_limit")
        )
        if is_hold:
            if current and event["t"] - current[-1]["t"] > 1.0:
                clusters.append(current)
                current = []
            current.append(event)
        elif current and event["t"] - current[-1]["t"] > 1.0:
            clusters.append(current)
            current = []
    if current:
        clusters.append(current)
    return clusters


def window(samples, start, end):
    times = [s[0] for s in samples]
    left = bisect_left(times, start)
    right = bisect_right(times, end)
    return samples[left:right]


def odom_motion(samples):
    if len(samples) < 2:
        return 0.0, 0.0
    total_xy = 0.0
    total_yaw = 0.0
    last = samples[0]
    for sample in samples[1:]:
        total_xy += math.hypot(sample[1] - last[1], sample[2] - last[2])
        total_yaw += abs(angle_diff(sample[3], last[3]))
        last = sample
    return total_xy, total_yaw


def cmd_stats(samples):
    if not samples:
        return {
            "cmd_count": 0,
            "moving_cmd_count": 0,
            "moving_ratio": 0.0,
            "max_linear": 0.0,
            "max_angular": 0.0,
        }
    moving = [
        s for s in samples
        if abs(s[1]) > 0.02 or abs(s[2]) > 0.05
    ]
    return {
        "cmd_count": len(samples),
        "moving_cmd_count": len(moving),
        "moving_ratio": len(moving) / len(samples),
        "max_linear": max(abs(s[1]) for s in samples),
        "max_angular": max(abs(s[2]) for s in samples),
    }


def render(report):
    lines = [
        "# Protected Policy Impact Estimate",
        "",
        "这不是 Nav2 闭环重跑；它只估算第一版保护策略下，哪些时段会冻结 map->odom 更新，以及原 bag 中这段时间机器人是否在动。",
        "",
        "## Summary",
        "",
        f"- hold clusters: {len(report['clusters'])}",
        f"- total hold duration: {report['total_hold_duration']:.3f}s",
        f"- total odom motion during holds: {report['total_odom_xy']:.3f}m",
        f"- total moving-cmd hold duration: {report['total_moving_hold_duration']:.3f}s",
        "",
        "## Clusters",
        "",
        "| label | phase | start | end | dur | max dx | degraded | odom xy | cmd moving ratio | max cmd v | max cmd w |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in report["clusters"]:
        lines.append(
            f"| {row['label']} | {row['phase']} | {row['start']:.3f} | {row['end']:.3f} | "
            f"{row['duration']:.3f} | {row['max_dx']:.3f} | {row['degraded_count']} | "
            f"{row['odom_xy']:.3f} | {row['moving_ratio']:.2f} | "
            f"{row['max_linear']:.3f} | {row['max_angular']:.3f} |"
        )
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--experimental-json", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--min-dx", type=float, default=0.5)
    args = parser.parse_args()

    exp = json.loads(Path(args.experimental_json).read_text(encoding="utf-8"))
    cmd, odom = read_motion_topics(args.bag)
    clusters = []
    for cluster in protected_hold_clusters(exp["protected"]["events"], args.min_dx):
        start = cluster[0]["t"]
        end = cluster[-1]["t"]
        label, phase = segment_for_time(start, exp["segments"])
        cmd_window = window(cmd, start, end)
        odom_window = window(odom, start, end)
        odom_xy, odom_yaw = odom_motion(odom_window)
        cs = cmd_stats(cmd_window)
        row = {
            "label": label,
            "phase": phase,
            "start": start,
            "end": end,
            "duration": end - start,
            "max_dx": max(e.get("dx", 0.0) for e in cluster),
            "degraded_count": len([e for e in cluster if e["action"] == "hold_degraded"]),
            "odom_xy": odom_xy,
            "odom_yaw": odom_yaw,
            **cs,
        }
        clusters.append(row)

    report = {
        "bag": args.bag,
        "experimental_json": args.experimental_json,
        "clusters": clusters,
        "total_hold_duration": sum(row["duration"] for row in clusters),
        "total_odom_xy": sum(row["odom_xy"] for row in clusters),
        "total_moving_hold_duration": sum(
            row["duration"] for row in clusters if row["moving_ratio"] > 0.2
        ),
    }
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "protected_policy_impact.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (out / "protected_policy_impact.md").write_text(render(report), encoding="utf-8")
    print(f"wrote {out / 'protected_policy_impact.json'}")
    print(f"wrote {out / 'protected_policy_impact.md'}")


if __name__ == "__main__":
    main()
