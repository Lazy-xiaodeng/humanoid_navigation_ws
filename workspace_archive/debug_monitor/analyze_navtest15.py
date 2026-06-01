#!/usr/bin/env python3
"""
分析 nav_drift_test15 bag 和 debug_output.txt。

本脚本只读 bag / 日志，不修改 ROS 数据。输出重点：
1. 每个点位的开始、到达、完成时间。
2. prior_map_odom_bridge 的 ACCEPTED / REJECTED / PENDING / SPIN_GUARD 事件。
3. bag 中实际 map->odom TF 的变化，判断是否发生大修正或长时间冻结。
4. 冻结区间内 odom 是否仍在移动，用来判断是否是“冻结 map->odom，靠 odom 推进”。
"""

import argparse
import json
import math
import re
from bisect import bisect_left, bisect_right
from collections import defaultdict
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage


LOG_RE = re.compile(r"\[(?P<node>[^\]]+)\]\s+\[(?P<level>[A-Z]+)\]\s+\[(?P<time>\d+\.\d+)\]\s+\[[^\]]+\]:\s+(?P<msg>.*)")
START_RE = re.compile(r"开始导航到路点:\s*(点位\d+)")
DONE_RE = re.compile(r"Nav2确认到达路点:\s*(点位\d+)")
ACCEPT_RE = re.compile(r"ACCEPTED\s+(?P<reason>.*?)\s+map_odom_xy_norm=(?P<norm>[-0-9.]+)\s+yaw=(?P<yaw>[-0-9.]+)")
DELTA_RE = re.compile(r"(?:dx|spread_xy)=(?P<dx>[-0-9.]+).*?(?:yaw|spread_yaw)=(?P<yaw>[-0-9.]+)")


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def tf_to_xyyaw(t: TransformStamped):
    return (
        float(t.transform.translation.x),
        float(t.transform.translation.y),
        yaw_from_quat(t.transform.rotation),
    )


def odom_to_xyyaw(msg: Odometry):
    p = msg.pose.pose.position
    return (float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))


def read_log(path: Path):
    events = []
    current_point = None
    point_segments = []
    point_started = {}

    for line in path.read_text(errors="replace").splitlines():
        m = LOG_RE.search(line)
        if not m:
            continue
        t = float(m.group("time"))
        text = m.group("msg")

        start = START_RE.search(text)
        if start:
            current_point = start.group(1)
            point_started[current_point] = t
            events.append({"t": t, "kind": "nav_start", "point": current_point, "text": text})
            continue

        done = DONE_RE.search(text)
        if done:
            point = done.group(1)
            start_t = point_started.get(point)
            if start_t is not None:
                point_segments.append({"point": point, "start": start_t, "done": t})
            events.append({"t": t, "kind": "nav_done", "point": point, "text": text})
            current_point = None
            continue

        if "Reached the goal!" in text:
            events.append({"t": t, "kind": "controller_reached", "point": current_point, "text": text})
        elif "Goal failed" in text or "Nav2导航失败" in text or "Nav2 路径规划失败" in text:
            events.append({"t": t, "kind": "nav_failed", "point": current_point, "text": text})
            if current_point and "Goal failed" in text:
                start_t = point_started.get(current_point)
                if start_t is not None:
                    point_segments.append({"point": current_point, "start": start_t, "done": t, "failed": True})
                current_point = None
        elif "SpinToPose attempt" in text or "SpinToPose angle check" in text or "Nav2 正在执行主动转向" in text:
            events.append({"t": t, "kind": "spin", "point": current_point, "text": text})
        elif "检测到机器人停滞" in text:
            events.append({"t": t, "kind": "stuck", "point": current_point, "text": text})
        elif "机器人恢复运动" in text:
            events.append({"t": t, "kind": "resume_motion", "point": current_point, "text": text})

        if "prior_map_odom_bridge" in line:
            if "ACCEPTED" in text:
                ma = ACCEPT_RE.search(text)
                events.append({
                    "t": t,
                    "kind": "accepted",
                    "point": current_point,
                    "text": text,
                    "norm": float(ma.group("norm")) if ma else None,
                    "yaw": float(ma.group("yaw")) if ma else None,
                })
            elif "REJECTED" in text:
                md = DELTA_RE.search(text)
                events.append({
                    "t": t,
                    "kind": "rejected",
                    "point": current_point,
                    "text": text,
                    "dx": float(md.group("dx")) if md else None,
                    "yaw_delta": float(md.group("yaw")) if md else None,
                })
            elif "PENDING" in text or "SPIN_GUARD" in text:
                md = DELTA_RE.search(text)
                events.append({
                    "t": t,
                    "kind": "pending",
                    "point": current_point,
                    "text": text,
                    "dx": float(md.group("dx")) if md else None,
                    "yaw_delta": float(md.group("yaw")) if md else None,
                })

    # 只保留 point1..N 的第一次完成段；本日志里每次 APP 下发都是单点。
    point_segments.sort(key=lambda x: x["start"])
    return events, point_segments


def open_reader(uri: str):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(storage_options, converter_options)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, topic_types


def read_bag(uri: str):
    reader, topic_types = open_reader(uri)
    wanted = {
        "/tf",
        "/odom",
        "/robot_realpose",
        "/prior_localization/odom",
        "/prior_localization/confidence",
        "/localization/prior_map_odom_bridge_status",
        "/cmd_vel",
    }
    msg_types = {topic: get_message(type_name) for topic, type_name in topic_types.items() if topic in wanted}

    map_odom = []
    odom = []
    prior = []
    robot_realpose = []
    cmd = []
    confidence = []
    bridge_status = []

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_types:
            continue
        msg = deserialize_message(data, msg_types[topic])
        t = t_ns * 1e-9
        if topic == "/tf":
            for tr in msg.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    map_odom.append((t, *tf_to_xyyaw(tr)))
        elif topic == "/odom":
            odom.append((t, *odom_to_xyyaw(msg)))
        elif topic == "/prior_localization/odom":
            prior.append((t, *odom_to_xyyaw(msg)))
        elif topic == "/robot_realpose":
            p = msg.pose.pose.position
            robot_realpose.append((t, float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation)))
        elif topic == "/prior_localization/confidence":
            confidence.append((t, float(msg.data)))
        elif topic == "/localization/prior_map_odom_bridge_status":
            bridge_status.append((t, msg.data))
        elif topic == "/cmd_vel":
            cmd.append((t, float(msg.linear.x), float(msg.angular.z)))

    return {
        "map_odom": compress_tf(map_odom),
        "odom": odom,
        "robot_realpose": robot_realpose,
        "prior": prior,
        "confidence": confidence,
        "bridge_status": bridge_status,
        "cmd": cmd,
    }


def compress_tf(samples):
    """去掉同一 TF 重复发布的样本，只保留数值变化点。"""
    out = []
    last = None
    for s in samples:
        if last is None:
            out.append(s)
            last = s
            continue
        dx = math.hypot(s[1] - last[1], s[2] - last[2])
        dyaw = abs(angle_diff(s[3], last[3]))
        if dx > 1e-4 or dyaw > 1e-4:
            out.append(s)
            last = s
    return out


def slice_samples(samples, start, end):
    times = [s[0] for s in samples]
    return samples[bisect_left(times, start):bisect_right(times, end)]


def movement(samples):
    if len(samples) < 2:
        return 0.0, 0.0
    first = samples[0]
    last = samples[-1]
    xy = math.hypot(last[1] - first[1], last[2] - first[2])
    yaw = abs(angle_diff(last[3], first[3]))
    return xy, yaw


def tf_jumps(samples):
    jumps = []
    for a, b in zip(samples, samples[1:]):
        dx = math.hypot(b[1] - a[1], b[2] - a[2])
        dyaw = abs(angle_diff(b[3], a[3]))
        if dx >= 0.05 or dyaw >= 0.03:
            jumps.append({
                "t": b[0],
                "from": a,
                "to": b,
                "dx": dx,
                "dyaw": dyaw,
            })
    return jumps


def summarize(args):
    events, segments = read_log(Path(args.log))
    bag = read_bag(args.bag)

    all_tf_jumps = tf_jumps(bag["map_odom"])
    report = {
        "bag": args.bag,
        "log": args.log,
        "segments": [],
        "total_map_odom_changes": len(bag["map_odom"]),
        "total_map_odom_jumps_ge_5cm_or_003rad": len(all_tf_jumps),
    }

    for idx, seg in enumerate(segments, start=1):
        start = seg["start"]
        done = seg["done"]
        # 到点后 6s 也纳入，用于观察 spin settle 之后的接受/跳变。
        end = done + 6.0
        point_events = [e for e in events if start - 0.5 <= e["t"] <= end]
        accepts = [e for e in point_events if e["kind"] == "accepted"]
        rejects = [e for e in point_events if e["kind"] == "rejected"]
        pendings = [e for e in point_events if e["kind"] == "pending"]
        spin_rejects = [e for e in rejects if "spin_to_pose_freeze_tf" in e["text"]]

        tf_samples = slice_samples(bag["map_odom"], start - 0.5, end)
        jumps = tf_jumps(tf_samples)
        odom_xy, odom_yaw = movement(slice_samples(bag["odom"], start, done))
        prior_xy, prior_yaw = movement(slice_samples(bag["prior"], start, done))
        robot_xy, robot_yaw = movement(slice_samples(bag["robot_realpose"], start, done))
        bridge_status = slice_samples(bag["bridge_status"], start - 0.5, end)
        bag_accepts = [
            {"t": t, "text": text}
            for t, text in bridge_status
            if text.startswith("ACCEPTED")
        ]
        bag_rejects = [
            {"t": t, "text": text}
            for t, text in bridge_status
            if text.startswith("REJECTED")
        ]
        bag_pendings = [
            {"t": t, "text": text}
            for t, text in bridge_status
            if text.startswith("PENDING") or text.startswith("SPIN_GUARD") or text.startswith("WAITING")
        ]

        # “冻结靠 odom 推进”的近似判定：
        # 点位段内有 spin guard 拒绝或无 TF 更新，同时 odom 有明显运动。
        freeze_like = False
        if odom_xy > 0.20:
            if spin_rejects:
                freeze_like = True
            elif len(tf_samples) <= 1:
                freeze_like = True

        report["segments"].append({
            "idx": idx,
            "point": seg["point"],
            "start": start,
            "done": done,
            "failed": bool(seg.get("failed", False)),
            "duration": done - start,
            "accepted": accepts,
            "rejected": rejects,
            "pending": pendings,
            "spin_reject_count": len(spin_rejects),
            "tf_change_count": max(0, len(tf_samples) - 1),
            "tf_jumps": jumps,
            "odom_motion_xy": odom_xy,
            "odom_motion_yaw": odom_yaw,
            "prior_motion_xy": prior_xy,
            "prior_motion_yaw": prior_yaw,
            "robot_realpose_motion_xy": robot_xy,
            "robot_realpose_motion_yaw": robot_yaw,
            "bag_bridge_accepted": bag_accepts,
            "bag_bridge_rejected": bag_rejects,
            "bag_bridge_pending": bag_pendings,
            "freeze_like": freeze_like,
        })

    if args.out:
        out_dir = Path(args.out)
        out_dir.mkdir(parents=True, exist_ok=True)
        json_path = out_dir / "analysis.json"
        md_path = out_dir / "report.md"
        json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        md_path.write_text(render_markdown(report), encoding="utf-8")
        print(f"wrote {json_path}")
        print(f"wrote {md_path}")
    else:
        print(json.dumps(report, ensure_ascii=False, indent=2))


def fmt_time(t):
    return f"{t:.3f}"


def render_event(e):
    text = e["text"]
    if len(text) > 140:
        text = text[:137] + "..."
    return f"- {fmt_time(e['t'])}: {text}"


def render_markdown(report):
    lines = []
    lines.append("# nav_drift_test15 新定位节点导航分析")
    lines.append("")
    lines.append(f"- bag: `{report['bag']}`")
    lines.append(f"- log: `{report['log']}`")
    lines.append(f"- map->odom 数值变化次数: {report['total_map_odom_changes']}")
    lines.append(f"- map->odom >=5cm 或 >=0.03rad 的变化次数: {report['total_map_odom_jumps_ge_5cm_or_003rad']}")
    lines.append("")
    lines.append("## 点位汇总")
    lines.append("")
    lines.append("| 点位 | 时间段 | 耗时s | accepted | rejected | spin冻结拒绝 | TF变化 | TF大变化 | odom位移m | 判断 |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---|")
    for seg in report["segments"]:
        accepted = len(seg.get("bag_bridge_accepted") or seg["accepted"])
        rejected = len(seg.get("bag_bridge_rejected") or seg["rejected"])
        jumps = len(seg["tf_jumps"])
        verdict = "失败" if seg.get("failed") else "正常完成"
        if seg["freeze_like"]:
            if seg["rejected"] and seg["spin_reject_count"] == len(seg["rejected"]):
                verdict = ("失败；" if seg.get("failed") else "") + "SpinToPose保护冻结，非定位失败"
            else:
                verdict = ("失败；" if seg.get("failed") else "") + "疑似定位失败冻结靠odom推进"
        if jumps:
            max_jump = max(j["dx"] for j in seg["tf_jumps"])
            if max_jump >= 0.5:
                verdict += "；存在较大TF修正"
        lines.append(
            f"| {seg['point']} | {fmt_time(seg['start'])}-{fmt_time(seg['done'])} | "
            f"{seg['duration']:.1f} | {accepted} | {rejected} | {seg['spin_reject_count']} | "
            f"{seg['tf_change_count']} | {jumps} | {seg['odom_motion_xy']:.2f} | {verdict} |"
        )

    lines.append("")
    lines.append("## 每个点位事件")
    for seg in report["segments"]:
        lines.append("")
        lines.append(f"### {seg['point']}")
        lines.append("")
        lines.append(
            f"- 导航: {fmt_time(seg['start'])} -> {fmt_time(seg['done'])}, "
            f"耗时 {seg['duration']:.1f}s, odom 位移 {seg['odom_motion_xy']:.2f}m, "
            f"odom yaw {seg['odom_motion_yaw']:.2f}rad"
        )
        lines.append(
            f"- bridge: ACCEPTED {len(seg.get('bag_bridge_accepted') or seg['accepted'])} 次, "
            f"REJECTED {len(seg.get('bag_bridge_rejected') or seg['rejected'])} 次, "
            f"PENDING/SPIN_GUARD {len(seg.get('bag_bridge_pending') or seg['pending'])} 次；"
            f"spin 冻结拒绝 {seg['spin_reject_count']} 次"
        )
        lines.append(
            f"- TF: map->odom 数值变化 {seg['tf_change_count']} 次，大变化 {len(seg['tf_jumps'])} 次"
        )
        accepted_events = seg.get("bag_bridge_accepted") or seg["accepted"]
        rejected_events = seg.get("bag_bridge_rejected") or seg["rejected"]
        pending_events = seg.get("bag_bridge_pending") or seg["pending"]
        if accepted_events:
            lines.append("- 接受的定位修正:")
            for e in accepted_events:
                lines.append(render_event(e))
        if rejected_events:
            non_spin = [e for e in rejected_events if "spin_to_pose_freeze_tf" not in e["text"]]
            if non_spin:
                lines.append("- 非旋转保护拒绝:")
                for e in non_spin:
                    lines.append(render_event(e))
            else:
                lines.append("- 拒绝全部来自 SpinToPose 冻结保护。")
        if pending_events:
            lines.append("- Pending / SpinGuard:")
            for e in pending_events:
                lines.append(render_event(e))
        if seg["tf_jumps"]:
            lines.append("- map->odom 大变化:")
            for j in seg["tf_jumps"]:
                before = j["from"]
                after = j["to"]
                lines.append(
                    f"- {fmt_time(j['t'])}: dx={j['dx']:.3f}m dyaw={j['dyaw']:.3f}rad, "
                    f"({before[1]:.3f},{before[2]:.3f},{before[3]:.3f}) -> "
                    f"({after[1]:.3f},{after[2]:.3f},{after[3]:.3f})"
                )
        else:
            lines.append("- 未检测到 >=5cm 或 >=0.03rad 的 map->odom 大变化。")

    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--log", required=True)
    parser.add_argument("--out")
    summarize(parser.parse_args())
