#!/usr/bin/env python3
"""Replay bags through the multi-frame NDT node and build a Markdown report."""

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import threading
import time
from collections import Counter, defaultdict
from pathlib import Path

import rclpy
import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


BAGS = [
    ("navtest0", "/home/ubuntu/nav_drift_test/navtest0"),
    ("nav_drift_test17", "/home/ubuntu/nav_drift_test/nav_drift_test17"),
    ("nav_drift_test19", "/home/ubuntu/nav_drift_test/nav_drift_test19"),
]

TOPICS = [
    "/fast_lio/cloud_registered",
    "/tf",
    "/tf_static",
    "/navigation/status",
    "/cmd_vel",
    "/odom",
]


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def percentile(values, p):
    if not values:
        return None
    vals = sorted(values)
    idx = min(len(vals) - 1, max(0, int(round((len(vals) - 1) * p))))
    return vals[idx]


def storage_id_for_bag(path):
    if list(Path(path).glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def open_reader(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id=storage_id_for_bag(uri)),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, topic_types


def first_recorded_map_odom(uri):
    reader, topic_types = open_reader(uri)
    msg_type = get_message(topic_types["/tf"])
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != "/tf":
            continue
        msg = deserialize_message(data, msg_type)
        for tr in msg.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                yaw = yaw_from_quat(tr.transform.rotation)
                return {
                    "t": t_ns * 1e-9,
                    "x": float(tr.transform.translation.x),
                    "y": float(tr.transform.translation.y),
                    "z": float(tr.transform.translation.z),
                    "yaw": yaw,
                }
    return {"t": 0.0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}


class ReplayRecorder(Node):
    def __init__(self, initial_pose, output_jsonl):
        super().__init__("multiframe_ndt_replay_recorder")
        self.output_jsonl = Path(output_jsonl)
        self.output_jsonl.parent.mkdir(parents=True, exist_ok=True)
        self.events = []
        self.tf_samples = []
        self.nav_status = []
        self.recovery_events = []
        self.initial_pose = initial_pose
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )
        self.create_subscription(String, "/localization/ndt_status", self.ndt_cb, 1000)
        self.create_subscription(String, "/navigation/status", self.nav_cb, 1000)
        self.create_subscription(String, "/localization/recovery_status", self.recovery_cb, 100)
        self.create_subscription(TFMessage, "/tf", self.tf_cb, 1000)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 1000)
        self.create_subscription(TFMessage, "/tf_bag_raw", self.tf_bag_raw_cb, 1000)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.initial_pose["x"]
        msg.pose.pose.position.y = self.initial_pose["y"]
        msg.pose.pose.position.z = self.initial_pose["z"]
        qx, qy, qz, qw = quat_from_yaw(self.initial_pose["yaw"])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self.initial_pub.publish(msg)

    def ndt_cb(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            event = {"raw": msg.data}
        event["_wall_time"] = time.time()
        self.events.append(event)

    def nav_cb(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            status = {"raw": msg.data}
        status["_wall_time"] = time.time()
        self.nav_status.append(status)

    def recovery_cb(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            status = {"raw": msg.data}
        status["_wall_time"] = time.time()
        self.recovery_events.append(status)

    def tf_cb(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                self.tf_samples.append(
                    (
                        stamp,
                        float(tr.transform.translation.x),
                        float(tr.transform.translation.y),
                        yaw_from_quat(tr.transform.rotation),
                    )
                )

    def tf_bag_raw_cb(self, msg):
        filtered = TFMessage()
        filtered.transforms = [
            tr for tr in msg.transforms
            if not (tr.header.frame_id == "map" and tr.child_frame_id == "odom")
        ]
        if filtered.transforms:
            self.tf_pub.publish(filtered)

    def flush(self):
        with self.output_jsonl.open("w") as f:
            for event in self.events:
                f.write(json.dumps(event, ensure_ascii=False, sort_keys=True) + "\n")


def call_service(node, srv_name, srv_type, request, timeout=30.0):
    client = node.create_client(srv_type, srv_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if future.done():
            return future.result()
        return None
    finally:
        node.destroy_client(client)


def change_lifecycle_state(node, transition_id, timeout=60.0):
    req = ChangeState.Request()
    req.transition.id = transition_id
    result = call_service(node, "/lidar_localization/change_state", ChangeState, req, timeout)
    return result is not None and result.success


def get_lifecycle_state(node, timeout=10.0):
    result = call_service(node, "/lidar_localization/get_state", GetState, GetState.Request(), timeout)
    if result is None:
        return None
    return result.current_state.id


def activate_lifecycle(node):
    state = get_lifecycle_state(node)
    if state == State.PRIMARY_STATE_UNCONFIGURED:
        if not change_lifecycle_state(node, Transition.TRANSITION_CONFIGURE, timeout=90.0):
            return False
        state = get_lifecycle_state(node)
    if state == State.PRIMARY_STATE_INACTIVE:
        if not change_lifecycle_state(node, Transition.TRANSITION_ACTIVATE, timeout=60.0):
            return False
    return get_lifecycle_state(node) == State.PRIMARY_STATE_ACTIVE


def terminate_process(proc, name):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def run_one_bag(label, bag, output_dir, rate, playback_duration):
    bag = Path(bag)
    initial = first_recorded_map_odom(bag)
    jsonl_path = Path(output_dir) / label / "ndt_status.jsonl"
    log_path = Path(output_dir) / label / "process.log"
    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    ros_log_dir = Path(output_dir) / label / "ros_logs"
    ros_log_dir.mkdir(parents=True, exist_ok=True)

    ndt_cmd = [
        "ros2",
        "run",
        "lidar_localization_ros2",
        "lidar_localization_node",
        "--ros-args",
        "--params-file",
        "/home/ubuntu/humanoid_ws/src/lidar_localization/param/localization.yaml",
        "-p",
        "use_sim_time:=true",
        "-p",
        "set_initial_pose:=true",
        "-p",
        f"initial_pose_x:={initial['x']}",
        "-p",
        f"initial_pose_y:={initial['y']}",
        "-p",
        f"initial_pose_z:={initial['z']}",
        "-p",
        f"initial_pose_qz:={math.sin(initial['yaw'] * 0.5)}",
        "-p",
        f"initial_pose_qw:={math.cos(initial['yaw'] * 0.5)}",
        "-p",
        "score_threshold:=0.3",
        "-p",
        "reject_pose_jump:=true",
        "-p",
        "max_pose_jump_translation:=0.50",
        "-p",
        "max_pose_jump_yaw:=0.40",
        "-p",
        "initialpose_relax_duration_sec:=4.0",
        "-p",
        "initialpose_max_pose_jump_translation:=2.00",
        "-p",
        "initialpose_max_pose_jump_yaw:=3.00",
        "-p",
        "pose_jump_reacquire_enabled:=true",
        "-p",
        "pose_jump_reacquire_max_translation:=1.50",
        "-p",
        "pose_jump_reacquire_max_yaw:=0.12",
        "-p",
        "pose_jump_reacquire_max_fitness:=0.03",
        "-p",
        "pose_jump_reacquire_required_frames:=4",
        "-p",
        "rotation_guard_enabled:=true",
        "-p",
        "rotation_guard_use_cmd_vel_fallback:=true",
        "-p",
        "rotation_guard_navigation_status_topic:=/navigation/status",
        "-p",
        "rotation_guard_settle_sec:=1.2",
        "-p",
        "multi_frame_matching_enabled:=true",
        "-p",
        "multi_frame_use_only_when_rotating:=false",
        "-p",
        "multi_frame_window_sec:=1.2",
        "-p",
        "multi_frame_max_frames:=12",
        "-p",
        "multi_frame_voxel_leaf_size:=0.20",
        "-p",
        "multi_frame_max_points:=30000",
        "-p",
        "multi_frame_keyframe_filter_enabled:=true",
        "-p",
        "multi_frame_keyframe_translation_threshold:=0.10",
        "-p",
        "multi_frame_keyframe_yaw_threshold:=0.052",
        "-p",
        "multi_frame_keyframe_max_interval_sec:=0.15",
        "-p",
        "republish_last_good_tf_on_failure:=true",
        "-p",
        "max_last_good_tf_age_sec:=5.0",
        "-p",
        "use_fastlio_delta_guess:=true",
        "-p",
        "fastlio_delta_guess_mode:=map_body_to_map_odom",
        "-p",
        "fastlio_camera_frame:=camera_init",
        "-p",
        "fastlio_body_frame:=body",
        "-p",
        "fastlio_max_delta_translation:=0.20",
        "-p",
        "fastlio_max_delta_yaw:=0.25",
        "-p",
        "fastlio_max_delta_dt:=0.50",
        "-p",
        "fastlio_max_dead_reckon_sec:=2.0",
        "-p",
        "ndt_outlier_ratio:=0.30",
        "-p",
        "ndt_max_corr_dist:=2.0",
        "-p",
        "ndt_rotation_prior_enabled:=true",
        "-p",
        "ndt_rotation_prior_weight:=10.0",
        "-p",
        "ndt_rotation_prior_roll_pitch_only:=true",
        "--log-level",
        "warn",
    ]

    play_cmd = [
        "ros2",
        "bag",
        "play",
        str(bag),
        "--clock",
        "50",
        "--rate",
        str(rate),
        "--disable-keyboard-controls",
        "--remap",
        "/tf:=/tf_bag_raw",
        "--topics",
        *TOPICS,
    ]
    if playback_duration > 0:
        play_cmd.extend(["--playback-duration", str(playback_duration)])

    ndt_proc = None
    play_proc = None
    os.environ.setdefault("ROS_LOG_DIR", str(ros_log_dir))
    rclpy.init(args=None)
    control_node = Node(f"{label}_lifecycle_controller")
    recorder = ReplayRecorder(initial, jsonl_path)
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    with log_path.open("w") as log:
        try:
            log.write("NDT command: " + " ".join(ndt_cmd) + "\n")
            log.write("Play command: " + " ".join(play_cmd) + "\n")
            log.flush()
            ndt_proc = subprocess.Popen(ndt_cmd, stdout=log, stderr=log)
            time.sleep(4.0)
            if not activate_lifecycle(control_node):
                raise RuntimeError("failed to activate /lidar_localization lifecycle node")

            for _ in range(5):
                recorder.publish_initial_pose()
                time.sleep(0.2)

            play_proc = subprocess.Popen(play_cmd, stdout=log, stderr=log)
            ret = play_proc.wait()
            if ret != 0:
                raise RuntimeError(f"ros2 bag play exited with code {ret}")
            time.sleep(3.0)
        finally:
            recorder.flush()
            terminate_process(play_proc, "ros2 bag play")
            terminate_process(ndt_proc, "lidar_localization_node")
            executor.shutdown()
            spin_thread.join(timeout=5.0)
            control_node.destroy_node()
            recorder.destroy_node()
            rclpy.shutdown()

    return analyze_run(label, bag, initial, recorder.events, recorder.tf_samples, recorder.nav_status, jsonl_path)


def tf_jump_stats(samples):
    compressed = []
    last = None
    jumps = []
    for sample in samples:
        if last is None:
            compressed.append(sample)
            last = sample
            continue
        dx = math.hypot(sample[1] - last[1], sample[2] - last[2])
        dyaw = abs(angle_diff(sample[3], last[3]))
        if dx > 1e-4 or dyaw > 1e-4:
            compressed.append(sample)
            jumps.append((sample[0], dx, dyaw))
            last = sample
    return {
        "changes": len(compressed),
        "jumps": jumps,
        "big_jumps": [j for j in jumps if j[1] >= 0.05 or j[2] >= 0.03],
        "max_jump": max([j[1] for j in jumps], default=0.0),
        "max_yaw_jump": max([j[2] for j in jumps], default=0.0),
    }


def analyze_run(label, bag, initial, events, tf_samples, nav_status, jsonl_path):
    states = Counter(e.get("state", "?") for e in events)
    reasons = Counter(e.get("reason", "?") for e in events)
    accepted = [e for e in events if e.get("state") == "accepted"]
    rejected = [e for e in events if e.get("state") == "rejected"]
    confirming = [e for e in events if e.get("state") == "confirming"]
    fitness = [float(e["fitness_score"]) for e in events if e.get("fitness_score") is not None]
    accepted_fitness = [float(e["fitness_score"]) for e in accepted if e.get("fitness_score") is not None]
    corrections = [
        float(e["correction_translation"])
        for e in events
        if e.get("correction_translation") is not None
    ]
    accepted_corrections = [
        float(e["correction_translation"])
        for e in accepted
        if e.get("correction_translation") is not None
    ]
    source_frames = [
        int(e["multi_frame_source_frames"])
        for e in events
        if e.get("multi_frame_source_frames") is not None
    ]
    source_points = [
        int(e["multi_frame_source_points"])
        for e in events
        if e.get("multi_frame_source_points") is not None
    ]
    guard_events = [e for e in events if e.get("rotation_guard_active")]
    guard_holds = [e for e in events if str(e.get("reason", "")).startswith("rotation_guard")]
    pose_jump = [e for e in events if e.get("reason") == "pose_jump"]
    pose_jump_candidates = [e for e in events if e.get("reason") == "pose_jump_candidate"]
    confirmed_pose_jump = [e for e in events if e.get("reason") == "confirmed_pose_jump"]
    high_fitness = [e for e in events if "high_fitness" in str(e.get("reason", ""))]
    fastlio_applied = [e for e in events if e.get("fastlio_delta_applied")]
    tf_stats = tf_jump_stats(tf_samples)

    nav_states = Counter()
    nav_active_count = 0
    for status in nav_status:
        state = str(status.get("current_state", status.get("state", ""))).lower()
        detail = str(status.get("current_detailed_state", status.get("detailed_state", "")))
        if state:
            nav_states[state] += 1
        if state in ("executing", "planning", "running", "active"):
            nav_active_count += 1
        if detail:
            nav_states[f"detail:{detail}"] += 1

    total = len(events)
    duration = 0.0
    stamps = [float(e["stamp_sec"]) for e in events if e.get("stamp_sec") is not None]
    if stamps:
        duration = max(stamps) - min(stamps)

    return {
        "label": label,
        "bag": str(bag),
        "jsonl": str(jsonl_path),
        "initial": initial,
        "total": total,
        "duration": duration,
        "states": states,
        "reasons": reasons,
        "accepted": len(accepted),
        "rejected": len(rejected),
        "confirming": len(confirming),
        "accept_rate": (len(accepted) / total * 100.0) if total else 0.0,
        "reject_rate": (len(rejected) / total * 100.0) if total else 0.0,
        "fitness": fitness,
        "accepted_fitness": accepted_fitness,
        "corrections": corrections,
        "accepted_corrections": accepted_corrections,
        "source_frames": source_frames,
        "source_points": source_points,
        "guard_events": len(guard_events),
        "guard_holds": len(guard_holds),
        "pose_jump": len(pose_jump),
        "pose_jump_candidates": len(pose_jump_candidates),
        "confirmed_pose_jump": len(confirmed_pose_jump),
        "high_fitness": len(high_fitness),
        "fastlio_applied": len(fastlio_applied),
        "tf": tf_stats,
        "nav_states": nav_states,
        "nav_active_count": nav_active_count,
        "worst_events": sorted(
            events,
            key=lambda e: float(e.get("correction_translation") or 0.0),
            reverse=True,
        )[:12],
    }


def fmt_float(value, digits=3, suffix=""):
    if value is None:
        return "-"
    return f"{value:.{digits}f}{suffix}"


def stats_line(values, digits=3):
    if not values:
        return "n/a"
    return (
        f"p50 {fmt_float(percentile(values, 0.50), digits)}, "
        f"p90 {fmt_float(percentile(values, 0.90), digits)}, "
        f"p99 {fmt_float(percentile(values, 0.99), digits)}, "
        f"max {fmt_float(max(values), digits)}"
    )


def verdict(result):
    if result["total"] == 0:
        return "无 NDT 输出"
    if result["pose_jump"] > 0:
        return "触发跳变拒绝"
    if result["guard_holds"] > 0:
        return "旋转保护接管"
    if result["reject_rate"] > 20.0:
        return "拒绝偏多"
    if result["accept_rate"] >= 90.0 and result["tf"]["max_jump"] < 0.5:
        return "整体稳定"
    return "可用但需复核"


def write_report(results, output_dir, rate, playback_duration):
    output_dir = Path(output_dir)
    report = output_dir / "multiframe_ndt_replay_report.md"
    now = time.strftime("%Y-%m-%d %H:%M:%S")
    lines = []
    lines.append("# 多帧 NDT Bag 回放模拟报告")
    lines.append("")
    lines.append(f"- 生成时间: `{now}`")
    lines.append(f"- 回放倍率: `{rate}x`")
    if playback_duration > 0:
        lines.append(f"- 每包截断: `{playback_duration:.1f}s`")
    else:
        lines.append("- 每包截断: `未截断，完整回放`")
    lines.append("- 输入: `/fast_lio/cloud_registered` + `/tf` + `/tf_static` + `/navigation/status` + `/cmd_vel` + `/odom`")
    lines.append("- NDT 模式: `multi_frame_matching_enabled=true`, `window=1.2s`, `max_frames=12`, `rotation_guard_enabled=true`")
    lines.append("")
    lines.append("> 说明: 这是离线影子回放。它能验证同一批传感器数据下多帧 NDT 的匹配、拒绝、旋转保护和 TF 输出表现；不能完全替代实机闭环，因为 Nav2 控制不会根据新的 NDT 输出重新生成真实运动。")
    lines.append("")
    lines.append("## 总览")
    lines.append("")
    lines.append("| Bag | NDT帧 | 接受率 | 拒绝率 | 最大TF跳变 | pose_jump | rotation_guard | 多帧p50/p90 | 结论 |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---|")
    for r in results:
        lines.append(
            "| {label} | {total} | {accept:.1f}% | {reject:.1f}% | {jump:.3f}m | {pose_jump} | {guard} | {mf50}/{mf90} | {verdict} |".format(
                label=r["label"],
                total=r["total"],
                accept=r["accept_rate"],
                reject=r["reject_rate"],
                jump=r["tf"]["max_jump"],
                pose_jump=r["pose_jump"],
                guard=r["guard_holds"],
                mf50=fmt_float(percentile(r["source_frames"], 0.50), 0),
                mf90=fmt_float(percentile(r["source_frames"], 0.90), 0),
                verdict=verdict(r),
            )
        )
    lines.append("")
    lines.append("## 关键判断")
    lines.append("")
    for r in results:
        if r["total"] == 0:
            lines.append(f"- `{r['label']}` 没有收到 NDT 状态输出，不能评估。")
            continue
        lines.append(
            f"- `{r['label']}`: 接受 {r['accepted']}/{r['total']} 帧，"
            f"最大 map->odom TF 跳变 {r['tf']['max_jump']:.3f}m / {r['tf']['max_yaw_jump']:.3f}rad，"
            f"pose_jump={r['pose_jump']}，rotation_guard={r['guard_holds']}。"
        )
    lines.append("")
    lines.append("## 分包详情")
    lines.append("")
    for r in results:
        lines.append(f"### {r['label']}")
        lines.append("")
        lines.append(f"- Bag: `{r['bag']}`")
        lines.append(f"- 原始 NDT JSONL: `{r['jsonl']}`")
        lines.append(
            f"- 初始 map->odom: x={r['initial']['x']:.3f}, y={r['initial']['y']:.3f}, "
            f"z={r['initial']['z']:.3f}, yaw={r['initial']['yaw']:.3f}rad"
        )
        lines.append(f"- NDT 时间跨度: `{r['duration']:.1f}s`")
        lines.append("")
        lines.append("| 指标 | 数值 |")
        lines.append("|---|---:|")
        lines.append(f"| NDT状态帧 | {r['total']} |")
        lines.append(f"| accepted | {r['accepted']} ({r['accept_rate']:.1f}%) |")
        lines.append(f"| rejected | {r['rejected']} ({r['reject_rate']:.1f}%) |")
        lines.append(f"| confirming | {r['confirming']} |")
        lines.append(f"| high_fitness拒绝 | {r['high_fitness']} |")
        lines.append(f"| pose_jump拒绝 | {r['pose_jump']} |")
        lines.append(f"| pose_jump候选确认中 | {r['pose_jump_candidates']} |")
        lines.append(f"| confirmed_pose_jump放行 | {r['confirmed_pose_jump']} |")
        lines.append(f"| rotation_guard hold/settle | {r['guard_holds']} |")
        lines.append(f"| Fast-LIO delta guess应用 | {r['fastlio_applied']} |")
        lines.append(f"| map->odom变化次数 | {r['tf']['changes']} |")
        lines.append(f"| map->odom大变化(>=5cm或>=0.03rad) | {len(r['tf']['big_jumps'])} |")
        lines.append(f"| 最大map->odom平移跳变 | {r['tf']['max_jump']:.3f}m |")
        lines.append(f"| 最大map->odom yaw跳变 | {r['tf']['max_yaw_jump']:.3f}rad |")
        lines.append("")
        lines.append("**分布**")
        lines.append("")
        lines.append(f"- fitness: {stats_line(r['fitness'], 4)}")
        lines.append(f"- accepted fitness: {stats_line(r['accepted_fitness'], 4)}")
        lines.append(f"- correction translation: {stats_line(r['corrections'], 3)}m")
        lines.append(f"- accepted correction translation: {stats_line(r['accepted_corrections'], 3)}m")
        lines.append(f"- multi-frame source frames: {stats_line(r['source_frames'], 0)} 帧")
        lines.append(f"- multi-frame source points: {stats_line(r['source_points'], 0)} 点")
        lines.append("")
        lines.append("**拒绝/确认原因 Top 10**")
        lines.append("")
        lines.append("| reason | count |")
        lines.append("|---|---:|")
        for reason, count in r["reasons"].most_common(10):
            lines.append(f"| `{reason}` | {count} |")
        lines.append("")
        lines.append("**最大 correction 事件 Top 12**")
        lines.append("")
        lines.append("| t | state | reason | correction | yaw | fitness | source_frames | guard |")
        lines.append("|---:|---|---|---:|---:|---:|---:|---|")
        for e in r["worst_events"]:
            lines.append(
                "| {t:.3f} | `{state}` | `{reason}` | {corr:.3f} | {yaw:.3f} | {fit:.4f} | {frames} | {guard} |".format(
                    t=float(e.get("stamp_sec") or 0.0),
                    state=e.get("state", "?"),
                    reason=e.get("reason", "?"),
                    corr=float(e.get("correction_translation") or 0.0),
                    yaw=float(e.get("correction_yaw") or 0.0),
                    fit=float(e.get("fitness_score") or 0.0),
                    frames=e.get("multi_frame_source_frames", "-"),
                    guard="Y" if e.get("rotation_guard_active") else "",
                )
            )
        lines.append("")
    lines.append("## 结论边界")
    lines.append("")
    lines.append("- 本报告验证的是定位节点和保护策略在录制输入上的表现。")
    lines.append("- `navigation_auto_paused` 和真正重定位闭环需要同时启动导航状态管理器与重定位 bridge；本次 NDT 回放结果可作为是否会触发这些链路的依据。")
    lines.append("- 当前代码的多帧合并会累积最近帧并体素降采样，但没有对历史帧做运动补偿；机器人移动较快或转弯时，多帧可能增强几何约束，也可能引入轻微拖影。")
    lines.append("")
    report.write_text("\n".join(lines), encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--playback-duration", type=float, default=-1.0)
    parser.add_argument("--bags", nargs="*", default=[])
    args = parser.parse_args()

    started = time.strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or f"/home/ubuntu/humanoid_ws/debug_monitor/multiframe_ndt_replay_{started}"
    selected = BAGS
    if args.bags:
        wanted = set(args.bags)
        selected = [b for b in BAGS if b[0] in wanted or b[1] in wanted]
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    results = []
    for label, bag in selected:
        print(f"=== Replaying {label}: {bag} ===", flush=True)
        result = run_one_bag(label, bag, output_dir, args.rate, args.playback_duration)
        results.append(result)
        print(
            f"=== {label}: {result['accepted']}/{result['total']} accepted, "
            f"max jump {result['tf']['max_jump']:.3f}m ===",
            flush=True,
        )

    report = write_report(results, output_dir, args.rate, args.playback_duration)
    print(f"Report: {report}")


if __name__ == "__main__":
    main()
