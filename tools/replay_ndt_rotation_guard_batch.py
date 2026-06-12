#!/usr/bin/env python3
"""Replay one bag against the current NDT rotation guard implementation."""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import Counter
from pathlib import Path

import rclpy
import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.msg import BehaviorTreeLog
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


ndt_proc = None
recorded = []


def cleanup():
    global ndt_proc
    if ndt_proc:
        ndt_proc.terminate()
        try:
            ndt_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            ndt_proc.kill()
        ndt_proc = None


signal.signal(signal.SIGINT, lambda _s, _f: (cleanup(), sys.exit(130)))
signal.signal(signal.SIGTERM, lambda _s, _f: (cleanup(), sys.exit(143)))


def open_reader(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def bag_topics(bag):
    reader = open_reader(bag)
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def bag_start_end(bag):
    reader = open_reader(bag)
    start = None
    end = None
    while reader.has_next():
        _topic, _data, ts = reader.read_next()
        start = ts if start is None else min(start, ts)
        end = ts if end is None else max(end, ts)
    if start is None:
        raise RuntimeError(f"empty bag: {bag}")
    return start / 1e9, end / 1e9


def choose_replay_window(bag, requested_start, requested_duration):
    bag_start, bag_end = bag_start_end(bag)
    if requested_start is None:
        reader = open_reader(bag)
        reader.set_filter(rosbag2_py.StorageFilter(topics=["/pcl_pose"]))
        first_pcl = None
        while reader.has_next():
            _topic, _data, ts = reader.read_next()
            first_pcl = ts / 1e9
            break
        start = first_pcl if first_pcl is not None else bag_start
    elif requested_start < 1000000000.0:
        start = bag_start + requested_start
    else:
        start = requested_start
    start = max(bag_start, min(start, bag_end))
    end = bag_end if requested_duration is None else min(bag_end, start + requested_duration)
    return bag_start, bag_end, start, end


def load_initial_context(bag, start_sec):
    reader = open_reader(bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/pcl_pose", "/tf_static"]))
    cutoff_ns = int(start_sec * 1e9)
    initialpose = None
    tf_static_msg = TFMessage()
    first_pose = None

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic == "/tf_static":
            tf_static_msg.transforms.extend(deserialize_message(data, TFMessage).transforms)
        elif topic == "/pcl_pose":
            pose_msg = deserialize_message(data, PoseWithCovarianceStamped)
            candidate = PoseWithCovarianceStamped()
            candidate.header.frame_id = "map"
            candidate.header.stamp = pose_msg.header.stamp
            candidate.pose = pose_msg.pose
            if first_pose is None:
                first_pose = candidate
            if ts <= cutoff_ns:
                initialpose = candidate
            elif initialpose is not None:
                break

    existing_static = {(tr.header.frame_id, tr.child_frame_id) for tr in tf_static_msg.transforms}
    if ("odom", "camera_init") not in existing_static:
        tr = TransformStamped()
        tr.header.frame_id = "odom"
        tr.child_frame_id = "camera_init"
        tr.transform.rotation.x = -0.5
        tr.transform.rotation.y = -0.5
        tr.transform.rotation.z = 0.5
        tr.transform.rotation.w = 0.5
        tf_static_msg.transforms.append(tr)
    if ("body", "base_footprint") not in existing_static:
        tr = TransformStamped()
        tr.header.frame_id = "body"
        tr.child_frame_id = "base_footprint"
        tr.transform.rotation.x = 0.5
        tr.transform.rotation.y = 0.5
        tr.transform.rotation.z = -0.5
        tr.transform.rotation.w = 0.5
        tf_static_msg.transforms.append(tr)

    if initialpose is None:
        initialpose = first_pose
    if initialpose is None:
        raise RuntimeError("No /pcl_pose available for initialization")
    return initialpose, tf_static_msg


def original_ndt_stats(bag, start_sec, end_sec):
    reader = open_reader(bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/localization/ndt_status"]))
    rows = []
    start_ns = int(start_sec * 1e9)
    end_ns = int(end_sec * 1e9)
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if ts < start_ns:
            continue
        if ts > end_ns:
            break
        msg = deserialize_message(data, String)
        try:
            rows.append(json.loads(msg.data))
        except json.JSONDecodeError:
            pass
    return summarize(rows)


def call_service(node, srv_name, srv_type, request, timeout=30.0):
    client = node.create_client(srv_type, srv_name)
    if not client.wait_for_service(timeout_sec=5.0):
        node.destroy_client(client)
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    node.destroy_client(client)
    return future.result() if future.done() else None


def transition_to(node, target_state_id, timeout=60.0):
    state = call_service(node, "/lidar_localization/get_state", GetState, GetState.Request())
    if state is None:
        return False
    if state.current_state.id == target_state_id:
        return True

    req = ChangeState.Request()
    req.transition = Transition()
    if target_state_id == State.PRIMARY_STATE_INACTIVE:
        req.transition.id = Transition.TRANSITION_CONFIGURE
    elif target_state_id == State.PRIMARY_STATE_ACTIVE:
        req.transition.id = Transition.TRANSITION_ACTIVATE
    else:
        return False

    result = call_service(node, "/lidar_localization/change_state", ChangeState, req, timeout=timeout)
    if result is None or not result.success:
        return False

    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(1.0)
        state = call_service(node, "/lidar_localization/get_state", GetState, GetState.Request())
        if state is not None and state.current_state.id == target_state_id:
            return True
    return False


def start_ndt(
    label,
    *,
    reject_pose_jump=True,
    rotation_guard_enabled=True,
    multi_frame_enabled=True,
    multi_frame_only_when_rotating=True,
    multi_frame_window_sec=1.2,
    multi_frame_max_frames=12,
    multi_frame_voxel_leaf_size=0.20,
    multi_frame_max_points=20000,
    multi_frame_keyframe_max_interval_sec=0.25,
    ndt_resolution=None,
    ndt_step_size=None,
    ndt_max_iterations=None,
    voxel_leaf_size=None,
):
    log_path = f"/tmp/ndt_replay_{label}_launch.log"
    cmd = [
            "ros2", "run", "lidar_localization_ros2", "lidar_localization_node",
            "--ros-args",
            "--params-file", "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/lidar_localization/param/localization.yaml",
            "-p", "use_sim_time:=true",
            "-p", "set_initial_pose:=false",
            "-p", "initialpose_base_frame_id:=odom",
            "-p", "score_threshold:=0.3",
            "-p", f"reject_pose_jump:={'true' if reject_pose_jump else 'false'}",
            "-p", "max_pose_jump_translation:=0.50",
            "-p", "max_pose_jump_yaw:=0.40",
            "-p", "initialpose_relax_duration_sec:=4.0",
            "-p", "initialpose_max_pose_jump_translation:=2.00",
            "-p", "initialpose_max_pose_jump_yaw:=3.00",
            "-p", "pose_jump_reacquire_enabled:=true",
            "-p", "pose_jump_reacquire_max_translation:=1.50",
            "-p", "pose_jump_reacquire_max_yaw:=0.12",
            "-p", "pose_jump_reacquire_max_fitness:=0.03",
            "-p", "pose_jump_reacquire_required_frames:=4",
            "-p", "pose_jump_reacquire_xy_tolerance:=0.30",
            "-p", "pose_jump_reacquire_yaw_tolerance:=0.08",
            "-p", f"rotation_guard_enabled:={'true' if rotation_guard_enabled else 'false'}",
            "-p", "rotation_guard_use_cmd_vel_fallback:=false",
            "-p", "rotation_guard_navigation_status_topic:=/navigation/status",
            "-p", "rotation_guard_settle_sec:=1.2",
            "-p", "rotation_guard_max_duration_sec:=8.0",
            "-p", f"multi_frame_matching_enabled:={'true' if multi_frame_enabled else 'false'}",
            "-p", f"multi_frame_use_only_when_rotating:={'true' if multi_frame_only_when_rotating else 'false'}",
            "-p", f"multi_frame_window_sec:={multi_frame_window_sec}",
            "-p", f"multi_frame_max_frames:={multi_frame_max_frames}",
            "-p", f"multi_frame_voxel_leaf_size:={multi_frame_voxel_leaf_size}",
            "-p", f"multi_frame_max_points:={multi_frame_max_points}",
            "-p", "multi_frame_keyframe_filter_enabled:=true",
            "-p", "multi_frame_keyframe_translation_threshold:=0.10",
            "-p", "multi_frame_keyframe_yaw_threshold:=0.052",
            "-p", f"multi_frame_keyframe_max_interval_sec:={multi_frame_keyframe_max_interval_sec}",
            "-p", "republish_last_good_tf_on_failure:=true",
            "-p", "max_last_good_tf_age_sec:=5.0",
            "-p", "use_fastlio_delta_guess:=true",
            "-p", "fastlio_delta_guess_mode:=map_body_to_map_odom",
            "-p", "fastlio_camera_frame:=camera_init",
            "-p", "fastlio_body_frame:=body",
            "-p", "tf_max_stamp_mismatch_sec:=0.2",
            "-p", "fastlio_max_delta_translation:=0.20",
            "-p", "fastlio_max_delta_yaw:=0.25",
            "-p", "fastlio_max_delta_dt:=0.50",
            "-p", "fastlio_max_dead_reckon_sec:=2.0",
            "-p", "ndt_outlier_ratio:=0.30",
            "-p", "ndt_max_corr_dist:=2.0",
            "-p", "ndt_rotation_prior_enabled:=true",
            "-p", "ndt_rotation_prior_weight:=10.0",
            "-p", "ndt_rotation_prior_roll_pitch_only:=true",
            "-p", "use_odom:=false",
            "-p", "enable_debug:=false",
            "--log-level", "warn",
    ]
    for name, value in (
        ("ndt_resolution", ndt_resolution),
        ("ndt_step_size", ndt_step_size),
        ("ndt_max_iterations", ndt_max_iterations),
        ("voxel_leaf_size", voxel_leaf_size),
    ):
        if value is not None:
            cmd.extend(["-p", f"{name}:={value}"])
    return subprocess.Popen(
        cmd,
        stdout=open(log_path, "w"),
        stderr=subprocess.STDOUT,
    )


class ReplayMonitor(Node):
    def __init__(self, initialpose, tf_static_msg, status_source):
        super().__init__("ndt_replay_rotation_guard_monitor")
        self.active_bt_nodes = set()
        self.cmd_turning = False
        self.initialpose = initialpose
        self.tf_static_msg = tf_static_msg
        self.initialpose_publish_count = 0
        self.status_source = status_source
        self.status_pub = self.create_publisher(String, "/navigation/status", 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        static_qos = QoSProfile(depth=1)
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self.create_subscription(BehaviorTreeLog, "/behavior_tree_log", self.on_bt, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(String, "/localization/ndt_status", self.on_ndt, 1000)
        self.create_timer(0.1, self.publish_status)
        self.create_timer(0.2, self.publish_initial_context)

    def on_bt(self, msg):
        for event in msg.event_log:
            if event.node_name in {"SpinToPose", "Spin", "BackUp"}:
                if event.current_status == "RUNNING":
                    self.active_bt_nodes.add(event.node_name)
                else:
                    self.active_bt_nodes.discard(event.node_name)

    def on_cmd_vel(self, msg):
        # Offline-only fallback for bags without navigation state logs.
        self.cmd_turning = abs(msg.angular.z) > 0.18 and abs(msg.linear.x) < 0.12

    def publish_status(self):
        if self.status_source == "bt":
            turning = "SpinToPose" in self.active_bt_nodes
            source = "behavior_tree_log"
        elif self.status_source == "cmd_vel":
            turning = self.cmd_turning
            source = "cmd_vel_offline"
        else:
            turning = False
            source = "none"
        payload = {
            "timestamp": time.time(),
            "current_state": "executing",
            "detailed_state": "TURNING" if turning else "EXECUTING",
            "active_bt_nodes": sorted(self.active_bt_nodes),
            "offline_status_source": source,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def on_ndt(self, msg):
        recorded.append(msg.data)

    def publish_initial_context(self):
        if self.tf_static_msg is not None and self.initialpose_publish_count < 20:
            self.tf_static_pub.publish(self.tf_static_msg)
        if self.initialpose is not None and self.initialpose_publish_count < 10:
            self.initialpose_pub.publish(self.initialpose)
        self.initialpose_publish_count += 1


def summarize(rows):
    total = len(rows)
    reasons = Counter(row.get("reason", "") for row in rows)
    states = Counter(row.get("state", "") for row in rows)
    guard = sum(1 for row in rows if row.get("rotation_guard_active"))
    multiframe = sum(1 for row in rows if int(row.get("multi_frame_source_frames", 1)) > 1)
    ordinary_jump = sum(1 for row in rows if row.get("reason") == "pose_jump")
    correction = [float(row.get("correction_translation", 0.0) or 0.0) for row in rows]
    yaw = [abs(float(row.get("correction_yaw", 0.0) or 0.0)) for row in rows]
    fitness = [float(row.get("fitness_score", 0.0) or 0.0) for row in rows]
    return {
        "total": total,
        "states": dict(states),
        "reasons": dict(reasons),
        "guard": guard,
        "multiframe": multiframe,
        "ordinary_jump": ordinary_jump,
        "corr_max": max(correction) if correction else 0.0,
        "yaw_max": max(yaw) if yaw else 0.0,
        "fitness_max": max(fitness) if fitness else 0.0,
    }


def print_summary(name, summary):
    print(f"  {name}: total={summary['total']} states={summary['states']}")
    print(f"    reasons={summary['reasons']}")
    print(
        "    "
        f"guard={summary['guard']} multiframe={summary['multiframe']} "
        f"ordinary_pose_jump={summary['ordinary_jump']} "
        f"corr_max={summary['corr_max']:.3f} yaw_max={summary['yaw_max']:.3f} "
        f"fitness_max={summary['fitness_max']:.3f}"
    )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--label")
    parser.add_argument("--start", type=float)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--status-source", choices=["auto", "bt", "cmd_vel", "none"], default="auto")
    parser.add_argument("--disable-pose-jump-reject", action="store_true")
    parser.add_argument("--disable-rotation-guard", action="store_true")
    parser.add_argument("--disable-multi-frame", action="store_true")
    parser.add_argument("--multi-frame-always", action="store_true")
    parser.add_argument("--multi-frame-window-sec", type=float, default=1.2)
    parser.add_argument("--multi-frame-max-frames", type=int, default=12)
    parser.add_argument("--multi-frame-voxel-leaf-size", type=float, default=0.20)
    parser.add_argument("--multi-frame-max-points", type=int, default=20000)
    parser.add_argument("--multi-frame-keyframe-max-interval-sec", type=float, default=0.25)
    parser.add_argument("--ndt-resolution", type=float)
    parser.add_argument("--ndt-step-size", type=float)
    parser.add_argument("--ndt-max-iterations", type=int)
    parser.add_argument("--voxel-leaf-size", type=float)
    return parser.parse_args()


def main():
    global ndt_proc, recorded
    args = parse_args()
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")
    bag = Path(args.bag)
    label = args.label or bag.parent.name or bag.stem
    topics = bag_topics(bag)
    status_source = args.status_source
    if status_source == "auto":
        status_source = "bt" if "/behavior_tree_log" in topics else ("cmd_vel" if "/cmd_vel" in topics else "none")

    bag_start, bag_end, replay_start, replay_end = choose_replay_window(bag, args.start, args.duration)
    initialpose, tf_static_msg = load_initial_context(bag, replay_start)
    output = f"/tmp/ndt_replay_{label}_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"

    print(f"[0/5] {label}")
    print(f"  bag={bag}")
    print(f"  bag_span={bag_start:.3f}..{bag_end:.3f}")
    print(f"  replay={replay_start:.3f}..{replay_end:.3f} duration={replay_end - replay_start:.1f}s")
    print(f"  status_source={status_source}")
    print(
        "  "
        f"reject_pose_jump={not args.disable_pose_jump_reject} "
        f"rotation_guard={not args.disable_rotation_guard} "
        f"multi_frame={not args.disable_multi_frame} "
        f"multi_frame_only_when_rotating={not args.multi_frame_always}"
    )
    print(
        "  "
        f"multi_frame_window_sec={args.multi_frame_window_sec} "
        f"multi_frame_max_frames={args.multi_frame_max_frames} "
        f"multi_frame_voxel_leaf_size={args.multi_frame_voxel_leaf_size} "
        f"multi_frame_max_points={args.multi_frame_max_points} "
        f"multi_frame_keyframe_max_interval_sec={args.multi_frame_keyframe_max_interval_sec}"
    )
    print(f"  initial x={initialpose.pose.pose.position.x:.3f} y={initialpose.pose.pose.position.y:.3f}")

    rclpy.init()
    controller = Node(f"{label}_rotation_guard_controller")

    print("[1/5] starting NDT")
    ndt_proc = start_ndt(
        label,
        reject_pose_jump=not args.disable_pose_jump_reject,
        rotation_guard_enabled=not args.disable_rotation_guard,
        multi_frame_enabled=not args.disable_multi_frame,
        multi_frame_only_when_rotating=not args.multi_frame_always,
        multi_frame_window_sec=args.multi_frame_window_sec,
        multi_frame_max_frames=args.multi_frame_max_frames,
        multi_frame_voxel_leaf_size=args.multi_frame_voxel_leaf_size,
        multi_frame_max_points=args.multi_frame_max_points,
        multi_frame_keyframe_max_interval_sec=args.multi_frame_keyframe_max_interval_sec,
        ndt_resolution=args.ndt_resolution,
        ndt_step_size=args.ndt_step_size,
        ndt_max_iterations=args.ndt_max_iterations,
        voxel_leaf_size=args.voxel_leaf_size,
    )
    time.sleep(4.0)

    print("[2/5] activating lifecycle")
    if not transition_to(controller, State.PRIMARY_STATE_INACTIVE, timeout=30):
        print("ERROR: configure failed")
        cleanup()
        return 1
    if not transition_to(controller, State.PRIMARY_STATE_ACTIVE, timeout=90):
        print("ERROR: activate failed")
        cleanup()
        return 1

    print("[3/5] starting monitor")
    monitor = ReplayMonitor(initialpose, tf_static_msg, status_source)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    print("[4/5] playing bag")
    topics_to_play = ["/fast_lio/cloud_registered", "/tf"]
    if "/behavior_tree_log" in topics:
        topics_to_play.append("/behavior_tree_log")
    if status_source == "cmd_vel":
        topics_to_play.append("/cmd_vel")
    play = subprocess.run(
        [
            "ros2", "bag", "play", str(bag), "--clock", "100",
            "--rate", f"{args.rate:.3f}",
            "--start-offset", f"{replay_start - bag_start:.3f}",
            "--playback-duration", f"{replay_end - replay_start:.3f}",
            "--topics",
            *topics_to_play,
        ],
        text=True,
        capture_output=True,
        timeout=max(180, int((replay_end - replay_start) * 5 + 120)),
    )
    if play.returncode != 0:
        print(play.stdout[-1000:])
        print(play.stderr[-1000:])
        print(f"ERROR: bag play failed with {play.returncode}")

    print("[5/5] analyzing")
    time.sleep(5.0)
    executor.shutdown()
    spin_thread.join(timeout=5.0)
    cleanup()

    with open(output, "w") as f:
        for raw in recorded:
            f.write(raw + "\n")

    new_rows = []
    for raw in recorded:
        try:
            new_rows.append(json.loads(raw))
        except json.JSONDecodeError:
            pass
    print(f"  output={output}")
    print_summary("new", summarize(new_rows))
    print_summary("original", original_ndt_stats(bag, replay_start, replay_end))

    controller.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
