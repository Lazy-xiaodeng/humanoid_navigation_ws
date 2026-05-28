#!/usr/bin/env python3
"""Replay nav_drift_test6 against the current NDT rotation guard implementation."""

import json
import signal
import subprocess
import sys
import threading
import time
from collections import Counter

import rclpy
import rosbag2_py
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import BehaviorTreeLog
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


BAG = "/home/ubuntu/nav_drift_test/nav_drift_test6/nav_drift_test6_0.mcap"
OUTPUT = f"/tmp/ndt_replay_bag6_rotation_guard_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
LAUNCH_LOG = "/tmp/ndt_replay_bag6_rotation_guard_launch.log"
BAG_START_SEC = 1779966622.992477706
POINT5_INIT_STAMP_SEC = 1779966918.0
POINT5_START_OFFSET_SEC = 292.0
POINT5_PLAY_DURATION_SEC = 55.0

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


signal.signal(signal.SIGINT, lambda _s, _f: (cleanup(), sys.exit(130)))
signal.signal(signal.SIGTERM, lambda _s, _f: (cleanup(), sys.exit(143)))


class ReplayMonitor(Node):
    def __init__(self, initialpose, tf_static_msg):
        super().__init__("bag6_rotation_guard_monitor")
        self.active_bt_nodes = set()
        self.initialpose = initialpose
        self.tf_static_msg = tf_static_msg
        self.initialpose_publish_count = 0
        self.status_pub = self.create_publisher(String, "/navigation/status", 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        static_qos = QoSProfile(depth=1)
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self.create_subscription(BehaviorTreeLog, "/behavior_tree_log", self.on_bt, 10)
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

    def publish_status(self):
        detailed_state = "TURNING" if "SpinToPose" in self.active_bt_nodes else "EXECUTING"
        payload = {
            "timestamp": time.time(),
            "current_state": "executing",
            "detailed_state": detailed_state,
            "active_bt_nodes": sorted(self.active_bt_nodes),
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


def load_point5_initial_context():
    storage_options = rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/pcl_pose", "/tf_static"]))

    initialpose = None
    tf_static_msg = TFMessage()
    cutoff_ns = int(POINT5_INIT_STAMP_SEC * 1e9)

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == "/tf_static":
            msg = deserialize_message(data, TFMessage)
            tf_static_msg.transforms.extend(msg.transforms)
        elif topic == "/pcl_pose" and timestamp <= cutoff_ns:
            pose_msg = deserialize_message(data, PoseWithCovarianceStamped)
            initialpose = PoseWithCovarianceStamped()
            initialpose.header.frame_id = "map"
            initialpose.header.stamp = pose_msg.header.stamp
            initialpose.pose = pose_msg.pose
        elif timestamp > cutoff_ns and initialpose is not None:
            break

    if initialpose is None:
        raise RuntimeError("No /pcl_pose found before point5 init stamp")
    if not tf_static_msg.transforms:
        tf_static_msg = None
    return initialpose, tf_static_msg


def call_service(node, srv_name, srv_type, request, timeout=30.0):
    client = node.create_client(srv_type, srv_name)
    if not client.wait_for_service(timeout_sec=5.0):
        print(f"  service unavailable: {srv_name}")
        node.destroy_client(client)
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    node.destroy_client(client)
    if future.done():
        return future.result()
    return None


def transition_to(node, target_state_id, timeout=60.0):
    get_req = GetState.Request()
    state = call_service(node, "/lidar_localization/get_state", GetState, get_req)
    if state is None:
        return False

    current_id = state.current_state.id
    if current_id == target_state_id:
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


def start_ndt():
    return subprocess.Popen(
        [
            "ros2", "run", "lidar_localization_ros2", "lidar_localization_node",
            "--ros-args",
            "--params-file", "/home/ubuntu/humanoid_ws/src/lidar_localization/param/localization.yaml",
            "-p", "use_sim_time:=true",
            "-p", "set_initial_pose:=false",
            "-p", "initialpose_base_frame_id:=odom",
            "-p", "score_threshold:=0.3",
            "-p", "reject_pose_jump:=true",
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
            "-p", "rotation_guard_enabled:=true",
            "-p", "rotation_guard_use_cmd_vel_fallback:=false",
            "-p", "rotation_guard_navigation_status_topic:=/navigation/status",
            "-p", "rotation_guard_settle_sec:=1.2",
            "-p", "rotation_guard_max_duration_sec:=8.0",
            "-p", "multi_frame_matching_enabled:=true",
            "-p", "multi_frame_use_only_when_rotating:=true",
            "-p", "multi_frame_window_sec:=0.6",
            "-p", "multi_frame_max_frames:=8",
            "-p", "multi_frame_voxel_leaf_size:=0.20",
            "-p", "multi_frame_max_points:=40000",
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
            "--log-level", "warn",
        ],
        stdout=open(LAUNCH_LOG, "w"),
        stderr=subprocess.STDOUT,
    )


def analyze(lines):
    total = len(lines)
    accepted = [x for x in lines if x.get("state") == "accepted"]
    rejected = [x for x in lines if x.get("state") == "rejected"]
    confirming = [x for x in lines if x.get("state") == "confirming"]
    reasons = Counter(x.get("reason", "") for x in lines)

    print("\nRESULTS")
    print(f"  output: {OUTPUT}")
    print(f"  total={total} accepted={len(accepted)} rejected={len(rejected)} confirming={len(confirming)}")
    if total:
        print(f"  accept_rate={len(accepted) / total * 100:.1f}%")
        print("  reasons:")
        for reason, count in reasons.most_common(12):
            print(f"    {reason or '<empty>'}: {count}")

    guarded = [x for x in lines if x.get("rotation_guard_active")]
    multiframe = [x for x in lines if int(x.get("multi_frame_source_frames", 1)) > 1]
    pose_jump = [x for x in lines if x.get("reason") == "pose_jump"]
    guard_hold = [x for x in lines if str(x.get("reason", "")).startswith("rotation_guard")]
    print(f"  rotation_guard_active_frames={len(guarded)}")
    print(f"  multi_frame_frames={len(multiframe)}")
    print(f"  ordinary_pose_jump_frames={len(pose_jump)}")
    print(f"  rotation_guard_hold_frames={len(guard_hold)}")

    if guarded:
        first = guarded[0].get("stamp_sec", 0.0)
        last = guarded[-1].get("stamp_sec", 0.0)
        print(f"  guard_time={first:.3f}..{last:.3f} duration={last - first:.2f}s")

    point5_start = 1779966921.0
    point5_end = 1779966945.0
    p5 = [x for x in lines if point5_start <= float(x.get("stamp_sec", 0.0)) <= point5_end]
    if p5:
        p5_reasons = Counter(x.get("reason", "") for x in p5)
        p5_guard = sum(1 for x in p5 if x.get("rotation_guard_active"))
        p5_mf = sum(1 for x in p5 if int(x.get("multi_frame_source_frames", 1)) > 1)
        print("\nPOINT5 WINDOW 1779966921..1779966945")
        print(f"  frames={len(p5)} guard={p5_guard} multiframe={p5_mf}")
        for reason, count in p5_reasons.most_common():
            print(f"    {reason or '<empty>'}: {count}")
        for x in p5:
            reason = x.get("reason", "")
            if reason == "pose_jump" or str(reason).startswith("rotation_guard"):
                print(
                    "    event "
                    f"t={x.get('stamp_sec', 0):.3f} state={x.get('state')} reason={reason} "
                    f"corr={x.get('correction_translation', 0):.3f} "
                    f"yaw={x.get('correction_yaw', 0):.3f} "
                    f"fit={x.get('fitness_score', 0):.3f} "
                    f"guard={x.get('rotation_guard_active')} "
                    f"mf={x.get('multi_frame_source_frames')}"
                )


def main():
    global ndt_proc
    rclpy.init()
    controller = Node("bag6_rotation_guard_controller")

    print("[0/5] loading point5 initial context from recorded /pcl_pose")
    initialpose, tf_static_msg = load_point5_initial_context()
    print(
        "  initial map->odom "
        f"x={initialpose.pose.pose.position.x:.3f} "
        f"y={initialpose.pose.pose.position.y:.3f}"
    )

    print("[1/5] starting NDT")
    ndt_proc = start_ndt()
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

    print("[3/5] starting status bridge and recorder")
    monitor = ReplayMonitor(initialpose, tf_static_msg)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    print("[4/5] playing bag")
    play = subprocess.run(
        [
            "ros2", "bag", "play", BAG, "--clock", "100",
            "--start-offset", str(POINT5_START_OFFSET_SEC),
            "--playback-duration", str(POINT5_PLAY_DURATION_SEC),
            "--topics",
            "/fast_lio/cloud_registered",
            "/tf",
            "/behavior_tree_log",
        ],
        text=True,
        capture_output=True,
        timeout=900,
    )
    if play.returncode != 0:
        print(play.stdout[-1000:])
        print(play.stderr[-1000:])
        print(f"ERROR: bag play failed with {play.returncode}")

    print("[5/5] analyzing")
    time.sleep(5.0)
    executor.shutdown()
    spin_thread.join(timeout=5.0)

    with open(OUTPUT, "w") as f:
        for raw in recorded:
            f.write(raw + "\n")

    lines = []
    for raw in recorded:
        try:
            lines.append(json.loads(raw))
        except json.JSONDecodeError:
            pass
    analyze(lines)

    cleanup()
    controller.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
