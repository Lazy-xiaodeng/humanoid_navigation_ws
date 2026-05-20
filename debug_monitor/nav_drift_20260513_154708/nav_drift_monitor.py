#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import signal
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from hdl_localization.msg import ScanMatchingStatus
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def path_len(path_msg):
    total = 0.0
    prev = None
    for ps in path_msg.poses:
        p = ps.pose.position
        if prev is not None:
            total += math.hypot(p.x - prev[0], p.y - prev[1])
        prev = (p.x, p.y)
    return total


class DriftMonitor(Node):
    def __init__(self, args):
        super().__init__("nav_drift_monitor")
        self.args = args
        self.out_dir = os.path.abspath(args.out_dir)
        os.makedirs(self.out_dir, exist_ok=True)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.start_wall = time.time()
        self.latest = {}
        self.sensor_stats = {}
        self.last_event_wall = {}
        self.last_tf_sample = {}

        self.sample_file = open(os.path.join(self.out_dir, "samples.csv"), "w", newline="")
        self.event_file = open(os.path.join(self.out_dir, "events.log"), "a", buffering=1)
        self.rosout_file = open(os.path.join(self.out_dir, "rosout_filtered.log"), "a", buffering=1)
        self.status_file = open(os.path.join(self.out_dir, "status.csv"), "w", newline="")

        self.sample_writer = None
        self.status_writer = csv.DictWriter(
            self.status_file,
            fieldnames=[
                "wall_time",
                "ros_time",
                "msg_stamp",
                "has_converged",
                "matching_error",
                "inlier_fraction",
                "relative_x",
                "relative_y",
                "relative_z",
                "relative_yaw",
            ],
        )
        self.status_writer.writeheader()

        self.create_subscription(ScanMatchingStatus, "/status", self.status_cb, 10)
        self.create_subscription(Odometry, "/hdl/odom", lambda m: self.odom_cb("hdl_odom", m), 20)
        self.create_subscription(Odometry, "/odom", lambda m: self.odom_cb("fastlio_odom", m), 20)
        self.create_subscription(PoseWithCovarianceStamped, "/robot_realpose", self.robot_pose_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 20)
        self.create_subscription(Path, "/plan", self.plan_cb, 10)
        self.create_subscription(Path, "/local_plan", self.local_plan_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.create_subscription(String, "/navigation/status", self.nav_status_cb, 10)
        self.create_subscription(Log, "/rosout", self.rosout_cb, 100)

        for topic, msg_type in [
            ("/fast_lio/cloud_registered", PointCloud2),
            ("/airy_points_filtered", PointCloud2),
            ("/airy_points", PointCloud2),
            ("/imu_standard", Imu),
            ("/airy_imu", Imu),
        ]:
            self.sensor_stats[topic] = {
                "count": 0,
                "last_wall": float("nan"),
                "last_stamp": float("nan"),
                "periods": deque(maxlen=50),
            }
            self.create_subscription(
                msg_type,
                topic,
                lambda msg, t=topic: self.sensor_cb(t, msg),
                self.sensor_qos,
            )

        self.tf_pairs = [
            ("map", "odom"),
            ("odom", "base_footprint"),
            ("map", "base_footprint"),
            ("map_ground", "base_footprint"),
            ("odom_ground", "base_footprint"),
        ]

        self.write_meta()
        self.log_event("monitor_start", "nav drift monitor started")
        self.timer = self.create_timer(1.0 / args.rate, self.sample_cb)

    def now_ros_sec(self):
        return stamp_to_sec(self.get_clock().now().to_msg())

    def wall_elapsed(self):
        return time.time() - self.start_wall

    def write_meta(self):
        path = os.path.join(self.out_dir, "meta.txt")
        with open(path, "w") as f:
            f.write(f"start_wall_epoch={self.start_wall:.6f}\n")
            f.write(f"out_dir={self.out_dir}\n")
            f.write(f"sample_rate_hz={self.args.rate}\n")
            f.write("tf_pairs=" + ",".join([f"{a}->{b}" for a, b in self.tf_pairs]) + "\n")

    def log_event(self, key, msg, throttle_sec=0.0):
        now = time.time()
        if throttle_sec > 0.0 and now - self.last_event_wall.get(key, 0.0) < throttle_sec:
            return
        self.last_event_wall[key] = now
        line = f"{self.wall_elapsed():.3f},ros={self.now_ros_sec():.6f},{key},{msg}"
        self.event_file.write(line + "\n")
        if not self.args.quiet:
            self.get_logger().warn(line)

    def status_cb(self, msg):
        rel = msg.relative_pose
        row = {
            "wall_time": f"{self.wall_elapsed():.6f}",
            "ros_time": f"{self.now_ros_sec():.6f}",
            "msg_stamp": f"{stamp_to_sec(msg.header.stamp):.6f}",
            "has_converged": int(msg.has_converged),
            "matching_error": f"{float(msg.matching_error):.6f}",
            "inlier_fraction": f"{float(msg.inlier_fraction):.6f}",
            "relative_x": f"{rel.translation.x:.6f}",
            "relative_y": f"{rel.translation.y:.6f}",
            "relative_z": f"{rel.translation.z:.6f}",
            "relative_yaw": f"{yaw_from_quat(rel.rotation):.6f}",
        }
        self.status_writer.writerow(row)
        self.status_file.flush()
        self.latest["status"] = {
            "stamp": stamp_to_sec(msg.header.stamp),
            "converged": bool(msg.has_converged),
            "error": float(msg.matching_error),
            "inlier": float(msg.inlier_fraction),
            "rel_x": rel.translation.x,
            "rel_y": rel.translation.y,
            "rel_yaw": yaw_from_quat(rel.rotation),
        }
        if (not msg.has_converged) or msg.matching_error > self.args.bad_fitness or msg.inlier_fraction < self.args.bad_inlier:
            self.log_event(
                "bad_scan_match",
                f"converged={int(msg.has_converged)} fitness={msg.matching_error:.4f} inlier={msg.inlier_fraction:.4f}",
                throttle_sec=1.0,
            )

    def odom_cb(self, key, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        tw = msg.twist.twist
        self.latest[key] = {
            "stamp": stamp_to_sec(msg.header.stamp),
            "frame": msg.header.frame_id,
            "child": msg.child_frame_id,
            "x": p.x,
            "y": p.y,
            "z": p.z,
            "yaw": yaw_from_quat(q),
            "vx": tw.linear.x,
            "vy": tw.linear.y,
            "wz": tw.angular.z,
        }

    def robot_pose_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest["robot_realpose"] = {
            "stamp": stamp_to_sec(msg.header.stamp),
            "frame": msg.header.frame_id,
            "x": p.x,
            "y": p.y,
            "z": p.z,
            "yaw": yaw_from_quat(q),
        }

    def cmd_vel_cb(self, msg):
        self.latest["cmd_vel"] = {"vx": msg.linear.x, "vy": msg.linear.y, "wz": msg.angular.z}

    def plan_cb(self, msg):
        self.latest["plan"] = {"stamp": stamp_to_sec(msg.header.stamp), "n": len(msg.poses), "len": path_len(msg)}

    def local_plan_cb(self, msg):
        self.latest["local_plan"] = {"stamp": stamp_to_sec(msg.header.stamp), "n": len(msg.poses), "len": path_len(msg)}

    def goal_cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        self.latest["goal"] = {
            "stamp": stamp_to_sec(msg.header.stamp),
            "frame": msg.header.frame_id,
            "x": p.x,
            "y": p.y,
            "z": p.z,
            "yaw": yaw_from_quat(q),
        }
        self.log_event("goal", f"frame={msg.header.frame_id} x={p.x:.3f} y={p.y:.3f} yaw={yaw_from_quat(q):.3f}")

    def nav_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            signature = {
                "current_state": data.get("current_state"),
                "detailed_state": data.get("detailed_state"),
                "navigation_mode": data.get("navigation_mode"),
                "sequence_id": data.get("sequence_id"),
                "current_waypoint_index": data.get("current_waypoint_index"),
                "total_waypoints": data.get("total_waypoints"),
                "is_active": data.get("is_active"),
                "recovery_active": data.get("recovery_active"),
                "obstacle_blocked": data.get("obstacle_blocked"),
                "block_reported": data.get("block_reported"),
            }
            event_text = json.dumps(signature, ensure_ascii=False)
        except Exception:
            signature = msg.data
            event_text = msg.data[:1000]

        old = self.latest.get("nav_status", {}).get("signature")
        self.latest["nav_status"] = {"signature": signature, "data": msg.data}
        if signature != old:
            self.log_event("nav_status", event_text)

    def sensor_cb(self, topic, msg):
        st = self.sensor_stats[topic]
        now = self.now_ros_sec()
        last_wall = st["last_wall"]
        if not math.isnan(last_wall):
            st["periods"].append(time.time() - last_wall)
        st["count"] += 1
        st["last_wall"] = time.time()
        st["last_stamp"] = stamp_to_sec(msg.header.stamp)
        st["last_age"] = now - st["last_stamp"]

    def rosout_cb(self, msg):
        interesting_name = any(
            token in msg.name
            for token in [
                "HdlLocalizationNodelet",
                "global_localization",
                "fast_lio",
                "controller_server",
                "planner_server",
                "bt_navigator",
                "costmap",
                "navigation_state_manager",
                "robot_realpose",
            ]
        )
        interesting_text = any(
            token in msg.msg
            for token in [
                "scan matching",
                "relocal",
                "TF",
                "tf",
                "extrapolation",
                "Failed",
                "failed",
                "reject",
                "fitness",
                "missed",
                "transform",
                "stuck",
            ]
        )
        if msg.level >= Log.WARN or interesting_name or interesting_text:
            self.rosout_file.write(
                f"{self.wall_elapsed():.3f},ros={stamp_to_sec(msg.stamp):.6f},level={msg.level},name={msg.name},msg={msg.msg}\n"
            )

    def lookup_tf(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except TransformException:
            return None
        tr = tf.transform.translation
        rot = tf.transform.rotation
        return {
            "stamp": stamp_to_sec(tf.header.stamp),
            "x": tr.x,
            "y": tr.y,
            "z": tr.z,
            "yaw": yaw_from_quat(rot),
        }

    def flatten_pose(self, row, prefix, data, include_vel=False):
        if not data:
            for suffix in ["stamp", "x", "y", "z", "yaw"]:
                row[f"{prefix}_{suffix}"] = ""
            if include_vel:
                for suffix in ["vx", "vy", "wz"]:
                    row[f"{prefix}_{suffix}"] = ""
            return
        for suffix in ["stamp", "x", "y", "z", "yaw"]:
            row[f"{prefix}_{suffix}"] = data.get(suffix, "")
        if include_vel:
            for suffix in ["vx", "vy", "wz"]:
                row[f"{prefix}_{suffix}"] = data.get(suffix, "")

    def flatten_tf(self, row, prefix, data):
        if not data:
            for suffix in ["stamp", "x", "y", "z", "yaw"]:
                row[f"{prefix}_{suffix}"] = ""
            return
        for suffix in ["stamp", "x", "y", "z", "yaw"]:
            row[f"{prefix}_{suffix}"] = data.get(suffix, "")

    def check_jump(self, label, tf):
        if tf is None:
            return
        prev = self.last_tf_sample.get(label)
        self.last_tf_sample[label] = dict(tf)
        if prev is None:
            return
        dxy = math.hypot(tf["x"] - prev["x"], tf["y"] - prev["y"])
        dyaw = abs(angle_delta(tf["yaw"], prev["yaw"]))
        if label == "map__odom":
            if dxy > self.args.map_odom_jump_xy or dyaw > self.args.map_odom_jump_yaw:
                self.log_event("map_odom_jump", f"dxy={dxy:.3f} dyaw={dyaw:.3f} x={tf['x']:.3f} y={tf['y']:.3f} yaw={tf['yaw']:.3f}")
        elif label == "map__base_footprint":
            if dxy > self.args.map_base_jump_xy or dyaw > self.args.map_base_jump_yaw:
                self.log_event("map_base_jump", f"dxy={dxy:.3f} dyaw={dyaw:.3f} x={tf['x']:.3f} y={tf['y']:.3f} yaw={tf['yaw']:.3f}")

    def sample_cb(self):
        row = {
            "wall_time": f"{self.wall_elapsed():.6f}",
            "ros_time": f"{self.now_ros_sec():.6f}",
        }

        tf_data = {}
        for target, source in self.tf_pairs:
            label = f"{target}__{source}"
            tf = self.lookup_tf(target, source)
            tf_data[label] = tf
            self.flatten_tf(row, f"tf_{label}", tf)
            self.check_jump(label, tf)

        status = self.latest.get("status", {})
        row.update(
            {
                "status_stamp": status.get("stamp", ""),
                "status_converged": int(status.get("converged", False)) if status else "",
                "status_error": status.get("error", ""),
                "status_inlier": status.get("inlier", ""),
                "status_rel_x": status.get("rel_x", ""),
                "status_rel_y": status.get("rel_y", ""),
                "status_rel_yaw": status.get("rel_yaw", ""),
            }
        )

        self.flatten_pose(row, "hdl_odom", self.latest.get("hdl_odom"), include_vel=True)
        self.flatten_pose(row, "fastlio_odom", self.latest.get("fastlio_odom"), include_vel=True)
        self.flatten_pose(row, "robot_realpose", self.latest.get("robot_realpose"))

        cmd = self.latest.get("cmd_vel", {})
        row["cmd_vx"] = cmd.get("vx", "")
        row["cmd_vy"] = cmd.get("vy", "")
        row["cmd_wz"] = cmd.get("wz", "")

        plan = self.latest.get("plan", {})
        row["plan_n"] = plan.get("n", "")
        row["plan_len"] = plan.get("len", "")
        local_plan = self.latest.get("local_plan", {})
        row["local_plan_n"] = local_plan.get("n", "")
        row["local_plan_len"] = local_plan.get("len", "")
        goal = self.latest.get("goal", {})
        row["goal_x"] = goal.get("x", "")
        row["goal_y"] = goal.get("y", "")
        row["goal_yaw"] = goal.get("yaw", "")

        for topic, st in self.sensor_stats.items():
            prefix = "topic_" + topic.strip("/").replace("/", "_")
            periods = list(st["periods"])
            hz = (1.0 / (sum(periods) / len(periods))) if periods else ""
            row[f"{prefix}_count"] = st["count"]
            row[f"{prefix}_hz"] = hz
            row[f"{prefix}_stamp"] = st.get("last_stamp", "")
            row[f"{prefix}_age"] = st.get("last_age", "")

        if self.sample_writer is None:
            self.sample_writer = csv.DictWriter(self.sample_file, fieldnames=list(row.keys()))
            self.sample_writer.writeheader()
        self.sample_writer.writerow(row)
        self.sample_file.flush()

    def close(self):
        self.log_event("monitor_stop", "nav drift monitor stopped")
        for f in [self.sample_file, self.event_file, self.rosout_file, self.status_file]:
            try:
                f.flush()
                f.close()
            except Exception:
                pass


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--bad-fitness", type=float, default=1.5)
    parser.add_argument("--bad-inlier", type=float, default=0.55)
    parser.add_argument("--map-odom-jump-xy", type=float, default=0.20)
    parser.add_argument("--map-odom-jump-yaw", type=float, default=0.12)
    parser.add_argument("--map-base-jump-xy", type=float, default=0.45)
    parser.add_argument("--map-base-jump-yaw", type=float, default=0.35)
    parser.add_argument("--quiet", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = DriftMonitor(args)

    stopping = {"value": False}

    def handle_signal(signum, frame):
        stopping["value"] = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not stopping["value"]:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
