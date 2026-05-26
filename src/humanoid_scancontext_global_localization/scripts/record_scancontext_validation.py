#!/usr/bin/env python3
"""Record and summarize ScanContext sidecar localization results."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ScanContextValidationRecorder(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("scancontext_validation_recorder")
        self.args = args
        self.latest_odom: Odometry | None = None
        self.pending_trigger = False
        self.rows: list[dict[str, object]] = []

        self.output = Path(args.output).expanduser()
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.output.open("w", newline="", encoding="utf-8")
        self.fieldnames = [
            "wall_time",
            "stamp",
            "accepted",
            "keyframe_id",
            "sc_distance",
            "gicp_fitness",
            "best_x",
            "best_y",
            "best_z",
            "best_yaw_deg",
            "odom_x",
            "odom_y",
            "odom_z",
            "odom_yaw_deg",
            "error_xy",
            "error_z",
            "error_yaw_deg",
            "cloud_frame_mode",
            "raw_json",
        ]
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)
        self.writer.writeheader()

        self.create_subscription(Odometry, args.odom_topic, self.odom_callback, 50)
        self.create_subscription(String, args.candidates_topic, self.candidates_callback, 20)
        self.trigger_client = self.create_client(Trigger, args.trigger_service)
        self.trigger_timer = self.create_timer(args.trigger_period, self.trigger_once)
        self.get_logger().info(f"recording ScanContext validation to {self.output}")

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def trigger_once(self) -> None:
        if self.pending_trigger:
            return
        if not self.trigger_client.service_is_ready():
            self.trigger_client.wait_for_service(timeout_sec=0.05)
            return
        self.pending_trigger = True
        future = self.trigger_client.call_async(Trigger.Request())
        future.add_done_callback(self.trigger_done)

    def trigger_done(self, future) -> None:
        self.pending_trigger = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"trigger failed: {exc}")
            return
        self.get_logger().info(f"trigger success={response.success}: {response.message}")

    def candidates_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("failed to parse candidates JSON")
            return

        best = data.get("best_pose", {})
        candidates = data.get("candidates", [])
        first = candidates[0] if candidates else {}
        row: dict[str, object] = {
            "wall_time": f"{time.time():.6f}",
            "stamp": data.get("stamp", ""),
            "accepted": bool(data.get("accepted", False)),
            "keyframe_id": first.get("keyframe_id", ""),
            "sc_distance": first.get("distance", ""),
            "gicp_fitness": data.get("gicp_fitness", ""),
            "best_x": best.get("x", ""),
            "best_y": best.get("y", ""),
            "best_z": best.get("z", ""),
            "best_yaw_deg": best.get("yaw_deg", ""),
            "odom_x": "",
            "odom_y": "",
            "odom_z": "",
            "odom_yaw_deg": "",
            "error_xy": "",
            "error_z": "",
            "error_yaw_deg": "",
            "cloud_frame_mode": data.get("cloud_frame_mode", ""),
            "raw_json": msg.data,
        }

        if self.latest_odom is not None and {"x", "y", "z", "yaw_deg"} <= set(best):
            p = self.latest_odom.pose.pose.position
            odom_yaw = yaw_from_quat(self.latest_odom.pose.pose.orientation)
            dx = float(best["x"]) - float(p.x)
            dy = float(best["y"]) - float(p.y)
            dz = float(best["z"]) - float(p.z)
            dyaw = wrap_pi(math.radians(float(best["yaw_deg"])) - odom_yaw)
            row.update(
                {
                    "odom_x": f"{p.x:.6f}",
                    "odom_y": f"{p.y:.6f}",
                    "odom_z": f"{p.z:.6f}",
                    "odom_yaw_deg": f"{math.degrees(odom_yaw):.3f}",
                    "error_xy": f"{math.hypot(dx, dy):.6f}",
                    "error_z": f"{abs(dz):.6f}",
                    "error_yaw_deg": f"{abs(math.degrees(dyaw)):.3f}",
                }
            )

        self.writer.writerow(row)
        self.csv_file.flush()
        self.rows.append(row)

    def close(self) -> None:
        self.csv_file.close()

    def print_summary(self) -> None:
        if not self.rows:
            print("No ScanContext results recorded.")
            return
        accepted = [r for r in self.rows if r["accepted"]]
        print(f"records: {len(self.rows)}")
        print(f"accepted: {len(accepted)} ({100.0 * len(accepted) / len(self.rows):.1f}%)")
        for label, key in (
            ("xy error m", "error_xy"),
            ("z error m", "error_z"),
            ("yaw error deg", "error_yaw_deg"),
            ("sc distance", "sc_distance"),
            ("gicp fitness", "gicp_fitness"),
        ):
            values = [float(r[key]) for r in accepted if r.get(key) not in ("", None)]
            if values:
                arr = np.asarray(values, dtype=np.float32)
                print(
                    f"{label}: median={np.median(arr):.4f}, "
                    f"p95={np.quantile(arr, 0.95):.4f}, max={np.max(arr):.4f}"
                )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="/tmp/scancontext_validation.csv")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--trigger-period", type=float, default=2.0)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument(
        "--candidates-topic",
        default="/scancontext_global_localization/candidates",
    )
    parser.add_argument(
        "--trigger-service",
        default="/scancontext_global_localization/trigger",
    )
    args = parser.parse_args()

    rclpy.init()
    node = ScanContextValidationRecorder(args)
    end_time = time.time() + args.duration if args.duration > 0.0 else None
    try:
        while rclpy.ok() and (end_time is None or time.time() < end_time):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.print_summary()
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
