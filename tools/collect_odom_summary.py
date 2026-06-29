#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry


def stamp_sec(msg):
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def pos_tuple(msg):
    p = msg.pose.pose.position
    return (float(p.x), float(p.y), float(p.z))


def fmt(sample):
    t, p = sample
    norm = math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])
    return f"stamp={t:.9f} x={p[0]:.6f} y={p[1]:.6f} z={p[2]:.6f} norm={norm:.6f}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--duration", type=float, default=55.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("odom_summary_collector")
    samples = []

    def cb(msg):
        samples.append((stamp_sec(msg), pos_tuple(msg)))

    node.create_subscription(Odometry, args.topic, cb, 100)

    deadline = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    print(f"topic={args.topic} count={len(samples)}")
    if not samples:
        return
    print("first " + fmt(samples[0]))
    print("mid   " + fmt(samples[len(samples) // 2]))
    print("last  " + fmt(samples[-1]))


if __name__ == "__main__":
    main()
