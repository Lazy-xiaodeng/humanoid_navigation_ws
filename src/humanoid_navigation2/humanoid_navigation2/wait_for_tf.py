#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class WaitForTf(Node):
    def __init__(self):
        super().__init__('wait_for_tf')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_footprint')
        self.declare_parameter('timeout_sec', 0.0)
        self.declare_parameter('poll_period', 0.2)
        self.declare_parameter('stable_count', 3)

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.poll_period = max(0.05, float(self.get_parameter('poll_period').value))
        self.stable_count = max(1, int(self.get_parameter('stable_count').value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_time = time.monotonic()
        self.last_warn_time = 0.0
        self.ready_count = 0

        self.get_logger().info(
            f'waiting for TF {self.target_frame}->{self.source_frame} '
            f'before starting dependent nodes'
        )

        self.timer = self.create_timer(self.poll_period, self.timer_callback)

    def timer_callback(self):
        try:
            self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())
            self.ready_count += 1
            if self.ready_count >= self.stable_count:
                self.get_logger().info(
                    f'TF ready: {self.target_frame}->{self.source_frame}; '
                    'dependent nodes may start'
                )
                rclpy.shutdown()
                return
        except TransformException as exc:
            self.ready_count = 0
            now = time.monotonic()
            if now - self.last_warn_time > 5.0:
                self.get_logger().warn(
                    f'TF not ready: {self.target_frame}->{self.source_frame}: {exc}'
                )
                self.last_warn_time = now

        if self.timeout_sec > 0.0 and time.monotonic() - self.start_time > self.timeout_sec:
            self.get_logger().error(
                f'timed out waiting for TF {self.target_frame}->{self.source_frame}'
            )
            rclpy.shutdown()
            sys.exit(1)


def main(args=None):
    rclpy.init(args=args)
    node = WaitForTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
