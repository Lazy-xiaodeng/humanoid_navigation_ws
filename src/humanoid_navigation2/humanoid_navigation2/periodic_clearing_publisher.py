#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
periodic_clearing_publisher.py

全向暴力清除版。

功能：
  定时发布 /clearing_scan。
  不再读取点云。
  不再根据点云生成最近障碍距离。
  每个方向都发布 RANGE_MAX，用于 Nav2 obstacle_layer 的 clearing。

用途：
  清除 costmap 中动态障碍物残影。
  特别适合处理：
    - 人走过后 costmap 残留
    - 点云没有残影但 costmap 不清除
    - 地面反光/瓷砖噪点导致普通 clearing_scan 被截断

注意：
  这个话题只能在 costmap 中配置为：
    marking: False
    clearing: True
  绝对不要用于 marking。
"""

import math

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy

from sensor_msgs.msg import LaserScan

# ==============================================================================
# 参数区
# ==============================================================================

OUTPUT_SCAN_TOPIC = "/clearing_scan"

# 使用 base_footprint，让清除射线始终从机器人脚下中心发出
TARGET_FRAME = "base_footprint"

# 全向 360 度
ANGLE_MIN_DEG = -180.0
ANGLE_MAX_DEG = 180.0

# 角度分辨率
# 1.0 度已经足够暴力清除了，没必要 0.5 度，减轻 costmap 压力
ANGLE_INCREMENT_DEG = 1.0

# 清除范围
# 建议先 4~5m，不要一上来 10m / 20m
RANGE_MIN = 0.05
RANGE_MAX = 8.0

# 发布频率
# 建议 1~2Hz，太高可能把真实动态障碍冲掉导致闪烁
PUBLISH_RATE_HZ = 2.0

# 模式：
#   "range_max"：每个方向发布 RANGE_MAX，推荐
#   "inf"      ：每个方向发布 inf，需要 costmap 里 inf_is_valid: True
CLEAR_MODE = "range_max"
# CLEAR_MODE = "inf"

class PeriodicClearingPublisher(Node):
    def __init__(self):
        super().__init__("periodic_clearing_publisher")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.pub = self.create_publisher(
            LaserScan,
            OUTPUT_SCAN_TOPIC,
            sensor_qos
        )

        self.angle_min = math.radians(ANGLE_MIN_DEG)
        self.angle_max = math.radians(ANGLE_MAX_DEG)
        self.angle_increment = math.radians(ANGLE_INCREMENT_DEG)

        self.scan_size = int(
            math.floor((self.angle_max - self.angle_min) / self.angle_increment)
        ) + 1

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self.publish_clearing_scan
        )

        self.get_logger().info("全向暴力 clearing_scan 节点已启动")
        self.get_logger().info(f"output topic      : {OUTPUT_SCAN_TOPIC}")
        self.get_logger().info(f"target frame      : {TARGET_FRAME}")
        self.get_logger().info(f"angle range       : {ANGLE_MIN_DEG} ~ {ANGLE_MAX_DEG} deg")
        self.get_logger().info(f"angle increment   : {ANGLE_INCREMENT_DEG} deg")
        self.get_logger().info(f"scan size         : {self.scan_size}")
        self.get_logger().info(f"range             : {RANGE_MIN} ~ {RANGE_MAX} m")
        self.get_logger().info(f"publish rate      : {PUBLISH_RATE_HZ} Hz")
        self.get_logger().info(f"clear mode        : {CLEAR_MODE}")

    def publish_clearing_scan(self):
        scan_msg = LaserScan()

        scan_msg.header.frame_id = TARGET_FRAME

        # 关键：必须用当前时间，避免 MessageFilter 因 TF 时间戳丢消息
        scan_msg.header.stamp = self.get_clock().now().to_msg()

        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_min + self.angle_increment * (self.scan_size - 1)
        scan_msg.angle_increment = self.angle_increment

        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / PUBLISH_RATE_HZ

        scan_msg.range_min = RANGE_MIN
        scan_msg.range_max = RANGE_MAX

        if CLEAR_MODE == "inf":
            scan_msg.ranges = [float("inf")] * self.scan_size
        else:
            # 推荐模式：每个方向都发布 RANGE_MAX
            # 对 costmap 来说就是每个方向都可以 raytrace 清除到 RANGE_MAX
            scan_msg.ranges = [RANGE_MAX] * self.scan_size

        scan_msg.intensities = []

        self.pub.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PeriodicClearingPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()