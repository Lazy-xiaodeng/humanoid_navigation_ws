#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
 periodic_clearing_3d_publisher.py

 功能：
  给 Nav2 local_costmap 的 VoxelLayer 发布一个专用 3D clearing 点云：
    /clearing_cloud_3d

 最终策略：
  1. /airy_points_filtered 负责 marking 当前真实障碍；
  2. /clearing_cloud_3d 负责持续 raytrace clearing 旧体素；
  3. 本节点不再根据输入点云判断 occupied_bins；
  4. 本节点发布所有方向、多距离、多高度的 clearing endpoint。

"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# ==============================================================================
# 固定参数区
# ==============================================================================

# ------------------------------------------------------------------------------
# 输入点云话题
#
# 这里只作为 heartbeat 使用。
# 也就是说：
#   只要这个话题持续有消息，说明感知系统在运行，
#   本节点就按 MAX_PUBLISH_RATE_HZ 发布 clearing cloud。
#
# 不再解析这个点云的内容。
# ------------------------------------------------------------------------------
INPUT_POINTS_TOPIC = "/fast_lio/cloud_registered"

# ------------------------------------------------------------------------------
# 输出 clearing 点云话题
# ------------------------------------------------------------------------------
OUTPUT_CLEARING_CLOUD_TOPIC = "/clearing_cloud_3d"

# ------------------------------------------------------------------------------
# 输出坐标系
#
# 输出 clearing cloud 的点全部在 base_footprint 坐标系下。
# 对 local_costmap 来说，这样 TF 最简单。
# ------------------------------------------------------------------------------
TARGET_FRAME = "base_footprint"

# ------------------------------------------------------------------------------
# 水平角度范围
#
# 360 度 clearing。
# ------------------------------------------------------------------------------
ANGLE_MIN_DEG = -180.0
ANGLE_MAX_DEG = 180.0

# 角度分辨率。
# 1 度通常足够。
# 如果想更密，可以改成 0.5，但点数会翻倍。
ANGLE_INCREMENT_DEG = 1.0

# ------------------------------------------------------------------------------
# clearing 射线距离范围
#
# 你的 local_costmap 是 8m x 8m，机器人在中心，半径约 4m。
# 所以 5m 已经足够覆盖整个 local costmap。
#
# 不建议继续用 10m，否则点数和 CPU 压力会明显增加。
# ------------------------------------------------------------------------------
CLEARING_RANGE_MIN = 0.30
CLEARING_RANGE_MAX = 5.0
CLEARING_RANGE_STEP = 0.50

# ------------------------------------------------------------------------------
# 3D clearing 高度层
#
# 这些高度决定 VoxelLayer 中哪些 Z 层会被 raytrace clearing。
#
# 需要覆盖人体、桌腿、箱子等障碍常见高度。
# ------------------------------------------------------------------------------
CLEARING_HEIGHT_LAYERS = [
    0.05,
    0.08,
    0.10,
    0.30,
    0.50,
    0.70,
    0.90,
    1.10,
    1.30,
    1.50,
    1.70,
]

# ------------------------------------------------------------------------------
# 发布频率限制
#
# 暴力 3D clearing 点云比较大，不需要 10Hz。
# 2Hz 通常足够清除动态残影，同时 CPU 压力较低。
# ------------------------------------------------------------------------------
MAX_PUBLISH_RATE_HZ = 2.0

# ------------------------------------------------------------------------------
# QoS
#
# 点云/雷达一般使用 BEST_EFFORT。
# Nav2 costmap observation source 通常可以接收 sensor data QoS。
# ------------------------------------------------------------------------------
QOS_DEPTH = 5

# ------------------------------------------------------------------------------
# 调试开关
# ------------------------------------------------------------------------------
DEBUG = False

# 日志节流
WARN_THROTTLE_SEC = 2.0

class PeriodicClearing3DPublisher(Node):
    """
    3D VoxelLayer clearing 发布节点。

    输入：
      /fast_lio/cloud_registered

    输出：
      /clearing_cloud_3d

    说明：
      输入点云只作为 heartbeat。
      本节点不读取、不解析、不判断输入点云内容。
    """

    def __init__(self):
        super().__init__("periodic_clearing_3d_publisher")

        # ----------------------------------------------------------------------
        # QoS
        # ----------------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=QOS_DEPTH,
        )

        # ----------------------------------------------------------------------
        # 订阅输入点云 heartbeat
        # ----------------------------------------------------------------------
        self.sub = self.create_subscription(
            PointCloud2,
            INPUT_POINTS_TOPIC,
            self.cloud_callback,
            sensor_qos,
        )

        # ----------------------------------------------------------------------
        # 发布 3D clearing cloud
        # ----------------------------------------------------------------------
        self.pub = self.create_publisher(
            PointCloud2,
            OUTPUT_CLEARING_CLOUD_TOPIC,
            sensor_qos,
        )

        # ----------------------------------------------------------------------
        # 角度预计算
        # ----------------------------------------------------------------------
        self.angle_min = math.radians(ANGLE_MIN_DEG)
        self.angle_max = math.radians(ANGLE_MAX_DEG)
        self.angle_increment = math.radians(ANGLE_INCREMENT_DEG)

        self.bin_count = int(
            math.floor((self.angle_max - self.angle_min) / self.angle_increment)
        ) + 1

        if self.bin_count <= 0:
            raise RuntimeError("bin_count <= 0，请检查角度参数")

        self.bin_angles = [
            self.angle_min + i * self.angle_increment
            for i in range(self.bin_count)
        ]

        # ----------------------------------------------------------------------
        # 发布限频状态
        # ----------------------------------------------------------------------
        self.last_publish_time = 0.0
        self.last_warn_time = 0.0

        # ----------------------------------------------------------------------
        # 预生成 clearing 点
        #
        # clearing 点云形状固定，因此只生成一次。
        # 每次发布时只更新时间戳和 frame_id。
        # ----------------------------------------------------------------------
        self.clearing_points = self.build_clearing_points()

        # ----------------------------------------------------------------------
        # 启动日志
        # ----------------------------------------------------------------------
        self.get_logger().info("periodic_clearing_3d_publisher started.")
        self.get_logger().info(f"input_points_topic        : {INPUT_POINTS_TOPIC}")
        self.get_logger().info(f"output_clearing_cloud     : {OUTPUT_CLEARING_CLOUD_TOPIC}")
        self.get_logger().info(f"target_frame              : {TARGET_FRAME}")
        self.get_logger().info(f"angle_min_deg             : {ANGLE_MIN_DEG}")
        self.get_logger().info(f"angle_max_deg             : {ANGLE_MAX_DEG}")
        self.get_logger().info(f"angle_increment_deg       : {ANGLE_INCREMENT_DEG}")
        self.get_logger().info(f"bin_count                 : {self.bin_count}")
        self.get_logger().info(
            f"clearing_range            : "
            f"{CLEARING_RANGE_MIN} ~ {CLEARING_RANGE_MAX}, "
            f"step={CLEARING_RANGE_STEP}"
        )
        self.get_logger().info(f"clearing_height_layers    : {CLEARING_HEIGHT_LAYERS}")
        self.get_logger().info(f"max_publish_rate_hz       : {MAX_PUBLISH_RATE_HZ}")
        self.get_logger().info(
            f"precomputed clearing_points : {len(self.clearing_points)}"
        )

    def warn_throttled(self, msg):
        """
        节流 warning，避免刷屏。
        """
        now = time.time()

        if now - self.last_warn_time > WARN_THROTTLE_SEC:
            self.get_logger().warn(msg)
            self.last_warn_time = now

    def should_publish_now(self):
        """
        控制最大发布频率。
        """
        if MAX_PUBLISH_RATE_HZ <= 0.0:
            return True

        now = time.time()
        min_dt = 1.0 / MAX_PUBLISH_RATE_HZ

        if now - self.last_publish_time >= min_dt:
            self.last_publish_time = now
            return True

        return False

    def build_clearing_points(self):
        """
        生成固定形状的 3D clearing endpoint 点云。

        策略：
          所有方向 × 多个距离 × 多个高度

        不再判断 occupied_bins。
        不再跳过任何方向。

        原因：
          真实障碍由 /airy_points_filtered 下一帧重新 marking。
          本节点只负责清除 VoxelLayer 中的旧体素。
        """

        clearing_points = []

        # ----------------------------------------------------------------------
        # 距离层
        #
        # 例如：
        #   0.30, 0.80, 1.30, ..., 4.80
        # ----------------------------------------------------------------------
        ranges = []
        r = CLEARING_RANGE_MIN

        while r <= CLEARING_RANGE_MAX + 1e-6:
            ranges.append(r)
            r += CLEARING_RANGE_STEP

        # ----------------------------------------------------------------------
        # 角度 × 距离 × 高度
        # ----------------------------------------------------------------------
        for angle in self.bin_angles:
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            for rr in ranges:
                x = rr * cos_a
                y = rr * sin_a

                for z in CLEARING_HEIGHT_LAYERS:
                    clearing_points.append([x, y, z])

        return clearing_points

    def cloud_callback(self, cloud_msg):
        """
        输入点云回调。

        注意：
          不解析 cloud_msg。
          cloud_msg 只作为 heartbeat 使用。

        只要输入点云持续到来，本节点按 MAX_PUBLISH_RATE_HZ 发布：
          /clearing_cloud_3d
        """

        if not self.should_publish_now():
            return

        if len(self.clearing_points) == 0:
            self.warn_throttled("clearing_points 为空，跳过发布")
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = TARGET_FRAME

        clearing_cloud_msg = pc2.create_cloud_xyz32(
            header,
            self.clearing_points,
        )

        self.pub.publish(clearing_cloud_msg)

        if DEBUG:
            self.get_logger().info(
                f"3D brute clearing published: "
                f"clearing_points={len(self.clearing_points)}"
            )

def main(args=None):
    rclpy.init(args=args)

    node = PeriodicClearing3DPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()