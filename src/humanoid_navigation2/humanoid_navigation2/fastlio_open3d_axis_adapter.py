#!/usr/bin/env python3
"""
fastlio_open3d_axis_adapter.py

这个节点专门解决本系统的 Fast-LIO 原始坐标轴问题。

背景：
  Fast-LIO 原始 camera_init/body 坐标轴为：
    x 朝左、y 朝下、z 朝后

  Nav2 / 2D map / grounded PCD 使用 ROS 标准导航坐标轴：
    x 朝前、y 朝左、z 朝上

如果让 open3d_loc 直接订阅 raw /odom 和 raw /fast_lio/cloud_registered，
它内部配准可以运行，但输出位姿会停留在 raw Fast-LIO 坐标约定里。
随后 prior_map_odom_bridge 再拿 TF 树里的标准 odom->base_footprint 去相乘，
就会把两套坐标轴混在一起，表现为方向错、平移错，严重时看起来像定位跳变。

本节点做两件事：
  1. 把 raw Fast-LIO 世界点云转换成标准轴 grounded 点云，供 open3d_loc 配准。
  2. 把 raw /odom 中的 camera_init->body 位姿转换成标准轴
     odom->prior_open3d_base，并发布同名 TF，供 bridge 精确计算 map->odom。

转换公式与 scripts/make_localization_pcd.py 保持一致：
  p_std = R_raw_to_std * (p_raw - t_initial_body_to_base_raw)

其中 t_initial_body_to_base_raw 来自 navigation2.launch.py 里的 body->base_footprint
平移。这样地图 PCD、实时点云、open3d 位姿输入都在同一套标准坐标里。
"""

import math
from typing import Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster


def normalize_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float, float, float, float]:
    """归一化四元数，避免上游偶发的非单位四元数污染矩阵计算。"""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """ROS 四元数(x,y,z,w) -> 3x3 旋转矩阵。"""
    x, y, z, w = normalize_quaternion(x, y, z, w)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(matrix: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 旋转矩阵 -> ROS 四元数(x,y,z,w)。"""
    m = matrix
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(max(1.0 + m[0, 0] - m[1, 1] - m[2, 2], 1e-12)) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(max(1.0 + m[1, 1] - m[0, 0] - m[2, 2], 1e-12)) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(max(1.0 + m[2, 2] - m[0, 0] - m[1, 1], 1e-12)) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return normalize_quaternion(qx, qy, qz, qw)


class FastLioOpen3dAxisAdapter(Node):
    """把 Fast-LIO raw 轴输入转换成 open3d_loc 可直接使用的标准轴输入。"""

    def __init__(self):
        super().__init__("fastlio_open3d_axis_adapter")

        self.raw_odom_topic = self.declare_parameter("raw_odom_topic", "/odom").value
        self.raw_cloud_topic = self.declare_parameter("raw_cloud_topic", "/fast_lio/cloud_registered").value
        self.output_odom_topic = self.declare_parameter(
            "output_odom_topic", "/prior_localization/open3d_input_odom").value
        self.output_cloud_topic = self.declare_parameter(
            "output_cloud_topic", "/prior_localization/open3d_input_cloud").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.output_base_frame = self.declare_parameter("output_base_frame", "prior_open3d_base").value
        self.publish_tf = bool(self.declare_parameter("publish_tf", True).value)

        # 与 make_localization_pcd.py 中的 R_CAMERA_TO_MAP 完全一致：
        #   X_std = -Z_raw, Y_std = X_raw, Z_std = -Y_raw
        self.raw_to_std = np.array(
            [
                [0.0, 0.0, -1.0],
                [1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ],
            dtype=np.float64,
        )

        # body->base_footprint 的 raw 平移，来自 navigation2.launch.py。
        # 地图生成脚本会减掉这段平移，使 grounded PCD 的原点落在初始 base_footprint。
        offset = self.declare_parameter(
            "initial_body_to_base_translation_raw",
            [0.004, 1.215, 0.072],
        ).value
        self.initial_body_to_base_translation_raw = np.asarray(offset, dtype=np.float64)

        # body->base_footprint 的旋转。它把 raw body 轴旋到标准 base_footprint 轴。
        self.body_to_base_rotation_raw = np.array(
            [
                [0.0, 1.0, 0.0],
                [0.0, 0.0, -1.0],
                [-1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )

        input_cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
        )
        output_cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
        )

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 30)
        # open3d_loc 的 C++ 订阅默认是 reliable。这里输出必须用 reliable，
        # 否则 DDS 会报 QoS 不兼容，外部定位节点收不到任何点云。
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_cloud_topic, output_cloud_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, self.raw_odom_topic, self.on_raw_odom, 50)
        self.create_subscription(PointCloud2, self.raw_cloud_topic, self.on_raw_cloud, input_cloud_qos)

        self.get_logger().info(
            "fastlio_open3d_axis_adapter started: "
            f"{self.raw_odom_topic}->{self.output_odom_topic}, "
            f"{self.raw_cloud_topic}->{self.output_cloud_topic}, "
            f"tf={self.odom_frame}->{self.output_base_frame}"
        )

    def on_raw_odom(self, msg: Odometry):
        """
        raw /odom 里通常是 camera_init->body。

        这里先算 raw camera_init 下的 base_footprint 位姿，再转到标准 odom：
          p_base_std = R_raw_to_std * (p_body_raw + R_body_raw * t_body_base - t_initial_base)
          R_base_std = R_raw_to_std * R_body_raw * R_body_to_base_raw

        初始时 p_body_raw=0、R_body_raw=I，所以 p_base_std=0、R_base_std=I。
        """
        pose = msg.pose.pose
        raw_rotation = quaternion_to_matrix(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        raw_translation = np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float64,
        )

        base_translation_raw = raw_translation + raw_rotation @ self.initial_body_to_base_translation_raw
        base_translation_std = self.raw_to_std @ (
            base_translation_raw - self.initial_body_to_base_translation_raw
        )
        base_rotation_std = self.raw_to_std @ raw_rotation @ self.body_to_base_rotation_raw
        qx, qy, qz, qw = matrix_to_quaternion(base_rotation_std)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.output_base_frame
        out.pose.pose.position.x = float(base_translation_std[0])
        out.pose.pose.position.y = float(base_translation_std[1])
        out.pose.pose.position.z = float(base_translation_std[2])
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist = msg.twist
        self.odom_pub.publish(out)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header = out.header
            tf_msg.child_frame_id = self.output_base_frame
            tf_msg.transform.translation.x = out.pose.pose.position.x
            tf_msg.transform.translation.y = out.pose.pose.position.y
            tf_msg.transform.translation.z = out.pose.pose.position.z
            tf_msg.transform.rotation = out.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    def on_raw_cloud(self, msg: PointCloud2):
        """
        将 Fast-LIO 已配准到 camera_init/raw 世界系的点云转成标准 grounded 世界系。

        这里只保留 x/y/z，open3d_loc 的 ICP 不依赖 intensity/ring/time 字段。
        如果后续需要颜色或强度，可以在这里扩展字段复制逻辑。
        """
        try:
            points = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        except Exception as exc:
            self.get_logger().warn(f"failed to read raw cloud: {exc}")
            return

        if points.size == 0:
            return

        xyz_raw = np.asarray(points[:, :3], dtype=np.float64)
        xyz_std = (self.raw_to_std @ (xyz_raw - self.initial_body_to_base_translation_raw).T).T

        out = pc2.create_cloud_xyz32(msg.header, xyz_std.astype(np.float32))
        out.header.frame_id = self.odom_frame
        self.cloud_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FastLioOpen3dAxisAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
