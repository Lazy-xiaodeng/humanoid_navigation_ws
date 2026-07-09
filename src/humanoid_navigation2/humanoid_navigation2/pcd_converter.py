#!/usr/bin/env python3
"""
PCD 地图坐标系转换工具 (一次性使用)

将 Fast-LIO 建图得到的 PCD 地图从非标准 LiDAR 坐标系 (camera_init: X左 Y下 Z后)
转换到 ROS 标准坐标系 (X前 Y左 Z上)，供 hdl_localization 使用。

转换使用的是与现有 lidar_localization 相同的旋转：
  q_cam_to_ros = (w=0.5, x=-0.5, y=-0.5, z=0.5)
  等效于绕轴 (-1,-1,1)/√3 旋转 120 度

用法:
  python3 pcd_converter.py <输入.pcd> [输出.pcd]

  默认输出: 输入文件名 + _standard.pcd

示例:
  python3 pcd_converter.py /path/to/hall.pcd
  → 生成 /path/to/hall_standard.pcd
"""

import sys
import os
import numpy as np
import open3d as o3d


# 与 lidar_localization 中 q_cam_to_ros 一致的旋转矩阵
# Eigen::Quaternionf q_cam_to_ros(0.5, -0.5, -0.5, 0.5); // (w, x, y, z)
# 旋转矩阵: R * p_lidar = p_ros
#   X_ros = 0*X_lidar + 0*Y_lidar + (-1)*Z_lidar = -Z_lidar  (向后变向前)
#   Y_ros = 1*X_lidar + 0*Y_lidar + 0*Z_lidar     =  X_lidar  (右保持不变)
#   Z_ros = 0*X_lidar + (-1)*Y_lidar + 0*Z_lidar  = -Y_lidar  (下变向上)
def get_rotation_matrix():
    """返回 camera_init → ROS 标准坐标系的 3x3 旋转矩阵"""
    # q = (w=0.5, x=-0.5, y=-0.5, z=0.5)
    w, x, y, z = 0.5, -0.5, -0.5, 0.5
    R = np.array([
        [1 - 2*y*y - 2*z*z,  2*x*y - 2*w*z,      2*x*z + 2*w*y],
        [2*x*y + 2*w*z,      1 - 2*x*x - 2*z*z,  2*y*z - 2*w*x],
        [2*x*z - 2*w*y,      2*y*z + 2*w*x,      1 - 2*x*x - 2*y*y]
    ])
    return R


def convert_pcd(input_path: str, output_path: str):
    """读取PCD，旋转点云，保存"""
    print(f"读取: {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    points = np.asarray(pcd.points)
    print(f"  点数: {len(points)}")

    # 应用旋转
    R = get_rotation_matrix()
    points_rotated = (R @ points.T).T
    pcd_rotated = o3d.geometry.PointCloud()
    pcd_rotated.points = o3d.utility.Vector3dVector(points_rotated)

    # 保存
    o3d.io.write_point_cloud(output_path, pcd_rotated)
    print(f"输出: {output_path}")
    print(f"  点数: {len(pcd_rotated.points)}")
    print("转换完成!")
    print()
    print("验证方法:")
    print(f"  - 原始PCD的Z轴(向后) → 转换后应该指向 -X 方向")
    print(f"  - 在 RViz 中加载转换后的PCD，Z轴应朝上，点云应平放在地面上")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    input_path = sys.argv[1]
    if len(sys.argv) >= 3:
        output_path = sys.argv[2]
    else:
        base, ext = os.path.splitext(input_path)
        output_path = f"{base}_standard{ext}"

    if not os.path.exists(input_path):
        print(f"错误: 文件不存在: {input_path}")
        sys.exit(1)

    convert_pcd(input_path, output_path)


if __name__ == "__main__":
    main()
