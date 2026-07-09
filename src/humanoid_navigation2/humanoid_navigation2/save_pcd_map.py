#!/usr/bin/env python3
"""
save_pcd_map.py - 实时累积 fast_lio 的配准点云并保存为 PCD 文件
用法：
  python3 save_pcd_map.py /path/to/output.pcd

运行方式：
  建图过程中，另开终端运行此脚本。
  走完一圈后，按 Ctrl+C 结束，自动保存全局地图。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import sys
import signal
import struct

class PCDSaver(Node):
    def __init__(self, output_path):
        super().__init__('pcd_saver')
        self.output_path = output_path
        self.all_points = []
        self.frame_count = 0
        self.voxel_size = 0.05  # 体素降采样，避免文件太大
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/fast_lio/cloud_registered',
            self.cloud_callback,
            10
        )
        
        # 每隔 30 秒做一次全局降采样（防止内存爆炸）
        self.cleanup_timer = self.create_timer(30.0, self.periodic_cleanup)
        
        self.get_logger().info(f'PCD 保存器已启动，输出路径: {output_path}')
        self.get_logger().info('建图完成后按 Ctrl+C 保存 PCD 文件')
    
    def cloud_callback(self, msg):
        points = pc2.read_points_numpy(msg, field_names=("x", "y", "z", "intensity"))
        if len(points) == 0:
            return
        
        # 过滤无效点
        valid = np.isfinite(points).all(axis=1)
        points = points[valid]
        
        self.all_points.append(points)
        self.frame_count += 1
        
        if self.frame_count % 50 == 0:
            total = sum(len(p) for p in self.all_points)
            self.get_logger().info(
                f'已累积 {self.frame_count} 帧，共 {total:,} 个点'
            )
    
    def periodic_cleanup(self):
        """定期合并并降采样，防止内存溢出"""
        if len(self.all_points) < 10:
            return
        
        merged = np.vstack(self.all_points)
        downsampled = self.voxel_downsample(merged, self.voxel_size)
        self.all_points = [downsampled]
        self.get_logger().info(
            f'定期清理: {len(merged):,} → {len(downsampled):,} 个点'
        )
    
    def voxel_downsample(self, points, voxel_size):
        """体素降采样"""
        if len(points) == 0:
            return points
        grid = (points[:, :3] / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(grid, axis=0, return_index=True)
        return points[unique_idx]
    
    def save_pcd(self):
        """保存为 PCD 文件"""
        if not self.all_points:
            self.get_logger().error('没有收集到任何点云数据！')
            return
        
        # 合并所有点云
        all_pts = np.vstack(self.all_points)
        self.get_logger().info(f'合并后总点数: {len(all_pts):,}')
        
        # 最终降采样
        all_pts = self.voxel_downsample(all_pts, self.voxel_size)
        self.get_logger().info(f'降采样后总点数: {len(all_pts):,}')
        
        # 写 PCD 文件（ASCII 格式，兼容性最好）
        with open(self.output_path, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write(f"WIDTH {len(all_pts)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(all_pts)}\n")
            f.write("DATA ascii\n")
            
            for pt in all_pts:
                f.write(f"{pt[0]:.6f} {pt[1]:.6f} {pt[2]:.6f} {pt[3]:.2f}\n")
        
        self.get_logger().info(f'★★★ PCD 文件已保存: {self.output_path} ★★★')
        self.get_logger().info(f'文件大小: {os.path.getsize(self.output_path) / 1024 / 1024:.1f} MB')

import os

def main():
    if len(sys.argv) < 2:
        output_path = os.path.expanduser(
            '~/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd'
        )
        print(f"未指定输出路径，使用默认: {output_path}")
    else:
        output_path = sys.argv[1]
    
    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    rclpy.init()
    node = PCDSaver(output_path)
    
    def signal_handler(sig, frame):
        print("\n\n正在保存 PCD 文件，请勿关闭终端...")
        node.save_pcd()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except Exception:
        pass

if __name__ == '__main__':
    main()