#!/usr/bin/env python3
"""
从 Fast-LIO 建图过程中提取关键帧，构建 Scan Context 数据库

使用方法:
ros2 run humanoid_relocalization build_sc_database.py \
    --bag /path/to/mapping_bag \
    --output /path/to/sc_database.bin \
    --cloud-topic /fast_lio/cloud_registered \
    --odom-topic /fast_lio/odom \
    --interval 2.0
"""

import argparse
import numpy as np
import struct
from pathlib import Path
from dataclasses import dataclass
from typing import List, Tuple, Optional
import yaml

# ROS2 bag 读取
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2

@dataclass
class ScanContextConfig:
    num_sectors: int = 60
    num_rings: int = 20
    max_range: float = 80.0
    lidar_height: float = 1.5

@dataclass
class KeyFrame:
    id: int
    pose: np.ndarray  # 4x4 变换矩阵
    sc_descriptor: np.ndarray  # (num_rings, num_sectors)
    ring_key: np.ndarray  # (num_rings,)
    cloud: np.ndarray  # (N, 4) - x, y, z, intensity

def get_storage_id(bag_path: str) -> str:
    """自动检测 rosbag 存储格式"""
    bag_dir = Path(bag_path)
    metadata_file = bag_dir / 'metadata.yaml'
    
    if metadata_file.exists():
        with open(metadata_file, 'r') as f:
            metadata = yaml.safe_load(f)
            storage_id = metadata.get('rosbag2_bagfile_information', {}).get('storage_identifier', 'sqlite3')
            print(f"Detected storage format: {storage_id}")
            return storage_id
    
    # 根据文件扩展名判断
    if list(bag_dir.glob('*.mcap')):
        return 'mcap'
    elif list(bag_dir.glob('*.db3')):
        return 'sqlite3'
    
    return 'sqlite3'

def compute_scan_context(cloud: np.ndarray, config: ScanContextConfig) -> np.ndarray:
    """计算 Scan Context 描述子"""
    sc = np.zeros((config.num_rings, config.num_sectors), dtype=np.float32)
    
    if len(cloud) == 0:
        return sc
    
    ring_step = config.max_range / config.num_rings
    sector_step = 2.0 * np.pi / config.num_sectors
    
    # 计算水平距离和角度
    x, y, z = cloud[:, 0], cloud[:, 1], cloud[:, 2]
    ranges = np.sqrt(x**2 + y**2)
    angles = np.arctan2(y, x) + np.pi  # [0, 2*pi]
    
    # 过滤有效点
    valid = (ranges > 0.1) & (ranges < config.max_range)
    ranges = ranges[valid]
    angles = angles[valid]
    heights = z[valid] + config.lidar_height
    
    if len(ranges) == 0:
        return sc
    
    # 计算索引
    ring_indices = np.clip((ranges / ring_step).astype(int), 0, config.num_rings - 1)
    sector_indices = np.clip((angles / sector_step).astype(int), 0, config.num_sectors - 1)
    
    # 填充描述子 (取最大高度)
    for ri, si, h in zip(ring_indices, sector_indices, heights):
        if h > sc[ri, si]:
            sc[ri, si] = h
    
    return sc

def compute_ring_key(sc: np.ndarray) -> np.ndarray:
    """计算 Ring Key"""
    return sc.mean(axis=1).astype(np.float32)

def pose_to_matrix(odom_msg) -> np.ndarray:
    """从 Odometry 消息提取 4x4 变换矩阵"""
    pos = odom_msg.pose.pose.position
    ori = odom_msg.pose.pose.orientation
    
    # 四元数转旋转矩阵
    x, y, z, w = ori.x, ori.y, ori.z, ori.w
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ], dtype=np.float32)
    
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3] = [pos.x, pos.y, pos.z]
    
    return T

def read_pointcloud(msg: PointCloud2) -> Optional[np.ndarray]:
    """读取点云数据，兼容有无 intensity 字段"""
    try:
        # 读取结构化数组
        gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        pts = np.array(list(gen))
        
        if len(pts) > 0:
            # ★ 关键修复：结构化数组 → 普通 float32 二维数组 ★
            result = np.column_stack([
                pts['x'].astype(np.float32),
                pts['y'].astype(np.float32),
                pts['z'].astype(np.float32),
                pts['intensity'].astype(np.float32),
            ])
            return result
    except Exception:
        pass
    
    try:
        # 没有 intensity，只读取 xyz
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.array(list(gen))
        
        if len(pts) > 0:
            result = np.zeros((len(pts), 4), dtype=np.float32)
            result[:, 0] = pts['x'].astype(np.float32)
            result[:, 1] = pts['y'].astype(np.float32)
            result[:, 2] = pts['z'].astype(np.float32)
            return result
    except Exception as e:
        print(f"Error reading point cloud: {e}")
    
    return None

def read_bag(bag_path: str, cloud_topic: str, odom_topic: str, 
             keyframe_interval: float = 1.0) -> Tuple[List, List]:
    """读取 rosbag，提取点云和里程计"""
    
    reader = SequentialReader()
    storage_id = get_storage_id(bag_path)
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions('', '')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        print("Trying alternative storage format...")
        alt_storage = 'mcap' if storage_id == 'sqlite3' else 'sqlite3'
        storage_options = StorageOptions(uri=bag_path, storage_id=alt_storage)
        reader.open(storage_options, converter_options)
    
    clouds = []
    odoms = []
    last_kf_time = None
    
    # 获取话题类型映射
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    print(f"Reading bag: {bag_path}")
    print(f"Available topics: {list(type_map.keys())}")
    print(f"Cloud topic: {cloud_topic}")
    print(f"Odom topic: {odom_topic}")
    
    # 检查话题是否存在
    if cloud_topic not in type_map:
        print(f"WARNING: Cloud topic '{cloud_topic}' not found in bag!")
        print(f"Available topics: {list(type_map.keys())}")
    if odom_topic not in type_map:
        print(f"WARNING: Odom topic '{odom_topic}' not found in bag!")
    
    msg_count = 0
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        time_sec = timestamp / 1e9
        msg_count += 1
        
        if topic == odom_topic:
            try:
                msg = deserialize_message(data, Odometry)
                odoms.append((time_sec, msg))
            except Exception as e:
                print(f"Error deserializing odom: {e}")
            
        elif topic == cloud_topic:
            # 关键帧间隔检查
            if last_kf_time is not None and (time_sec - last_kf_time) < keyframe_interval:
                continue
            
            try:
                msg = deserialize_message(data, PointCloud2)
                points = read_pointcloud(msg)
                
                if points is not None and len(points) > 100:
                    clouds.append((time_sec, points))
                    last_kf_time = time_sec
                    print(f"  Keyframe {len(clouds)}: t={time_sec:.2f}s, {len(points)} points")
            except Exception as e:
                print(f"Error processing cloud: {e}")
    
    print(f"Processed {msg_count} messages")
    print(f"Extracted {len(clouds)} keyframes, {len(odoms)} odom messages")
    return clouds, odoms

def find_closest_odom(cloud_time: float, odoms: List) -> np.ndarray:
    """找到最接近的里程计位姿"""
    if len(odoms) == 0:
        print("WARNING: No odometry data, using identity pose")
        return np.eye(4, dtype=np.float32)
    
    min_dt = float('inf')
    closest_odom = None
    
    for odom_time, odom_msg in odoms:
        dt = abs(cloud_time - odom_time)
        if dt < min_dt:
            min_dt = dt
            closest_odom = odom_msg
    
    if closest_odom is None:
        return np.eye(4, dtype=np.float32)
    
    return pose_to_matrix(closest_odom)

def save_database(keyframes: List[KeyFrame], output_path: str, config: ScanContextConfig):
    """保存数据库为二进制文件 - 严格匹配 C++ ScanContext::loadDatabase() 格式"""
    
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'wb') as f:
        # 1. 配置头
        f.write(struct.pack('i', config.num_sectors))
        f.write(struct.pack('i', config.num_rings))
        f.write(struct.pack('d', config.max_range))
        f.write(struct.pack('d', config.lidar_height))
        
        # 2. 关键帧数量 (size_t = uint64 on 64-bit)
        f.write(struct.pack('Q', len(keyframes)))
        
        # 3. 逐帧写入
        for kf in keyframes:
            # ID
            f.write(struct.pack('i', kf.id))
            
            # Pose 4x4 (Eigen 列优先)
            pose_f32 = np.asfortranarray(kf.pose.astype(np.float32))
            f.write(pose_f32.tobytes())
            
            # SC descriptor: rows, cols, data (Eigen 列优先)
            rows, cols = kf.sc_descriptor.shape
            f.write(struct.pack('i', rows))
            f.write(struct.pack('i', cols))
            sc_f32 = np.asfortranarray(kf.sc_descriptor.astype(np.float32))
            f.write(sc_f32.tobytes())
            
            # Ring key: size, data (向量，列优先和行优先一样)
            f.write(struct.pack('i', len(kf.ring_key)))
            f.write(kf.ring_key.astype(np.float32).tobytes())
            
            # ★ 不写点云数据 ★
    
    file_size = Path(output_path).stat().st_size
    header_size = 4 + 4 + 8 + 8 + 8
    per_kf = 4 + 64 + 4 + 4 + 4*config.num_rings*config.num_sectors + 4 + 4*config.num_rings
    expected = header_size + len(keyframes) * per_kf
    
    print(f"\n✅ Saved database to {output_path}")
    print(f"   File size: {file_size} bytes, Expected: {expected} bytes")
    print(f"   Match: {'✅' if file_size == expected else '❌ MISMATCH!'}")
    print(f"   Keyframes: {len(keyframes)}, Rings: {config.num_rings}, Sectors: {config.num_sectors}")

def build_database(bag_path: str, output_path: str, 
                   cloud_topic: str = "/fast_lio/cloud_registered",
                   odom_topic: str = "/fast_lio/odom",
                   keyframe_interval: float = 2.0):
    """构建 Scan Context 数据库"""
    
    config = ScanContextConfig()
    
    # 读取 bag
    clouds, odoms = read_bag(bag_path, cloud_topic, odom_topic, keyframe_interval)
    
    if len(clouds) == 0:
        print("ERROR: No keyframes extracted!")
        print("Please check:")
        print(f"  1. Bag path exists: {bag_path}")
        print(f"  2. Cloud topic is correct: {cloud_topic}")
        print(f"  3. Bag contains point cloud data")
        return
    
    # 构建关键帧
    keyframes = []
    for i, (cloud_time, cloud_points) in enumerate(clouds):
        print(f"Processing keyframe {i+1}/{len(clouds)}...", end='\r')
        
        # 获取位姿
        pose = find_closest_odom(cloud_time, odoms)
        
        # 计算描述子
        sc = compute_scan_context(cloud_points, config)
        ring_key = compute_ring_key(sc)
        
        # 降采样点云
        if len(cloud_points) > 5000:
            indices = np.random.choice(len(cloud_points), 5000, replace=False)
            cloud_points = cloud_points[indices]
        
        kf = KeyFrame(
            id=i,
            pose=pose,
            sc_descriptor=sc,
            ring_key=ring_key,
            cloud=cloud_points
        )
        keyframes.append(kf)
    
    print()  # 换行
    
    # 保存
    save_database(keyframes, output_path, config)

def main():
    parser = argparse.ArgumentParser(description='Build Scan Context database from rosbag')
    parser.add_argument('--bag', required=True, help='Path to rosbag directory')
    parser.add_argument('--output', required=True, help='Output database path (.bin)')
    parser.add_argument('--cloud-topic', default='/fast_lio/cloud_registered', 
                        help='Point cloud topic')
    parser.add_argument('--odom-topic', default='/fast_lio/odom', 
                        help='Odometry topic')
    parser.add_argument('--interval', type=float, default=2.0, 
                        help='Keyframe interval in seconds')
    
    args = parser.parse_args()
    
    # 验证输入
    if not Path(args.bag).exists():
        print(f"ERROR: Bag path does not exist: {args.bag}")
        return
    
    build_database(
        bag_path=args.bag,
        output_path=args.output,
        cloud_topic=args.cloud_topic,
        odom_topic=args.odom_topic,
        keyframe_interval=args.interval
    )

if __name__ == '__main__':
    main()