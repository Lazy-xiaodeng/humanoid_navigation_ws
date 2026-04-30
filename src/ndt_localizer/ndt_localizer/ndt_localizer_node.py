#!/usr/bin/env python3
"""
NDT Localizer 节点
基于 Open3D 的 NDT 算法实现 3D 点云与 PCD 地图的匹配定位
持续发布 map -> base_footprint 的 TF 变换
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import open3d as o3d
from tf2_ros import TransformBroadcaster
import struct
from scipy.spatial.transform import Rotation

# 导入 PCL 点云转换工具
try:
    import sensor_msgs_py.point_cloud2 as pc2
except ImportError:
    # 如果 sensor_msgs_py 不可用，使用自定义转换
    pc2 = None


class NDTLocalizer(Node):
    def __init__(self):
        super().__init__('ndt_localizer')
        
        # ===== 参数声明 =====
        self.declare_parameter('pcd_map_path', '/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd')
        self.declare_parameter('input_cloud_topic', '/fast_lio/cloud_registered')
        self.declare_parameter('output_pose_topic', '/ndt_pose')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('ndt_resolution', 1.0)
        self.declare_parameter('ndt_iterations', 100)
        self.declare_parameter('ndt_epsilon', 0.01)
        self.declare_parameter('min_points_for_matching', 100)
        self.declare_parameter('downsample_voxel_size', 0.2)
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('camera_init_frame_id', 'camera_init')
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_z', 0.0)
        self.declare_parameter('initial_roll', 0.0)
        self.declare_parameter('initial_pitch', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('use_initial_pose', False)
        
        # ===== 获取参数 =====
        self.pcd_map_path = self.get_parameter('pcd_map_path').value
        self.input_cloud_topic = self.get_parameter('input_cloud_topic').value
        self.output_pose_topic = self.get_parameter('output_pose_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.ndt_resolution = self.get_parameter('ndt_resolution').value
        self.ndt_iterations = self.get_parameter('ndt_iterations').value
        self.ndt_epsilon = self.get_parameter('ndt_epsilon').value
        self.min_points = self.get_parameter('min_points_for_matching').value
        self.downsample_size = self.get_parameter('downsample_voxel_size').value
        self.map_frame = self.get_parameter('map_frame_id').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.camera_init_frame = self.get_parameter('camera_init_frame_id').value
        
        # ===== 初始化状态 =====
        self.is_initialized = False
        self.last_pose = None  # 上一次成功的位姿
        self.match_count = 0
        self.fail_count = 0
        
        # ===== 加载 PCD 地图 =====
        self.get_logger().info(f'Loading PCD map from: {self.pcd_map_path}')
        try:
            self.pcd_map = o3d.io.read_point_cloud(self.pcd_map_path)
            self.get_logger().info(f'PCD map loaded with {len(self.pcd_map.points)} points')
            
            # 体素下采样地图（减少计算量）
            self.pcd_map_downsampled = self.pcd_map.voxel_down_sample(voxel_size=0.5)
            self.get_logger().info(f'PCD map downsampled to {len(self.pcd_map_downsampled.points)} points')
            
            # 构建 NDT 模型（参考点云）
            self.get_logger().info('Building NDT model...')
            # 注意：这里使用简化的 ICP 替代完整的 NDT
            # 因为 Open3D 的 NDT 实现需要自定义
            self.pcd_map_np = np.asarray(self.pcd_map_downsampled.points)
            self.get_logger().info('NDT model built successfully')
            
            self.is_initialized = True
        except Exception as e:
            self.get_logger().error(f'Failed to load PCD map: {str(e)}')
            return
        
        # ===== TF 广播器 =====
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ===== 订阅器 =====
        self.cloud_subscription = self.create_subscription(
            PointCloud2,
            self.input_cloud_topic,
            self.cloud_callback,
            10
        )
        self.get_logger().info(f'Subscribed to: {self.input_cloud_topic}')
        
        # ===== 发布器 =====
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.output_pose_topic,
            10
        )
        self.get_logger().info(f'Publishing pose to: {self.output_pose_topic}')
        
        # 初始位姿
        if self.get_parameter('use_initial_pose').value:
            self.current_transform = self._get_initial_transform()
            self.get_logger().info(f'Initial pose set: {self.current_transform}')
        else:
            # 等待第一次匹配
            self.current_transform = np.eye(4)
            self.get_logger().info('Waiting for initial localization...')
        
        # ===== 定时器：定期发布 TF =====
        self.tf_timer = self.create_timer(1.0 / self.publish_rate, self.publish_tf_timer_callback)
        
        self.get_logger().info('NDT Localizer initialized successfully')
    
    def _get_initial_transform(self):
        """获取初始位姿变换矩阵"""
        x = self.get_parameter('initial_pose_x').value
        y = self.get_parameter('initial_pose_y').value
        z = self.get_parameter('initial_pose_z').value
        roll = self.get_parameter('initial_roll').value
        pitch = self.get_parameter('initial_pitch').value
        yaw = self.get_parameter('initial_yaw').value
        
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        R = Rotation.from_euler('xyz', [roll, pitch, yaw])
        T[:3, :3] = R.as_matrix()
        
        return T
    
    def pointcloud2_to_o3d(self, cloud_msg):
        """将 PointCloud2 转换为 Open3D 点云"""
        points_list = []
        
        try:
            if pc2:
                # 使用 sensor_msgs_py 转换
                for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
                    points_list.append([point[0], point[1], point[2]])
            else:
                # 自定义转换
                points_list = self._parse_pointcloud2(cloud_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse point cloud: {str(e)}')
            return None
        
        if len(points_list) < self.min_points:
            self.get_logger().warn(f'Too few points: {len(points_list)} < {self.min_points}')
            return None
        
        # 创建 Open3D 点云
        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(np.array(points_list))
        
        # 降采样
        if self.downsample_size > 0:
            cloud_o3d = cloud_o3d.voxel_down_sample(voxel_size=self.downsample_size)
        
        return cloud_o3d
    
    def _parse_pointcloud2(self, cloud_msg):
        """自定义 PointCloud2 解析"""
        points = []
        fields = {field.name: (field.offset, field.datatype) for field in cloud_msg.fields}
        
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            self.get_logger().error('PointCloud2 missing x, y, or z fields')
            return points
        
        x_offset, x_type = fields['x']
        y_offset, y_type = fields['y']
        z_offset, z_type = fields['z']
        
        struct_format = ''
        for offset, datatype in [fields['x'], fields['y'], fields['z']]:
            if datatype == 7:  # FLOAT32
                struct_format += 'f'
            elif datatype == 8:  # FLOAT64
                struct_format += 'd'
        
        data = cloud_msg.data
        point_step = cloud_msg.point_step
        
        for i in range(cloud_msg.height):
            for j in range(cloud_msg.width):
                offset = i * cloud_msg.row_step + j * point_step
                try:
                    point_data = data[offset + x_offset:offset + x_offset + struct.calcsize(struct_format)]
                    if len(point_data) == struct.calcsize(struct_format):
                        x, y, z = struct.unpack(struct_format, point_data)
                        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                            points.append([x, y, z])
                except:
                    continue
        
        return points
    
    def ndt_matching(self, source_cloud):
        """
        NDT 匹配算法
        使用 Open3D 的 ICP 作为简化实现
        TODO: 后续可替换为真正的 NDT 实现
        """
        if not self.is_initialized:
            return None
        
        try:
            # 使用当前的变换作为初始估计
            initial_transform = self.current_transform.copy()
            
            # 将源点云从 body 坐标系变换到 camera_init 坐标系
            # 这里假设 source_cloud 已经在 camera_init 坐标系下（Fast-LIO 输出）
            
            # 使用 Point-to-Plane ICP 作为 NDT 的近似
            # 这比 Point-to-Point 更稳定
            source_down = source_cloud
            target_down = self.pcd_map_downsampled
            
            # 计算法线（用于 Point-to-Plane ICP）
            target_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.ndt_resolution * 2, max_nn=30
                )
            )
            
            # ICP 配准
            result = o3d.pipelines.registration.registration_icp(
                source_down,
                target_down,
                max_correspondence_distance=self.ndt_resolution * 2,
                init=initial_transform,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.ndt_iterations,
                    relative_fitness=self.ndt_epsilon
                )
            )
            
            # 检查匹配质量
            fitness = result.fitness
            inlier_rmse = result.inlier_rmse
            
            self.get_logger().debug(f'NDT matching - fitness: {fitness:.4f}, RMSE: {inlier_rmse:.4f}')
            
            # 阈值判断
            if fitness > 0.3 and inlier_rmse < 1.0:
                self.match_count += 1
                return result.transformation, fitness, inlier_rmse
            else:
                self.fail_count += 1
                if self.fail_count > 10:
                    self.get_logger().warn(f'Matching failed {self.fail_count} times, fitness={fitness:.4f}, rmse={inlier_rmse:.4f}')
                return None, fitness, inlier_rmse
                
        except Exception as e:
            self.get_logger().error(f'NDT matching error: {str(e)}')
            return None, 0.0, 0.0
    
    def cloud_callback(self, msg):
        """点云回调函数"""
        if not self.is_initialized:
            return
        
        # 转换点云
        source_cloud = self.pointcloud2_to_o3d(msg)
        if source_cloud is None:
            return
        
        # NDT 匹配
        transform, fitness, rmse = self.ndt_matching(source_cloud)
        
        if transform is not None:
            self.current_transform = transform
            self.last_pose = transform
            self.fail_count = 0  # 重置失败计数
            
            # 发布位姿
            self.publish_pose(transform, fitness, rmse, msg.header.stamp)
            
            if self.match_count % 50 == 0:
                self.get_logger().info(
                    f'Localization OK - fitness: {fitness:.4f}, RMSE: {rmse:.4f}, '
                    f'pose: [{transform[0,3]:.2f}, {transform[1,3]:.2f}, {transform[2,3]:.2f}]'
                )
    
    def publish_pose(self, transform, fitness, rmse, stamp):
        """发布位姿消息"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        
        # 从变换矩阵提取位姿
        pose_msg.pose.pose.position.x = float(transform[0, 3])
        pose_msg.pose.pose.position.y = float(transform[1, 3])
        pose_msg.pose.pose.position.z = float(transform[2, 3])
        
        # 旋转矩阵转四元数
        R = Rotation.from_matrix(transform[:3, :3])
        quat = R.as_quat()  # [x, y, z, w]
        pose_msg.pose.pose.orientation.x = float(quat[0])
        pose_msg.pose.pose.orientation.y = float(quat[1])
        pose_msg.pose.pose.orientation.z = float(quat[2])
        pose_msg.pose.pose.orientation.w = float(quat[3])
        
        # 协方差矩阵（根据匹配质量设置）
        cov_scale = max(0.01, 1.0 - fitness) * 0.1
        pose_msg.pose.covariance = [
            cov_scale, 0, 0, 0, 0, 0,
            0, cov_scale, 0, 0, 0, 0,
            0, 0, cov_scale, 0, 0, 0,
            0, 0, 0, cov_scale, 0, 0,
            0, 0, 0, 0, cov_scale, 0,
            0, 0, 0, 0, 0, cov_scale
        ]
        
        self.pose_publisher.publish(pose_msg)
    
    def publish_tf_timer_callback(self):
        """定期发布 TF 变换"""
        if self.current_transform is None:
            return
        
        now = self.get_clock().now().to_msg()
        
        # ★★★ 关键步骤：坐标系转换 (歪 -> 正) ★★★
        # NDT 计算出的 current_transform 是 map -> camera_init (歪坐标系)
        # 我们需要发布 map -> odom (正坐标系)
        # 关系：map -> odom = (map -> camera_init) * (camera_init -> odom)
        
        # 定义 camera_init -> odom 的变换 (将歪坐标系转正)
        # 根据 launch 文件: odom -> camera_init 四元数为 [-0.5, -0.5, 0.5, 0.5]
        # 其逆矩阵 (camera_init -> odom) 四元数为 [0.5, 0.5, -0.5, 0.5]
        R_cam_to_odom = Rotation.from_quat([0.5, 0.5, -0.5, 0.5])
        T_cam_to_odom = np.eye(4)
        T_cam_to_odom[:3, :3] = R_cam_to_odom.as_matrix()
        # 平移为 0，因为原点重合
        
        # 计算 map -> odom 的变换
        transform_to_publish = self.current_transform @ T_cam_to_odom
        
        # 发布 map -> odom 的 TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        
        # 从变换矩阵提取平移和旋转
        tf_msg.transform.translation.x = float(transform_to_publish[0, 3])
        tf_msg.transform.translation.y = float(transform_to_publish[1, 3])
        tf_msg.transform.translation.z = float(transform_to_publish[2, 3])
        
        R = Rotation.from_matrix(transform_to_publish[:3, :3])
        quat = R.as_quat()
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NDTLocalizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('NDT Localizer shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
