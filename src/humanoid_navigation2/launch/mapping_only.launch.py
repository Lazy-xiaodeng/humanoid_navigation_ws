"""
建图专用 Launch 文件（首次建图用）

系统架构：
1. 雷达驱动 → Fast-LIO → 输出点云
2. Fast-LIO同时保存PCD地图（3D点云地图）
3. 点云转2D激光 → slam_toolbox → 生成2D栅格地图

用途：
- 首次进入新环境时使用此文件建图
- 建图完成后保存hall.pcd和hall.yaml
- 之后使用 navigation_only.launch.py 进行导航
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    pkg_fast_lio = get_package_share_directory('fast_lio_robosense')
    fast_lio_params = os.path.join(pkg_fast_lio, 'config', 'robosenseAiry.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    pkg_nav2 = get_package_share_directory('humanoid_navigation2')
    default_save_path = os.path.join(pkg_nav2, 'pcd', 'hall.pcd')
    slam_mapping_file = os.path.join(pkg_nav2, 'config', 'slam_toolbox_mapping.yaml')
    
    pkg_share = get_package_share_directory('humanoid_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'humanoid_robot.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # =========================================================================
    # 1. 机器人描述与TF
    # =========================================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # =========================================================================
    # 2. 传感器与Fast-LIO
    # =========================================================================
    
    # 雷达驱动
    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Fast-LIO节点（开启PCD保存）
    fast_lio_node = Node(
        package='fast_lio_robosense',
        executable='fastlio_mapping',
        name='fast_lio_node',
        output='log',
        parameters=[
            fast_lio_params,
            {
                'use_sim_time': use_sim_time,
                'pcd_save_en': True,  # 开启保存PCD
                'map_file_path': default_save_path,  # 保存路径
                'pcd_save.interval': -1  # 退出时保存
            }
        ],
        remappings=[
            ('/CloudPoints', '/airy_points'),
            ('/Imu', '/airy_imu'),
            ('/Odometry', '/odom'),
            ('/cloud_registered', '/fast_lio/cloud_registered')
        ]
    )

    # =========================================================================
    # 3. TF桥接
    # =========================================================================
    tf_bridge_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_camera_init',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '-0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5',
            '--frame-id', 'odom', '--child-frame-id', 'camera_init'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    tf_bridge_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_body_to_base_footprint',
        arguments=[
            '--x', '0.004', '--y', '1.215', '--z', '0.072',
            '--qx', '0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
            '--frame-id', 'body', '--child-frame-id', 'base_footprint'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # =========================================================================
    # 4. 3D点云转2D激光
    # =========================================================================
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/fast_lio/cloud_registered'),
            ('scan', '/scan')
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'base_footprint',
            'transform_tolerance': 1.0,
            'min_height': 0.05,
            'max_height': 1.65,
            'range_min': 0.6,
            'range_max': 20.0,
            'scan_time': 0.1,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'queue_size': 50
        }]
    )

    # =========================================================================
    # 5. slam_toolbox建图
    # =========================================================================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_mapping_file,
            {
                'use_sim_time': use_sim_time,
                'mode': 'mapping'  # ★ 建图模式
            }
        ],
        output='screen'
    )

    # slam_toolbox生命周期管理
    slam_lifecycle_manager = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'bond_timeout': 0.0,
                    'node_names': ['slam_toolbox']
                }]
            )
        ]
    )

    # =========================================================================
    # 组装Launch描述
    # =========================================================================
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # 1. 机器人描述
        robot_state_publisher_node,
        joint_state_publisher_node,

        # 2. 传感器与Fast-LIO
        rslidar_node,
        fast_lio_node,

        # 3. TF桥接
        tf_bridge_odom,
        tf_bridge_base,

        # 4. 数据转换
        pc_to_scan_node,

        # 5. 建图节点
        slam_toolbox_node,
        slam_lifecycle_manager,

    ])
