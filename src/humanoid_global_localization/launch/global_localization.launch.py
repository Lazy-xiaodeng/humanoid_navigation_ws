"""
humanoid_global_localization 启动文件

启动全局定位节点及其生命周期管理器。
该节点发布 map->odom TF 变换和 /pcl_pose 话题，
与现有的 TF 树和 Nav2 导航栈兼容。

使用方式：
  ros2 launch humanoid_global_localization global_localization.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('humanoid_global_localization')
    params_file = os.path.join(pkg_dir, 'config', 'global_localization.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 全局定位节点
    global_localization_node = Node(
        package='humanoid_global_localization',
        executable='global_localization_node',
        name='global_localization',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            # 如果点云话题不是 /fast_lio/cloud_registered，在此修改
        ]
    )

    # 生命周期管理器 (自动 configure + activate)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_global_loc',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['global_localization']
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # 延迟启动：等 Fast-LIO 点云稳定
        TimerAction(period=5.0, actions=[global_localization_node]),
        TimerAction(period=7.0, actions=[lifecycle_manager]),
    ])
