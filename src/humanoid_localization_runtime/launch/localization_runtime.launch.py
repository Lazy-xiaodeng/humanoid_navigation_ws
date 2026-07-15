#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单独启动定位运行层 C++ 辅助节点。

该 launch 用于调试 humanoid_localization_runtime，不替代正式导航启动链路。
正式导航中仍建议由 humanoid_navigation2/launch/navigation2_robosense_lidar.launch.py 统一编排。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='是否使用仿真时间。'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_localization_runtime'),
                'config',
                'localization_runtime.yaml',
            ]),
            description='定位运行层参考参数文件。',
        ),
        Node(
            package='humanoid_localization_runtime',
            executable='dynamic_odom_ground_publisher',
            name='dynamic_map_ground_publisher',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='humanoid_localization_runtime',
            executable='dynamic_odom_ground_publisher',
            name='dynamic_odom_ground_publisher',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='humanoid_localization_runtime',
            executable='prior_map_odom_bridge_cpp',
            name='prior_map_odom_bridge',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='humanoid_localization_runtime',
            executable='rviz_initialpose_adapter',
            name='rviz_initialpose_adapter',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='humanoid_localization_runtime',
            executable='global_relocalization_coordinator',
            name='global_relocalization_coordinator',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='humanoid_localization_runtime',
            executable='robot_realpose_publisher',
            name='robot_realpose_publisher',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
    ])
