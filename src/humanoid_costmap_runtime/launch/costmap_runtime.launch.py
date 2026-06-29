#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单独启动 Nav2 costmap 辅助运行层 C++ 节点。"""

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
                FindPackageShare('humanoid_costmap_runtime'),
                'config',
                'costmap_runtime.yaml',
            ]),
            description='Nav2 costmap 辅助运行层参数文件。',
        ),
        Node(
            package='humanoid_costmap_runtime',
            executable='periodic_clearing_3d_publisher',
            name='periodic_clearing_3d_publisher',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
    ])
