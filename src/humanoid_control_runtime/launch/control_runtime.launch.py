#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动当前单地图版本的 C++ 点位管理节点。

本 launch 只启动 dynamic_waypoints_manager_cpp，不包含 Todesk 多地图版的 map_context_manager。
在完成对比验证前，robot_real.launch.py 仍默认使用 Python 版入口。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('control_runtime_config_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='是否使用仿真时间'),
        DeclareLaunchArgument(
            'control_runtime_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_control_runtime'),
                'config',
                'control_runtime.yaml',
            ]),
            description='C++ 点位管理节点参数文件',
        ),
        Node(
            package='humanoid_control_runtime',
            executable='dynamic_waypoints_manager_cpp',
            name='dynamic_waypoints_manager_cpp',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
    ])
