#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 humanoid_control_runtime 控制层运行节点。

这个 launch 文件用于单独启动控制层运行包，也被控制层运行开关复用。
其中两个节点都属于常驻控制层：一个维护点位与路线命令，一个维护地图上下文与切图状态。
默认单独启动本包节点；是否纳入正式链路由上层 bringup 的运行开关决定。
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_control_runtime'),
                'config',
                'control_runtime.yaml',
            ]),
            description='控制层 C++ 节点参数文件',
        ),
        Node(
            package='humanoid_control_runtime',
            executable='dynamic_waypoints_manager_cpp',
            name='dynamic_waypoints_manager_cpp',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='humanoid_control_runtime',
            executable='map_context_manager_cpp',
            name='map_context_manager_cpp',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
    ])
