#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 humanoid_expression_runtime 表情运行层。

本 launch 只启动 C++ 表情串口驱动 facial_driver_cpp。
上游节点通常是 app_gateway_node，它把 APP facial_control 命令转换成 /robot/facial_raw_cmd。
下游硬件是仿生头串口控制板。
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
            description='是否使用仿真时间；实机表情串口驱动通常为 false。',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_expression_runtime'),
                'config',
                'expression_runtime.yaml',
            ]),
            description='C++ 表情运行层参数文件。',
        ),
        Node(
            package='humanoid_expression_runtime',
            executable='facial_driver_cpp',
            name='facial_driver',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time},
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
