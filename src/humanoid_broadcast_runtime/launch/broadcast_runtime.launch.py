#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 humanoid_broadcast_runtime 播报运行层。

本 launch 只启动 C++ 播报服务 broadcast_service_node_cpp。
上游节点通常是 APP 网关或调试脚本，它们调用 /xiaorui_broadcast/play、/set_volume、/health。
下游是本机音频系统，包括 PipeWire/Pulse sink、ALSA 设备或外部 edge-tts 播放脚本。
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
            description='是否使用仿真时间；实机音频播报服务通常为 false。',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_broadcast_runtime'),
                'config',
                'broadcast_runtime.yaml',
            ]),
            description='C++ 播报运行层参数文件。',
        ),
        Node(
            package='humanoid_broadcast_runtime',
            executable='broadcast_service_node_cpp',
            name='xiaorui_broadcast_service',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time},
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
