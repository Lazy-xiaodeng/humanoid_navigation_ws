#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 humanoid_global_relocalization_runtime 全局重定位运行层节点。

该 launch 用于单独启动全局重定位运行层。节点会按参数决定是否实时搜索；
默认只输出恢复候选和恢复状态，不直接发布 TF，也不直接改写定位桥。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_global_relocalization_runtime'),
                'config',
                'global_relocalization_runtime.yaml',
            ]),
            description='全局重定位运行层 YAML 配置文件。',
        ),
        Node(
            package='humanoid_global_relocalization_runtime',
            executable='global_relocalization_node',
            name='global_relocalization_node',
            output='screen',
            parameters=[{'config_file': config_file}],
        ),
    ])
