#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 humanoid_global_relocalization_runtime 全局重定位运行层节点。

文件作用：
  作为系统集成入口启动全局重定位运行层。

该 launch 用于系统集成。默认使用 relocalization_runtime.yaml，节点只输出恢复候选
和恢复状态，不直接发布 TF，也不直接改写定位桥。

正式链路中，该节点必须与 global_relocalization_coordinator 配合：
算法节点只负责搜索和候选合同，协调器才负责 attempt/map/stamp 关联、
RO 精修交接和 bridge 授权。
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
                'relocalization_runtime.yaml',
            ]),
            description='全局重定位运行层 YAML 配置文件。默认使用正式运行模板。',
        ),
        # 纯算法运行层：订阅恢复请求和点云，发布候选位姿、
        # 候选 map->odom 与结构化状态。它没有正式 TF 写入权。
        Node(
            package='humanoid_global_relocalization_runtime',
            executable='global_relocalization_node',
            name='global_relocalization_node',
            output='screen',
            parameters=[{'config_file': config_file}],
        ),
    ])
