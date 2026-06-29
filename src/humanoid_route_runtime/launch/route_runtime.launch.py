#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 humanoid_route_runtime 路线任务运行层节点。

这个 launch 文件用于单独启动 C++ 路线任务运行层，也被导航层运行开关复用。
默认 `route_task.nav2_execution_enable=true`，和正式导航运行态保持一致，允许向 Nav2 下发 action goal。
如需离线协议验证，可显式传 nav2_execution_enable:=false。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('route_runtime_config_file')
    nav2_execution_enable = LaunchConfiguration('nav2_execution_enable')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间',
        ),
        DeclareLaunchArgument(
            'route_runtime_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_route_runtime'),
                'config',
                'route_runtime.yaml',
            ]),
            description='路线任务运行层节点参数文件',
        ),
        DeclareLaunchArgument(
            'nav2_execution_enable',
            default_value='true',
            description='是否允许路线任务运行层向 Nav2 下发 action goal；离线验证可设为 false',
        ),
        Node(
            package='humanoid_route_runtime',
            executable='navigation_state_manager_cpp',
            name='navigation_state_manager_cpp',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    'route_task.nav2_execution_enable': nav2_execution_enable,
                },
            ],
        ),
    ])
