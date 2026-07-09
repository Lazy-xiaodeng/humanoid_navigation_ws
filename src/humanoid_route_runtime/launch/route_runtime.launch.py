#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动单地图导航状态管理 C++ 节点。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pkg_route_runtime = FindPackageShare("humanoid_route_runtime")
    pkg_navigation2 = FindPackageShare("humanoid_navigation2")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="是否使用仿真时钟"),
        Node(
            package="humanoid_route_runtime",
            executable="navigation_state_manager_cpp",
            name="navigation_state_manager",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_route_runtime, "config", "route_runtime.yaml"]),
                {
                    "use_sim_time": use_sim_time,
                    "reverse_navigation_bt_xml": PathJoinSubstitution([
                        pkg_navigation2,
                        "behavior_tree",
                        "navigate_reverse_xy_then_yaw.xml",
                    ]),
                },
            ],
        ),
    ])
