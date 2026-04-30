#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图坐标查看器启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_navigation2',
            executable='map_coordinate_viewer',
            name='map_coordinate_viewer',
            output='screen'
        )
    ])
