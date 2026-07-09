#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 C++ WebSocket/机器人本体网关运行层。

文件用途：
1. 提供 APP WebSocket 和机器人本体 WebSocket 的统一 C++ 启动入口。
2. 同时启动 APP 网关、数据整合节点和机器人本体网关。
3. 上游：start_websocket.sh 或人工单独调试 WebSocket 链路。
4. 下游：humanoid_app_gateway_runtime 与 humanoid_robot_gateway_runtime。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    app_gateway_pkg = get_package_share_directory('humanoid_app_gateway_runtime')
    robot_gateway_pkg = get_package_share_directory('humanoid_robot_gateway_runtime')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(app_gateway_pkg, 'launch', 'app_gateway_runtime.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_gateway_pkg, 'launch', 'robot_gateway_runtime.launch.py')
            )
        ),
    ])
