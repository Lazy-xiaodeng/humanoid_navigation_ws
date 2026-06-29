#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启动 WebSocket 客户端和数据整合节点。

默认使用 C++ 数据整合和 C++ 机器人本体网关。
需要回退旧 Python 链路时，通过 use_cpp_data_integration=false / use_cpp_robot_gateway=false 显式关闭。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件的路径
    config_dir = os.path.join(get_package_share_directory('humanoid_websocket'), 'config')
    pkg_app_gateway_runtime = get_package_share_directory('humanoid_app_gateway_runtime')
    pkg_robot_gateway_runtime = get_package_share_directory('humanoid_robot_gateway_runtime')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_cpp_data_integration = LaunchConfiguration('use_cpp_data_integration')
    use_cpp_robot_gateway = LaunchConfiguration('use_cpp_robot_gateway')
    cpp_robot_ws_enable = LaunchConfiguration('cpp_robot_ws_enable')
    cpp_robot_walk_velocity_send_enable = LaunchConfiguration('cpp_robot_walk_velocity_send_enable')
    cpp_robot_motion_execution_enable = LaunchConfiguration('cpp_robot_motion_execution_enable')
    cpp_robot_motion_allow_enter_menu = LaunchConfiguration('cpp_robot_motion_allow_enter_menu')
    cpp_robot_motion_allow_return_walk = LaunchConfiguration('cpp_robot_motion_allow_return_walk')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_cpp_data_integration',
            default_value='true',
            description='是否使用 C++ 数据整合节点；默认 true，false 使用 Python data_integration_node_recoverable',
        ),
        DeclareLaunchArgument(
            'use_cpp_robot_gateway',
            default_value='true',
            description='是否使用 C++ 机器人本体网关；默认 true，false 使用 Python websocket_client_node',
        ),
        DeclareLaunchArgument(
            'cpp_robot_ws_enable',
            default_value='true',
            description='use_cpp_robot_gateway=true 时是否连接真实机器人本体 WebSocket；空载验证可设为 false',
        ),
        DeclareLaunchArgument(
            'cpp_robot_walk_velocity_send_enable',
            default_value='false',
            description='C++ 机器人本体网关是否允许真实发送 /cmd_vel；默认 false',
        ),
        DeclareLaunchArgument(
            'cpp_robot_motion_execution_enable',
            default_value='false',
            description='C++ 机器人本体网关是否允许真实执行 APP 动作；默认 false',
        ),
        DeclareLaunchArgument(
            'cpp_robot_motion_allow_enter_menu',
            default_value='false',
            description='C++ 动作执行前是否允许自动切换 Menu；默认 false',
        ),
        DeclareLaunchArgument(
            'cpp_robot_motion_allow_return_walk',
            default_value='false',
            description='C++ 动作结束后是否允许自动切回 Walk；默认 false',
        ),
        # WebSocket客户端节点（连接机器人本体）
        Node(
            package='humanoid_websocket',
            executable='websocket_client_node',
            name='websocket_client',
            output='screen',
            parameters=[os.path.join(config_dir, 'websocket_config.yaml')],
            condition=UnlessCondition(use_cpp_robot_gateway),
        ),
        # 数据整合节点（存储和整合数据）
        Node(
            package='humanoid_websocket',
            executable='data_integration_node_recoverable',
            name='data_integration_node',
            output='screen',
            condition=UnlessCondition(use_cpp_data_integration),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_app_gateway_runtime,
                'launch',
                'app_gateway_runtime.launch.py',
            )),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'websocket_server_enable': 'false',
                'websocket_host': '0.0.0.0',
                'websocket_port': '8765',
                'data_integration_enable': 'true',
            }.items(),
            condition=IfCondition(use_cpp_data_integration),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_robot_gateway_runtime,
                'launch',
                'robot_gateway_runtime.launch.py',
            )),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_ws_enable': cpp_robot_ws_enable,
                'walk_velocity_send_enable': cpp_robot_walk_velocity_send_enable,
                'motion_execution_enable': cpp_robot_motion_execution_enable,
                'motion_allow_enter_menu': cpp_robot_motion_allow_enter_menu,
                'motion_allow_return_walk': cpp_robot_motion_allow_return_walk,
            }.items(),
            condition=IfCondition(use_cpp_robot_gateway),
        ),
    ])
