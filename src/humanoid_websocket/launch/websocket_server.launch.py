#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启动 WebSocket / 数据整合 / 机器人本体网关链路。

默认使用 C++ 链路：
- humanoid_app_gateway_runtime：APP WebSocket + data_integration。
- humanoid_robot_gateway_runtime：机器人本体 WebSocket client。

回退开关：
- use_cpp_app_gateway=false 时，回退 Python websocket_server_node + data_integration_node_recoverable。
- use_cpp_robot_gateway=false 时，回退 Python websocket_client_node。
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
    config_dir = os.path.join(get_package_share_directory('humanoid_websocket'), 'config')
    pkg_app_gateway_runtime = get_package_share_directory('humanoid_app_gateway_runtime')
    pkg_robot_gateway_runtime = get_package_share_directory('humanoid_robot_gateway_runtime')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_cpp_app_gateway = LaunchConfiguration('use_cpp_app_gateway')
    use_cpp_robot_gateway = LaunchConfiguration('use_cpp_robot_gateway')
    cpp_app_websocket_server_enable = LaunchConfiguration('cpp_app_websocket_server_enable')
    cpp_app_websocket_host = LaunchConfiguration('cpp_app_websocket_host')
    cpp_app_websocket_port = LaunchConfiguration('cpp_app_websocket_port')
    cpp_data_integration_enable = LaunchConfiguration('cpp_data_integration_enable')
    cpp_robot_ws_enable = LaunchConfiguration('cpp_robot_ws_enable')
    cpp_robot_walk_velocity_send_enable = LaunchConfiguration('cpp_robot_walk_velocity_send_enable')
    cpp_robot_motion_execution_enable = LaunchConfiguration('cpp_robot_motion_execution_enable')
    cpp_robot_motion_allow_enter_menu = LaunchConfiguration('cpp_robot_motion_allow_enter_menu')
    cpp_robot_motion_allow_return_walk = LaunchConfiguration('cpp_robot_motion_allow_return_walk')
    cpp_robot_gesture_sync_enable = LaunchConfiguration('cpp_robot_gesture_sync_enable')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_cpp_app_gateway',
            default_value='true',
            description='是否使用 C++ APP 网关 + C++ 数据整合；默认 true，false 使用 Python 版',
        ),
        DeclareLaunchArgument(
            'cpp_app_websocket_server_enable',
            default_value='true',
            description='use_cpp_app_gateway=true 时是否打开 C++ APP WebSocket 服务；空载验证可设为 false',
        ),
        DeclareLaunchArgument(
            'cpp_app_websocket_host',
            default_value='0.0.0.0',
            description='C++ APP WebSocket 监听地址；本机 smoke 可设为 127.0.0.1',
        ),
        DeclareLaunchArgument(
            'cpp_app_websocket_port',
            default_value='8765',
            description='C++ APP WebSocket 监听端口；本机 smoke 可设为临时端口避免冲突',
        ),
        DeclareLaunchArgument(
            'cpp_data_integration_enable',
            default_value='true',
            description='use_cpp_app_gateway=true 时是否打开 C++ 数据整合订阅发布；空载验证通常保持 true',
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
        DeclareLaunchArgument(
            'cpp_robot_gesture_sync_enable',
            default_value='false',
            description='C++ 机器人本体网关是否连接后同步动作库；默认 false',
        ),
        # WebSocket服务器节点（连接APP）
        Node(
            package='humanoid_websocket',
            executable='websocket_server_node',
            name='websocket_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'websocket_config.yaml')],
            condition=UnlessCondition(use_cpp_app_gateway),
        ),
        # 数据整合节点（存储和整合数据）
        Node(
            package='humanoid_websocket',
            executable='data_integration_node_recoverable',
            name='data_integration_node',
            output='screen',
            condition=UnlessCondition(use_cpp_app_gateway),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_app_gateway_runtime,
                'launch',
                'app_gateway_runtime.launch.py',
            )),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'websocket_server_enable': cpp_app_websocket_server_enable,
                'websocket_host': cpp_app_websocket_host,
                'websocket_port': cpp_app_websocket_port,
                'data_integration_enable': cpp_data_integration_enable,
            }.items(),
            condition=IfCondition(use_cpp_app_gateway),
        ),
        # WebSocket客户端节点（连接机器人本体）
        Node(
            package='humanoid_websocket',
            executable='websocket_client_node',
            name='websocket_client',
            output='screen',
            parameters=[os.path.join(config_dir, 'robot_config.yaml')],
            condition=UnlessCondition(use_cpp_robot_gateway),
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
                'gesture_sync_enable': cpp_robot_gesture_sync_enable,
            }.items(),
            condition=IfCondition(use_cpp_robot_gateway),
        ),
    ])
