#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 humanoid_app_gateway_runtime APP 网关运行层运行节点。

当前 launch 用于启动 C++ APP 网关和数据整合链路。
默认打开 APP WebSocket 服务和数据整合订阅发布，和一键启动的正式运行态保持一致。
如需离线验证或避免抢占 8765 端口，可显式传 websocket_server_enable:=false 或 data_integration_enable:=false。

上游：
- 开发者手动 ros2 launch，或 bringup 运行开关。

下游：
- app_gateway_node：面向 APP WebSocket、APP 协议转发和业务命令路由。
- data_integration_node：面向系统数据聚合、请求响应和订阅推送输出。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    app_gateway_config = LaunchConfiguration('app_gateway_config')
    data_integration_config = LaunchConfiguration('data_integration_config')
    websocket_server_enable = LaunchConfiguration('websocket_server_enable')
    websocket_host = LaunchConfiguration('websocket_host')
    websocket_port = LaunchConfiguration('websocket_port')
    data_integration_enable = LaunchConfiguration('data_integration_enable')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间',
        ),
        DeclareLaunchArgument(
            'app_gateway_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_app_gateway_runtime'),
                'config',
                'app_gateway.yaml',
            ]),
            description='APP 网关节点参数文件',
        ),
        DeclareLaunchArgument(
            'data_integration_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_app_gateway_runtime'),
                'config',
                'data_integration.yaml',
            ]),
            description='数据整合节点参数文件',
        ),
        DeclareLaunchArgument(
            'websocket_server_enable',
            default_value='true',
            description='是否启用真实 APP WebSocket 服务；默认 true，离线验证可设为 false',
        ),
        DeclareLaunchArgument(
            'websocket_host',
            default_value='0.0.0.0',
            description='APP WebSocket 监听地址；独立测试可设为 127.0.0.1',
        ),
        DeclareLaunchArgument(
            'websocket_port',
            default_value='8765',
            description='APP WebSocket 监听端口；独立测试可指定临时端口避免冲突',
        ),
        DeclareLaunchArgument(
            'data_integration_enable',
            default_value='true',
            description='是否启用 C++ 数据整合订阅和发布；默认 true，离线验证可设为 false',
        ),
        Node(
            package='humanoid_app_gateway_runtime',
            executable='app_gateway_node',
            name='app_gateway_node',
            output='screen',
            parameters=[
                app_gateway_config,
                {
                    'use_sim_time': use_sim_time,
                    'websocket_server_enable': websocket_server_enable,
                    'websocket_host': websocket_host,
                    'websocket_port': websocket_port,
                },
            ],
        ),
        Node(
            package='humanoid_app_gateway_runtime',
            executable='data_integration_node',
            name='data_integration_node',
            output='screen',
            parameters=[
                data_integration_config,
                {
                    'use_sim_time': use_sim_time,
                    'data_integration_enable': data_integration_enable,
                },
            ],
        ),
    ])
