#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启动WebSocket服务器节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('humanoid_websocket'), 'config')
    
    return LaunchDescription([
        # WebSocket服务器节点（连接APP）
        Node(
            package='humanoid_websocket',
            executable='websocket_server_node',
            name='websocket_server',
            output='screen',
            parameters=[os.path.join(config_dir, 'websocket_config.yaml')]
        ),
        # 数据整合节点（存储和整合数据）
        Node(
            package='humanoid_websocket',
            executable='data_integration_node',
            name='data_integration_node',
            output='screen'
        ),
        # WebSocket客户端节点（连接机器人本体）
        Node(
            package='humanoid_websocket',
            executable='websocket_client_node',
            name='websocket_client',
            output='screen',
            parameters=[os.path.join(config_dir, 'robot_config.yaml')]
        ),
        #消息桥接节点（转换IMU数据等）
        Node(
            package='humanoid_websocket',
            executable='message_bridge_node',
            name='message_bridge',
            output='screen',
        )
    ])