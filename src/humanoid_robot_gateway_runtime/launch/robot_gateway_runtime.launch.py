#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动 humanoid_robot_gateway_runtime 机器人本体网关运行节点。

当前 launch 用于启动 C++ 机器人本体网关。
默认连接机器人本体 WebSocket，用于读取 robot_status_raw 并向 APP 回传机器人身份/状态。
速度发送、动作执行、动作前后模式切换和动作库同步默认打开，便于正式导航联调。

上游：
- 开发者手动 ros2 launch，或 bringup 运行开关。

下游：
- robot_gateway_node：连接机器人本体 WebSocket，发布机器人状态并接收速度/动作命令；默认只读取状态，不发送速度/动作。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('robot_gateway_config_file')
    robot_ws_enable = LaunchConfiguration('robot_ws_enable')
    walk_velocity_send_enable = LaunchConfiguration('walk_velocity_send_enable')
    motion_execution_enable = LaunchConfiguration('motion_execution_enable')
    motion_allow_enter_menu = LaunchConfiguration('motion_allow_enter_menu')
    motion_allow_return_walk = LaunchConfiguration('motion_allow_return_walk')
    gesture_sync_enable = LaunchConfiguration('gesture_sync_enable')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间',
        ),
        DeclareLaunchArgument(
            'robot_gateway_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_robot_gateway_runtime'),
                'config',
                'robot_gateway.yaml',
            ]),
            description='机器人本体网关节点参数文件',
        ),
        DeclareLaunchArgument(
            'robot_ws_enable',
            default_value='true',
            description='是否连接真实机器人本体 WebSocket；默认 true，离线验证可设为 false',
        ),
        DeclareLaunchArgument(
            'walk_velocity_send_enable',
            default_value='true',
            description='是否允许把 /cmd_vel 真实发送给机器人；默认 true',
        ),
        DeclareLaunchArgument(
            'motion_execution_enable',
            default_value='true',
            description='是否允许真实执行 APP 上半身动作；默认 true',
        ),
        DeclareLaunchArgument(
            'motion_allow_enter_menu',
            default_value='true',
            description='执行动作前是否允许自动切换 Menu；默认 true',
        ),
        DeclareLaunchArgument(
            'motion_allow_return_walk',
            default_value='true',
            description='动作结束后是否允许自动切回 Walk；默认 true',
        ),
        DeclareLaunchArgument(
            'gesture_sync_enable',
            default_value='true',
            description='是否连接成功后拉取机器人动作库并写入 gestures.yaml；默认 true',
        ),
        Node(
            package='humanoid_robot_gateway_runtime',
            executable='robot_gateway_node',
            name='robot_gateway_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    'robot_ws_enable': robot_ws_enable,
                    'walk_velocity_send_enable': walk_velocity_send_enable,
                    'motion_execution_enable': motion_execution_enable,
                    'motion_allow_enter_menu': motion_allow_enter_menu,
                    'motion_allow_return_walk': motion_allow_return_walk,
                    'gesture_sync_enable': gesture_sync_enable,
                },
            ],
        ),
    ])
