#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航控制层启动文件。

控制层只负责 APP/ROS 常驻通信、点位管理和地图切换状态机，切换地图时不能被停止。
真正依赖 2D/3D 地图的 Nav2、定位链路和 navigation_state_manager 放在导航层启动。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_cpp_control_runtime = LaunchConfiguration('use_cpp_control_runtime')
    cpp_control_config_file = LaunchConfiguration('cpp_control_config_file')
    pkg_humanoid_control_runtime = FindPackageShare('humanoid_control_runtime')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时钟',
    )
    declare_use_cpp_control_runtime = DeclareLaunchArgument(
        'use_cpp_control_runtime',
        default_value='true',
        description='是否使用 C++ 控制层运行包；默认 true，false 时回退 Python dynamic/map 节点',
    )
    declare_cpp_control_config_file = DeclareLaunchArgument(
        'cpp_control_config_file',
        default_value=PathJoinSubstitution([
            pkg_humanoid_control_runtime,
            'config',
            'control_runtime.yaml',
        ]),
        description='C++ 控制层运行包参数文件',
    )

    dynamic_waypoints_node = Node(
        package='humanoid_navigation',
        executable='dynamic_waypoints_manager',
        name='dynamic_waypoints_manager',
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_cpp_control_runtime, "' != 'true'"])),
        parameters=[{
            'use_sim_time': use_sim_time,
            # APP 设置点位后写入 Todesk 工作区，避免误写主工作区在线源码。
            'data_storage.enabled': True,
            'data_storage.file_path': '/home/ubuntu/software/Todesk/Files/humanoid_ws/data/dynamic_waypoints.json',
            # 多地图一期主存储：每张地图独立一个点位 JSON，例如 data/waypoints/hall.json。
            'data_storage.waypoints_dir': '/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints',
            'data_storage.default_map_id': 'hall',
            'navigation.position_tolerance': 0.15,
            'navigation.orientation_tolerance': 0.2,
            'navigation.default_frame_id': 'map',
        }],
    )

    map_context_node = Node(
        package='humanoid_navigation',
        executable='map_context_manager',
        name='map_context_manager',
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_cpp_control_runtime, "' != 'true'"])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_registry_path': '/home/ubuntu/software/Todesk/Files/humanoid_ws/data/maps/map_registry.json',
            'default_map_id': 'hall',
            # switch_map 只允许重启导航定位层，不能停止 websocket/点位/地图上下文控制层。
            'map_switch_script': '/home/ubuntu/software/Todesk/Files/humanoid_ws/switch_navigation_map.sh',
            # 真实切图包含停止旧定位、启动新定位、等待 prior-map 稳定，超时时间需要覆盖进程重启耗时。
            'switch_localization_timeout_sec': 45.0,
            'switch_localization_stable_frames': 2,
        }],
    )

    dynamic_waypoints_cpp_node = Node(
        package='humanoid_control_runtime',
        executable='dynamic_waypoints_manager_cpp',
        name='dynamic_waypoints_manager_cpp',
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_cpp_control_runtime, "' == 'true'"])),
        parameters=[
            cpp_control_config_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    map_context_cpp_node = Node(
        package='humanoid_control_runtime',
        executable='map_context_manager_cpp',
        name='map_context_manager_cpp',
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_cpp_control_runtime, "' == 'true'"])),
        parameters=[
            cpp_control_config_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_cpp_control_runtime,
        declare_cpp_control_config_file,
        dynamic_waypoints_node,
        map_context_node,
        dynamic_waypoints_cpp_node,
        map_context_cpp_node,
    ])
