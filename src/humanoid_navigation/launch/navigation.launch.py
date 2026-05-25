#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航系统启动文件 - 纯净版（仅包含路点管理与状态管理）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取功能包共享路径
    pkg_humanoid_navigation = FindPackageShare('humanoid_navigation')
    pkg_humanoid_navigation2 = FindPackageShare('humanoid_navigation2')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时钟'
    )

    # 1. 动态路点管理器节点 (负责与 APP 的点位增删改查)
    dynamic_waypoints_node = Node(
        package='humanoid_navigation',
        executable='dynamic_waypoints_manager',  
        name='dynamic_waypoints_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoints_config_file': PathJoinSubstitution([
                pkg_humanoid_navigation, 'config', 'waypoints.yaml'
            ]),
            'data_storage.enabled': True,
            'data_storage.file_path': '/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json',
            'navigation.position_tolerance': 0.15,
            'navigation.orientation_tolerance': 0.2,
            'navigation.default_frame_id': 'map'
        }]
    )
    
    # 2. 导航状态管理器节点 (负责监听 Nav2 进度并生成 JSON 状态)
    navigation_state_node = Node(
        package='humanoid_navigation',
        executable='navigation_state_manager_recoverable', 
        name='navigation_state_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'update_frequency': 10.0,
            'position_tolerance': 0.15,
            'publish_tf': True,
            'default_frame_id': 'map',
            'auto_pause_on_localization_recovery': True,
            'localization_stop_hold_sec': 2.0,
            'localization_resume_settle_sec': 1.0,
            'localization_recovery_status_topic': '/localization/recovery_status',
            'localization_recovery_request_topic': '/localization/recovery_requests',
            'request_localization_recovery_on_nav_failure': True,
            'request_navigation_context_recovery_on_localization_failure': True,
            'localization_recovery_request_cooldown_sec': 20.0,
            'localization_recovery_prior_radius_m': 10.0,
            'localization_context_recovery_request_cooldown_sec': 4.0,
            'localization_context_prior_radius_m': 5.0,
            'localization_context_prior_max_previous_age_sec': 300.0,
            'localization_context_prior_min_segment_length_m': 0.2,
            'localization_resume_reverse_enabled': True,
            'localization_resume_reverse_max_distance_m': 2.0,
            'localization_resume_reverse_rear_angle_deg': 70.0,
            'reverse_navigation_bt_xml': PathJoinSubstitution([
                pkg_humanoid_navigation2, 'behavior_tree', 'navigate_reverse_xy_then_yaw.xml'
            ])
        }]
    )
    
    # 创建启动描述并添加动作
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(dynamic_waypoints_node)
    ld.add_action(navigation_state_node)
    
    return ld
