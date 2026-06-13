#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路线任务运行层启动文件。

该层包含 navigation_state_manager，它依赖当前 Nav2 action server 和定位健康状态。
切换地图时它会跟随导航定位层一起停止和重启，避免旧 action client / 旧地图状态残留。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_humanoid_navigation2 = FindPackageShare('humanoid_navigation2')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时钟',
    )

    navigation_state_node = Node(
        package='humanoid_navigation',
        executable='navigation_state_manager',
        name='navigation_state_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'update_frequency': 10.0,
            'position_tolerance': 0.15,
            'publish_tf': True,
            'default_frame_id': 'map',
            'localization_resume_stable_frames': 3,
            'localization_health_status_topic': '/localization/prior_map_odom_bridge_status',
            'localization_health_timeout_sec': 3.0,
            'localization_allow_start_with_last_good_tf': True,
            'localization_last_good_tf_max_age_sec': 0.0,
            'localization_resume_reverse_rear_angle_deg': 70.0,
            'route_task.first_task_reached_tolerance_m': 0.4,
            'route_task.transit_passed_tolerance_m': 0.5,
            'route_task.transit_projection_passed_enabled': True,
            'route_task.nav2_feedback_timeout_sec': 3.0,
            'route_task.goal_cancel_timeout_sec': 2.0,
            'route_task.default_interrupt_broadcast': True,
            'reverse_navigation_bt_xml': PathJoinSubstitution([
                pkg_humanoid_navigation2, 'behavior_tree', 'navigate_reverse_xy_then_yaw.xml'
            ]),
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        navigation_state_node,
    ])
