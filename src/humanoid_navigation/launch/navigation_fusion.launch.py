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
    
    # 2. 导航状态管理器节点 (新 prior-map 定位版本，旧 fusion 可单独回退)
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
            'auto_pause_on_localization_recovery': False,
            'localization_stop_hold_sec': 2.0,
            'localization_resume_settle_sec': 1.0,
            'localization_auto_resume_require_recovery_done': True,
            'localization_resume_stable_frames': 3,
            'localization_health_status_topic': '/localization/prior_map_odom_bridge_status',
            'localization_health_timeout_sec': 3.0,
            # bridge 已经接受过 map->odom 后，即使当前候选被拒绝，也允许靠 last good TF + odom 启动/推进。
            'localization_allow_start_with_last_good_tf': True,
            # 0 表示不限制 last good TF 年龄；如果现场需要更保守，可改成 10~30 秒。
            'localization_last_good_tf_max_age_sec': 0.0,
            'localization_recovery_status_topic': '/localization/recovery_status',
            'localization_recovery_request_topic': '/localization/recovery_requests',
            'request_localization_recovery_on_nav_failure': False,
            'request_navigation_context_recovery_on_localization_failure': False,
            'localization_recovery_request_cooldown_sec': 20.0,
            'localization_recovery_prior_radius_m': 10.0,
            'localization_context_recovery_request_cooldown_sec': 4.0,
            'localization_context_prior_radius_m': 5.0,
            'localization_context_prior_max_previous_age_sec': 300.0,
            'localization_context_prior_min_segment_length_m': 0.2,
            'localization_resume_reverse_enabled': True,
            'localization_resume_reverse_max_distance_m': 2.0,
            'localization_resume_reverse_rear_angle_deg': 70.0,
            # 路线任务参数：首个 task 的近距离判定阈值。
            # 如果机器人启动时已经在首个 task 附近，仍会进入 task 完成/播报流程，不额外生成虚拟起点。
            'route_task.first_task_reached_tolerance_m': 0.4,
            # 路线任务参数：through feedback 缺少 number_of_poses_remaining 时，
            # 用当前机器人 pose 到 transit 的水平距离判断 waypoint_passed。
            'route_task.transit_passed_tolerance_m': 0.5,
            # 路线任务参数：允许“上一执行点 -> transit”投影越过终点时补判 transit 已经过，
            # 用来兜底机器人擦边经过 transit、但没有正好落入阈值圆的情况。
            'route_task.transit_projection_passed_enabled': True,
            # 路线任务参数：预留的 Nav2 feedback 超时阈值，后续如需监控 through feedback 中断可直接使用。
            'route_task.nav2_feedback_timeout_sec': 3.0,
            # 路线任务参数：jump 或停止时取消旧 goal 的等待阈值，避免旧回调污染新段。
            'route_task.goal_cancel_timeout_sec': 2.0,
            # 路线任务参数：首版只支持 jump 打断播报，false 会返回明确错误。
            'route_task.default_interrupt_broadcast': True,
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
