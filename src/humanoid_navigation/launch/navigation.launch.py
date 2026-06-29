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
            'data_storage.file_path': '/home/ubuntu/software/Todesk/Files/humanoid_ws/data/dynamic_waypoints.json',
            # 多地图一期主存储目录：每张地图一个点位 JSON，例如 data/waypoints/hall.json。
            'data_storage.waypoints_dir': '/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints',
            'data_storage.default_map_id': 'hall',
            'navigation.position_tolerance': 0.15,
            'navigation.orientation_tolerance': 0.2,
            'navigation.default_frame_id': 'map'
        }]
    )

    # 1.5 地图上下文管理器（多地图一期：只提供地图查询，不做自动切图）
    map_context_node = Node(
        package='humanoid_navigation',
        executable='map_context_manager',
        name='map_context_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_registry_path': '/home/ubuntu/software/Todesk/Files/humanoid_ws/data/maps/map_registry.json',
            'default_map_id': 'hall',
        }]
    )
    
    # 2. 导航状态管理器节点
    # 说明：路线任务首版验收依赖 NavigateThroughPoses、jump、broadcast 等新逻辑，
    # 这些能力集中在 navigation_state_manager.py 中；旧 recoverable 入口保留为可回退脚本，
    # 但默认 launch 必须启动新入口，否则 APP 下发 start_route_task 时运行时收不到新协议处理。
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
            # 性能优化：完整导航状态 JSON 降到 1Hz；事件类状态仍会即时发布，不影响按钮/任务事件。
            'status_publish_rate': 1.0,
            # 性能优化：正常导航时 local costmap 只缓存，只有障碍等待恢复时才分析前方窗口。
            'obstacle_costmap_analyze_only_when_waiting': True,
            # 性能优化：障碍恢复窗口只扫描前方小区域的外接 cell，不再遍历整张 costmap。
            'obstacle_costmap_window_bounded_scan': True,
            'waypoint_speed_min_mps': 0.15,
            'waypoint_speed_max_mps': 1.0,
            'default_navigation_speed_mps': 0.5,
            'default_reverse_navigation_speed_mps': 0.5,
            'controller_server_name': '/controller_server',
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
    ld.add_action(map_context_node)
    ld.add_action(navigation_state_node)
    
    return ld
