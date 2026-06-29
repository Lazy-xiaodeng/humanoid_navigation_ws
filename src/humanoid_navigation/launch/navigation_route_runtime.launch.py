#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路线任务运行层启动文件。

该层包含 navigation_state_manager，它依赖当前 Nav2 action server 和定位健康状态。
切换地图时它会跟随导航定位层一起停止和重启，避免旧 action client / 旧地图状态残留。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_humanoid_navigation2 = FindPackageShare('humanoid_navigation2')
    pkg_humanoid_route_runtime = FindPackageShare('humanoid_route_runtime')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_cpp_route_runtime = LaunchConfiguration('use_cpp_route_runtime')
    cpp_config_file = LaunchConfiguration('cpp_config_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时钟',
    )
    declare_use_cpp_route_runtime = DeclareLaunchArgument(
        'use_cpp_route_runtime',
        default_value='true',
        description='是否使用 C++ 路线任务运行层；默认 true，false 时回退 Python navigation_state_manager',
    )
    declare_cpp_config_file = DeclareLaunchArgument(
        'cpp_config_file',
        default_value=PathJoinSubstitution([
            pkg_humanoid_route_runtime,
            'config',
            'route_runtime.yaml',
        ]),
        description='C++ 路线任务运行层参数文件',
    )

    navigation_state_py_node = Node(
        package='humanoid_navigation',
        executable='navigation_state_manager',
        name='navigation_state_manager',
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_cpp_route_runtime, "' != 'true'"])),
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

    navigation_state_cpp_node = Node(
        package='humanoid_route_runtime',
        executable='navigation_state_manager_cpp',
        name='navigation_state_manager_cpp',
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_cpp_route_runtime, "' == 'true'"])),
        parameters=[
            cpp_config_file,
            {
                'use_sim_time': use_sim_time,
                # 导航栈正式接入时必须允许 C++ 运行层向 Nav2 下发 action goal；
                # 单包灰度测试仍可直接启动 humanoid_route_runtime/route_runtime.launch.py 使用默认 false。
                'route_task.nav2_execution_enable': True,
            },
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_cpp_route_runtime,
        declare_cpp_config_file,
        navigation_state_py_node,
        navigation_state_cpp_node,
    ])
