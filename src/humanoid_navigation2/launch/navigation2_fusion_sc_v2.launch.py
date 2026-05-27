"""
完整导航栈 Launch 文件 v2 — NDT + Odom Fusion + ScanContext + HDL 双引擎全局重定位
Phase 1: TF 抑制 + DEGRADED 停车 + LOST TF 保持
Phase 2: 推算位姿先验 + SC 先验搜索
Phase 3: HDL fallback + NDT 旧帧点云清理 + Waypoint 保留强化

与 v1 (navigation2_fusion_sc.launch.py) 的关键差异:
  1. NDT 新增 fusion_status 超时参数 (fusion_status_timeout_sec, allow_ndt_tf_when_fusion_timeout)
  2. fusion 锁定期缩短 (min/max_degraded_lock_sec: 10)
  3. HDL global localization: enable_runtime_auto_recovery=false (仅 SC 显式 fallback 触发)
  4. max_odom_displacement_m: 30 → 5 (缩短接管距离)
  5. SC bridge: prior 解析 + HDL fallback 触发

系统架构：
  TF树: map → odom → camera_init → body → base_footprint → base_link → 传感器
  TF ownership (Plan B): NDT 在 HEALTHY 发 map->odom, fusion 在 DEGRADED+/LOST 发冻结 map->odom

使用方式：
  ros2 launch humanoid_navigation2 navigation2_fusion_sc_v2.launch.py use_sim_time:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    GroupAction,
    TimerAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # =========================================================================
    # FastDDS 共享内存优化配置
    # =========================================================================
    enable_fastdds_shm = LaunchConfiguration('enable_fastdds_shm', default='true')

    fastdds_config_file = PathJoinSubstitution([
        os.path.expanduser('~'),
        '.config',
        'fastdds_shm.xml'
    ])

    fastdds_env_setup = [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_config_file),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
    ]

    # ========== 路径配置 ==========
    pkg_nav2 = get_package_share_directory('humanoid_navigation2')
    pkg_global_loc = get_package_share_directory('humanoid_global_localization')
    pkg_lidar_loc = get_package_share_directory('lidar_localization_ros2')
    pkg_scancontext_loc = get_package_share_directory('humanoid_scancontext_global_localization')
    pkg_hdl_global_loc = get_package_share_directory('hdl_global_localization')
    pkg_hdl_loc = get_package_share_directory('hdl_localization')

    default_nav2_params_file = os.path.join(pkg_nav2, 'config', 'nav2_params_xy_yaw.yaml')
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=default_nav2_params_file)
    global_localization_params_file = os.path.join(pkg_global_loc, 'config', 'global_localization.yaml')
    localization_params_file = os.path.join(pkg_lidar_loc, 'param', 'localization.yaml')
    scancontext_params_file = os.path.join(
        pkg_scancontext_loc, 'config', 'scancontext_global_localization.yaml')
    scancontext_database_file = os.path.join(pkg_nav2, 'maps', 'hall_sc_fastlio_registered.bin')
    scancontext_pcd_map_file = os.path.join(pkg_nav2, 'pcd', 'hall.pcd')
    hdl_globalmap_pcd = os.path.join(pkg_nav2, 'pcd', 'hall_localization_grounded.pcd')
    map_yaml_file = os.path.join(pkg_nav2, 'maps', 'hall.yaml')
    default_bt_xml_file = os.path.join(pkg_nav2, 'behavior_tree', 'navigate_xy_then_yaw.xml')
    bt_xml_file = LaunchConfiguration('bt_xml_file', default=default_bt_xml_file)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    def nav2_python_node(executable, node_name, extra_parameters=None):
        parameters = [{'use_sim_time': use_sim_time}]
        if extra_parameters:
            parameters.append(extra_parameters)
        return Node(
            package='humanoid_navigation2',
            executable=executable,
            name=node_name,
            output='screen',
            parameters=parameters,
        )

    enable_elevation_map = LaunchConfiguration('enable_elevation_map', default='false')
    enable_terrain_analysis = LaunchConfiguration('enable_terrain_analysis', default='false')
    enable_periodic_clearing = LaunchConfiguration('enable_periodic_clearing', default='true')

    # =========================================================================
    # 第一部分：传感器与Fast-LIO节点
    # =========================================================================
    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    fast_lio_node = Node(
        package='fast_lio_robosense',
        executable='fastlio_mapping',
        name='fast_lio_node',
        output='log',
        parameters=[
            os.path.join(get_package_share_directory('fast_lio_robosense'), 'config', 'robosenseAiry.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/CloudPoints', '/airy_points'),
            ('/Imu', '/airy_imu'),
            ('/Odometry', '/odom'),
            ('/cloud_registered', '/fast_lio/cloud_registered')
        ]
    )

    # TF桥接
    tf_bridge_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_camera_init',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '-0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5',
            '--frame-id', 'odom', '--child-frame-id', 'camera_init'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    tf_map_to_ground = Node(
        package='humanoid_navigation2',
        executable='dynamic_odom_ground_publisher',
        name='dynamic_map_ground_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': 'map',
            'base_frame': 'base_footprint',
            'child_frame': 'map_ground',
            'publish_rate': 30.0,
            'z_offset': 0.0,
        }],
        output='screen'
    )

    tf_odom_to_ground = Node(
        package='humanoid_navigation2',
        executable='dynamic_odom_ground_publisher',
        name='dynamic_odom_ground_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': 'odom',
            'base_frame': 'base_footprint',
            'child_frame': 'odom_ground',
            'publish_rate': 30.0,
            'z_offset': 0.0,
        }],
        output='screen'
    )

    tf_bridge_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_body_to_base_footprint',
        arguments=[
            '--x', '0.004', '--y', '1.215', '--z', '0.072',
            '--qx', '0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
            '--frame-id', 'body', '--child-frame-id', 'base_footprint'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    tf_bridge_clearing_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_footprint_to_clearing_lidar',
        arguments=[
            '--x', '0', '--y', '0', '--z', '1.215',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_footprint', '--child-frame-id', 'clearing_lidar'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # =========================================================================
    # 第二部分：感知层
    # =========================================================================
    point_cloud_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('humanoid_point_cloud_filter'),
                'launch',
                'point_cloud_filter.launch.py'
            )
        )
    )

    elevation_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('elevation_mapping_opencl'),
                'launch',
                'elevation_mapping.launch.py'
            )
        )
    )

    terrain_analyzer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('humanoid_terrain_analyzer'),
                'launch',
                'terrain_analyzer.launch.py'
            )
        )
    )

    # =========================================================================
    # 第三部分：定位层
    # =========================================================================
    map_server_node = TimerAction(
        period=1.0,
        actions=[Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_yaml_file}, {'frame_id': 'map_ground'}])]
    )

    map_server_lifecycle = TimerAction(
        period=6.0,
        actions=[Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
            parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': ['map_server']}])]
    )

    robot_realpose_publisher = nav2_python_node(
        'robot_realpose_publisher',
        'robot_realpose_publisher',
        {'global_frame': 'map_ground'}
    )

    # ScanContext 全局重定位节点
    scancontext_global_localizer_node = Node(
        package='humanoid_scancontext_global_localization',
        executable='scancontext_global_localizer_node',
        name='scancontext_global_localizer',
        output='screen',
        parameters=[
            scancontext_params_file,
            {
                'use_sim_time': use_sim_time,
                'database_path': scancontext_database_file,
                'pcd_map_path': scancontext_pcd_map_file,
                'cloud_topic': '/fast_lio/cloud_registered',
                'odom_topic': '/odom',
                'cloud_frame_mode': 'registered',
                'map_frame': 'camera_init',
                'publish_initialpose': False,
                'query_on_cloud': False,
                'query_period_sec': 0.0,
                'sc_distance_threshold': 0.25,
                'enable_odom_consistency_gate': True,
                'max_odom_consistency_distance': 1.0,
                'enable_candidate_confidence_gate': True,
                'min_sc_distance_gap': 0.03,
                'max_ambiguous_candidate_distance': 2.0,
                'max_refined_odom_consistency_distance': 1.5,
                'enable_gicp_refinement': True,
                'gicp_fitness_threshold': 0.6,
                'global_recovery_top_k': 15,
                'global_recovery_ring_candidates': 80,
                'global_recovery_service': '/scancontext_global_localization/trigger_global',
                'global_recovery_sc_distance_threshold': 0.25,
                'global_recovery_gicp_fitness_threshold': 0.09,
                'global_recovery_min_gicp_fitness_gap': 0.006,
                'global_recovery_same_solution_xy_tolerance': 0.5,
                'global_recovery_same_solution_yaw_tolerance': 0.35,
                'global_recovery_required_consistent_results': 4,
                'global_recovery_consistency_window': 8,
                'global_recovery_consistency_xy_tolerance': 0.5,
                'global_recovery_consistency_yaw_tolerance': 0.25,
                'global_recovery_observation_max_age_sec': 20.0,
                'global_recovery_enable_candidate_confidence_gate': True,
                'global_recovery_max_refined_odom_consistency_distance': 0.0,
            }
        ],
    )

    scancontext_to_initialpose_node = nav2_python_node(
        'scancontext_to_initialpose',
        'scancontext_to_initialpose',
        {
            'map_frame': 'map',
            'best_pose_topic': '/scancontext_global_localization/best_pose',
            'localization_pose_topic': '/pcl_pose',
            'ndt_status_topic': '/localization/ndt_status',
            'initialpose_topic': '/initialpose',
            'recovery_status_topic': '/localization/recovery_status',
            'recovery_request_topic': '/localization/recovery_requests',
            'trigger_service': '/scancontext_global_localization/trigger',
            'global_trigger_service': '/scancontext_global_localization/trigger_global',
            'global_recovery_after_attempts': 3,
            'startup_trigger_period_sec': 2.0,
            'runtime_trigger_period_sec': 8.0,
            'startup_duration_sec': 30.0,
            'monitor_localization': True,
            'localization_pose_stale_sec': 2.5,
            'recovery_settle_sec': 6.0,
            'require_ndt_stable_status_for_recovery': True,
            'ndt_recovery_required_stable_status_count': 3,
            'ndt_rejected_recovery_count': 2,
            'external_recovery_request_duration_sec': 12.0,
            'enable_runtime_auto_recovery': False,
            'publish_repetitions': 8,
            'publish_period_sec': 0.25,
            'min_publish_interval_sec': 12.0,
            'xy_covariance': 0.25,
            'z_covariance': 0.04,
            'yaw_covariance': 0.10,
            # ★ Phase 3: HDL fallback topic
            'hdl_fallback_request_topic': '/localization/hdl_fallback_request',
        }
    )

    # ★★★ Phase 3: HDL fallback 桥接节点 ★★★
    # HDL 作为双引擎的 fallback 引擎: SC bridge 全局搜索耗尽后
    # 通过 /localization/hdl_fallback_request 显式触发 HDL 全图重定位。
    # 正常运行时 (monitor_localization=false) 不主动监控定位健康,
    # 只在收到 SC bridge 的显式 fallback 请求时才启动 HDL recovery。
    hdl_bootstrap_to_initialpose_node = nav2_python_node(
        'hdl_bootstrap_to_initialpose',
        'hdl_bootstrap_to_initialpose',
        {
            'hdl_odom_topic': '/hdl_bootstrap/odom',
            'initialpose_topic': '/initialpose',
            'relocalize_service': '/relocalize',
            'relocalize_with_prior_service': '/relocalize_with_prior',
            'relocalize_checked_service': '/relocalize_checked',
            'relocalize_with_prior_checked_service': '/relocalize_with_prior_checked',
            'startup_origin_relocalize_checked_service': '/relocalize_startup_origin_checked',
            'use_checked_relocalize_service': True,
            'hdl_standby_service': '/hdl_bootstrap/standby',
            'hdl_clear_relocalize_buffer_service': '/hdl_bootstrap/clear_relocalize_buffer',
            'external_relocalize_prior_topic': '/hdl_relocalize_prior',
            'recovery_request_topic': '/localization/recovery_requests',
            'recovery_status_topic': '/localization/recovery_status',
            'ndt_status_topic': '/localization/ndt_status',
            # ★ Phase 3: SC bridge 显式 fallback 触发
            'hdl_fallback_request_topic': '/localization/hdl_fallback_request',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'startup_delay_sec': 2.0,
            'relocalize_retry_sec': 6.0,
            'startup_relocalize_retry_sec': 1.0,
            'runtime_relocalize_retry_sec': 2.0,
            'runtime_relocalize_start_delay_sec': 0.5,
            'clear_relocalize_buffer_on_runtime_recovery': True,
            'runtime_relocalize_buffer_refill_sec': 1.5,
            'wait_stationary_before_runtime_relocalize': True,
            'runtime_stationary_settle_sec': 1.0,
            'runtime_stationary_max_xy_delta': 0.08,
            'runtime_stationary_max_yaw_delta': 0.08,
            'publish_zero_cmd_vel_during_recovery': True,
            'recovery_stop_cmd_vel_topic': '/cmd_vel',
            'recovery_stop_cmd_vel_period_sec': 0.1,
            'max_relocalize_attempts': 0,
            'startup_max_relocalize_attempts': 0,
            'max_runtime_relocalize_attempts': 0,
            'runtime_recovery_failure_cooldown_sec': 10.0,
            'startup_use_origin_prior': False,
            'startup_origin_prior_max_attempts': 3,
            'startup_origin_prior_timeout_sec': 8.0,
            'required_stable_samples': 3,
            'startup_required_stable_samples': 3,
            'runtime_required_stable_samples': 3,
            'stable_xy_tolerance': 0.20,
            'stable_yaw_tolerance': 0.12,
            'sample_wait_timeout_sec': 5.0,
            'startup_sample_wait_timeout_sec': 5.0,
            'runtime_sample_wait_timeout_sec': 4.0,
            'publish_repetitions': 8,
            'startup_publish_repetitions': 8,
            'runtime_publish_repetitions': 8,
            'publish_period_sec': 0.25,
            'exit_after_publish': False,
            'monitor_localization': False,
            'ndt_failure_triggers_recovery': False,
            'ndt_rejected_recovery_count': 2,
            'localization_pose_topic': '/pcl_pose',
            'localization_pose_stale_sec': 2.5,
            'recovery_settle_sec': 6.0,
            'require_ndt_stable_status_for_recovery': True,
            'ndt_recovery_required_stable_status_count': 3,
            'use_prior_relocalize_on_recovery': True,
            'allow_full_global_recovery_without_prior': True,
            'compare_with_hdl': False,
            'hdl_divergence_triggers_recovery': False,
            'xy_covariance': 0.35,
            'yaw_covariance': 0.35,
        },
    )

    # ★★★ HDL 全局重定位服务 (FPFH+RANSAC) ★★★
    # 提供 /relocalize 服务，供 hdl_bootstrap_to_initialpose 调用
    hdl_global_localization_node = Node(
        package='hdl_global_localization',
        executable='hdl_global_localization_node',
        namespace='hdl_global_localization',
        name='global_localization_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'global_localization_engine': 'FPFH_RANSAC',
                'globalmap_downsample_resolution': 0.5,
                'query_downsample_resolution': 0.5,
                'fpfh/normal_estimation_radius': 1.0,
                'fpfh/search_radius': 3.0,
                'ransac/voxel_based': True,
                'ransac/max_iterations': 80000,
                'ransac/matching_budget': 2000,
                'ransac/max_correspondence_distance': 0.8,
                'ransac/similarity_threshold': 0.5,
                'ransac/correspondence_randomness': 3,
                'ransac/inlier_fraction': 0.12,
                'bbs/map_min_z': 0.2,
                'bbs/map_max_z': 1.8,
                'bbs/scan_min_z': 0.2,
                'bbs/scan_max_z': 1.8,
                'bbs/map_width': 160,
                'bbs/map_height': 160,
                'bbs/map_resolution': 0.5,
                'bbs/min_score_ratio': 0.65,
                'bbs/min_tx': -50.0,
                'bbs/max_tx': 50.0,
                'bbs/min_ty': -50.0,
                'bbs/max_ty': 50.0,
                'bbs/min_theta': -3.15,
                'bbs/max_theta': 3.15,
            }
        ],
    )

    # ★ HDL bootstrap 容器 — 细定位 NDT + PCD 地图服务
    hdl_bootstrap_container = ComposableNodeContainer(
        name='hdl_bootstrap_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::GlobalmapServerNodelet',
                name='BootstrapGlobalmapServerNodelet',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'globalmap_pcd': hdl_globalmap_pcd,
                    'convert_utm_to_local': False,
                    'downsample_resolution': 0.1,
                }]
            ),
            ComposableNode(
                package='hdl_localization',
                plugin='hdl_localization::HdlLocalizationNodelet',
                name='BootstrapHdlLocalizationNodelet',
                remappings=[
                    ('/ouster/points', '/fast_lio/cloud_registered'),
                ],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'odom_child_frame_id': 'base_footprint',
                    'robot_odom_frame_id': 'odom',
                    'odom_topic': '/hdl_bootstrap/odom',
                    'send_tf_transforms': False,
                    'use_imu': False,
                    'enable_robot_odometry_prediction': True,
                    'reg_method': 'NDT_OMP',
                    'ndt_resolution': 1.0,
                    'downsample_resolution': 0.1,
                    'reject_scan_matching_without_convergence': True,
                    'max_scan_matching_fitness_score': 0.20,
                }]
            ),
        ],
    )

    # ★★★ Phase 1 NDT 定位节点 v2 ★★★
    # 新增参数:
    #   fusion_status_timeout_sec: 5.0       — fusion_status 超时阈值
    #   allow_ndt_tf_when_fusion_timeout: false — 超时后继续抑制 TF (安全优先)
    ndt_localization_node = Node(
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        name='lidar_localization',
        parameters=[
            localization_params_file,
            {
                'use_sim_time': use_sim_time,
                'set_initial_pose': False,
                'score_threshold': 0.3,
                'reject_pose_jump': True,
                'max_pose_jump_translation': 0.40,
                'max_pose_jump_yaw': 0.30,
                'initialpose_relax_duration_sec': 4.0,
                'initialpose_max_pose_jump_translation': 2.00,
                'initialpose_max_pose_jump_yaw': 1.20,
                'pose_jump_reacquire_enabled': True,
                'pose_jump_reacquire_max_translation': 0.80,
                'pose_jump_reacquire_max_yaw': 0.30,
                'pose_jump_reacquire_max_fitness': 0.08,
                'pose_jump_reacquire_required_frames': 2,
                'pose_jump_reacquire_xy_tolerance': 0.50,
                'pose_jump_reacquire_yaw_tolerance': 0.25,
                'min_scan_points': 50,
                'localization_status_topic': '/localization/ndt_status',
                'republish_last_good_tf_on_failure': False,
                'max_last_good_tf_age_sec': 0.5,
                # ★ NDT 鲁棒性参数 (退化区域防漂移)
                'ndt_outlier_ratio': 0.30,
                'ndt_max_corr_dist': 2.0,
                'ndt_rotation_prior_enabled': True,
                'ndt_rotation_prior_weight': 10.0,
                'ndt_rotation_prior_roll_pitch_only': True,
                # ★ Phase 1 新增: fusion_status 超时保护
                'fusion_status_timeout_sec': 5.0,
                'allow_ndt_tf_when_fusion_timeout': False,
            }
        ],
        remappings=[('/cloud', '/fast_lio/cloud_registered')],
    )
    ndt_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_ndt',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['lidar_localization']
        }]
    )

    # ★★★ Phase 1 fusion 节点 v2 ★★★
    # 关键参数变更 (相对 v1):
    #   min_degraded_lock_sec: 30 → 10     (缩短锁定期, 机器人已静止)
    #   max_degraded_lock_sec: 180 → 10    (同上)
    #   max_odom_displacement_m: 30 → 5    (缩短接管距离)
    localization_odom_fusion_node = nav2_python_node(
        'localization_odom_fusion',
        'localization_odom_fusion',
        {
            'degraded_error_threshold': 0.5,
            'healthy_error_threshold': 0.15,
            'healthy_consecutive_frames': 3,
            'degraded_consecutive_frames': 2,
            'max_degraded_duration_sec': 120.0,
            'max_odom_displacement_m': 5.0,        # ★ v2: 30→5, 缩短接管距离
            'max_total_odom_displacement_m': 100.0,
            'nav_active_lost_timeout_sec': 120.0,
            'nav_idle_lost_timeout_sec': 600.0,
            'nav_idle_extreme_error': 5.0,
            'nav_status_topic': '/navigation_status',
            'recovery_request_cooldown_sec': 15.0,
            'init_timeout_sec': 20.0,
            'recovery_pose_soft_gate_enabled': True,
            'recovery_pose_max_xy_error_m': 5.0,
            'recovery_pose_accept_if_ndt_error_below': 0.03,
            'recovery_pose_skip_odom_after_displacement_m': 20.0,
            'recovery_pose_max_status_age_sec': 2.0,
            'recovery_pose_max_pcl_age_sec': 2.0,
            'pose_jump_degraded_from_status': True,
            'pose_jump_degraded_from_pcl': True,
            'pose_jump_pcl_threshold_m': 0.5,
            'pose_jump_correction_threshold_m': 0.35,
            'transition_duration_sec': 2.0,
            # ★ v2: 缩短锁定期 (机器人 DEGRADED 后立即停车, 不需要长锁定期)
            'min_degraded_lock_sec': 10.0,           # ★ v2: 30→10
            'max_degraded_lock_sec': 10.0,           # ★ v2: 180→10
            'lock_recovery_healthy_consecutive_frames': 10,
            'lock_recovery_max_correction_m': 0.3,
            'lock_early_lost_rejection_rate': 0.9,
            'lock_early_lost_min_frames': 30,
            'recovery_pose_jump_max_m': 5.0,
            'publish_rate_hz': 30.0,
            'verbose_logging': True,
        },
    )

    # =========================================================================
    # 第四部分：辅助节点
    # =========================================================================
    periodic_clearing_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='humanoid_navigation2',
                executable='periodic_clearing_publisher',
                name='periodic_clearing_publisher',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    periodic_clearing_3d_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='humanoid_navigation2',
                executable='periodic_clearing_3d_publisher',
                name='periodic_clearing_3d_publisher',
                parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # =========================================================================
    # 第五部分：Nav2导航栈
    # =========================================================================
    localization_ready_gate = nav2_python_node(
        'wait_for_tf',
        'wait_for_localization_tf',
        {
            'target_frame': 'map_ground',
            'source_frame': 'base_footprint',
            'timeout_sec': 0.0,
            'poll_period': 0.2,
            'stable_count': 3,
        }
    )

    nav2_core_nodes = GroupAction([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                nav2_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'default_nav_to_pose_bt_xml': bt_xml_file,
                    'default_nav_through_poses_bt_xml': bt_xml_file
                }
            ]
        )
    ])

    nav2_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'bond_timeout': 0.0},
            {'node_names': [
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator'
            ]}
        ]
    )

    start_nav2_after_localization = RegisterEventHandler(
        OnProcessExit(
            target_action=localization_ready_gate,
            on_exit=[
                nav2_core_nodes,
                TimerAction(period=3.0, actions=[nav2_lifecycle_node]),
            ],
        )
    )

    # =========================================================================
    # 组装Launch描述
    # =========================================================================
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params_file),
        DeclareLaunchArgument('bt_xml_file', default_value=default_bt_xml_file),
        DeclareLaunchArgument('enable_elevation_map', default_value='false'),
        DeclareLaunchArgument('enable_terrain_analysis', default_value='false'),
        DeclareLaunchArgument('enable_periodic_clearing', default_value='true'),
        DeclareLaunchArgument('enable_fastdds_shm', default_value='true',
                              description='Enable FastDDS shared memory optimization'),
        DeclareLaunchArgument('fusion_mode', default_value='false',
                              description='Enable NDT+odometry fusion mode'),

        *fastdds_env_setup,

        rslidar_node,
        fast_lio_node,

        tf_bridge_odom,
        tf_map_to_ground,
        tf_odom_to_ground,
        tf_bridge_base,
        tf_bridge_clearing_lidar,

        point_cloud_filter_launch,

        map_server_node,
        map_server_lifecycle,

        # HDL 全局重定位基础设施 (FPFH+RANSAC + 细定位 NDT)
        # 必须在 hdl_bootstrap_to_initialpose 之前启动，否则 /relocalize 服务不可用
        hdl_global_localization_node,
        hdl_bootstrap_container,

        TimerAction(period=4.0, actions=[scancontext_global_localizer_node]),
        TimerAction(period=5.0, actions=[scancontext_to_initialpose_node]),
        # ★ Phase 3: HDL fallback 桥接 (10s 延迟, 确保 HDL 容器 + SC bridge 先就绪)
        TimerAction(period=10.0, actions=[hdl_bootstrap_to_initialpose_node]),

        TimerAction(period=5.0, actions=[ndt_localization_node]),
        TimerAction(period=12.0, actions=[ndt_lifecycle_manager]),

        TimerAction(period=8.0, actions=[localization_odom_fusion_node]),

        TimerAction(period=7.5, actions=[robot_realpose_publisher]),

        periodic_clearing_node,
        periodic_clearing_3d_node,

        localization_ready_gate,
        start_nav2_after_localization,
    ])
