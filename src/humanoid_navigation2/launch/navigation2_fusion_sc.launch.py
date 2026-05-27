"""
完整导航栈 Launch 文件 - NDT + Odom Fusion + ScanContext 全局重定位

系统架构：
1. 感知层：点云滤波、高程图（可选）、地形分析（可选）
2. 定位层：定位节点（发布map→odom TF）+ map_server（发布2D栅格地图）
   支持三种定位方案（三选一，注释切换）：
     方案A - lidar_localization_ros2 (单分辨率NDT, 旧方案)
     方案B - humanoid_global_localization (多分辨率NDT网格搜索)
     方案C - hdl_localization (UKF+NDT_OMP, Humble移植)
   当前实验版：方案C负责启动和运行期全局重定位，桥接到方案A的/initialpose；
   方案A发布map→odom并持续定位，失锁后由C自动恢复。
3. 导航层：Nav2导航栈（规划、控制、行为树）

启动顺序：
0秒：感知层（点云滤波）
1秒：map_server（加载2D地图）
3秒：map_server生命周期管理
5秒：定位节点启动
7秒：定位生命周期管理
7.5秒：机器人实时位姿发布器
6秒：Nav2核心节点
12秒：Nav2生命周期激活

TF树：
map → odom → camera_init → body → base_footprint → base_link → 传感器
  ↑        ↑
定位发布  Fast-LIO发布
(动态)   (动态)

使用方式：
ros2 launch humanoid_navigation2 navigation2_fusion_sc.launch.py use_sim_time:=false
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
    SetEnvironmentVariable  # ★ 新增：用于设置 FastDDS 环境变量
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # =========================================================================
    # FastDDS 共享内存优化配置
    # 用途：解决点云传输导致的帧率下降问题
    # 官方文档：https://robosense.feishu.cn/wiki/KBr3wQgqZiLLrNk4pTdcNZfpnld
    # =========================================================================
    
    # 允许通过参数禁用 FastDDS 优化（默认为启用）
    enable_fastdds_shm = LaunchConfiguration('enable_fastdds_shm', default='true')
    
    # FastDDS 配置文件路径
    fastdds_config_file = PathJoinSubstitution([
        os.path.expanduser('~'),
        '.config',
        'fastdds_shm.xml'
    ])
    
    # 设置 FastDDS 共享内存环境变量
    # 这些环境变量会在所有节点启动前设置，确保整个系统使用共享内存
    fastdds_env_setup = [
        # 设置 RMW 实现为 FastDDS（如果未设置则使用默认值）
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            'rmw_fastrtps_cpp'
        ),
        # 设置 FastDDS 配置文件路径
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            fastdds_config_file
        ),
        # 启用从 XML 读取 QoS 配置
        SetEnvironmentVariable(
            'RMW_FASTRTPS_USE_QOS_FROM_XML',
            '1'
        ),
    ]
    
    # ========== 路径配置 ==========
    pkg_nav2 = get_package_share_directory('humanoid_navigation2')
    pkg_global_loc = get_package_share_directory('humanoid_global_localization')
    pkg_hdl_loc = get_package_share_directory('hdl_localization')
    pkg_hdl_global_loc = get_package_share_directory('hdl_global_localization')
    pkg_lidar_loc = get_package_share_directory('lidar_localization_ros2')  # [方案A: 旧NDT]
    pkg_scancontext_loc = get_package_share_directory('humanoid_scancontext_global_localization')

    # 参数文件
    default_nav2_params_file = os.path.join(pkg_nav2, 'config', 'nav2_params_xy_yaw.yaml')
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=default_nav2_params_file)
    # --- 方案B: 多分辨率 NDT 全局定位 (当前使用) ---
    global_localization_params_file = os.path.join(pkg_global_loc, 'config', 'global_localization.yaml')
    # --- 方案A: 单分辨率 NDT 定位 (旧方案) ---
    localization_params_file = os.path.join(pkg_lidar_loc, 'param', 'localization.yaml')
    # --- ScanContext: 替代 HDL bootstrap 的启动/恢复重定位候选 ---
    scancontext_params_file = os.path.join(
        pkg_scancontext_loc, 'config', 'scancontext_global_localization.yaml')
    scancontext_database_file = os.path.join(pkg_nav2, 'maps', 'hall_sc_fastlio_registered.bin')
    scancontext_pcd_map_file = os.path.join(pkg_nav2, 'pcd', 'hall.pcd')
    # --- 方案C: hdl_localization UKF+NDT (Humble移植) ---
    hdl_globalmap_pcd = '/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall_localization_grounded.pcd'  # 已转为标准坐标系并以base_footprint为地面原点
    # hdl 输入点云已通过 body -> base_footprint 转到机器人导航基准系。
    # Nav2 和定位统一使用 map/odom，避免重复叠加高度偏移。
    
    # 地图文件（2D栅格地图，用于Nav2）
    map_yaml_file = os.path.join(pkg_nav2, 'maps', 'hall.yaml')
    
    # 行为树文件
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
    
    # 功能开关（可选启用/禁用某些模块）
    enable_elevation_map = LaunchConfiguration('enable_elevation_map', default='false')
    enable_terrain_analysis = LaunchConfiguration('enable_terrain_analysis', default='false')
    enable_periodic_clearing = LaunchConfiguration('enable_periodic_clearing', default='true')

    # =========================================================================
    # 第一部分：传感器与Fast-LIO节点
    # =========================================================================
    
    # 1.雷达驱动
    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2.Fast-LIO节点（发布/fast_lio/cloud_registered）
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

    # 3.TF桥接（静态变换）
    # odom → camera_init
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

    # map -> map_ground 的动态 TF。
    # NDT 只发布平面 map->odom，但 Fast-LIO 的 odom->base_footprint 高度会随
    # 路线漂移。如果 map_ground 固定在启动高度，RViz 和跨 frame 的 costmap
    # 会在远处被整体抬高/压低。这里让 map_ground 跟随当前 base 高度，使
    # base_footprint 在 map_ground 中保持接近 z=0，并与 odom_ground 对齐。
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

    # odom -> odom_ground 的动态 TF。
    # Fast-LIO 的 odom 高度会有慢漂，固定 -1.215m 会让 local_costmap
    # 相对真实地面上下偏移。这里让 odom_ground 始终贴到 base_footprint
    # 当前高度，使 base_footprint 在 local costmap 中接近 z=0。
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

    # body → base_footprint
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

    # base_footprint → clearing_lidar
    # The local VoxelLayer uses this as the true LiDAR-height sensor origin for
    # raytracing. Publish it from launch so obstacle marking still has a valid
    # sensor frame even if the optional clearing cloud publisher is disabled.
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
    # 第二部分：感知层 （延迟启动，等待传感器数据稳定）
    # =========================================================================
    
    # 1. 点云滤波节点（处理原始点云，输出滤波后的点云）
    point_cloud_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('humanoid_point_cloud_filter'),
                'launch',
                'point_cloud_filter.launch.py'
            )
        )
    )

    # 2. 高程图节点（可选，用于3D地形感知）
    elevation_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('elevation_mapping_opencl'),
                'launch',
                'elevation_mapping.launch.py'
            )
        )
    )

    # 3. 地形分析节点（可选，用于地形分类）
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
    # 第三部分：定位层（延迟启动，等待数据稳定）
    # =========================================================================
    
    # 1.map_server先启动，加载2D地图 (延迟时间改为 1.0)
    map_server_node = TimerAction(
        period=1.0,
        actions=[ Node(
                package='nav2_map_server', executable='map_server', name='map_server',
                parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_yaml_file}, {'frame_id': 'map_ground'}]) ]
    )

    # 2.map_server生命周期管理，等待图层加载完再激活
    # 开机时 Fast-LIO/SC/GICP 同时启动，CPU 和 DDS discovery 压力较大；
    # lifecycle_manager 太早配置会偶发 Failed to change state。
    map_server_lifecycle = TimerAction(
        period=6.0,
        actions=[ Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
                parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': ['map_server']}]) ]
    )

    # ╔══════════════════════════════════════════════════════════════════════════╗
    # ║                   定位方案选择 (三选一，注释/取消注释切换)                ║
    # ║                                                                          ║
    # ║  方案A: lidar_localization_ros2 — 单分辨率NDT (旧方案，简单稳定)         ║
    # ║  方案B: humanoid_global_localization — 多分辨率NDT网格搜索 (推荐)    ║
    # ║  方案C: hdl_localization — UKF+NDT_OMP (Humble移植，最成熟)          ║
    # ║                                                                          ║
    # ║  切换步骤：                                                              ║
    # ║    1. 在下面"节点定义"区域注释掉当前方案的代码块                           ║
    # ║    2. 取消注释目标方案的代码块                                            ║
    # ║    3. 在底部 LaunchDescription 区域做同样操作                             ║
    # ╚══════════════════════════════════════════════════════════════════════════╝

    # ┌──────────────────────────────────────────────────────────────────────────┐
    # │  方案B：多分辨率 NDT 全局定位 (已注释, 切换到方案C)                       │
    # │  算法：粗NDT(3m)网格搜索 → 中NDT(1.5m)精化 → 细NDT(1m)跟踪 + 指数平滑   │
    # │  特点：可在任意位置启动，仅依赖PCL，550行代码易维护                       │
    # │  启动：延迟5秒启动节点 + 7秒激活生命周期                                   │
    # └──────────────────────────────────────────────────────────────────────────┘
    # global_localization_node = Node(
    #     package='humanoid_global_localization',
    #     executable='global_localization_node',
    #     name='global_localization',
    #     output='screen',
    #     parameters=[global_localization_params_file, {'use_sim_time': use_sim_time}],
    # )
    # global_loc_lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_global_loc',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'autostart': True,
    #         'bond_timeout': 0.0,
    #         'node_names': ['global_localization']
    #     }]
    # )

    # 5. 机器人实时位姿发布器（从 TF 读取 map_ground->base_footprint）
    #    与 /pcl_pose 不同：/pcl_pose 发布的是 map->odom 偏移量（通常 0.1-0.5m），
    #    本节点通过完整 TF 链计算机器人在导航地面坐标系中的实际位姿，发布到 /robot_realpose
    robot_realpose_publisher = nav2_python_node(
        'robot_realpose_publisher',
        'robot_realpose_publisher',
        {'global_frame': 'map_ground'}
    )

    # hdl_localization 的全局重定位服务（3D FPFH + RANSAC）
    # 用于机器人不在原点附近启动或被搬动后的 /relocalize。
    # hdl_global_localization_node = Node(
    #     package='hdl_global_localization',
    #     executable='hdl_global_localization_node',
    #     namespace='hdl_global_localization',
    #     name='global_localization_node',
    #     output='screen',
    #     parameters=[
    #         {
    #             'use_sim_time': use_sim_time,
    #             'global_localization_engine': 'FPFH_RANSAC',
    #             'globalmap_downsample_resolution': 0.5,
    #             'query_downsample_resolution': 0.5,
    #             'fpfh/normal_estimation_radius': 1.0,
    #             'fpfh/search_radius': 3.0,
    #             'ransac/voxel_based': True,
    #             'ransac/max_iterations': 80000,
    #             'ransac/matching_budget': 2000,
    #             'ransac/max_correspondence_distance': 0.8,
    #             'ransac/similarity_threshold': 0.5,
    #             'ransac/correspondence_randomness': 3,
    #             'ransac/inlier_fraction': 0.12,
    #             'bbs/map_min_z': 0.2,
    #             'bbs/map_max_z': 1.8,
    #             'bbs/scan_min_z': 0.2,
    #             'bbs/scan_max_z': 1.8,
    #             'bbs/map_width': 160,
    #             'bbs/map_height': 160,
    #             'bbs/map_resolution': 0.5,
    #             'bbs/min_score_ratio': 0.65,
    #             'bbs/min_tx': -50.0,
    #             'bbs/max_tx': 50.0,
    #             'bbs/min_ty': -50.0,
    #             'bbs/max_ty': 50.0,
    #             'bbs/min_theta': -3.15,
    #             'bbs/max_theta': 3.15,
    #         }
    #     ],
    # )

    # ┌──────────────────────────────────────────────────────────────────────────┐
    # │  ScanContext-recovery + 方案A-tracking 实验方案                           │
    # │                                                                          │
    # │  ScanContext 不发布 map->odom TF；它只在启动或 fusion LOST 时给 NDT        │
    # │  一个经过 odom gate/GICP gate 的 /initialpose。HEALTHY 时由 NDT 发布       │
    # │  map->odom；DEGRADED/TRANSITIONING 时由 fusion 接管 map->odom。            │
    # └──────────────────────────────────────────────────────────────────────────┘
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
                'global_recovery_min_gicp_fitness_gap': 0.02,
                'global_recovery_same_solution_xy_tolerance': 0.5,
                'global_recovery_same_solution_yaw_tolerance': 0.35,
                'global_recovery_required_consistent_results': 4,
                'global_recovery_consistency_window': 8,
                'global_recovery_consistency_xy_tolerance': 0.5,
                'global_recovery_consistency_yaw_tolerance': 0.25,
                'global_recovery_observation_max_age_sec': 20.0,
                # ★ 全局模式下跳过 SC descriptor ambiguity gate，但 GICP 必须有清晰第一名。
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
            'global_recovery_after_attempts': 3,  # 运行时: conservative 先验证 odom gate，失败后升级 global
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
            # ★ 融合模式: 运行期只由 fusion LOST 请求触发 recovery
            #   禁止 SC bridge 自行根据 /pcl_pose stale 触发 recovery
            'enable_runtime_auto_recovery': False,
            'publish_repetitions': 8,
            'publish_period_sec': 0.25,
            'min_publish_interval_sec': 12.0,
            'xy_covariance': 0.25,
            'z_covariance': 0.04,
            'yaw_covariance': 0.10,
        }
    )



    # ┌──────────────────────────────────────────────────────────────────────────┐
    # │  方案A：单分辨率 NDT 定位 (旧方案, 已注释)                                │
    # │  算法：PCL NDT_OMP, 1m分辨率，无全局搜索                                │
    # │  特点：需要人工指定初始位姿或放在原点附近(<0.5m)，简单稳定               │
    # │  依赖：lidar_localization_ros2 包                                        │
    # └──────────────────────────────────────────────────────────────────────────┘
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
                # ★ 严格阈值: odom兜底可靠,NDT宁可拒帧也不接受低质量匹配
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
                # NDT拒帧时不重发旧TF，避免与fusion DEGRADED冻结TF冲突
                'republish_last_good_tf_on_failure': False,
                'max_last_good_tf_age_sec': 0.5,
                # ★ NDT 鲁棒性参数 (退化区域防漂移)
                'ndt_outlier_ratio': 0.30,                  # 降低离群率 → 更强约束
                'ndt_max_corr_dist': 2.0,                   # 2m外关联直接跳过
                'ndt_rotation_prior_enabled': True,         # 启用 roll/pitch 先验
                'ndt_rotation_prior_weight': 10.0,          # roll/pitch 正则化权重
                'ndt_rotation_prior_roll_pitch_only': True, # 只约束 roll/pitch（yaw 给 NDT）
                # ★ 退化诊断指标（在 ndt_status JSON 里自动输出 mean_corr_dist）
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

    # ┌──────────────────────────────────────────────────────────────────────────┐
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
    # 组装Launch描述（按启动顺序）
    # =========================================================================
    return LaunchDescription([
        # ========== 参数声明 ==========
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params_file),
        DeclareLaunchArgument('bt_xml_file', default_value=default_bt_xml_file),
        DeclareLaunchArgument('enable_elevation_map', default_value='false'),
        DeclareLaunchArgument('enable_terrain_analysis', default_value='false'),
        DeclareLaunchArgument('enable_periodic_clearing', default_value='true'),
        # ★ 新增：允许通过参数控制 FastDDS 共享内存优化（默认启用）
        DeclareLaunchArgument('enable_fastdds_shm', default_value='true', description='Enable FastDDS shared memory optimization for point cloud topics'),
        # 定位融合模式开关 (true=融合节点兜底+不打断导航, false=原逻辑立刻recovery)

        # ========== FastDDS 共享内存优化（在所有节点前设置） ==========
        # 这些环境变量设置会在所有节点启动前执行
        # 确保雷达、Fast-LIO、Filter等所有节点都使用共享内存
        *fastdds_env_setup,

        # ========== 第一部分：硬件层 传感器与Fast-LIO节点 ==========
        # 1. 传感器与Fast-LIO
        rslidar_node,
        fast_lio_node,

        # 2. TF桥接
        tf_bridge_odom,
        tf_map_to_ground,
        tf_odom_to_ground,
        tf_bridge_base,
        tf_bridge_clearing_lidar,
        # ========== 第二部分：感知层（最先启动） ==========
        # 点云滤波最先启动，处理原始点云数据dan
        point_cloud_filter_launch,
        
        # 高程图和地形分析（可选，根据参数条件启动）
        # 如果启用，延迟启动等待滤波稳定
        # TimerAction(period=1.0, actions=[elevation_map_launch]),  # 需要时取消注释
        # TimerAction(period=2.0, actions=[terrain_analyzer_launch]),  # 需要时取消注释

        # ========== 第三部分：定位层（延迟启动，确保Fast-LIO数据就绪） ==========
        # map_server先启动（1秒），加载2D地图
        map_server_node,

        # map_server生命周期管理（3秒），自动激活
        map_server_lifecycle,

        # ┌─ 方案B：多分辨率 NDT 全局定位 (已注释) ─┐
        # TimerAction(period=5.0, actions=[global_localization_node]),
        # TimerAction(period=7.0, actions=[global_loc_lifecycle_manager]),
        # └──────────────────────────────────────────┘

        # ┌─ ScanContext-recovery：启动和运行期全局重定位，不发布 map->odom TF ─┐
        TimerAction(period=4.0, actions=[scancontext_global_localizer_node]),
        TimerAction(period=5.0, actions=[scancontext_to_initialpose_node]),
        # └──────────────────────────────────────────────────────────────┘

        # ┌─ 方案A：单分辨率 NDT 定位，等待 SC recovery 发布 /initialpose ─┐
        TimerAction(period=5.0, actions=[ndt_localization_node]),
        TimerAction(period=12.0, actions=[ndt_lifecycle_manager]),
        # └──────────────────────────────────────────────┘
