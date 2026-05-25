"""
完整导航栈 Launch 文件（包含感知+定位+导航）

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
ros2 launch humanoid_navigation2 navigation2.launch.py use_sim_time:=false
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

    # 参数文件
    default_nav2_params_file = os.path.join(pkg_nav2, 'config', 'nav2_params_xy_yaw.yaml')
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=default_nav2_params_file)
    # --- 方案B: 多分辨率 NDT 全局定位 (当前使用) ---
    global_localization_params_file = os.path.join(pkg_global_loc, 'config', 'global_localization.yaml')
    # --- 方案A: 单分辨率 NDT 定位 (旧方案) ---
    localization_params_file = os.path.join(pkg_lidar_loc, 'param', 'localization.yaml')
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

    # 2.map_server生命周期管理，等待图层加载完再激活 (延迟时间改为 3.0)
    map_server_lifecycle = TimerAction(
        period=3.0,
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
    # │  方案C-recovery + 方案A-tracking 实验方案                                 │
    # │                                                                          │
    # │  C 不发布 map->odom TF；桥接节点把 C 的 /hdl_bootstrap/odom 换算成 A 需要的 │
    # │  /initialpose。A 独占发布 map->odom；A 失锁时桥接节点自动调用 /relocalize。 │
    # └──────────────────────────────────────────────────────────────────────────┘
    hdl_bootstrap_global_localization_node = Node(
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
                    'min_scan_matching_inlier_fraction': 0.78,
                    'scan_matching_inlier_max_correspondence_distance': 0.5,
                    'max_scan_matching_correction_translation': 0.80,
                    'max_scan_matching_correction_yaw': 0.45,
                    'recovery_jump_gate_relax_frames': 8,
                    'recovery_jump_gate_relax_sec': 5.0,
                    'recovery_max_scan_matching_correction_translation': 3.0,
                    'recovery_max_scan_matching_correction_yaw': 1.2,
                    'scan_matching_jump_override_max_fitness_score': 0.12,
                    'scan_matching_jump_override_min_inlier_fraction': 0.90,
                    'scan_matching_rejected_log_throttle_ms': 30000,
                    'publish_odom_prediction_on_rejection': False,
                    'max_odom_prediction_rejections': 3,
                    'pointcloud_transform_timeout_sec': 0.15,
                    'cool_time_duration': 2.0,
                    'specify_init_pose': False,
                    'use_global_localization': True,
                    'auto_relocalize_on_start': False,
                    'auto_relocalize_after_rejections': 0,
                    'global_localization_pose_z_offset': 0.0,
                    'validate_global_localization_with_scan_matching': True,
                    'global_localization_max_fitness_score': 0.12,
                    'global_localization_max_candidates': 10,
                    'global_localization_min_fitness_margin': 0.03,
                    'global_localization_ambiguous_max_fitness_score': 0.07,
                    'global_localization_recovery_prior_max_xy': 4.0,
                    'global_localization_recovery_prior_max_yaw': 0.0,
                    'global_localization_recovery_prior_hard_gate': True,
                    # 开机 bootstrap 专用原点先验；运行中恢复关闭先验，
                    # 直接走 /relocalize_checked 做全图搜索。
                    'startup_origin_prior_enabled': True,
                    'startup_origin_prior_x': 0.0,
                    'startup_origin_prior_y': 0.0,
                    'startup_origin_prior_z': 0.0,
                    'startup_origin_prior_yaw': 0.0,
                    'startup_origin_prior_max_xy': 1.0,
                    'startup_origin_prior_max_yaw': 0.0,
                    'startup_origin_prior_hard_gate': True,
                    'external_recovery_prior_topic': '/hdl_relocalize_prior',
                    'external_recovery_prior_max_age_sec': 15.0,
                    'global_localization_required_consistent_results': 3,
                    'global_localization_consistency_window': 5,
                    'global_localization_consistency_xy_tolerance': 0.8,
                    'global_localization_consistency_yaw_tolerance': 0.35,
                    # /fast_lio/cloud_registered 的 frame_id 是 camera_init，原始轴为
                    # x左/y下/z后；上面的 odom->camera_init 静态 TF 已经负责把它
                    # 转到 ROS 标准轴，HDL 内部再转到 base_footprint。
                    'global_localization_query_accumulation_frames': 5,
                    'global_localization_query_min_accumulation_frames': 3,
                    'global_localization_post_accept_validation_frames': 5,
                    'global_localization_post_accept_max_rejections': 1,
                    'global_localization_enforce_xy_bounds': True,
                    'global_localization_min_x': -6.675,
                    'global_localization_max_x': 26.325,
                    'global_localization_min_y': -12.819,
                    'global_localization_max_y': 26.231,
                    'force_2d_pose': True,
                    'force_2d_fixed_z': True,
                    'global_localization_use_height_filter': True,
                    'global_localization_min_z': 0.0,
                    'global_localization_max_z': 0.0,
                    'global_localization_use_max_z_filter': False,
                    'global_localization_query_timeout_sec': 30.0,
                    'global_localization_recovery_query_timeout_sec': 30.0,
                }]
            )
        ],
        output='screen',
    )

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
            'ndt_status_topic': '/localization/ndt_status',
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
            'runtime_recovery_failure_cooldown_sec': 5.0,
            'startup_use_origin_prior': True,
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
            'monitor_localization': True,
            # ★ 融合模式: NDT异常时不触发recovery，由fusion节点决定何时recovery
            'ndt_failure_triggers_recovery': False,
            'ndt_rejected_recovery_count': 2,
            'localization_pose_topic': '/pcl_pose',
            'localization_pose_stale_sec': 2.5,
            'map_to_odom_tf_stale_sec': 3.0,
            'recovery_settle_sec': 6.0,
            'startup_recovery_settle_sec': 6.0,
            'runtime_recovery_settle_sec': 6.0,
            'min_recovery_interval_sec': 12.0,
            'hdl_status_topic': '/status',
            'require_hdl_status': True,
            'hdl_status_stale_sec': 2.0,
            'hdl_max_matching_error': 0.20,
            'hdl_min_inlier_fraction': 0.78,
            'use_prior_relocalize_on_recovery': False,
            'allow_full_global_recovery_without_prior': True,
            'compare_with_hdl': False,
            'hdl_divergence_triggers_recovery': False,
            'hdl_pose_stale_sec': 2.0,
            'a_hdl_max_xy_delta': 0.80,
            'a_hdl_max_yaw_delta': 0.50,
            'prior_requires_recent_agreement_sec': 8.0,
            'trusted_pose_max_age_sec': 90.0,
            'trusted_pose_update_min_interval_sec': 0.5,
            'trusted_pose_requires_hdl_status': False,
            'trusted_pose_log_interval_sec': 30.0,
            'manual_initialpose_recovery_lockout_sec': 6.0,
            'external_prior_ready_delay_sec': 0.2,
            'external_recovery_request_cooldown_sec': 10.0,
            'require_ndt_stable_status_for_recovery': True,
            'ndt_recovery_required_stable_status_count': 3,
            'ndt_recovery_status_stale_sec': 1.5,
            'ndt_recovery_max_correction_translation': 0.80,
            'ndt_recovery_max_correction_yaw': 0.45,
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
                'score_threshold': 2.0,
                'reject_pose_jump': False,
                'max_pose_jump_translation': 0.80,
                'max_pose_jump_yaw': 0.45,
                'initialpose_relax_duration_sec': 4.0,
                'initialpose_max_pose_jump_translation': 2.00,
                'initialpose_max_pose_jump_yaw': 1.20,
                'pose_jump_reacquire_enabled': True,
                'pose_jump_reacquire_max_translation': 2.00,
                'pose_jump_reacquire_max_yaw': 0.45,
                'pose_jump_reacquire_max_fitness': 0.10,
                'pose_jump_reacquire_required_frames': 2,
                'pose_jump_reacquire_xy_tolerance': 0.50,
                'pose_jump_reacquire_yaw_tolerance': 0.25,
                'min_scan_points': 50,
                'localization_status_topic': '/localization/ndt_status',
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
    # │  NDT定位 + 里程计融合节点 (localization_odom_fusion)                     │
    # │                                                                          │
    # │  核心职责:                                                               │
    # │    当 NDT matching_error > 0.5 时，冻结 map->odom，                      │
    # │    让 Fast-LIO 里程计 (camera_init->body) 驱动机器人运动。               │
    # │    NDT 恢复后 (error < 0.15) 平滑过渡切回。                              │
    # │                                                                          │
    # │  状态机:                                                                 │
    # │    HEALTHY ──error>0.5──▶ DEGRADED ──error<0.15──▶ TRANSITIONING        │
    # │              DEGRADED ──超时/位移过大──▶ LOST ──recovery──▶ HEALTHY      │
    # │                                                                          │
    # │  关键参数:                                                               │
    # │    - degraded_error_threshold: 进入 DEGRADED 的 NDT error 阈值           │
    # │    - healthy_error_threshold: 恢复 HEALTHY 的 NDT error 阈值              │
    # │    - max_degraded_duration_sec: 最长冻结时间，超时进入 LOST               │
    # │    - max_odom_displacement_m: 最大 odom 位移，超过进入 LOST               │
    # │    - transition_duration_sec: 平滑过渡时间                                │
    # └──────────────────────────────────────────────────────────────────────────┘
    localization_odom_fusion_node = nav2_python_node(
        'localization_odom_fusion',
        'localization_odom_fusion',
        {
            'degraded_error_threshold': 0.5,
            'healthy_error_threshold': 0.15,
            'healthy_consecutive_frames': 3,
            'degraded_consecutive_frames': 2,
            'max_degraded_duration_sec': 120.0,
            'max_odom_displacement_m': 30.0,
            'transition_duration_sec': 2.0,
            'publish_rate_hz': 30.0,
            'verbose_logging': True,
        },
    )

    # ┌──────────────────────────────────────────────────────────────────────────┐
    # │  方案C：hdl_localization (Humble移植, 已注释)                             │
    # │                                                                          │
    # │  算法：UKF(16维状态)预测 + NDT_OMP扫描匹配校正                           │
    # │        状态: [位置(3),速度(3),姿态(4),加速度偏置(3),陀螺仪偏置(3)]       │
    # │        预测: IMU数据驱动(加速度+角速度) → UKF sigma点传播                │
    # │        校正: NDT_OMP多线程扫描匹配结果 → UKF观测更新                     │
    # │        辅助: TF链odometry delta预测(可选)                                │
    # │        + hdl_global_localization BBS/FPFH+RANSAC全局重定位(可选)          │
    # │                                                                          │
    # │  特点：论文方案(koide3,名古屋大学/AIST)，UKF比指数平滑理论更完备         │
    # │                                                                          │
    # │  依赖：ndt_omp + fast_gicp (已编译适配Jazzy, 修复4处API)                 │
    # │                                                                          │
    # │  ╔══════════════════════════════════════════════════════════════╗        │
    # │  ║  使用方案C前的准备工作:                                      ║        │
    # │  ║                                                              ║        │
    # │  ║  1. PCD地图预转换 (一次性):                                  ║        │
    # │  ║     python3 -c "from humanoid_navigation2.pcd_converter     ║        │
    # │  ║         import convert_pcd;                                  ║        │
    # │  ║         convert_pcd('hall.pcd', 'hall_standard.pcd')"       ║        │
    # │  ║                                                              ║        │
    # │  ║  2. IMU坐标转换节点 (自动启动):                              ║        │
    # │  ║     imu_transformer: /airy_imu(body帧)→/imu_standard(标准帧) ║        │
    # │  ║     转换矩阵: R_body_to_std=[[0,0,-1],[1,0,0],[0,-1,0]]    ║        │
    # │  ║                                                              ║        │
    # │  ║  3. 点云坐标转换 (hdl内部通过TF自动完成):                   ║        │
    # │  ║     body帧→base_footprint帧 (由body→base_footprint静态TF)   ║        │
    # │  ║                                                              ║        │
    # │  ║  4. Odom预测 (hdl内部通过TF链自动完成):                      ║        │
    # │  ║     通过TF查base_footprint帧间delta, 在odom帧表达           ║        │
    # │  ╚══════════════════════════════════════════════════════════════╝        │
    # └──────────────────────────────────────────────────────────────────────────┘
    # --- 方案C数据预处理: IMU坐标转换 ---
    # 将 /airy_imu 从 body帧(非标) 旋转到 base_footprint帧(标准), 发布 /imu_standard
    # imu_transformer_node = nav2_python_node(
    #     'imu_transformer',
    #     'imu_transformer'
    # )

    # # --- 方案C核心: hdl_localization 容器 ---
    # # 包含 GlobalmapServer (加载标准坐标系PCD地图) + HdlLocalization (UKF+NDT定位)
    # hdl_container = ComposableNodeContainer(
    #     name='hdl_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container_mt',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='hdl_localization',
    #             plugin='hdl_localization::GlobalmapServerNodelet',
    #             name='GlobalmapServerNodelet',
    #             parameters=[{
    #                 'use_sim_time': use_sim_time,
    #                 'globalmap_pcd': hdl_globalmap_pcd,  # 预转换后的标准坐标系PCD
    #                 'convert_utm_to_local': False,       # 不需要UTM转换
    #                 'downsample_resolution': 0.1,
    #             }]
    #         ),
    #         ComposableNode(
    #             package='hdl_localization',
    #             plugin='hdl_localization::HdlLocalizationNodelet',
    #             name='HdlLocalizationNodelet',
    #             remappings=[
    #                 ('/ouster/points', '/fast_lio/cloud_registered'),    # Fast-LIO世界点云(camera_init帧), hdl内部TF到base_footprint
    #                 ('/gpsimu_driver/imu_data', '/imu_standard'),       # IMU(标准帧, 经imu_transformer转换)
    #             ],
    #             parameters=[{
                    # 'use_sim_time': use_sim_time,
                    # # --- 帧配置 ---
                    # 'odom_child_frame_id': 'base_footprint',  # 传感器/机器人帧(标准坐标系)
                    # 'robot_odom_frame_id': 'odom',            # 里程计帧ID
                    # 'odom_topic': '/hdl/odom',
                    # # --- IMU ---
                    # 'use_imu': False,           # Fast-LIO 已融合 IMU；hdl 只用 NDT + Fast-LIO TF 预测
                    # 'invert_acc': False,         # 不需要翻转, imu_transformer已处理坐标转换
                    # 'invert_gyro': False,
                    # # --- 里程计预测 ---
                    # 'enable_robot_odometry_prediction': True,  # 启用odom预测 (通过TF链自动转换)
                    # # --- 扫描匹配 ---
                    # 'reg_method': 'NDT_OMP',     # 多线程NDT加速
                    # 'ndt_resolution': 1.0,        # NDT分辨率(米), 室内推荐0.5~1.0
                    # 'downsample_resolution': 0.1, # 点云降采样(米)
                    # 'reject_scan_matching_without_convergence': True,
                    # # 正常定位约 fitness=0.05；弯角漂移时约 0.30。
                    # # 收紧阈值，避免错误局部最优继续发布 map->odom。
                    # 'max_scan_matching_fitness_score': 0.20,
                    # # 正常定位 inlier_fraction 约 0.95；弯角漂移时降到约 0.76。
                    # # 这次 2026-05-14 19:29 的首个失配帧为 0.846，fitness 仍正常；
                    # # 0.85 太贴边会过早断开 NDT 校正，之后只靠 odom 预测容易飘进墙。
                    # # 低于阈值说明当前点云和地图匹配置信度不足，拒绝本帧 NDT 校正。
                    # # 19:46:33 的首个失配帧为 fitness=0.192、inlier=0.798；
                    # # 仍在 fitness 门限内，避免因 0.002 的边界误差过早进入纯 odom 预测。
                    # 'min_scan_matching_inlier_fraction': 0.78,
                    # 'scan_matching_inlier_max_correspondence_distance': 0.5,
                    # # Fast-LIO 原始 camera_init/body 轴不是 ROS 标准轴，所以不要直接用
                    # # /odom 消息；这里的门限比较的是 TF 链转换后的 odom->base_footprint
                    # # 预测位姿。超过该修正量时拒绝 NDT 校正，短时间用该 odom 预测保持平滑。
                    # 'max_scan_matching_correction_translation': 0.80,
                    # 'max_scan_matching_correction_yaw': 0.45,
                    # # 抖动时 Fast-LIO/odom 预测可能短时跳变；如果 NDT 本身非常可信，
                    # # 允许地图匹配校正覆盖 jump gate，避免持续拒绝好校正。
                    # 'scan_matching_jump_override_max_fitness_score': 0.12,
                    # 'scan_matching_jump_override_min_inlier_fraction': 0.90,
                    # 'publish_odom_prediction_on_rejection': True,
                    # # odom 预测只用于短时间补帧；连续拒绝后冻结在最后可信地图位姿，
                    # # 防止定位失锁时继续把机器人发布到墙里。
                    # 'max_odom_prediction_rejections': 3,
                    # # 点云可能先于同时间戳的 Fast-LIO TF 到达，短等一下避免运动时偶发丢帧。
                    # 'pointcloud_transform_timeout_sec': 0.15,
                    # # --- 初始位姿 ---
                    # 'cool_time_duration': 2.0,    # 初始收敛冷却时间(秒)
                    # # 不在启动时猜测位姿，避免机器人先出现在错误房间。
                    # # 启动后通过 RViz /initialpose 或 /relocalize 初始化。
                    # 'specify_init_pose': False,
                    # 'init_pos_x': 0.0,
                    # 'init_pos_y': 0.0,
                    # 'init_pos_z': 0.0,
                    # 'init_ori_w': 1.0,
                    # 'init_ori_x': 0.0,
                    # 'init_ori_y': 0.0,
                    # 'init_ori_z': 0.0,
                    # 'use_global_localization': True,
                    # # 启动后等 globalmap 和首帧扫描都准备好，自动调用 3D 全局重定位。
                    # 'auto_relocalize_on_start': True,
                    # # 连续 NDT 拒绝说明 map->odom 已经失去地图约束，触发 3D 全局重定位恢复。
                    # # 手动 /relocalize 仍可立即覆盖；启动阶段手动 /initialpose 不会被启动自动定位覆盖。
                    # 'auto_relocalize_after_rejections': 4,
                    # 'global_localization_pose_z_offset': 0.0,
                    # 'validate_global_localization_with_scan_matching': True,
                    # 'global_localization_max_fitness_score': 0.12,
                    # 'global_localization_max_candidates': 10,
                    # 'global_localization_min_fitness_margin': 0.10,
                    # # 如果候选之间仍有明显歧义，只允许 NDT 绝对误差足够低的结果进入
                    # # 连续一致性队列。2026-05-20 两次启动中正确簇约 0.033~0.039，
                    # # 错误歧义簇约 0.073~0.091。
                    # 'global_localization_ambiguous_max_fitness_score': 0.045,
                    # # 连续拒绝后的自动恢复不做“全图随便跳”，只接受离最后可信位姿
                    # # 足够近的候选；手动 /relocalize 仍保留全局搜索。
                    # 'global_localization_recovery_prior_max_xy': 2.0,
                    # 'global_localization_recovery_prior_max_yaw': 1.2,
                    # # A/B 房间结构相似时，单次 FPFH/RANSAC+NDT 结果不够可信。
                    # # 启动阶段必须多次落在同一个位姿簇内，才真正发布 map->odom。
                    # 'global_localization_required_consistent_results': 3,
                    # 'global_localization_consistency_window': 5,
                    # 'global_localization_consistency_xy_tolerance': 0.8,
                    # 'global_localization_consistency_yaw_tolerance': 0.35,
                    # # 使用启动后多帧局部点云作为 query，增加门框/墙角/天花板等判别特征。
                    # 'global_localization_query_accumulation_frames': 8,
                    # 'global_localization_query_min_accumulation_frames': 3,
                    # # 发布后前几帧必须能被 NDT 跟踪；否则撤销该全局定位并继续自动重试。
                    # 'global_localization_post_accept_validation_frames': 5,
                    # 'global_localization_post_accept_max_rejections': 1,
                    # # hall.yaml: origin=(-6.675,-12.819), size=(33.0m,39.05m)。
                    # # 使用完整栅格边界；候选位姿仍会先经过 NDT 和连续一致性验证。
                    # 'global_localization_enforce_xy_bounds': True,
                    # 'global_localization_min_x': -6.675,
                    # 'global_localization_max_x': 26.325,
                    # 'global_localization_min_y': -12.819,
                    # 'global_localization_max_y': 26.231,
                    # 'force_2d_pose': True,
    #                 'force_2d_fixed_z': True,
    #                 'global_localization_use_height_filter': True,
    #                 'global_localization_min_z': 0.0,
    #                 'global_localization_max_z': 0.0,
    #                 'global_localization_use_max_z_filter': False,
    #                 'global_localization_query_timeout_sec': 60.0,
    #             }]
    #         )
    #     ],
    #     output='screen',
    # )

    # =========================================================================
    # 第四部分：辅助节点（可选功能）
    # =========================================================================
    
    # 定期清除点云发布器（可选）
    # 功能：定期发布覆盖整个视野的点云，强制Nav2触发射线追踪清除障碍物
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
    # 第五部分：Nav2导航栈（最后启动，依赖定位和地图、分步避峰启动策略）
    # =========================================================================
    
    # 5.1 等定位 TF ready 后再启动 Nav2。
    # 自动全局重定位或手动 /initialpose 任一方式发布 map->odom 后，
    # map->base_footprint 可查，wait_for_tf 退出，下面的事件处理器再启动 Nav2。
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
        DeclareLaunchArgument('fusion_mode', default_value='false',
                              description='Enable NDT+odometry fusion mode: NDT drift -> freeze map->odom, robot keeps moving on odometry'),

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

        # ┌─ 方案C-recovery：启动和运行期全局重定位，不发布 map->odom TF ─┐
        TimerAction(period=4.0, actions=[hdl_bootstrap_global_localization_node]),
        TimerAction(period=5.0, actions=[hdl_bootstrap_container]),
        TimerAction(period=6.0, actions=[hdl_bootstrap_to_initialpose_node]),
        # └──────────────────────────────────────────────────────────────┘

        # ┌─ 方案A：单分辨率 NDT 定位，等待 C recovery 发布 /initialpose ─┐
        TimerAction(period=5.0, actions=[ndt_localization_node]),
        TimerAction(period=7.0, actions=[ndt_lifecycle_manager]),
        # └──────────────────────────────────────────────┘

        # ┌─ NDT定位 + 里程计融合节点 ───────────────────────────────────┐
        # 在 NDT 定位和 TF 树就绪后启动（period=8.0）
        # HEALTHY 时不干预，NDT 漂移时冻结 map->odom 让 odom 传播位姿
        TimerAction(period=8.0, actions=[localization_odom_fusion_node]),
        # └──────────────────────────────────────────────────────────────┘

        # ┌─ 方案C：hdl_localization UKF+NDT (Humble移植, ★当前使用★) ─┐
        #   hdl组件容器自带生命周期, 无需TimerAction和lifecycle_manager
        # hdl_global_localization_node,  # hdl 全局重定位服务 (/relocalize 使用)
        # imu_transformer_node,  # IMU坐标转换 (先启动)
        # hdl_container,         # hdl定位容器 (GlobalmapServer + HdlLocalization)
        # └─────────────────────────────────────────────────────────┘

        # 机器人实时位姿发布器（延迟7.5秒，等 TF 树完整）
        TimerAction(period=7.5, actions=[robot_realpose_publisher]),

        # ========== 第四部分：辅助节点（可选） ==========
        # 定期清除点云发布器（2秒）
        periodic_clearing_node, 
        #protected_clearing_3d_node,
        periodic_clearing_3d_node,

        # ========== 第五部分：导航层（定位 ready 后启动） ==========
        localization_ready_gate,
        start_nav2_after_localization,
    ])
