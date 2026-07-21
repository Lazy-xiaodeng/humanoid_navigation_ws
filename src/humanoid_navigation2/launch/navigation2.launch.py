"""
Open3D prior-map 定位导航启动文件。

数据流：
  rslidar_sdk -> Fast-LIO -> fastlio_open3d_axis_adapter
      -> humanoid_prior_localization_runtime
      -> /prior_localization/odom
      -> prior_map_odom_bridge -> map->odom
      -> Nav2

说明：
  - 本文件保留现有 Open3D 定位完整链路。
  - 旧 NDT、HDL、ScanContext 等未使用节点已从本 launch 中移除。
  - map->odom 只由 prior_map_odom_bridge 发布，避免 TF 冲突。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('humanoid_navigation2')
    pkg_obstacle_runtime = get_package_share_directory('humanoid_obstacle_runtime')
    pkg_localization_runtime = get_package_share_directory('humanoid_localization_runtime')

    default_nav2_params_file = os.path.join(pkg_nav2, 'config', 'nav2_params.yaml')
    default_bt_xml_file = os.path.join(pkg_nav2, 'behavior_tree', 'navigate_xy_then_yaw.xml')
    default_through_bt_xml_file = os.path.join(pkg_nav2, 'behavior_tree', 'navigate_through_poses_no_backup.xml')
    default_prior_map_path = os.path.join(pkg_nav2, 'pcd', 'hall_open3d_grounded.pcd')
    default_roi_obstacle_params_file = os.path.join(pkg_obstacle_runtime, 'config', 'obstacle_runtime.yaml')
    default_localization_runtime_config_file = os.path.join(
        pkg_localization_runtime, 'config', 'localization_runtime.yaml')
    default_open3d_localization_config_file = os.path.join(
        pkg_localization_runtime, 'config', 'localization_runtime_open3d.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=default_nav2_params_file)
    bt_xml_file = LaunchConfiguration('bt_xml_file', default=default_bt_xml_file)
    through_bt_xml_file = LaunchConfiguration('through_bt_xml_file', default=default_through_bt_xml_file)
    enable_fastdds_shm = LaunchConfiguration('enable_fastdds_shm', default='true')
    enable_periodic_clearing = LaunchConfiguration('enable_periodic_clearing', default='true')
    enable_prior_map_localization = LaunchConfiguration('enable_prior_map_localization', default='true')
    enable_roi_obstacle_detector = LaunchConfiguration('enable_roi_obstacle_detector', default='true')
    roi_obstacle_params_file = LaunchConfiguration(
        'roi_obstacle_params_file',
        default=default_roi_obstacle_params_file,
    )
    localization_runtime_config_file = LaunchConfiguration(
        'localization_runtime_config_file',
        default=default_localization_runtime_config_file,
    )
    open3d_localization_config_file = LaunchConfiguration(
        'open3d_localization_config_file',
        default=default_open3d_localization_config_file,
    )
    prior_map_path = LaunchConfiguration('prior_map_path', default=default_prior_map_path)
    map_yaml_file = LaunchConfiguration(
        'map_yaml_file',
        default=os.path.join(pkg_nav2, 'maps', 'hall.yaml'),
    )

    fastdds_config_file = PathJoinSubstitution([
        os.path.expanduser('~'),
        '.config',
        'fastdds_shm.xml',
    ])

    fastdds_env_setup = [
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            'rmw_fastrtps_cpp',
            condition=IfCondition(enable_fastdds_shm),
        ),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            fastdds_config_file,
            condition=IfCondition(enable_fastdds_shm),
        ),
        SetEnvironmentVariable(
            'RMW_FASTRTPS_USE_QOS_FROM_XML',
            '1',
            condition=IfCondition(enable_fastdds_shm),
        ),
    ]

    def localization_node(executable, node_name):
        parameters = [
            localization_runtime_config_file,
            open3d_localization_config_file,
            {'use_sim_time': use_sim_time},
        ]
        return Node(
            package='humanoid_localization_runtime',
            executable=executable,
            name=node_name,
            output='screen',
            parameters=parameters,
        )

    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[
            {
                # 让雷达驱动时间戳跟随系统时间或 /clock。
                'use_sim_time': use_sim_time,
            }
        ],
    )

    fast_lio_node = Node(
        package='fast_lio_robosense',
        executable='fastlio_mapping',
        name='fast_lio_node',
        output='log',
        parameters=[
            # Fast-LIO 主参数：雷达类型、滤波、建图、话题等。
            os.path.join(get_package_share_directory('fast_lio_robosense'), 'config', 'robosenseAiry.yaml'),
            {
                # 与整套系统时间源保持一致。
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            # RoboSense SDK 输出点云。
            ('/CloudPoints', '/airy_points'),
            # RoboSense SDK 输出 IMU。
            ('/Imu', '/airy_imu'),
            # Fast-LIO 里程计统一发布到导航使用的 /odom。
            ('/Odometry', '/odom'),
            # Fast-LIO 注册点云统一发布到 /fast_lio/cloud_registered。
            ('/cloud_registered', '/fast_lio/cloud_registered'),
        ],
    )

    tf_bridge_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_camera_init',
        arguments=[
            # Fast-LIO raw odom 到 camera_init 的固定轴变换。
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '-0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5',
            '--frame-id', 'odom', '--child-frame-id', 'camera_init',
        ],
        parameters=[
            {
                # 时间源。
                'use_sim_time': use_sim_time,
            }
        ],
    )

    tf_map_to_ground = Node(
        package='humanoid_localization_runtime',
        executable='dynamic_odom_ground_publisher',
        name='dynamic_map_ground_publisher',
        output='screen',
        parameters=[localization_runtime_config_file, {'use_sim_time': use_sim_time}],
    )

    tf_odom_to_ground = Node(
        package='humanoid_localization_runtime',
        executable='dynamic_odom_ground_publisher',
        name='dynamic_odom_ground_publisher',
        output='screen',
        parameters=[localization_runtime_config_file, {'use_sim_time': use_sim_time}],
    )

    tf_bridge_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_body_to_base_footprint',
        arguments=[
            # Fast-LIO body/raw 轴到导航 base_footprint 的外参。
            '--x', '0.004', '--y', '1.215', '--z', '0.072',
            '--qx', '0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
            '--frame-id', 'body', '--child-frame-id', 'base_footprint',
        ],
        parameters=[
            {
                # 时间源。
                'use_sim_time': use_sim_time,
            }
        ],
    )

    tf_bridge_clearing_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_footprint_to_clearing_lidar',
        arguments=[
            # 清障点云射线追踪使用的虚拟雷达高度。
            '--x', '0', '--y', '0', '--z', '1.215',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_footprint', '--child-frame-id', 'clearing_lidar',
        ],
        parameters=[
            {
                # 时间源。
                'use_sim_time': use_sim_time,
            }
        ],
    )

    point_cloud_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('humanoid_point_cloud_filter'),
                'launch',
                'point_cloud_filter.launch.py',
            )
        )
    )

    roi_obstacle_detector_node = Node(
        package='humanoid_obstacle_runtime',
        executable='roi_obstacle_detector_cpp',
        name='roi_obstacle_detector',
        output='screen',
        condition=IfCondition(enable_roi_obstacle_detector),
        parameters=[
            roi_obstacle_params_file,
            {
                # 跟随导航系统时间源；实机 false，bag/仿真 true。
                'use_sim_time': use_sim_time,
            },
        ],
    )

    map_server_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[
                    nav2_params_file,
                    {
                        'use_sim_time': use_sim_time,
                        'yaml_filename': map_yaml_file,
                    },
                ],
            )
        ],
    )

    map_server_lifecycle = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                parameters=[
                    nav2_params_file,
                    {'use_sim_time': use_sim_time},
                ],
            )
        ],
    )

    robot_realpose_publisher = localization_node(
        'robot_realpose_publisher',
        'robot_realpose_publisher',
    )

    fastlio_open3d_axis_adapter_node = Node(
        package='humanoid_prior_localization_runtime',
        executable='fastlio_open3d_axis_adapter',
        name='fastlio_open3d_axis_adapter',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        condition=IfCondition(enable_prior_map_localization),
        parameters=[
            {
                # 时间源。
                'use_sim_time': use_sim_time,
            },
            {
                # Fast-LIO 原始里程计输入。
                'raw_odom_topic': '/odom',
                # Fast-LIO 原始注册点云输入。
                'raw_cloud_topic': '/fast_lio/cloud_registered',
                # 转成标准轴后的 odom 输出，供先验地图定位节点和 bridge 使用。
                'output_odom_topic': '/prior_localization/open3d_input_odom',
                # 转成标准轴后的点云输出，供先验地图定位节点使用。
                'output_cloud_topic': '/prior_localization/open3d_input_cloud',
                # 输出 odom 的父坐标系。
                'odom_frame': 'odom',
                # 输出标准轴虚拟 base，用于隔离 raw body 坐标。
                'output_base_frame': 'prior_open3d_base',
                # 发布 odom->prior_open3d_base TF，bridge 可作为 fallback 查询。
                'publish_tf': True,
                # raw body 到 base_footprint 的平移，用于对齐 grounded 地图原点。
                'initial_body_to_base_translation_raw': [0.004, 1.215, 0.072],
            },
        ],
    )

    prior_map_localization_node = Node(
        package='humanoid_prior_localization_runtime',
        executable='global_localization_node',
        name='prior_map_open3d_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        condition=IfCondition(enable_prior_map_localization),
        parameters=[
            # Open3D 定位包自带默认参数。
            PathJoinSubstitution([
                FindPackageShare('humanoid_prior_localization_runtime'),
                'config',
                'loc_param_g1.yaml',
            ]),
            {
                # 时间源。
                'use_sim_time': use_sim_time,
                # Open3D 使用的先验 PCD 地图，必须是标准轴 grounded 地图。
                'path_map': prior_map_path,
                # 初始位姿，格式 x/y/z/roll/pitch/yaw。
                'initialpose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                # Kalman x 方向过程/测量噪声参数。
                'kf_baselink2map/x': [0.001, 0.002],
                # Kalman y 方向过程/测量噪声参数。
                'kf_baselink2map/y': [0.001, 0.005],
                # Kalman z 方向过程/测量噪声参数。
                'kf_baselink2map/z': [0.00001, 0.04],
                # 禁止先验地图定位节点发布 TF，map->odom 只由 bridge 发布。
                'publish_tf': False,
                # 输入点云队列长度。
                'pcd_queue_maxsize': 10,
                # 粗匹配体素尺寸，单位 m。
                'voxelsize_coarse': 0.01,
                # 细匹配体素尺寸，单位 m。
                'voxelsize_fine': 0.20,
                # 跟踪阶段最低 fitness。
                'threshold_fitness': 0.50,
                # 初始化阶段最低 fitness。
                'threshold_fitness_init': 0.50,
                # 定位更新间隔，单位秒；原参数名保留上游拼写。
                'loc_frequence': 1.0,
                # 是否保存 scan 调试文件。
                'save_scan': False,
                # 是否使用 hidden point removal。
                'hidden_removal': False,
                # 输入 scan 最大点数。
                'maxpoints_source': 60000,
                # 地图目标点最大点数。
                'maxpoints_target': 250000,
                # 是否对 odom2map 做内部滤波；当前交给 bridge 处理。
                'filter_odom2map': False,
                # 直接使用适配后的输入 odom 作为初值。
                'use_input_odom_pose_directly': True,
                # Open3D 内部 Kalman 过程噪声。
                'kalman_processVar2': 0.001,
                # Open3D 内部 Kalman 测量噪声。
                'kalman_estimatedMeasVar2': 0.02,
                # Open3D 内部定位置信度阈值。
                'confidence_loc_th': 0.7,
                # 触发局部地图更新的距离阈值，单位 m。
                'dis_updatemap': 3.5,
            },
        ],
        remappings=[
            # 输入：适配后的标准轴 odom。
            ('/Odometry_loc', '/prior_localization/open3d_input_odom'),
            # 输入：适配后的标准轴点云。
            ('/cloud_registered_1', '/prior_localization/open3d_input_cloud'),
            # 输出：Open3D 全局定位候选。
            ('/baselink2map', '/prior_localization/odom'),
            # 输出：Open3D 置信度。
            ('/localization_3d_confidence', '/prior_localization/confidence'),
            # 输出：调试 pose。
            ('/localization_3d', '/prior_localization/pose_motion_link'),
            # 以下为 Open3D 调试/可视化输出，统一挂到 prior_localization 命名空间。
            ('/odom2map', '/prior_localization/open3d_odom2map'),
            ('/odom2map_kalman', '/prior_localization/open3d_odom2map_kalman'),
            ('/baselink2map_kalman', '/prior_localization/open3d_baselink2map_kalman'),
            ('/motionlink2map', '/prior_localization/open3d_motionlink2map'),
            ('/map', '/prior_localization/open3d_map'),
            ('/submap', '/prior_localization/open3d_submap'),
            ('/scan', '/prior_localization/open3d_scan'),
            ('/scan2map', '/prior_localization/open3d_scan2map'),
        ],
    )

    # 唯一 map->odom 发布者；Open3D 专属输入差异由覆盖 YAML 管理。
    prior_map_odom_bridge_node = localization_node(
        'prior_map_odom_bridge_cpp',
        'prior_map_odom_bridge',
    )

    periodic_clearing_3d_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='humanoid_costmap_runtime',
                executable='periodic_clearing_3d_publisher',
                name='periodic_clearing_3d_publisher',
                condition=IfCondition(enable_periodic_clearing),
                parameters=[
                    # Nav2 参数中包含 3D 清障点云层配置。
                    nav2_params_file,
                    {
                        # 时间源。
                        'use_sim_time': use_sim_time,
                    },
                ],
                output='screen',
            )
        ],
    )

    localization_ready_gate = Node(
        package='humanoid_localization_runtime',
        executable='wait_for_tf',
        name='wait_for_localization_tf',
        parameters=[localization_runtime_config_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    nav2_core_nodes = GroupAction([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                # planner/costmap 参数。
                nav2_params_file,
                {
                    # 时间源。
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                # controller/local costmap 参数。
                nav2_params_file,
                {
                    # 时间源。
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[
                # recovery/behavior 参数。
                nav2_params_file,
                {
                    # 时间源。
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                # bt_navigator 参数。
                nav2_params_file,
                {
                    # 时间源。
                    'use_sim_time': use_sim_time,
                    # 单点导航行为树。
                    'default_nav_to_pose_bt_xml': bt_xml_file,
                    # 多点导航行为树。
                    'default_nav_through_poses_bt_xml': through_bt_xml_file,
                },
            ],
        ),
    ])

    nav2_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='是否使用 /clock；实机 false，bag/仿真 true'),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params_file, description='Nav2 参数文件'),
        DeclareLaunchArgument('bt_xml_file', default_value=default_bt_xml_file, description='Nav2 单点行为树 XML'),
        DeclareLaunchArgument('through_bt_xml_file', default_value=default_through_bt_xml_file, description='Nav2 through poses 行为树 XML'),
        DeclareLaunchArgument('enable_periodic_clearing', default_value='true', description='是否启动周期性清障节点'),
        DeclareLaunchArgument('enable_roi_obstacle_detector', default_value='true', description='是否启动前方 ROI 点云障碍检测节点'),
        DeclareLaunchArgument('roi_obstacle_params_file', default_value=default_roi_obstacle_params_file, description='ROI 障碍检测参数文件'),
        DeclareLaunchArgument('localization_runtime_config_file', default_value=default_localization_runtime_config_file, description='定位运行层通用参数 YAML'),
        DeclareLaunchArgument('open3d_localization_config_file', default_value=default_open3d_localization_config_file, description='Open3D 定位链路覆盖参数 YAML'),
        DeclareLaunchArgument('map_yaml_file', default_value=os.path.join(pkg_nav2, 'maps', 'hall.yaml'), description='Nav2 map_server 加载的 2D 栅格地图 YAML'),
        DeclareLaunchArgument('enable_prior_map_localization', default_value='true', description='是否启动 Open3D 定位节点'),
        DeclareLaunchArgument('prior_map_path', default_value=default_prior_map_path, description='Open3D 标准轴 grounded PCD 地图路径'),
        DeclareLaunchArgument('enable_fastdds_shm', default_value='true', description='是否设置 FastDDS 共享内存环境变量'),
        *fastdds_env_setup,
        rslidar_node,
        fast_lio_node,
        tf_bridge_odom,
        tf_map_to_ground,
        tf_odom_to_ground,
        tf_bridge_base,
        tf_bridge_clearing_lidar,
        point_cloud_filter_launch,
        roi_obstacle_detector_node,
        map_server_node,
        map_server_lifecycle,
        TimerAction(period=3.5, actions=[fastlio_open3d_axis_adapter_node]),
        TimerAction(period=4.5, actions=[prior_map_localization_node]),
        TimerAction(period=5.5, actions=[prior_map_odom_bridge_node]),
        TimerAction(period=7.5, actions=[robot_realpose_publisher]),
        periodic_clearing_3d_node,
        localization_ready_gate,
        start_nav2_after_localization,
    ])
