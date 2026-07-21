"""
RoboSense lidar_localization 导航启动文件。

数据流：
  rslidar_sdk -> Fast-LIO -> humanoid_robosense_localization_runtime
      -> /prior_localization/robosense_odom
      -> prior_map_odom_bridge -> map->odom
      -> Nav2

说明：
  - RoboSense 负责局部精匹配，全局重定位只在可信度监督层请求时运行。
  - 恢复协调器审核全局候选后同步更新 bridge 与 RoboSense 初值。
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


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('humanoid_navigation2')
    pkg_obstacle_runtime = get_package_share_directory('humanoid_obstacle_runtime')
    pkg_localization_runtime = get_package_share_directory(
        'humanoid_localization_runtime'
    )
    pkg_global_relocalization = get_package_share_directory(
        'humanoid_global_relocalization_runtime'
    )

    default_nav2_params_file = os.path.join(pkg_nav2, 'config', 'nav2_params.yaml')
    default_bt_xml_file = os.path.join(pkg_nav2, 'behavior_tree', 'navigate_xy_then_yaw.xml')
    default_through_bt_xml_file = os.path.join(pkg_nav2, 'behavior_tree', 'navigate_through_poses_no_backup.xml')
    default_roi_obstacle_params_file = os.path.join(pkg_obstacle_runtime, 'config', 'obstacle_runtime.yaml')
    default_robosense_config_file = (
        '/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_robosense_localization_runtime/config/'
        'robosense_lidar_localization.yaml'
    )
    default_global_relocalization_config_file = os.path.join(
        pkg_global_relocalization, 'config', 'relocalization_runtime.yaml'
    )
    default_localization_runtime_config_file = os.path.join(
        pkg_localization_runtime, 'config', 'localization_runtime.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=default_nav2_params_file)
    bt_xml_file = LaunchConfiguration('bt_xml_file', default=default_bt_xml_file)
    through_bt_xml_file = LaunchConfiguration('through_bt_xml_file', default=default_through_bt_xml_file)
    enable_fastdds_shm = LaunchConfiguration('enable_fastdds_shm', default='true')
    enable_rslidar = LaunchConfiguration('enable_rslidar', default='true')
    enable_periodic_clearing = LaunchConfiguration('enable_periodic_clearing', default='true')
    enable_roi_obstacle_detector = LaunchConfiguration('enable_roi_obstacle_detector', default='true')
    enable_global_relocalization = LaunchConfiguration(
        'enable_global_relocalization', default='true'
    )
    global_relocalization_config_file = LaunchConfiguration(
        'global_relocalization_config_file',
        default=default_global_relocalization_config_file,
    )
    localization_runtime_config_file = LaunchConfiguration(
        'localization_runtime_config_file',
        default=default_localization_runtime_config_file,
    )
    roi_obstacle_params_file = LaunchConfiguration(
        'roi_obstacle_params_file',
        default=default_roi_obstacle_params_file,
    )

    robosense_config_file = LaunchConfiguration(
        'robosense_config_file',
        default=default_robosense_config_file,
    )
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

    def localization_node(executable, node_name, condition=None):
        parameters = [
            # 节点自身参数统一由定位运行层 YAML 管理。
            localization_runtime_config_file,
            # 时间源由启动场景决定，保留为唯一 launch 覆盖项。
            {'use_sim_time': use_sim_time},
        ]
        return Node(
            package='humanoid_localization_runtime',
            executable=executable,
            name=node_name,
            output='screen',
            condition=condition,
            parameters=parameters,
        )

    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        condition=IfCondition(enable_rslidar),
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
                # 静态 TF 也跟随系统时间源。
                'use_sim_time': use_sim_time,
            }
        ],
    )

    tf_map_to_ground = Node(
        package='humanoid_localization_runtime',
        executable='dynamic_odom_ground_publisher',
        name='dynamic_map_ground_publisher',
        output='screen',
        parameters=[
            localization_runtime_config_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    tf_odom_to_ground = Node(
        package='humanoid_localization_runtime',
        executable='dynamic_odom_ground_publisher',
        name='dynamic_odom_ground_publisher',
        output='screen',
        parameters=[
            localization_runtime_config_file,
            {'use_sim_time': use_sim_time},
        ],
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
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    roi_obstacle_detector_cpp_node = Node(
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

    robosense_lidar_localization_node = Node(
        package='humanoid_robosense_localization_runtime',
        executable='robosense_lidar_localization_node',
        name='robosense_lidar_localization_node',
        output='screen',
        parameters=[
            {
                # 时间源。
                'use_sim_time': use_sim_time,
            },
            {
                # RoboSense 定位 YAML，配置地图、话题、外参、初值和匹配阈值。
                'config_file': robosense_config_file,
            },
        ],
    )

    # 唯一 map->odom 发布者。所有定位门槛和话题均读取定位运行层 YAML。
    prior_map_odom_bridge_cpp_node = localization_node(
        'prior_map_odom_bridge_cpp',
        'prior_map_odom_bridge',
    )

    localization_trust_supervisor_node = Node(
        package='humanoid_navigation2',
        executable='localization_trust_supervisor',
        name='localization_trust_supervisor',
        output='screen',
        parameters=[localization_runtime_config_file, {'use_sim_time': use_sim_time}],
    )

    global_relocalization_node = Node(
        package='humanoid_global_relocalization_runtime',
        executable='global_relocalization_node',
        name='global_relocalization_node',
        output='screen',
        condition=IfCondition(enable_global_relocalization),
        parameters=[{
            'config_file': global_relocalization_config_file,
            'use_sim_time': use_sim_time,
        }],
    )

    global_relocalization_coordinator_node = Node(
        package='humanoid_localization_runtime',
        executable='global_relocalization_coordinator',
        name='global_relocalization_coordinator',
        output='screen',
        condition=IfCondition(enable_global_relocalization),
        parameters=[localization_runtime_config_file, {'use_sim_time': use_sim_time}],
    )

    rviz_initialpose_adapter_node = localization_node(
        'rviz_initialpose_adapter',
        'rviz_initialpose_adapter',
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
        package='humanoid_navigation2',
        executable='wait_for_localization_trust',
        name='wait_for_localization_trust',
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
        DeclareLaunchArgument('map_yaml_file', default_value=os.path.join(pkg_nav2, 'maps', 'hall.yaml'), description='Nav2 map_server 加载的 2D 栅格地图 YAML'),
        DeclareLaunchArgument('robosense_config_file', default_value=default_robosense_config_file, description='RoboSense 定位配置 YAML'),
        DeclareLaunchArgument('enable_fastdds_shm', default_value='true', description='是否设置 FastDDS 共享内存环境变量'),
        DeclareLaunchArgument('enable_rslidar', default_value='true', description='是否启动真实 RoboSense 雷达驱动；bag 回放验证时可设为 false'),
        DeclareLaunchArgument('enable_global_relocalization', default_value='true', description='是否启动按需全局重定位闭环'),
        DeclareLaunchArgument('localization_runtime_config_file', default_value=default_localization_runtime_config_file, description='定位桥、可信度监督和恢复协调器统一参数 YAML'),
        DeclareLaunchArgument('global_relocalization_config_file', default_value=default_global_relocalization_config_file, description='全局重定位正式参数文件'),
        *fastdds_env_setup,
        rslidar_node,
        fast_lio_node,
        tf_bridge_odom,
        tf_map_to_ground,
        tf_odom_to_ground,
        tf_bridge_base,
        tf_bridge_clearing_lidar,
        point_cloud_filter_launch,
        roi_obstacle_detector_cpp_node,
        map_server_node,
        map_server_lifecycle,
        TimerAction(period=4.5, actions=[robosense_lidar_localization_node]),
        TimerAction(period=5.5, actions=[prior_map_odom_bridge_cpp_node]),
        TimerAction(period=5.7, actions=[localization_trust_supervisor_node]),
        TimerAction(period=5.8, actions=[global_relocalization_node]),
        TimerAction(period=6.0, actions=[global_relocalization_coordinator_node]),
        TimerAction(period=6.1, actions=[rviz_initialpose_adapter_node]),
        TimerAction(period=7.5, actions=[robot_realpose_publisher]),
        periodic_clearing_3d_node,
        localization_ready_gate,
        start_nav2_after_localization,
    ])
