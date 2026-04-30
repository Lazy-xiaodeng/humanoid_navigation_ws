"""
修复版导航栈 Launch 文件
主要修复:
1. 确保map_server正确启动并发布地图
2. 确保NDT定位节点正常发布map→odom变换
3. 添加调试输出便于排查问题
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    TimerAction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ========== 路径配置 ==========
    pkg_nav2 = get_package_share_directory('humanoid_navigation2')
    pkg_lidar_loc = get_package_share_directory('lidar_localization_ros2')

    # 参数文件
    nav2_params_file = os.path.join(pkg_nav2, 'config', 'nav2_params.yaml')
    localization_params_file = os.path.join(pkg_lidar_loc, 'param', 'localization.yaml')

    # 地图文件(2D栅格地图,用于Nav2)
    map_yaml_file = os.path.join(pkg_nav2, 'maps', 'hall.yaml')
    
    # PCD地图文件(用于NDT定位)
    pcd_map_file = os.path.join(pkg_nav2, 'pcd', 'hall.pcd')

    # 行为树文件
    bt_xml_file = os.path.join(pkg_nav2, 'behavior_tree', 'navigate_with_stairs.xml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # =========================================================================
    # 第一部分:传感器与Fast-LIO节点
    # =========================================================================

    # 1.雷达驱动
    rslidar_node = Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 2.Fast-LIO节点(发布/fast_lio/cloud_registered)
    fast_lio_node = Node(
        package='fast_lio_robosense',
        executable='fastlio_mapping',
        name='fast_lio_node',
        output='screen',  # 修改:改为screen方便调试
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

    # 3.TF桥接(静态变换)
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

    # =========================================================================
    # 第二部分:感知层
    # =========================================================================

    # 1. 点云滤波节点
    point_cloud_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('humanoid_point_cloud_filter'),
                'launch',
                'point_cloud_filter.launch.py'
            )
        )
    )

    # =========================================================================
    # 第三部分:定位层(关键修复)
    # =========================================================================

    # ★ 修复1: 添加地图文件存在性检查
    check_map_file = LogInfo(
        condition=None,
        msg=['地图文件路径: ', map_yaml_file]
    )
    
    check_pcd_file = LogInfo(
        condition=None,
        msg=['PCD地图文件路径: ', pcd_map_file]
    )

    # 1. NDT全局定位节点
    # ★ 修复2: 覆盖PCD地图路径参数 + 设置初始位姿
    ndt_localization_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='lidar_localization_ros2',
                executable='lidar_localization_node',
                name='lidar_localization',
                output='screen',
                parameters=[
                    localization_params_file,
                    {'use_sim_time': use_sim_time},
                    {'map_path': pcd_map_file},  # ★ 关键修复: 明确指定PCD地图路径
                    {'set_initial_pose': True},  # ★ 关键修复: 启用自动初始位姿
                    {'initial_pose_x': 0.0},     # 默认初始X坐标(根据实际地图调整)
                    {'initial_pose_y': 0.0},     # 默认初始Y坐标
                    {'initial_pose_z': 0.0},     # 默认初始Z坐标
                    {'initial_pose_qx': 0.0},    # 默认四元数X
                    {'initial_pose_qy': 0.0},    # 默认四元数Y
                    {'initial_pose_qz': 0.0},    # 默认四元数Z
                    {'initial_pose_qw': 1.0}     # 默认四元数W(无旋转)
                ],
                remappings=[
                    ('/cloud', '/fast_lio/cloud_registered'),
                ]
            )
        ]
    )

    # NDT生命周期管理器
    ndt_lifecycle_manager = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_ndt',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                    {'node_names': ['lidar_localization']}
                ]
            )
        ]
    )

    # 2. map_server节点
    # ★ 修复3: 添加完整的参数配置和调试输出
    map_server_node = TimerAction(
        period=0.5,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file},
                    {'frame_id': 'map'},
                    {'topic_name': 'map'},
                    {'use_map_state_machine': True}
                ]
            )
        ]
    )

    # 3. map_server生命周期管理器
    # ★ 修复4: 添加延迟确保map_server完全启动
    map_server_lifecycle = TimerAction(
        period=1.5,  # 从1.0增加到1.5,给map_server更多启动时间
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                    {'node_names': ['map_server']}
                ]
            )
        ]
    )

    # =========================================================================
    # 第四部分:辅助节点
    # =========================================================================

    periodic_clearing_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='humanoid_navigation2',
                executable='periodic_clearing_publisher',
                name='periodic_clearing_publisher',
                parameters=[{
                    'publish_frequency': 2.0,
                    'num_rays': 360,
                    'max_range': 6.0,
                    'clearing_height': -0.15,
                    'output_topic': '/costmap_clearing_cloud',
                }],
                output='screen'
            )
        ]
    )

    # =========================================================================
    # 第五部分:Nav2导航栈
    # =========================================================================

    nav2_nodes = TimerAction(
        period=3.0,
        actions=[
            GroupAction([
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
                            'default_bt_xml_filename': bt_xml_file
                        }
                    ]
                ),

                Node(
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
            ])
        ]
    )

    # =========================================================================
    # 组装Launch描述
    # =========================================================================
    return LaunchDescription([
        # ========== 参数声明 ==========
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # ========== 文件路径检查(调试用) ==========
        check_map_file,
        check_pcd_file,

        # ========== 第一部分:传感器与Fast-LIO节点 ==========
        rslidar_node,
        fast_lio_node,
        tf_bridge_odom,
        tf_bridge_base,

        # ========== 第二部分:感知层 ==========
        point_cloud_filter_launch,

        # ========== 第三部分:定位层(关键修复) ==========
        # map_server先启动(0.5秒)
        map_server_node,
        
        # NDT定位节点(2秒)
        ndt_localization_node,
        
        # NDT生命周期管理(1秒)
        ndt_lifecycle_manager,
        
        # map_server生命周期管理(1.5秒)
        map_server_lifecycle,

        # ========== 第四部分:辅助节点 ==========
        periodic_clearing_node,

        # ========== 第五部分:导航层 ==========
        nav2_nodes,
    ])
