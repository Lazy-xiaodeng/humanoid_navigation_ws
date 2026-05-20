"""
完整导航栈 Launch 文件（包含感知+定位+导航）

系统架构：
1. 感知层：点云滤波、高程图（可选）、地形分析（可选）
2. 定位层：定位节点（发布map→odom TF）+ map_server（发布2D栅格地图）
   支持三种定位方案（三选一，注释切换）：
     方案A - lidar_localization_ros2 (单分辨率NDT, 旧方案)
     方案B - humanoid_global_localization (多分辨率NDT网格搜索)
     方案C - hdl_localization (UKF+NDT_OMP, Humble移植, ★当前使用★)
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
    # --- 方案A: 单分辨率 NDT 定位 (旧方案) ---
    localization_params_file = os.path.join(pkg_lidar_loc, 'param', 'localization.yaml')
    
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

    # map -> map_ground 的静态 TF
    tf_map_to_ground = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_ground',
        arguments=[
            '0.0', '0.0', '-1.215',    # ★ Z 轴向下掉 1.215 米 (距离依你底盘到雷达的实际高度微调)
            '0.0', '0.0', '0.0', '1.0',
            'map', 'map_ground'        # 父节点是 map，子节点是预留的 map_ground
        ]
    )

    # odom -> odom_ground 的静态 TF
    tf_odom_to_ground = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_ground',
        arguments=[
            '0.0', '0.0', '-1.215',    # ★ Z 轴向下掉 1.215 米
            '0.0', '0.0', '0.0', '1.0',
            'odom', 'odom_ground'      # 父节点是 odom，子节点是 odom_ground
        ]
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


    # 5. 机器人实时位姿发布器（从 TF 读取 map->base_footprint）
    #    与 /pcl_pose 不同：/pcl_pose 发布的是 map->odom 偏移量（通常 0.1-0.5m），
    #    本节点通过完整 TF 链计算机器人在地图中的实际位姿，发布到 /robot_realpose
    robot_realpose_publisher = nav2_python_node(
        'robot_realpose_publisher',
        'robot_realpose_publisher'
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
        parameters=[localization_params_file, {'use_sim_time': use_sim_time}],
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

        # ┌─ 方案A：单分辨率 NDT 定位 (旧方案, 已注释) ─┐
        TimerAction(period=5.0, actions=[ndt_localization_node]),
        TimerAction(period=7.0, actions=[ndt_lifecycle_manager]),
        # └──────────────────────────────────────────────┘

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
