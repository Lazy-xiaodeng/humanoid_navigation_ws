"""
完整导航栈 Launch 文件（包含感知+定位+导航）

系统架构：
1. 感知层：点云滤波、高程图（可选）、地形分析（可选）
2. 定位层：NDT定位（发布map→odom TF）+ map_server（发布2D栅格地图）
3. 导航层：Nav2导航栈（规划、控制、行为树）

启动顺序：
0秒：感知层（点云滤波）
0.5秒：map_server（加载2D地图）
1秒：NDT定位节点（全局定位）
1秒：map_server生命周期管理
2秒：定期清除点云发布器（可选功能）
3秒：Nav2导航栈（规划、控制、行为树）

TF树：
map → odom → camera_init → body → base_footprint → base_link → 传感器
  ↑        ↑
NDT发布  Fast-LIO发布
(动态)   (动态)

使用方式：
ros2 launch humanoid_navigation2 navigation_stack.launch.py use_sim_time:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument, 
    GroupAction, 
    TimerAction, 
    IncludeLaunchDescription
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ========== 路径配置 ==========
    pkg_nav2 = get_package_share_directory('humanoid_navigation2')
    pkg_ndt = get_package_share_directory('ndt_localizer')
    
    # 参数文件
    nav2_params_file = os.path.join(pkg_nav2, 'config', 'nav2_params.yaml')
    ndt_params_file = os.path.join(pkg_ndt, 'config', 'ndt_localizer_params.yaml')
    
    # 地图文件（2D栅格地图，用于Nav2）
    map_yaml_file = os.path.join(pkg_nav2, 'maps', 'hall.yaml')
    
    # 行为树文件
    bt_xml_file = os.path.join(pkg_nav2, 'behavior_tree', 'navigate_with_stairs.xml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
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
    
    # 1. NDT全局定位节点
    # 功能：订阅Fast-LIO的点云 + 加载PCD地图 → 发布map→odom的TF变换
    # 延迟1秒启动，确保Fast-LIO已经发布点云数据
    ndt_localization_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ndt_localizer',
                executable='ndt_localizer_node.py',
                name='ndt_localizer',
                output='screen',
                parameters=[
                    ndt_params_file,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    # 2. map_server节点（发布2D栅格地图，供Nav2使用）
    # ★ 关键：NDT只发布TF变换，不发布栅格地图！
    # Nav2的costmap需要订阅/map话题，所以必须有map_server
    map_server_node = TimerAction(
        period=0.5,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}
                ]
            )
        ]
    )

    # 3.map_server生命周期管理器（自动激活map_server）
    map_server_lifecycle = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}
                ]
            )
        ]
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
                parameters=[{
                    'publish_frequency': 2.0,         # 2Hz，每0.5秒发布一次
                    'num_rays': 360,                  # 360条射线，覆盖全方向
                    'max_range': 6.0,                 # 最大距离6米，与costmap一致
                    'clearing_height': -0.15,         # 地面附近，不被过滤掉
                    'output_topic': '/costmap_clearing_cloud',
                }],
                output='screen'
            )
        ]
    )

    # =========================================================================
    # 第五部分：Nav2导航栈（最后启动，依赖定位和地图）
    # =========================================================================
    
    nav2_nodes = TimerAction(
        period=3.0,  # 延迟3秒，等待定位和地图稳定
        actions=[
            GroupAction([
                # 1. 全局路径规划服务器（A*/Dijkstra算法）
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
                ),

                # 2. 局部轨迹跟踪控制器（DWB/MPPI算法）
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
                ),

                # 3. 行为服务器（等待、旋转等行为）
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
                ),

                # 4. 行为树导航执行器（组合行为完成任务）
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

                # 5. Nav2生命周期管理器（统一启动和管理导航节点）
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'bond_timeout': 0.0},  # 关闭bond心跳检测
                        {'node_names': [
                            'planner_server',
                            'behavior_server',
                            'bt_navigator',
                            'global_costmap/global_costmap',
                            'local_costmap/local_costmap',
                            'controller_server'
                        ]}
                    ]
                )
            ])
        ]
    )

    # =========================================================================
    # 组装Launch描述（按启动顺序）
    # =========================================================================
    return LaunchDescription([
        # ========== 参数声明 ==========
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_elevation_map', default_value='false'),
        DeclareLaunchArgument('enable_terrain_analysis', default_value='false'),
        DeclareLaunchArgument('enable_periodic_clearing', default_value='true'),
        
        # ========== 第一部分：硬件层 传感器与Fast-LIO节点 ==========
        # 1. 传感器与Fast-LIO
        rslidar_node,
        fast_lio_node,

        # 2. TF桥接
        tf_bridge_odom,
        tf_bridge_base,

        # ========== 第二部分：感知层（最先启动） ==========
        # 点云滤波最先启动，处理原始点云数据
        point_cloud_filter_launch,
        
        # 高程图和地形分析（可选，根据参数条件启动）
        # 如果启用，延迟启动等待滤波稳定
        # TimerAction(period=1.0, actions=[elevation_map_launch]),  # 需要时取消注释
        # TimerAction(period=2.0, actions=[terrain_analyzer_launch]),  # 需要时取消注释

        # ========== 第三部分：定位层（延迟启动） ==========
        # map_server先启动（0.5秒），加载2D地图
        map_server_node,
        
        # NDT定位节点后启动（1秒），等待Fast-LIO点云数据
        ndt_localization_node,
        
        # map_server生命周期管理（1秒），自动激活
        map_server_lifecycle,

        # ========== 第四部分：辅助节点（可选） ==========
        # 定期清除点云发布器（2秒）
        periodic_clearing_node,  # 需要时取消注释

        # ========== 第五部分：导航层（最后启动） ==========
        # Nav2导航栈（3秒），等待定位和地图稳定
        nav2_nodes,
    ])
