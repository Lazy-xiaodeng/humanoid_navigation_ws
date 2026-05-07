"""
完整导航栈 Launch 文件（包含感知+定位+导航）

系统架构：
1. 感知层：点云滤波、高程图（可选）、地形分析（可选）
2. 定位层：定位节点（发布map→odom TF）+ map_server（发布2D栅格地图）
   支持三种定位方案（三选一，注释切换）：
     方案A - lidar_localization_ros2 (单分辨率NDT, 旧方案)
     方案B - humanoid_global_localization (多分辨率NDT网格搜索, ★当前使用★)
     方案C - hdl_localization (UKF+NDT_OMP, Humble移植)
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
    GroupAction,
    TimerAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable  # ★ 新增：用于设置 FastDDS 环境变量
)
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
    # pkg_lidar_loc = get_package_share_directory('lidar_localization_ros2')  # [方案A: 旧NDT]

    # 参数文件
    nav2_params_file = os.path.join(pkg_nav2, 'config', 'nav2_params_rpp.yaml')
    # --- 方案B: 多分辨率 NDT 全局定位 (当前使用) ---
    global_localization_params_file = os.path.join(pkg_global_loc, 'config', 'global_localization.yaml')
    # --- 方案A: 单分辨率 NDT 定位 (旧方案) ---
    # localization_params_file = os.path.join(pkg_lidar_loc, 'param', 'localization.yaml')
    # --- 方案C: hdl_localization UKF+NDT (Humble移植) ---
    # hdl_globalmap_pcd = '/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall_standard.pcd'  # 需要预转换到标准坐标系
    
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
    # │  方案B：多分辨率 NDT 全局定位 (推荐, ★当前使用★)                         │
    # │  算法：粗NDT(3m)网格搜索 → 中NDT(1.5m)精化 → 细NDT(1m)跟踪 + 指数平滑   │
    # │  特点：可在任意位置启动，仅依赖PCL，550行代码易维护                       │
    # │  启动：延迟5秒启动节点 + 7秒激活生命周期                                   │
    # └──────────────────────────────────────────────────────────────────────────┘
    global_localization_node = Node(
        package='humanoid_global_localization',
        executable='global_localization_node',
        name='global_localization',
        output='screen',
        parameters=[global_localization_params_file, {'use_sim_time': use_sim_time}],
    )
    global_loc_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_global_loc',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['global_localization']
        }]
    )

    # 5. 机器人实时位姿发布器（从 TF 读取 map_ground->base_footprint）
    #    与 /pcl_pose 不同：/pcl_pose 发布的是 map->odom 偏移量（通常 0.1-0.5m），
    #    本节点通过完整 TF 链计算机器人在地图中的实际位姿，发布到 /robot_realpose
    robot_realpose_publisher = Node(
        package='humanoid_navigation2',
        executable='robot_realpose_publisher',
        name='robot_realpose_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ┌──────────────────────────────────────────────────────────────────────────┐
    # │  方案A：单分辨率 NDT 定位 (旧方案, 已注释)                                │
    # │  算法：PCL NDT_OMP, 1m分辨率，无全局搜索                                │
    # │  特点：需要人工指定初始位姿或放在原点附近(<0.5m)，简单稳定               │
    # │  依赖：lidar_localization_ros2 包                                        │
    # └──────────────────────────────────────────────────────────────────────────┘
    # ndt_localization_node = Node(
    #     package='lidar_localization_ros2',
    #     executable='lidar_localization_node',
    #     name='lidar_localization',
    #     parameters=[localization_params_file, {'use_sim_time': use_sim_time}],
    #     remappings=[('/cloud', '/fast_lio/cloud_registered')],
    # )
    # ndt_lifecycle_manager = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_ndt',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'autostart': True,
    #         'bond_timeout': 0.0,
    #         'node_names': ['lidar_localization']
    #     }]
    # )

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
    # │  ║     转换矩阵: R_body_to_std=[[0,1,0],[0,0,-1],[-1,0,0]]    ║        │
    # │  ║                                                              ║        │
    # │  ║  3. 点云坐标转换 (hdl内部通过TF自动完成):                   ║        │
    # │  ║     body帧→base_footprint帧 (由body→base_footprint静态TF)   ║        │
    # │  ║                                                              ║        │
    # │  ║  4. Odom预测 (hdl内部通过TF链自动完成):                      ║        │
    # │  ║     通过TF查base_footprint帧间delta, 在odom帧表达           ║        │
    # │  ╚══════════════════════════════════════════════════════════════╝        │
    # └──────────────────────────────────────────────────────────────────────────┘
    # # --- 方案C数据预处理: IMU坐标转换 ---
    # # 将 /airy_imu 从 body帧(非标) 旋转到 base_footprint帧(标准), 发布 /imu_standard
    # imu_transformer_node = Node(
    #     package='humanoid_navigation2',
    #     executable='imu_transformer',
    #     name='imu_transformer',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )
    #
    # # --- 方案C核心: hdl_localization 容器 ---
    # # 包含 GlobalmapServer (加载标准坐标系PCD地图) + HdlLocalization (UKF+NDT定位)
    # hdl_container = ComposableNodeContainer(
    #     name='hdl_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='hdl_localization',
    #             plugin='hdl_localization::GlobalmapServerNodelet',
    #             name='GlobalmapServerNodelet',
    #             parameters=[{
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
    #                 ('/velodyne_points', '/fast_lio/cloud_registered'),  # 点云(body帧)
    #                 ('/gpsimu_driver/imu_data', '/imu_standard'),       # IMU(标准帧, 经imu_transformer转换)
    #             ],
    #             parameters=[{
    #                 # --- 帧配置 ---
    #                 'odom_child_frame_id': 'base_footprint',  # 传感器/机器人帧(标准坐标系)
    #                 'robot_odom_frame_id': 'odom',            # 里程计帧ID
    #                 # --- IMU ---
    #                 'use_imu': True,            # 启用IMU预测 (使用转换后的标准帧IMU)
    #                 'invert_acc': False,         # 不需要翻转, imu_transformer已处理坐标转换
    #                 'invert_gyro': False,
    #                 # --- 里程计预测 ---
    #                 'enable_robot_odometry_prediction': True,  # 启用odom预测 (通过TF链自动转换)
    #                 # --- 扫描匹配 ---
    #                 'reg_method': 'NDT_OMP',     # 多线程NDT加速
    #                 'ndt_resolution': 1.0,        # NDT分辨率(米), 室内推荐0.5~1.0
    #                 'downsample_resolution': 0.1, # 点云降采样(米)
    #                 # --- 初始位姿 ---
    #                 'cool_time_duration': 2.0,    # 初始收敛冷却时间(秒)
    #                 'specify_init_pose': False,   # 不指定初始位姿, 从/initialpose或全局定位获取
    #                 'use_global_localization': False,  # 改为True启用 BBS/FPFH+RANSAC 全局重定位
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
    
    # 5.1 核心节点：延迟 6.0 秒启动
    nav2_core_nodes = TimerAction(
        period=6.0,  
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
        ]
    )

    # 5.2 生命周期激活器：延迟到 12.0 秒！
    # （给核心节点充足的时间实例化并构建代价地图内存，等 CPU 闲置后，司令部再统一激活它们，防止 2.0 秒超时假死）
    nav2_lifecycle_node = TimerAction(
        period=12.0,
        actions=[
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
        tf_bridge_base,
        tf_map_to_ground,
        tf_odom_to_ground,

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

        # ┌─ 方案B：多分辨率 NDT 全局定位 (★当前使用★) ─┐
        TimerAction(period=5.0, actions=[global_localization_node]),
        TimerAction(period=7.0, actions=[global_loc_lifecycle_manager]),
        # └──────────────────────────────────────────┘

        # ┌─ 方案A：单分辨率 NDT 定位 (旧方案, 已注释) ─┐
        # TimerAction(period=5.0, actions=[ndt_localization_node]),
        # TimerAction(period=7.0, actions=[ndt_lifecycle_manager]),
        # └──────────────────────────────────────────────┘

        # ┌─ 方案C：hdl_localization UKF+NDT (Humble移植, 已注释) ─┐
        #   hdl组件容器自带生命周期, 无需TimerAction和lifecycle_manager
        # imu_transformer_node,  # IMU坐标转换 (先启动)
        # hdl_container,         # hdl定位容器 (GlobalmapServer + HdlLocalization)
        # └─────────────────────────────────────────────────────────┘

        # 机器人实时位姿发布器（延迟7.5秒，等 TF 树完整）
        TimerAction(period=7.5, actions=[robot_realpose_publisher]),

        # ========== 第四部分：辅助节点（可选） ==========
        # 定期清除点云发布器（2秒）
        periodic_clearing_node, 
        periodic_clearing_3d_node,

        # ========== 第五部分：导航层（最后启动） ==========
        # 1. 释放 Nav2 导航栈各组件的执行对象 (第 6 秒)
        nav2_core_nodes,
        # 2. 等场地稳定、代价地图缓存建好后，释放总司令接管并激活 (第 12 秒)
        nav2_lifecycle_node,
    ])
