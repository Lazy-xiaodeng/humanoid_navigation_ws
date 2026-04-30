import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # 保持原有的路径获取方式
    package_path = get_package_share_directory('fast_lio_robosense')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz', 'fastlio.rviz')

    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_path = LaunchConfiguration('config_path', default=default_config_path)
    config_file = LaunchConfiguration('config_file', default='robosenseAiry.yaml')
    rviz_use = LaunchConfiguration('rviz', default='true')
    rviz_cfg = LaunchConfiguration('rviz_cfg', default=default_rviz_config_path)
    map_file_path = LaunchConfiguration('map_file_path', default='')
    
    # 1. 获取 humanoid_navigation2 的路径
    pkg_nav2 = get_package_share_directory('humanoid_navigation2')
    # 2. 定义默认的 pcd 保存完整路径
    default_pcd_path = os.path.join(pkg_nav2, 'pcd', 'scans.pcd')
    
    # 3. 修改声明参数，将默认值设为刚才定义的路径
    declare_map_file_path_cmd = DeclareLaunchArgument(
        'map_file_path', 
        default_value=default_pcd_path,
        description='Path to save the map PCD file'
    )

    declare_save_pcd_cmd = DeclareLaunchArgument(
        'save_pcd',
        default_value='false',  # 默认不保存，保证绝对安全
        description='Whether to save PCD map on exit'  
    )
    save_pcd = LaunchConfiguration('save_pcd')

    # 1. 核心 Fast-LIO 节点 (使用多线程executor)
    fast_lio_node = Node(
        package='fast_lio_robosense',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {
                     'use_sim_time': use_sim_time,
                     'map_file_path': map_file_path,
                     'pcd_save.pcd_save_en': save_pcd
                     }
                ],
        output='screen',
        # 使用MultiThreadedExecutor提高回调处理速度
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 2. 新增：点云转激光节点 (实时切片)
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/fast_lio/cloud_registered'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.1, # 取离地10cm以上
            'max_height': 1.8, # 到1.8m以下
            'range_min': 0.2,
            'range_max': 50.0,
            'use_inf': True
        }]
    )

    # 3. 新增：2D建图节点 (Slam Toolbox)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'mode': 'mapping'
        }]
    )

    # 4. 可视化节点 (保持原样)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    # 添加所有原本的参数声明
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('config_path', default_value=default_config_path))
    ld.add_action(DeclareLaunchArgument('config_file', default_value='robosenseAiry.yaml'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('rviz_cfg', default_value=default_rviz_config_path))
    ld.add_action(declare_map_file_path_cmd)
    ld.add_action(declare_save_pcd_cmd)

    # 启动所有节点
    ld.add_action(fast_lio_node)
    ld.add_action(pc_to_scan_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)

    return ld