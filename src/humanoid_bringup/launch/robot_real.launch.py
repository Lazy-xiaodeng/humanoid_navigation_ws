import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """兼容旧入口的一键 launch。

    新多地图架构已经拆成：
      1. robot_control_plane.launch.py：APP/ROS 常驻控制层，切图时不能停。
      2. robot_navigation_stack.launch.py：绑定当前地图的导航定位层，切图时可重启。

    直接 ros2 launch robot_real.launch.py 时仍会同时启动两层，保证旧启动命令可用；
    运行期切图则由脚本只重启 robot_navigation_stack.launch.py。
    """
    pkg_bringup = get_package_share_directory('humanoid_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('rviz', default='false')
    map_id = LaunchConfiguration('map_id', default='hall')
    map_yaml_file = LaunchConfiguration(
        'map_yaml_file',
        default='/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml',
    )
    prior_map_path = LaunchConfiguration(
        'prior_map_path',
        default='/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd',
    )
    robosense_config_file = LaunchConfiguration(
        'robosense_config_file',
        default='/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_robosense_localization_runtime/config/robosense_lidar_localization.yaml',
    )
    reloc_engine = LaunchConfiguration('relocalization_engine', default='ro')

    control_plane = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'robot_control_plane.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'robot_navigation_stack.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': use_rviz,
            'map_id': map_id,
            'map_yaml_file': map_yaml_file,
            'prior_map_path': prior_map_path,
            'robosense_config_file': robosense_config_file,
            'relocalization_engine': reloc_engine,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true', description='Whether to start RViz'),
        DeclareLaunchArgument('map_id', default_value='hall', description='当前启动的地图 ID'),
        DeclareLaunchArgument('map_yaml_file', default_value='/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml'),
        DeclareLaunchArgument('prior_map_path', default_value='/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd'),
        DeclareLaunchArgument('robosense_config_file', default_value='/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_robosense_localization_runtime/config/robosense_lidar_localization.yaml'),
        DeclareLaunchArgument('relocalization_engine', default_value='ro'),
        control_plane,
        navigation_stack,
    ])
