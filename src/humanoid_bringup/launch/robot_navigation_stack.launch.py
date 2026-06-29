import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('humanoid_description')
    pkg_navigation2 = get_package_share_directory('humanoid_navigation2')
    pkg_navigation = get_package_share_directory('humanoid_navigation')

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
        default='/home/ubuntu/software/Todesk/Files/humanoid_ws/src/robosense_lidar_localization/config/robosense_lidar_localization.yaml',
    )
    reloc_engine = LaunchConfiguration('relocalization_engine', default='ro')
    use_cpp_route_runtime = LaunchConfiguration('use_cpp_route_runtime', default='true')

    use_ro = PythonExpression(["'", reloc_engine, "' == 'ro' or '", reloc_engine, "' == 'robosense'"])
    use_op = PythonExpression(["'", reloc_engine, "' == 'op' or '", reloc_engine, "' == 'prior'"])

    display_layer = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch', 'display.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'rviz': 'false'}.items(),
        ),
    ])

    nav2_ro = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation2, 'launch', 'navigation2_robosense_lidar.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_yaml_file': map_yaml_file,
            'robosense_config_file': robosense_config_file,
        }.items(),
        condition=IfCondition(use_ro),
    )

    nav2_op = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation2, 'launch', 'navigation2.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_yaml_file': map_yaml_file,
            'prior_map_path': prior_map_path,
        }.items(),
        condition=IfCondition(use_op),
    )

    route_runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation, 'launch', 'navigation_route_runtime.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_cpp_route_runtime': use_cpp_route_runtime,
        }.items(),
    )

    rviz_config_path = os.path.join(pkg_navigation2, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path, '--ros-args', '--log-level', 'ERROR'],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true', description='Whether to start RViz'),
        DeclareLaunchArgument('map_id', default_value='hall', description='当前导航层加载的地图 ID'),
        DeclareLaunchArgument('map_yaml_file', default_value='/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml'),
        DeclareLaunchArgument('prior_map_path', default_value='/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd'),
        DeclareLaunchArgument('robosense_config_file', default_value='/home/ubuntu/software/Todesk/Files/humanoid_ws/src/robosense_lidar_localization/config/robosense_lidar_localization.yaml'),
        DeclareLaunchArgument('relocalization_engine', default_value='ro'),
        DeclareLaunchArgument(
            'use_cpp_route_runtime',
            default_value='true',
            description='是否使用 C++ 路线任务运行层；默认 true，false 回退 Python navigation_state_manager',
        ),
        # 导航层随地图重启：先发布机器人模型/TF，再拉起定位+Nav2，最后启动路线任务运行层。
        display_layer,
        TimerAction(period=6.0, actions=[nav2_ro, nav2_op]),
        TimerAction(period=9.0, actions=[route_runtime]),
        TimerAction(period=10.0, actions=[rviz_node]),
    ])
