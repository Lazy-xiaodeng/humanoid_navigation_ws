import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    #pkg_description = get_package_share_directory('humanoid_description')
    pkg_navigation2 = get_package_share_directory('humanoid_navigation2')
    pkg_app_gateway_runtime = get_package_share_directory('humanoid_app_gateway_runtime')
    pkg_robot_gateway_runtime = get_package_share_directory('humanoid_robot_gateway_runtime')

    # 2. 声明全局参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pcd_map_file = LaunchConfiguration('pcd_map_file')
    use_rviz = LaunchConfiguration('rviz', default='true')
    use_app_layer = LaunchConfiguration('app_layer', default='false')

    # ================= 第一阶段：建图核心 =================
    launch_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation2, 'launch', 'mapping_only.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'pcd_map_file': pcd_map_file,
        }.items()
    )

    # ================= 第二阶段：应用层（延迟5秒）=================
    launch_app_layer = TimerAction(
        period=5.0,
        actions=[
            GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_app_gateway_runtime, 'launch', 'app_gateway_runtime.launch.py')),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_robot_gateway_runtime, 'launch', 'robot_gateway_runtime.launch.py')),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                )
            ], condition=IfCondition(use_app_layer))
        ]
    )

    # ================= 第三阶段：可视化（延迟6秒）=================
    rviz_config_path = os.path.join(pkg_navigation2, 'rviz', 'mapping.rviz') 

    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path],
                condition=IfCondition(use_rviz)
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('pcd_map_file', default_value=os.path.expanduser('~/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd')),
        DeclareLaunchArgument('rviz', default_value='true', description='Whether to start RViz'),
        DeclareLaunchArgument('app_layer', default_value='false', description='Whether to start C++ APP/robot gateway app layer'),
        
        launch_mapping,
        launch_app_layer,
        rviz_node
    ])
