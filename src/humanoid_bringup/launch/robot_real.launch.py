import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument,TimerAction 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # 1. 获取各个功能包的路径
    pkg_description = get_package_share_directory('humanoid_description')
    pkg_navigation2 = get_package_share_directory('humanoid_navigation2')
    pkg_navigation = get_package_share_directory('humanoid_navigation')
    pkg_websocket = get_package_share_directory('humanoid_websocket')
    pkg_locomotion = get_package_share_directory('humanoid_locomotion')

    # 2. 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('rviz', default='true')

    # ================= 第一阶段：基础设施 =================
    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch', 'display.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': 'false'
        }.items()
    )

    # ================= 第二阶段：导航栈（延迟6秒）=================
    launch_nav2_stack = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_navigation2, 'launch', 'navigation2.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # ================= 第三阶段：应用层（延迟9秒）=================
    launch_app_layer = TimerAction(
        period=9.0,
        actions=[
            GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_websocket, 'launch', 'websocket_server.launch.py')),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_locomotion, 'launch', 'locomotion.launch.py')),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                )
            ])
        ]
    )

    # ================= 第四阶段：可视化（延迟10秒）=================
    rviz_config_path = os.path.join(pkg_navigation2, 'rviz', 'navigation.rviz')
    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path, '--ros-args', '--log-level', 'ERROR'],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true', description='Whether to start RViz'),
        
        # 按顺序启动
        launch_description,
        launch_nav2_stack,
        launch_app_layer,
        rviz_node,
    ])