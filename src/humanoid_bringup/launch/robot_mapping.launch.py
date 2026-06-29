import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation2 = get_package_share_directory('humanoid_navigation2')
    pkg_app_gateway_runtime = get_package_share_directory('humanoid_app_gateway_runtime')
    pkg_robot_gateway_runtime = get_package_share_directory('humanoid_robot_gateway_runtime')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('rviz', default='true')

    launch_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation2, 'launch', 'mapping_only.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 建图模式不启动表情串口，避免建图时没有头部串口也反复重连报错。
    # 这里只保留 APP 网关和机器人状态网关，供页面查看建图状态和机器人身份。
    launch_app_layer = TimerAction(
        period=5.0,
        actions=[
            GroupAction([
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_app_gateway_runtime, 'launch', 'app_gateway_runtime.launch.py')),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'websocket_server_enable': 'true',
                        'data_integration_enable': 'true',
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_robot_gateway_runtime, 'launch', 'robot_gateway_runtime.launch.py')),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'robot_ws_enable': 'true',
                        'walk_velocity_send_enable': 'false',
                        'motion_execution_enable': 'false',
                        'gesture_sync_enable': 'false',
                    }.items(),
                ),
            ])
        ],
    )

    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(pkg_navigation2, 'rviz', 'mapping.rviz')],
                condition=IfCondition(use_rviz),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true', description='Whether to start RViz'),
        launch_mapping,
        launch_app_layer,
        rviz_node,
    ])
