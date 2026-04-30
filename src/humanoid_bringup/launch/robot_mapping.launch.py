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
    pkg_websocket = get_package_share_directory('humanoid_websocket')
    pkg_locomotion = get_package_share_directory('humanoid_locomotion')

    # 2. 声明全局参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 【新增】控制 RViz 是否启动的全局开关
    use_rviz = LaunchConfiguration('rviz', default='true')

    # ================= 第一阶段：建图核心 =================
    launch_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_navigation2, 'launch', 'mapping_only.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ================= 第二阶段：应用层（延迟5秒）=================
    launch_app_layer = TimerAction(
        period=5.0,
        actions=[
            GroupAction([
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
        DeclareLaunchArgument('rviz', default_value='true', description='Whether to start RViz'),
        
        launch_mapping,
        launch_app_layer,
        rviz_node
    ])