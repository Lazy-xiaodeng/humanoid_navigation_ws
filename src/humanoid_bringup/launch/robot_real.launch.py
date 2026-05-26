import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

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
    reloc_engine = LaunchConfiguration('relocalization_engine', default='hdl')

    # 条件表达式: 是否使用 SC (否则用 HDL)
    use_sc = PythonExpression(["'", reloc_engine, "' == 'sc'"])

    # ================= 第一阶段：基础设施 =================
    launch_description = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch', 'display.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'rviz': 'false'
                }.items()
            )
        ],
        scoped=True,
    )

    # ================= 第二阶段：导航栈（延迟6秒）=================
    # SC 版本 (relocalization_engine:=sc)
    launch_nav2_sc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation2, 'launch', 'navigation2_fusion_sc.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_sc)
    )

    # HDL 版本 (relocalization_engine:=hdl, 默认)
    launch_nav2_hdl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation2, 'launch', 'navigation2_fusion.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(use_sc)
    )

    launch_nav2_stack = TimerAction(
        period=6.0,
        actions=[launch_nav2_sc, launch_nav2_hdl]
    )

    # ================= 第三阶段：应用层（延迟9秒）=================
    # SC 版本 APP 层
    launch_app_sc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation_fusion_sc.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_sc)
    )

    # HDL 版本 APP 层
    launch_app_hdl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation_fusion.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(use_sc)
    )

    launch_app_layer = TimerAction(
        period=9.0,
        actions=[
            GroupAction([
                launch_app_sc,
                launch_app_hdl,
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
                condition=IfCondition(use_rviz),
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true', description='Whether to start RViz'),
        DeclareLaunchArgument('relocalization_engine', default_value='hdl',
                              description='Global relocalization engine: hdl (FPFH+RANSAC) or sc (ScanContext)'),

        # 按顺序启动
        launch_description,
        launch_nav2_stack,
        launch_app_layer,
        rviz_node,
    ])
