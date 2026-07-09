import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    # 1. 获取各个功能包的路径
    pkg_description = get_package_share_directory('humanoid_description')
    pkg_navigation2 = get_package_share_directory('humanoid_navigation2')
    pkg_control_runtime = get_package_share_directory('humanoid_control_runtime')
    pkg_route_runtime = get_package_share_directory('humanoid_route_runtime')
    pkg_app_gateway_runtime = get_package_share_directory('humanoid_app_gateway_runtime')
    pkg_robot_gateway_runtime = get_package_share_directory('humanoid_robot_gateway_runtime')
    pkg_expression_runtime = get_package_share_directory('humanoid_expression_runtime')

    # 2. 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('rviz', default='false')
    # relocalization_engine 用来选择“一键启动”里的定位链路：
    #   ro / robosense : RoboSense lidar_localization + prior_map_odom_bridge
    #   op / prior     : Open3D prior-map localization + prior_map_odom_bridge
    reloc_engine = LaunchConfiguration('relocalization_engine', default='ro')

    # 条件表达式: 只保留 ro / op(prior) 两类导航栈。
    use_ro = PythonExpression([
        "'", reloc_engine, "' == 'ro' or '", reloc_engine, "' == 'robosense'"
    ])
    use_op = PythonExpression([
        "'", reloc_engine, "' == 'op' or '", reloc_engine, "' == 'prior'"
    ])
    use_ro_or_op = PythonExpression([
        "'", reloc_engine, "' == 'ro' or '", reloc_engine, "' == 'robosense' or '",
        reloc_engine, "' == 'op' or '", reloc_engine, "' == 'prior'"
    ])

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
    # ro 版本 (默认) — RoboSense lidar_localization 输出 map->base_footprint 候选位姿，
    # prior_map_odom_bridge 独占发布 map->odom，并沿用现有 Nav2/状态管理链路。
    launch_nav2_ro = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation2, 'launch', 'navigation2_robosense_lidar.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_ro)
    )

    # op/prior 版本 — Open3D prior-map 定位 + bridge 独占 map->odom。
    # 这个入口用于和 ro 做 A/B 对比；op 链路同样会启动 ROI 障碍检测。
    launch_nav2_op = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation2, 'launch', 'navigation2.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_op)
    )

    launch_nav2_stack = GroupAction([launch_nav2_ro, launch_nav2_op])

    # ================= 第三阶段：应用层（延迟9秒）=================
    # ro/op 共用 APP 层：C++ 点位管理、C++ 导航状态管理、APP 网关、机器人本体网关、表情驱动。
    launch_app_layer = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_control_runtime, 'launch', 'control_runtime.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(use_ro_or_op)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_route_runtime, 'launch', 'route_runtime.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(use_ro_or_op)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_app_gateway_runtime, 'launch', 'app_gateway_runtime.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(use_ro_or_op)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_robot_gateway_runtime, 'launch', 'robot_gateway_runtime.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(use_ro_or_op)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_expression_runtime, 'launch', 'expression_runtime.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(use_ro_or_op)
        )
    ])

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
        DeclareLaunchArgument('relocalization_engine', default_value='ro',
                              description='Nav2 stack: ro/robosense (default) | op/prior (Open3D prior-map)'),

        # 按顺序启动
        launch_description,
        launch_nav2_stack,
        launch_app_layer,
        rviz_node,
    ])
