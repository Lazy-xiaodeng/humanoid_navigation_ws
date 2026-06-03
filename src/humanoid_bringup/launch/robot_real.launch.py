import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    use_rviz = LaunchConfiguration('rviz', default='false')
    # relocalization_engine 用来选择“一键启动”里的定位链路：
    #   ro / robosense : RoboSense lidar_localization + prior_map_odom_bridge
    #   op / prior     : Open3D prior-map localization + prior_map_odom_bridge
    #   sc             : 旧 ScanContext/NDT 调试链路，保留为手动回退入口
    #   hdl            : 旧 HDL/FPFH 调试链路，保留为手动回退入口
    #
    # 当前默认改成 ro，避免一键启动默认进入原 v2 NDT 组合链路。
    reloc_engine = LaunchConfiguration('relocalization_engine', default='ro')

    # 条件表达式: ro / op(prior) / SC / HDL 四类导航栈
    use_ro = PythonExpression([
        "'", reloc_engine, "' == 'ro' or '", reloc_engine, "' == 'robosense'"
    ])
    use_op = PythonExpression([
        "'", reloc_engine, "' == 'op' or '", reloc_engine, "' == 'prior'"
    ])
    use_sc = PythonExpression(["'", reloc_engine, "' == 'sc'"])
    # ro / op / SC 共用 app 层：路点管理、导航状态管理器、websocket、运动控制。
    use_fusion_sc_app = PythonExpression(["'", reloc_engine, "' != 'hdl'"])

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
    # 这个入口用于和 ro 做 A/B 对比；原有 navigation2.launch.py 不在本次改动里修改。
    launch_nav2_op = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation2, 'launch', 'navigation2.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_op)
    )

    # SC 版本 (relocalization_engine:=sc)
    launch_nav2_sc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation2, 'launch', 'navigation2_fusion_sc.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_sc)
    )

    # HDL 版本 (relocalization_engine:=hdl)
    launch_nav2_hdl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation2, 'launch', 'navigation2_fusion.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(PythonExpression(["'", reloc_engine, "' == 'hdl'"]))
    )

    launch_nav2_stack = TimerAction(
        period=6.0,
        actions=[launch_nav2_ro, launch_nav2_op, launch_nav2_sc, launch_nav2_hdl]
    )

    # ================= 第三阶段：应用层（延迟9秒）=================
    # v2/SC 版本 APP 层 (共用)
    launch_app_sc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation_fusion_sc.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_fusion_sc_app)
    )

    # HDL 版本 APP 层
    launch_app_hdl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation_fusion.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(PythonExpression(["'", reloc_engine, "' == 'hdl'"]))
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
        DeclareLaunchArgument('relocalization_engine', default_value='ro',
                              description='Nav2 stack: ro/robosense (default) | op/prior (Open3D prior-map) | sc (legacy) | hdl (legacy)'),

        # 按顺序启动
        launch_description,
        launch_nav2_stack,
        launch_app_layer,
        rviz_node,
    ])
