import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # 1. 获取各个功能包的路径
    pkg_bringup = get_package_share_directory('humanoid_bringup')
    pkg_simulation = get_package_share_directory('humanoid_simulation')
    pkg_navigation2 = get_package_share_directory('humanoid_navigation2')
    pkg_navigation = get_package_share_directory('humanoid_navigation')
    pkg_websocket = get_package_share_directory('humanoid_websocket')

    # ==== 强行把所有节点设置成使用仿真时间 ====
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ================= 仿真器基础层 =================
    # 1. 启动 Gazebo 及仿真世界
    world_file = os.path.join(pkg_simulation, 'worlds', 'office_world.world')
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # 2. 状态发布 (加载我们写的套娃型仿真 URDF)
    sim_urdf_file = os.path.join(pkg_simulation, 'urdf', 'sim_humanoid.urdf.xacro')
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', sim_urdf_file]),
            'use_sim_time': True
        }]
    )

    # 3. 将机器人投放进 Gazebo (z稍微高一点，防止一出现就卡在地下)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'humanoid_robot', '-z', '1.0'],
        output='screen'
    )

    # ================= 业务层 (延迟启动防崩溃) =================
    # Gazebo 启动并落稳需要一两秒，先让 tf 发出来，业务层全部延时加载
    
    # [ 第一阶段：感知与定位 ]
    launch_localization = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_navigation2, 'launch', 'localization.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()
        )]
    )

    launch_perception = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_navigation2, 'launch', 'perception.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()
        )]
    )

    # [ 第二阶段：规划与执行 ]
    launch_nav2_stack = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_navigation2, 'launch', 'navigation_stack.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()
        )]
    )

    launch_nav_manager = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()
        )]
    )

    # [ 第三阶段：交互与 APP ]
    launch_websocket_bridge = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_websocket, 'launch', 'websocket_server.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()
        )]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # 基础仿真环境
        launch_gazebo,
        robot_state_pub,
        spawn_entity,
        
        # 业务逻辑流水线
        launch_localization,
        launch_perception,
        launch_nav2_stack,
        launch_nav_manager,
        launch_websocket_bridge
    ])