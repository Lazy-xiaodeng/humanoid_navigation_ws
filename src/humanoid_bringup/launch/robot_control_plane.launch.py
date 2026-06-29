import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory('humanoid_navigation')
    pkg_websocket = get_package_share_directory('humanoid_websocket')
    pkg_locomotion = get_package_share_directory('humanoid_locomotion')
    pkg_expression_runtime = get_package_share_directory('humanoid_expression_runtime')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_cpp_control_runtime = LaunchConfiguration('use_cpp_control_runtime', default='true')
    use_cpp_app_gateway = LaunchConfiguration('use_cpp_app_gateway', default='true')
    cpp_app_websocket_server_enable = LaunchConfiguration('cpp_app_websocket_server_enable', default='true')
    cpp_app_websocket_host = LaunchConfiguration('cpp_app_websocket_host', default='0.0.0.0')
    cpp_app_websocket_port = LaunchConfiguration('cpp_app_websocket_port', default='8765')
    cpp_data_integration_enable = LaunchConfiguration('cpp_data_integration_enable', default='true')
    use_cpp_robot_gateway = LaunchConfiguration('use_cpp_robot_gateway', default='true')
    cpp_robot_ws_enable = LaunchConfiguration('cpp_robot_ws_enable', default='true')
    cpp_robot_walk_velocity_send_enable = LaunchConfiguration('cpp_robot_walk_velocity_send_enable', default='false')
    cpp_robot_motion_execution_enable = LaunchConfiguration('cpp_robot_motion_execution_enable', default='false')
    cpp_robot_motion_allow_enter_menu = LaunchConfiguration('cpp_robot_motion_allow_enter_menu', default='false')
    cpp_robot_motion_allow_return_walk = LaunchConfiguration('cpp_robot_motion_allow_return_walk', default='false')
    cpp_robot_gesture_sync_enable = LaunchConfiguration('cpp_robot_gesture_sync_enable', default='false')
    use_cpp_expression_runtime = LaunchConfiguration('use_cpp_expression_runtime', default='true')
    cpp_expression_config_file = LaunchConfiguration(
        'cpp_expression_config_file',
        default=os.path.join(pkg_expression_runtime, 'config', 'expression_runtime.yaml'),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_cpp_control_runtime',
            default_value='true',
            description='是否使用 C++ 控制层运行包；默认 true，false 回退 Python 控制层节点',
        ),
        DeclareLaunchArgument(
            'use_cpp_app_gateway',
            default_value='true',
            description='是否使用 C++ APP 网关与数据整合；默认 true，false 回退 Python ws/data 节点',
        ),
        DeclareLaunchArgument(
            'cpp_app_websocket_server_enable',
            default_value='true',
            description='use_cpp_app_gateway=true 时是否打开 C++ APP WebSocket 服务；空载验证可设为 false',
        ),
        DeclareLaunchArgument(
            'cpp_app_websocket_host',
            default_value='0.0.0.0',
            description='C++ APP WebSocket 监听地址；本机 smoke 可设为 127.0.0.1',
        ),
        DeclareLaunchArgument(
            'cpp_app_websocket_port',
            default_value='8765',
            description='C++ APP WebSocket 监听端口；本机 smoke 可设为临时端口避免冲突',
        ),
        DeclareLaunchArgument(
            'cpp_data_integration_enable',
            default_value='true',
            description='use_cpp_app_gateway=true 时是否打开 C++ 数据整合订阅发布；空载验证通常保持 true',
        ),
        DeclareLaunchArgument(
            'use_cpp_robot_gateway',
            default_value='true',
            description='是否使用 C++ 机器人本体网关；默认 true，false 回退 Python websocket_client_node',
        ),
        DeclareLaunchArgument(
            'cpp_robot_ws_enable',
            default_value='true',
            description='use_cpp_robot_gateway=true 时是否连接真实机器人本体 WebSocket；空载验证可设为 false',
        ),
        DeclareLaunchArgument(
            'cpp_robot_walk_velocity_send_enable',
            default_value='false',
            description='C++ 机器人本体网关是否允许真实发送 /cmd_vel；默认 false',
        ),
        DeclareLaunchArgument(
            'cpp_robot_motion_execution_enable',
            default_value='false',
            description='C++ 机器人本体网关是否允许真实执行 APP 动作；默认 false',
        ),
        DeclareLaunchArgument(
            'cpp_robot_motion_allow_enter_menu',
            default_value='false',
            description='C++ 动作执行前是否允许自动切换 Menu；默认 false',
        ),
        DeclareLaunchArgument(
            'cpp_robot_motion_allow_return_walk',
            default_value='false',
            description='C++ 动作结束后是否允许自动切回 Walk；默认 false',
        ),
        DeclareLaunchArgument(
            'cpp_robot_gesture_sync_enable',
            default_value='false',
            description='C++ 机器人本体网关是否连接后同步动作库；默认 false',
        ),
        DeclareLaunchArgument(
            'use_cpp_expression_runtime',
            default_value='true',
            description='是否使用 C++ 表情运行层；默认 true，false 回退 Python facial_driver',
        ),
        DeclareLaunchArgument(
            'cpp_expression_config_file',
            default_value=os.path.join(pkg_expression_runtime, 'config', 'expression_runtime.yaml'),
            description='C++ 表情运行层参数文件',
        ),
        # 控制层常驻：APP 连接、点位管理、地图切换状态机都在这里，切图时不能停止。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_navigation, 'launch', 'navigation_control_plane.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_cpp_control_runtime': use_cpp_control_runtime,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_websocket, 'launch', 'websocket_server.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_cpp_app_gateway': use_cpp_app_gateway,
                'cpp_app_websocket_server_enable': cpp_app_websocket_server_enable,
                'cpp_app_websocket_host': cpp_app_websocket_host,
                'cpp_app_websocket_port': cpp_app_websocket_port,
                'cpp_data_integration_enable': cpp_data_integration_enable,
                'use_cpp_robot_gateway': use_cpp_robot_gateway,
                'cpp_robot_ws_enable': cpp_robot_ws_enable,
                'cpp_robot_walk_velocity_send_enable': cpp_robot_walk_velocity_send_enable,
                'cpp_robot_motion_execution_enable': cpp_robot_motion_execution_enable,
                'cpp_robot_motion_allow_enter_menu': cpp_robot_motion_allow_enter_menu,
                'cpp_robot_motion_allow_return_walk': cpp_robot_motion_allow_return_walk,
                'cpp_robot_gesture_sync_enable': cpp_robot_gesture_sync_enable,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_locomotion, 'launch', 'locomotion.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=UnlessCondition(use_cpp_expression_runtime),
        ),
        Node(
            package='humanoid_expression_runtime',
            executable='facial_driver_cpp',
            name='facial_driver',
            output='screen',
            parameters=[
                cpp_expression_config_file,
                {'use_sim_time': use_sim_time},
            ],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(use_cpp_expression_runtime),
        ),
    ])
