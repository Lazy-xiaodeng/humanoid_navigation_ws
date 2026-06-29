import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_control_runtime = get_package_share_directory('humanoid_control_runtime')
    pkg_app_gateway_runtime = get_package_share_directory('humanoid_app_gateway_runtime')
    pkg_robot_gateway_runtime = get_package_share_directory('humanoid_robot_gateway_runtime')
    pkg_expression_runtime = get_package_share_directory('humanoid_expression_runtime')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    app_websocket_server_enable = LaunchConfiguration('app_websocket_server_enable', default='true')
    app_websocket_host = LaunchConfiguration('app_websocket_host', default='0.0.0.0')
    app_websocket_port = LaunchConfiguration('app_websocket_port', default='8765')
    data_integration_enable = LaunchConfiguration('data_integration_enable', default='true')
    robot_ws_enable = LaunchConfiguration('robot_ws_enable', default='true')
    robot_walk_velocity_send_enable = LaunchConfiguration('robot_walk_velocity_send_enable', default='true')
    robot_motion_execution_enable = LaunchConfiguration('robot_motion_execution_enable', default='true')
    robot_motion_allow_enter_menu = LaunchConfiguration('robot_motion_allow_enter_menu', default='true')
    robot_motion_allow_return_walk = LaunchConfiguration('robot_motion_allow_return_walk', default='true')
    robot_gesture_sync_enable = LaunchConfiguration('robot_gesture_sync_enable', default='true')
    expression_config_file = LaunchConfiguration(
        'expression_config_file',
        default=os.path.join(pkg_expression_runtime, 'config', 'expression_runtime.yaml'),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='是否使用 /clock；实机 false。'),
        DeclareLaunchArgument('app_websocket_server_enable', default_value='true', description='是否启动 APP WebSocket 服务。'),
        DeclareLaunchArgument('app_websocket_host', default_value='0.0.0.0', description='APP WebSocket 监听地址。'),
        DeclareLaunchArgument('app_websocket_port', default_value='8765', description='APP WebSocket 监听端口。'),
        DeclareLaunchArgument('data_integration_enable', default_value='true', description='是否启动数据整合订阅与推送。'),
        DeclareLaunchArgument('robot_ws_enable', default_value='true', description='是否连接机器人本体 WebSocket。'),
        DeclareLaunchArgument('robot_walk_velocity_send_enable', default_value='true', description='是否允许真实下发 /cmd_vel。'),
        DeclareLaunchArgument('robot_motion_execution_enable', default_value='true', description='是否允许真实执行 APP 动作。'),
        DeclareLaunchArgument('robot_motion_allow_enter_menu', default_value='true', description='动作前是否允许自动进入 Menu。'),
        DeclareLaunchArgument('robot_motion_allow_return_walk', default_value='true', description='动作后是否允许自动返回 Walk。'),
        DeclareLaunchArgument('robot_gesture_sync_enable', default_value='true', description='是否连接后同步机器人动作库。'),
        DeclareLaunchArgument('expression_config_file', default_value=os.path.join(pkg_expression_runtime, 'config', 'expression_runtime.yaml'), description='表情运行层参数文件。'),
        # 常驻控制层：点位、地图上下文、APP 网关、机器人网关和表情串口统一由 C++ runtime 提供。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_control_runtime, 'launch', 'control_runtime.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_app_gateway_runtime, 'launch', 'app_gateway_runtime.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'websocket_server_enable': app_websocket_server_enable,
                'websocket_host': app_websocket_host,
                'websocket_port': app_websocket_port,
                'data_integration_enable': data_integration_enable,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_robot_gateway_runtime, 'launch', 'robot_gateway_runtime.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_ws_enable': robot_ws_enable,
                'walk_velocity_send_enable': robot_walk_velocity_send_enable,
                'motion_execution_enable': robot_motion_execution_enable,
                'motion_allow_enter_menu': robot_motion_allow_enter_menu,
                'motion_allow_return_walk': robot_motion_allow_return_walk,
                'gesture_sync_enable': robot_gesture_sync_enable,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_expression_runtime, 'launch', 'expression_runtime.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'expression_config_file': expression_config_file,
            }.items(),
        ),
    ])
