import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取当前包路径
    pkg_path = get_package_share_directory('humanoid_elevation_mapping')
    
    # 参数文件路径
    core_params = os.path.join(pkg_path, 'config', 'core_params.yaml')
    #plug_params = os.path.join(pkg_path, 'config', 'plugin_config.yaml')

    return LaunchDescription([
        Node(
            package='elevation_mapping_cupy',
            executable='elevation_mapping_node',
            name='elevation_mapping',
            output='screen',
            parameters=[
                core_params
            ]
        )
    ])