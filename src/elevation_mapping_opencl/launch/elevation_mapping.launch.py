"""
elevation_mapping_opencl 高程图启动文件
- 节点：elevation_mapping_opencl_node
- 配置：来自本包自己的 config/elevation_mapping_params.yaml
- 不再依赖 humanoid_elevation_mapping
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ===== 只从自己包取配置 =====
    pkg_opencl = get_package_share_directory('elevation_mapping_opencl')

    params_file = os.path.join(
        pkg_opencl, 'config', 'elevation_mapping_params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),

        Node(
            package='elevation_mapping_opencl',
            executable='elevation_mapping_opencl_node',
            name='elevation_mapping_opencl_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ]
        )
    ])