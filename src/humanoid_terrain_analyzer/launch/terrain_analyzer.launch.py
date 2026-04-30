"""
launch/terrain_analyzer.launch.py

启动 humanoid_terrain_analyzer 地形分析节点。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取本包 share 目录，用于定位 yaml 参数文件
    pkg_share = get_package_share_directory('humanoid_terrain_analyzer')
    params_file = os.path.join(pkg_share, 'config', 'terrain_analyzer.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    terrain_analyzer_node = Node(
        package='humanoid_terrain_analyzer',
        executable='terrain_analyzer_node',
        name='terrain_analyzer',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        # 如果需要 remap 话题，在这里加：
        # remappings=[
        #     ('/elevation_mapping/output', '/your_map_topic'),
        # ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        terrain_analyzer_node,
    ])