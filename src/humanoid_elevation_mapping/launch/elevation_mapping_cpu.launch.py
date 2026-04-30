import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('humanoid_elevation_mapping'), 'config', 'cpu_params.yaml')

    return LaunchDescription([
        Node(
            package='elevation_mapping', # 指向 CPU 版本的包名
            executable='elevation_mapping_node',
            name='elevation_mapping',
            output='screen',
            parameters=[config],
            # 关键：把输出话题统一命名为 /elevation_map
            # remappings=[
            #      ('/pose', '/odom'),
            #      ('/points', '/airy_points_filtered'),
            #      ('/elevation_mapping/elevation_map', '/elevation_map'), # 对应 CPU 版
            #      ('/elevation_mapping/elevation_map_raw', '/elevation_map_raw')
            # ]
        )
    ])