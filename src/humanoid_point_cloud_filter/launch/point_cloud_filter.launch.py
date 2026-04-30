from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_point_cloud_filter')
    params_file = os.path.join(pkg_share, 'config', 'point_cloud_filter_config.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    point_cloud_filter_node = Node(
        package='humanoid_point_cloud_filter',
        executable='point_cloud_filter_node',
        name='point_cloud_filter_node',  # 必须与 YAML 配置文件中的命名空间一致
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        point_cloud_filter_node,
    ])