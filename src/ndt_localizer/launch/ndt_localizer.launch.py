import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_ndt = get_package_share_directory('ndt_localizer')
    ndt_params_file = os.path.join(pkg_ndt, 'config', 'ndt_localizer_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    ndt_localizer_node = Node(
        package='ndt_localizer',
        executable='ndt_localizer_node.py',
        name='ndt_localizer',
        output='screen',
        parameters=[
            ndt_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        ndt_localizer_node
    ])
