import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_relocalization = get_package_share_directory('humanoid_relocalization')
    
    params_file = os.path.join(pkg_relocalization, 'config', 'relocalization_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    relocalization_node = Node(
        package='humanoid_relocalization',
        executable='relocalization_node',
        name='relocalization_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # 如果需要重映射话题
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        relocalization_node
    ])