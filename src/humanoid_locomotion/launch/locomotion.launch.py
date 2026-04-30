import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 获取功能包的共享资源路径
    pkg_share = get_package_share_directory('humanoid_locomotion')
    
    # 配置文件路径
    facial_gestures = os.path.join(pkg_share, 'config', 'facial_gestures.yaml')

  
    #4.表情控制节点
    facial_node= Node(
        package='humanoid_locomotion',
        executable='facial_driver',
        name='facial_driver',
        parameters=[{'config_path': facial_gestures}], # 加载 YAML 路径
        output='screen'
    )

    return LaunchDescription([

        facial_node,
    ])