import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_description')
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'humanoid_robot.urdf.xacro')

    # 解析 Xacro 生成 URDF 描述
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}
    
    # 声明 rviz 参数，默认给 'true'（为了兼容单独调试）
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', 
        default_value='true',
        description='Whether to start RViz'
    )
    use_rviz = LaunchConfiguration('rviz')

    # 1. 启动机器人状态发布者 (发布 TF 树)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    # 2. 启动关节状态发布器
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )


    
    # 3. 启动 RViz 验证模型
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz) 

    )

    return LaunchDescription([
        declare_rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])