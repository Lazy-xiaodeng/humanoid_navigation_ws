#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
仿真导航启动文件 - ROS2 版本
功能：启动 Gazebo 仿真环境和导航系统
修改说明：从 ROS1 XML 迁移到 ROS2 Python launch 文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import FrontendLaunchDescriptionSource
import os

def generate_launch_description():
    # 获取功能包路径
    pkg_humanoid_navigation = FindPackageShare('humanoid_navigation')
    pkg_humanoid_simulation = FindPackageShare('humanoid_simulation')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    debug = LaunchConfiguration('debug', default='false')
    world_name = LaunchConfiguration('world_name', default=PathJoinSubstitution([
        pkg_humanoid_navigation, 'worlds', 'exhibition_hall.world'
    ]))
    robot_name = LaunchConfiguration('robot_name', default='humanoid_robot')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # 参数声明
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间（Gazebo 时钟）'
    )
    
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='启动 Gazebo GUI'
    )
    
    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='无头模式（不启动 GUI）'
    )
    
    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='调试模式'
    )
    
    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value=PathJoinSubstitution([pkg_humanoid_simulation, 'worlds', 'exhibition_hall.world']),
        description='Gazebo 世界文件路径'
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='机器人模型名称'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='启动 RViz2 可视化'
    )
    
    # 设置仿真时间参数
    set_use_sim_time = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
        output='screen'
    )
    
    # 启动 Gazebo 空世界
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'false',
            'pause': 'false',
            'world': world_name,
            'gui': gui,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # 加载机器人描述
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            pkg_humanoid_simulation, 'urdf', 'humanoid_robot.urdf.xacro'
        ])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # 机器人状态发布器节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )
    
    # 关节状态发布器节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_gui': 'false'
        }]
    )
    
    # 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '1.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # 启动导航栈
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_humanoid_navigation, 'launch', 'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': PathJoinSubstitution([pkg_humanoid_navigation, 'maps', 'exhibition_hall.yaml']),
            'params_file': PathJoinSubstitution([pkg_humanoid_navigation, 'config', 'nav2_params.yaml'])
        }.items()
    )
    
    # 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            pkg_humanoid_navigation, 'config', 'navigation.rviz'
        ])],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 键盘控制节点（用于测试）
    teleop_twist_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/cmd_vel')]
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_gui)
    ld.add_action(declare_headless)
    ld.add_action(declare_debug)
    ld.add_action(declare_world_name)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_use_rviz)
    
    # 添加节点和启动文件
    ld.add_action(set_use_sim_time)
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity)
    ld.add_action(navigation_launch)
    ld.add_action(rviz_node)
    ld.add_action(teleop_twist_keyboard)
    
    return ld

if __name__ == '__main__':
    generate_launch_description()