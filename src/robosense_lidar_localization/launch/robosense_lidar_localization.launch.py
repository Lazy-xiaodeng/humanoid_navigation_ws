from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 是否使用仿真时间：
    #   false：实机运行，使用系统时间。
    #   true ：bag 回放或仿真，跟随 /clock。
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # RoboSense lidar_localization 的 YAML 配置。
    # 该配置决定输入点云/odom话题、地图路径、初始位姿、坐标转换、输出话题和匹配阈值。
    config_file = LaunchConfiguration(
        'config_file',
        default='/home/ubuntu/humanoid_ws/src/robosense_lidar_localization/config/robosense_lidar_localization.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用 /clock；实机 false，bag/仿真 true'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='/home/ubuntu/humanoid_ws/src/robosense_lidar_localization/config/robosense_lidar_localization.yaml',
            description='RoboSense lidar_localization YAML 配置文件路径'
        ),

        # 单独启动 RoboSense 定位节点。
        #
        # 节点输入：
        #   /odom
        #   /fast_lio/cloud_registered
        #
        # 节点输出：
        #   /prior_localization/robosense_odom        map->base_footprint 位姿候选
        #   /prior_localization/robosense_input_odom  bridge 用于同时间戳插值的 odom cache
        #
        # 注意：
        #   这里不直接发布 map->odom；完整导航链路中由 prior_map_odom_bridge 统一发布。
        Node(
            package='robosense_lidar_localization',
            executable='robosense_lidar_localization_node',
            name='robosense_lidar_localization_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'config_file': config_file},
            ],
        ),
    ])
