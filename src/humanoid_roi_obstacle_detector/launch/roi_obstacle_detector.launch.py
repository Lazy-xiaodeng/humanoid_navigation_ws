import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the direct LiDAR ROI obstacle detector.

    The launch file intentionally exposes only the high-value overrides. All
    tuning parameters live in the YAML file so field tests remain reproducible.
    """
    pkg_share = get_package_share_directory('humanoid_roi_obstacle_detector')
    default_params = os.path.join(pkg_share, 'config', 'roi_obstacle_detector.yaml')

    params_file = LaunchConfiguration('params_file')
    input_topic = LaunchConfiguration('input_topic')
    target_frame = LaunchConfiguration('target_frame')
    use_cpp = LaunchConfiguration('use_cpp')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to roi_obstacle_detector.yaml.'),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/airy_points_filtered',
            description='Input sensor_msgs/msg/PointCloud2 topic.'),
        DeclareLaunchArgument(
            'target_frame',
            default_value='base_footprint',
            description='Stable robot frame where ROI bounds are interpreted.'),
        DeclareLaunchArgument(
            'use_cpp',
            default_value='true',
            description='Use the C++ detector; false keeps the Python detector for rollback.'),
        Node(
            package='humanoid_navigation2_cpp_nodes',
            executable='roi_obstacle_detector_cpp',
            name='roi_obstacle_detector',
            output='screen',
            condition=IfCondition(PythonExpression(["'", use_cpp, "' == 'true'"])),
            parameters=[
                params_file,
                {
                    'input_topic': input_topic,
                    'target_frame': target_frame,
                },
            ],
        ),
        Node(
            package='humanoid_roi_obstacle_detector',
            executable='roi_obstacle_detector',
            name='roi_obstacle_detector',
            output='screen',
            condition=IfCondition(PythonExpression(["'", use_cpp, "' != 'true'"])),
            parameters=[
                params_file,
                {
                    'input_topic': input_topic,
                    'target_frame': target_frame,
                },
            ],
        ),
    ])
