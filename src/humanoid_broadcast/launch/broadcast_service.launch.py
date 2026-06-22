from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="humanoid_broadcast",
            executable="broadcast_service_node",
            name="xiaorui_broadcast_service",
            output="screen",
        ),
    ])
