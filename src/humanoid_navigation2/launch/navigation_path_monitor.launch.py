from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    plan_topic = LaunchConfiguration("plan_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    push_topic = LaunchConfiguration("push_topic")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")

    return LaunchDescription([
        DeclareLaunchArgument("plan_topic", default_value="/plan"),
        DeclareLaunchArgument("pose_topic", default_value="/robot_realpose"),
        DeclareLaunchArgument("push_topic", default_value="/integration/push_messages"),
        DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
        Node(
            package="humanoid_navigation2",
            executable="navigation_path_monitor",
            name="navigation_path_monitor",
            output="screen",
            parameters=[{
                "plan_topic": plan_topic,
                "pose_topic": pose_topic,
                "push_topic": push_topic,
                "publish_rate_hz": publish_rate_hz,
            }],
        ),
    ])
