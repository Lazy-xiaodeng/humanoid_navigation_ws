from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    command_topic = LaunchConfiguration("command_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    push_topic = LaunchConfiguration("push_topic")
    command_rate_hz = LaunchConfiguration("command_rate_hz")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")

    return LaunchDescription([
        DeclareLaunchArgument("command_topic", default_value="/app/robot_control"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("pose_topic", default_value="/robot_realpose"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("push_topic", default_value="/integration/push_messages"),
        DeclareLaunchArgument("command_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
        Node(
            package="humanoid_navigation2",
            executable="gait_velocity_test_monitor",
            name="gait_velocity_test_monitor",
            output="screen",
            parameters=[{
                "command_topic": command_topic,
                "cmd_vel_topic": cmd_vel_topic,
                "pose_topic": pose_topic,
                "odom_topic": odom_topic,
                "push_topic": push_topic,
                "command_rate_hz": command_rate_hz,
                "publish_rate_hz": publish_rate_hz,
            }],
        ),
    ])
