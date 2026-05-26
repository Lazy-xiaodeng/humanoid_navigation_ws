from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare("humanoid_scancontext_global_localization")
    default_params = PathJoinSubstitution(
        [pkg_share, "config", "scancontext_global_localization.yaml"]
    )

    params_file = LaunchConfiguration("params_file")
    database_path = LaunchConfiguration("database_path")
    pcd_map_path = LaunchConfiguration("pcd_map_path")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    cloud_frame_mode = LaunchConfiguration("cloud_frame_mode")
    publish_initialpose = LaunchConfiguration("publish_initialpose")
    query_on_cloud = LaunchConfiguration("query_on_cloud")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("database_path", default_value=""),
            DeclareLaunchArgument("pcd_map_path", default_value=""),
            DeclareLaunchArgument("cloud_topic", default_value="/fast_lio/cloud_registered"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("cloud_frame_mode", default_value="registered"),
            DeclareLaunchArgument("publish_initialpose", default_value="false"),
            DeclareLaunchArgument("query_on_cloud", default_value="false"),
            Node(
                package="humanoid_scancontext_global_localization",
                executable="scancontext_global_localizer_node",
                name="scancontext_global_localizer",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "database_path": database_path,
                        "pcd_map_path": pcd_map_path,
                        "cloud_topic": cloud_topic,
                        "odom_topic": odom_topic,
                        "cloud_frame_mode": cloud_frame_mode,
                        "publish_initialpose": ParameterValue(
                            publish_initialpose, value_type=bool
                        ),
                        "query_on_cloud": ParameterValue(
                            query_on_cloud, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
