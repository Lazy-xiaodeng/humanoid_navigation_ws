from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True
    jump_protection_mode = LaunchConfiguration("jump_protection_mode", default="monitor")
    enable_spin_to_pose_guard = LaunchConfiguration("enable_spin_to_pose_guard", default="false")
    config_file = LaunchConfiguration(
        "config_file",
        default=PathJoinSubstitution([
            FindPackageShare("robosense_lidar_localization"),
            "config",
            "robosense_lidar_localization_bag.yaml",
        ]),
    )

    robosense = Node(
        package="robosense_lidar_localization",
        executable="robosense_lidar_localization_node",
        name="robosense_lidar_localization_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "config_file": config_file,
        }],
    )

    bridge = Node(
        package="humanoid_navigation2_cpp_nodes",
        executable="prior_map_odom_bridge_cpp",
        name="prior_map_odom_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_frame": "map",
            "odom_frame": "odom",
            "localized_frame": "base_footprint",
            "prior_pose_topic": "/prior_localization/pose",
            "prior_pose_with_covariance_topic": "/prior_localization/pose_with_covariance",
            "prior_odom_topic": "/prior_localization/robosense_odom",
            "confidence_topic": "/prior_localization/confidence",
            "require_confidence": False,
            "publish_rate": 30.0,
            "tf_lookup_timeout_sec": 0.08,
            "pose_timeout_sec": 0.8,
            "accept_zero_stamp": True,
            "allow_initial_pose": True,
            "use_odom_cache": True,
            "odom_cache_topic": "/prior_localization/robosense_input_odom",
            "odom_cache_duration_sec": 5.0,
            "odom_interpolation_max_gap_sec": 0.25,
            "odom_lookup_tolerance_sec": 0.03,
            "odom_future_wait_sec": 0.20,
            "fallback_to_tf_lookup": True,
            "max_small_correction_translation": 0.25,
            "max_small_correction_yaw": 0.12,
            "max_large_correction_translation": 3.0,
            "max_large_correction_yaw": 1.2,
            "required_consistent_frames": 5,
            "consistency_translation_tolerance": 0.25,
            "consistency_yaw_tolerance": 0.10,
            "jump_protection_mode": ParameterValue(jump_protection_mode, value_type=str),
            "nav_medium_correction_translation": 0.50,
            "nav_medium_correction_yaw": 0.20,
            "nav_medium_required_frames": 5,
            "nav_large_correction_translation": 0.50,
            "nav_large_correction_yaw": 0.20,
            "allow_nav_large_jump": False,
            "idle_large_correction_translation": 1.00,
            "idle_large_correction_yaw": 0.35,
            "idle_large_required_frames": 5,
            "allow_idle_large_jump": True,
            "hard_reject_translation": 1.00,
            "hard_reject_yaw": 0.50,
            "large_jump_degraded_after_sec": 3.0,
            "enable_spin_to_pose_guard": ParameterValue(enable_spin_to_pose_guard, value_type=bool),
            "navigation_status_topic": "/navigation/status",
            "spin_to_pose_guard_settle_sec": 3.0,
            "spin_to_pose_guard_max_duration_sec": 8.0,
            "force_2d": True,
            "force_z": 0.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "jump_protection_mode",
            default_value="monitor",
            description="bag isolated bridge protection mode: off, monitor, or protect",
        ),
        DeclareLaunchArgument(
            "enable_spin_to_pose_guard",
            default_value="false",
            description="Enable bridge spin-to-pose TF freeze guard during RO bag validation.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("robosense_lidar_localization"),
                "config",
                "robosense_lidar_localization_bag.yaml",
            ]),
            description="RoboSense lidar_localization YAML for bag replay",
        ),
        robosense,
        TimerAction(period=1.0, actions=[bridge]),
    ])
