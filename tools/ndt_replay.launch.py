import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_lidar_loc = get_package_share_directory("lidar_localization_ros2")
    pkg_nav2 = get_package_share_directory("humanoid_navigation2")

    use_sim_time = LaunchConfiguration("use_sim_time")
    recovery_gate = LaunchConfiguration("recovery_gate")

    localization_params_file = os.path.join(pkg_lidar_loc, "param", "localization.yaml")

    ndt_localization_node = Node(
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        name="lidar_localization",
        output="screen",
        parameters=[
            localization_params_file,
            {
                "use_sim_time": use_sim_time,
                "set_initial_pose": True,
                "initial_pose_x": 0.11854200810194016,
                "initial_pose_y": 0.18987514078617096,
                "initial_pose_z": 0.0,
                "initial_pose_qz": 0.0041739433708416615,
                "initial_pose_qw": 0.9999912827827067,
                "score_threshold": 0.3,
                "reject_pose_jump": True,
                "max_pose_jump_translation": 0.50,
                "max_pose_jump_yaw": 0.40,
                "initialpose_relax_duration_sec": 4.0,
                "initialpose_max_pose_jump_translation": 2.00,
                "initialpose_max_pose_jump_yaw": 3.00,
                "pose_jump_reacquire_enabled": True,
                "pose_jump_reacquire_max_translation": 1.50,
                "pose_jump_reacquire_max_yaw": 0.12,
                "pose_jump_reacquire_max_fitness": 0.03,
                "pose_jump_reacquire_required_frames": 4,
                "pose_jump_reacquire_xy_tolerance": 0.30,
                "pose_jump_reacquire_yaw_tolerance": 0.08,
                "rotation_guard_enabled": True,
                "rotation_guard_use_cmd_vel_fallback": True,
                "rotation_guard_freeze_corrections": True,
                "rotation_guard_recovery_gate_enabled": recovery_gate,
                "rotation_guard_navigation_status_topic": "/navigation/status",
                "rotation_guard_angular_threshold": 0.20,
                "rotation_guard_linear_threshold": 0.05,
                "rotation_guard_settle_sec": 2.0,
                "rotation_guard_max_duration_sec": 8.0,
                "rotation_guard_recovery_max_translation": 0.15,
                "rotation_guard_recovery_max_yaw": 0.05,
                "rotation_guard_recovery_required_frames": 4,
                "rotation_guard_recovery_max_duration_sec": 6.0,
                "multi_frame_matching_enabled": True,
                "multi_frame_use_only_when_rotating": True,
                "multi_frame_window_sec": 0.6,
                "multi_frame_max_frames": 8,
                "multi_frame_voxel_leaf_size": 0.20,
                "multi_frame_max_points": 40000,
                "min_scan_points": 50,
                "localization_status_topic": "/localization/ndt_status",
                "republish_last_good_tf_on_failure": True,
                "max_last_good_tf_age_sec": 5.0,
                "use_fastlio_delta_guess": True,
                "fastlio_delta_guess_mode": "map_body_to_map_odom",
                "fastlio_camera_frame": "camera_init",
                "fastlio_body_frame": "body",
                "tf_max_stamp_mismatch_sec": 0.2,
                "fastlio_max_delta_translation": 0.20,
                "fastlio_max_delta_yaw": 0.25,
                "fastlio_max_delta_dt": 0.50,
                "fastlio_max_dead_reckon_sec": 2.0,
                "ndt_outlier_ratio": 0.30,
                "ndt_max_corr_dist": 2.0,
                "ndt_rotation_prior_enabled": True,
                "ndt_rotation_prior_weight": 10.0,
                "ndt_rotation_prior_roll_pitch_only": True,
                "map_path": os.path.join(pkg_nav2, "pcd", "hall.pcd"),
            },
        ],
        remappings=[("/cloud", "/fast_lio/cloud_registered")],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_ndt",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": ["lidar_localization"],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("recovery_gate", default_value="false"),
            ndt_localization_node,
            lifecycle_manager,
        ]
    )
