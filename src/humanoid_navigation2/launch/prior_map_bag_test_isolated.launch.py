from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True

    axis_adapter = Node(
        package="humanoid_navigation2",
        executable="fastlio_open3d_axis_adapter",
        name="fastlio_open3d_axis_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "raw_odom_topic": "/bag/odom",
            "raw_cloud_topic": "/bag/fast_lio/cloud_registered",
            "output_odom_topic": "/prior_localization/open3d_input_odom",
            "output_cloud_topic": "/prior_localization/open3d_input_cloud",
            "odom_frame": "odom",
            "output_base_frame": "prior_open3d_base",
            "publish_tf": True,
            "map_origin_x": -18.5,
            "map_origin_y": -10.5,
            "map_origin_z": 0.0,
        }],
    )

    open3d_loc = Node(
        package="open3d_loc",
        executable="global_localization_node",
        name="prior_map_open3d_localization",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("open3d_loc"), "config", "loc_param_g1.yaml"]),
            {
                "use_sim_time": use_sim_time,
                "path_map": PathJoinSubstitution([
                    FindPackageShare("humanoid_navigation2"),
                    "pcd",
                    "hall_open3d_grounded.pcd",
                ]),
                "initialpose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "kf_baselink2map/x": [0.001, 0.002],
                "kf_baselink2map/y": [0.001, 0.005],
                "kf_baselink2map/z": [0.00001, 0.04],
                "publish_tf": False,
                "pcd_queue_maxsize": 10,
                "voxelsize_coarse": 0.01,
                "voxelsize_fine": 0.20,
                "threshold_fitness": 0.50,
                "threshold_fitness_init": 0.50,
                "loc_frequence": 1.0,
                "save_scan": False,
                "hidden_removal": False,
                "maxpoints_source": 60000,
                "maxpoints_target": 250000,
                "filter_odom2map": False,
                "use_input_odom_pose_directly": True,
                "kalman_processVar2": 0.001,
                "kalman_estimatedMeasVar2": 0.02,
                "confidence_loc_th": 0.7,
                "dis_updatemap": 3.5,
            },
        ],
        remappings=[
            ("/Odometry_loc", "/prior_localization/open3d_input_odom"),
            ("/cloud_registered_1", "/prior_localization/open3d_input_cloud"),
            ("/baselink2map", "/prior_localization/odom"),
            ("/localization_3d_confidence", "/prior_localization/confidence"),
            ("/localization_3d", "/prior_localization/pose_motion_link"),
            ("/odom2map", "/prior_localization/open3d_odom2map"),
            ("/odom2map_kalman", "/prior_localization/open3d_odom2map_kalman"),
            ("/baselink2map_kalman", "/prior_localization/open3d_baselink2map_kalman"),
            ("/motionlink2map", "/prior_localization/open3d_motionlink2map"),
            ("/map", "/prior_localization/open3d_map"),
            ("/submap", "/prior_localization/open3d_submap"),
            ("/scan", "/prior_localization/open3d_scan"),
            ("/scan2map", "/prior_localization/open3d_scan2map"),
        ],
    )

    bridge = Node(
        package="humanoid_navigation2",
        executable="prior_map_odom_bridge",
        name="prior_map_odom_bridge",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_frame": "map",
            "odom_frame": "odom",
            "localized_frame": "prior_open3d_base",
            "prior_pose_topic": "/prior_localization/pose",
            "prior_pose_with_covariance_topic": "/prior_localization/pose_with_covariance",
            "prior_odom_topic": "/prior_localization/odom",
            "confidence_topic": "/prior_localization/confidence",
            "require_confidence": True,
            "min_confidence": 0.50,
            "confidence_timeout_sec": 2.0,
            "publish_rate": 30.0,
            "tf_lookup_timeout_sec": 0.08,
            "pose_timeout_sec": 0.8,
            "accept_zero_stamp": True,
            "allow_initial_pose": True,
            "use_odom_cache": True,
            "odom_cache_topic": "/prior_localization/open3d_input_odom",
            "odom_cache_duration_sec": 5.0,
            "odom_interpolation_max_gap_sec": 0.25,
            "odom_lookup_tolerance_sec": 0.03,
            "odom_future_wait_sec": 0.20,
            "fallback_to_tf_lookup": True,
            "max_small_correction_translation": 0.25,
            "max_small_correction_yaw": 0.12,
            "max_large_correction_translation": 3.0,
            "max_large_correction_yaw": 1.2,
            "required_consistent_frames": 3,
            "consistency_translation_tolerance": 0.25,
            "consistency_yaw_tolerance": 0.10,
            "enable_spin_to_pose_guard": True,
            "navigation_status_topic": "/bag/navigation/status",
            "spin_to_pose_guard_settle_sec": 3.0,
            "spin_to_pose_guard_max_duration_sec": 8.0,
            "force_2d": True,
            "force_z": 0.0,
        }],
    )

    return LaunchDescription([
        axis_adapter,
        TimerAction(period=2.0, actions=[open3d_loc]),
        TimerAction(period=3.0, actions=[bridge]),
    ])
