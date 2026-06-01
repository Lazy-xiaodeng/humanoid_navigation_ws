from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True
    voxelsize_fine = LaunchConfiguration("voxelsize_fine")

    axis_adapter = Node(
        package="humanoid_open3d_adapter",
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
                "voxelsize_fine": ParameterValue(voxelsize_fine, value_type=float),
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
            "required_consistent_frames": 5,
            "consistency_translation_tolerance": 0.25,
            "consistency_yaw_tolerance": 0.10,
            # 第一版大跳保护。bag 测试默认开 monitor，只观察 WOULD_*，不改变 TF。
            # 要验证保护真实影响时，把这里改成 "protect"。
            "jump_protection_mode": "monitor",              # 大跳保护模式：off=关闭；monitor=只记录不拦截；protect=真实冻结大跳更新。
            "nav_medium_correction_translation": 0.50,      # 导航中“中等平移修正”上限，单位 m；超过它就按大跳处理。
            "nav_medium_correction_yaw": 0.20,              # 导航中“中等角度修正”上限，单位 rad；约 11.5 度。
            "nav_medium_required_frames": 5,                # 中等修正需要连续稳定多少帧才真正接受。
            "nav_large_correction_translation": 0.50,       # 导航中“大跳平移”判定阈值，单位 m；大于该值默认冻结 map->odom 更新。
            "nav_large_correction_yaw": 0.20,               # 导航中“大跳角度”判定阈值，单位 rad；大于该值默认冻结 map->odom 更新。
            "allow_nav_large_jump": False,                  # 是否允许导航中大跳经连续帧确认后接受；第一版先关掉。
            "idle_large_correction_translation": 1.00,      # 空闲/讲解/到点后允许自动回正的平移上限，单位 m。
            "idle_large_correction_yaw": 0.35,              # 空闲/讲解/到点后允许自动回正的角度上限，单位 rad；约 20 度。
            "idle_large_required_frames": 5,                # 空闲阶段大修正需要连续稳定多少帧才接受。
            "allow_idle_large_jump": True,                  # 是否允许空闲阶段接受较大回正。
            "hard_reject_translation": 1.00,                # 绝对保护平移阈值，单位 m；超过后不自动接受，只进入 hold/degraded。
            "hard_reject_yaw": 0.50,                        # 绝对保护角度阈值，单位 rad；约 28.6 度。
            "large_jump_degraded_after_sec": 3.0,           # 大跳冻结持续多久后认为定位退化，单位 s；只发状态，不主动断 TF。
            "enable_spin_to_pose_guard": True,
            "navigation_status_topic": "/bag/navigation/status",
            "spin_to_pose_guard_settle_sec": 3.0,
            "spin_to_pose_guard_max_duration_sec": 8.0,
            "force_2d": True,
            "force_z": 0.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "voxelsize_fine",
            default_value="0.20",
            description="Open3D fine registration voxel size in meters for bag validation.",
        ),
        axis_adapter,
        TimerAction(period=2.0, actions=[open3d_loc]),
        TimerAction(period=3.0, actions=[bridge]),
    ])
