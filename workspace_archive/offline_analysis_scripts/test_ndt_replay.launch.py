"""Minimal NDT launch for offline bag replay testing."""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    loc_params = "/home/ubuntu/humanoid_ws/src/lidar_localization/param/localization.yaml"

    ndt_node = Node(
        package='lidar_localization_ros2',
        executable='lidar_localization_node',
        name='lidar_localization',
        namespace='',
        parameters=[loc_params, {
            'use_sim_time': True,
            'set_initial_pose': True,
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_pose_z': 0.0,
            'score_threshold': 0.3,
            'reject_pose_jump': True,
            'max_pose_jump_translation': 0.50,
            'max_pose_jump_yaw': 0.40,
            'initialpose_relax_duration_sec': 4.0,
            'initialpose_max_pose_jump_translation': 2.00,
            'initialpose_max_pose_jump_yaw': 3.00,
            'pose_jump_reacquire_enabled': True,
            'pose_jump_reacquire_max_translation': 0.50,
            'pose_jump_reacquire_max_yaw': 0.30,
            'pose_jump_reacquire_max_fitness': 0.08,
            'use_fastlio_delta_guess': True,
            'fastlio_camera_frame': 'camera_init',
            'fastlio_body_frame': 'body',
            'fastlio_max_delta_translation': 0.20,
            'fastlio_max_delta_yaw': 0.25,
            'fastlio_max_dead_reckon_sec': 2.0,
            'republish_last_good_tf_on_failure': True,
            'max_last_good_tf_age_sec': 5.0,
            'ndt_outlier_ratio': 0.30,
            'ndt_max_corr_dist': 2.0,
            'ndt_rotation_prior_enabled': True,
            'ndt_rotation_prior_weight': 10.0,
            'use_odom': False,
            'enable_debug': True,
        }],
        output='screen',
    )

    return LaunchDescription([ndt_node])
