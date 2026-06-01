# Integrated RoboSense(ro) vs Open3D(op) Bag Validation

RoboSense run dir: `/home/ubuntu/humanoid_ws/debug_monitor/integrated_voxel01_validation_20260601`
Open3D baseline dir: `/home/ubuntu/humanoid_ws/debug_monitor/integrated_voxel01_validation_20260601`

| bag | method | samples | map->odom max xy(m) | map->odom max yaw(deg) | prior max xy(m) | prior max yaw(deg) | status |
|---|---:|---:|---:|---:|---:|---:|---|
| nav_drift_test23 | ro integrated | 956 | 1.2038 | 4.30 | 1.2348 | 77.86 | {"WAITING": 2, "ACCEPTED": 774, "REJECTED": 145, "PENDING": 16, "SPIN_GUARD": 6} |
| nav_drift_test23 | op baseline | 961 | 1.1301 | 4.61 | 1.3417 | 75.00 | {"WAITING": 3, "REJECTED": 152, "ACCEPTED": 772, "PENDING": 17} |
| nav_drift_test24 | ro integrated | 857 | 1.7335 | 3.32 | 1.2342 | 83.31 | {"WAITING": 2, "ACCEPTED": 660, "REJECTED": 167, "PENDING": 12, "SPIN_GUARD": 4} |
| nav_drift_test24 | op baseline | 861 | 0.1433 | 7.38 | 0.8228 | 70.36 | {"WAITING": 2, "ACCEPTED": 75, "REJECTED": 7, "PENDING": 3, "SPIN_GUARD": 758} |
| nav_drift_test25 | ro integrated | 1023 | 1.0468 | 6.01 | 0.0007 | 0.01 | {"WAITING": 1, "ACCEPTED": 1, "PENDING": 174, "SPIN_GUARD": 837} |
| nav_drift_test25 | op baseline | 1029 | 1.7298 | 5.51 | 1.3024 | 81.00 | {"WAITING": 3, "REJECTED": 163, "ACCEPTED": 822, "PENDING": 22, "SPIN_GUARD": 1, "WOULD_PENDING": 2} |

## Max Jump Detail

| bag | method | map->odom max xy time | map->odom xy status | map->odom max yaw time | map->odom yaw status |
|---|---:|---:|---|---:|---|
| nav_drift_test23 | robosense_integrated | 1780279812.939 | ACCEPTED small_correction dx=0.019 yaw=0.001 map_odom_xy_norm=0.702 yaw=-0.117 | 1780280053.439 | ACCEPTED small_correction dx=0.018 yaw=0.002 map_odom_xy_norm=0.160 yaw=-0.074 |
| nav_drift_test23 | open3d | 1780281423.375 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=2.226 yaw=-0.220 | 1780281486.374 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=0.318 yaw=-0.078 |
| nav_drift_test24 | robosense_integrated | 1780280485.055 | ACCEPTED small_correction dx=0.119 yaw=0.004 map_odom_xy_norm=1.518 yaw=-0.191 | 1780280321.555 | ACCEPTED small_correction dx=0.014 yaw=0.001 map_odom_xy_norm=1.042 yaw=-0.102 |
| nav_drift_test24 | open3d | 1780281556.342 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=0.143 yaw=-0.129 | 1780281556.342 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=0.143 yaw=-0.129 |
| nav_drift_test25 | robosense_integrated | 1780280552.106 | PENDING large_correction count=4/5 | 1780280552.106 | PENDING large_correction count=4/5 |
| nav_drift_test25 | open3d | 1780282464.986 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=1.626 yaw=-0.059 | 1780282476.486 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=0.794 yaw=0.129 |

## Notes

- ro integrated uses `robosense_lidar_localization` + `prior_map_odom_bridge` with `jump_protection_mode=monitor`.
- SpinToPose guard is enabled and listens to `/navigation/status` from the bag.
- Because monitor mode does not block jumps, `WOULD_*` statuses show what protect mode would have held/pended.
