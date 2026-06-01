# Integrated RoboSense(ro) vs Open3D(op) Bag Validation

RoboSense run dir: `/home/ubuntu/humanoid_ws/debug_monitor/integrated_robosense_validation_20260601_protect`
Open3D baseline dir: `/home/ubuntu/humanoid_ws/debug_monitor/robosense_open3d_validation_20260531_full`

| bag | method | samples | map->odom max xy(m) | map->odom max yaw(deg) | prior max xy(m) | prior max yaw(deg) | status |
|---|---:|---:|---:|---:|---:|---:|---|
| nav_drift_test23 | ro integrated | 955 | 1.0806 | 3.75 | 1.3230 | 80.52 | {"WAITING": 2, "ACCEPTED": 665, "REJECTED": 142, "PENDING": 6, "HOLD": 21, "DEGRADED": 105, "SPIN_GUARD": 2} |
| nav_drift_test23 | op baseline | 959 | 1.0932 | 2.87 | 1.2015 | 76.22 | {"WAITING": 5, "ACCEPTED": 785, "SPIN_GUARD": 17, "REJECTED": 133, "PENDING": 3} |
| nav_drift_test24 | ro integrated | 857 | 1.2373 | 3.11 | 1.2388 | 101.48 | {"WAITING": 2, "DEGRADED": 212, "ACCEPTED": 418, "REJECTED": 160, "HOLD": 33, "SPIN_GUARD": 18, "PENDING": 2} |
| nav_drift_test24 | op baseline | 862 | 0.4153 | 7.33 | 0.8662 | 74.86 | {"WAITING": 1, "ACCEPTED": 109, "REJECTED": 20, "SPIN_GUARD": 714} |
| nav_drift_test25 | ro integrated | 1023 | 0.1912 | 0.37 | 0.0000 | 0.00 | {"WAITING": 1, "ACCEPTED": 175, "SPIN_GUARD": 837} |
| nav_drift_test25 | op baseline | 1031 | 2.3808 | 6.40 | 1.2717 | 78.89 | {"WAITING": 1, "REJECTED": 152, "ACCEPTED": 804, "PENDING": 52, "SPIN_GUARD": 3, "WOULD_PENDING": 1} |

## Max Jump Detail

| bag | method | map->odom max xy time | map->odom xy status | map->odom max yaw time | map->odom yaw status |
|---|---:|---:|---|---:|---|
| nav_drift_test23 | robosense_integrated | 1780275209.700 | ACCEPTED small_correction dx=0.004 yaw=0.000 map_odom_xy_norm=1.579 yaw=-0.209 | 1780275059.200 | ACCEPTED confirmed_idle_large_correction count=5 map_odom_xy_norm=0.139 yaw=-0.114 |
| nav_drift_test23 | open3d | 1780243633.594 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=2.320 yaw=-0.225 | 1780243482.595 | ACCEPTED confirmed_large_correction count=5 map_odom_xy_norm=0.466 yaw=-0.147 |
| nav_drift_test24 | robosense_integrated | 1780275457.610 | REJECTED spin_to_pose_freeze_tf phase=settle | 1780275531.610 | REJECTED spin_to_pose_freeze_tf phase=settle |
| nav_drift_test24 | open3d | 1780243820.124 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=0.234 yaw=-0.115 | 1780243766.124 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=0.128 yaw=-0.128 |
| nav_drift_test25 | robosense_integrated | 1780275824.824 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=0.147 yaw=-0.144 | 1780275824.824 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=0.147 yaw=-0.144 |
| nav_drift_test25 | open3d | 1780244652.151 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=1.594 yaw=-0.125 | 1780244692.651 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=1.077 yaw=-0.039 |

## Notes

- ro integrated uses `robosense_lidar_localization` + `prior_map_odom_bridge` with `jump_protection_mode=monitor`.
- SpinToPose guard is enabled and listens to `/navigation/status` from the bag.
- Because monitor mode does not block jumps, `WOULD_*` statuses show what protect mode would have held/pended.
