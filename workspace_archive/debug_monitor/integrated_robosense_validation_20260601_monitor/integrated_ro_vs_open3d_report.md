# Integrated RoboSense(ro) vs Open3D(op) Bag Validation

RoboSense run dir: `/home/ubuntu/humanoid_ws/debug_monitor/integrated_robosense_validation_20260601_monitor`
Open3D baseline dir: `/home/ubuntu/humanoid_ws/debug_monitor/robosense_open3d_validation_20260531_full`

| bag | method | samples | map->odom max xy(m) | map->odom max yaw(deg) | prior max xy(m) | prior max yaw(deg) | status |
|---|---:|---:|---:|---:|---:|---:|---|
| nav_drift_test23 | ro integrated | 956 | 1.2315 | 4.13 | 1.1627 | 94.07 | {"WAITING": 3, "ACCEPTED": 780, "REJECTED": 151, "PENDING": 6, "SPIN_GUARD": 4} |
| nav_drift_test23 | op baseline | 959 | 1.0932 | 2.87 | 1.2015 | 76.22 | {"WAITING": 5, "ACCEPTED": 785, "SPIN_GUARD": 17, "REJECTED": 133, "PENDING": 3} |
| nav_drift_test24 | ro integrated | 857 | 1.7069 | 3.95 | 1.1219 | 77.35 | {"WAITING": 2, "ACCEPTED": 671, "SPIN_GUARD": 66, "REJECTED": 101, "PENDING": 5} |
| nav_drift_test24 | op baseline | 862 | 0.4153 | 7.33 | 0.8662 | 74.86 | {"WAITING": 1, "ACCEPTED": 109, "REJECTED": 20, "SPIN_GUARD": 714} |
| nav_drift_test25 | ro integrated | 1023 | 1.7296 | 8.45 | 1.0953 | 92.59 | {"WAITING": 1, "ACCEPTED": 812, "PENDING": 15, "REJECTED": 169, "SPIN_GUARD": 15} |
| nav_drift_test25 | op baseline | 1031 | 2.3808 | 6.40 | 1.2717 | 78.89 | {"WAITING": 1, "REJECTED": 152, "ACCEPTED": 804, "PENDING": 52, "SPIN_GUARD": 3, "WOULD_PENDING": 1} |

## Max Jump Detail

| bag | method | map->odom max xy time | map->odom xy status | map->odom max yaw time | map->odom yaw status |
|---|---:|---:|---|---:|---|
| nav_drift_test23 | robosense_integrated | 1780250593.011 | PENDING reset_large_candidate spread_xy=0.251 spread_yaw=0.008 | 1780250654.511 | ACCEPTED small_correction dx=0.032 yaw=0.004 map_odom_xy_norm=0.272 yaw=-0.075 |
| nav_drift_test23 | open3d | 1780243633.594 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=2.320 yaw=-0.225 | 1780243482.595 | ACCEPTED confirmed_large_correction count=5 map_odom_xy_norm=0.466 yaw=-0.147 |
| nav_drift_test24 | robosense_integrated | 1780251085.736 | ACCEPTED small_correction dx=0.106 yaw=0.004 map_odom_xy_norm=1.615 yaw=-0.195 | 1780250922.236 | ACCEPTED small_correction dx=0.002 yaw=0.000 map_odom_xy_norm=1.061 yaw=-0.101 |
| nav_drift_test24 | open3d | 1780243820.124 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=0.234 yaw=-0.115 | 1780243766.124 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=0.128 yaw=-0.128 |
| nav_drift_test25 | robosense_integrated | 1780251605.287 | ACCEPTED small_correction dx=0.113 yaw=0.005 map_odom_xy_norm=0.893 yaw=-0.087 | 1780251646.288 | ACCEPTED small_correction dx=0.006 yaw=0.000 map_odom_xy_norm=1.348 yaw=-0.097 |
| nav_drift_test25 | open3d | 1780244652.151 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=1.594 yaw=-0.125 | 1780244692.651 | ACCEPTED small_correction dx=0.000 yaw=0.000 map_odom_xy_norm=1.077 yaw=-0.039 |

## Notes

- ro integrated uses `robosense_lidar_localization` + `prior_map_odom_bridge` with `jump_protection_mode=monitor`.
- SpinToPose guard is enabled and listens to `/navigation/status` from the bag.
- Because monitor mode does not block jumps, `WOULD_*` statuses show what protect mode would have held/pended.
