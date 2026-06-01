# RoboSense vs Open3D Bag Validation Report

Run dir: `debug_monitor/robosense_open3d_validation_20260531_full`

| bag | method | samples | duration(s) | max step xy(m) | max step yaw(deg) | status/confidence |
|---|---:|---:|---:|---:|---:|---|
| nav_drift_test23 | RoboSense | 13159 | 1401.616 | 0.5395 | 37.13 | {"1.0": 6, "2.0": 13153} |
| nav_drift_test23 | Open3D bridge | 959 | 468.500 | 1.0932 | 2.87 | conf mean=0.969, min=0.849, max=1.000; {"WAITING": 5, "ACCEPTED": 785, "SPIN_GUARD": 17, "REJECTED": 133, "PENDING": 3} |
| nav_drift_test24 | RoboSense | 23174 | 1255.019 | 0.4717 | 39.33 | {"2.0": 23138, "1.0": 36} |
| nav_drift_test24 | Open3D bridge | 862 | 421.500 | 0.4153 | 7.33 | conf mean=0.947, min=0.882, max=0.987; {"WAITING": 1, "ACCEPTED": 109, "REJECTED": 20, "SPIN_GUARD": 714} |
| nav_drift_test25 | RoboSense | 41080 | 1507.718 | 0.8974 | 26.70 | {"2.0": 40886, "1.0": 194} |
| nav_drift_test25 | Open3D bridge | 1031 | 506.000 | 2.3808 | 6.40 | conf mean=0.970, min=0.859, max=1.000; {"WAITING": 1, "REJECTED": 152, "ACCEPTED": 804, "PENDING": 52, "SPIN_GUARD": 3, "WOULD_PENDING": 1} |

## Notes

- RoboSense max step is computed from `/lidar_pose_xyz` pose samples.
- Open3D max step is computed from bridge-maintained `map->odom` samples recorded by `prior_map_bag_monitor`.
- This report is a first-pass numerical summary; detailed failure timing still needs log inspection around large steps.
- Open3D `duration(s)` is monitor wall/runtime duration under 3x replay; RoboSense `duration(s)` is bag message timestamp duration.

## Max Jump Details

RoboSense:

| bag | max xy jump | max xy stamp | max yaw jump | max yaw stamp | status around jump |
|---|---:|---:|---:|---:|---|
| nav_drift_test23 | 0.5395 m | 1780221363.615 | 37.13 deg | 1780221271.913 | NORMAL -> NORMAL |
| nav_drift_test24 | 0.4717 m | 1780222521.330 | 39.33 deg | 1780223281.441 | xy: NORMAL -> NORMAL; yaw: LOW_ACCURACY -> NORMAL |
| nav_drift_test25 | 0.8974 m | 1780235044.823 | 26.70 deg | 1780236068.338 | xy: NORMAL -> LOW_ACCURACY; yaw: NORMAL -> NORMAL |

Open3D bridge:

| bag | max map->odom xy jump | monitor time | bridge status |
|---|---:|---:|---|
| nav_drift_test23 | 1.0932 m | 1780243633.594 | ACCEPTED small_correction |
| nav_drift_test24 | 0.4153 m | 1780243820.124 | ACCEPTED small_correction |
| nav_drift_test25 | 2.3808 m | 1780244652.151 | ACCEPTED small_correction |

## Generated Plots

Plots are under `debug_monitor/robosense_open3d_validation_20260531_full/plots`.

- `*_robosense_odom_vs_global.png`: Fast-LIO `/odom` initial-aligned vs RoboSense `/lidar_pose_xyz`.
- `*_open3d_prior_vs_robot_realpose.png`: current Open3D prior pose vs bag `/robot_realpose`.
- `*_open3d_map_odom_correction.png`: current Open3D bridge `map->odom` correction trajectory.

## Current Read

- RoboSense can run independently on all three bags and mostly reports NORMAL: test23 has 6 LOW_ACCURACY frames, test24 has 36, test25 has 194.
- RoboSense is not clean enough to integrate blindly yet: it still has single-frame pose jumps of 0.47-0.90 m and 27-39 deg, including jumps while status remains NORMAL.
- Open3D also shows accepted `map->odom` jumps, especially test25 at 2.38 m. This validates the original concern that bridge-side acceptance logic still needs scrutiny.
- The largest RoboSense jumps align with `/navigation/status` being idle in these bags, not active SpinToPose.
