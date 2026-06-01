# Experimental Bridge Replay Report

## Summary

- actual recorded max jump: 1.884 m
- current simulated max jump: 2.179 m
- protected simulated max jump: 0.843 m
- protected held candidates: 249
- protected degraded events: 76
- protected idle large accepts: 8
- protected nav large accepts: 0

## Per Point

| point | actual nav max | current nav max | protected nav max | protected recovery max | protected nav >=0.5m | failed |
|---|---:|---:|---:|---:|---:|---:|
| 点位1 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | False |
| 点位2 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | False |
| 点位3 | 0.278 | 0.278 | 0.278 | 0.000 | 0 | False |
| 点位4 | 0.000 | 0.000 | 0.000 | 0.067 | 0 | False |
| 点位5 | 0.109 | 0.109 | 0.109 | 0.000 | 0 | False |
| 点位6 | 0.123 | 0.123 | 0.123 | 0.000 | 0 | False |
| 点位7 | 0.134 | 0.134 | 0.134 | 0.000 | 0 | False |
| 点位8 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | False |
| 点位9 | 0.175 | 0.175 | 0.175 | 0.056 | 0 | False |
| 点位10 | 0.123 | 0.130 | 0.130 | 0.112 | 0 | False |
| 点位11 | 0.063 | 0.227 | 0.227 | 0.000 | 0 | False |
| 点位12 | 0.221 | 0.221 | 0.105 | 0.071 | 0 | False |
| 点位13 | 0.173 | 0.173 | 0.173 | 0.101 | 0 | False |
| 点位14 | 0.177 | 0.177 | 0.177 | 0.117 | 0 | False |
| 点位15 | 0.561 | 0.561 | 0.310 | 0.082 | 0 | False |
| 点位16 | 0.253 | 1.414 | 0.000 | 0.122 | 0 | False |
| 点位17 | 1.884 | 2.179 | 0.000 | 0.172 | 0 | False |

## Protected Idle Large Accepts

- 1780201080.734 idle dx=0.289 yaw=0.036 pending_age=0.38s
- 1780201114.726 idle dx=0.301 yaw=0.011 pending_age=0.40s
- 1780201473.730 idle dx=0.668 yaw=0.032 pending_age=0.36s
- 1780201556.248 idle dx=0.819 yaw=0.056 pending_age=0.41s
- 1780201681.726 idle dx=0.843 yaw=0.050 pending_age=0.39s
- 1780201714.222 idle dx=0.705 yaw=0.044 pending_age=0.40s
- 1780201774.248 idle dx=0.532 yaw=0.020 pending_age=0.43s
- 1780201969.742 idle dx=0.337 yaw=0.027 pending_age=0.38s

## Protected Notable Events

### idle
- 1780200762.210 reject no_confidence dx=0.000 yaw=0.000 age=0.00s
- 1780200766.502 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780200803.904 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780200808.711 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780200815.508 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780200847.604 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780200891.020 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780200927.211 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780200953.217 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780200971.211 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780200992.418 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780200992.912 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780201000.720 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780201016.814 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201016.914 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201017.016 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201017.113 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201017.226 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201017.341 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201017.440 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位2
- 1780201074.824 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201074.931 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201075.018 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201075.117 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201075.214 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201075.313 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201075.411 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201075.511 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201075.616 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201075.722 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201075.829 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201075.914 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201076.010 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201076.109 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201076.217 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201076.313 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201076.411 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201076.509 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201076.610 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201076.721 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位3
- 1780201098.318 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201098.428 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201098.528 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201098.614 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201098.736 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201098.820 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201098.923 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201099.013 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201099.112 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201099.228 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201099.323 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201099.438 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201099.531 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201099.616 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201099.717 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201099.817 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201099.932 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201100.017 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201100.113 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201100.220 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位4
- 1780201156.319 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201156.427 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201156.528 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201156.615 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201156.723 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201156.823 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201156.918 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201157.019 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201157.128 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201157.227 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201157.333 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201157.442 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201157.528 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201157.622 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201157.722 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201157.818 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201157.921 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201158.016 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201158.111 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201158.236 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位5
- 1780201217.315 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201217.435 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201217.533 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201217.625 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201217.728 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201217.821 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201217.927 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201218.028 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201218.147 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201218.238 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201218.327 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201218.426 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201218.514 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201218.611 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201218.722 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201218.820 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201218.922 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201219.034 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201219.138 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201219.239 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位6
- 1780201244.818 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201244.913 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201245.018 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201245.113 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201245.226 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201245.328 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201245.435 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201245.528 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201245.621 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201245.725 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201245.830 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201245.923 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201246.030 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201246.124 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201246.220 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201246.323 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201246.424 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201246.512 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201246.610 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201246.716 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位7
- 1780201301.816 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201301.932 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201302.029 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201302.137 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201302.230 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201302.324 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201302.420 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201302.516 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201302.628 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201302.723 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201302.818 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201302.924 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201303.013 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201303.114 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201303.220 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201303.327 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201303.428 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201303.528 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201303.620 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201303.709 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位8
- 1780201372.012 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780201373.819 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201373.922 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201374.029 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201374.119 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201374.223 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201374.311 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201374.425 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201374.512 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201374.611 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201374.720 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201374.820 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201374.934 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201375.017 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201375.113 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201375.234 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201375.326 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201375.439 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201375.523 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201375.624 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位9
- 1780201432.832 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201432.922 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201433.038 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201433.111 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201433.227 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201433.333 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201433.424 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201433.519 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201433.618 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201433.733 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201433.820 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201433.920 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201434.019 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201434.129 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201434.245 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201434.320 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201434.420 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201434.526 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201434.620 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201434.726 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位10
- 1780201453.333 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201453.430 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201453.532 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201453.630 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201453.728 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201453.834 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201453.942 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201454.038 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201454.136 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201454.225 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201454.333 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201454.432 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201454.528 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201454.631 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201454.728 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201454.824 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201454.937 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201455.036 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201455.124 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201455.227 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位11
- 1780201515.330 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201515.437 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201515.532 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201515.634 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201515.747 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201515.840 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201515.928 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201516.024 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201516.134 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201516.229 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201516.325 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201516.428 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201516.530 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201516.640 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201516.742 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201516.848 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201516.940 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201517.024 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201517.130 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201517.238 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位12
- 1780201528.627 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201528.721 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201528.826 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201528.925 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201529.026 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201529.129 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201529.230 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201529.319 hold nav_large_blocked dx=0.612 yaw=0.025 age=0.00s
- 1780201529.431 hold nav_large_blocked dx=0.612 yaw=0.025 age=0.11s
- 1780201529.521 hold nav_large_blocked dx=0.612 yaw=0.025 age=0.20s
- 1780201529.621 hold nav_large_blocked dx=0.714 yaw=0.029 age=0.30s
- 1780201529.728 hold nav_large_blocked dx=0.714 yaw=0.029 age=0.41s
- 1780201529.830 hold nav_large_blocked dx=0.714 yaw=0.029 age=0.51s
- 1780201529.920 hold nav_large_blocked dx=0.714 yaw=0.029 age=0.60s
- 1780201530.027 hold nav_large_blocked dx=0.714 yaw=0.029 age=0.71s
- 1780201530.128 hold nav_large_blocked dx=0.714 yaw=0.029 age=0.81s
- 1780201530.233 hold nav_large_blocked dx=0.714 yaw=0.029 age=0.91s
- 1780201530.334 hold nav_large_blocked dx=0.714 yaw=0.029 age=1.02s
- 1780201530.419 hold nav_large_blocked dx=0.714 yaw=0.029 age=1.10s
- 1780201530.538 hold nav_large_blocked dx=0.761 yaw=0.032 age=1.22s
### 点位13
- 1780201539.828 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201539.922 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201540.019 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201540.143 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201540.241 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201549.325 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201549.432 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201549.538 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201549.631 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201549.726 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201549.833 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201549.918 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201550.019 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201550.133 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201550.226 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201550.346 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201550.432 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201550.532 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201550.640 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201550.721 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位14
- 1780201709.335 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201709.442 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201709.556 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201709.652 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201709.731 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201709.838 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201709.936 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201710.040 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201710.154 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201710.243 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201710.341 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201710.437 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201710.540 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201710.629 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201710.734 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201710.827 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201710.933 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201711.021 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位15
- 1780201757.822 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201757.924 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201758.024 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201758.121 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201758.221 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201758.327 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201758.462 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201758.542 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201758.656 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201758.764 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201758.871 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201758.942 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201759.045 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201759.140 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201759.233 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201759.328 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201759.442 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201759.544 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201759.640 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201759.740 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位16
- 1780201838.338 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201838.439 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201838.537 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201838.633 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201838.737 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201838.852 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201838.961 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201839.031 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201839.120 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201839.231 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201839.335 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201839.440 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201839.546 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201839.651 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201839.739 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201839.841 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201839.941 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201840.037 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201840.133 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201840.224 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位17
- 1780201950.829 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201950.938 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201951.049 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201951.144 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201951.243 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201951.340 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201951.443 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201951.535 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201951.639 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201951.751 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201951.854 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201951.948 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201952.029 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201952.127 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201952.230 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201952.340 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201952.433 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201952.526 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201952.626 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780201952.735 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
