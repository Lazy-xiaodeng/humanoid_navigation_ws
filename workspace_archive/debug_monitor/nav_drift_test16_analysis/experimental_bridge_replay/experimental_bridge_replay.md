# Experimental Bridge Replay Report

## Summary

- actual recorded max jump: 2.760 m
- current simulated max jump: 2.760 m
- protected simulated max jump: 2.758 m
- protected held candidates: 147
- protected degraded events: 7
- protected idle large accepts: 12
- protected nav large accepts: 0

## Per Point

| point | actual nav max | current nav max | protected nav max | protected recovery max | protected nav >=0.5m | failed |
|---|---:|---:|---:|---:|---:|---:|
| 点位1 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | False |
| 点位2 | 0.169 | 0.169 | 0.169 | 0.000 | 0 | False |
| 点位3 | 0.192 | 0.192 | 0.192 | 0.000 | 0 | False |
| 点位4 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | False |
| 点位5 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | False |
| 点位6 | 0.198 | 0.198 | 0.198 | 0.000 | 0 | False |
| 点位7 | 0.139 | 0.139 | 0.139 | 0.000 | 0 | False |
| 点位8 | 0.155 | 0.155 | 0.155 | 0.000 | 0 | False |
| 点位9 | 0.215 | 0.215 | 0.215 | 0.051 | 0 | False |
| 点位10 | 0.249 | 0.249 | 0.249 | 0.076 | 0 | False |
| 点位11 | 0.455 | 0.455 | 0.455 | 0.000 | 0 | False |
| 点位12 | 0.268 | 0.268 | 0.268 | 0.075 | 0 | False |
| 点位13 | 0.415 | 0.415 | 0.415 | 0.254 | 0 | False |
| 点位14 | 0.888 | 0.888 | 0.483 | 0.579 | 0 | False |
| 点位15 | 1.598 | 1.598 | 0.000 | 0.095 | 0 | False |
| 点位16 | 1.368 | 2.232 | 0.000 | 0.000 | 0 | False |
| 点位17 | 1.110 | 1.110 | 0.000 | 0.131 | 0 | False |
| 点位18 | 2.220 | 2.220 | 0.000 | 0.000 | 0 | False |
| 点位19 | 2.760 | 2.760 | 0.000 | 0.000 | 0 | False |
| 点位20 | 1.224 | 1.224 | 0.000 | 0.000 | 0 | False |
| 点位21 | 0.459 | 0.459 | 0.000 | 0.000 | 0 | False |
| 点位22 | 0.577 | 0.577 | 0.000 | 0.000 | 0 | True |

## Protected Idle Large Accepts

- 1780137582.650 idle dx=0.388 yaw=0.053 pending_age=0.20s
- 1780137637.647 idle dx=0.527 yaw=0.065 pending_age=0.20s
- 1780137837.650 idle dx=0.349 yaw=0.015 pending_age=0.18s
- 1780137881.156 idle dx=0.463 yaw=0.047 pending_age=0.21s
- 1780137920.173 idle dx=0.304 yaw=0.018 pending_age=0.21s
- 1780138112.165 idle dx=0.883 yaw=0.031 pending_age=0.20s
- 1780138112.851 idle dx=0.254 yaw=0.009 pending_age=0.19s
- 1780138241.162 idle dx=0.505 yaw=0.002 pending_age=0.20s
- 1780138242.878 idle dx=0.579 yaw=0.030 pending_age=0.22s
- 1780138501.157 idle dx=2.758 yaw=0.101 pending_age=0.19s
- 1780138509.777 idle dx=0.262 yaw=0.008 pending_age=0.21s
- 1780138536.776 idle dx=0.251 yaw=0.009 pending_age=0.20s

## Protected Notable Events

### idle
- 1780137159.154 reject no_confidence dx=0.000 yaw=0.000 age=0.00s
- 1780137221.637 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137222.540 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137274.542 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137339.840 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137371.456 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137374.872 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137386.452 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137388.351 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137392.463 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137432.746 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137433.545 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137445.342 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780137544.352 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137544.447 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137544.552 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137544.647 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137544.748 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137544.864 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137544.956 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位1
- 1780137543.953 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137544.053 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137544.150 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137544.256 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位2
- 1780137557.453 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137557.547 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137557.647 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137557.748 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137557.848 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137557.950 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137558.052 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137558.151 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137558.250 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137558.350 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137558.450 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137558.547 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137558.648 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137558.752 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137558.852 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137558.945 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137559.048 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137559.151 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137559.248 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137559.354 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位3
- 1780137621.949 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137622.051 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137622.155 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137622.249 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137622.352 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137622.449 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137622.547 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137622.651 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137622.751 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137622.853 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137622.950 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137623.053 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137623.151 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137623.252 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137623.348 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137623.465 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137623.548 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137623.646 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137623.761 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137623.857 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位4
- 1780137679.948 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137680.049 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137680.165 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137680.265 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137680.353 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137680.460 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137680.549 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137680.649 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137680.752 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137680.848 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137680.950 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137681.049 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137681.171 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137681.262 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137681.365 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137681.457 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137681.547 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137681.649 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137681.747 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137681.847 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位5
- 1780137742.460 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137742.559 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137742.647 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137742.757 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137742.854 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137742.978 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137743.084 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137743.171 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137743.274 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137743.363 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137743.461 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137743.566 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137743.656 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137743.763 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137743.849 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137743.955 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137744.050 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137744.155 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137744.258 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137744.362 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位6
- 1780137771.965 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137772.057 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137772.148 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137772.265 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137772.356 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137772.459 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137772.563 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137772.672 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137772.784 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137772.871 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137772.949 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137773.052 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137773.147 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137773.253 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137773.354 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137773.450 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137773.553 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137773.652 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137773.757 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137773.856 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位7
- 1780137821.466 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137821.556 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137821.660 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137821.763 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137821.853 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137821.948 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137822.056 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137822.150 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137822.274 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137822.357 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137822.466 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137822.557 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137822.660 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137822.750 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137822.864 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137822.957 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137823.049 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137823.148 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137823.256 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137823.358 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位8
- 1780137857.955 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137858.051 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137858.147 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137858.252 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137858.350 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137858.468 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137858.558 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137858.658 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137858.751 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137858.855 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137858.952 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137859.046 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137859.157 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137859.256 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137859.361 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137859.468 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137859.558 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137859.662 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137859.759 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137859.863 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位9
- 1780137900.468 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137900.561 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137900.678 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137900.773 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137900.865 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137900.985 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137901.080 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137901.167 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137901.270 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137901.382 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137901.471 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137901.578 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137901.680 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137901.760 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137901.848 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137901.952 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137902.048 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137902.163 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137902.269 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137902.378 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位10
- 1780137999.481 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137999.567 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137999.660 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137999.758 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137999.860 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780137999.970 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138000.069 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138000.173 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138000.258 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138000.366 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138000.463 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138000.564 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138000.672 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138000.758 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138000.869 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138000.971 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138001.063 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138001.182 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138001.267 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138001.350 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位11
- 1780138060.979 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138061.067 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138061.172 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138061.269 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138061.369 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138061.485 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138061.569 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138061.657 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138061.763 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138061.864 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138061.966 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138062.056 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138062.147 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138062.257 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138062.362 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138062.476 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138062.556 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780138062.690 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138062.788 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138062.897 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位12
- 1780138073.472 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138073.568 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138073.661 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138073.764 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138073.853 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138073.956 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138074.052 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138074.154 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138074.251 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138074.356 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138074.466 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138074.559 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138074.655 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138074.750 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138074.856 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138074.962 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138075.052 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138075.147 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138075.253 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138075.357 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位13
- 1780138095.474 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138095.569 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138095.677 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138095.768 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138095.869 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138095.955 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138096.056 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138096.157 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138096.257 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138096.373 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138105.461 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138105.555 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138105.655 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138105.765 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138105.863 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138105.961 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138106.065 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138106.156 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138106.252 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138106.352 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位14
- 1780138228.962 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138229.075 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138229.171 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138229.268 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138229.372 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138229.475 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138229.572 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138229.661 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138229.758 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138229.864 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138229.971 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138230.064 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138230.162 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138230.293 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138230.383 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138230.474 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138230.563 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138230.677 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138230.781 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138230.885 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位15
- 1780138308.975 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138309.061 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138309.158 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138309.257 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138309.360 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138309.472 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138309.559 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138309.668 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138309.761 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138309.865 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138309.967 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138310.089 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138310.188 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138310.272 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138310.365 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138310.471 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138310.583 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138310.665 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138310.762 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138310.867 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位16
- 1780138382.468 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138382.574 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138382.662 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138382.760 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138382.860 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138382.957 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138383.060 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138383.164 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138383.262 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138383.361 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138383.476 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138383.578 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138383.661 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138383.763 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138383.857 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138383.964 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138384.059 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138384.162 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138384.260 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138384.361 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位17
- 1780138472.710 reject too_large dx=3.838 yaw=0.164 age=0.00s
- 1780138472.814 reject too_large dx=3.838 yaw=0.164 age=0.00s
- 1780138472.900 reject too_large dx=3.838 yaw=0.164 age=0.00s
- 1780138472.968 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138473.062 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138473.185 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138473.288 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138473.382 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138473.476 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138473.574 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138473.671 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138473.771 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138473.876 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138473.969 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138474.063 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138474.168 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138474.263 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138474.358 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138474.469 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138474.577 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位18
- 1780138564.489 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138564.592 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138564.683 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138564.782 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138564.867 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138564.974 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138565.066 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138565.189 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138565.281 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138565.377 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138565.474 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138565.569 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138565.680 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138565.769 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138565.859 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138565.958 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138566.067 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138566.159 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138566.257 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138566.357 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位19
- 1780138621.080 reject too_large dx=3.324 yaw=0.148 age=0.00s
- 1780138621.177 reject too_large dx=3.324 yaw=0.148 age=0.00s
- 1780138621.286 reject too_large dx=3.324 yaw=0.148 age=0.00s
- 1780138621.385 reject too_large dx=3.324 yaw=0.148 age=0.00s
- 1780138621.471 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780138621.579 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138621.678 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138621.776 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138621.872 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138621.978 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138622.068 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138622.182 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138622.271 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138622.376 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138622.480 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138622.572 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138622.677 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138622.784 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138622.864 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138622.976 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位20
- 1780138687.977 reject too_large dx=7.419 yaw=0.269 age=0.00s
- 1780138688.082 reject too_large dx=7.419 yaw=0.269 age=0.00s
- 1780138688.190 reject too_large dx=7.419 yaw=0.269 age=0.00s
- 1780138688.286 reject too_large dx=7.419 yaw=0.269 age=0.00s
- 1780138688.379 reject too_large dx=7.419 yaw=0.269 age=0.00s
- 1780138688.496 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138688.591 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138688.671 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138688.769 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138688.869 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138688.980 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138689.078 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138689.166 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138689.261 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138689.381 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138689.486 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138689.574 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138689.684 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138689.775 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138689.868 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位21
- 1780138705.363 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138705.469 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138705.572 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138705.681 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138705.793 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138705.886 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138705.978 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138706.083 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138706.170 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138706.281 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138706.391 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138706.478 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138706.580 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138706.680 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138706.783 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138706.877 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138706.968 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138707.067 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138707.174 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138707.273 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位22
- 1780138730.764 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138730.870 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138730.978 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138731.072 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780138731.169 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138731.271 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138731.367 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138731.466 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138731.574 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138731.684 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138731.779 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138731.888 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138731.984 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138732.095 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138732.166 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780138732.268 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138732.373 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138732.483 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138732.571 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780138732.677 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
