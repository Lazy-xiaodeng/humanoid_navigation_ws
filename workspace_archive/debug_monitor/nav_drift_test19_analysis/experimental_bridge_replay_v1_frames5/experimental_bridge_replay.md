# Experimental Bridge Replay Report

## Summary

- actual recorded max jump: 1.021 m
- current simulated max jump: 1.021 m
- protected simulated max jump: 0.686 m
- protected held candidates: 125
- protected degraded events: 19
- protected idle large accepts: 8
- protected nav large accepts: 0

## Per Point

| point | actual nav max | current nav max | protected nav max | protected recovery max | protected nav >=0.5m | failed |
|---|---:|---:|---:|---:|---:|---:|
| 点位1 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | False |
| 点位2 | 0.141 | 0.141 | 0.141 | 0.000 | 0 | False |
| 点位3 | 0.166 | 0.179 | 0.179 | 0.000 | 0 | False |
| 点位4 | 0.000 | 0.000 | 0.000 | 0.052 | 0 | False |
| 点位5 | 0.087 | 0.087 | 0.087 | 0.064 | 0 | False |
| 点位6 | 0.097 | 0.097 | 0.097 | 0.000 | 0 | False |
| 点位7 | 0.083 | 0.142 | 0.142 | 0.000 | 0 | False |
| 点位8 | 0.125 | 0.125 | 0.125 | 0.000 | 0 | False |
| 点位9 | 0.386 | 0.386 | 0.386 | 0.072 | 0 | False |
| 点位10 | 0.115 | 0.115 | 0.115 | 0.000 | 0 | False |
| 点位11 | 0.088 | 0.088 | 0.088 | 0.000 | 0 | False |
| 点位12 | 0.567 | 0.567 | 0.414 | 0.133 | 0 | False |
| 点位13 | 0.133 | 0.133 | 0.133 | 0.000 | 0 | False |
| 点位14 | 0.841 | 0.841 | 0.305 | 0.087 | 0 | False |
| 点位15 | 0.484 | 0.484 | 0.484 | 0.063 | 0 | False |
| 点位16 | 0.547 | 0.547 | 0.458 | 0.141 | 0 | False |
| 点位17 | 0.725 | 0.725 | 0.495 | 0.060 | 0 | False |
| 点位18 | 0.607 | 1.021 | 0.471 | 0.362 | 0 | False |
| 点位19 | 0.868 | 0.868 | 0.363 | 0.401 | 0 | False |
| 点位20 | 0.505 | 0.505 | 0.473 | 0.151 | 0 | False |
| 点位21 | 0.217 | 0.217 | 0.217 | 0.160 | 0 | False |
| 点位22 | 0.138 | 0.160 | 0.160 | 0.000 | 0 | False |
| 点位23 | 0.185 | 0.185 | 0.185 | 0.215 | 0 | False |
| 点位24 | 0.225 | 0.225 | 0.225 | 0.000 | 0 | False |
| 点位25 | 0.054 | 0.054 | 0.054 | 0.000 | 0 | False |

## Protected Idle Large Accepts

- 1780149103.594 idle dx=0.609 yaw=0.039 pending_age=0.41s
- 1780149139.082 idle dx=0.279 yaw=0.014 pending_age=0.40s
- 1780149158.080 idle dx=0.337 yaw=0.014 pending_age=0.40s
- 1780149179.592 idle dx=0.261 yaw=0.011 pending_age=0.40s
- 1780149200.179 idle dx=0.362 yaw=0.013 pending_age=0.38s
- 1780149201.269 idle dx=0.298 yaw=0.011 pending_age=0.38s
- 1780149216.172 idle dx=0.686 yaw=0.028 pending_age=0.38s
- 1780149217.075 idle dx=0.401 yaw=0.013 pending_age=0.39s

## Protected Notable Events

### idle
- 1780148792.668 reject no_confidence dx=0.000 yaw=0.000 age=0.00s
- 1780148831.566 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780148849.574 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780148871.171 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148871.270 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148871.373 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148871.466 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148871.583 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148871.680 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148871.781 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148871.866 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148871.967 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148872.067 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148872.169 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148872.270 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148872.370 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148872.469 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148872.569 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148872.672 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148872.771 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位2
- 1780148850.667 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148850.768 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148850.868 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148850.969 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148851.065 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148851.168 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148851.267 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148851.367 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148851.467 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148851.571 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148851.670 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148851.767 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148851.867 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148851.967 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148852.078 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148852.180 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148852.271 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148852.369 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148852.478 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148852.576 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位3
- 1780148877.669 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148877.767 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148877.867 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148877.969 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148878.068 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148878.166 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148878.268 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148878.370 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148878.479 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148878.568 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148878.668 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148878.766 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148878.869 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148878.967 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148879.069 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148879.167 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148879.267 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148879.378 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148879.471 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148879.569 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位4
- 1780148896.172 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148896.268 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148896.368 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148896.467 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148896.571 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148896.668 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148896.781 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148896.877 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148896.972 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148897.071 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148897.167 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148897.271 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148897.366 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148897.472 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148897.569 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148897.667 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148897.766 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148897.866 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148897.970 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148898.071 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位5
- 1780148913.665 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148913.769 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148913.867 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148913.970 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148914.072 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148914.168 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148914.269 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148914.370 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148914.470 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148914.568 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148914.669 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148914.768 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148914.871 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148914.967 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148915.066 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148915.168 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148915.271 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148915.383 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148915.468 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148915.570 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位6
- 1780148930.171 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148930.278 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148930.372 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148930.473 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148930.584 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148930.671 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148930.771 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148930.877 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148930.977 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148931.069 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148931.170 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148931.266 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148931.371 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148931.477 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148931.583 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148931.676 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148931.770 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148931.869 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148931.971 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148932.068 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位7
- 1780148948.666 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148948.767 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148948.867 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148948.968 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148949.067 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148949.166 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148949.267 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148949.381 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148949.480 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148949.583 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148949.671 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148949.771 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148949.866 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148949.969 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148950.066 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148950.170 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148950.266 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148950.371 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148950.471 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148950.579 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位8
- 1780148968.168 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148968.266 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148968.376 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148968.471 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148968.569 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148968.667 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148968.771 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148968.869 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148968.971 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148969.070 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148969.166 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148969.267 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148969.366 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148969.466 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148969.569 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148969.665 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148969.772 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148969.871 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148969.972 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148970.067 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位9
- 1780148995.171 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148995.276 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148995.375 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148995.482 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148995.582 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148995.694 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148995.774 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148995.872 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148995.966 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148996.073 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148996.179 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148996.275 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148996.395 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148996.486 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148996.578 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148996.685 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148996.789 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148996.881 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148996.984 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780148997.081 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位10
- 1780149019.671 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149019.768 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149019.879 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149019.979 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149020.073 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149020.189 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149020.282 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149020.398 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149020.480 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149020.590 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149020.678 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149020.780 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149020.871 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149020.968 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149021.067 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149021.175 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149021.288 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149021.369 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780149021.484 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149021.584 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位11
- 1780149041.596 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149041.698 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149041.784 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149041.876 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149041.969 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149042.075 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149042.173 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149042.275 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149042.372 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149042.469 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149042.589 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149042.690 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149042.792 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149042.887 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149042.986 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149043.090 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149043.180 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149043.274 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149043.371 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149043.485 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位12
- 1780149055.166 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149055.266 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149055.365 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149055.475 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149055.570 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149055.671 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149055.765 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149055.878 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149055.974 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149056.068 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149056.172 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149056.273 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149056.370 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149056.472 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149056.575 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149056.669 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149056.769 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149056.866 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149056.969 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149057.065 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位13
- 1780149087.385 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149087.472 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149087.582 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149087.683 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149087.778 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149087.903 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149087.993 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149088.084 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149091.071 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780149096.679 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149096.776 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149096.866 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149096.966 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149097.065 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149097.171 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149097.267 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149097.365 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149097.467 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149097.584 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149097.698 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位14
- 1780149106.613 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149106.715 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149106.801 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149106.897 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149106.992 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149107.091 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149107.196 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149107.288 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149107.399 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149107.502 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149107.615 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149107.681 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149107.795 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149107.907 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149108.002 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149108.084 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149108.191 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149108.289 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149108.395 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149108.480 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位15
- 1780149122.669 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149122.778 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149122.891 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149122.986 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149123.071 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149123.178 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149123.281 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149123.383 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149123.474 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149123.578 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149123.670 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149123.766 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149123.891 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149123.988 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149124.075 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149124.178 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149124.283 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149124.381 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149124.498 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149124.587 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位16
- 1780149142.104 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149142.202 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149142.293 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149142.390 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149142.500 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149142.606 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149142.699 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149142.797 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149142.876 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149142.977 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149143.082 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149143.192 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149143.293 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149143.399 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149143.477 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149143.582 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149143.670 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149143.774 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149143.884 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149143.996 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位17
- 1780149161.181 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149161.283 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149161.387 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149161.482 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149161.586 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149161.681 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149161.774 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149161.871 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.025 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.084 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.182 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.288 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.379 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.471 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.577 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.688 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.792 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.895 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149162.986 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149163.083 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位18
- 1780149182.687 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149182.790 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149182.884 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149182.994 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149183.077 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149183.183 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149183.270 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149183.376 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149183.467 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149183.578 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149183.687 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149183.785 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149183.877 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149184.001 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149184.100 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149184.182 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149184.287 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149184.366 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149184.488 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149184.580 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位19
- 1780149203.092 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149203.195 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149203.306 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149203.396 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149203.488 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149203.584 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149203.676 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149203.776 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149203.876 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149203.977 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149204.077 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149204.185 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149204.283 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149204.395 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149204.487 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149204.588 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149204.677 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149204.785 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149204.886 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149204.988 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位20
- 1780149219.186 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149219.295 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149219.403 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149219.496 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149219.581 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149219.689 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149219.793 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149219.884 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149219.972 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149220.078 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149220.182 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149220.278 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149220.388 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149220.486 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149220.582 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149220.680 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149220.787 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149220.888 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149220.980 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149221.090 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位21
- 1780149237.880 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
### 点位22
- 1780149256.588 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149256.683 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149256.788 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149256.866 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149256.974 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149257.064 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149257.168 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149257.275 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149257.371 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149257.466 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149257.582 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149257.688 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149257.778 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149257.882 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149257.987 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149258.082 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149258.181 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149258.277 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149258.378 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149258.491 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位23
- 1780149260.680 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149260.788 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149260.891 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149260.991 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149261.101 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149261.204 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149261.284 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149261.393 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149261.492 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149261.578 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149261.685 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149261.780 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149261.892 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149261.994 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149262.085 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149262.177 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149262.281 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149262.387 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149262.471 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149262.577 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位24
- 1780149273.175 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149273.301 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149273.394 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149273.509 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149273.608 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149273.703 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149273.795 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149273.897 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149273.981 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149274.067 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149274.169 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149274.281 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149274.379 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149274.472 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149274.578 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149274.689 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149274.789 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149274.893 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149274.981 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149275.081 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位25
- 1780149291.077 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149291.188 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149291.284 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149291.384 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149291.480 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149291.579 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149291.689 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149291.796 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149291.887 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149291.973 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149292.075 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149292.181 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149292.292 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149292.380 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149292.502 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149292.627 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149292.707 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149292.791 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149292.879 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780149292.999 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
