# Experimental Bridge Replay Report

## Summary

- actual recorded max jump: 1.341 m
- current simulated max jump: 1.341 m
- protected simulated max jump: 0.981 m
- protected held candidates: 930
- protected degraded events: 361
- protected idle large accepts: 11
- protected nav large accepts: 0

## Per Point

| point | actual nav max | current nav max | protected nav max | protected recovery max | protected nav >=0.5m | failed |
|---|---:|---:|---:|---:|---:|---:|
| 点位1 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | False |
| 点位2 | 0.197 | 0.197 | 0.197 | 0.000 | 0 | False |
| 点位3 | 0.244 | 0.244 | 0.244 | 0.000 | 0 | False |
| 点位4 | 0.000 | 0.000 | 0.000 | 0.000 | 0 | False |
| 点位5 | 0.051 | 0.069 | 0.069 | 0.000 | 0 | False |
| 点位6 | 0.136 | 0.136 | 0.136 | 0.095 | 0 | False |
| 点位7 | 0.148 | 0.148 | 0.148 | 0.000 | 0 | False |
| 点位8 | 0.167 | 0.167 | 0.167 | 0.000 | 0 | False |
| 点位9 | 0.582 | 0.582 | 0.000 | 0.121 | 0 | False |
| 点位10 | 0.313 | 0.694 | 0.472 | 0.101 | 0 | False |
| 点位11 | 0.679 | 0.679 | 0.000 | 0.000 | 0 | False |
| 点位12 | 0.217 | 0.217 | 0.000 | 0.000 | 0 | False |
| 点位13 | 0.253 | 0.253 | 0.000 | 0.000 | 0 | False |
| 点位14 | 0.122 | 0.122 | 0.122 | 0.190 | 0 | False |
| 点位15 | 0.230 | 0.230 | 0.230 | 0.104 | 0 | False |
| 点位16 | 0.570 | 0.570 | 0.062 | 0.187 | 0 | False |
| 点位17 | 1.341 | 1.341 | 0.339 | 0.221 | 0 | False |
| 点位18 | 0.901 | 1.180 | 0.195 | 0.233 | 0 | False |
| 点位19 | 1.265 | 1.265 | 0.408 | 0.000 | 0 | True |

## Protected Idle Large Accepts

- 1780146282.534 idle dx=0.339 yaw=0.050 pending_age=0.20s
- 1780146534.534 idle dx=0.566 yaw=0.058 pending_age=0.20s
- 1780146670.539 idle dx=0.981 yaw=0.027 pending_age=0.19s
- 1780146751.546 idle dx=0.386 yaw=0.030 pending_age=0.21s
- 1780146887.037 idle dx=0.846 yaw=0.039 pending_age=0.19s
- 1780146894.749 idle dx=0.282 yaw=0.012 pending_age=0.21s
- 1780146953.844 idle dx=0.834 yaw=0.037 pending_age=0.21s
- 1780146984.539 idle dx=0.355 yaw=0.016 pending_age=0.20s
- 1780147044.052 idle dx=0.709 yaw=0.021 pending_age=0.20s
- 1780147136.046 idle dx=0.422 yaw=0.015 pending_age=0.19s
- 1780147219.047 idle dx=0.635 yaw=0.026 pending_age=0.20s

## Protected Notable Events

### idle
- 1780146130.620 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780146130.714 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780146171.716 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780146178.019 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780146178.126 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780146179.027 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780146216.534 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146216.633 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146216.735 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146216.848 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146216.937 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146217.038 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146217.136 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146217.230 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146217.334 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146217.438 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146217.533 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146217.634 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146217.732 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146217.851 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位2
- 1780146195.321 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146195.416 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146195.529 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146195.624 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146195.731 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146195.827 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146195.926 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146196.025 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146196.123 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146196.230 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146196.328 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146196.431 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146196.542 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146196.625 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146196.720 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146196.825 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146196.924 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146197.035 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146197.135 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146197.237 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位3
- 1780146257.325 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146257.427 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146257.531 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146257.634 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146257.733 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146257.833 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146257.934 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146258.035 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146258.136 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146258.238 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146258.338 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146258.432 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146258.531 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146258.629 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146258.723 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146258.834 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146258.926 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146259.022 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146259.122 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146259.223 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位4
- 1780146324.834 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146324.935 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146325.029 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146325.134 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146325.238 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146325.333 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146325.451 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146325.540 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146325.637 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146325.733 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146325.840 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146325.940 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146326.033 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146326.133 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146326.224 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146326.332 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146326.434 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146326.532 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146326.625 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146326.736 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位5
- 1780146386.352 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146386.450 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146386.550 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146386.632 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146386.729 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146386.834 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146386.923 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146387.031 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146387.141 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146387.233 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146387.337 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146387.434 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146387.535 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146387.624 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146387.731 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146387.837 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146387.953 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146388.045 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146388.134 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146388.225 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位6
- 1780146422.338 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146422.443 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146422.544 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146422.632 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146422.738 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146422.835 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146422.930 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146423.025 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146423.137 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146423.265 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146423.356 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146423.444 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146423.538 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146423.631 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146423.730 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146423.833 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146423.936 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146424.034 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146424.133 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146424.233 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位7
- 1780146473.327 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146473.436 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146473.539 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146473.646 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146473.764 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146473.842 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146473.934 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146474.033 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146474.137 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146474.243 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146474.342 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146474.462 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146474.545 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146474.643 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146474.755 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146474.859 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146474.964 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146475.035 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146475.153 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146475.236 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位8
- 1780146511.625 reject missing_odom_cache dx=0.000 yaw=0.000 age=0.00s
- 1780146511.835 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146511.937 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146512.035 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146512.127 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146512.225 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146512.324 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146512.423 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146512.556 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146512.648 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146512.732 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146512.832 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146512.937 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146513.033 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146513.143 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146513.232 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146513.328 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146513.449 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146513.542 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146513.639 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位9
- 1780146553.335 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146553.429 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146553.540 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146553.638 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146553.724 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146553.827 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146553.924 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146554.025 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146554.159 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146554.256 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146554.348 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146554.440 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146554.542 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146554.633 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146554.730 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146554.852 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146554.939 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146555.034 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146555.136 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146555.236 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位10
- 1780146652.343 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146652.436 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146652.561 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146652.644 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146652.745 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146652.836 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146652.947 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146653.034 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146653.134 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146653.243 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146653.356 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146653.447 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146653.538 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146653.645 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146653.740 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146653.834 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146653.939 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146654.050 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146654.143 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146654.242 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位11
- 1780146712.332 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146712.432 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146712.541 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146712.638 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146712.737 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146712.838 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146712.933 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146713.036 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146713.133 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146713.244 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146713.357 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146713.433 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146713.529 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146713.627 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146713.723 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146713.830 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146713.924 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146714.045 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146714.124 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146714.222 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位12
- 1780146725.136 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146725.233 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146725.324 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146725.428 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146725.532 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146725.628 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146725.728 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146725.828 hold nav_large_blocked dx=1.342 yaw=0.056 age=0.00s
- 1780146725.930 hold nav_large_blocked dx=1.342 yaw=0.056 age=0.10s
- 1780146726.041 hold nav_large_blocked dx=1.342 yaw=0.056 age=0.21s
- 1780146726.138 hold nav_large_blocked dx=1.342 yaw=0.056 age=0.31s
- 1780146726.243 hold nav_large_blocked dx=1.342 yaw=0.056 age=0.41s
- 1780146726.338 hold nav_large_blocked dx=1.342 yaw=0.056 age=0.51s
- 1780146726.428 hold nav_large_blocked dx=1.342 yaw=0.056 age=0.60s
- 1780146726.533 hold nav_large_blocked dx=1.202 yaw=0.048 age=0.71s
- 1780146726.624 hold nav_large_blocked dx=1.202 yaw=0.048 age=0.80s
- 1780146726.733 hold nav_large_blocked dx=1.202 yaw=0.048 age=0.90s
- 1780146726.833 hold nav_large_blocked dx=1.202 yaw=0.048 age=1.00s
- 1780146726.937 hold nav_large_blocked dx=1.202 yaw=0.048 age=1.11s
- 1780146727.027 hold nav_large_blocked dx=1.202 yaw=0.048 age=1.20s
### 点位13
- 1780146735.347 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146735.438 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146735.534 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146735.639 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146735.733 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146735.835 hold nav_large_blocked dx=0.909 yaw=0.032 age=0.00s
- 1780146735.927 hold nav_large_blocked dx=0.909 yaw=0.032 age=0.09s
- 1780146736.031 hold nav_large_blocked dx=0.909 yaw=0.032 age=0.20s
- 1780146736.133 hold nav_large_blocked dx=0.909 yaw=0.032 age=0.30s
- 1780146736.233 hold nav_large_blocked dx=0.909 yaw=0.032 age=0.40s
- 1780146736.333 hold nav_large_blocked dx=0.909 yaw=0.032 age=0.50s
- 1780146736.425 hold nav_large_blocked dx=0.932 yaw=0.033 age=0.59s
- 1780146736.535 hold nav_large_blocked dx=0.932 yaw=0.033 age=0.70s
- 1780146736.645 hold nav_large_blocked dx=0.932 yaw=0.033 age=0.81s
- 1780146736.741 hold nav_large_blocked dx=0.932 yaw=0.033 age=0.91s
- 1780146736.841 hold nav_large_blocked dx=0.932 yaw=0.033 age=1.01s
- 1780146736.933 hold nav_large_blocked dx=0.932 yaw=0.033 age=1.10s
- 1780146737.023 hold nav_large_blocked dx=0.932 yaw=0.033 age=1.19s
- 1780146737.126 hold nav_large_blocked dx=0.932 yaw=0.033 age=1.29s
- 1780146737.225 hold nav_large_blocked dx=0.932 yaw=0.033 age=1.39s
### 点位14
- 1780146870.838 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146870.962 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146871.057 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146871.152 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146871.258 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146871.365 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146871.448 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146871.548 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146871.631 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146871.732 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146871.859 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146871.953 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146872.047 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146872.145 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146872.244 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146872.355 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146872.449 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146872.543 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146872.634 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146872.735 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位15
- 1780146977.334 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146977.430 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146977.544 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146977.647 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146977.755 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146977.866 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146977.967 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146978.054 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146978.147 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146978.245 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146978.353 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146978.445 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146978.554 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146978.644 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146978.755 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146978.850 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146978.947 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146979.047 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146979.150 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780146979.246 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位16
- 1780147029.350 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147029.470 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147029.563 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147029.656 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147029.740 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147029.834 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147029.938 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147030.038 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147030.139 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147030.236 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147030.337 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147030.445 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147030.549 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147030.645 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147030.735 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147030.836 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147030.960 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147031.053 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147031.145 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147031.251 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位17
- 1780147117.833 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147117.941 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147118.055 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147118.145 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147118.245 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147118.374 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147118.499 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147118.583 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147118.693 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147118.775 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147118.877 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147118.969 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147119.053 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147119.153 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147119.249 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147119.355 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147119.456 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147119.565 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147119.665 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147119.760 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位18
- 1780147201.367 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147201.457 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147201.553 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147201.660 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147201.749 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147201.841 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147201.944 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147202.047 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147202.141 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147202.255 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147202.355 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147202.449 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147202.568 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147202.644 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147202.749 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147202.847 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147202.948 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147203.042 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147203.149 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147203.247 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
### 点位19
- 1780147259.353 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147259.456 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147259.546 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147259.651 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147259.751 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147259.834 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147259.940 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147260.045 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147260.149 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147260.250 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147260.346 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147260.467 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147260.573 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147260.668 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147260.753 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147260.843 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147260.938 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147261.035 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147261.143 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
- 1780147261.263 reject spin_to_pose_freeze_tf dx=0.000 yaw=0.000 age=0.00s
