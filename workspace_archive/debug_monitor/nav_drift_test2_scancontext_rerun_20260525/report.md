# ScanContext Bag Validation

- Bag: `/home/ubuntu/nav_drift_test2`
- Database: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin`
- Waypoints: `/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json` (`latest`)
- Samples: 1463
- SC accepted: 934/1463 (63.8%)
- Odom gate rejected: 528/1463 (36.1%), gate=1.00 m
- Ambiguity gate rejected: 0/1463 (0.0%)
- Odom route distance median/p95: 0.044 / 0.467 m
- SC route distance median/p95: 0.591 / 1.031 m
- SC-vs-odom error median/p95: 0.904 / 0.982 m
- SC distance median/p95: 0.130 / 0.169
- Rescue-possible samples: 0

## First Events

- First odom off route > 1.00 m: none
- First SC bad: {'sample_index': 51, 'stamp': '1779695655.601', 'odom_x': '-0.0260', 'odom_y': '-1.4357', 'odom_route_dist': '0.0431', 'odom_route_segment': 1, 'odom_nearest_waypoint': '点位1', 'odom_nearest_waypoint_id': '912', 'odom_nearest_waypoint_dist': '1.4150', 'sc_x': '6.4235', 'sc_y': '9.9765', 'sc_route_dist': '0.8343', 'sc_route_segment': 21, 'sc_nearest_waypoint': '点位22', 'sc_nearest_waypoint_id': '933', 'sc_nearest_waypoint_dist': '0.8343', 'sc_odom_error': '13.1086', 'sc_distance': '0.1071', 'odom_gate_error': '13.1086', 'odom_gate_pass': False, 'ambiguity_gate_pass': True, 'sc_accepted': False, 'sc_keyframe_id': 681, 'sc_yaw_shift': 0, 'rescue_possible': False}

## Waypoint Reach Table

| id | name | x | y | min odom dist | first odom near | min SC dist | first SC near |
|---|---|---:|---:|---:|---|---:|---|
| 912 | 点位1 | 0.002 | -0.021 | 0.012 | 1779695601.494 | 0.033 | 1779695601.494 |
| 913 | 点位2 | 0.094 | -8.642 | 0.169 | 1779695668.197 | 0.452 | 1779695668.197 |
| 914 | 点位3 | -0.120 | -11.477 | 0.147 | 1779695726.298 | 0.843 | 1779695734.603 |
| 915 | 点位4 | -3.473 | -11.238 | 0.096 | 1779695792.499 | 1.106 |  |
| 916 | 点位5 | -5.298 | -9.521 | 0.171 | 1779695862.200 | inf |  |
| 917 | 点位6 | -4.398 | -5.754 | 0.243 | 1779695897.000 | 0.875 | 1779695899.001 |
| 918 | 点位7 | -4.373 | -2.127 | 0.257 | 1779695957.001 | 0.375 | 1779695790.399 |
| 919 | 点位8 | -4.298 | 6.229 | 0.038 | 1779696007.101 | 1.540 |  |
| 920 | 点位9 | -3.998 | 11.129 | 0.076 | 1779696050.401 | 1.930 |  |
| 921 | 点位10 | 1.530 | 10.640 | 0.069 | 1779696141.402 | 1.038 |  |
| 922 | 点位11 | 6.102 | 11.429 | 0.075 | 1779696208.607 | 0.106 | 1779695732.603 |
| 923 | 点位12 | 6.204 | 14.277 | 0.106 | 1779696220.103 | 0.136 | 1779695718.898 |
| 924 | 点位13 | 4.980 | 18.518 | 0.158 | 1779696232.609 | 0.466 | 1779695866.400 |
| 925 | 点位14 | 8.010 | 18.294 | inf |  | 1.373 |  |
| 926 | 点位15 | 11.502 | 18.629 | inf |  | inf |  |
| 927 | 点位16 | 14.452 | 18.379 | inf |  | 0.433 | 1779695901.200 |
| 928 | 点位17 | 20.602 | 18.829 | inf |  | 0.766 | 1779695906.605 |
| 929 | 点位18 | 23.162 | 16.766 | inf |  | 1.475 |  |
| 930 | 点位19 | 22.659 | 14.919 | inf |  | 0.460 | 1779696014.606 |
| 931 | 点位20 | 18.452 | 16.229 | inf |  | 1.263 |  |
| 932 | 点位21 | 6.420 | 15.290 | 0.472 | 1779696226.305 | 0.463 | 1779695671.498 |
| 933 | 点位22 | 5.938 | 10.655 | 2.023 |  | 0.249 | 1779695655.601 |
| 934 | 点位23 | 0.852 | 7.479 | inf |  | 0.420 | 1779695900.100 |
| 935 | 点位24 | -1.548 | 3.829 | 2.890 |  | 1.677 |  |
