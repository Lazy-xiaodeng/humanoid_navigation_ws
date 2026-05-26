# ScanContext Bag Validation

- Bag: `/home/ubuntu/nav_drift_test2`
- Database: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin`
- Waypoints: `/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json` (`latest`)
- Samples: 753
- SC accepted: 488/753 (64.8%)
- Odom gate rejected: 264/753 (35.1%), gate=1.00 m
- Ambiguity gate rejected: 0/753 (0.0%)
- Odom route distance median/p95: 0.047 / 0.460 m
- SC route distance median/p95: 0.591 / 1.031 m
- SC-vs-odom error median/p95: 0.904 / 0.983 m
- SC distance median/p95: 0.129 / 0.169
- Rescue-possible samples: 0

## First Events

- First odom off route > 1.00 m: none
- First SC bad: {'sample_index': 30, 'stamp': '1779695663.197', 'odom_x': '0.2041', 'odom_y': '-5.6439', 'odom_route_dist': '0.1421', 'odom_route_segment': 1, 'odom_nearest_waypoint': '点位2', 'odom_nearest_waypoint_id': '913', 'odom_nearest_waypoint_dist': '3.0001', 'sc_x': '-0.3038', 'sc_y': '-7.3964', 'sc_route_dist': '0.3844', 'sc_route_segment': 1, 'sc_nearest_waypoint': '点位2', 'sc_nearest_waypoint_id': '913', 'sc_nearest_waypoint_dist': '1.3076', 'sc_odom_error': '1.8245', 'sc_distance': '0.0399', 'odom_gate_error': '1.8245', 'odom_gate_pass': False, 'ambiguity_gate_pass': True, 'sc_accepted': False, 'sc_keyframe_id': 126, 'sc_yaw_shift': 0, 'rescue_possible': False}

## Waypoint Reach Table

| id | name | x | y | min odom dist | first odom near | min SC dist | first SC near |
|---|---|---:|---:|---:|---|---:|---|
| 912 | 点位1 | 0.002 | -0.021 | 0.010 | 1779695601.494 | 0.029 | 1779695601.494 |
| 913 | 点位2 | 0.094 | -8.642 | 0.174 | 1779695667.297 | 0.452 | 1779695669.297 |
| 914 | 点位3 | -0.120 | -11.477 | 0.201 | 1779695726.698 | 0.843 | 1779695736.999 |
| 915 | 点位4 | -3.473 | -11.238 | 0.126 | 1779695792.399 | 1.106 |  |
| 916 | 点位5 | -5.298 | -9.521 | 0.250 | 1779695862.604 | inf |  |
| 917 | 点位6 | -4.398 | -5.754 | 0.224 | 1779695897.500 | 0.971 | 1779695897.500 |
| 918 | 点位7 | -4.373 | -2.127 | 0.257 | 1779695957.101 | 0.375 | 1779695790.399 |
| 919 | 点位8 | -4.298 | 6.229 | 0.023 | 1779696006.501 | 1.540 |  |
| 920 | 点位9 | -3.998 | 11.129 | 0.129 | 1779696049.901 | 2.228 |  |
| 921 | 点位10 | 1.530 | 10.640 | 0.121 | 1779696142.609 | 1.035 |  |
| 922 | 点位11 | 6.102 | 11.429 | 0.106 | 1779696210.203 | 0.106 | 1779695975.605 |
| 923 | 点位12 | 6.204 | 14.277 | 0.338 | 1779696220.404 | 0.136 | 1779695718.602 |
| 924 | 点位13 | 4.980 | 18.518 | 0.158 | 1779696232.609 | 0.466 | 1779696234.703 |
| 925 | 点位14 | 8.010 | 18.294 | inf |  | 1.051 |  |
| 926 | 点位15 | 11.502 | 18.629 | inf |  | 1.004 |  |
| 927 | 点位16 | 14.452 | 18.379 | inf |  | 0.433 | 1779695901.606 |
| 928 | 点位17 | 20.602 | 18.829 | inf |  | 0.766 | 1779695905.700 |
| 929 | 点位18 | 23.162 | 16.766 | inf |  | 1.475 |  |
| 930 | 点位19 | 22.659 | 14.919 | inf |  | inf |  |
| 931 | 点位20 | 18.452 | 16.229 | inf |  | 1.263 |  |
| 932 | 点位21 | 6.420 | 15.290 | 0.466 | 1779696226.610 | 0.699 | 1779695743.198 |
| 933 | 点位22 | 5.938 | 10.655 | 2.229 |  | 0.249 | 1779695730.798 |
| 934 | 点位23 | 0.852 | 7.479 | inf |  | 2.336 |  |
| 935 | 点位24 | -1.548 | 3.829 | 2.960 |  | 2.339 |  |
