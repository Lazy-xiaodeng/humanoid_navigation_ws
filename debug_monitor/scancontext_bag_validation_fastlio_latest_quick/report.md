# ScanContext Bag Validation

- Bag: `/home/ubuntu/fast-lio-bags/hall_mapping`
- Database: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin`
- Waypoints: `/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json` (`latest`)
- Samples: 173
- SC accepted: 173/173 (100.0%)
- Odom route distance median/p95: 0.510 / 1.988 m
- SC route distance median/p95: 0.462 / 1.751 m
- SC-vs-odom error median/p95: 0.016 / 17.405 m
- SC distance median/p95: 0.000 / 0.020
- Rescue-possible samples: 0

## First Events

- First odom off route > 1.00 m: {'sample_index': 13, 'stamp': '1778847599.176', 'odom_x': '0.9773', 'odom_y': '0.3146', 'odom_route_dist': '1.0267', 'odom_route_segment': 0, 'odom_nearest_waypoint': '点位1', 'odom_nearest_waypoint_id': '912', 'odom_nearest_waypoint_dist': '1.0314', 'sc_x': '-0.1382', 'sc_y': '-0.4398', 'sc_route_dist': '0.1447', 'sc_route_segment': 1, 'sc_nearest_waypoint': '点位1', 'sc_nearest_waypoint_id': '912', 'sc_nearest_waypoint_dist': '0.4416', 'sc_odom_error': '1.3466', 'sc_distance': '0.0000', 'sc_accepted': True, 'sc_keyframe_id': 48, 'sc_yaw_shift': 41, 'rescue_possible': False}
- First SC bad: {'sample_index': 12, 'stamp': '1778847588.571', 'odom_x': '0.6354', 'odom_y': '0.4750', 'odom_route_dist': '0.7933', 'odom_route_segment': 0, 'odom_nearest_waypoint': '点位1', 'odom_nearest_waypoint_id': '912', 'odom_nearest_waypoint_dist': '0.8045', 'sc_x': '-0.1109', 'sc_y': '-0.3143', 'sc_route_dist': '0.1160', 'sc_route_segment': 1, 'sc_nearest_waypoint': '点位1', 'sc_nearest_waypoint_id': '912', 'sc_nearest_waypoint_dist': '0.3143', 'sc_odom_error': '1.0863', 'sc_distance': '0.0000', 'sc_accepted': True, 'sc_keyframe_id': 49, 'sc_yaw_shift': 44, 'rescue_possible': False}

## Waypoint Reach Table

| id | name | x | y | min odom dist | first odom near | min SC dist | first SC near |
|---|---|---:|---:|---:|---|---:|---|
| 912 | 点位1 | 0.002 | -0.021 | 0.025 | 1778847466.269 | 0.029 | 1778847466.269 |
| 913 | 点位2 | 0.094 | -8.642 | 0.683 | 1778847740.674 | 0.487 | 1778847740.674 |
| 914 | 点位3 | -0.120 | -11.477 | 1.081 |  | 1.166 |  |
| 915 | 点位4 | -3.473 | -11.238 | 1.065 |  | inf |  |
| 916 | 点位5 | -5.298 | -9.521 | 2.027 |  | 1.981 |  |
| 917 | 点位6 | -4.398 | -5.754 | 0.863 | 1778847801.975 | 0.971 | 1778847801.975 |
| 918 | 点位7 | -4.373 | -2.127 | 0.793 | 1778847822.375 | 0.365 | 1778847812.275 |
| 919 | 点位8 | -4.298 | 6.229 | 1.497 |  | 2.399 |  |
| 920 | 点位9 | -3.998 | 11.129 | 1.582 |  | 1.582 |  |
| 921 | 点位10 | 1.530 | 10.640 | 1.035 |  | 1.035 |  |
| 922 | 点位11 | 6.102 | 11.429 | 0.159 | 1778848248.483 | 0.187 | 1778848156.982 |
| 923 | 点位12 | 6.204 | 14.277 | 0.359 | 1778849044.899 | 1.428 |  |
| 924 | 点位13 | 4.980 | 18.518 | 0.303 | 1778848389.686 | 0.706 | 1778848491.088 |
| 925 | 点位14 | 8.010 | 18.294 | 1.215 |  | 1.215 |  |
| 926 | 点位15 | 11.502 | 18.629 | 0.388 | 1778848543.089 | 0.603 | 1778849159.206 |
| 927 | 点位16 | 14.452 | 18.379 | 0.289 | 1778848554.089 | 0.075 | 1778848554.089 |
| 928 | 点位17 | 20.602 | 18.829 | 1.404 |  | 1.404 |  |
| 929 | 点位18 | 23.162 | 16.766 | 1.872 |  | 0.838 | 1778849035.798 |
| 930 | 点位19 | 22.659 | 14.919 | 0.429 | 1778848719.592 | 0.248 | 1778847700.178 |
| 931 | 点位20 | 18.452 | 16.229 | 1.033 |  | 1.033 |  |
| 932 | 点位21 | 6.420 | 15.290 | 0.440 | 1778848309.189 | 0.450 | 1778847954.678 |
| 933 | 点位22 | 5.938 | 10.655 | 0.186 | 1778848207.982 | 0.277 | 1778848197.882 |
| 934 | 点位23 | 0.852 | 7.479 | 0.485 | 1778848086.380 | 0.600 | 1778848086.380 |
| 935 | 点位24 | -1.548 | 3.829 | 1.623 |  | 2.137 |  |
