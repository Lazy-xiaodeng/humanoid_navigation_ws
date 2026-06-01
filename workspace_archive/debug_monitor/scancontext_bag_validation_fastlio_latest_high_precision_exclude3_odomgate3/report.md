# ScanContext Bag Validation

- Bag: `/home/ubuntu/fast-lio-bags/hall_mapping`
- Database: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin`
- Waypoints: `/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json` (`latest`)
- Samples: 743
- SC accepted: 686/743 (92.3%)
- Odom gate rejected: 55/743 (7.4%), gate=3.00 m
- Odom route distance median/p95: 0.483 / 2.231 m
- SC route distance median/p95: 0.467 / 1.806 m
- SC-vs-odom error median/p95: 0.395 / 2.735 m
- SC distance median/p95: 0.048 / 0.225
- Rescue-possible samples: 15

## First Events

- First odom off route > 1.00 m: {'sample_index': 58, 'stamp': '1778847591.772', 'odom_x': '0.9528', 'odom_y': '0.3443', 'odom_route_dist': '1.0131', 'odom_route_segment': 0, 'odom_nearest_waypoint': '点位1', 'odom_nearest_waypoint_id': '912', 'odom_nearest_waypoint_dist': '1.0185', 'sc_x': '0.9804', 'sc_y': '0.3126', 'sc_route_dist': '1.0290', 'sc_route_segment': 0, 'sc_nearest_waypoint': '点位1', 'sc_nearest_waypoint_id': '912', 'sc_nearest_waypoint_dist': '1.0337', 'sc_odom_error': '0.0420', 'sc_distance': '0.0679', 'odom_gate_error': '0.0420', 'odom_gate_pass': True, 'sc_accepted': True, 'sc_keyframe_id': 65, 'sc_yaw_shift': 0, 'rescue_possible': False}
- First SC bad: {'sample_index': 29, 'stamp': '1778847528.770', 'odom_x': '0.5840', 'odom_y': '0.1537', 'odom_route_dist': '0.6038', 'odom_route_segment': 0, 'odom_nearest_waypoint': '点位1', 'odom_nearest_waypoint_id': '912', 'odom_nearest_waypoint_dist': '0.6076', 'sc_x': '-0.4160', 'sc_y': '-0.2919', 'sc_route_dist': '0.4209', 'sc_route_segment': 1, 'sc_nearest_waypoint': '点位1', 'sc_nearest_waypoint_id': '912', 'sc_nearest_waypoint_dist': '0.4982', 'sc_odom_error': '1.0948', 'sc_distance': '0.0809', 'odom_gate_error': '1.0948', 'odom_gate_pass': True, 'sc_accepted': True, 'sc_keyframe_id': 709, 'sc_yaw_shift': 0, 'rescue_possible': False}

## Waypoint Reach Table

| id | name | x | y | min odom dist | first odom near | min SC dist | first SC near |
|---|---|---:|---:|---:|---|---:|---|
| 912 | 点位1 | 0.002 | -0.021 | 0.029 | 1778847466.269 | 0.030 | 1778847466.269 |
| 913 | 点位2 | 0.094 | -8.642 | 0.452 | 1778847741.574 | 0.487 | 1778847735.074 |
| 914 | 点位3 | -0.120 | -11.477 | 0.843 | 1778847754.674 | 0.981 | 1778847743.974 |
| 915 | 点位4 | -3.473 | -11.238 | 0.994 | 1778847769.775 | inf |  |
| 916 | 点位5 | -5.298 | -9.521 | 1.981 |  | 1.981 |  |
| 917 | 点位6 | -4.398 | -5.754 | 0.875 | 1778847801.275 | 0.971 | 1778847910.777 |
| 918 | 点位7 | -4.373 | -2.127 | 0.365 | 1778847814.375 | 0.365 | 1778847807.975 |
| 919 | 点位8 | -4.298 | 6.229 | 1.469 |  | 1.469 |  |
| 920 | 点位9 | -3.998 | 11.129 | 1.473 |  | 1.562 |  |
| 921 | 点位10 | 1.530 | 10.640 | 1.035 |  | 1.035 |  |
| 922 | 点位11 | 6.102 | 11.429 | 0.106 | 1778848239.883 | 0.106 | 1778848239.883 |
| 923 | 点位12 | 6.204 | 14.277 | 0.136 | 1778848303.984 | 0.136 | 1778848342.085 |
| 924 | 点位13 | 4.980 | 18.518 | 0.133 | 1778848381.686 | 0.219 | 1778848117.881 |
| 925 | 点位14 | 8.010 | 18.294 | 0.345 | 1778848526.788 | 1.215 |  |
| 926 | 点位15 | 11.502 | 18.629 | 0.171 | 1778848542.789 | 0.171 | 1778848533.488 |
| 927 | 点位16 | 14.452 | 18.379 | 0.075 | 1778848554.193 | 0.075 | 1778848547.089 |
| 928 | 点位17 | 20.602 | 18.829 | 0.547 | 1778848608.590 | 0.547 | 1778848879.895 |
| 929 | 点位18 | 23.162 | 16.766 | 0.806 | 1778848713.292 | 1.475 |  |
| 930 | 点位19 | 22.659 | 14.919 | 0.036 | 1778848719.592 | 0.036 | 1778848115.681 |
| 931 | 点位20 | 18.452 | 16.229 | 1.033 |  | 1.085 |  |
| 932 | 点位21 | 6.420 | 15.290 | 0.348 | 1778848310.284 | 0.450 | 1778847732.974 |
| 933 | 点位22 | 5.938 | 10.655 | 0.160 | 1778848205.583 | 0.249 | 1778847759.179 |
| 934 | 点位23 | 0.852 | 7.479 | 0.365 | 1778848083.185 | 0.571 | 1778847955.378 |
| 935 | 点位24 | -1.548 | 3.829 | 1.677 |  | 1.677 |  |
