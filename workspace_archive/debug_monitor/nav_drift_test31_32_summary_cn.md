# nav_drift_test31/32 回放验证汇总

## nav_drift_test31

- bag 时长: 1603.8s
- 单点导航命令数: 25，waypoint_ids: ['1107', '1108', '1109', '1110', '1111', '1112', '1113', '1114', '1115', '1116', '1117', '1118', '1119', '1120', '1121', '1122', '1123', '1124', '1125', '1126', '1127', '1128', '1129', '1130', '1131']
- navigation_started 成功: 25，navigation_completed 成功: 25
- pause 命令: 8，resume 命令: 2
- pause ack: [{'status': 'error', 'message': '当前没有在执行导航', 'timestamp': 1780373960.9015367}, {'status': 'error', 'message': '当前没有在执行导航', 'timestamp': 1780374023.8042955}, {'status': 'error', 'message': '当前没有在执行导航', 'timestamp': 1780374176.9546728}, {'status': 'error', 'message': '当前没有在执行导航', 'timestamp': 1780374436.2868767}, {'status': 'error', 'message': '当前没有在执行导航', 'timestamp': 1780374523.6380622}, {'status': 'error', 'message': '当前没有在执行导航', 'timestamp': 1780374812.7118268}, {'status': 'success', 'message': '导航已暂停', 'timestamp': 1780374945.2888951}, {'status': 'success', 'message': '导航已暂停', 'timestamp': 1780374963.120015}]
- resume ack: [{'status': 'success', 'message': '导航已恢复', 'timestamp': 1780374952.4555197}, {'status': 'success', 'message': '导航已恢复', 'timestamp': 1780374970.087204}]
- bridge 事件: {'ACCEPTED': 16033}
- 最大 map->odom 单步跳变: 0.198m @ 1780374103.592, dyaw=-0.024rad
- odom 兜底窗口: 0
- shadow 拦截候选: 0, hold 段: 0
- 资源采样: 前 300s bag 时间，3倍速，墙钟 102.7s
| 排名 | 进程 | 最大CPU% | 平均CPU% | 最大RSS MB |
|---:|---|---:|---:|---:|
| 1 | rosbag2_player | 49.12 | 15.92 | 166.0 |
| 2 | ros2 | 1.00 | 1.00 | 0.0 |

- 详细分析报告: `workspace_archive/debug_monitor/nav_drift_test31_analysis/nav_drift_test31_protection_analysis_cn.md`
- 资源报告: `workspace_archive/debug_monitor/nav_drift_test31_analysis/resource_replay/resource_report.md`

## nav_drift_test32

- bag 时长: 1240.6s
- 单点导航命令数: 25，waypoint_ids: ['1107', '1108', '1109', '1110', '1111', '1112', '1113', '1114', '1115', '1116', '1117', '1118', '1119', '1120', '1121', '1122', '1123', '1124', '1125', '1126', '1127', '1128', '1129', '1130', '1131']
- navigation_started 成功: 25，navigation_completed 成功: 25
- pause 命令: 1，resume 命令: 1
- pause ack: [{'status': 'success', 'message': '导航已暂停', 'timestamp': 1780375313.5839534}]
- resume ack: [{'status': 'success', 'message': '导航已恢复', 'timestamp': 1780375331.0070825}]
- bridge 事件: {'ACCEPTED': 12405, 'PENDING': 4}
- 最大 map->odom 单步跳变: 0.110m @ 1780375463.292, dyaw=-0.011rad
- odom 兜底窗口: 0
- shadow 拦截候选: 0, hold 段: 0
- 资源采样: 前 300s bag 时间，3倍速，墙钟 102.2s
| 排名 | 进程 | 最大CPU% | 平均CPU% | 最大RSS MB |
|---:|---|---:|---:|---:|
| 1 | rosbag2_player | 40.23 | 16.36 | 153.2 |
| 2 | ros2 | 10.94 | 10.94 | 0.0 |

- 详细分析报告: `workspace_archive/debug_monitor/nav_drift_test32_analysis/nav_drift_test32_protection_analysis_cn.md`
- 资源报告: `workspace_archive/debug_monitor/nav_drift_test32_analysis/resource_replay/resource_report.md`
