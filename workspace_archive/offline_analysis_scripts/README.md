# offline_analysis_scripts

离线 bag 回放、NDT/Fast-LIO/融合结果分析、地图对比和 TF 诊断脚本。

## 使用边界

- 这里的内容是归档资料、离线脚本、历史日志或调试结果，不在当前正式 `src/` 源码包、`build/`、`install/`、`log/` 和一键启动入口中。
- 正式启动脚本仍保留在工作空间根目录，例如 `start_navigation.sh`、`start_rslidar_with_fastdds.sh`、`start_websocket.sh`。
- 如需运行脚本，先在工作空间根目录执行 `source install/setup.bash`，再按下表说明进入对应目录运行。

## 文件说明

| 文件 | 用途 | 使用方法/注意事项 |
|---|---|---|
| `bag_final_analysis.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `bag_fusion_live_test.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `bag_fusion_validation.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `bag_fusion_validation_v2.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `bag_waypoint_analysis.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `compare_maps.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `diagnose_tf.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fix_tf_timestamp.sh` | 调试/回放辅助脚本；确认脚本顶部路径变量后用 bash 执行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `ndt_baseline_analysis.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `planb_dryrun_analysis.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `record_ndt.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `run_ndt_replay.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `run_ndt_replay_test.sh` | 调试/回放辅助脚本；确认脚本顶部路径变量后用 bash 执行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `test_ndt_replay.launch.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `test_planb_bag.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `test_planb_bag.sh` | 调试/回放辅助脚本；确认脚本顶部路径变量后用 bash 执行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
