# debug_monitor

导航漂移、ScanContext、Open3D/Robosense 对比、路径规划仿真和 live monitor 调试结果。

## 使用边界

- 这里的内容是归档资料、离线脚本、历史日志或调试结果，不在当前正式 `src/` 源码包、`build/`、`install/`、`log/` 和一键启动入口中。
- 正式启动脚本仍保留在工作空间根目录，例如 `start_navigation.sh`、`start_rslidar_with_fastdds.sh`、`start_websocket.sh`。
- 如需运行脚本，先在工作空间根目录执行 `source install/setup.bash`，再按下表说明进入对应目录运行。

## 文件说明

| 文件 | 用途 | 使用方法/注意事项 |
|---|---|---|
| `BAG_ANALYSIS_REQUIREMENTS_CN.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `PROTECTION_PARAM_TUNING_LOG_CN.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_integrated_ro_vs_open3d.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_nav_bag_continuity.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_nav_episodes.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_nav_execution_segments.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_navtest15.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_protected_policy_impact.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_recent_plan_bags.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_replanning_loop.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_robosense_open3d_validation.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_scancontext_recovery_success.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `check_plan_costs.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `experiment_bridge_replay.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_dashboard.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_195548.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_195548_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_195601.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_195601_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_201240.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_201240_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_210052.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_210052_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_210613.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_210613_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_222826.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260524_222826_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_104345.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_104345_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_104718.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_104718_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_135356.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_135356_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_135845.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_135845_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_144210.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_144210_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_152304.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260525_152304_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260526_160139.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `fastlio_drift_live_20260526_160139_meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `latest_execution_1114_1116_analysis.html` | 离线可视化报告；用浏览器打开查看路径、曲线或对比结果。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `make_integrated_ro_open3d_plots.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `make_robosense_open3d_track_plots.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `nav_pose_issue_monitor_live.jsonl` | 逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `navigation_app_simulator.html` | 离线可视化报告；用浏览器打开查看路径、曲线或对比结果。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `plot_straight_first_with_actual.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `replanning_loop_test28_1114_1116.html` | 离线可视化报告；用浏览器打开查看路径、曲线或对比结果。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `run_integrated_robosense_bag_validation.sh` | 调试/回放辅助脚本；确认脚本顶部路径变量后用 bash 执行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `run_integrated_voxel01_bag_validation.sh` | 调试/回放辅助脚本；确认脚本顶部路径变量后用 bash 执行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `run_robosense_open3d_bag_validation.sh` | 调试/回放辅助脚本；确认脚本顶部路径变量后用 bash 执行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `shadow_bridge_policy.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `simulate_scancontext_drift_recovery.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `simulate_straight_first_from_bags.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `simulate_theta_star_from_bags.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `start_fastlio_dashboard.sh` | 调试/回放辅助脚本；确认脚本顶部路径变量后用 bash 执行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `start_live_monitor.sh` | 调试/回放辅助脚本；确认脚本顶部路径变量后用 bash 执行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `stop_live_monitor.sh` | 调试/回放辅助脚本；确认脚本顶部路径变量后用 bash 执行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `straight_first_bag27_28_1114_1116_simulation.html` | 离线可视化报告；用浏览器打开查看路径、曲线或对比结果。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `straight_first_bag27_28_all_simulation.html` | 离线可视化报告；用浏览器打开查看路径、曲线或对比结果。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `straight_first_bag27_28_with_actual.html` | 离线可视化报告；用浏览器打开查看路径、曲线或对比结果。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `theta_star_all_paths_simulation.html` | 离线可视化报告；用浏览器打开查看路径、曲线或对比结果。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `theta_star_focus_7-8_9-10_simulation.html` | 离线可视化报告；用浏览器打开查看路径、曲线或对比结果。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `theta_star_latest_1114_1116_simulation.html` | 离线可视化报告；用浏览器打开查看路径、曲线或对比结果。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `validate_fusion_fix.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |

## 子目录说明

| 子目录 | 内容 | 使用方法/注意事项 |
|---|---|---|
| `__pycache__/` | 归档目录，包含 14 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/__pycache__ -maxdepth 2 -type f` 快速浏览。 |
| `code_backups/` | 归档目录，包含 1551 个文件、466 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/code_backups -maxdepth 2 -type f` 快速浏览。 |
| `integrated_robosense_validation_20260601_monitor/` | 归档目录，包含 38 个文件、11 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/integrated_robosense_validation_20260601_monitor -maxdepth 2 -type f` 快速浏览。 |
| `integrated_robosense_validation_20260601_protect/` | 归档目录，包含 39 个文件、11 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/integrated_robosense_validation_20260601_protect -maxdepth 2 -type f` 快速浏览。 |
| `integrated_voxel01_validation_20260601/` | 归档目录，包含 72 个文件、21 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/integrated_voxel01_validation_20260601 -maxdepth 2 -type f` 快速浏览。 |
| `live_20260524_165144/` | 归档目录，包含 28 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/live_20260524_165144 -maxdepth 2 -type f` 快速浏览。 |
| `live_20260524_165253/` | 归档目录，包含 28 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/live_20260524_165253 -maxdepth 2 -type f` 快速浏览。 |
| `multiframe_ndt_filtered_final_20260530/` | 归档目录，包含 1 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_filtered_final_20260530 -maxdepth 2 -type f` 快速浏览。 |
| `multiframe_ndt_filtered_navtest0_20260530/` | 归档目录，包含 6 个文件、2 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_filtered_navtest0_20260530 -maxdepth 2 -type f` 快速浏览。 |
| `multiframe_ndt_filtered_smoke/` | 归档目录，包含 6 个文件、2 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_filtered_smoke -maxdepth 2 -type f` 快速浏览。 |
| `multiframe_ndt_filtered_test17_20260530/` | 归档目录，包含 6 个文件、2 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_filtered_test17_20260530 -maxdepth 2 -type f` 快速浏览。 |
| `multiframe_ndt_filtered_test19_20260530/` | 归档目录，包含 6 个文件、2 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_filtered_test19_20260530 -maxdepth 2 -type f` 快速浏览。 |
| `multiframe_ndt_replay_full_20260530/` | 归档目录，包含 9 个文件、4 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_replay_full_20260530 -maxdepth 2 -type f` 快速浏览。 |
| `multiframe_ndt_replay_test17_20260530/` | 归档目录，包含 6 个文件、2 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_replay_test17_20260530 -maxdepth 2 -type f` 快速浏览。 |
| `multiframe_ndt_replay_test19_20260530/` | 归档目录，包含 6 个文件、2 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_replay_test19_20260530 -maxdepth 2 -type f` 快速浏览。 |
| `multiframe_ndt_smoke/` | 归档目录，包含 9 个文件、2 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_smoke -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_20260513_154708/` | 归档目录，包含 7 个文件、1 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_20260513_154708 -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_20260513_161301/` | 归档目录，包含 5 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_20260513_161301 -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_20260513_170025/` | 归档目录，包含 5 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_20260513_170025 -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_20260513_170025_quiet/` | 归档目录，包含 5 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_20260513_170025_quiet -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test14_analysis/` | 归档目录，包含 18 个文件、1 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test14_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test15_analysis/` | 归档目录，包含 2 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test15_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test16_analysis/` | 归档目录，包含 4 个文件、1 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test16_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test17_analysis/` | 归档目录，包含 6 个文件、1 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test17_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test18_analysis/` | 归档目录，包含 8 个文件、3 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test18_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test19_analysis/` | 归档目录，包含 6 个文件、2 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test19_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test20_analysis/` | 归档目录，包含 7 个文件、2 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test20_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test21_analysis/` | 归档目录，包含 8 个文件、2 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test21_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test22_analysis/` | 归档目录，包含 3 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test22_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test28_analysis/` | 归档目录，包含 1 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test28_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_analysis/` | 归档目录，包含 6 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_analysis -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_fusion_scheme_continuity_20260525/` | 归档目录，包含 5 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_fusion_scheme_continuity_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_scancontext_current_params/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_scancontext_current_params -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_scancontext_rerun_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_scancontext_rerun_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_scancontext_rerun_20260525_latest/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_scancontext_rerun_20260525_latest -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_simulated_drift_recovery_12_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_simulated_drift_recovery_12_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_simulated_drift_recovery_12_ndt2m_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_simulated_drift_recovery_12_ndt2m_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_simulated_drift_recovery_24_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_simulated_drift_recovery_24_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_simulated_drift_recovery_24_ndt2m_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_simulated_drift_recovery_24_ndt2m_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_simulated_drift_recovery_latest_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_simulated_drift_recovery_latest_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `nav_drift_test2_simulated_drift_recovery_latest_ndt2m_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test2_simulated_drift_recovery_latest_ndt2m_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_nav_drift_test14/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_nav_drift_test14 -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_odomcache_navtest0_isolated/` | 归档目录，包含 2 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_odomcache_navtest0_isolated -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_odomcache_pending_navtest0_isolated/` | 归档目录，包含 0 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_odomcache_pending_navtest0_isolated -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_odomcache_pending_navtest0_rerun/` | 归档目录，包含 3 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_odomcache_pending_navtest0_rerun -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_rotfreeze_nav_drift_test14/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_rotfreeze_nav_drift_test14 -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_rotguard_nav_drift_test14/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_rotguard_nav_drift_test14 -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_spintopose_guard_3s_nav_drift_test13/` | 归档目录，包含 3 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_spintopose_guard_3s_nav_drift_test13 -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_spintopose_guard_3s_nav_drift_test14/` | 归档目录，包含 3 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_spintopose_guard_3s_nav_drift_test14 -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_spintopose_guard_3s_navtest0/` | 归档目录，包含 0 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_spintopose_guard_3s_navtest0 -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_spintopose_guard_3s_navtest0_isolated/` | 归档目录，包含 3 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_spintopose_guard_3s_navtest0_isolated -maxdepth 2 -type f` 快速浏览。 |
| `prior_map_open3d_spintopose_guard_nav_drift_test14/` | 归档目录，包含 3 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/prior_map_open3d_spintopose_guard_nav_drift_test14 -maxdepth 2 -type f` 快速浏览。 |
| `robosense_open3d_validation_20260531_full/` | 归档目录，包含 83 个文件、18 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/robosense_open3d_validation_20260531_full -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_current_params_rerun/` | 归档目录，包含 5 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_current_params_rerun -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_fastlio_latest/` | 归档目录，包含 0 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_fastlio_latest -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_fastlio_latest_high_precision/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_fastlio_latest_high_precision -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_fastlio_latest_high_precision_exclude3/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_fastlio_latest_high_precision_exclude3 -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_fastlio_latest_high_precision_exclude3_odomgate1p0/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_fastlio_latest_high_precision_exclude3_odomgate1p0 -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_fastlio_latest_high_precision_exclude3_odomgate1p0_scth025/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_fastlio_latest_high_precision_exclude3_odomgate1p0_scth025 -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_fastlio_latest_high_precision_exclude3_odomgate1p5/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_fastlio_latest_high_precision_exclude3_odomgate1p5 -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_fastlio_latest_high_precision_exclude3_odomgate3/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_fastlio_latest_high_precision_exclude3_odomgate3 -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_fastlio_latest_high_precision_offset1s/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_fastlio_latest_high_precision_offset1s -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_fastlio_latest_quick/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_fastlio_latest_quick -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_final_rerun/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_final_rerun -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_bag_validation_stability_gates_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_bag_validation_stability_gates_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_global_trigger_hall_mapping_20260525/` | 归档目录，包含 2 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_global_trigger_hall_mapping_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_recovery_success_hall_mapping_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_recovery_success_hall_mapping_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_recovery_success_nav_drift_test2_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_recovery_success_nav_drift_test2_20260525 -maxdepth 2 -type f` 快速浏览。 |
| `scancontext_recovery_success_nav_drift_test2_latest_20260525/` | 归档目录，包含 4 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/scancontext_recovery_success_nav_drift_test2_latest_20260525 -maxdepth 2 -type f` 快速浏览。 |
