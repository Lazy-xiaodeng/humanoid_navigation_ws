# nav_drift_test14_analysis

该目录是一次调试、回放、监控或对比实验的结果归档。目录名保留了实验主题、bag/test 编号或日期，便于追溯。

## 使用方法

- 优先查看 `summary.json`、`report.md`、`*_report.md`、`*.html` 或 `plots/` 下图片。
- `samples.csv`、`poses_*.csv`、`*.jsonl` 用于后续脚本复算和统计。
- `ros_logs/`、`*.log` 用于排查当次 launch、节点启动、TF 或定位状态。

## 文件说明

| 文件 | 用途 | 使用方法/注意事项 |
|---|---|---|
| `analysis_summary.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_multiframe_replay_metrics.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_nav_ndt.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `analyze_replayed_ndt_nojump.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `initialposes.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `inspect_bag.py` | 离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `inspect_output.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `jump_clusters.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `nav_drift_test14_ndt_report.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `nav_drift_test14_replayed_nojump_report.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `recovery_events.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `replayed_multiframe_point18_timeline.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `replayed_multiframe_waypoint_metrics.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `replayed_nojump_jump_clusters.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `replayed_nojump_waypoint_summary.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `waypoint_ndt_summary.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |

## 子目录说明

| 子目录 | 内容 | 使用方法/注意事项 |
|---|---|---|
| `__pycache__/` | 归档目录，包含 2 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test14_analysis/__pycache__ -maxdepth 2 -type f` 快速浏览。 |
