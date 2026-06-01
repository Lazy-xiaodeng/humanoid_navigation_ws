# nav_drift_20260513_170025

该目录是一次调试、回放、监控或对比实验的结果归档。目录名保留了实验主题、bag/test 编号或日期，便于追溯。

## 使用方法

- 优先查看 `summary.json`、`report.md`、`*_report.md`、`*.html` 或 `plots/` 下图片。
- `samples.csv`、`poses_*.csv`、`*.jsonl` 用于后续脚本复算和统计。
- `ros_logs/`、`*.log` 用于排查当次 launch、节点启动、TF 或定位状态。

## 文件说明

| 文件 | 用途 | 使用方法/注意事项 |
|---|---|---|
| `events.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `rosout_filtered.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `samples.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `status.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
