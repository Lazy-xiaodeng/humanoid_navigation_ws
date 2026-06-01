# nav_drift_test19_analysis

该目录是一次调试、回放、监控或对比实验的结果归档。目录名保留了实验主题、bag/test 编号或日期，便于追溯。

## 使用方法

- 优先查看 `summary.json`、`report.md`、`*_report.md`、`*.html` 或 `plots/` 下图片。
- `samples.csv`、`poses_*.csv`、`*.jsonl` 用于后续脚本复算和统计。
- `ros_logs/`、`*.log` 用于排查当次 launch、节点启动、TF 或定位状态。

## 文件说明

| 文件 | 用途 | 使用方法/注意事项 |
|---|---|---|
| `analysis.json` | 结构化测试摘要或配置快照；可用 jq/python 读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `report.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |

## 子目录说明

| 子目录 | 内容 | 使用方法/注意事项 |
|---|---|---|
| `experimental_bridge_replay_v1_frames5/` | 归档目录，包含 2 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test19_analysis/experimental_bridge_replay_v1_frames5 -maxdepth 2 -type f` 快速浏览。 |
| `protected_policy_impact_v1_frames5/` | 归档目录，包含 2 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/nav_drift_test19_analysis/protected_policy_impact_v1_frames5 -maxdepth 2 -type f` 快速浏览。 |
