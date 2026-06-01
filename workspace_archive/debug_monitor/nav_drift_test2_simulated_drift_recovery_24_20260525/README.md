# nav_drift_test2_simulated_drift_recovery_24_20260525

该目录是一次调试、回放、监控或对比实验的结果归档。目录名保留了实验主题、bag/test 编号或日期，便于追溯。

## 使用方法

- 优先查看 `summary.json`、`report.md`、`*_report.md`、`*.html` 或 `plots/` 下图片。
- `samples.csv`、`poses_*.csv`、`*.jsonl` 用于后续脚本复算和统计。
- `ros_logs/`、`*.log` 用于排查当次 launch、节点启动、TF 或定位状态。

## 文件说明

| 文件 | 用途 | 使用方法/注意事项 |
|---|---|---|
| `drift_trials.csv` | 表格化采样/统计结果；可用表格软件、pandas 或脚本读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `drift_trials_detail.json` | 结构化测试摘要或配置快照；可用 jq/python 读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `report.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `summary.json` | 结构化测试摘要或配置快照；可用 jq/python 读取。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
