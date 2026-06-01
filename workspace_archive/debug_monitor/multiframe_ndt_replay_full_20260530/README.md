# multiframe_ndt_replay_full_20260530

该目录是一次调试、回放、监控或对比实验的结果归档。目录名保留了实验主题、bag/test 编号或日期，便于追溯。

## 使用方法

- 优先查看 `summary.json`、`report.md`、`*_report.md`、`*.html` 或 `plots/` 下图片。
- `samples.csv`、`poses_*.csv`、`*.jsonl` 用于后续脚本复算和统计。
- `ros_logs/`、`*.log` 用于排查当次 launch、节点启动、TF 或定位状态。

## 子目录说明

| 子目录 | 内容 | 使用方法/注意事项 |
|---|---|---|
| `nav_drift_test17/` | 归档目录，包含 2 个文件、1 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_replay_full_20260530/nav_drift_test17 -maxdepth 2 -type f` 快速浏览。 |
| `navtest0/` | 归档目录，包含 7 个文件、1 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/debug_monitor/multiframe_ndt_replay_full_20260530/navtest0 -maxdepth 2 -type f` 快速浏览。 |
