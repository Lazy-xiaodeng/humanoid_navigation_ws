# workspace_archive

该目录集中存放从工作空间根目录整理出来的历史资料、离线分析脚本、调试监控工具和必要的说明文档。

## 目录划分

| 目录 | 用途 |
|---|---|
| `design_docs/` | 方案设计、问题复盘、运维说明和历史修改记录。 |
| `offline_analysis_scripts/` | bag 回放、定位/融合/地图对比、TF 诊断等离线脚本。 |
| `debug_monitor/` | 导航漂移、ScanContext、Open3D/Robosense、路径规划仿真等调试脚本、HTML 监控页面和分析说明。 |

## 不入库内容

调试运行产生的日志、JSON/JSONL、CSV、PNG、TXT、ROS `ros_logs/`、临时代码备份和历史输出目录已从 Git 跟踪中移除，并在 `.gitignore` 中排除。后续需要复现实验时，保留的脚本和 HTML 监控页面会在本地重新生成这些输出。

## 未移动的正常工程内容

`src/`、`build/`、`install/`、`log/`、`data/`、`docs/`、`tools/`、`docker/`、`third_party/` 和根目录正式启动脚本保持原位。这样不会改变当前 ROS 包源码结构、编译产物位置、安装环境或一键启动入口。
