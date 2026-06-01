# design_docs

工作区归档子目录。

## 使用边界

- 这里的内容是归档资料、离线脚本、历史日志或调试结果，不在当前正式 `src/` 源码包、`build/`、`install/`、`log/` 和一键启动入口中。
- 正式启动脚本仍保留在工作空间根目录，例如 `start_navigation.sh`、`start_rslidar_with_fastdds.sh`、`start_websocket.sh`。
- 如需运行脚本，先在工作空间根目录执行 `source install/setup.bash`，再按下表说明进入对应目录运行。

## 子目录说明

| 子目录 | 内容 | 使用方法/注意事项 |
|---|---|---|
| `fastlio_lidar/` | 归档目录，包含 10 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/design_docs/fastlio_lidar -maxdepth 2 -type f` 快速浏览。 |
| `ndt_localization/` | 归档目录，包含 10 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/design_docs/ndt_localization -maxdepth 2 -type f` 快速浏览。 |
| `recovery_fusion/` | 归档目录，包含 10 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/design_docs/recovery_fusion -maxdepth 2 -type f` 快速浏览。 |
| `system_operations/` | 归档目录，包含 6 个文件、0 个子目录。 | 进入目录查看本目录 `README.md` 或用 `find workspace_archive/design_docs/system_operations -maxdepth 2 -type f` 快速浏览。 |
