# ndt_localization

NDT、定位漂移、pose jump、局部化参数和 Nav2 实机状态相关文档。

## 使用边界

- 这里的内容是归档资料、离线脚本、历史日志或调试结果，不在当前正式 `src/` 源码包、`build/`、`install/`、`log/` 和一键启动入口中。
- 正式启动脚本仍保留在工作空间根目录，例如 `start_navigation.sh`、`start_rslidar_with_fastdds.sh`、`start_websocket.sh`。
- 如需运行脚本，先在工作空间根目录执行 `source install/setup.bash`，再按下表说明进入对应目录运行。

## 文件说明

| 文件 | 用途 | 使用方法/注意事项 |
|---|---|---|
| `定位简化实现说明.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `定位简化方案.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `2026-05-18_导航实现状态.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `NDT漂移根因与参数重设计.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `NDT漂移根因修复技术方案.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `NDT与FastLIO增量初值重设计.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `NDT位姿跳变根因与修复方案.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `NDT旋转保护与多帧方案.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `导航漂移测试4分析.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `导航定位漂移分析.md` | 历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
