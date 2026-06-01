#!/usr/bin/env python3
"""Generate README files for the workspace archive folders."""
from pathlib import Path


ROOT = Path(__file__).resolve().parent
WS = ROOT.parent


def write(path: Path, text: str) -> None:
    path.write_text(text.rstrip() + "\n", encoding="utf-8")


def rel(path: Path) -> str:
    return str(path.relative_to(WS))


def file_usage(path: Path) -> str:
    name = path.name
    suffix = path.suffix.lower()
    if suffix == ".md":
        return "历史设计、问题复盘、测试结论或使用说明文档；直接用 Markdown 阅读。"
    if suffix == ".py":
        return "离线分析或数据处理脚本；先 source install/setup.bash，再按脚本内参数或 --help/源码注释运行。"
    if suffix == ".sh":
        return "调试/回放辅助脚本；确认脚本顶部路径变量后用 bash 执行。"
    if suffix in {".html", ".htm"}:
        return "离线可视化报告；用浏览器打开查看路径、曲线或对比结果。"
    if suffix == ".json":
        return "结构化测试摘要或配置快照；可用 jq/python 读取。"
    if suffix == ".jsonl":
        return "逐行 JSON 调试日志；适合用 rg、jq 或 Python 流式分析。"
    if suffix == ".csv":
        return "表格化采样/统计结果；可用表格软件、pandas 或脚本读取。"
    if suffix == ".log":
        return "进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。"
    if suffix == ".png":
        return "测试曲线或轨迹图；用于快速对比定位/里程计/路径结果。"
    if suffix == ".txt":
        return "文本日志、说明或元数据；直接打开或用 rg 检索关键字。"
    if suffix == ".launch.py":
        return "ROS 2 launch 文件；通常由同目录 replay/test 脚本调用，也可 ros2 launch 运行。"
    if suffix == ".bak":
        return "历史备份文件；用于对比或手工恢复，不参与当前默认启动链路。"
    return "归档文件；根据扩展名和所在目录判断用途，默认不参与当前正式启动链路。"


def table_for_files(paths) -> str:
    rows = ["| 文件 | 用途 | 使用方法/注意事项 |", "|---|---|---|"]
    for path in sorted(paths, key=lambda p: p.name):
        usage = file_usage(path)
        note = "路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。"
        rows.append(f"| `{path.name}` | {usage} | {note} |")
    return "\n".join(rows)


def table_for_dirs(paths) -> str:
    rows = ["| 子目录 | 内容 | 使用方法/注意事项 |", "|---|---|---|"]
    for path in sorted(paths, key=lambda p: p.name):
        files = sum(1 for p in path.rglob("*") if p.is_file())
        dirs = sum(1 for p in path.rglob("*") if p.is_dir())
        rows.append(
            f"| `{path.name}/` | 归档目录，包含 {files} 个文件、{dirs} 个子目录。 | "
            f"进入目录查看本目录 `README.md` 或用 `find {rel(path)} -maxdepth 2 -type f` 快速浏览。 |"
        )
    return "\n".join(rows)


def describe_dir(path: Path) -> str:
    name = path.name
    if name == "ndt_localization":
        return "NDT、定位漂移、pose jump、局部化参数和 Nav2 实机状态相关文档。"
    if name == "recovery_fusion":
        return "双引擎恢复、NDT/Fast-LIO/ScanContext 融合、恢复策略评审和交接文档。"
    if name == "fastlio_lidar":
        return "Fast-LIO、FastDDS、Robosense 雷达、坐标系和性能优化相关文档。"
    if name == "system_operations":
        return "系统说明、迁移部署、Git 操作、一键启动说明和修改日志。"
    if name == "offline_analysis_scripts":
        return "离线 bag 回放、NDT/Fast-LIO/融合结果分析、地图对比和 TF 诊断脚本。"
    if name == "runtime_logs":
        return "归档的 ROS 节点日志、NDT JSONL 调试日志和历史运行输出。"
    if name == "backups":
        return "历史源码/配置备份，不参与当前编译或启动。"
    if name == "debug_outputs":
        return "根目录散落的 debug_output 文本输出归档。"
    if name == "debug_monitor":
        return "导航漂移、ScanContext、Open3D/Robosense 对比、路径规划仿真和 live monitor 调试结果。"
    return "工作区归档子目录。"


def write_readme_for_dir(path: Path) -> None:
    files = [p for p in path.iterdir() if p.is_file() and p.name != "README.md"]
    dirs = [p for p in path.iterdir() if p.is_dir()]
    parts = [
        f"# {path.name}",
        "",
        describe_dir(path),
        "",
        "## 使用边界",
        "",
        "- 这里的内容是归档资料、离线脚本、历史日志或调试结果，不在当前正式 `src/` 源码包、`build/`、`install/`、`log/` 和一键启动入口中。",
        "- 正式启动脚本仍保留在工作空间根目录，例如 `start_navigation.sh`、`start_rslidar_with_fastdds.sh`、`start_websocket.sh`。",
        "- 如需运行脚本，先在工作空间根目录执行 `source install/setup.bash`，再按下表说明进入对应目录运行。",
    ]
    if files:
        parts += ["", "## 文件说明", "", table_for_files(files)]
    if dirs:
        parts += ["", "## 子目录说明", "", table_for_dirs(dirs)]
    write(path / "README.md", "\n".join(parts))


def write_debug_monitor_child_readmes() -> None:
    debug_root = ROOT / "debug_monitor"
    if not debug_root.exists():
        return
    for child in debug_root.iterdir():
        if not child.is_dir() or child.name == "__pycache__":
            continue
        files = [p for p in child.iterdir() if p.is_file() and p.name != "README.md"]
        dirs = [p for p in child.iterdir() if p.is_dir()]
        parts = [
            f"# {child.name}",
            "",
            "该目录是一次调试、回放、监控或对比实验的结果归档。目录名保留了实验主题、bag/test 编号或日期，便于追溯。",
            "",
            "## 使用方法",
            "",
            "- 优先查看 `summary.json`、`report.md`、`*_report.md`、`*.html` 或 `plots/` 下图片。",
            "- `samples.csv`、`poses_*.csv`、`*.jsonl` 用于后续脚本复算和统计。",
            "- `ros_logs/`、`*.log` 用于排查当次 launch、节点启动、TF 或定位状态。",
        ]
        if files:
            parts += ["", "## 文件说明", "", table_for_files(files)]
        if dirs:
            parts += ["", "## 子目录说明", "", table_for_dirs(dirs)]
        write(child / "README.md", "\n".join(parts))


def main() -> None:
    top_dirs = [
        ROOT / "design_docs",
        ROOT / "design_docs" / "ndt_localization",
        ROOT / "design_docs" / "recovery_fusion",
        ROOT / "design_docs" / "fastlio_lidar",
        ROOT / "design_docs" / "system_operations",
        ROOT / "offline_analysis_scripts",
        ROOT / "runtime_logs",
        ROOT / "backups",
        ROOT / "debug_outputs",
        ROOT / "debug_monitor",
    ]
    for path in top_dirs:
        if path.exists():
            write_readme_for_dir(path)
    write_debug_monitor_child_readmes()
    write(
        ROOT / "README.md",
        """# workspace_archive

该目录集中存放从工作空间根目录整理出来的历史资料、调试结果、离线分析脚本、日志和备份。

## 目录划分

| 目录 | 用途 |
|---|---|
| `design_docs/` | 方案设计、问题复盘、运维说明和历史修改记录。 |
| `offline_analysis_scripts/` | bag 回放、定位/融合/地图对比、TF 诊断等离线脚本。 |
| `debug_monitor/` | 导航漂移、ScanContext、Open3D/Robosense、路径规划仿真等实验结果。 |
| `runtime_logs/` | 历史 ROS 日志和 NDT JSONL 日志。 |
| `backups/` | 历史源码与配置备份。 |
| `debug_outputs/` | 根目录散落的文本调试输出。 |

## 未移动的正常工程内容

`src/`、`build/`、`install/`、`log/`、`data/`、`docs/`、`tools/`、`docker/`、`third_party/` 和根目录正式启动脚本保持原位。这样不会改变当前 ROS 包源码结构、编译产物位置、安装环境或一键启动入口。
""",
    )


if __name__ == "__main__":
    main()
