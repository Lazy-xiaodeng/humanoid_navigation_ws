#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_documentation_contract.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 检查本功能包是否继续满足第一阶段约定的中文文档/注释要求。
  3. 覆盖功能源码文件头、测试脚本文件头、主 YAML 参数中文注释、README 与验证报告关键章节。
  4. 该脚本不评价算法效果，只防止后续改代码时把可维护性和复现说明弄丢。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/check_documentation_contract.py
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path


CHINESE_RE = re.compile(r"[\u4e00-\u9fff]")


def repo_root_from_script() -> Path:
    """根据脚本所在位置反推出 humanoid_ws 根目录，方便在任意 cwd 执行。"""
    return Path(__file__).resolve().parents[3]


def has_chinese(text: str) -> bool:
    """判断文本里是否包含中文字符。"""
    return CHINESE_RE.search(text) is not None


def require(condition: bool, message: str, failures: list[str]) -> None:
    """统一收集失败项，便于一次运行看到所有文档缺口。"""
    if not condition:
        failures.append(message)


def read_text(path: Path) -> str:
    """读取 UTF-8 文本文件，失败时让调用方得到清晰路径。"""
    return path.read_text(encoding="utf-8")


def check_file_headers(package_dir: Path, failures: list[str]) -> None:
    """检查功能源码、launch 和测试脚本是否有中文文件头说明。"""
    source_files = sorted((package_dir / "src").glob("*.cpp"))
    source_files += sorted((package_dir / "include/humanoid_global_relocalization_runtime").glob("*.hpp"))
    source_files += sorted((package_dir / "launch").glob("*.py"))
    source_files += sorted((package_dir / "test").glob("*.py"))

    checked = 0
    for path in source_files:
        text = read_text(path)
        head = "\n".join(text.splitlines()[:40])
        checked += 1
        require(has_chinese(head), f"{path} 文件头缺少中文说明", failures)
        require(
            "文件作用" in head or "启动全局重定位" in head,
            f"{path} 文件头缺少“文件作用”或等价用途说明",
            failures,
        )
    print(f"[documentation_headers] checked={checked}")


def yaml_parameter_lines(text: str) -> list[tuple[int, str]]:
    """提取主配置 ros__parameters 下的顶层参数行；列表子字段不按发布参数处理。"""
    params: list[tuple[int, str]] = []
    for index, line in enumerate(text.splitlines(), start=1):
        match = re.match(r"^    ([A-Za-z_][A-Za-z0-9_]*):", line)
        if match:
            params.append((index, match.group(1)))
    return params


def has_recent_chinese_comment(lines: list[str], line_number: int, window: int = 8) -> bool:
    """检查某个参数行之前是否有中文注释；允许中间有空行，以适应分组注释。"""
    begin = max(0, line_number - window - 1)
    for line in lines[begin:line_number - 1]:
        stripped = line.lstrip()
        if stripped.startswith("#") and has_chinese(stripped):
            return True
    return False


def check_main_yaml(package_dir: Path, failures: list[str]) -> None:
    """检查主 YAML 是否保留每个关键参数附近的中文说明。"""
    path = package_dir / "config/relocalization_runtime.yaml"
    text = read_text(path)
    lines = text.splitlines()
    params = yaml_parameter_lines(text)
    missing_comment = [
        f"{path}:{line_number} {name}"
        for line_number, name in params
        if not has_recent_chinese_comment(lines, line_number)
    ]
    for item in missing_comment:
        failures.append(f"主 YAML 参数缺少邻近中文注释: {item}")

    required_params = [
        "input_mode",
        "registered_world_topic",
        "body_topic",
        "odom_topic",
        "map_dir",
        "map_candidates",
        "convert_raw_body_to_base",
        "map_leaf_size",
        "scan_leaf_size",
        "bbs_min_level_res",
        "top_k",
        "refine_method",
        "enable_temporal_consistency",
        "temporal_consistency_online_min_support_frames",
        "enable_relocalization",
        "recovery_pose_topic",
        "recovery_map_odom_topic",
    ]
    present = {name for _, name in params}
    for name in required_params:
        require(name in present, f"主 YAML 缺少关键参数: {name}", failures)

    print(f"[documentation_yaml] parameters={len(params)} missing_comments={len(missing_comment)}")


def check_markdown_docs(package_dir: Path, failures: list[str]) -> None:
    """检查 README 和验证报告是否保留关键章节、复现入口和边界说明。"""
    readme = read_text(package_dir / "README.md")
    report = read_text(package_dir / "test/nav_drift_validation_report.md")

    readme_terms = [
        "功能包作用",
        "上下游关系",
        "坐标系关系",
        "工作原理",
        "使用方法",
        "CSV 指标说明",
        "nav_drift_test 当前验证结论",
        "后续接入导航方案",
        "run_validation_gates.py",
        "run_real_body_validation.py",
        "compare_input_modes.py",
        "online_smoke_evidence.csv",
        "真实 `/cloud_registered_body`",
    ]
    report_terms = [
        "bag 与话题检查",
        "推荐配置",
        "nav_drift 30 帧扩展验证",
        "temporal decision",
        "合成 body 链路验证",
        "在线 debug 节点验证",
        "复现命令",
        "run_validation_gates.py",
        "run_real_body_validation.py",
        "compare_input_modes.py",
        "online_smoke_evidence.csv",
        "body_ready: 0",
    ]
    for term in readme_terms:
        require(term in readme, f"README 缺少关键内容: {term}", failures)
    for term in report_terms:
        require(term in report, f"验证报告缺少关键内容: {term}", failures)

    print(f"[documentation_markdown] readme_terms={len(readme_terms)} report_terms={len(report_terms)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check Chinese documentation/comment contract.")
    parser.add_argument(
        "--workspace",
        type=Path,
        default=repo_root_from_script(),
        help="humanoid_ws 工作空间路径，默认根据脚本位置推断",
    )
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    package_dir = workspace / "src/humanoid_global_relocalization_runtime"
    failures: list[str] = []

    require(package_dir.exists(), f"功能包目录不存在: {package_dir}", failures)
    if package_dir.exists():
        check_file_headers(package_dir, failures)
        check_main_yaml(package_dir, failures)
        check_markdown_docs(package_dir, failures)

    if failures:
        print("[check_documentation_contract] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[check_documentation_contract] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
