#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_workspace_boundaries.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 检查本功能包的源码、配置、测试脚本和 bag inventory 是否误依赖 /home/ubuntu/humanoid_ws。
  3. 用户明确要求只能改动和使用当前工作空间 /home/ubuntu/software/Todesk/Files/humanoid_ws；
     /home/ubuntu/humanoid_ws 是正在使用的源码工作空间，不能作为本轮功能包的输入、输出或依赖路径。
  4. README/验证报告允许出现该路径作为“禁止修改/未使用”的说明，其它运行入口和配置不允许出现。

使用示例：
  python3 src/humanoid_global_relocalization_runtime/test/check_workspace_boundaries.py
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


FORBIDDEN_ROOT = "/home/ubuntu/humanoid_ws"
EXPECTED_WORKSPACE = "/home/ubuntu/software/Todesk/Files/humanoid_ws"


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 根目录。"""
    return Path(__file__).resolve().parents[3]


def require(condition: bool, message: str, failures: list[str]) -> None:
    """统一收集失败项。"""
    if not condition:
        failures.append(message)


def source_files_to_scan(package_dir: Path) -> list[Path]:
    """列出需要严格扫描的运行相关文件；文档说明文件单独处理。"""
    allowed_suffixes = {".py", ".yaml", ".cpp", ".hpp", ".xml", ".txt"}
    files: list[Path] = []
    for path in package_dir.rglob("*"):
        if not path.is_file():
            continue
        if path.name in {"check_workspace_boundaries.py", "check_goal_evidence.py", "run_validation_gates.py"}:
            continue
        if "third_party" in path.parts:
            continue
        if path.name in {"README.md", "nav_drift_validation_report.md"}:
            continue
        if path.name == "CMakeLists.txt" or path.suffix in allowed_suffixes:
            files.append(path)
    return sorted(files)


def check_no_forbidden_runtime_paths(package_dir: Path, artifacts_root: Path, failures: list[str]) -> None:
    """检查运行相关文件和 inventory 产物不包含禁止工作空间路径。"""
    scanned = 0
    for path in source_files_to_scan(package_dir):
        scanned += 1
        text = path.read_text(encoding="utf-8", errors="ignore")
        if FORBIDDEN_ROOT in text:
            failures.append(f"运行相关文件包含禁止工作空间路径: {path}")

    for artifact in [
        artifacts_root / "bag_inventory_v2.csv",
        artifacts_root / "bag_inventory_v2.md",
        artifacts_root / "bag_inventory_home_scan.csv",
        artifacts_root / "bag_inventory_home_scan.md",
    ]:
        if artifact.exists() and FORBIDDEN_ROOT in artifact.read_text(encoding="utf-8", errors="ignore"):
            failures.append(f"bag inventory 包含禁止工作空间路径: {artifact}")

    print(f"[workspace_boundaries] scanned_runtime_files={scanned}")


def check_expected_workspace(package_dir: Path, workspace: Path, failures: list[str]) -> None:
    """检查脚本实际所在工作空间是否为用户指定的当前工作空间。"""
    workspace_str = str(workspace)
    require(
        workspace_str == EXPECTED_WORKSPACE,
        f"当前工作空间应为 {EXPECTED_WORKSPACE}，实际为 {workspace_str}",
        failures,
    )
    require(package_dir.exists(), f"功能包目录不存在: {package_dir}", failures)


def check_docs_only_explain_forbidden(package_dir: Path, failures: list[str]) -> None:
    """文档可以提到禁止目录，但必须以不使用/未修改的语义出现。"""
    docs = [
        package_dir / "README.md",
        package_dir / "test/nav_drift_validation_report.md",
    ]
    for path in docs:
        if not path.exists():
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        if FORBIDDEN_ROOT not in text:
            continue
        allowed_phrases = ["未修改", "没有把它们作为验证输入", "没有对其做任何修改", "不能作为"]
        require(
            any(phrase in text for phrase in allowed_phrases),
            f"文档提到 {FORBIDDEN_ROOT} 时必须明确说明未修改/未使用: {path}",
            failures,
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Check workspace boundary requirements.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--artifacts-root", type=Path, default=None, help="验证产物根目录，默认 <workspace>/.codex_tmp")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    package_dir = workspace / "src/humanoid_global_relocalization_runtime"
    artifacts_root = (args.artifacts_root or workspace / ".codex_tmp").resolve()
    failures: list[str] = []

    check_expected_workspace(package_dir, workspace, failures)
    if package_dir.exists():
        check_no_forbidden_runtime_paths(package_dir, artifacts_root, failures)
        check_docs_only_explain_forbidden(package_dir, failures)

    if failures:
        print("[check_workspace_boundaries] FAIL", file=sys.stderr)
        for item in failures:
            print(f"  - {item}", file=sys.stderr)
        return 1

    print("[check_workspace_boundaries] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
