#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_cpp_build_artifacts.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 检查 C++ ROS2 包在当前工作空间中是否已经构建并安装出关键产物。
  3. 该脚本不调用 colcon，避免在总门控里递归构建；它只验证最近一次构建后的二进制、库和共享文件是否齐全。
  4. 如果失败，说明需要先执行 `colcon build --packages-select humanoid_global_relocalization_runtime` 或构建产物已经损坏。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def repo_root_from_script() -> Path:
    """根据脚本位置反推出工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def executable(path: Path) -> bool:
    """判断文件是否存在且当前用户可执行。"""
    return path.is_file() and os.access(path, os.X_OK)


def readable_file(path: Path) -> bool:
    """判断普通文件是否存在且可读。"""
    return path.is_file() and os.access(path, os.R_OK)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify installed C++ build artifacts.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    install_root = workspace / "install/humanoid_global_relocalization_runtime"
    build_root = workspace / "build/humanoid_global_relocalization_runtime"

    required_executables = [
        install_root / "lib/humanoid_global_relocalization_runtime/global_relocalization_node",
        build_root / "global_relocalization_node",
        build_root / "global_relocalization_offline_eval",
    ]
    required_files = [
        install_root / "lib/libglobal_relocalization_core.a",
        install_root / "lib/libcpu_bbs3d.so",
        install_root / "share/humanoid_global_relocalization_runtime/config/global_relocalization_runtime.yaml",
        install_root / "share/humanoid_global_relocalization_runtime/launch/global_relocalization_runtime.launch.py",
        install_root / "share/humanoid_global_relocalization_runtime/README.md",
        build_root / "libglobal_relocalization_core.a",
    ]

    failures: list[str] = []
    for path in required_executables:
        if not executable(path):
            failures.append(f"missing or non-executable: {path}")
    for path in required_files:
        if not readable_file(path):
            failures.append(f"missing or unreadable: {path}")

    if failures:
        print("[verify_cpp_build_artifacts] FAIL", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("[verify_cpp_build_artifacts] installed executables=2 build executables=2 libraries=3 share_files=3")
    print("[verify_cpp_build_artifacts] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
