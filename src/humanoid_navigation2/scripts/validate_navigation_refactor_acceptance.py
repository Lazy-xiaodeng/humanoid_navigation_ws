#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""新导航系统改造统一验收脚本。

默认只做安全验证：
1. shell/python 语法检查。
2. 多地图切图静态护栏。
3. 路线任务静态护栏。
4. 多地图 switch_map 语义模拟。
5. hall 地图导航层 validate-only。
6. 分层 launch 的 --show-args 可发现性。

不会启动真实雷达、Nav2 或定位链路。需要构建时可加 --build。
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Iterable, List


def find_workspace_root() -> Path:
    """从当前脚本路径向上寻找 Todesk 工作区根目录。"""
    current = Path(__file__).resolve()
    for candidate in [current, *current.parents]:
        if (candidate / "start_navigation.sh").exists() and (candidate / "src/humanoid_navigation2").exists():
            return candidate
    raise RuntimeError("无法定位 humanoid_ws 工作区根目录")


WORKSPACE_ROOT = find_workspace_root()


def print_step(title: str) -> None:
    """打印统一格式的验收步骤标题。"""
    print(f"\n=== {title} ===", flush=True)


def run_command(command: List[str], *, title: str, env: dict | None = None) -> None:
    """运行命令，失败时抛出异常并保留原始输出。"""
    print_step(title)
    print(" ".join(command), flush=True)
    subprocess.run(command, cwd=WORKSPACE_ROOT, env=env, check=True)


def run_bash(script: str, *, title: str, env: dict | None = None) -> None:
    """通过 bash -lc 运行需要 source ROS 环境的命令。"""
    print_step(title)
    print(script, flush=True)
    subprocess.run(["/bin/bash", "-lc", script], cwd=WORKSPACE_ROOT, env=env, check=True)


def ros_env_script(command: str) -> str:
    """拼接 ROS + 当前工作区 install 环境。"""
    return (
        "set -eo pipefail; "
        "source /opt/ros/jazzy/setup.bash; "
        "source install/local_setup.bash; "
        f"{command}"
    )


def cleanup_runtime_files() -> None:
    """清理 validate-only 生成的运行时文件，避免验收脚本污染工作区。"""
    for relative in (".active_navigation_map.env",):
        path = WORKSPACE_ROOT / relative
        if path.exists():
            path.unlink()
    runtime_maps = WORKSPACE_ROOT / "data/runtime_maps"
    if runtime_maps.exists():
        shutil.rmtree(runtime_maps)


def existing_paths(paths: Iterable[str]) -> List[str]:
    """只返回当前工作区真实存在的路径，方便 py_compile 兼容增删文件。"""
    return [path for path in paths if (WORKSPACE_ROOT / path).exists()]


def build_if_requested(args: argparse.Namespace) -> None:
    """可选构建受影响包；默认跳过以提高验收速度。"""
    if not args.build:
        print_step("跳过构建")
        print("未传 --build，默认不执行 colcon build。", flush=True)
        return
    run_bash(
        "source /opt/ros/jazzy/setup.bash; "
        "colcon build --packages-select "
        "humanoid_bringup humanoid_navigation humanoid_navigation2 humanoid_websocket "
        "--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release",
        title="构建受影响包",
    )


def validate_shell_syntax() -> None:
    """检查所有启动/停止/切图脚本语法。"""
    scripts = [
        "start_navigation.sh",
        "stop_navigation.sh",
        "start_ros_control_plane.sh",
        "stop_ros_control_plane.sh",
        "start_navigation_stack.sh",
        "stop_navigation_stack.sh",
        "switch_navigation_map.sh",
        "start_mapping.sh",
    ]
    run_command(["bash", "-n", *scripts], title="Shell 脚本语法检查")


def validate_python_syntax() -> None:
    """检查本轮新增/修改的 launch 和验证脚本语法。"""
    files = existing_paths([
        "src/humanoid_bringup/launch/robot_real.launch.py",
        "src/humanoid_bringup/launch/robot_control_plane.launch.py",
        "src/humanoid_bringup/launch/robot_navigation_stack.launch.py",
        "src/humanoid_navigation/launch/navigation_control_plane.launch.py",
        "src/humanoid_navigation/launch/navigation_route_runtime.launch.py",
        "src/humanoid_navigation/humanoid_navigation/map_context_manager.py",
        "src/humanoid_navigation2/launch/navigation2.launch.py",
        "src/humanoid_navigation2/launch/navigation2_robosense_lidar.launch.py",
        "src/humanoid_navigation2/scripts/validate_multimap_switch_static.py",
        "src/humanoid_navigation2/scripts/simulate_multimap_switch_semantics.py",
        "src/humanoid_navigation2/scripts/validate_map_assets.py",
        "src/humanoid_navigation2/scripts/validate_navigation_refactor_acceptance.py",
        "src/humanoid_navigation2/scripts/validate_route_task_static.py",
    ])
    run_command(["python3", "-m", "py_compile", *files], title="Python/Launch 语法检查")


def validate_static_guards() -> None:
    """运行多地图和路线任务静态护栏。"""
    run_command(
        ["python3", "src/humanoid_navigation2/scripts/validate_multimap_switch_static.py"],
        title="多地图切换静态护栏",
    )
    run_bash(
        ros_env_script("python3 src/humanoid_navigation2/scripts/validate_route_task_static.py"),
        title="路线任务静态护栏",
    )


def validate_multimap_semantics(args: argparse.Namespace) -> None:
    """运行多地图 switch_map 语义模拟。"""
    domain_id = str(args.ros_domain_id or (150 + int(time.time()) % 40))
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = domain_id
    env["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    run_bash(
        ros_env_script("python3 src/humanoid_navigation2/scripts/simulate_multimap_switch_semantics.py"),
        title=f"多地图 switch_map 语义模拟 ROS_DOMAIN_ID={domain_id}",
        env=env,
    )


def validate_hall_map() -> None:
    """只对真实保留的 hall 地图执行导航层启动前校验，不启动真实导航层。"""
    run_command(
        ["python3", "src/humanoid_navigation2/scripts/validate_map_assets.py", "--map-id", "hall"],
        title="hall 地图资产预检",
    )
    env = os.environ.copy()
    env["WORKSPACE"] = str(WORKSPACE_ROOT)
    env["MAP_ID"] = "hall"
    env["VALIDATE_ONLY"] = "1"
    run_command(["./start_navigation_stack.sh"], title="hall 地图导航层 validate-only", env=env)


def validate_launch_discovery() -> None:
    """确认分层 launch 能被 ROS 发现并展示参数。"""
    run_bash(
        ros_env_script("ros2 launch humanoid_bringup robot_control_plane.launch.py --show-args >/tmp/robot_control_plane_args.txt"),
        title="控制层 launch 可发现性",
    )
    run_bash(
        ros_env_script("ros2 launch humanoid_bringup robot_navigation_stack.launch.py --show-args >/tmp/robot_navigation_stack_args.txt"),
        title="导航层 launch 可发现性",
    )
    run_bash(
        ros_env_script("ros2 launch humanoid_bringup robot_real.launch.py --show-args >/tmp/robot_real_args.txt"),
        title="兼容入口 robot_real launch 可发现性",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="新导航系统路线任务 + 多地图改造统一验收")
    parser.add_argument("--build", action="store_true", help="验收前先构建受影响包")
    parser.add_argument("--ros-domain-id", type=int, default=0, help="指定动态模拟使用的 ROS_DOMAIN_ID")
    args = parser.parse_args()

    try:
        build_if_requested(args)
        validate_shell_syntax()
        validate_python_syntax()
        validate_static_guards()
        validate_multimap_semantics(args)
        validate_hall_map()
        validate_launch_discovery()
    finally:
        cleanup_runtime_files()

    print("\n统一验收通过：路线任务、多地图切图、分层启动和 hall 地图启动前校验均符合预期。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
