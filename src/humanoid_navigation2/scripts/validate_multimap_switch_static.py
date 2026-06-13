#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多地图切换静态护栏。

这个脚本不启动真实 ROS 节点，只检查源码和脚本里的关键边界：
1. APP/ROS 控制层必须常驻，切图脚本不能停止 websocket / 地图上下文。
2. 导航定位层必须按 map_id 重启，并同时接收 2D map yaml 和 3D prior map。
3. 目标地图校验必须发生在停止旧导航层之前，避免切坏后当前地图也不可用。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


def find_workspace_root() -> Path:
    """从源码路径向上寻找工作区根目录，保证源码态和 install 态运行都能定位。"""
    current = Path(__file__).resolve()
    for candidate in [current, *current.parents]:
        if (candidate / "src/humanoid_navigation2").exists() and (candidate / "start_navigation.sh").exists():
            return candidate
    raise RuntimeError("无法定位 humanoid_ws 工作区根目录")


WORKSPACE_ROOT = find_workspace_root()


def read_text(relative_path: str) -> str:
    """读取工作区内文件，缺失时直接报错，避免校验漏掉关键产物。"""
    path = WORKSPACE_ROOT / relative_path
    if not path.exists():
        raise FileNotFoundError(f"缺失文件: {relative_path}")
    return path.read_text(encoding="utf-8")


def require_contains(text: str, needle: str, label: str, failures: list[str]) -> None:
    """要求文本包含指定片段。"""
    if needle not in text:
        failures.append(f"缺失: {label} -> {needle}")


def require_not_contains(text: str, needle: str, label: str, failures: list[str]) -> None:
    """要求文本不包含指定片段。"""
    if needle in text:
        failures.append(f"不应存在: {label} -> {needle}")


def require_order(text: str, first: str, second: str, label: str, failures: list[str]) -> None:
    """要求 first 出现在 second 之前。"""
    first_index = text.find(first)
    second_index = text.find(second)
    if first_index < 0 or second_index < 0 or first_index >= second_index:
        failures.append(f"顺序错误: {label} -> {first} 必须早于 {second}")


def strip_shell_comments(text: str) -> str:
    """去掉 shell 注释，避免注释里的旧命令触发误判。"""
    lines = []
    for line in text.splitlines():
        stripped = line.lstrip()
        if stripped.startswith("#"):
            continue
        lines.append(line)
    return "\n".join(lines)


def main() -> int:
    failures: list[str] = []

    start_navigation = read_text("start_navigation.sh")
    stop_navigation = read_text("stop_navigation.sh")
    switch_navigation_map = read_text("switch_navigation_map.sh")
    start_navigation_stack = read_text("start_navigation_stack.sh")
    stop_navigation_stack = read_text("stop_navigation_stack.sh")
    start_ros_control_plane = read_text("start_ros_control_plane.sh")
    stop_ros_control_plane = read_text("stop_ros_control_plane.sh")
    robot_real = read_text("src/humanoid_bringup/launch/robot_real.launch.py")
    robot_control_plane = read_text("src/humanoid_bringup/launch/robot_control_plane.launch.py")
    robot_navigation_stack = read_text("src/humanoid_bringup/launch/robot_navigation_stack.launch.py")
    navigation_control_plane = read_text("src/humanoid_navigation/launch/navigation_control_plane.launch.py")
    navigation_route_runtime = read_text("src/humanoid_navigation/launch/navigation_route_runtime.launch.py")
    map_context_manager = read_text("src/humanoid_navigation/humanoid_navigation/map_context_manager.py")
    gitignore = read_text(".gitignore")

    switch_without_comments = strip_shell_comments(switch_navigation_map)
    stop_stack_without_comments = strip_shell_comments(stop_navigation_stack)
    start_stack_without_comments = strip_shell_comments(start_navigation_stack)

    # 一键启动必须启动两层；运行期切图只重启导航层。
    require_contains(start_navigation, "start_ros_control_plane.sh", "一键启动拉起控制层", failures)
    require_contains(start_navigation, "start_navigation_stack.sh", "一键启动拉起导航层", failures)
    require_not_contains(start_navigation, "ros2 launch humanoid_bringup robot_real.launch.py", "start_navigation 不再直接拉全家桶 launch", failures)

    # 完整停止可以停两层，但 switch_map 不能调用完整停止入口。
    require_contains(stop_navigation, "stop_navigation_stack.sh", "完整停止先停导航层", failures)
    require_contains(stop_navigation, "stop_ros_control_plane.sh", "完整停止再停控制层", failures)
    require_not_contains(switch_without_comments, "stop_navigation.sh", "切图脚本不能停止完整 ROS 系统", failures)
    require_not_contains(switch_without_comments, "start_navigation.sh", "切图脚本不能重新启动完整 ROS 系统", failures)
    require_contains(switch_navigation_map, "VALIDATE_ONLY=1", "切图前先 validate-only 校验目标地图", failures)
    require_order(
        switch_navigation_map,
        "VALIDATE_ONLY=1",
        '"$WORKSPACE/stop_navigation_stack.sh" || true',
        "切图必须先校验目标地图再停止旧导航层",
        failures,
    )
    require_contains(switch_navigation_map, "start_navigation_stack.sh", "切图脚本重启导航层", failures)

    # 导航层启动脚本必须把 map_id 解析成 2D/3D 地图路径，并且 validate-only 不能写 current_map_id。
    require_contains(start_navigation_stack, "map_yaml_file", "导航层校验/传递 2D map yaml", failures)
    require_contains(start_navigation_stack, "open3d_prior_map_file", "导航层校验/传递 3D prior map", failures)
    require_contains(start_navigation_stack, "robosense_lidar_localization.yaml", "导航层生成 RoboSense runtime config", failures)
    require_contains(start_navigation_stack, 'os.environ.get("VALIDATE_ONLY", "0") != "1"', "validate-only 不写 current_map_id", failures)
    require_order(
        start_navigation_stack,
        'resolve_map_env "$WORKSPACE" "$REQUESTED_MAP_ID" "$ACTIVE_MAP_ENV_FILE"',
        '"$WORKSPACE/stop_navigation_stack.sh" || true',
        "导航层启动必须先解析目标地图再停止旧导航层",
        failures,
    )
    require_contains(start_navigation_stack, "robot_navigation_stack.launch.py", "导航层脚本启动 robot_navigation_stack", failures)

    # 停导航层时不能杀控制层相关进程。
    for forbidden in (
        "websocket_server_node",
        "data_integration_node",
        "websocket_client_node",
        "message_bridge_node",
        "dynamic_waypoints_manager",
        "map_context_manager",
        "facial_driver",
        "robot_control_plane.launch.py",
    ):
        require_not_contains(stop_stack_without_comments, forbidden, f"stop_navigation_stack 不应停止控制层 {forbidden}", failures)
    require_contains(stop_navigation_stack, "navigation_state_manager", "导航层停止包含 navigation_state_manager", failures)
    require_contains(stop_navigation_stack, "prior_map_odom_bridge", "导航层停止包含 prior_map_odom_bridge", failures)

    # 控制层脚本必须只管理控制层。
    require_contains(start_ros_control_plane, "robot_control_plane.launch.py", "控制层脚本启动 robot_control_plane", failures)
    require_contains(stop_ros_control_plane, "websocket_server_node", "控制层停止包含 websocket", failures)
    require_contains(stop_ros_control_plane, "map_context_manager", "控制层停止包含 map_context_manager", failures)
    require_not_contains(stop_ros_control_plane, "nav2_", "stop_ros_control_plane 不应停止 Nav2", failures)
    require_not_contains(stop_ros_control_plane, "prior_map_odom_bridge", "stop_ros_control_plane 不应停止定位 bridge", failures)

    # launch 分层必须清晰，避免同名 navigation_state_manager 或 websocket 重复启动。
    require_contains(robot_real, "robot_control_plane.launch.py", "robot_real include 控制层", failures)
    require_contains(robot_real, "robot_navigation_stack.launch.py", "robot_real include 导航层", failures)
    require_contains(robot_control_plane, "navigation_control_plane.launch.py", "控制层 include 点位/地图上下文", failures)
    require_contains(robot_control_plane, "websocket_server.launch.py", "控制层 include websocket", failures)
    require_contains(robot_navigation_stack, "navigation2_robosense_lidar.launch.py", "导航层 include RO 定位 Nav2", failures)
    require_contains(robot_navigation_stack, "navigation2.launch.py", "导航层 include OP 定位 Nav2", failures)
    require_contains(robot_navigation_stack, "navigation_route_runtime.launch.py", "导航层 include 路线运行层", failures)
    require_not_contains(
        navigation_control_plane,
        "executable='navigation_state_manager'",
        "控制层不应启动 navigation_state_manager",
        failures,
    )
    require_not_contains(robot_navigation_stack, "websocket_server.launch.py", "导航层不应启动 websocket", failures)
    require_contains(navigation_route_runtime, "navigation_state_manager", "路线运行层启动 navigation_state_manager", failures)

    # map_context_manager 必须是常驻状态机，负责触发脚本并继续等待定位 ready。
    require_contains(map_context_manager, "subprocess.Popen", "map_context_manager 异步触发切图脚本", failures)
    require_contains(map_context_manager, "switch_process.poll()", "map_context_manager 跟踪切图脚本退出状态", failures)
    require_contains(map_context_manager, "map_switch_script", "map_context_manager 使用 map_switch_script 参数", failures)
    require_contains(map_context_manager, "prior_map_file_missing", "switch_map 校验 3D prior map 缺失", failures)
    require_contains(map_context_manager, "map_switch_restart_started", "switch_map 推送重启开始状态", failures)
    require_contains(map_context_manager, "map_ready", "switch_map 推送 ready 结果", failures)

    # 运行时生成文件必须忽略，避免把当前地图 env/runtime YAML 当源码提交。
    require_contains(gitignore, ".active_navigation_map.env", ".gitignore 忽略 active map env", failures)
    require_contains(gitignore, "data/runtime_maps/", ".gitignore 忽略 runtime map config", failures)
    require_contains(gitignore, ".navigation_stack.pid", ".gitignore 忽略导航层 pid", failures)
    require_contains(gitignore, ".ros_control_plane.pid", ".gitignore 忽略控制层 pid", failures)

    # 防止脚本误伤主工作区。
    runtime_texts = {
        "start_navigation.sh": start_navigation,
        "start_navigation_stack.sh": start_navigation_stack,
        "switch_navigation_map.sh": switch_navigation_map,
        "src/humanoid_navigation/launch/navigation_control_plane.launch.py": navigation_control_plane,
        "src/humanoid_navigation/humanoid_navigation/map_context_manager.py": map_context_manager,
    }
    for path, text in runtime_texts.items():
        if path != "src/humanoid_navigation/humanoid_navigation/map_context_manager.py":
            require_not_contains(text, "/home/ubuntu/humanoid_ws", f"{path} 不应引用主工作区路径", failures)

    if failures:
        print("多地图切换静态校验失败：")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("多地图切换静态校验通过：控制层常驻、导航层重启、切图前校验等关键边界均存在。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
