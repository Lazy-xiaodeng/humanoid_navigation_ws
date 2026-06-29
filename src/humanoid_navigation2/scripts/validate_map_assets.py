#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多地图资产预检工具。

这个脚本只检查 `data/maps/map_registry.json` 中登记的地图文件是否存在，
不启动 ROS、Nav2、雷达或定位节点。用途：

1. 一键启动前确认当前地图文件齐全。
2. 真实切换到 hall1/hall2 前，先确认目标地图 2D/3D 资产已经生成。
3. 给 APP/现场同事一个清晰的缺文件错误列表，避免切图时才发现问题。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List


REQUIRED_FOR_NAVIGATION = (
    "map_yaml_file",
    "map_pgm_file",
    "open3d_prior_map_file",
)

OPTIONAL_BUT_RECOMMENDED = (
    "map_posegraph_file",
    "map_data_file",
    "raw_pcd_file",
    "standard_pcd_file",
    "waypoints_file",
)


def find_workspace_root() -> Path:
    """从脚本位置向上定位工作区根目录。"""
    current = Path(__file__).resolve()
    for candidate in [current, *current.parents]:
        if (candidate / "data/maps/map_registry.json").exists() and (candidate / "start_navigation.sh").exists():
            return candidate
    raise RuntimeError("无法定位 humanoid_ws 工作区根目录")


def normalize_path(value: Any) -> Path | None:
    """把注册表里的路径字段转换为 Path；空值返回 None。"""
    text = str(value or "").strip()
    if not text:
        return None
    return Path(text).expanduser()


def load_registry(registry_path: Path) -> Dict[str, Any]:
    """读取地图注册表并做基础结构校验。"""
    if not registry_path.exists():
        raise FileNotFoundError(f"地图注册表不存在: {registry_path}")
    data = json.loads(registry_path.read_text(encoding="utf-8"))
    if not isinstance(data.get("maps"), list):
        raise ValueError("地图注册表格式错误: maps 必须是列表")
    return data


def selected_maps(registry: Dict[str, Any], map_id: str, all_maps: bool) -> List[Dict[str, Any]]:
    """按参数选择要检查的地图。"""
    maps = [item for item in registry.get("maps", []) if isinstance(item, dict)]
    if all_maps:
        return maps
    target_id = map_id or str(registry.get("current_map_id") or registry.get("default_map_id") or "hall")
    selected = [item for item in maps if str(item.get("map_id", "")).strip() == target_id]
    if not selected:
        raise ValueError(f"地图未注册: {target_id}")
    return selected


def check_field(map_info: Dict[str, Any], field: str) -> tuple[bool, str]:
    """检查单个路径字段是否存在。"""
    path = normalize_path(map_info.get(field))
    if path is None:
        return False, "<empty>"
    return path.exists(), str(path)


def validate_one_map(map_info: Dict[str, Any], require_optional: bool) -> Dict[str, Any]:
    """检查单张地图并返回结构化结果。"""
    map_id = str(map_info.get("map_id", "") or "<unknown>")
    enabled = bool(map_info.get("enabled", True))
    missing_required = []
    missing_optional = []
    present = []

    for field in REQUIRED_FOR_NAVIGATION:
        exists, path = check_field(map_info, field)
        if exists:
            present.append((field, path))
        else:
            missing_required.append((field, path))

    for field in OPTIONAL_BUT_RECOMMENDED:
        exists, path = check_field(map_info, field)
        if exists:
            present.append((field, path))
        else:
            missing_optional.append((field, path))

    ok = enabled and not missing_required and (not require_optional or not missing_optional)
    return {
        "map_id": map_id,
        "enabled": enabled,
        "ok": ok,
        "present": present,
        "missing_required": missing_required,
        "missing_optional": missing_optional,
    }


def print_result(result: Dict[str, Any], verbose: bool) -> None:
    """打印单张地图检查结果。"""
    status = "OK" if result["ok"] else "FAIL"
    enabled_text = "enabled" if result["enabled"] else "disabled"
    print(f"[{status}] map_id={result['map_id']} ({enabled_text})")

    if result["missing_required"]:
        print("  缺失必需文件:")
        for field, path in result["missing_required"]:
            print(f"    - {field}: {path}")

    if result["missing_optional"]:
        print("  缺失可选/建议文件:")
        for field, path in result["missing_optional"]:
            print(f"    - {field}: {path}")

    if verbose and result["present"]:
        print("  已存在文件:")
        for field, path in result["present"]:
            print(f"    - {field}: {path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="检查多地图注册表中地图资产是否齐全")
    parser.add_argument("--registry", default="", help="自定义 map_registry.json 路径")
    parser.add_argument("--map-id", default="", help="只检查指定 map_id；默认检查 current_map_id")
    parser.add_argument("--all", action="store_true", help="检查注册表中的所有地图")
    parser.add_argument("--require-optional", action="store_true", help="可选/建议文件缺失也返回失败")
    parser.add_argument("--verbose", action="store_true", help="输出已存在文件列表")
    args = parser.parse_args()

    workspace = find_workspace_root()
    registry_path = Path(args.registry).expanduser() if args.registry else workspace / "data/maps/map_registry.json"
    registry = load_registry(registry_path)

    results = [
        validate_one_map(map_info, require_optional=args.require_optional)
        for map_info in selected_maps(registry, args.map_id, args.all)
    ]
    for result in results:
        print_result(result, args.verbose)

    failed = [result for result in results if not result["ok"]]
    if failed:
        print("\n地图资产预检失败：存在缺失或禁用的地图。")
        return 1

    print("\n地图资产预检通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
