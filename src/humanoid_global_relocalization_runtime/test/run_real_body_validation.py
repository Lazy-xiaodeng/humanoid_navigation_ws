#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_real_body_validation.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 当机器上补录了真实 /cloud_registered_body bag 后，自动生成真实 body 输入链路验证 YAML。
  3. 可选调用 global_relocalization_offline_eval，生成门禁要求的真实 body 验证产物。
  4. 如果当前 latest inventory 仍没有真实 body bag，本脚本会明确输出 KNOWN_LIMIT 并正常退出。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_real_body_validation.py --run
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from pathlib import Path

import yaml


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def read_inventory(path: Path) -> list[dict[str, str]]:
    """读取 scan_bag_inventory.py 生成的 latest inventory CSV。"""
    if not path.exists():
        raise FileNotFoundError(f"latest inventory does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def load_template(path: Path) -> dict:
    """读取合成 body 验证 YAML 作为真实 body 验证的参数模板。"""
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def write_config(path: Path, config: dict) -> None:
    """写出自动生成的真实 body 验证 YAML。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "# real_body_validation_config 自动生成文件。",
        "#",
        "# 作用：",
        "#   - 由 test/run_real_body_validation.py 根据 latest bag inventory 生成。",
        "#   - 用真实 /cloud_registered_body bag 验证 body 输入链路。",
        "#   - 输出目录固定为 .codex_tmp/global_relocalization_real_body_validation_v1，供目标证据矩阵检查。",
        "",
    ]
    body = yaml.safe_dump(config, allow_unicode=True, sort_keys=False)
    path.write_text("\n".join(header) + body, encoding="utf-8")


def body_bag_paths(rows: list[dict[str, str]]) -> list[str]:
    """从 inventory 中提取真实 /cloud_registered_body bag 路径。"""
    return sorted(row["bag_path"] for row in rows if row.get("body_ready") == "yes")


def build_config(workspace: Path, bag_paths: list[str], max_frames: int, stride: int) -> dict:
    """基于现有 body 参数模板构造真实 body 验证配置。"""
    template_path = workspace / "src/humanoid_global_relocalization_runtime/test/synthetic_body_nav_drift_all.yaml"
    config = load_template(template_path)
    params = config["global_relocalization_eval"]["ros__parameters"]

    params["input_mode"] = "body"
    params["bag_paths"] = bag_paths
    params["bag_start_frame_skip"] = 0
    params["max_bag_frames"] = max_frames
    params["bag_frame_stride"] = stride
    params["odom_time_tolerance_sec"] = 0.10
    params["output_dir"] = str(workspace / ".codex_tmp/global_relocalization_real_body_validation_v1")
    params["use_bag_reference_pose"] = True
    params["save_aligned_cloud"] = False
    params["bbs_num_threads"] = 2
    params["max_refine_candidates"] = 8
    # 这里必须使用 C++ config_loader.cpp 实际读取的新参数名。
    # 早期脚本里曾写过 temporal_window_before / temporal_min_support_frames，
    # 这些旧名字不会被当前 C++ 节点读取，容易造成“以为改了时序窗口但实际没生效”。
    params["temporal_consistency_window_before"] = 4
    params["temporal_consistency_window_after"] = 0
    params["temporal_consistency_online_min_support_frames"] = 2
    return config


def run_offline_eval(workspace: Path, config_path: Path) -> int:
    """调用 C++ 离线 evaluator 运行真实 body 验证。"""
    command = [
        "ros2",
        "run",
        "humanoid_global_relocalization_runtime",
        "global_relocalization_offline_eval",
        "--config",
        str(config_path),
    ]
    process = subprocess.run(
        command,
        cwd=str(workspace),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    print(process.stdout, end="")
    if process.stderr:
        print(process.stderr, end="", file=sys.stderr)
    return process.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate and optionally run real /cloud_registered_body validation.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--inventory", type=Path, default=None, help="latest inventory CSV")
    parser.add_argument("--config-output", type=Path, default=None, help="自动生成的真实 body 验证 YAML")
    parser.add_argument("--max-frames", type=int, default=10, help="每个真实 body bag 最多抽样帧数")
    parser.add_argument("--stride", type=int, default=250, help="真实 body bag 抽样帧间隔")
    parser.add_argument("--run", action="store_true", help="生成 YAML 后立即调用 offline evaluator")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    inventory = args.inventory or workspace / ".codex_tmp/bag_inventory_home_scan_latest.csv"
    config_output = args.config_output or workspace / ".codex_tmp/real_body_validation_config.yaml"

    rows = read_inventory(inventory)
    bags = body_bag_paths(rows)
    if not bags:
        print("[real_body_validation] KNOWN_LIMIT no real /cloud_registered_body bag in latest inventory")
        return 0

    config = build_config(workspace, bags, args.max_frames, args.stride)
    write_config(config_output, config)
    print(f"[real_body_validation] generated {config_output}")
    print(f"[real_body_validation] body_bags={len(bags)}")
    for bag in bags:
        print(f"  - {bag}")

    if not args.run:
        print("[real_body_validation] dry-run only; pass --run to execute offline evaluator")
        return 0

    return run_offline_eval(workspace, config_output)


if __name__ == "__main__":
    raise SystemExit(main())
