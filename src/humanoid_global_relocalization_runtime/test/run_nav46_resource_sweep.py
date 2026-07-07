#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_nav46_resource_sweep.py

文件作用：
  1. 这是离线验证辅助脚本，不属于功能源码。
  2. 针对 nav_drift_test46 的真实 /cloud_registered_body，生成多组资源优化参数配置。
  3. 每组配置都会调用 C++ offline evaluator，比较准确率、时序门控结果、CPU、内存和耗时。
  4. 该脚本的产物写入 .codex_tmp，不会修改功能包运行配置。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_nav46_resource_sweep.py --run
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import subprocess
import sys
from pathlib import Path
from typing import Any

import yaml


def repo_root_from_script() -> Path:
    """根据脚本位置反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def load_yaml(path: Path) -> dict[str, Any]:
    """读取作为模板的 YAML 配置。"""
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def dump_yaml(path: Path, config: dict[str, Any]) -> None:
    """写出临时 sweep 配置，并在文件头说明这是自动生成产物。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "# nav46 resource sweep 自动生成配置。",
        "#",
        "# 作用：",
        "#   - 由 test/run_nav46_resource_sweep.py 生成。",
        "#   - 用真实 /cloud_registered_body 验证降 CPU 参数是否仍能通过时序门控。",
        "#   - 该文件位于 .codex_tmp，功能源码和主 YAML 不依赖它。",
        "",
    ]
    path.write_text("\n".join(header) + yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")


def scenario_table() -> list[dict[str, Any]]:
    """定义资源 sweep 场景；每个场景只改少量关键参数，便于判断影响来源。"""
    return [
        {
            "name": "baseline_body30",
            "description": "当前真实 body 30 帧基线，用于和后续轻量参数对照。",
            "patch": {},
        },
        {
            "name": "threads4_refine10",
            "description": "BBS 线程从 8 降到 4，GICP 精配准候选从 20 降到 10，降低峰值 CPU。",
            "patch": {
                "bbs_num_threads": 4,
                "max_refine_candidates": 10,
            },
        },
        {
            "name": "threads2_refine8",
            "description": "更激进地把 BBS 线程降到 2，精配准候选降到 8，用于观察 CPU/耗时折中。",
            "patch": {
                "bbs_num_threads": 2,
                "max_refine_candidates": 8,
            },
        },
        {
            "name": "coarser_scan_threads4",
            "description": "在 4 线程基础上把 scan_leaf_size 从 0.30 增大到 0.40，减少 scan 点数和搜索成本。",
            "patch": {
                "bbs_num_threads": 4,
                "max_refine_candidates": 10,
                "scan_leaf_size": 0.40,
            },
        },
    ]


def params(config: dict[str, Any]) -> dict[str, Any]:
    """取得 ROS 参数字典。"""
    return config["global_relocalization_eval"]["ros__parameters"]


def build_config(template: dict[str, Any], workspace: Path, case: dict[str, Any]) -> dict[str, Any]:
    """把模板配置复制为单个 sweep case，并应用参数补丁。"""
    config = yaml.safe_load(yaml.safe_dump(template, allow_unicode=True, sort_keys=False))
    p = params(config)
    p["input_mode"] = "body"
    p["bag_paths"] = ["/home/ubuntu/nav_drift_test/nav_drift_test46"]
    p["bag_start_frame_skip"] = 0
    p["max_bag_frames"] = 30
    p["bag_frame_stride"] = 1200
    p["use_bag_reference_pose"] = True
    p["save_aligned_cloud"] = False
    p["temporal_window_before"] = 4
    p["temporal_window_after"] = 0
    p["temporal_min_support_frames"] = 2
    p["output_dir"] = str(workspace / ".codex_tmp" / f"global_relocalization_nav46_resource_{case['name']}")
    for key, value in case["patch"].items():
        p[key] = value
    return config


def run_case(workspace: Path, config_path: Path) -> int:
    """运行 C++ offline evaluator，返回进程退出码。"""
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


def read_no_prior(path: Path) -> list[dict[str, str]]:
    """读取任意点启动行，避免三个模拟场景重复统计同一个 scan。"""
    with path.open(newline="", encoding="utf-8") as f:
        return [row for row in csv.DictReader(f) if row.get("scenario_name") == "arbitrary_start_no_prior"]


def f(row: dict[str, str], key: str) -> float:
    """安全读取浮点字段。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return math.nan


def median(values: list[float]) -> float:
    """计算有限数值中位数。"""
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else math.nan


def summarize_case(workspace: Path, case: dict[str, Any]) -> dict[str, str]:
    """读取单个 case 产物并汇总准确率、时序门控和资源指标。"""
    if case["name"] == "baseline_body30":
        out_dir = workspace / ".codex_tmp/global_relocalization_real_body_validation_30frame_v1"
    else:
        out_dir = workspace / ".codex_tmp" / f"global_relocalization_nav46_resource_{case['name']}"
    metrics = read_no_prior(out_dir / "global_relocalization_metrics.csv")
    decisions = read_no_prior(out_dir / "global_relocalization_temporal_decisions.csv")
    reference_rows = [row for row in metrics if row.get("has_reference") == "1"]
    accepted_ref = [
        row for row in decisions
        if row.get("decision") == "accept" and row.get("has_reference") == "1"
    ]
    total = [f(row, "total_ms") for row in metrics]
    cpu = [f(row, "delta_user_cpu_ms") + f(row, "delta_system_cpu_ms") for row in metrics]
    peak_rss = [f(row, "peak_rss_mb") for row in metrics]
    threads = [f(row, "thread_count") for row in metrics]
    return {
        "name": case["name"],
        "description": case["description"],
        "localized": f"{sum(row.get('localized') == '1' for row in metrics)}/{len(metrics)}",
        "reference_success": f"{sum(row.get('success') == '1' for row in reference_rows)}/{len(reference_rows)}",
        "accepted": str(sum(row.get("decision") == "accept" for row in decisions)),
        "rejected": str(sum(row.get("decision") == "reject" for row in decisions)),
        "accepted_ref_success": (
            f"{sum(row.get('refined_success') == '1' for row in accepted_ref)}/{len(accepted_ref)}"
        ),
        "median_total_ms": f"{median(total):.1f}",
        "median_cpu_ms": f"{median(cpu):.1f}",
        "peak_rss_mb": f"{max(peak_rss):.1f}",
        "thread_count_min": f"{min(threads):.0f}",
        "thread_count_max": f"{max(threads):.0f}",
    }


def write_summary(path: Path, rows: list[dict[str, str]]) -> None:
    """写出 sweep 汇总 CSV。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "name",
        "description",
        "localized",
        "reference_success",
        "accepted",
        "rejected",
        "accepted_ref_success",
        "median_total_ms",
        "median_cpu_ms",
        "peak_rss_mb",
        "thread_count_min",
        "thread_count_max",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run nav46 real body resource sweep.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument(
        "--template",
        type=Path,
        default=None,
        help="YAML 模板；默认使用 .codex_tmp/real_body_validation_config.yaml",
    )
    parser.add_argument("--run", action="store_true", help="生成配置后立即运行 evaluator")
    parser.add_argument("--summarize-existing", action="store_true", help="仅读取已存在产物并生成汇总 CSV")
    parser.add_argument("--only", nargs="*", default=None, help="只运行指定 case name")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    template_path = args.template or workspace / ".codex_tmp/real_body_validation_config.yaml"
    template = load_yaml(template_path)
    cases = scenario_table()
    if args.only:
        selected = set(args.only)
        cases = [case for case in cases if case["name"] in selected]
    if not cases:
        print("[nav46_resource_sweep] no cases selected", file=sys.stderr)
        return 2

    config_root = workspace / ".codex_tmp/nav46_resource_sweep_configs"
    for case in cases:
        config = build_config(template, workspace, case)
        config_path = config_root / f"{case['name']}.yaml"
        dump_yaml(config_path, config)
        print(f"[nav46_resource_sweep] generated {config_path}")
        if args.run:
            rc = run_case(workspace, config_path)
            if rc != 0:
                return rc

    if args.run or args.summarize_existing:
        rows = [summarize_case(workspace, case) for case in cases]
        summary_path = workspace / ".codex_tmp/nav46_resource_sweep_summary.csv"
        write_summary(summary_path, rows)
        print(f"[nav46_resource_sweep] wrote {summary_path}")
        for row in rows:
            print(
                "[nav46_resource_sweep] "
                f"{row['name']} localized={row['localized']} ref_success={row['reference_success']} "
                f"accepted_ref_success={row['accepted_ref_success']} "
                f"median_total_ms={row['median_total_ms']} median_cpu_ms={row['median_cpu_ms']} "
                f"peak_rss_mb={row['peak_rss_mb']} threads={row['thread_count_min']}-{row['thread_count_max']}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
