#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_accumulated_precision_sweep.py

文件作用：
  1. 复用 run_accumulated_waypoint_sweep.py 已经生成的多帧累积 scan PCD。
  2. 不再重复读取大 bag，只改 evaluator YAML 中的搜索/精配准参数。
  3. 对仍然失败的点位尝试更细 BBS 分辨率、更多候选和更多精配准候选。
  4. 输出 CSV，记录“牺牲 CPU 后是否能救回失败位置”。
"""

from __future__ import annotations

import argparse
import csv
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Variant:
    """一次精细搜索参数组合。"""

    name: str
    scan_leaf_size: float
    bbs_min_level_res: float
    bbs_num_threads: int
    top_k: int
    max_refine_candidates: int
    refine_methods_for_sweep: str
    refine_max_correspondence_distance: float
    gicp_max_correspondence_distance: float


def repo_root_from_script() -> Path:
    """根据脚本路径反推 humanoid_ws 根目录。"""
    return Path(__file__).resolve().parents[3]


def replace_param(text: str, key: str, value: str) -> str:
    """替换生成 YAML 中的单行参数。"""
    pattern = re.compile(rf"^(\s*{re.escape(key)}:\s*).*$", re.MULTILINE)
    new_text, count = pattern.subn(rf"\g<1>{value}", text)
    if count != 1:
        raise RuntimeError(f"parameter {key} not found exactly once")
    return new_text


def make_variant_yaml(
    base_yaml: Path,
    output_yaml: Path,
    output_dir: Path,
    variant: Variant,
) -> None:
    """从基础 YAML 派生一个强搜索配置。"""
    text = base_yaml.read_text(encoding="utf-8")
    text = replace_param(text, "output_dir", f'"{output_dir}"')
    text = replace_param(text, "scan_leaf_size", f"{variant.scan_leaf_size:.2f}")
    text = replace_param(text, "bbs_min_level_res", f"{variant.bbs_min_level_res:.2f}")
    text = replace_param(text, "bbs_num_threads", str(variant.bbs_num_threads))
    text = replace_param(text, "top_k", str(variant.top_k))
    text = replace_param(text, "max_refine_candidates", str(variant.max_refine_candidates))
    text = replace_param(text, "refine_methods_for_sweep", variant.refine_methods_for_sweep)
    text = replace_param(
        text,
        "refine_max_correspondence_distance",
        f"{variant.refine_max_correspondence_distance:.2f}",
    )
    text = replace_param(
        text,
        "gicp_max_correspondence_distance",
        f"{variant.gicp_max_correspondence_distance:.2f}",
    )
    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    output_yaml.write_text(text, encoding="utf-8")


def run_offline_eval(workspace: Path, config_path: Path) -> int:
    """调用 C++ offline evaluator。"""
    command = [
        "ros2",
        "run",
        "humanoid_global_relocalization_runtime",
        "global_relocalization_offline_eval",
        "--config",
        str(config_path),
    ]
    process = subprocess.run(command, cwd=str(workspace), text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    print(process.stdout, end="")
    if process.stderr:
        print(process.stderr, end="", file=sys.stderr)
    return process.returncode


def read_metrics(output_dir: Path) -> list[dict[str, str]]:
    """读取 evaluator 输出的所有 refine method 结果。"""
    path = output_dir / "global_relocalization_metrics.csv"
    with path.open(newline="", encoding="utf-8") as f:
        return [row for row in csv.DictReader(f) if row.get("scenario_name") == "arbitrary_start_no_prior"]


def write_summary(path: Path, rows: list[dict[str, str]]) -> None:
    """写强搜索 sweep 汇总 CSV。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "variant",
        "case_name",
        "waypoint_id",
        "frames",
        "refine_method",
        "localized",
        "success",
        "translation_error_m",
        "yaw_error_deg",
        "refined_candidate_rank",
        "refine_fitness_score",
        "scan_points",
        "search_ms",
        "refine_ms",
        "total_ms",
        "delta_cpu_ms",
        "peak_rss_mb",
        "thread_count",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_case_name(case_name: str) -> tuple[str, str]:
    """从 wp4_3f 这种名字解析 waypoint_id 和帧数。"""
    match = re.fullmatch(r"wp(.+)_(\d+)f", case_name)
    if not match:
        raise RuntimeError(f"bad case name: {case_name}")
    return match.group(1), match.group(2)


def default_variants() -> list[Variant]:
    """默认验证两个强度，先看成功率，再看 CPU 代价。"""
    return [
        Variant(
            name="precision_gicp",
            scan_leaf_size=0.20,
            bbs_min_level_res=0.50,
            bbs_num_threads=4,
            top_k=80,
            max_refine_candidates=30,
            refine_methods_for_sweep='["gicp"]',
            refine_max_correspondence_distance=1.20,
            gicp_max_correspondence_distance=1.20,
        ),
        Variant(
            name="precision_multi_refine",
            scan_leaf_size=0.20,
            bbs_min_level_res=0.50,
            bbs_num_threads=4,
            top_k=80,
            max_refine_candidates=30,
            refine_methods_for_sweep='["gicp", "icp", "ndt"]',
            refine_max_correspondence_distance=1.20,
            gicp_max_correspondence_distance=1.20,
        ),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description="Run high-cost precision sweep on accumulated scan PCD cases.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument(
        "--accumulated-root",
        type=Path,
        default=Path(".codex_tmp/accumulated_waypoint_sweep"),
        help="多帧累积 sweep 输出目录",
    )
    parser.add_argument("--cases", nargs="*", default=["wp4_3f", "wp4_5f", "wp8_3f", "wp8_5f"], help="要复测的 case")
    parser.add_argument("--run", action="store_true", help="生成 YAML 后立即运行 evaluator")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    accumulated_root = (workspace / args.accumulated_root).resolve() if not args.accumulated_root.is_absolute() else args.accumulated_root
    output_root = workspace / ".codex_tmp/accumulated_precision_sweep"
    rows: list[dict[str, str]] = []

    for variant in default_variants():
        for case_name in args.cases:
            base_yaml = accumulated_root / "configs" / f"{case_name}.yaml"
            if not base_yaml.exists():
                print(f"[precision_sweep] missing base yaml: {base_yaml}", file=sys.stderr)
                return 2
            eval_output_dir = output_root / "eval" / variant.name / case_name
            yaml_path = output_root / "configs" / variant.name / f"{case_name}.yaml"
            make_variant_yaml(base_yaml, yaml_path, eval_output_dir.resolve(), variant)
            print(f"[precision_sweep] generated variant={variant.name} case={case_name} yaml={yaml_path}")
            if not args.run:
                continue

            rc = run_offline_eval(workspace, yaml_path)
            if rc != 0:
                return rc
            waypoint_id, frames = parse_case_name(case_name)
            for metric in read_metrics(eval_output_dir):
                rows.append(
                    {
                        "variant": variant.name,
                        "case_name": case_name,
                        "waypoint_id": waypoint_id,
                        "frames": frames,
                        "refine_method": metric.get("refine_method", ""),
                        "localized": metric.get("localized", ""),
                        "success": metric.get("success", ""),
                        "translation_error_m": metric.get("translation_error_m", ""),
                        "yaw_error_deg": metric.get("yaw_error_deg", ""),
                        "refined_candidate_rank": metric.get("refined_candidate_rank", ""),
                        "refine_fitness_score": metric.get("refine_fitness_score", ""),
                        "scan_points": metric.get("scan_points", ""),
                        "search_ms": metric.get("search_ms", ""),
                        "refine_ms": metric.get("refine_ms", ""),
                        "total_ms": metric.get("total_ms", ""),
                        "delta_cpu_ms": (
                            f"{float(metric.get('delta_user_cpu_ms', '0')) + float(metric.get('delta_system_cpu_ms', '0')):.3f}"
                        ),
                        "peak_rss_mb": metric.get("peak_rss_mb", ""),
                        "thread_count": metric.get("thread_count", ""),
                    }
                )

    if rows:
        summary_path = output_root / "summary.csv"
        write_summary(summary_path, rows)
        print(f"[precision_sweep] wrote {summary_path}")
        for row in rows:
            print(
                "[precision_sweep] "
                f"variant={row['variant']} case={row['case_name']} method={row['refine_method']} "
                f"success={row['success']} err={row['translation_error_m']}m/{row['yaw_error_deg']}deg "
                f"total={row['total_ms']}ms cpu={row['delta_cpu_ms']}ms"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
