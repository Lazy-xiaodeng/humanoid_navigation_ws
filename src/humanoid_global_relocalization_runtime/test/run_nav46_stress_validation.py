#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_nav46_stress_validation.py

文件作用：
  1. 这是 nav_drift_test46 专项压力验证脚本，不属于线上功能源码。
  2. 自动生成 body 与 registered_world 两路输入的临时 YAML，并调用 C++ offline evaluator。
  3. 沿 bag46 时间轴均匀抽取更多位置，用于模拟任意点启动和定位大跳后的全局重定位恢复。
  4. 汇总单帧重定位、时序门控恢复、CPU、内存、耗时和危险误接受情况，帮助判断算法稳定性。

使用示例：
  source /opt/ros/jazzy/setup.bash
  source install/local_setup.bash
  python3 src/humanoid_global_relocalization_runtime/test/run_nav46_stress_validation.py --run
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import shutil
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
    """读取主验证 YAML，复用当前推荐参数作为压力测试基线。"""
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def params(config: dict[str, Any]) -> dict[str, Any]:
    """取得 ROS 参数字典，避免多处重复写长 key。"""
    return config["global_relocalization_eval"]["ros__parameters"]


def dump_generated_yaml(path: Path, config: dict[str, Any], note: str) -> None:
    """写出自动生成的临时 YAML，并在文件头说明来源和用途。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "# nav46 stress validation 自动生成配置。",
        "#",
        "# 作用：",
        "#   - 由 test/run_nav46_stress_validation.py 生成。",
        "#   - 用 bag46 扩大抽样验证全局重定位在更多启动点/漂移恢复点上的稳定性。",
        "#   - 该文件只位于 .codex_tmp，不作为线上运行配置。",
        f"#   - {note}",
        "",
    ]
    path.write_text(
        "\n".join(header) + yaml.safe_dump(config, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def topic_message_count(bag_path: Path, topic_name: str) -> int:
    """从 rosbag2 metadata.yaml 读取指定话题消息数，用于随机帧索引边界。"""
    metadata_path = bag_path / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    topics = metadata["rosbag2_bagfile_information"]["topics_with_message_count"]
    for topic in topics:
        if topic["topic_metadata"]["name"] == topic_name:
            return int(topic["message_count"])
    raise RuntimeError(f"topic {topic_name} not found in {metadata_path}")


def make_random_indices(
    bag_path: Path,
    mode: str,
    count: int,
    seed: int,
    min_index: int,
    max_index: int | None,
) -> list[int]:
    """按输入模式生成可复现的随机点云帧序号列表。"""
    topic = "/cloud_registered_body" if mode == "body" else "/fast_lio/cloud_registered"
    total = topic_message_count(bag_path, topic)
    begin = max(0, min_index)
    end = min(total - 1, max_index if max_index is not None else total - 1)
    if begin > end:
        raise RuntimeError(f"invalid random range [{begin}, {end}] for topic {topic} total={total}")
    population = list(range(begin, end + 1))
    sample_count = min(count, len(population))
    rng = random.Random(seed)
    return sorted(rng.sample(population, sample_count))


def build_case_config(
    template: dict[str, Any],
    workspace: Path,
    mode: str,
    frames: int,
    stride: int,
    skip: int,
    output_dir: Path,
    sample_indices: list[int] | None = None,
) -> dict[str, Any]:
    """基于主 YAML 构造单个输入模式的压力测试配置。"""
    config = yaml.safe_load(yaml.safe_dump(template, allow_unicode=True, sort_keys=False))
    p = params(config)

    # 这里固定使用 bag46，因为它同时包含真实 /cloud_registered_body、/fast_lio/cloud_registered、
    # /odom 和 /robot_realpose，能对两路输入做同一时间轴的公平对照。
    p["input_mode"] = mode
    p["bag_paths"] = ["/home/ubuntu/nav_drift_test/nav_drift_test46"]
    p["bag_start_frame_skip"] = skip
    p["max_bag_frames"] = len(sample_indices) if sample_indices else frames
    p["bag_frame_stride"] = stride
    p["bag_sample_frame_indices"] = sample_indices or []
    p["odom_time_tolerance_sec"] = 0.10
    p["use_bag_reference_pose"] = True
    p["reference_time_tolerance_sec"] = 0.05
    p["save_aligned_cloud"] = False
    p["output_dir"] = str(output_dir)

    # 当前推荐低负载参数：保持 0.30m scan 体素，不使用已证明有误接受风险的 0.40m。
    p["scan_leaf_size"] = 0.30
    p["bbs_num_threads"] = 2
    p["max_refine_candidates"] = 8
    p["refine_method"] = "gicp"
    p["refine_methods_for_sweep"] = ["gicp"]

    # 压测仍使用因果时序门控：只看当前帧和历史帧，不偷看未来帧。
    p["enable_temporal_consistency"] = True
    p["temporal_consistency_window_before"] = 4
    p["temporal_consistency_window_after"] = 0
    p["temporal_consistency_online_min_support_frames"] = 2
    return config


def run_offline_eval(workspace: Path, config_path: Path) -> int:
    """调用 C++ offline evaluator，stdout/stderr 透传给当前终端。"""
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


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV；如果文件缺失则返回空列表，调用方再写入失败摘要。"""
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def no_prior_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    """只统计 arbitrary_start_no_prior，避免三个模拟场景重复计数同一次搜索。"""
    return [row for row in rows if row.get("scenario_name") == "arbitrary_start_no_prior"]


def to_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """安全读取浮点字段，CSV 为空或非法时返回 NaN。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def median(values: list[float]) -> float:
    """计算有限数值中位数，用于抵抗极慢帧对典型耗时的污染。"""
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else math.nan


def percentile(values: list[float], pct: float) -> float:
    """计算简单百分位，用于观察尾部误差和尾部耗时。"""
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    if len(finite) == 1:
        return finite[0]
    pos = (len(finite) - 1) * pct
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return finite[lo]
    return finite[lo] * (hi - pos) + finite[hi] * (pos - lo)


def key(row: dict[str, str]) -> str:
    """用 stamp 作为 metrics 与 temporal decision 的连接键。"""
    return f"{to_float(row, 'stamp_sec', 0.0):.6f}"


def summarize_case(label: str, output_dir: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    """汇总单个输入模式的准确率、恢复门控、资源消耗和问题帧。"""
    metrics = no_prior_rows(read_csv(output_dir / "global_relocalization_metrics.csv"))
    decisions = no_prior_rows(read_csv(output_dir / "global_relocalization_temporal_decisions.csv"))
    decisions_by_stamp = {key(row): row for row in decisions}

    reference_rows = [row for row in metrics if row.get("has_reference") == "1"]
    localized_rows = [row for row in metrics if row.get("localized") == "1"]
    single_success = [row for row in reference_rows if row.get("success") == "1"]
    single_fail = [row for row in reference_rows if row.get("success") != "1"]

    accepted = [row for row in decisions if row.get("decision") == "accept"]
    rejected = [row for row in decisions if row.get("decision") == "reject"]
    accepted_ref = [row for row in accepted if row.get("has_reference") == "1"]
    accepted_ref_success = [row for row in accepted_ref if row.get("refined_success") == "1"]
    false_accept = [row for row in accepted_ref if row.get("refined_success") != "1"]

    # 这里统计 temporal 是否救回“单帧最终结果错误、但稳定候选精配准正确”的帧。
    recovered_from_single_fail = 0
    conservative_reject_success = 0
    problem_rows: list[dict[str, str]] = []
    for metric in metrics:
        stamp = key(metric)
        decision = decisions_by_stamp.get(stamp)
        if metric.get("has_reference") != "1":
            continue
        if decision and decision.get("decision") == "accept":
            if metric.get("success") != "1" and decision.get("refined_success") == "1":
                recovered_from_single_fail += 1
            if decision.get("refined_success") != "1":
                problem_rows.append(problem_row(label, "temporal_false_accept", metric, decision))
        elif decision and decision.get("decision") == "reject":
            if metric.get("success") == "1":
                conservative_reject_success += 1
            problem_rows.append(problem_row(label, "temporal_reject", metric, decision))
        if metric.get("success") != "1":
            problem_rows.append(problem_row(label, "single_frame_fail", metric, decision))

    trans = [to_float(row, "translation_error_m") for row in reference_rows]
    yaw = [to_float(row, "yaw_error_deg") for row in reference_rows]
    accepted_trans = [to_float(row, "refined_translation_error_m") for row in accepted_ref]
    accepted_yaw = [to_float(row, "refined_yaw_error_deg") for row in accepted_ref]
    total_ms = [to_float(row, "total_ms") for row in metrics]
    search_ms = [to_float(row, "search_ms") for row in metrics]
    refine_ms = [to_float(row, "refine_ms") for row in metrics]
    cpu_ms = [
        to_float(row, "delta_user_cpu_ms") + to_float(row, "delta_system_cpu_ms")
        for row in metrics
    ]
    rss = [to_float(row, "peak_rss_mb") for row in metrics]
    threads = [to_float(row, "thread_count") for row in metrics]

    summary = {
        "label": label,
        "frames": str(len(metrics)),
        "localized": f"{len(localized_rows)}/{len(metrics)}",
        "reference": f"{len(reference_rows)}/{len(metrics)}",
        "single_ref_success": f"{len(single_success)}/{len(reference_rows)}",
        "single_ref_fail": str(len(single_fail)),
        "temporal_accept": str(len(accepted)),
        "temporal_reject": str(len(rejected)),
        "temporal_accepted_ref_success": f"{len(accepted_ref_success)}/{len(accepted_ref)}",
        "temporal_false_accept": str(len(false_accept)),
        "temporal_recovered_from_single_fail": str(recovered_from_single_fail),
        "temporal_rejected_single_success": str(conservative_reject_success),
        "median_trans_err_m": f"{median(trans):.3f}",
        "p95_trans_err_m": f"{percentile(trans, 0.95):.3f}",
        "median_yaw_err_deg": f"{median(yaw):.3f}",
        "p95_yaw_err_deg": f"{percentile(yaw, 0.95):.3f}",
        "median_accepted_trans_err_m": f"{median(accepted_trans):.3f}",
        "p95_accepted_trans_err_m": f"{percentile(accepted_trans, 0.95):.3f}",
        "median_accepted_yaw_err_deg": f"{median(accepted_yaw):.3f}",
        "p95_accepted_yaw_err_deg": f"{percentile(accepted_yaw, 0.95):.3f}",
        "median_search_ms": f"{median(search_ms):.1f}",
        "median_refine_ms": f"{median(refine_ms):.1f}",
        "median_total_ms": f"{median(total_ms):.1f}",
        "p95_total_ms": f"{percentile(total_ms, 0.95):.1f}",
        "median_cpu_ms": f"{median(cpu_ms):.1f}",
        "p95_cpu_ms": f"{percentile(cpu_ms, 0.95):.1f}",
        "peak_rss_mb": f"{max([v for v in rss if math.isfinite(v)], default=math.nan):.1f}",
        "thread_count_min": f"{min([v for v in threads if math.isfinite(v)], default=math.nan):.0f}",
        "thread_count_max": f"{max([v for v in threads if math.isfinite(v)], default=math.nan):.0f}",
    }
    return summary, problem_rows


def problem_row(
    label: str,
    reason: str,
    metric: dict[str, str],
    decision: dict[str, str] | None,
) -> dict[str, str]:
    """把问题帧压缩成便于人工回查的 CSV 行。"""
    return {
        "label": label,
        "reason": reason,
        "stamp_sec": f"{to_float(metric, 'stamp_sec', 0.0):.6f}",
        "single_success": metric.get("success", ""),
        "single_trans_err_m": metric.get("translation_error_m", ""),
        "single_yaw_err_deg": metric.get("yaw_error_deg", ""),
        "single_rank": metric.get("refined_candidate_rank", ""),
        "single_fitness": metric.get("refine_fitness_score", ""),
        "reference_x_m": metric.get("reference_x_m", ""),
        "reference_y_m": metric.get("reference_y_m", ""),
        "reference_yaw_deg": metric.get("reference_yaw_deg", ""),
        "final_x_m": metric.get("final_x_m", ""),
        "final_y_m": metric.get("final_y_m", ""),
        "final_yaw_deg": metric.get("final_yaw_deg", ""),
        "decision": decision.get("decision", "") if decision else "",
        "decision_reason": decision.get("decision_reason", "") if decision else "",
        "selected_support_frames": decision.get("selected_support_frames", "") if decision else "",
        "best_support_frames": decision.get("best_support_frames", "") if decision else "",
        "best_seed_rank": decision.get("best_seed_rank", "") if decision else "",
        "refined_success": decision.get("refined_success", "") if decision else "",
        "refined_trans_err_m": decision.get("refined_translation_error_m", "") if decision else "",
        "refined_yaw_err_deg": decision.get("refined_yaw_error_deg", "") if decision else "",
    }


def write_dict_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str]) -> None:
    """写出字典列表 CSV，目录不存在时自动创建。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run larger nav46 stress validation for both input modes.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--template", type=Path, default=None, help="参数模板 YAML，默认使用 config/global_relocalization_eval.yaml")
    parser.add_argument("--frames", type=int, default=60, help="每路输入从 bag46 抽样多少帧")
    parser.add_argument("--stride", type=int, default=600, help="抽样帧间隔；600 约等于每 60 秒取一帧")
    parser.add_argument("--skip", type=int, default=100, help="bag 开头跳过多少帧点云")
    parser.add_argument("--run", action="store_true", help="生成 YAML 后立即运行 evaluator")
    parser.add_argument("--summarize-existing", action="store_true", help="只汇总已存在产物")
    parser.add_argument(
        "--mode",
        choices=["both", "body", "registered_world"],
        default="both",
        help="选择验证 body、registered_world 或两路都跑",
    )
    parser.add_argument("--keep-existing", action="store_true", help="运行前不删除已有 output_dir")
    parser.add_argument("--random-count", type=int, default=0, help="大于 0 时随机抽取该数量的点云帧")
    parser.add_argument("--seed", type=int, default=46, help="随机抽样种子，保证失败点可复现")
    parser.add_argument("--random-min-index", type=int, default=100, help="随机抽样最小点云帧序号，默认避开 bag 开头")
    parser.add_argument("--random-max-index", type=int, default=None, help="随机抽样最大点云帧序号，默认到该话题末尾")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    template_path = args.template or workspace / "src/humanoid_global_relocalization_runtime/config/global_relocalization_eval.yaml"
    template = load_yaml(template_path)
    modes = ["body", "registered_world"] if args.mode == "both" else [args.mode]

    output_root = workspace / ".codex_tmp/nav46_stress_validation"
    config_root = output_root / "configs"
    summaries: list[dict[str, str]] = []
    problems: list[dict[str, str]] = []

    for mode in modes:
        bag_path = Path("/home/ubuntu/nav_drift_test/nav_drift_test46")
        sample_indices: list[int] | None = None
        if args.random_count > 0:
            sample_indices = make_random_indices(
                bag_path,
                mode,
                args.random_count,
                args.seed,
                args.random_min_index,
                args.random_max_index,
            )
        sample_label = (
            f"rand{len(sample_indices)}_seed{args.seed}"
            if sample_indices
            else f"{args.frames}f_s{args.stride}_skip{args.skip}"
        )
        label = f"nav46_{mode}_{sample_label}"
        output_dir = output_root / label
        config_path = config_root / f"{label}.yaml"
        if args.run and not args.keep_existing and output_dir.exists():
            # 只清理本脚本自己的 .codex_tmp 子目录，避免旧 CSV 追加导致统计重复。
            shutil.rmtree(output_dir)

        config = build_case_config(
            template,
            workspace,
            mode,
            args.frames,
            args.stride,
            args.skip,
            output_dir,
            sample_indices,
        )
        dump_generated_yaml(
            config_path,
            config,
            f"mode={mode}, frames={params(config)['max_bag_frames']}, stride={args.stride}, skip={args.skip}, random_seed={args.seed if sample_indices else 'none'}",
        )
        print(f"[nav46_stress] generated {config_path}")

        if args.run:
            rc = run_offline_eval(workspace, config_path)
            if rc != 0:
                return rc

        if args.run or args.summarize_existing:
            summary, case_problems = summarize_case(label, output_dir)
            summaries.append(summary)
            problems.extend(case_problems)

    if args.run or args.summarize_existing:
        suffix = (
            f"rand{args.random_count}_seed{args.seed}"
            if args.random_count > 0
            else f"{args.frames}f_s{args.stride}_skip{args.skip}"
        )
        summary_path = output_root / f"summary_{suffix}.csv"
        problem_path = output_root / f"problem_frames_{suffix}.csv"
        write_dict_csv(summary_path, summaries, list(summaries[0].keys()) if summaries else ["label"])
        problem_fields = [
            "label",
            "reason",
            "stamp_sec",
            "single_success",
            "single_trans_err_m",
            "single_yaw_err_deg",
            "single_rank",
            "single_fitness",
            "reference_x_m",
            "reference_y_m",
            "reference_yaw_deg",
            "final_x_m",
            "final_y_m",
            "final_yaw_deg",
            "decision",
            "decision_reason",
            "selected_support_frames",
            "best_support_frames",
            "best_seed_rank",
            "refined_success",
            "refined_trans_err_m",
            "refined_yaw_err_deg",
        ]
        write_dict_csv(problem_path, problems, problem_fields)
        print(f"[nav46_stress] wrote {summary_path}")
        print(f"[nav46_stress] wrote {problem_path}")
        for row in summaries:
            print(
                "[nav46_stress] "
                f"{row['label']} single={row['single_ref_success']} "
                f"accept={row['temporal_accept']} reject={row['temporal_reject']} "
                f"accepted_success={row['temporal_accepted_ref_success']} "
                f"false_accept={row['temporal_false_accept']} "
                f"median_total_ms={row['median_total_ms']} "
                f"median_cpu_ms={row['median_cpu_ms']} "
                f"peak_rss_mb={row['peak_rss_mb']}"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
