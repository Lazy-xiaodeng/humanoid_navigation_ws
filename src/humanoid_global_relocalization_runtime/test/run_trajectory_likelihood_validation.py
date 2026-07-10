#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_trajectory_likelihood_validation.py

文件作用：
  1. 专门验证“3D-BBS top-K 候选池 + 历史 odom 轨迹 + 多帧 scan-map likelihood 排序”。
  2. 从点位匹配 CSV 读取目标 cloud index，为每个目标额外加入历史或主动恢复后的新视角 cloud index。
  3. 生成 evaluator YAML 并调用 C++ offline evaluator，产出 trajectory likelihood CSV。
  4. 汇总目标帧上轨迹验证选择的候选是否正确，以及相对单帧 GICP 是否有改善。
"""

from __future__ import annotations

import argparse
import csv
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class Target:
    """一个需要验证历史轨迹 likelihood 的目标点位。"""

    waypoint_id: str
    name: str
    cloud_index: int


def repo_root_from_script() -> Path:
    """根据脚本路径反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def params(config: dict[str, Any]) -> dict[str, Any]:
    """取得 ROS 参数字典。"""
    return config["global_relocalization_eval"]["ros__parameters"]


def read_targets(matches_path: Path, ids: set[str]) -> list[Target]:
    """从 waypoint pose validation 的 matches CSV 中读取目标帧。"""
    targets: list[Target] = []
    with matches_path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if not ids or row["waypoint_id"] in ids:
                targets.append(Target(row["waypoint_id"], row["name"], int(row["cloud_index"])))
    return targets


def build_sample_indices(targets: list[Target], relative_offsets: list[int]) -> tuple[list[int], dict[int, Target]]:
    """构造显式抽帧序号，并记录哪些 index 是真正目标帧。

    offset 采用“目标帧减 offset”的约定：
      - 正数表示目标之前的历史帧，例如 40 表示 target_index - 40；
      - 0 表示目标帧本身；
      - 负数表示目标之后的新视角帧，例如 -40 表示 target_index + 40。
    """
    target_by_index = {target.cloud_index: target for target in targets}
    indices: set[int] = set()
    for target in targets:
        for offset in relative_offsets:
            index = target.cloud_index - offset
            if index >= 0:
                indices.add(index)
        indices.add(target.cloud_index)
    return sorted(indices), target_by_index


def build_eval_config(
    template: dict[str, Any],
    mode: str,
    bag_path: Path,
    output_dir: Path,
    sample_indices: list[int],
    target_indices: list[int],
    history_count: int,
    future_count: int,
    args: argparse.Namespace,
) -> dict[str, Any]:
    """基于主 YAML 生成轨迹 likelihood 专项配置。"""
    config = yaml.safe_load(yaml.safe_dump(template, allow_unicode=True, sort_keys=False))
    p = params(config)
    p["input_mode"] = mode
    p["bag_paths"] = [str(bag_path)]
    p["bag_sample_frame_indices"] = sample_indices
    p["max_bag_frames"] = len(sample_indices)
    p["bag_start_frame_skip"] = 0
    p["bag_frame_stride"] = 1
    p["odom_time_tolerance_sec"] = 0.10
    p["use_bag_reference_pose"] = True
    p["reference_time_tolerance_sec"] = 0.05
    p["save_aligned_cloud"] = False
    p["output_dir"] = str(output_dir)
    p["scan_leaf_size"] = args.scan_leaf_size
    p["bbs_min_level_res"] = args.bbs_min_level_res
    p["bbs_num_threads"] = args.bbs_num_threads
    p["top_k"] = args.top_k
    p["max_refine_candidates"] = args.max_refine_candidates
    p["refine_method"] = "gicp"
    p["refine_methods_for_sweep"] = ["gicp"]
    p["simulated_relocalization_cases"] = [
        {
            "name": "arbitrary_start_no_prior",
            "offset_xyzrpy": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    ]
    p["enable_temporal_consistency"] = True
    p["enable_trajectory_likelihood"] = True
    p["temporal_consistency_window_before"] = history_count
    p["temporal_consistency_window_after"] = future_count
    p["temporal_consistency_online_min_support_frames"] = min(2, max(1, history_count + future_count + 1))
    p["temporal_consistency_online_max_refine_fitness"] = 0.12
    p["trajectory_likelihood_max_candidates"] = args.trajectory_max_candidates
    p["trajectory_likelihood_voxel_size"] = args.trajectory_voxel_size
    p["trajectory_likelihood_neighbor_radius"] = args.trajectory_neighbor_radius
    p["trajectory_likelihood_min_overlap_ratio"] = args.trajectory_min_overlap_ratio
    p["trajectory_likelihood_min_average_overlap"] = args.trajectory_min_average_overlap
    p["trajectory_likelihood_min_margin"] = args.trajectory_min_margin
    p["trajectory_likelihood_center_frame_indices"] = target_indices
    p["trajectory_refine_enable"] = args.trajectory_refine_enable
    p["trajectory_refine_top_n"] = args.trajectory_refine_top_n
    p["trajectory_refine_max_fitness"] = args.trajectory_refine_max_fitness
    p["trajectory_refine_min_fitness_margin"] = args.trajectory_refine_min_fitness_margin
    return config


def write_config(path: Path, config: dict[str, Any]) -> None:
    """写自动生成 YAML。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "# trajectory likelihood validation 自动生成配置。",
        "#",
        "# 作用：",
        "#   - 由 test/run_trajectory_likelihood_validation.py 生成。",
        "#   - 用目标帧前的历史帧，或目标帧后的主动恢复新视角，验证 top-K 候选能否被 scan-map likelihood 正确重排。",
        "#   - 该文件位于 .codex_tmp，不作为线上运行配置。",
        "",
    ]
    path.write_text("\n".join(header) + yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")


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


def read_csv(path: Path) -> list[dict[str, str]]:
    """读取 CSV，文件不存在时返回空列表。"""
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def to_float(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """安全读取浮点字段。"""
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def key_from_stamp(stamp: float) -> str:
    """统一 stamp key 精度。"""
    return f"{stamp:.6f}"


def summarize(
    output_dir: Path,
    sample_indices: list[int],
    target_by_index: dict[int, Target],
    success_translation_thresh: float,
    success_yaw_thresh_deg: float,
) -> list[dict[str, str]]:
    """汇总目标帧的单帧结果与 trajectory likelihood 选择结果。"""
    metrics = [
        row for row in read_csv(output_dir / "global_relocalization_metrics.csv")
        if row.get("scenario_name") == "arbitrary_start_no_prior"
    ]
    target_stamp_to_index: dict[str, int] = {}
    target_metric_by_stamp: dict[str, dict[str, str]] = {}
    for row_index, row in enumerate(metrics):
        if row_index >= len(sample_indices):
            break
        sample_index = sample_indices[row_index]
        if sample_index not in target_by_index:
            continue
        stamp_key = key_from_stamp(to_float(row, "stamp_sec"))
        target_stamp_to_index[stamp_key] = sample_index
        target_metric_by_stamp[stamp_key] = row

    trajectory_rows = read_csv(output_dir / "global_relocalization_trajectory_likelihood.csv")
    rows_by_stamp: dict[str, list[dict[str, str]]] = {}
    for row in trajectory_rows:
      stamp_key = key_from_stamp(to_float(row, "stamp_sec"))
      if stamp_key in target_stamp_to_index:
          rows_by_stamp.setdefault(stamp_key, []).append(row)

    selected_rows: list[dict[str, str]] = []
    for stamp_key, rows in rows_by_stamp.items():
        accepted = [row for row in rows if row.get("trajectory_decision") == "accept"]
        if accepted:
            selected_rows.append(accepted[0])
            continue
        trajectory_best = [row for row in rows if row.get("selected_by_trajectory") == "1"]
        if trajectory_best:
            selected_rows.append(trajectory_best[0])

    summaries: list[dict[str, str]] = []
    for row in selected_rows:
        stamp_key = key_from_stamp(to_float(row, "stamp_sec"))
        sample_index = target_stamp_to_index[stamp_key]
        target = target_by_index[sample_index]
        metric = target_metric_by_stamp[stamp_key]
        has_refined_pose = row.get("selected_by_refine") == "1" and row.get("refined_converged") == "1"
        traj_trans = to_float(
            row,
            "refined_translation_error_m" if has_refined_pose else "candidate_translation_error_m",
        )
        traj_yaw = to_float(
            row,
            "refined_yaw_error_deg" if has_refined_pose else "candidate_yaw_error_deg",
        )
        traj_success = traj_trans <= success_translation_thresh and traj_yaw <= success_yaw_thresh_deg
        summaries.append(
            {
                "waypoint_id": target.waypoint_id,
                "name": target.name,
                "target_index": str(sample_index),
                "stamp_sec": stamp_key,
                "single_success": metric.get("success", ""),
                "single_error_m": metric.get("translation_error_m", ""),
                "single_yaw_deg": metric.get("yaw_error_deg", ""),
                "single_rank": metric.get("refined_candidate_rank", ""),
                "trajectory_success": "1" if traj_success else "0",
                "trajectory_rank": row.get("candidate_rank", ""),
                "trajectory_error_m": row.get("candidate_translation_error_m", ""),
                "trajectory_yaw_deg": row.get("candidate_yaw_error_deg", ""),
                "refined_selected": row.get("selected_by_refine", ""),
                "refined_converged": row.get("refined_converged", ""),
                "refined_fitness": row.get("refined_fitness", ""),
                "refined_fitness_margin": row.get("refined_fitness_margin", ""),
                "refined_error_m": row.get("refined_translation_error_m", ""),
                "refined_yaw_deg": row.get("refined_yaw_error_deg", ""),
                "support_frames": row.get("support_frames", ""),
                "window_frames": row.get("window_frames", ""),
                "average_overlap": row.get("average_overlap", ""),
                "margin": row.get("margin", ""),
                "trajectory_decision": row.get("trajectory_decision", ""),
            }
        )
    return summaries


def write_summary(path: Path, rows: list[dict[str, str]]) -> None:
    """写汇总 CSV。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "mode",
        "waypoint_id",
        "name",
        "target_index",
        "stamp_sec",
        "single_success",
        "single_error_m",
        "single_yaw_deg",
        "single_rank",
        "trajectory_success",
        "trajectory_rank",
        "trajectory_error_m",
        "trajectory_yaw_deg",
        "refined_selected",
        "refined_converged",
        "refined_fitness",
        "refined_fitness_margin",
        "refined_error_m",
        "refined_yaw_deg",
        "support_frames",
        "window_frames",
        "average_overlap",
        "margin",
        "trajectory_decision",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_mode_stats(path: Path, rows: list[dict[str, str]]) -> None:
    """按输入模式汇总单帧/轨迹验证成功、接受和误接受数量。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    modes = sorted({row["mode"] for row in rows})
    fieldnames = [
        "mode",
        "targets",
        "single_success",
        "trajectory_success",
        "trajectory_accept",
        "trajectory_accept_success",
        "trajectory_false_accept",
        "trajectory_reject_success",
        "median_trajectory_error_m",
        "max_trajectory_error_m",
        "median_margin",
        "min_margin",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for mode in modes:
            mode_rows = [row for row in rows if row["mode"] == mode]
            errors = sorted(to_float(row, "trajectory_error_m") for row in mode_rows if row["trajectory_success"] == "1")
            margins = sorted(to_float(row, "margin") for row in mode_rows)
            accepted = [row for row in mode_rows if row["trajectory_decision"] == "accept"]
            accepted_success = [row for row in accepted if row["trajectory_success"] == "1"]
            rejected_success = [
                row for row in mode_rows
                if row["trajectory_decision"] != "accept" and row["trajectory_success"] == "1"
            ]
            writer.writerow(
                {
                    "mode": mode,
                    "targets": str(len(mode_rows)),
                    "single_success": str(sum(1 for row in mode_rows if row["single_success"] == "1")),
                    "trajectory_success": str(sum(1 for row in mode_rows if row["trajectory_success"] == "1")),
                    "trajectory_accept": str(len(accepted)),
                    "trajectory_accept_success": str(len(accepted_success)),
                    "trajectory_false_accept": str(len(accepted) - len(accepted_success)),
                    "trajectory_reject_success": str(len(rejected_success)),
                    "median_trajectory_error_m": f"{errors[len(errors)//2]:.6f}" if errors else "nan",
                    "max_trajectory_error_m": f"{max(errors):.6f}" if errors else "nan",
                    "median_margin": f"{margins[len(margins)//2]:.6f}" if margins else "nan",
                    "min_margin": f"{min(margins):.6f}" if margins else "nan",
                }
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate trajectory likelihood relocalization on waypoint targets.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument("--bag", type=Path, default=Path("/home/ubuntu/nav_drift_test/nav_drift_test46"), help="验证 bag")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(".codex_tmp/trajectory_likelihood_validation"),
        help="输出目录；批量验证多个 bag 时建议为每个 bag 单独指定",
    )
    parser.add_argument(
        "--matches",
        type=Path,
        default=Path(".codex_tmp/waypoint_pose_validation/dynamic_waypoints_matches.csv"),
        help="点位匹配 CSV",
    )
    parser.add_argument("--ids", nargs="*", default=[], help="要验证的 waypoint_id；不传表示 matches CSV 中全部已匹配点位")
    parser.add_argument("--modes", nargs="*", default=["registered_world"], help="输入模式，可选 registered_world/body")
    parser.add_argument(
        "--history-offsets",
        nargs="*",
        type=int,
        default=[240, 200, 160, 120, 80, 40, 0],
        help="相对目标帧的 cloud index 偏移；正数表示历史帧，0 表示目标帧，负数表示目标之后的新视角帧",
    )
    parser.add_argument("--scan-leaf-size", type=float, default=0.30, help="scan 降采样体素大小")
    parser.add_argument("--bbs-min-level-res", type=float, default=0.75, help="3D-BBS 最底层分辨率")
    parser.add_argument("--bbs-num-threads", type=int, default=2, help="3D-BBS 搜索线程数")
    parser.add_argument("--top-k", type=int, default=60, help="3D-BBS 保留候选数")
    parser.add_argument("--max-refine-candidates", type=int, default=8, help="单帧 GICP refine 候选数")
    parser.add_argument("--trajectory-max-candidates", type=int, default=60, help="轨迹 likelihood 评估候选数")
    parser.add_argument("--trajectory-voxel-size", type=float, default=0.35, help="轨迹 likelihood 占据体素大小")
    parser.add_argument("--trajectory-neighbor-radius", type=int, default=1, help="轨迹 likelihood 占据邻域半径")
    parser.add_argument("--trajectory-min-overlap-ratio", type=float, default=0.18, help="轨迹 likelihood 单帧支持 overlap 阈值")
    parser.add_argument("--trajectory-min-average-overlap", type=float, default=0.95, help="轨迹 likelihood 历史窗口平均 overlap 接受阈值")
    parser.add_argument("--trajectory-min-margin", type=float, default=0.0015, help="轨迹 likelihood 第一名和第二个不同位姿簇最小 overlap 差值")
    parser.add_argument("--trajectory-refine-enable", action=argparse.BooleanOptionalAction, default=False, help="是否对轨迹 top-N 候选做二次精配准")
    parser.add_argument("--trajectory-refine-top-n", type=int, default=5, help="轨迹候选二次精配准数量")
    parser.add_argument("--trajectory-refine-max-fitness", type=float, default=0.12, help="轨迹二次精配准最大允许 fitness")
    parser.add_argument("--trajectory-refine-min-fitness-margin", type=float, default=0.005, help="轨迹二次精配准最佳和第二名最小 fitness 差值")
    parser.add_argument("--run", action="store_true", help="生成 YAML 后立即运行 evaluator")
    parser.add_argument("--keep-existing", action="store_true", help="保留旧输出目录")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    matches_path = (workspace / args.matches).resolve() if not args.matches.is_absolute() else args.matches
    output_root = (workspace / args.output_root).resolve() if not args.output_root.is_absolute() else args.output_root
    if args.run and output_root.exists() and not args.keep_existing:
        shutil.rmtree(output_root)

    targets = read_targets(matches_path, set(args.ids))
    if not targets:
        print("[trajectory_validation] no target matched from matches csv", file=sys.stderr)
        return 2

    sample_indices, target_by_index = build_sample_indices(targets, args.history_offsets)
    template = yaml.safe_load((workspace / "src/humanoid_global_relocalization_runtime/config/relocalization_validation.yaml").read_text(encoding="utf-8"))
    history_count = max(0, len([offset for offset in args.history_offsets if offset > 0]))
    future_count = max(0, len([offset for offset in args.history_offsets if offset < 0]))

    all_rows: list[dict[str, str]] = []
    for mode in args.modes:
        mode_output = output_root / mode
        config = build_eval_config(
            template,
            mode,
            args.bag.resolve(),
            mode_output.resolve(),
            sample_indices,
            sorted(target_by_index.keys()),
            history_count,
            future_count,
            args,
        )
        config_path = output_root / "configs" / f"{mode}.yaml"
        write_config(config_path, config)
        print(
            f"[trajectory_validation] mode={mode} targets={len(targets)} samples={len(sample_indices)} "
            f"config={config_path}"
        )
        if args.run:
            rc = run_offline_eval(workspace, config_path)
            if rc != 0:
                return rc
            rows = summarize(
                mode_output,
                sample_indices,
                target_by_index,
                float(params(config)["success_translation_thresh"]),
                float(params(config)["success_yaw_thresh_deg"]),
            )
            for row in rows:
                row["mode"] = mode
            all_rows.extend(rows)

    if all_rows:
        summary_path = output_root / "summary.csv"
        stats_path = output_root / "stats.csv"
        write_summary(summary_path, all_rows)
        write_mode_stats(stats_path, all_rows)
        print(f"[trajectory_validation] wrote {summary_path}")
        print(f"[trajectory_validation] wrote {stats_path}")
        for row in all_rows:
            print(
                "[trajectory_validation] "
                f"mode={row['mode']} wp={row['waypoint_id']} "
                f"single={row['single_success']} err={row['single_error_m']}m/{row['single_yaw_deg']}deg "
                f"trajectory={row['trajectory_success']} rank={row['trajectory_rank']} "
                f"err={row['trajectory_error_m']}m/{row['trajectory_yaw_deg']}deg "
                f"refined={row['refined_selected']} fitness={row['refined_fitness']} "
                f"refined_err={row['refined_error_m']}m/{row['refined_yaw_deg']}deg "
                f"overlap={row['average_overlap']} margin={row['margin']} decision={row['trajectory_decision']}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
