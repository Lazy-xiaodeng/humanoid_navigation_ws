#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_online_trajectory_recovery_smoke.py

文件作用：
  1. 这是在线 trajectory recovery 专项冒烟测试脚本，不属于功能源码。
  2. 基于已有 synthetic_body_online_debug.yaml 生成临时 YAML，把第一层 online refine fitness 阈值压到 0。
  3. 第一层 selected support 会因为 fitness 阈值被拒绝，随后必须由 trajectory_single_agreement fallback 发布 verified。
  4. 该脚本用于证明新增在线 trajectory recovery 分支真的被执行，而不是只验证普通 state=verified。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Any

import yaml


def repo_root_from_script() -> Path:
    """根据脚本路径反推出 humanoid_ws 工作空间根目录。"""
    return Path(__file__).resolve().parents[3]


def params(config: dict[str, Any]) -> dict[str, Any]:
    """取得 ROS 参数字典。"""
    return config["global_relocalization_eval"]["ros__parameters"]


def write_forced_config(template_path: Path, output_path: Path) -> None:
    """写强制 trajectory recovery 的临时 YAML。"""
    config = yaml.safe_load(template_path.read_text(encoding="utf-8"))
    p = params(config)

    # 把第一层 selected refine 的 online fitness 阈值压到 0，让普通 verified 分支必然拒绝。
    # trajectory_single_agreement_max_fitness 保持 0.04，因此同一个低 fitness 单帧结果仍可作为 fallback 证据。
    p["temporal_consistency_online_max_refine_fitness"] = 0.0
    p["trajectory_single_agreement_fallback_enable"] = True
    p["trajectory_single_agreement_max_fitness"] = 0.04
    p["trajectory_single_agreement_max_xy_m"] = 1.0
    p["trajectory_single_agreement_max_yaw_deg"] = 3.0
    p["trajectory_single_agreement_min_overlap"] = 0.80
    p["trajectory_single_agreement_min_margin"] = 0.005
    p["output_dir"] = str(output_path.parent / "global_relocalization_online_trajectory_recovery_smoke")

    header = [
        "# online trajectory recovery smoke 自动生成配置。",
        "#",
        "# 作用：",
        "#   - 由 test/run_online_trajectory_recovery_smoke.py 生成。",
        "#   - 强制普通 online verified 分支因 fitness 阈值失败，验证 trajectory_single_agreement fallback 是否能发布 debug 恢复量。",
        "#   - 该文件位于 .codex_tmp，不作为线上运行配置。",
        "",
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        "\n".join(header) + yaml.safe_dump(config, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Run online trajectory recovery smoke test.")
    parser.add_argument("--workspace", type=Path, default=repo_root_from_script(), help="humanoid_ws 工作空间")
    parser.add_argument(
        "--template",
        type=Path,
        default=Path("src/humanoid_global_relocalization_runtime/test/synthetic_body_online_debug.yaml"),
        help="在线 smoke YAML 模板",
    )
    parser.add_argument("--bag", type=Path, default=Path(".codex_tmp/synthetic_body_nav_drift_test43"), help="播放 bag")
    parser.add_argument("--timeout", type=float, default=180.0, help="等待 verified 的超时时间")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    template = (workspace / args.template).resolve() if not args.template.is_absolute() else args.template
    bag = (workspace / args.bag).resolve() if not args.bag.is_absolute() else args.bag
    config = workspace / ".codex_tmp/online_trajectory_recovery_smoke.yaml"
    pose_output = workspace / ".codex_tmp/online_trajectory_recovery_smoke_pose.csv"
    write_forced_config(template, config)

    command = [
        sys.executable,
        str(workspace / "src/humanoid_global_relocalization_runtime/test/run_online_debug_smoke.py"),
        "--config",
        str(config),
        "--bag",
        str(bag),
        "--timeout",
        str(args.timeout),
        "--expected-state",
        "verified_trajectory_single_agreement",
        "--pose-output",
        str(pose_output),
    ]
    print("[online_trajectory_recovery_smoke] running", " ".join(command), flush=True)
    result = subprocess.run(command, cwd=str(workspace), text=True)
    if result.returncode != 0:
        return result.returncode
    print(f"[online_trajectory_recovery_smoke] PASS pose_output={pose_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
