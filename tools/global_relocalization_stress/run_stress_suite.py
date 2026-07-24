#!/usr/bin/env python3
"""统一运行全局重定位随机点、导航点、集成合同和最终链路统计。"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import yaml


HERE = Path(__file__).resolve().parent


def find_workspace(start: Path) -> Path:
    for path in (start, *start.parents):
        if (path / "src/humanoid_global_relocalization_runtime").is_dir():
            return path
    raise RuntimeError("无法自动找到 humanoid_ws；请在 YAML 中设置 workspace")


def resolve_path(value: str | Path, base: Path) -> Path:
    expanded = Path(os.path.expandvars(os.path.expanduser(str(value))))
    return expanded.resolve() if expanded.is_absolute() else (base / expanded).resolve()


def load_config(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("version") != 1:
        raise ValueError("配置必须是 version: 1 的 YAML 字典")
    return data


class Runner:
    def __init__(self, workspace: Path, output_root: Path, setup: str, dry_run: bool) -> None:
        self.workspace = workspace
        self.output_root = output_root
        self.setup = setup.strip()
        self.dry_run = dry_run
        self.records: list[dict[str, Any]] = []

    def run(self, label: str, command: list[str]) -> None:
        log_path = self.output_root / "logs" / f"{label}.log"
        display = shlex.join(command)
        shell_command = f"{self.setup} && {display}" if self.setup else display
        print(f"[run] {label}: {shell_command}")
        started = time.time()
        rc = 0
        if not self.dry_run:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with log_path.open("w", encoding="utf-8") as log:
                result = subprocess.run(
                    ["bash", "-lc", shell_command],
                    cwd=self.workspace,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
            rc = result.returncode
        self.records.append(
            {
                "label": label,
                "command": shell_command,
                "returncode": rc,
                "elapsed_sec": round(time.time() - started, 3),
                "log": str(log_path),
            }
        )
        self.write_manifest()
        if rc != 0:
            raise RuntimeError(f"{label} 失败，返回码 {rc}，日志：{log_path}")

    def write_manifest(self) -> None:
        if self.dry_run:
            return
        self.output_root.mkdir(parents=True, exist_ok=True)
        path = self.output_root / "run_manifest.json"
        path.write_text(json.dumps(self.records, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def require_paths(config: dict[str, Any], config_dir: Path, workspace: Path) -> list[str]:
    problems: list[str] = []
    for section_name in ("random_stress", "waypoint_stress"):
        section = config.get(section_name, {})
        if not section.get("enabled", False):
            continue
        for bag in section.get("bags", []):
            path = resolve_path(bag["path"], config_dir)
            if not path.exists():
                problems.append(f"{section_name}.{bag.get('name', '?')}: bag 不存在：{path}")
    waypoint = config.get("waypoint_stress", {})
    if waypoint.get("enabled", False):
        path = resolve_path(waypoint["waypoints"], workspace)
        if not path.exists():
            problems.append(f"导航点文件不存在：{path}")
    return problems


def add_random_jobs(
    runner: Runner,
    config: dict[str, Any],
    config_dir: Path,
) -> None:
    section = config.get("random_stress", {})
    if not section.get("enabled", False):
        return
    script = runner.workspace / "src/humanoid_global_relocalization_runtime/test/run_nav46_stress_validation.py"
    for bag in section.get("bags", []):
        bag_name = str(bag["name"])
        bag_path = resolve_path(bag["path"], config_dir)
        for seed in section.get("seeds", [46]):
            label = f"random_{bag_name}_seed{seed}"
            output = runner.output_root / "random" / bag_name / f"seed_{seed}"
            command = [
                sys.executable,
                str(script),
                "--workspace",
                str(runner.workspace),
                "--bag",
                str(bag_path),
                "--output-root",
                str(output),
                "--random-count",
                str(section.get("count_per_round", 100)),
                "--seed",
                str(seed),
                "--mode",
                str(section.get("mode", "body")),
                "--run",
            ]
            runner.run(label, command)


def add_waypoint_jobs(
    runner: Runner,
    config: dict[str, Any],
    config_dir: Path,
) -> None:
    section = config.get("waypoint_stress", {})
    if not section.get("enabled", False):
        return
    script = runner.workspace / "src/humanoid_global_relocalization_runtime/test/run_waypoint_pose_validation.py"
    waypoint_path = resolve_path(section["waypoints"], runner.workspace)
    for bag in section.get("bags", []):
        bag_name = str(bag["name"])
        command = [
            sys.executable,
            str(script),
            "--workspace",
            str(runner.workspace),
            "--waypoints",
            str(waypoint_path),
            "--bag",
            str(resolve_path(bag["path"], config_dir)),
            "--output-root",
            str(runner.output_root / "waypoints" / bag_name),
            "--max-distance",
            str(section.get("max_distance_m", 1.0)),
            "--match-cloud-topic",
            str(bag.get("cloud_topic", "/cloud_registered_body")),
            "--mode",
            str(section.get("mode", "body")),
            "--run",
        ]
        if section.get("allow_duplicate_cloud_indices", True):
            command.append("--allow-duplicate-cloud-indices")
        runner.run(f"waypoints_{bag_name}", command)


def add_smoke_jobs(runner: Runner, config: dict[str, Any]) -> None:
    section = config.get("integration_smoke", {})
    if not section.get("enabled", False):
        return
    for item in section.get("scripts", []):
        script = resolve_path(item, runner.workspace)
        runner.run(f"smoke_{script.stem}", [sys.executable, str(script)])


def add_gate_job(runner: Runner, config: dict[str, Any]) -> None:
    section = config.get("validation_gates", {})
    if not section.get("enabled", False):
        return
    script = runner.workspace / "src/humanoid_global_relocalization_runtime/test/run_validation_gates.py"
    artifacts = resolve_path(section.get("artifacts_root", ".codex_tmp"), runner.workspace)
    runner.run(
        "validation_gates",
        [
            sys.executable,
            str(script),
            "--workspace",
            str(runner.workspace),
            "--artifacts-root",
            str(artifacts),
        ],
    )


def add_final_chain_job(
    runner: Runner,
    config: dict[str, Any],
    config_dir: Path,
) -> None:
    section = config.get("final_chain", {})
    if not section.get("enabled", False):
        return
    command = [
        sys.executable,
        str(HERE / "summarize_final_chain.py"),
        "--output",
        str(runner.output_root / "final_chain_summary.json"),
        "--max-error",
        str(section.get("max_translation_error_m", 0.20)),
        "--stable-tf-frames",
        str(section.get("stable_tf_frames", 20)),
    ]
    for item in section.get("inputs", []):
        command.extend(["--input", str(resolve_path(item, config_dir))])
    runner.run("final_chain_summary", command)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=HERE / "stress_suite.yaml")
    parser.add_argument("--check-only", action="store_true", help="只检查环境与输入路径")
    parser.add_argument("--dry-run", action="store_true", help="打印命令但不执行")
    args = parser.parse_args()

    config_path = args.config.resolve()
    config = load_config(config_path)
    workspace_value = str(config.get("workspace", "")).strip()
    workspace = (
        resolve_path(workspace_value, config_path.parent)
        if workspace_value
        else find_workspace(HERE)
    )
    output_root = resolve_path(config.get("output_root", "output"), config_path.parent)

    if shutil.which("bash") is None:
        print("[error] 找不到 bash", file=sys.stderr)
        return 2
    problems = require_paths(config, config_path.parent, workspace)
    if problems:
        for problem in problems:
            print(f"[error] {problem}", file=sys.stderr)
        return 2
    print(f"[workspace] {workspace}")
    print(f"[output] {output_root}")
    print("[check] 输入路径检查通过")
    if args.check_only:
        return 0

    runner = Runner(workspace, output_root, str(config.get("setup_command", "")), args.dry_run)
    try:
        add_random_jobs(runner, config, config_path.parent)
        add_waypoint_jobs(runner, config, config_path.parent)
        add_smoke_jobs(runner, config)
        add_gate_job(runner, config)
        add_final_chain_job(runner, config, config_path.parent)
    except RuntimeError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1
    print(f"[done] jobs={len(runner.records)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
