#!/usr/bin/env python3
"""按“候选 -> RO -> Bridge -> 稳定 TF”合同统计最终成功率。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any


TRUE_VALUES = {"1", "true", "yes", "y", "accepted", "success"}


def as_bool(value: Any) -> bool:
    return str(value).strip().lower() in TRUE_VALUES


def as_float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def as_int(value: Any) -> int:
    number = as_float(value)
    return int(number) if math.isfinite(number) else 0


def read_rows(path: Path) -> list[dict[str, Any]]:
    if path.suffix.lower() == ".json":
        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            data = data.get("cases", data.get("rows", []))
        if not isinstance(data, list):
            raise ValueError(f"{path}: JSON 顶层必须是列表，或包含 cases/rows 列表")
        return [dict(row) for row in data]
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finite(values: list[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def percentile(values: list[float], ratio: float) -> float:
    ordered = sorted(finite(values))
    if not ordered:
        return math.nan
    index = min(len(ordered) - 1, max(0, math.ceil(ratio * len(ordered)) - 1))
    return ordered[index]


def metric(values: list[float], op: str) -> float | None:
    values = finite(values)
    if not values:
        return None
    if op == "mean":
        return statistics.fmean(values)
    if op == "max":
        return max(values)
    return percentile(values, 0.5)


def summarize(
    rows: list[dict[str, Any]],
    max_error_m: float,
    stable_tf_frames: int,
) -> dict[str, dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row.get("mode", "unknown")).strip() or "unknown"].append(row)

    result: dict[str, dict[str, Any]] = {}
    for mode, cases in sorted(grouped.items()):
        chain_complete: list[dict[str, Any]] = []
        for row in cases:
            complete = (
                as_bool(row.get("global_candidate_published"))
                and as_bool(row.get("ro_trusted_commit"))
                and as_bool(row.get("bridge_accepted"))
                and as_int(row.get("tf_stable_frames")) >= stable_tf_frames
            )
            row["_chain_complete"] = complete
            if complete:
                chain_complete.append(row)

        errors = [as_float(row.get("translation_error_m")) for row in chain_complete]
        accepted_errors = finite(errors)
        success = [
            row
            for row in chain_complete
            if math.isfinite(as_float(row.get("translation_error_m")))
            and as_float(row.get("translation_error_m")) <= max_error_m
        ]
        false_accept = [
            row
            for row in cases
            if as_bool(row.get("bridge_accepted"))
            and (
                not math.isfinite(as_float(row.get("translation_error_m")))
                or as_float(row.get("translation_error_m")) > max_error_m
            )
        ]
        elapsed = [as_float(row.get("elapsed_sec")) for row in success]
        cpu = [as_float(row.get("cpu_percent")) for row in success]
        rss = [as_float(row.get("peak_rss_mb")) for row in success]
        threshold_counts = {
            f"success_le_{int(limit * 100):02d}cm": sum(error <= limit for error in accepted_errors)
            for limit in (0.05, 0.10, 0.20)
        }
        result[mode] = {
            "cases": len(cases),
            "chain_complete": len(chain_complete),
            "success": len(success),
            "success_rate": len(success) / len(cases) if cases else 0.0,
            "false_accept": len(false_accept),
            **threshold_counts,
            "translation_error_min_m": min(accepted_errors) if accepted_errors else None,
            "translation_error_median_m": metric(errors, "median"),
            "translation_error_max_m": max(accepted_errors) if accepted_errors else None,
            "elapsed_median_sec": metric(elapsed, "median"),
            "elapsed_p95_sec": percentile(elapsed, 0.95) if finite(elapsed) else None,
            "cpu_mean_percent": metric(cpu, "mean"),
            "peak_rss_max_mb": metric(rss, "max"),
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", action="append", type=Path, required=True, help="CSV/JSON，可重复传入")
    parser.add_argument("--output", type=Path, required=True, help="汇总 JSON")
    parser.add_argument("--max-error", type=float, default=0.20, help="最终成功的平移误差上限")
    parser.add_argument("--stable-tf-frames", type=int, default=20, help="Bridge 发布后要求的连续稳定 TF 帧数")
    args = parser.parse_args()

    rows: list[dict[str, Any]] = []
    for path in args.input:
        rows.extend(read_rows(path.resolve()))
    summary = summarize(rows, args.max_error, args.stable_tf_frames)
    payload = {
        "contract": {
            "max_translation_error_m": args.max_error,
            "required_stable_tf_frames": args.stable_tf_frames,
            "success_definition": "candidate + RO trusted commit + bridge accepted + stable TF + error threshold",
        },
        "summary": summary,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for mode, stats in summary.items():
        print(
            f"[{mode}] success={stats['success']}/{stats['cases']} "
            f"rate={stats['success_rate']:.2%} chain={stats['chain_complete']} "
            f"false_accept={stats['false_accept']} "
            f"error={stats['translation_error_min_m']}/"
            f"{stats['translation_error_median_m']}/{stats['translation_error_max_m']}m"
        )
    print(f"[summary] {args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
