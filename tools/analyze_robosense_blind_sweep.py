#!/usr/bin/env python3
import csv
import json
import math
import re
import statistics
import sys
from pathlib import Path


def percentile(values, pct):
    if not values:
        return None
    values = sorted(values)
    k = (len(values) - 1) * pct / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return values[int(k)]
    return values[f] * (c - k) + values[c] * (k - f)


def stats(values):
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "p50": percentile(values, 50),
        "p95": percentile(values, 95),
        "max": max(values),
    }


def parse_launch_log(path):
    text = path.read_text(errors="replace")
    cloud = []
    semantic = []
    valid_pairs = []
    valid_pair_ratios = []
    match_scores = []
    prior_delta_xy = []
    prior_delta_yaw = []
    no_source_sizes = []
    no_pair_sizes = []
    large_delta_xy = []
    large_delta_yaw = []
    too_large_dx = []
    too_large_yaw = []

    for match in re.finditer(
        r"cloud_size raw_base=(\d+) after_blind=(\d+) blind_distance=([-0-9.]+)", text
    ):
        cloud.append((int(match.group(1)), int(match.group(2))))
    for match in re.finditer(
        r"semantic_cloud size:\s*(\d+)\s+raw_base=(\d+)\s+after_blind=(\d+)", text
    ):
        semantic.append((int(match.group(1)), int(match.group(2)), int(match.group(3))))
    for match in re.finditer(r"valid_pair_num:\s*(\d+)", text):
        valid_pairs.append(int(match.group(1)))
    for match in re.finditer(r"valid_pair_ratio:\s*([-0-9.]+)", text):
        valid_pair_ratios.append(float(match.group(1)))
    for match in re.finditer(
        r"match_score:\s*([-0-9.]+)\s+prior_delta_xy:\s*([-0-9.]+)\s+"
        r"prior_delta_z:\s*[-0-9.]+\s+prior_delta_yaw:\s*([-0-9.]+)",
        text,
    ):
        match_scores.append(float(match.group(1)))
        prior_delta_xy.append(float(match.group(2)))
        prior_delta_yaw.append(float(match.group(3)))
    for match in re.finditer(r"NO_ENOUGH_SOURCE_POINTS(?::\s*(\d+))?", text):
        if match.group(1) is not None:
            no_source_sizes.append(int(match.group(1)))
    for match in re.finditer(r"NO_ENOUGH_POINT_PAIR:\s*(\d+)", text):
        no_pair_sizes.append(int(match.group(1)))
    for match in re.finditer(
        r"Large delta Lidar pose, delta yaw:\s*([-0-9.]+)\s*, delta xy dis:\s*([-0-9.]+)",
        text,
    ):
        large_delta_yaw.append(float(match.group(1)))
        large_delta_xy.append(float(match.group(2)))
    for match in re.finditer(r"REJECTED too_large dx=([-0-9.]+) yaw=([-0-9.]+)", text):
        too_large_dx.append(float(match.group(1)))
        too_large_yaw.append(float(match.group(2)))

    return {
        "raw_base": stats([x[0] for x in cloud]),
        "after_blind": stats([x[1] for x in cloud]),
        "semantic": stats([x[0] for x in semantic]),
        "valid_pairs": stats(valid_pairs),
        "valid_pair_ratio": stats(valid_pair_ratios),
        "match_score": stats(match_scores),
        "prior_delta_xy": stats(prior_delta_xy),
        "prior_delta_yaw": stats(prior_delta_yaw),
        "no_source_count": len(re.findall(r"NO_ENOUGH_SOURCE_POINTS", text)),
        "no_source_size": stats(no_source_sizes),
        "no_pair_count": len(no_pair_sizes),
        "no_pair_size": stats(no_pair_sizes),
        "large_delta_count": len(large_delta_xy),
        "large_delta_xy": stats(large_delta_xy),
        "large_delta_yaw": stats(large_delta_yaw),
        "too_large_count": len(too_large_dx),
        "too_large_dx": stats(too_large_dx),
        "too_large_yaw": stats(too_large_yaw),
    }


def parse_samples(path):
    if not path.exists():
        return {}
    prior_steps = []
    map_odom_steps = []
    prior_robot_error = []
    prev_prior = None
    prev_map_odom = None
    with path.open(newline="") as fp:
        for row in csv.DictReader(fp):
            def triple(prefix):
                vals = [row.get(f"{prefix}_x"), row.get(f"{prefix}_y"), row.get(f"{prefix}_yaw")]
                if any(v in (None, "") for v in vals):
                    return None
                return tuple(float(v) for v in vals)

            prior = triple("prior")
            robot = triple("robot")
            map_odom = triple("map_odom")
            if prior and prev_prior:
                prior_steps.append(math.hypot(prior[0] - prev_prior[0], prior[1] - prev_prior[1]))
            if map_odom and prev_map_odom:
                map_odom_steps.append(math.hypot(map_odom[0] - prev_map_odom[0], map_odom[1] - prev_map_odom[1]))
            if prior and robot:
                prior_robot_error.append(math.hypot(prior[0] - robot[0], prior[1] - robot[1]))
            if prior:
                prev_prior = prior
            if map_odom:
                prev_map_odom = map_odom
    return {
        "prior_step": stats(prior_steps),
        "map_odom_step": stats(map_odom_steps),
        "prior_robot_xy_error": stats(prior_robot_error),
    }


def fmt(value):
    if value is None:
        return ""
    if isinstance(value, int):
        return str(value)
    return f"{value:.3f}"


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_robosense_blind_sweep.py RUN_DIR")
    run_dir = Path(sys.argv[1])
    rows = []
    details = {}
    for child in sorted(run_dir.glob("blind_*")):
        if not child.is_dir():
            continue
        info = {}
        info_path = child / "run_info.txt"
        if info_path.exists():
            for line in info_path.read_text(errors="replace").splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    info[k] = v
        metrics = parse_launch_log(child / "launch.log")
        metrics.update(parse_samples(child / "samples.csv"))
        summary_path = child / "summary.json"
        if summary_path.exists():
            metrics["monitor_summary"] = json.loads(summary_path.read_text())
        label = child.name
        distance = info.get("blind_distance", label.replace("blind_", "").replace("p", "."))
        details[label] = {"blind_distance": distance, "metrics": metrics}
        rows.append({
            "label": label,
            "blind_distance": distance,
            "raw_mean": metrics["raw_base"].get("mean"),
            "raw_min": metrics["raw_base"].get("min"),
            "after_blind_mean": metrics["after_blind"].get("mean"),
            "after_blind_min": metrics["after_blind"].get("min"),
            "semantic_mean": metrics["semantic"].get("mean"),
            "semantic_min": metrics["semantic"].get("min"),
            "semantic_p50": metrics["semantic"].get("p50"),
            "semantic_p95": metrics["semantic"].get("p95"),
            "valid_pair_mean": metrics["valid_pairs"].get("mean"),
            "valid_pair_p50": metrics["valid_pairs"].get("p50"),
            "valid_pair_ratio_mean": metrics["valid_pair_ratio"].get("mean"),
            "match_score_mean": metrics["match_score"].get("mean"),
            "match_score_p95": metrics["match_score"].get("p95"),
            "prior_delta_xy_p95": metrics["prior_delta_xy"].get("p95"),
            "prior_delta_yaw_p95": metrics["prior_delta_yaw"].get("p95"),
            "no_source_count": metrics["no_source_count"],
            "no_pair_count": metrics["no_pair_count"],
            "large_delta_count": metrics["large_delta_count"],
            "too_large_count": metrics["too_large_count"],
            "prior_step_max": metrics.get("prior_step", {}).get("max"),
            "map_odom_step_max": metrics.get("map_odom_step", {}).get("max"),
            "prior_robot_error_mean": metrics.get("prior_robot_xy_error", {}).get("mean"),
            "prior_robot_error_p95": metrics.get("prior_robot_xy_error", {}).get("p95"),
        })

    with (run_dir / "summary.json").open("w") as fp:
        json.dump(details, fp, indent=2, ensure_ascii=False)

    csv_fields = list(rows[0].keys()) if rows else []
    if rows:
        with (run_dir / "summary.csv").open("w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=csv_fields)
            writer.writeheader()
            writer.writerows(rows)

    lines = [
        "# RoboSense blind_distance sweep",
        "",
        "| label | blind | raw mean | raw min | after blind mean | after blind min | semantic mean | semantic min | valid ratio mean | match mean/p95 | prior dxy p95 | no source | no pair | large delta | too_large | prior step max | map->odom step max | prior-robot err mean/p95 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['label']} | {row['blind_distance']} | {fmt(row['raw_mean'])} | "
            f"{fmt(row['raw_min'])} | {fmt(row['after_blind_mean'])} | {fmt(row['after_blind_min'])} | "
            f"{fmt(row['semantic_mean'])} | {fmt(row['semantic_min'])} | "
            f"{fmt(row['valid_pair_ratio_mean'])} | {fmt(row['match_score_mean'])}/{fmt(row['match_score_p95'])} | "
            f"{fmt(row['prior_delta_xy_p95'])} | {row['no_source_count']} | "
            f"{row['no_pair_count']} | {row['large_delta_count']} | {row['too_large_count']} | "
            f"{fmt(row['prior_step_max'])} | {fmt(row['map_odom_step_max'])} | "
            f"{fmt(row['prior_robot_error_mean'])}/{fmt(row['prior_robot_error_p95'])} |"
        )
    lines.append("")
    lines.append("Raw details are in `summary.json`; per-run logs and samples are under each `blind_*` directory.")
    (run_dir / "report.md").write_text("\n".join(lines) + "\n")
    print(run_dir / "report.md")


if __name__ == "__main__":
    main()
