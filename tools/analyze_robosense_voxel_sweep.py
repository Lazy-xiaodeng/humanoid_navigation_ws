#!/usr/bin/env python3
import csv
import json
import math
import re
import statistics
import sys
from pathlib import Path


WINDOWS = (
    ("all", None, None),
    ("bad_150_163", 150.0, 163.0),
    ("bad_206_215", 206.0, 215.0),
)


def mean(values):
    return statistics.fmean(values) if values else None


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


def fmt(value):
    if value is None:
        return ""
    if isinstance(value, int):
        return str(value)
    return f"{value:.3f}"


def read_info(path):
    info = {}
    if not path.exists():
        return info
    for line in path.read_text(errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            info[key] = value
    return info


def parse_records(log_path):
    first_ts = None
    current = None
    records = []
    if not log_path.exists():
        return records

    def finish():
        if current is not None:
            records.append(current.copy())

    for line in log_path.read_text(errors="replace").splitlines():
        match = re.search(r"lidar timestamp :([0-9.]+)", line)
        if match:
            finish()
            ts = float(match.group(1))
            if first_ts is None:
                first_ts = ts
            current = {
                "rel_time": ts - first_ts,
                "no_pair": False,
                "no_source": False,
                "large_delta": False,
            }
            continue

        if current is None:
            continue

        match = re.search(r"cloud_size raw_base=(\d+) after_blind=(\d+)", line)
        if match:
            current["raw_base"] = int(match.group(1))
            current["after_blind"] = int(match.group(2))
            continue

        match = re.search(r"semantic_cloud size:\s*(\d+)", line)
        if match:
            current["semantic"] = int(match.group(1))
            continue

        match = re.search(
            r"pair_geometry source=(\d+) valid_pair=(\d+) valid_pair_ratio=([-0-9.]+) "
            r"gnd=(\d+) non_gnd=(\d+) scan_non_gnd=(\d+) .*normal_eigen=\s*"
            r"([-0-9.eE+]+)\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)",
            line,
        )
        if match:
            current["source"] = int(match.group(1))
            current["valid_pair"] = int(match.group(2))
            current["valid_pair_ratio"] = float(match.group(3))
            current["gnd"] = int(match.group(4))
            current["non_gnd"] = int(match.group(5))
            current["scan_non_gnd"] = int(match.group(6))
            current["normal_eig0"] = float(match.group(7))
            current["normal_eig1"] = float(match.group(8))
            current["normal_eig2"] = float(match.group(9))
            continue

        match = re.search(
            r"match_score:\s*([-0-9.]+)\s+prior_delta_xy:\s*([-0-9.]+).*"
            r"prior_delta_yaw:\s*([-0-9.]+)",
            line,
        )
        if match:
            current["match_score"] = float(match.group(1))
            current["prior_delta_xy"] = float(match.group(2))
            current["prior_delta_yaw"] = float(match.group(3))
            continue

        if "NO_ENOUGH_POINT_PAIR" in line:
            current["no_pair"] = True
            match = re.search(r"NO_ENOUGH_POINT_PAIR:\s*(\d+)", line)
            if match:
                current["no_pair_size"] = int(match.group(1))
        elif "NO_ENOUGH_SOURCE_POINTS" in line:
            current["no_source"] = True
        elif "Large delta Lidar pose" in line:
            current["large_delta"] = True

    finish()
    return records


def summarize_window(records, lo, hi):
    if lo is not None:
        records = [r for r in records if lo <= r.get("rel_time", -1) <= hi]

    def values(key):
        return [r[key] for r in records if key in r]

    no_pair_sizes = values("no_pair_size")
    return {
        "frames": len(records),
        "raw_mean": mean(values("raw_base")),
        "after_blind_mean": mean(values("after_blind")),
        "semantic_mean": mean(values("semantic")),
        "source_mean": mean(values("source")),
        "valid_pair_mean": mean(values("valid_pair")),
        "valid_pair_min": min(values("valid_pair")) if values("valid_pair") else None,
        "valid_pair_p50": percentile(values("valid_pair"), 50),
        "pair_ratio_mean": mean(values("valid_pair_ratio")),
        "no_pair_count": sum(1 for r in records if r.get("no_pair")),
        "no_pair_size_mean": mean(no_pair_sizes),
        "no_pair_size_min": min(no_pair_sizes) if no_pair_sizes else None,
        "no_source_count": sum(1 for r in records if r.get("no_source")),
        "large_delta_count": sum(1 for r in records if r.get("large_delta")),
        "prior_delta_xy_p95": percentile(values("prior_delta_xy"), 95),
        "normal_eig0_mean": mean(values("normal_eig0")),
        "normal_eig1_mean": mean(values("normal_eig1")),
        "normal_eig2_mean": mean(values("normal_eig2")),
    }


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_robosense_voxel_sweep.py RUN_DIR")
    run_dir = Path(sys.argv[1])
    rows = []
    details = {}
    for child in sorted(p for p in run_dir.iterdir() if p.is_dir()):
        info = read_info(child / "run_info.txt")
        records = parse_records(child / "launch.log")
        for window_name, lo, hi in WINDOWS:
            summary = summarize_window(records, lo, hi)
            row = {
                "label": child.name,
                "window": window_name,
                "map_voxel": info.get("map_voxel_leaf_size", ""),
                "source_voxel": info.get("source_voxel_leaf_size", ""),
                "neighbor_radius": info.get("neighbor_search_radius", ""),
                "max_pair_size": info.get("max_pair_size", ""),
                **summary,
            }
            rows.append(row)
        details[child.name] = {"info": info, "windows": {
            name: summarize_window(records, lo, hi) for name, lo, hi in WINDOWS
        }}

    if rows:
        fields = list(rows[0].keys())
        with (run_dir / "summary.csv").open("w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
    (run_dir / "summary.json").write_text(json.dumps(details, indent=2, ensure_ascii=False))

    lines = [
        "# RoboSense voxel sweep",
        "",
        "| label | window | map voxel | source voxel | radius | max pair | semantic mean | valid pair mean/min | no pair | no pair size mean/min | prior dxy p95 | normal eig mean |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['label']} | {row['window']} | {row['map_voxel']} | {row['source_voxel']} | {row['neighbor_radius']} | {row['max_pair_size']} | "
            f"{fmt(row['semantic_mean'])} | {fmt(row['valid_pair_mean'])}/{fmt(row['valid_pair_min'])} | "
            f"{row['no_pair_count']} | {fmt(row['no_pair_size_mean'])}/{fmt(row['no_pair_size_min'])} | "
            f"{fmt(row['prior_delta_xy_p95'])} | "
            f"{fmt(row['normal_eig0_mean'])},{fmt(row['normal_eig1_mean'])},{fmt(row['normal_eig2_mean'])} |"
        )
    lines.append("")
    lines.append("Raw details are in `summary.json`; per-run logs are under each run directory.")
    (run_dir / "report.md").write_text("\n".join(lines) + "\n")
    print(run_dir / "report.md")


if __name__ == "__main__":
    main()
