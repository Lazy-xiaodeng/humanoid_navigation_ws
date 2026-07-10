#!/usr/bin/env python3
"""Prepare a small custom LEADER dataset from existing nav46 PCD manifests.

文件作用：
  1. 这是离线验证辅助脚本，不属于运行态节点。
  2. 把现有 nav46 PCD manifest 转成 LEADER 自定义数据集格式。
  3. 生成的 manifest 只用于后续 LEADER 研究对照，不参与正式全局重定位发布链路。

The official LEADER repository only ships Oxford/NCLT dataloaders.  This
converter keeps our local data format explicit and produces a simple manifest
that a custom LEADER runner can train/test from.
"""

from __future__ import annotations

import argparse
import csv
import json
import struct
from pathlib import Path

import numpy as np


def read_pcd_xyzi(path: Path) -> np.ndarray:
    with path.open("rb") as f:
        header_lines: list[bytes] = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: missing DATA header")
            header_lines.append(line)
            if line.startswith(b"DATA "):
                break
        header = {}
        for raw in header_lines:
            text = raw.decode("ascii", errors="ignore").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            header[parts[0].upper()] = parts[1:]

        fields = header.get("FIELDS", [])
        points = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
        data_kind = header["DATA"][0].lower()
        if not {"x", "y", "z"}.issubset(fields):
            raise ValueError(f"{path}: expected x/y/z fields, got {fields}")

        if data_kind == "ascii":
            arr = np.loadtxt(f, dtype=np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
        elif data_kind == "binary":
            sizes = [int(x) for x in header["SIZE"]]
            types = header["TYPE"]
            counts = [int(x) for x in header.get("COUNT", ["1"] * len(fields))]
            if any(size != 4 or typ != "F" or count != 1 for size, typ, count in zip(sizes, types, counts)):
                raise ValueError(f"{path}: only float32 scalar binary PCD is supported")
            raw = f.read(points * len(fields) * 4)
            arr = np.asarray(struct.unpack("<" + "f" * (points * len(fields)), raw), dtype=np.float32)
            arr = arr.reshape(points, len(fields))
        else:
            raise ValueError(f"{path}: unsupported PCD DATA {data_kind}")

    idx = {name: i for i, name in enumerate(fields)}
    intensity = arr[:, idx["intensity"]] if "intensity" in idx else np.zeros(arr.shape[0], dtype=np.float32)
    xyzi = np.column_stack((arr[:, idx["x"]], arr[:, idx["y"]], arr[:, idx["z"]], intensity)).astype(np.float32)
    return xyzi[np.isfinite(xyzi).all(axis=1)]


def load_rows(csv_path: Path, split: str) -> list[dict[str, str]]:
    with csv_path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        row["split"] = split
    return rows


def row_id(row: dict[str, str]) -> str:
    if row.get("target_id"):
        return row["target_id"]
    return f"keyframe_{int(row['frame_id']):06d}"


def write_split(rows: list[dict[str, str]], output_dir: Path, split: str) -> list[dict[str, object]]:
    split_dir = output_dir / "scans" / split
    split_dir.mkdir(parents=True, exist_ok=True)
    manifest_rows: list[dict[str, object]] = []
    for row in rows:
        rid = row_id(row)
        pcd_path = Path(row["pcd"])
        if not pcd_path.exists():
            raise FileNotFoundError(pcd_path)
        scan = read_pcd_xyzi(pcd_path)
        bin_path = split_dir / f"{rid}.bin"
        scan.tofile(bin_path)
        manifest_rows.append(
            {
                "id": rid,
                "split": split,
                "cloud_index": int(row["cloud_index"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row.get("z", 0.0)),
                "yaw_deg": float(row["yaw_deg"]),
                "bin": str(bin_path.resolve()),
                "pcd": str(pcd_path.resolve()),
                "points": int(scan.shape[0]),
            }
        )
    return manifest_rows


def write_manifest(rows: list[dict[str, object]], output_dir: Path) -> None:
    csv_path = output_dir / "manifest.csv"
    fields = ["id", "split", "cloud_index", "x", "y", "z", "yaw_deg", "bin", "pcd", "points"]
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "manifest.json").write_text(
        json.dumps({"rows": rows, "manifest_csv": str(csv_path.resolve())}, indent=2),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-dataset",
        default=".codex_tmp/official_std_standalone/nav46_hard52_dataset",
        help="Dataset directory containing keyframes.csv and targets.csv.",
    )
    parser.add_argument("--output-dir", default=".codex_tmp/leader_nav46/hard52")
    args = parser.parse_args()

    source = Path(args.source_dataset).resolve()
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)

    keyframes = load_rows(source / "keyframes.csv", "train")
    targets = load_rows(source / "targets.csv", "test")
    rows = write_split(keyframes, output, "train") + write_split(targets, output, "test")
    write_manifest(rows, output)

    train_points = sum(int(r["points"]) for r in rows if r["split"] == "train")
    test_points = sum(int(r["points"]) for r in rows if r["split"] == "test")
    print(
        f"leader_dataset output={output} train={len(keyframes)} test={len(targets)} "
        f"train_points={train_points} test_points={test_points}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
