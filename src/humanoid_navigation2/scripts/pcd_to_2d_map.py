#!/usr/bin/env python3
"""
Project a Fast-LIO PCD map into a Nav2-compatible 2D occupancy map.

This is intended for keeping the 2D static map aligned with the 3D PCD map
used by lidar_localization. By default it applies the same Fast-LIO
camera_init -> ROS map rotation used by lidar_localization_component.cpp.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import numpy as np


R_FASTLIO_TO_ROS = np.array(
    [
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float32,
)

FREE = 254
OCCUPIED = 0
UNKNOWN = 205


def parse_simple_yaml(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def read_pgm(path: Path) -> np.ndarray:
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"Unsupported PGM magic {magic!r}: {path}")

        tokens: list[bytes] = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                raise ValueError(f"Incomplete PGM header: {path}")
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())

        width, height, max_value = (int(tokens[0]), int(tokens[1]), int(tokens[2]))
        if max_value > 255:
            raise ValueError(f"Only 8-bit PGM is supported: {path}")

        if magic == b"P5":
            data = np.frombuffer(f.read(width * height), dtype=np.uint8)
        else:
            data = np.array([int(v) for v in f.read().split()], dtype=np.uint8)

    if data.size != width * height:
        raise ValueError(f"PGM data size mismatch: {path}")
    return data.reshape((height, width)).copy()


def write_pgm(path: Path, image: np.ndarray) -> None:
    image = np.asarray(image, dtype=np.uint8)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = f"P5\n# Generated from PCD projection\n{image.shape[1]} {image.shape[0]}\n255\n"
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        f.write(image.tobytes())


def read_pcd_xyz(path: Path) -> np.ndarray:
    header: list[str] = []
    fields: list[str] | None = None
    data_type: str | None = None

    with path.open("rb") as f:
        while True:
            line_bytes = f.readline()
            if not line_bytes:
                raise ValueError(f"PCD header is incomplete: {path}")
            line = line_bytes.decode("ascii", errors="strict").strip()
            header.append(line)
            if line.startswith("FIELDS "):
                fields = line.split()[1:]
            elif line.startswith("DATA "):
                data_type = line.split()[1].lower()
                break

    if fields is None:
        raise ValueError(f"PCD FIELDS line is missing: {path}")
    if data_type != "ascii":
        raise ValueError(f"Only ASCII PCD input is supported, got DATA {data_type}: {path}")

    field_index = {name: i for i, name in enumerate(fields)}
    missing = [name for name in ("x", "y", "z") if name not in field_index]
    if missing:
        raise ValueError(f"PCD missing fields {missing}: {path}")

    xyz = np.loadtxt(
        path,
        dtype=np.float32,
        comments="#",
        skiprows=len(header),
        usecols=(field_index["x"], field_index["y"], field_index["z"]),
    )
    if xyz.ndim == 1:
        xyz = xyz.reshape(1, 3)
    return xyz[np.isfinite(xyz).all(axis=1)]


def parse_origin(value: str) -> tuple[float, float, float]:
    numbers = [float(x) for x in re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", value)]
    if len(numbers) < 3:
        raise ValueError(f"Could not parse origin: {value!r}")
    return numbers[0], numbers[1], numbers[2]


def dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask
    out = np.zeros_like(mask, dtype=bool)
    rows, cols = np.where(mask)
    height, width = mask.shape
    for dy in range(-radius, radius + 1):
        src_r0 = max(0, -dy)
        src_r1 = min(height, height - dy)
        dst_r0 = max(0, dy)
        dst_r1 = min(height, height + dy)
        for dx in range(-radius, radius + 1):
            src_c0 = max(0, -dx)
            src_c1 = min(width, width - dx)
            dst_c0 = max(0, dx)
            dst_c1 = min(width, width + dx)
            out[dst_r0:dst_r1, dst_c0:dst_c1] |= mask[src_r0:src_r1, src_c0:src_c1]
    _ = rows, cols
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_pcd", type=Path)
    parser.add_argument("output_yaml", type=Path)
    parser.add_argument("--base-map-yaml", type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--origin-x", type=float)
    parser.add_argument("--origin-y", type=float)
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--min-z", type=float, default=0.30)
    parser.add_argument("--max-z", type=float, default=1.80)
    parser.add_argument("--min-points-per-cell", type=int, default=2)
    parser.add_argument("--dilate-cells", type=int, default=1)
    parser.add_argument(
        "--transform",
        choices=("fastlio_to_ros", "none"),
        default="fastlio_to_ros",
    )
    args = parser.parse_args()

    base_image: np.ndarray | None = None
    origin_theta = 0.0
    if args.base_map_yaml:
        base_values = parse_simple_yaml(args.base_map_yaml)
        base_image_path = Path(base_values["image"])
        if not base_image_path.is_absolute():
            base_image_path = args.base_map_yaml.parent / base_image_path
        base_image = read_pgm(base_image_path)
        args.resolution = float(base_values["resolution"])
        args.origin_x, args.origin_y, origin_theta = parse_origin(base_values["origin"])
        args.height, args.width = base_image.shape

    if None in (args.origin_x, args.origin_y, args.width, args.height):
        xyz_for_bounds = read_pcd_xyz(args.input_pcd)
        if args.transform == "fastlio_to_ros":
            xyz_for_bounds = (R_FASTLIO_TO_ROS @ xyz_for_bounds.T).T
        z_mask = (xyz_for_bounds[:, 2] >= args.min_z) & (xyz_for_bounds[:, 2] <= args.max_z)
        xy = xyz_for_bounds[z_mask, :2]
        if len(xy) == 0:
            raise ValueError("No points remain after height filtering")
        min_xy = np.floor((xy.min(axis=0) - 0.5) / args.resolution) * args.resolution
        max_xy = np.ceil((xy.max(axis=0) + 0.5) / args.resolution) * args.resolution
        args.origin_x = float(min_xy[0])
        args.origin_y = float(min_xy[1])
        args.width = int(math.ceil((max_xy[0] - min_xy[0]) / args.resolution))
        args.height = int(math.ceil((max_xy[1] - min_xy[1]) / args.resolution))
        xyz = xyz_for_bounds
    else:
        xyz = read_pcd_xyz(args.input_pcd)
        if args.transform == "fastlio_to_ros":
            xyz = (R_FASTLIO_TO_ROS @ xyz.T).T

    z_mask = (xyz[:, 2] >= args.min_z) & (xyz[:, 2] <= args.max_z)
    xyz = xyz[z_mask]

    col = np.floor((xyz[:, 0] - args.origin_x) / args.resolution).astype(np.int32)
    row_from_bottom = np.floor((xyz[:, 1] - args.origin_y) / args.resolution).astype(np.int32)
    row = args.height - 1 - row_from_bottom
    in_bounds = (col >= 0) & (col < args.width) & (row >= 0) & (row < args.height)
    row = row[in_bounds]
    col = col[in_bounds]

    counts = np.zeros((args.height, args.width), dtype=np.uint16)
    np.add.at(counts, (row, col), 1)
    occupied_mask = counts >= args.min_points_per_cell
    occupied_mask = dilate(occupied_mask, args.dilate_cells)

    if base_image is not None:
        # Preserve known/unknown coverage from the existing map, but replace old
        # occupied cells with PCD-derived occupied cells.
        output = np.where(base_image == UNKNOWN, UNKNOWN, FREE).astype(np.uint8)
    else:
        output = np.full((args.height, args.width), FREE, dtype=np.uint8)
    output[occupied_mask] = OCCUPIED

    output_pgm = args.output_yaml.with_suffix(".pgm")
    write_pgm(output_pgm, output)
    args.output_yaml.parent.mkdir(parents=True, exist_ok=True)
    args.output_yaml.write_text(
        "\n".join(
            [
                f"image: {output_pgm.name}",
                "mode: trinary",
                f"resolution: {args.resolution:.3f}",
                f"origin: [{args.origin_x:.6f}, {args.origin_y:.6f}, {origin_theta:.6f}]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )

    print(f"input points after height filter: {len(xyz):,}")
    print(f"points inside output bounds: {len(row):,}")
    print(f"occupied cells before dilation: {int((counts >= args.min_points_per_cell).sum()):,}")
    print(f"occupied cells after dilation: {int(occupied_mask.sum()):,}")
    print(f"wrote: {args.output_yaml}")
    print(f"wrote: {output_pgm}")


if __name__ == "__main__":
    main()
