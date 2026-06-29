#!/usr/bin/env python3
"""
Create a grounded ROS-frame localization PCD from a Fast-LIO camera_init PCD.

Fast-LIO map frame used here:
  camera_init/body: x left, y down, z back

Navigation/localization frame:
  map/base_footprint: x forward, y left, z up

The transform matches navigation2.launch.py:
  odom -> camera_init rotation:
    qx=-0.5, qy=-0.5, qz=0.5, qw=0.5
  body -> base_footprint translation in body frame:
    x=0.004, y=1.215, z=0.072

For a map whose origin should be the initial base_footprint, points are
converted as:
  p_map = R_camera_to_map * (p_camera - t_camera_base)
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


R_CAMERA_TO_MAP = np.array(
    [
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float32,
)

T_CAMERA_BASE = np.array([0.004, 1.215, 0.072], dtype=np.float32)


def _parse_header_value(header: list[str], key: str) -> list[str]:
    prefix = key + " "
    for line in header:
        if line.startswith(prefix):
            return line.split()[1:]
    return []


def _pcd_dtype(fields: list[str], sizes: list[str], types: list[str], counts: list[str]) -> np.dtype:
    if len(fields) != len(sizes) or len(fields) != len(types) or len(fields) != len(counts):
        raise ValueError("PCD header FIELDS/SIZE/TYPE/COUNT lengths do not match")

    dtype_fields = []
    for field, size, type_name, count in zip(fields, sizes, types, counts):
        size_int = int(size)
        count_int = int(count)
        type_name = type_name.upper()
        if count_int != 1:
            raise ValueError(f"Unsupported PCD field COUNT for {field}: {count_int}")
        if type_name == "F" and size_int == 4:
            dtype = "<f4"
        elif type_name == "F" and size_int == 8:
            dtype = "<f8"
        elif type_name == "I" and size_int == 1:
            dtype = "<i1"
        elif type_name == "I" and size_int == 2:
            dtype = "<i2"
        elif type_name == "I" and size_int == 4:
            dtype = "<i4"
        elif type_name == "U" and size_int == 1:
            dtype = "<u1"
        elif type_name == "U" and size_int == 2:
            dtype = "<u2"
        elif type_name == "U" and size_int == 4:
            dtype = "<u4"
        else:
            raise ValueError(f"Unsupported PCD field type for {field}: TYPE={type_name}, SIZE={size_int}")
        dtype_fields.append((field, dtype))
    return np.dtype(dtype_fields)


def _structured_to_matrix(data: np.ndarray, fields: list[str]) -> np.ndarray:
    matrix = np.empty((len(data), len(fields)), dtype=np.float32)
    for i, field in enumerate(fields):
        matrix[:, i] = data[field].astype(np.float32, copy=False)
    return matrix


def read_pcd(path: Path) -> tuple[list[str], list[str], np.ndarray]:
    header: list[str] = []
    fields: list[str] | None = None
    data_offset = 0
    data_type = ""

    with path.open("rb") as f:
        while True:
            line_bytes = f.readline()
            if not line_bytes:
                raise ValueError(f"PCD header is incomplete: {path}")
            data_offset = f.tell()
            line = line_bytes.decode("ascii", errors="strict").strip()
            header.append(line)
            if line.startswith("FIELDS "):
                fields = line.split()[1:]
            if line.startswith("DATA "):
                data_type = line.split()[1].lower()
                break

    if not fields:
        raise ValueError(f"PCD FIELDS line is missing: {path}")

    if data_type == "ascii":
        data = np.loadtxt(path, dtype=np.float32, comments="#", skiprows=len(header))
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] != len(fields):
            raise ValueError(
                f"PCD field count mismatch: header has {len(fields)}, data has {data.shape[1]}"
            )
        return header, fields, data

    if data_type != "binary":
        raise ValueError(f"Unsupported PCD DATA type: {data_type}")

    sizes = _parse_header_value(header, "SIZE")
    types = _parse_header_value(header, "TYPE")
    counts = _parse_header_value(header, "COUNT") or ["1"] * len(fields)
    points_values = _parse_header_value(header, "POINTS")
    if not points_values:
        raise ValueError(f"PCD POINTS line is missing: {path}")

    dtype = _pcd_dtype(fields, sizes, types, counts)
    with path.open("rb") as f:
        f.seek(data_offset)
        structured = np.fromfile(f, dtype=dtype, count=int(points_values[0]))
    data = _structured_to_matrix(structured, fields)
    return header, fields, data


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0.0 or len(points) == 0:
        return points

    grid = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(grid, axis=0, return_index=True)
    unique_idx.sort()
    return points[unique_idx]


def write_binary_pcd(path: Path, points: np.ndarray, fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points, dtype=np.float32)

    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS " + " ".join(fields),
            "SIZE " + " ".join(["4"] * len(fields)),
            "TYPE " + " ".join(["F"] * len(fields)),
            "COUNT " + " ".join(["1"] * len(fields)),
            f"WIDTH {len(points)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(points)}",
            "DATA binary",
            "",
        ]
    ).encode("ascii")

    with path.open("wb") as f:
        f.write(header)
        points.tofile(f)


def summarize(label: str, xyz: np.ndarray) -> None:
    print(f"{label}: {len(xyz):,} points")
    if len(xyz) == 0:
        return
    print(f"  min: {xyz.min(axis=0)}")
    print(f"  max: {xyz.max(axis=0)}")
    print(
        "  z quantiles:",
        np.quantile(xyz[:, 2], [0.01, 0.05, 0.10, 0.50, 0.90, 0.95, 0.99]),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_pcd", type=Path)
    parser.add_argument("output_pcd", type=Path)
    parser.add_argument("--min-z", type=float, default=0.0)
    parser.add_argument("--max-z", type=float, default=2.30)
    parser.add_argument("--voxel-size", type=float, default=0.10)
    parser.add_argument(
        "--keep-ceiling",
        action="store_true",
        help="Keep all points above min-z instead of clipping at max-z.",
    )
    parser.add_argument(
        "--keep-floor",
        dest="keep_ceiling",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    args = parser.parse_args()

    _, fields, data = read_pcd(args.input_pcd)
    field_index = {name: i for i, name in enumerate(fields)}
    for required in ("x", "y", "z"):
        if required not in field_index:
            raise ValueError(f"Input PCD is missing required field: {required}")

    xyz_cols = [field_index["x"], field_index["y"], field_index["z"]]
    finite_mask = np.isfinite(data[:, xyz_cols]).all(axis=1)
    data = data[finite_mask]
    summarize("input camera_init", data[:, xyz_cols])

    xyz_map = (R_CAMERA_TO_MAP @ (data[:, xyz_cols] - T_CAMERA_BASE).T).T
    data[:, xyz_cols] = xyz_map
    if all(name in field_index for name in ("normal_x", "normal_y", "normal_z")):
        normal_cols = [field_index["normal_x"], field_index["normal_y"], field_index["normal_z"]]
        data[:, normal_cols] = (R_CAMERA_TO_MAP @ data[:, normal_cols].T).T
    summarize("grounded ROS map", xyz_map)

    if args.keep_ceiling:
        z_mask = xyz_map[:, 2] >= args.min_z
    else:
        z_mask = (xyz_map[:, 2] >= args.min_z) & (xyz_map[:, 2] <= args.max_z)
    data = data[z_mask]
    summarize("height-filtered map", data[:, xyz_cols])

    data = voxel_downsample(data, args.voxel_size)
    summarize("voxelized map", data[:, xyz_cols])

    write_binary_pcd(args.output_pcd, data, fields)
    print(f"wrote: {args.output_pcd}")


if __name__ == "__main__":
    main()
