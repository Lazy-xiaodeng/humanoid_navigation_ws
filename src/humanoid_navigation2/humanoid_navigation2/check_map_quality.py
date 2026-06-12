#!/usr/bin/env python3
"""
check_map_quality.py — 检查 PCD 地图的 NDT 定位退化风险

用法：
  python3 check_map_quality.py [/path/to/map.pcd]

输出：
  - 每 4m 区域的退化率（z-std < 0.1m 的 NDT 格占比，用 █ 可视化）
  - 退化热点区域坐标
  - 整体退化等级评估

解释：
  退化率 = 该区域内 z_std < 0.1m 的 NDT 网格占比
  - < 10%: 优秀（全方向约束充足）
  - 10-30%: 一般（可能有轻微滑动）
  - 30-50%: 较差（建议建图时多角度扫描）
  - > 50%: 差（NDT 大概率漂移，建议重建此区域）
"""

import numpy as np
import sys
import os

from scipy.spatial.transform import Rotation as R

NDT_RES = 0.5       # NDT 网格分辨率
REGION_SIZE = 4.0    # 分析区域大小
LOW_VAR_THRESH = 0.1  # z-std 低于此值视为退化

ROT_Q = R.from_quat([-0.5, -0.5, 0.5, 0.5])  # LiDAR→ROS 旋转


def load_pcd(path):
    points = []
    with open(path, 'r') as f:
        in_header = True
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if in_header:
                if line.startswith('DATA'):
                    in_header = False
                continue
            vals = line.split()
            if len(vals) >= 3:
                points.append([float(vals[0]), float(vals[1]), float(vals[2])])
    return np.array(points, dtype=np.float64)


def analyze(path):
    print(f"Loading {path}...")
    pts = load_pcd(path)
    print(f"Loaded {len(pts):,} points\n")

    # 应用 ROS 坐标转换
    pts_ros = ROT_Q.apply(pts)
    xr, yr, zr = pts_ros[:, 0], pts_ros[:, 1], pts_ros[:, 2]
    print(f"Map extent (ROS frame, Z-up, P0.1-P99.9 percentile):")
    z_lo = np.percentile(zr, 0.1)
    z_hi = np.percentile(zr, 99.9)
    print(f"  X: [{np.percentile(xr, 0.1):.1f}, {np.percentile(xr, 99.9):.1f}] span={xr.max()-xr.min():.0f}m")
    print(f"  Y: [{np.percentile(yr, 0.1):.1f}, {np.percentile(yr, 99.9):.1f}] span={yr.max()-yr.min():.0f}m")
    print(f"  Z: [{z_lo:.2f}, {z_hi:.2f}] span={z_hi-z_lo:.1f}m (ground floor to ceiling)")
    print(f"     absolute range: [{zr.min():.2f}, {zr.max():.2f}] — only {((zr<z_lo)|(zr>z_hi)).sum()} outlier pts\n")

    # NDT 格级退化分析
    grid_res = NDT_RES
    x_bins = np.arange(xr.min(), xr.max() + grid_res, grid_res)
    y_bins = np.arange(yr.min(), yr.max() + grid_res, grid_res)

    cells = []
    for i in range(len(x_bins)-1):
        for j in range(len(y_bins)-1):
            mask = (xr >= x_bins[i]) & (xr < x_bins[i+1]) & (yr >= y_bins[j]) & (yr < y_bins[j+1])
            count = mask.sum()
            if count >= 5:
                z_std = float(np.std(zr[mask]))
                cells.append({
                    'x': (x_bins[i] + x_bins[i+1]) / 2,
                    'y': (y_bins[j] + y_bins[j+1]) / 2,
                    'z_std': z_std,
                    'count': count,
                })

    if not cells:
        print("ERROR: no occupied cells found")
        return

    # 区域退化率
    big_grid = REGION_SIZE
    x_big = np.arange(xr.min(), xr.max() + big_grid, big_grid)
    y_big = np.arange(yr.min(), yr.max() + big_grid, big_grid)

    total_degenerate = 0
    total_cells = 0
    region_results = []

    for i in range(len(x_big)-1):
        for j in range(len(y_big)-1):
            region_cells = [c for c in cells if
                            x_big[i] <= c['x'] < x_big[i+1] and
                            y_big[j] <= c['y'] < y_big[j+1]]
            if len(region_cells) < 3:
                continue
            degenerate = sum(1 for c in region_cells if c['z_std'] < LOW_VAR_THRESH)
            pct = degenerate / len(region_cells) * 100
            total_degenerate += degenerate
            total_cells += len(region_cells)
            bar = '█' * int(pct / 5) if pct > 5 else ('·' if pct == 0 else '▁')
            region_results.append({
                'x_range': (x_big[i], x_big[i+1]),
                'y_range': (y_big[j], y_big[j+1]),
                'cells': len(region_cells),
                'degenerate_pct': pct,
                'bar': bar,
            })

    overall_pct = total_degenerate / total_cells * 100 if total_cells > 0 else 0

    print("=" * 72)
    print(f"  退化率地图 (每 {REGION_SIZE:.0f}m，NDT 格 z-std < {LOW_VAR_THRESH}m)")
    print("=" * 72)
    print(f"  {'X 范围':>16s}  {'Y 范围':>16s}  {'cells':>5s}  {'退化%':>6s}")
    print("-" * 72)
    for r in sorted(region_results, key=lambda x: x['degenerate_pct'], reverse=True):
        xrng = f"[{r['x_range'][0]:6.1f},{r['x_range'][1]:6.1f}]"
        yrng = f"[{r['y_range'][0]:6.1f},{r['y_range'][1]:6.1f}]"
        print(f"  {xrng} x {yrng}  {r['cells']:5d}  {r['degenerate_pct']:6.1f}% {r['bar']}")

    print("-" * 72)
    print(f"  整体退化率: {total_degenerate}/{total_cells} = {overall_pct:.1f}%")

    if overall_pct < 10:
        grade = "A (优秀)"
    elif overall_pct < 20:
        grade = "B (良好)"
    elif overall_pct < 30:
        grade = "C (一般)"
    elif overall_pct < 40:
        grade = "D (较差)"
    else:
        grade = "F (差)"
    print(f"  地图质量等级: {grade}")

    # 热点
    hotspots = sorted(cells, key=lambda c: c['z_std'])[:10]
    print(f"\n  10 个最退化 NDT 格 (建议建图时在这些位置多角度扫):")
    for h in hotspots:
        print(f"    ({h['x']:6.1f}, {h['y']:6.1f})  z_std={h['z_std']:.4f}m  pts={h['count']}")

    # 建议
    high_risk = [r for r in region_results if r['degenerate_pct'] > 30]
    if high_risk:
        print(f"\n  ⚠ {len(high_risk)} 个区域退化率 > 30%：")
        for r in high_risk:
            xrng = f"X∈[{r['x_range'][0]:.0f},{r['x_range'][1]:.0f}]"
            yrng = f"Y∈[{r['y_range'][0]:.0f},{r['y_range'][1]:.0f}]"
            print(f"    {xrng} {yrng} → {r['degenerate_pct']:.0f}% 退化")
        print(f"\n  建议:")
        print(f"    1. 在这些区域做 360° 原地旋转（每个热点 ≥10s）")
        print(f"    2. 沿 Z 字形慢速通过（让 LiDAR 多角度扫天花板结构）")
        print(f"    3. 重建后重新运行此脚本验证退化率是否下降")

    print(f"\n  指标解释:")
    print(f"    z_std: NDT 格内点云的 Z 方向标准差")
    print(f"    < 0.05m: 几乎纯平面（地面），无约束")
    print(f"    0.05-0.3m: 有部分结构（矮墙/家具）")
    print(f"    > 0.3m: 丰富 3D 结构（墙壁+天花板），强约束")


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = '/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd'
    analyze(path)


if __name__ == '__main__':
    main()
