#!/usr/bin/env python3
"""Compare old vs new PCD map quality for NDT degradation risk."""
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

NDT_RES = 0.5
REGION_SIZE = 4.0
LOW_VAR_THRESH = 0.1
ROT_Q = R.from_quat([-0.5, -0.5, 0.5, 0.5])

def load_and_analyze(name, path):
    t0 = time.time()

    # Load PCD
    pts = []
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
                pts.append([float(vals[0]), float(vals[1]), float(vals[2])])
    pts = np.array(pts, dtype=np.float64)
    load_t = time.time() - t0

    # Transform to ROS
    pts_ros = ROT_Q.apply(pts)
    xr, yr, zr = pts_ros[:, 0], pts_ros[:, 1], pts_ros[:, 2]

    # Vectorized cell assignment
    x_edges = np.arange(xr.min(), xr.max() + NDT_RES, NDT_RES)
    y_edges = np.arange(yr.min(), yr.max() + NDT_RES, NDT_RES)

    xi = np.digitize(xr, x_edges) - 1
    yi = np.digitize(yr, y_edges) - 1

    valid = (xi >= 0) & (xi < len(x_edges)-1) & (yi >= 0) & (yi < len(y_edges)-1)
    xi, yi, zr_v = xi[valid], yi[valid], zr[valid]

    # Group by cell
    order = np.lexsort((yi, xi))
    xi_sorted = xi[order]
    yi_sorted = yi[order]
    zr_sorted = zr_v[order]

    cell_id = xi_sorted * (len(y_edges)-1) + yi_sorted
    _, starts, counts = np.unique(cell_id, return_index=True, return_counts=True)

    cells = []
    for s, n in zip(starts, counts):
        if n >= 5:
            cx = x_edges[xi_sorted[s]] + NDT_RES/2
            cy = y_edges[yi_sorted[s]] + NDT_RES/2
            z_std = float(np.std(zr_sorted[s:s+n]))
            cells.append({'x': cx, 'y': cy, 'z_std': z_std, 'count': n})

    cell_t = time.time() - t0

    if not cells:
        return None

    # Region analysis
    xr_min, xr_max = xr.min(), xr.max()
    yr_min, yr_max = yr.min(), yr.max()
    x_big = np.arange(xr_min, xr_max + REGION_SIZE, REGION_SIZE)
    y_big = np.arange(yr_min, yr_max + REGION_SIZE, REGION_SIZE)

    total_degenerate = 0
    total_cells = 0
    regions = []

    for i in range(len(x_big)-1):
        for j in range(len(y_big)-1):
            region_cells = [c for c in cells if x_big[i] <= c['x'] < x_big[i+1] and y_big[j] <= c['y'] < y_big[j+1]]
            if len(region_cells) < 3:
                continue
            degenerate = sum(1 for c in region_cells if c['z_std'] < LOW_VAR_THRESH)
            pct = degenerate / len(region_cells) * 100
            total_degenerate += degenerate
            total_cells += len(region_cells)
            bar = '█' * int(pct/5) if pct > 5 else ('·' if pct == 0 else '▁')
            regions.append({
                'x0': x_big[i], 'x1': x_big[i+1],
                'y0': y_big[j], 'y1': y_big[j+1],
                'cells': len(region_cells),
                'pct': pct,
                'bar': bar,
            })

    overall_pct = total_degenerate / total_cells * 100 if total_cells > 0 else 0
    hotspots = sorted(cells, key=lambda c: c['z_std'])[:10]
    high_risk = [r for r in regions if r['pct'] > 30]

    return {
        'name': name,
        'n_points': len(pts),
        'load_t': load_t,
        'cell_t': cell_t,
        'x_range': (xr.min(), xr.max()),
        'y_range': (yr.min(), yr.max()),
        'z_hi_lo': (np.percentile(zr, 0.1), np.percentile(zr, 99.9)),
        'z_abs': (zr.min(), zr.max()),
        'outliers': ((zr < np.percentile(zr, 0.1)) | (zr > np.percentile(zr, 99.9))).sum(),
        'total_cells': total_cells,
        'total_degenerate': total_degenerate,
        'overall_pct': overall_pct,
        'regions': regions,
        'hotspots': hotspots,
        'high_risk': high_risk,
    }


# Run both
old = load_and_analyze('Old Map',
    '/home/ubuntu/地图文件存放处/5-10天津展厅图/pcd/hall.pcd')
new = load_and_analyze('New Map',
    '/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd')

# ── Print comparison ──
print("=" * 80)
print("  Old vs New NDT Degradation Risk Comparison")
print("=" * 80)

print("\n{:<30s} {:>22s} {:>22s}".format("Metric", "Old Map", "New Map"))
print("-" * 76)
print("{:<30s} {:>22,d} {:>22,d}".format("Total Points", old['n_points'], new['n_points']))
print("{:<30s} {:>22s} {:>22s}".format("X Range (ROS)",
    "[{:.0f},{:.0f}]".format(old['x_range'][0], old['x_range'][1]),
    "[{:.0f},{:.0f}]".format(new['x_range'][0], new['x_range'][1])))
print("{:<30s} {:>22s} {:>22s}".format("Y Range (ROS)",
    "[{:.0f},{:.0f}]".format(old['y_range'][0], old['y_range'][1]),
    "[{:.0f},{:.0f}]".format(new['y_range'][0], new['y_range'][1])))
print("{:<30s} {:>22s} {:>22s}".format("Z Range (P0.1-P99.9)",
    "[{:.1f},{:.1f}]m".format(old['z_hi_lo'][0], old['z_hi_lo'][1]),
    "[{:.1f},{:.1f}]m".format(new['z_hi_lo'][0], new['z_hi_lo'][1])))
print("{:<30s} {:>22,d} {:>22,d}".format("Z Outlier pts", old['outliers'], new['outliers']))
print("")

grade_old = 'A' if old['overall_pct']<10 else ('B' if old['overall_pct']<20 else 'C')
grade_new = 'A' if new['overall_pct']<10 else ('B' if new['overall_pct']<20 else 'C')
print("{:<30s} {:>21.1f}% {:>21.1f}%".format("Overall Degradation Rate", old['overall_pct'], new['overall_pct']))
print("{:<30s} {:>22s} {:>22s}".format("Map Grade", grade_old, grade_new))
print("{:<30s} {:>21s} {:>21s}".format("Degenerate Cells",
    f"{old['total_degenerate']}/{old['total_cells']}",
    f"{new['total_degenerate']}/{new['total_cells']}"))

# ── Region comparison ──
print("\n" + "=" * 80)
print("  Per-Region Degradation Rate Comparison (4m grid, sorted by delta)")
print("=" * 80)

def region_key(r):
    return (round(r['x0']), round(r['y0']))

old_reg = {region_key(r): r for r in old['regions']}
new_reg = {region_key(r): r for r in new['regions']}

all_keys = sorted(set(list(old_reg.keys()) + list(new_reg.keys())))
rows = []
for k in all_keys:
    o = old_reg.get(k)
    n = new_reg.get(k)
    if o and n:
        diff = n['pct'] - o['pct']
        rows.append((k, o['pct'], n['pct'], diff, o['cells'], n['cells']))
    elif o:
        rows.append((k, o['pct'], None, None, o['cells'], 0))
    else:
        rows.append((k, None, n['pct'], None, 0, n['cells']))

rows.sort(key=lambda r: r[3] if r[3] is not None else 999)

print("{:<24s} {:>8s} {:>8s} {:>7s} {:>7s} {:>7s}".format(
    "Location (X x Y)", "Old%", "New%", "Delta", "OldCell", "NewCell"))
print("-" * 65)

improved = 0
worsened = 0
same = 0
for k, old_pct, new_pct, diff, old_c, new_c in rows:
    if diff is None:
        continue
    sign = '▼' if diff < -5 else ('▲' if diff > 5 else ' ')
    if diff < -1:
        improved += 1
    elif diff > 1:
        worsened += 1
    else:
        same += 1
    xrng = "[{:>5d},{:>5d}]".format(k[0], k[0]+4)
    yrng = "[{:>5d},{:>5d}]".format(k[1], k[1]+4)
    o_s = "{:.1f}%".format(old_pct) if old_pct is not None else 'N/A'
    n_s = "{:.1f}%".format(new_pct) if new_pct is not None else 'N/A'
    d_s = "{:+.1f}%".format(diff)
    print("  {} x {}  {:>8s}  {:>8s}  {}{:>6s}  {:>6d}  {:>6d}".format(
        xrng, yrng, o_s, n_s, sign, d_s, old_c, new_c))

print("\nImproved: {}  Worsened: {}  Unchanged: {}".format(improved, worsened, same))

# ── Hotspot comparison ──
print("\n" + "=" * 80)
print("  Degradation Hotspot Comparison (Top 10 worst NDT cells)")
print("=" * 80)
print("  {:>30s}  |  {:>30s}".format("Old Map", "New Map"))
print("  {:>16s} {:>6s} {:>4s}  |  {:>16s} {:>6s} {:>4s}".format(
    "Position", "z_std", "pts", "Position", "z_std", "pts"))
print("-" * 65)
for i in range(10):
    oh = old['hotspots'][i]
    nh = new['hotspots'][i]
    print("  ({:6.1f},{:6.1f}) {:6.4f}m {:4d}  |  ({:6.1f},{:6.1f}) {:6.4f}m {:4d}".format(
        oh['x'], oh['y'], oh['z_std'], oh['count'],
        nh['x'], nh['y'], nh['z_std'], nh['count']))

# ── High risk regions ──
print("\n" + "=" * 80)
print("  High Risk Regions (>30% degradation)")
print("=" * 80)
print("\nOld Map: {} high-risk regions".format(len(old['high_risk'])))
for r in old['high_risk']:
    print("  X=[{:3.0f},{:3.0f}] Y=[{:4.0f},{:4.0f}] -> {:.0f}% degrade ({} cells)".format(
        r['x0'], r['x1'], r['y0'], r['y1'], r['pct'], r['cells']))

print("\nNew Map: {} high-risk regions".format(len(new['high_risk'])))
for r in new['high_risk']:
    print("  X=[{:3.0f},{:3.0f}] Y=[{:4.0f},{:4.0f}] -> {:.0f}% degrade ({} cells)".format(
        r['x0'], r['x1'], r['y0'], r['y1'], r['pct'], r['cells']))

# ── Changes analysis ──
print("\n" + "=" * 80)
print("  Risk Region Changes")
print("=" * 80)

old_risk_set = set((round(r['x0']), round(r['y0'])) for r in old['high_risk'])
new_risk_set = set((round(r['x0']), round(r['y0'])) for r in new['high_risk'])
fixed = old_risk_set - new_risk_set
regressed = new_risk_set - old_risk_set
persistent = old_risk_set & new_risk_set

if fixed:
    print("\nFIXED (old>30% -> new<=30%): {} regions".format(len(fixed)))
    for k in sorted(fixed):
        o = old_reg.get(k)
        n = new_reg.get(k)
        ov = o['pct'] if o else 0
        nv = n['pct'] if n else 0
        print("  X=[{},{}] Y=[{},{}]: {:.0f}% -> {:.0f}%".format(k[0], k[0]+4, k[1], k[1]+4, ov, nv))

if persistent:
    print("\nPERSISTENT (both >30%): {} regions - NEEDS REBUILD".format(len(persistent)))
    for k in sorted(persistent):
        o = old_reg.get(k)
        n = new_reg.get(k)
        ov = o['pct'] if o else 0
        nv = n['pct'] if n else 0
        print("  X=[{},{}] Y=[{},{}]: {:.0f}% -> {:.0f}%".format(k[0], k[0]+4, k[1], k[1]+4, ov, nv))

if regressed:
    print("\nREGRESSED (old<=30% -> new>30%): {} regions".format(len(regressed)))
    for k in sorted(regressed):
        o = old_reg.get(k)
        n = new_reg.get(k)
        ov = o['pct'] if o else 0
        nv = n['pct'] if n else 0
        print("  X=[{},{}] Y=[{},{}]: {:.0f}% -> {:.0f}%".format(k[0], k[0]+4, k[1], k[1]+4, ov, nv))

# ── Conclusions ──
print("\n" + "=" * 80)
print("  CONCLUSIONS")
print("=" * 80)

if new['overall_pct'] < old['overall_pct']:
    print("\n[OK] Overall degradation RATE IMPROVED: {:.1f}% -> {:.1f}% ({:.1f}pp better)".format(
        old['overall_pct'], new['overall_pct'], old['overall_pct']-new['overall_pct']))
elif new['overall_pct'] > old['overall_pct']:
    print("\n[WARN] Overall degradation rate worsened: {:.1f}% -> {:.1f}%".format(
        old['overall_pct'], new['overall_pct']))
else:
    print("\nOverall degradation rate unchanged: {:.1f}%".format(old['overall_pct']))

print("Map coverage: {} points -> {} points ({:.0f}% change)".format(
    old['n_points'], new['n_points'],
    (new['n_points']-old['n_points'])/old['n_points']*100))

# Z variability - critical for NDT
old_z_span = old['z_hi_lo'][1] - old['z_hi_lo'][0]
new_z_span = new['z_hi_lo'][1] - new['z_hi_lo'][0]
print("Z height range (usable): {:.1f}m -> {:.1f}m".format(old_z_span, new_z_span))

print("\nDone.")
