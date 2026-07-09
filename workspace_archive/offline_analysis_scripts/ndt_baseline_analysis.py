#!/usr/bin/env python3
"""NDT baseline statistics from fusion_monitor JSONL logs."""
import json
import sys
import os
from collections import defaultdict
import math

LOG_PATH = os.path.expanduser(
    '~/humanoid_ws/debug_logs/ndt_fusion_monitor_20260527_142701.jsonl')

def load_events(path):
    events = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return events

def analyze(events):
    ndt_events = [e for e in events if e.get('type') == 'ndt_status']
    pcl_poses = [e for e in events if e.get('type') == 'pcl_pose_snapshot']
    robot_poses = [e for e in events if e.get('type') == 'robot_pose_snapshot']
    fusion_states = [e for e in events if e.get('type') == 'fusion_status']

    if not ndt_events:
        print("ERROR: No NDT status events found")
        return

    # ── 1. Overall counts by state ──
    state_counts = defaultdict(int)
    reason_counts = defaultdict(int)
    for e in ndt_events:
        state_counts[e.get('state', '?')] += 1
        reason_counts[e.get('reason', '?')] += 1

    total = len(ndt_events)
    accepted = state_counts.get('accepted', 0)
    rejected = state_counts.get('rejected', 0)
    confirming = state_counts.get('confirming', 0)

    # ── 2. Per-state fitness & correction stats ──
    def stats(arr, key):
        vals = [e[key] for e in arr if e.get(key) is not None]
        if not vals:
            return {}
        vals.sort()
        n = len(vals)
        return {
            'min': min(vals), 'max': max(vals),
            'p50': vals[n//2], 'p90': vals[int(n*0.9)],
            'p95': vals[int(n*0.95)], 'p99': vals[int(n*0.99)],
            'mean': sum(vals)/n, 'count': n,
        }

    for label, subset in [
        ('ALL', ndt_events),
        ('accepted', [e for e in ndt_events if e.get('state') == 'accepted']),
        ('rejected', [e for e in ndt_events if e.get('state') == 'rejected']),
        ('confirming', [e for e in ndt_events if e.get('state') == 'confirming']),
    ]:
        if not subset:
            continue
        print(f"\n{'='*70}")
        print(f"  [{label}] NDT frames: {len(subset)} ({len(subset)/total*100:.1f}%)")
        print(f"{'='*70}")

        fs = stats(subset, 'fitness_score')
        ct = stats(subset, 'correction_translation')
        cy = stats(subset, 'correction_yaw')
        mcd = stats(subset, 'mean_corr_dist')
        fp = stats(subset, 'filtered_points')

        print(f"  {'Metric':<30s} {'min':>8s} {'p50':>8s} {'p90':>8s} {'p95':>8s} {'p99':>8s} {'max':>8s} {'mean':>8s} {'n':>6s}")
        print(f"  {'-'*30} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*6}")

        for name, s in [('fitness_score', fs), ('correction_trans(m)', ct),
                         ('correction_yaw(rad)', cy), ('mean_corr_dist(m)', mcd),
                         ('filtered_points', fp)]:
            if s:
                print(f"  {name:<30s} {s['min']:>8.4f} {s['p50']:>8.4f} {s['p90']:>8.4f} "
                      f"{s['p95']:>8.4f} {s['p99']:>8.4f} {s['max']:>8.4f} {s['mean']:>8.4f} {s['count']:>6d}")

    # ── 3. State transitions / session timeline ──
    print(f"\n{'='*70}")
    print(f"  Session Overview")
    print(f"{'='*70}")

    # Time range
    all_ts = [e['stamp_sec'] for e in ndt_events if 'stamp_sec' in e]
    if all_ts:
        start_t = min(all_ts)
        end_t = max(all_ts)
        duration = end_t - start_t
        print(f"  Time range: {start_t:.0f} -> {end_t:.0f}  (duration: {duration:.0f}s = {duration/60:.1f}min)")
        print(f"  Avg NDT frame rate: {len(ndt_events)/duration:.1f} Hz")

    # Fusion state timeline
    if fusion_states:
        state_timeline = []
        prev_state = None
        for e in fusion_states:
            st = e.get('state')
            if st != prev_state:
                state_timeline.append({'ts': e.get('ts', ''), 'state': st, 'ndt_error': e.get('ndt_error', 0)})
                prev_state = st

        print(f"\n  Fusion state changes ({len(state_timeline)} transitions):")
        for t in state_timeline:
            print(f"    {t['ts']}  -> {t['state']}  (ndt_error={t['ndt_error']:.4f})")

    # Session-level fusion state distribution
    fusion_state_counts = defaultdict(int)
    for e in fusion_states:
        fusion_state_counts[e.get('state', '?')] += 1
    print(f"\n  Fusion state distribution:")
    for st, cnt in sorted(fusion_state_counts.items()):
        print(f"    {st}: {cnt} ({cnt/len(fusion_states)*100:.1f}%)")

    # ── 4. Pose jump analysis ──
    print(f"\n{'='*70}")
    print(f"  Pose Jump & Rejection Analysis")
    print(f"{'='*70}")

    pose_jump_events = [e for e in ndt_events if e.get('reason') == 'pose_jump_candidate']
    confirmed_jumps = [e for e in ndt_events if e.get('reason') == 'confirmed_pose_jump']
    rejected_consecutive = [e for e in ndt_events if e.get('reason') == 'consecutive_rejected']

    print(f"  Pose jump candidates: {len(pose_jump_events)}")
    print(f"  Confirmed pose jumps: {len(confirmed_jumps)}")
    print(f"  Consecutive rejected frames: {len(rejected_consecutive)}")

    if pose_jump_events:
        print(f"\n  Pose jump candidate positions:")
        for e in pose_jump_events[:20]:
            # Find nearest pcl_pose
            t = e.get('stamp_sec', 0)
            pose = min(pcl_poses, key=lambda p: abs(p.get('_seq', 0) - e.get('_seq', 0))) if pcl_poses else {}
            print(f"    t={t:.0f}  corr={e['correction_translation']:.4f}m  "
                  f"yaw={e['correction_yaw']:.4f}  fitness={e['fitness_score']:.4f}  "
                  f"pose=({pose.get('x',0):.1f},{pose.get('y',0):.1f})")

    # ── 5. Mean correlation distance trend ──
    print(f"\n{'='*70}")
    print(f"  NDT-Odom Correlation Distance Analysis")
    print(f"{'='*70}")

    mcd_vals = [e['mean_corr_dist'] for e in ndt_events if e.get('mean_corr_dist', -1) >= 0]
    if mcd_vals:
        mcd_vals.sort()
        n = len(mcd_vals)
        print(f"  mean_corr_dist stats (NDT<->Odom agreement):")
        print(f"    p50: {mcd_vals[n//2]:.4f}m  p90: {mcd_vals[int(n*0.9)]:.4f}m  "
              f"p99: {mcd_vals[int(n*0.99)]:.4f}m  max: {max(mcd_vals):.4f}m")
        high_mcd = sum(1 for v in mcd_vals if v > 1.0)
        print(f"    > 1.0m (warn threshold): {high_mcd}/{n} ({high_mcd/n*100:.1f}%)")

    # ── 6. Fitness score distribution buckets ──
    print(f"\n{'='*70}")
    print(f"  Fitness Score Distribution (score_threshold=0.3)")
    print(f"{'='*70}")

    fitness_vals = [e['fitness_score'] for e in ndt_events if 'fitness_score' in e]
    buckets = [(0, 0.01), (0.01, 0.05), (0.05, 0.10), (0.10, 0.15),
               (0.15, 0.20), (0.20, 0.25), (0.25, 0.30), (0.30, 0.50), (0.50, float('inf'))]
    for lo, hi in buckets:
        cnt = sum(1 for v in fitness_vals if lo <= v < hi)
        label = f"[{lo:.2f}, {hi:.2f})" if hi != float('inf') else f"[{lo:.2f}, inf)"
        bar = '█' * int(cnt / max(fitness_vals) * 60) if fitness_vals else ''
        print(f"  {label:>16s}: {cnt:>6d} ({cnt/len(fitness_vals)*100:5.1f}%) {bar}")

    # ── 7. Correction translation buckets ──
    print(f"\n{'='*70}")
    print(f"  Correction Translation Distribution")
    print(f"{'='*70}")

    corr_vals = [e['correction_translation'] for e in ndt_events if 'correction_translation' in e]
    corr_buckets = [(0, 0.01), (0.01, 0.05), (0.05, 0.10), (0.10, 0.20),
                    (0.20, 0.30), (0.30, 0.40), (0.40, 0.80), (0.80, float('inf'))]
    for lo, hi in corr_buckets:
        cnt = sum(1 for v in corr_vals if lo <= v < hi)
        label = f"[{lo:.2f}, {hi:.2f})" if hi != float('inf') else f"[{hi:.2f}, inf)"
        bar = '█' * int(cnt / max(corr_vals) * 40) if corr_vals else ''
        print(f"  {label:>16s}: {cnt:>6d} ({cnt/len(corr_vals)*100:5.1f}%) {bar}")

    # ── 8. Spatial heatmap (position -> avg fitness) ──
    print(f"\n{'='*70}")
    print(f"  Spatial Fitness Analysis (position -> avg fitness)")
    print(f"{'='*70}")

    # Build (x_bin, y_bin) -> [fitness] map
    spatial = defaultdict(list)
    for e in ndt_events:
        # Find nearest pcl_pose by _seq
        seq = e.get('_seq', 0)
        pose = None
        for p in pcl_poses:
            if abs(p.get('_seq', 0) - seq) < 50:
                pose = p
                break
        if pose:
            bx = int(pose['x'] // 4) * 4
            by = int(pose['y'] // 4) * 4
            spatial[(bx, by)].append(e['fitness_score'])

    if spatial:
        print(f"\n  {'Region (4m grid)':<24s} {'frames':>6s} {'avg_fitness':>10s} {'max_fitness':>10s} {'status'}")
        print(f"  {'-'*24} {'-'*6} {'-'*10} {'-'*10} {'-----'}")
        for (bx, by), fv in sorted(spatial.items(), key=lambda x: max(x[1]), reverse=True):
            avg = sum(fv)/len(fv)
            mx = max(fv)
            # Flag regions with high fitness
            flag = '⚠ HIGH' if mx > 0.3 else ('!' if mx > 0.15 else '')
            print(f"  ({bx:>4d}, {by:>4d})            {len(fv):>6d} {avg:>10.5f} {mx:>10.5f} {flag}")

    # ── 9. Accepted with high fitness — potential stealth drift ──
    print(f"\n{'='*70}")
    print(f"  Stealth Drift Candidates (accepted but fitness > 0.10)")
    print(f"{'='*70}")

    stealth = [e for e in ndt_events
               if e.get('state') == 'accepted' and e.get('fitness_score', 1.0) > 0.10]
    print(f"  Count: {len(stealth)}/{len([e for e in ndt_events if e.get('state')=='accepted'])} "
          f"accepted frames have fitness > 0.10")
    if stealth:
        print(f"\n  Top 10 worst accepted frames:")
        stealth.sort(key=lambda e: e['fitness_score'], reverse=True)
        for e in stealth[:10]:
            seq = e.get('_seq', 0)
            pose = None
            for p in pcl_poses:
                if abs(p.get('_seq', 0) - seq) < 50:
                    pose = p
                    break
            px, py = (pose['x'], pose['y']) if pose else (0, 0)
            print(f"    fitness={e['fitness_score']:.5f}  corr={e['correction_translation']:.4f}m  "
                  f"reason={e.get('reason','?')}  pos=({px:.1f},{py:.1f})")

    # ── 10. Parameter snapshot ──
    print(f"\n{'='*70}")
    print(f"  NDT Parameter Snapshot (from log)")
    print(f"{'='*70}")

    # Find a representative accepted event to print params
    for e in ndt_events:
        if e.get('state') == 'accepted':
            print(f"  ndt_outlier_ratio: {e.get('ndt_outlier_ratio', '?')}")
            print(f"  ndt_max_corr_dist: {e.get('ndt_max_corr_dist', '?')}")
            print(f"  ndt_rotation_prior_weight: {e.get('ndt_rotation_prior_weight', '?')}")
            print(f"  score_threshold: {e.get('score_threshold', '?')}")
            print(f"  max_pose_jump_translation: {e.get('max_pose_jump_translation', '?')}")
            print(f"  max_pose_jump_yaw: {e.get('max_pose_jump_yaw', '?')}")
            print(f"  pose_jump_reacquire_enabled: {e.get('pose_jump_reacquire_enabled', '?')}")
            print(f"  pose_jump_reacquire_max_translation: {e.get('pose_jump_reacquire_max_translation', '?')}")
            print(f"  pose_jump_reacquire_max_yaw: {e.get('pose_jump_reacquire_max_yaw', '?')}")
            print(f"  pose_jump_reacquire_max_fitness: {e.get('pose_jump_reacquire_max_fitness', '?')}")
            print(f"  pose_jump_reacquire_required_frames: {e.get('pose_jump_reacquire_required_frames', '?')}")
            break

    # ── 11. State reason breakdown ──
    print(f"\n{'='*70}")
    print(f"  NDT Frame Reason Breakdown")
    print(f"{'='*70}")
    for reason, cnt in sorted(reason_counts.items(), key=lambda x: -x[1]):
        bar = '█' * int(cnt / total * 50)
        print(f"  {reason:<30s}: {cnt:>6d} ({cnt/total*100:5.1f}%) {bar}")

    # ── 12. Consecutive rejected streaks ──
    print(f"\n{'='*70}")
    print(f"  Rejection Streak Analysis")
    print(f"{'='*70}")
    max_streak = 0
    current_streak = 0
    streaks = []
    for e in ndt_events:
        if e.get('state') == 'rejected':
            current_streak += 1
        else:
            if current_streak > 0:
                streaks.append(current_streak)
                max_streak = max(max_streak, current_streak)
            current_streak = 0
    if current_streak > 0:
        streaks.append(current_streak)

    print(f"  Max consecutive rejected: {max_streak}")
    print(f"  Rejection streaks (len): {sorted(streaks, reverse=True)[:10]}")
    if max_streak >= 5:
        print(f"  ⚠ Long rejection streaks detected — NDT may be losing lock")

    # ── 13. Max correction by state ──
    print(f"\n{'='*70}")
    print(f"  Top 10 Largest Correction Frames")
    print(f"{'='*70}")
    print(f"  {'state':<12s} {'fitness':>8s} {'corr_trans':>10s} {'corr_yaw':>10s} {'reason':<25s}")
    print(f"  {'-'*12} {'-'*8} {'-'*10} {'-'*10} {'-'*25}")
    top_corr = sorted(ndt_events, key=lambda e: e.get('correction_translation', 0), reverse=True)[:10]
    for e in top_corr:
        print(f"  {e.get('state','?'):<12s} {e.get('fitness_score',0):>8.4f} "
              f"{e.get('correction_translation',0):>10.4f} {e.get('correction_yaw',0):>10.4f} "
              f"{e.get('reason','?'):<25s}")

    print(f"\n{'='*70}")
    print(f"  SUMMARY")
    print(f"{'='*70}")
    accept_rate = accepted / total * 100 if total else 0
    avg_fitness = sum(fitness_vals)/len(fitness_vals) if fitness_vals else 0
    print(f"  Total NDT frames: {total}")
    print(f"  Accept rate: {accepted}/{total} = {accept_rate:.1f}%")
    print(f"  Reject rate: {rejected}/{total} = {rejected/total*100:.1f}%")
    print(f"  Confirming rate: {confirming}/{total} = {confirming/total*100:.1f}%")
    print(f"  Overall avg fitness_score: {avg_fitness:.5f}")
    print(f"  Pose jump candidates: {len(pose_jump_events)}")
    print(f"  Confirmed pose jumps: {len(confirmed_jumps)}")
    print(f"  Max consecutive rejected: {max_streak}")

    # Health rating
    if accept_rate > 95 and avg_fitness < 0.05 and len(confirmed_jumps) < 10:
        health = "EXCELLENT"
    elif accept_rate > 90 and avg_fitness < 0.1 and len(confirmed_jumps) < 30:
        health = "GOOD"
    elif accept_rate > 80 and avg_fitness < 0.2:
        health = "FAIR"
    else:
        health = "POOR — needs tuning"
    print(f"\n  Overall NDT Health: {health}")


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else LOG_PATH
    print(f"Analyzing: {path}")
    events = load_events(path)
    print(f"Loaded {len(events)} events")

    # Quick type breakdown
    type_counts = defaultdict(int)
    for e in events:
        type_counts[e.get('type', '?')] += 1
    print(f"Event types: {dict(type_counts)}")

    analyze(events)
