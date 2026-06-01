#!/usr/bin/env python3
"""
离线验证 prior_map_odom_bridge 的大修正保护策略。

这个脚本不启动 ROS 节点，也不修改真实代码。它只读取 bag 中已经录下来的：
- /tf 里的实际 map->odom 变化，作为“外部定位候选序列”的近似输入；
- debug_output.txt 里的导航点位开始/完成时间；

然后用一套影子策略重新决定哪些候选会被接受、哪些会在导航中 pending/拒绝、
哪些会在空闲/讲解窗口被允许找回。
"""

import argparse
import json
import math
from pathlib import Path

from analyze_navtest15 import angle_diff, read_bag, read_log, tf_jumps


def pose_delta(a, b):
    xy = math.hypot(a[1] - b[1], a[2] - b[2])
    yaw = abs(angle_diff(a[3], b[3]))
    return xy, yaw


def in_any_interval(t, intervals):
    return any(start <= t <= end for start, end in intervals)


def interval_label(t, segments):
    for seg in segments:
        if seg["start"] <= t <= seg["done"]:
            return seg["point"]
    return "idle"


def simulate(map_odom, segments, args):
    nav_intervals = [(s["start"], s["done"]) for s in segments]
    if not map_odom:
        return {"events": [], "shadow": []}

    current = map_odom[0]
    shadow = [current]
    events = []
    pending = None
    pending_count = 0
    first_pending_time = None

    for candidate in map_odom[1:]:
        t = candidate[0]
        xy, yaw = pose_delta(candidate, current)
        navigating = in_any_interval(t, nav_intervals)
        phase = "nav" if navigating else "idle"
        point = interval_label(t, segments)

        def accept(reason):
            nonlocal current, pending, pending_count, first_pending_time
            before = current
            current = candidate
            shadow.append(candidate)
            events.append({
                "t": t,
                "point": point,
                "phase": phase,
                "action": "accept",
                "reason": reason,
                "dx": xy,
                "dyaw": yaw,
                "from": before,
                "to": candidate,
                "pending_age": (t - first_pending_time) if first_pending_time else 0.0,
            })
            pending = None
            pending_count = 0
            first_pending_time = None

        def hold(reason):
            nonlocal pending, pending_count, first_pending_time
            if pending is None:
                pending = candidate
                pending_count = 1
                first_pending_time = t
            else:
                spread_xy, spread_yaw = pose_delta(candidate, pending)
                if spread_xy <= args.stable_xy and spread_yaw <= args.stable_yaw:
                    pending_count += 1
                    pending = candidate
                else:
                    pending = candidate
                    pending_count = 1
                    first_pending_time = t
            events.append({
                "t": t,
                "point": point,
                "phase": phase,
                "action": "hold",
                "reason": reason,
                "dx": xy,
                "dyaw": yaw,
                "pending_count": pending_count,
                "from": current,
                "to": candidate,
            })

        if xy <= args.small_xy and yaw <= args.small_yaw:
            accept("small")
            continue

        if navigating:
            if xy <= args.medium_xy and yaw <= args.medium_yaw:
                hold("nav_medium_pending")
                if pending_count >= args.nav_medium_frames:
                    accept("nav_medium_stable")
                continue
            hold("nav_large_blocked")
            continue

        # 空闲/讲解窗口：允许大修正找回，但也要求连续稳定。
        hold("idle_large_pending")
        if pending_count >= args.idle_large_frames:
            accept("idle_large_stable")

    return {"events": events, "shadow": shadow}


def summarize_by_segment(original_map_odom, shadow, segments):
    rows = []
    for seg in segments:
        start = seg["start"]
        done = seg["done"]
        end = done + 6.0
        original_samples = [s for s in original_map_odom if start - 0.5 <= s[0] <= end]
        shadow_samples = [s for s in shadow if start - 0.5 <= s[0] <= end]
        original_jumps = tf_jumps(original_samples)
        shadow_jumps = tf_jumps(shadow_samples)
        rows.append({
            "point": seg["point"],
            "start": start,
            "done": done,
            "original_max_dx": max([j["dx"] for j in original_jumps] or [0.0]),
            "shadow_max_dx": max([j["dx"] for j in shadow_jumps] or [0.0]),
            "original_big_count": len([j for j in original_jumps if j["dx"] >= 0.5]),
            "shadow_big_count": len([j for j in shadow_jumps if j["dx"] >= 0.5]),
        })
    return rows


def render(report):
    lines = []
    lines.append("# Shadow Bridge Policy Report")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append(f"- original max jump: {report['original_max_dx']:.3f} m")
    lines.append(f"- shadow max jump: {report['shadow_max_dx']:.3f} m")
    lines.append(f"- held candidates: {report['held_count']}")
    lines.append(f"- accepted candidates: {report['accepted_count']}")
    lines.append(f"- idle large accepts: {report['idle_large_accept_count']}")
    lines.append(f"- nav large accepts: {report['nav_large_accept_count']}")
    lines.append("")
    lines.append("## Per Point")
    lines.append("")
    lines.append("| point | original max dx | shadow max dx | original >=0.5m | shadow >=0.5m |")
    lines.append("|---|---:|---:|---:|---:|")
    for row in report["segments"]:
        lines.append(
            f"| {row['point']} | {row['original_max_dx']:.3f} | {row['shadow_max_dx']:.3f} | "
            f"{row['original_big_count']} | {row['shadow_big_count']} |"
        )
    lines.append("")
    lines.append("## Held Large Candidates")
    for e in report["notable_holds"]:
        lines.append(
            f"- {e['t']:.3f} {e['point']} {e['phase']} {e['reason']} "
            f"dx={e['dx']:.3f} yaw={e['dyaw']:.3f} pending={e.get('pending_count', 0)}"
        )
    lines.append("")
    lines.append("## Idle Large Accepts")
    for e in report["idle_large_accepts"]:
        lines.append(
            f"- {e['t']:.3f} {e['point']} dx={e['dx']:.3f} yaw={e['dyaw']:.3f} "
            f"pending_age={e['pending_age']:.2f}s"
        )
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--log", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--small-xy", type=float, default=0.30)
    parser.add_argument("--small-yaw", type=float, default=0.08)
    parser.add_argument("--medium-xy", type=float, default=0.50)
    parser.add_argument("--medium-yaw", type=float, default=0.12)
    parser.add_argument("--stable-xy", type=float, default=0.20)
    parser.add_argument("--stable-yaw", type=float, default=0.05)
    parser.add_argument("--nav-medium-frames", type=int, default=6)
    parser.add_argument("--idle-large-frames", type=int, default=3)
    args = parser.parse_args()

    _, segments = read_log(Path(args.log))
    bag = read_bag(args.bag)
    original = bag["map_odom"]
    sim = simulate(original, segments, args)
    original_jumps = tf_jumps(original)
    shadow_jumps = tf_jumps(sim["shadow"])
    events = sim["events"]
    report = {
        "params": vars(args),
        "original_max_dx": max([j["dx"] for j in original_jumps] or [0.0]),
        "shadow_max_dx": max([j["dx"] for j in shadow_jumps] or [0.0]),
        "held_count": len([e for e in events if e["action"] == "hold"]),
        "accepted_count": len([e for e in events if e["action"] == "accept"]),
        "idle_large_accept_count": len([e for e in events if e["action"] == "accept" and e["reason"] == "idle_large_stable"]),
        "nav_large_accept_count": len([e for e in events if e["action"] == "accept" and e["phase"] == "nav" and e["dx"] > args.medium_xy]),
        "segments": summarize_by_segment(original, sim["shadow"], segments),
        "notable_holds": [e for e in events if e["action"] == "hold" and e["dx"] >= 0.5][:200],
        "idle_large_accepts": [e for e in events if e["action"] == "accept" and e["reason"] == "idle_large_stable"],
        "events": events,
    }

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "shadow_policy.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    (out_dir / "shadow_policy.md").write_text(render(report), encoding="utf-8")
    print(f"wrote {out_dir / 'shadow_policy.json'}")
    print(f"wrote {out_dir / 'shadow_policy.md'}")


if __name__ == "__main__":
    main()
