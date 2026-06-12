#!/usr/bin/env python3
import argparse
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


REQUIRED_TOPICS = [
    "/tf",
    "/tf_static",
    "/rosout",
    "/navigation/status",
    "/localization/prior_map_odom_bridge_status",
    "/prior_localization/odom",
    "/prior_localization/confidence",
    "/prior_localization/open3d_input_odom",
    "/odom",
    "/robot_realpose",
    "/robot_speed",
    "/cmd_vel",
    "/navigate_to_pose/_action/status",
    "/navigate_to_pose/_action/feedback",
    "/follow_path/_action/status",
    "/follow_path/_action/feedback",
    "/spin/_action/status",
    "/spin/_action/feedback",
    "/behavior_tree_log",
    "/plan",
]

READ_TOPICS = {
    "/tf",
    "/navigation/status",
    "/localization/prior_map_odom_bridge_status",
    "/odom",
    "/robot_realpose",
    "/cmd_vel",
    "/plan",
    "/behavior_tree_log",
    "/rosout",
}


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def dpose(a, b):
    dx = b["x"] - a["x"]
    dy = b["y"] - a["y"]
    return {
        "dx": dx,
        "dy": dy,
        "dxy": math.hypot(dx, dy),
        "dyaw": wrap(b["yaw"] - a["yaw"]),
    }


def pose_dict(t, x, y, yaw):
    return {"t": float(t), "x": float(x), "y": float(y), "yaw": float(yaw)}


def path_distance(rows):
    total = 0.0
    for a, b in zip(rows, rows[1:]):
        if 0.0 < b["t"] - a["t"] < 1.0:
            total += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    return total


def slice_rows(rows, start, end):
    return [r for r in rows if start <= r["t"] <= end]


def nearest(rows, t):
    if not rows:
        return None
    return min(rows, key=lambda r: abs(r["t"] - t))


def parse_json(text):
    try:
        return json.loads(text)
    except Exception:
        return None


def parse_bridge(text, t):
    parts = text.split()
    event = parts[0] if parts else "UNKNOWN"
    reason = parts[1] if len(parts) > 1 else ""

    def val(name):
        m = re.search(rf"{name}=([-+0-9.eE]+)", text)
        return float(m.group(1)) if m else None

    return {
        "t": t,
        "event": event,
        "reason": reason,
        "text": text,
        "dx_reported": val("dx"),
        "yaw_reported": val("yaw"),
        "map_odom_xy_norm": val("map_odom_xy_norm"),
    }


def read_bag(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = sorted(READ_TOPICS.intersection(type_map))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    msg_types = {topic: get_message(type_map[topic]) for topic in topics}

    data = {
        "type_map": type_map,
        "map_odom": [],
        "odom": [],
        "realpose": [],
        "cmd_vel": [],
        "nav_status": [],
        "bridge": [],
        "plans": [],
        "bt_spin_events": [],
        "rosout_hits": [],
    }
    start = None
    end = None

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns * 1e-9
        start = t if start is None else min(start, t)
        end = t if end is None else max(end, t)
        msg = deserialize_message(raw, msg_types[topic])
        if topic == "/tf":
            for tr in msg.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    q = tr.transform.rotation
                    data["map_odom"].append(pose_dict(
                        t, tr.transform.translation.x, tr.transform.translation.y, yaw_from_q(q)
                    ))
        elif topic == "/odom":
            p = msg.pose.pose.position
            data["odom"].append(pose_dict(t, p.x, p.y, yaw_from_q(msg.pose.pose.orientation)))
        elif topic == "/robot_realpose":
            p = msg.pose.pose.position
            data["realpose"].append(pose_dict(t, p.x, p.y, yaw_from_q(msg.pose.pose.orientation)))
        elif topic == "/cmd_vel":
            data["cmd_vel"].append({
                "t": t,
                "vx": float(msg.linear.x),
                "vy": float(msg.linear.y),
                "wz": float(msg.angular.z),
            })
        elif topic == "/navigation/status":
            obj = parse_json(msg.data)
            if obj:
                pose = obj.get("current_pose", {}).get("position", {})
                data["nav_status"].append({
                    "t": t,
                    "state": str(obj.get("current_state")),
                    "detailed_state": str(obj.get("detailed_state")),
                    "active": bool(obj.get("is_active")),
                    "index": obj.get("current_waypoint_index"),
                    "total": obj.get("total_waypoints"),
                    "progress": obj.get("progress_percentage"),
                    "paused": bool(obj.get("localization_auto_paused")) or str(obj.get("current_state")).lower() == "paused",
                    "pending": bool(obj.get("pending_navigation")),
                    "loc_recovery": bool(obj.get("localization_recovery_active")),
                    "x": pose.get("x"),
                    "y": pose.get("y"),
                    "raw": obj,
                })
        elif topic == "/localization/prior_map_odom_bridge_status":
            data["bridge"].append(parse_bridge(msg.data, t))
        elif topic == "/plan":
            poses = []
            for ps in msg.poses:
                p = ps.pose.position
                poses.append((p.x, p.y))
            length = sum(math.hypot(poses[i][0] - poses[i - 1][0], poses[i][1] - poses[i - 1][1]) for i in range(1, len(poses)))
            data["plans"].append({"t": t, "count": len(poses), "length": length})
        elif topic == "/behavior_tree_log":
            for ev in getattr(msg, "event_log", []):
                node = getattr(ev, "node_name", "")
                if "Spin" in node or "spin" in node:
                    data["bt_spin_events"].append({"t": t, "node": node, "status": getattr(ev, "current_status", "")})
        elif topic == "/rosout":
            text = getattr(msg, "msg", "")
            if any(k in text for k in ("开始导航到路点", "Nav2确认到达路点", "Goal failed", "paused", "暂停", "SpinToPose", "定位异常")):
                data["rosout_hits"].append({"t": t, "text": text})

    data["start"] = start or 0.0
    data["end"] = end or 0.0
    return data


def build_segments(status_rows):
    segments = []
    current = None
    last_key = None
    for r in status_rows:
        key = r["index"]
        active = r["active"]
        if active and (current is None or key != last_key):
            if current is not None:
                current["end"] = r["t"]
                current["result"] = "completed_or_next"
                segments.append(current)
            current = {
                "name": f"点位{key}",
                "index": key,
                "start": r["t"],
                "end": None,
                "result": "running",
                "total": r["total"],
            }
            last_key = key
        elif not active and current is not None:
            current["end"] = r["t"]
            current["result"] = "completed_or_idle"
            segments.append(current)
            current = None
            last_key = None
    if current is not None:
        current["end"] = status_rows[-1]["t"]
        current["result"] = "unfinished_in_bag"
        segments.append(current)
    return segments


def jump_events(rows, min_dxy=0.05, min_dyaw=0.03):
    out = []
    max_ev = None
    for a, b in zip(rows, rows[1:]):
        dt = b["t"] - a["t"]
        if dt <= 0.0 or dt > 0.5:
            continue
        d = dpose(a, b)
        ev = {
            "t": b["t"],
            "before": a,
            "after": b,
            "dt": dt,
            **d,
        }
        if max_ev is None or ev["dxy"] > max_ev["dxy"]:
            max_ev = ev
        if ev["dxy"] >= min_dxy or abs(ev["dyaw"]) >= min_dyaw:
            out.append(ev)
    return out, max_ev


def segment_stats(seg, data):
    start, end = seg["start"], seg["end"]
    in_nav = (start, end)
    full = (max(data["start"], start - 2.0), min(data["end"], end + 3.0))
    mo = slice_rows(data["map_odom"], *in_nav)
    mo_full = slice_rows(data["map_odom"], *full)
    odom = slice_rows(data["odom"], *in_nav)
    real = slice_rows(data["realpose"], *in_nav)
    cmd = slice_rows(data["cmd_vel"], *in_nav)
    bridge = [b for b in data["bridge"] if start <= b["t"] <= end]
    bt_spin = [b for b in data["bt_spin_events"] if full[0] <= b["t"] <= full[1]]
    nav = [n for n in data["nav_status"] if start <= n["t"] <= end]
    jumps, max_jump = jump_events(mo)
    jumps_full, max_jump_full = jump_events(mo_full)
    counts = Counter(b["event"] for b in bridge)
    spin_rej = sum(1 for b in bridge if "spin" in (b["reason"] + " " + b["text"]).lower())
    moving_cmd = [c for c in cmd if math.hypot(c["vx"], c["vy"]) > 0.03 or abs(c["wz"]) > 0.05]
    paused = any(n["paused"] or n["pending"] for n in nav)
    max_cmd_v = max((math.hypot(c["vx"], c["vy"]) for c in cmd), default=0.0)
    max_cmd_w = max((abs(c["wz"]) for c in cmd), default=0.0)
    return {
        **seg,
        "duration": end - start,
        "map_odom_changes": len(jumps),
        "map_odom_medium": sum(1 for j in jumps if j["dxy"] >= 0.25),
        "map_odom_large": sum(1 for j in jumps if j["dxy"] >= 0.50),
        "map_odom_max": max_jump["dxy"] if max_jump else 0.0,
        "map_odom_max_yaw": abs(max_jump["dyaw"]) if max_jump else 0.0,
        "map_odom_full_max": max_jump_full["dxy"] if max_jump_full else 0.0,
        "odom_distance": path_distance(odom),
        "realpose_distance": path_distance(real),
        "accepted": counts.get("ACCEPTED", 0),
        "rejected": counts.get("REJECTED", 0),
        "pending_hold": counts.get("PENDING", 0) + counts.get("HOLD", 0) + counts.get("DEGRADED", 0),
        "spin_rejected": spin_rej,
        "spin_to_pose": bool(bt_spin or spin_rej),
        "paused": paused,
        "cmd_moving_samples": len(moving_cmd),
        "max_cmd_v": max_cmd_v,
        "max_cmd_w": max_cmd_w,
        "drift": (max_jump and max_jump["dxy"] > 0.50) or any((b["dx_reported"] or 0.0) > 0.50 for b in bridge),
    }


def fallback_windows(data, segments):
    windows = []
    bad_events = [b for b in data["bridge"] if b["event"] != "ACCEPTED"]
    cur = []
    for b in bad_events:
        if not cur or b["t"] - cur[-1]["t"] <= 0.6:
            cur.append(b)
        else:
            windows.append(cur)
            cur = [b]
    if cur:
        windows.append(cur)

    out = []
    for win in windows:
        s, e = win[0]["t"], win[-1]["t"]
        if e - s < 1.0:
            continue
        od = slice_rows(data["odom"], s, e)
        cmd = slice_rows(data["cmd_vel"], s, e)
        dist = path_distance(od)
        max_v = max((math.hypot(c["vx"], c["vy"]) for c in cmd), default=0.0)
        max_w = max((abs(c["wz"]) for c in cmd), default=0.0)
        active = any(n["active"] for n in data["nav_status"] if s <= n["t"] <= e)
        if active and dist > 0.05 and (max_v > 0.03 or max_w > 0.05):
            seg = next((x for x in segments if x["start"] <= s <= x["end"]), None)
            out.append({
                "start": s,
                "end": e,
                "duration": e - s,
                "odom_distance": dist,
                "max_cmd_v": max_v,
                "max_cmd_w": max_w,
                "events": dict(Counter(b["event"] for b in win)),
                "segment": seg["name"] if seg else "unknown",
            })
    return out


def shadow_summary(data):
    blocked = []
    hold_segments = []
    cur = []
    for b in data["bridge"]:
        dx = abs(b["dx_reported"] or 0.0)
        should_hold = dx > 0.50 or b["event"] in ("HOLD", "DEGRADED")
        if should_hold:
            blocked.append(b)
            cur.append(b)
        else:
            if cur:
                hold_segments.append(cur)
                cur = []
    if cur:
        hold_segments.append(cur)
    return {
        "blocked_candidates": len(blocked),
        "hold_segments": [
            {
                "start": s[0]["t"],
                "end": s[-1]["t"],
                "duration": s[-1]["t"] - s[0]["t"],
                "max_dx": max(abs(x["dx_reported"] or 0.0) for x in s),
            }
            for s in hold_segments
            if s[-1]["t"] - s[0]["t"] > 0.1
        ],
    }


def fmt_pose(p):
    return f"({p['x']:.3f}, {p['y']:.3f}, {p['yaw']:.3f})"


def write_outputs(name, bag, out_dir, data):
    out_dir.mkdir(parents=True, exist_ok=True)
    present = set(data["type_map"])
    missing_required = [t for t in REQUIRED_TOPICS if t not in present]
    bridge_counts = Counter(b["event"] for b in data["bridge"])
    bridge_reason_counts = Counter(b["reason"] for b in data["bridge"])
    segments = [segment_stats(s, data) for s in build_segments(data["nav_status"])]
    fallback = fallback_windows(data, segments)
    mo_jumps, mo_max = jump_events(data["map_odom"])
    big_events = [j for j in mo_jumps if j["dxy"] >= 0.25 or abs(j["dyaw"]) >= 0.15]
    big_events = sorted(big_events, key=lambda x: x["dxy"], reverse=True)[:30]
    shadow = shadow_summary(data)

    with (out_dir / "segment_summary.csv").open("w", newline="", encoding="utf-8") as f:
        fields = [
            "name", "start", "end", "duration", "result", "spin_to_pose", "map_odom_max",
            "map_odom_full_max", "odom_distance", "realpose_distance", "accepted", "rejected",
            "pending_hold", "spin_rejected", "drift", "paused", "max_cmd_v", "max_cmd_w",
        ]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for s in segments:
            w.writerow({k: s.get(k) for k in fields})

    with (out_dir / "major_map_odom_events.csv").open("w", newline="", encoding="utf-8") as f:
        fields = ["t", "before", "after", "dx", "dy", "dxy", "dyaw"]
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for e in big_events:
            w.writerow({
                "t": f"{e['t']:.6f}",
                "before": fmt_pose(e["before"]),
                "after": fmt_pose(e["after"]),
                "dx": f"{e['dx']:.4f}",
                "dy": f"{e['dy']:.4f}",
                "dxy": f"{e['dxy']:.4f}",
                "dyaw": f"{e['dyaw']:.4f}",
            })

    summary = {
        "bag": str(bag),
        "duration": data["end"] - data["start"],
        "message_topics_present": sorted(present),
        "missing_required_topics": missing_required,
        "counts": {
            "map_odom": len(data["map_odom"]),
            "odom": len(data["odom"]),
            "robot_realpose": len(data["realpose"]),
            "cmd_vel": len(data["cmd_vel"]),
            "nav_status": len(data["nav_status"]),
            "bridge": len(data["bridge"]),
            "bt_spin_events": len(data["bt_spin_events"]),
        },
        "bridge_counts": dict(bridge_counts),
        "bridge_top_reasons": dict(bridge_reason_counts.most_common(20)),
        "map_odom_max_jump": mo_max,
        "segments": segments,
        "fallback_windows": fallback,
        "shadow": shadow,
    }
    (out_dir / "analysis_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    completed = sum(1 for s in segments if s["result"] != "unfinished_in_bag")
    max_seg = max(segments, key=lambda s: s["map_odom_max"], default=None)
    max_jump_text = "无"
    if mo_max:
        max_jump_text = f"{mo_max['t']:.3f} dxy={mo_max['dxy']:.3f}m dyaw={mo_max['dyaw']:.3f}rad"

    md = [
        f"# {name} 导航 bag 定位漂移分析",
        "",
        "## 1. 总体结论",
        "",
        f"- bag 时长：{data['end'] - data['start']:.1f}s。",
        f"- 从 `/navigation/status` 识别到 {len(segments)} 个导航点位段，完成/退出 active 的点位段 {completed} 个。",
        f"- 最大 `map->odom` 单步跳变：{max_jump_text}。",
        f"- bridge 事件统计：`{dict(bridge_counts)}`。",
        f"- odom 兜底窗口：{len(fallback)} 段。",
        "- prior 候选完整位姿话题未录到，因此候选位姿只能用 bridge 文本摘要和 TF 实际变化间接判断。",
        "",
        "## 2. 数据来源和限制",
        "",
        f"- 输入 bag：`{bag}`",
        f"- 缺失的必须话题：`{missing_required}`",
        f"- 已生成轨迹图：`/home/ubuntu/software/Todesk/Files/humanoid_ws/debug_logs/nav_drift_trajectories/{name}/full_trajectory_on_map.png`",
        "",
        "## 3. 真实导航状态",
        "",
        "| 点位 | 时间段 | 耗时s | 结果 | SpinToPose | 最大map->odom跳变m | 全段最大跳变m | odom位移m | realpose位移m | accepted | rejected | pending/hold | spin冻结拒绝 | 漂移 | odom兜底 | 暂停 |",
        "|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|",
    ]
    fallback_by_seg = Counter(f["segment"] for f in fallback)
    for s in segments:
        md.append(
            f"| {s['name']} | {s['start']:.3f}-{s['end']:.3f} | {s['duration']:.1f} | {s['result']} | "
            f"{'是' if s['spin_to_pose'] else '否'} | {s['map_odom_max']:.3f} | {s['map_odom_full_max']:.3f} | "
            f"{s['odom_distance']:.2f} | {s['realpose_distance']:.2f} | {s['accepted']} | {s['rejected']} | "
            f"{s['pending_hold']} | {s['spin_rejected']} | {'是' if s['drift'] else '否'} | "
            f"{'是' if fallback_by_seg.get(s['name'], 0) else '否'} | {'是' if s['paused'] else '否'} |"
        )

    md.extend([
        "",
        "## 4. 每个点位路线定位漂移情况",
        "",
    ])
    for s in segments:
        judgment = []
        if s["map_odom_max"] > 0.50:
            judgment.append("存在 >0.50m 大修正/漂移")
        elif s["map_odom_max"] > 0.25:
            judgment.append("存在 0.25~0.50m 中等修正")
        else:
            judgment.append("未见明显单步大跳")
        if s["spin_to_pose"]:
            judgment.append("包含 SpinToPose/旋转冻结相关事件")
        if fallback_by_seg.get(s["name"], 0):
            judgment.append("存在 odom 兜底窗口")
        md.append(f"- {s['name']}：{'; '.join(judgment)}。")

    md.extend([
        "",
        "## 5. 重点漂移/跳变事件",
        "",
        "| 时间 | 类型 | 跳变前 | 跳变后 | dx | dy | dxy | dyaw | 是否影响导航 |",
        "|---:|---|---|---|---:|---:|---:|---:|---|",
    ])
    if not big_events:
        md.append("| - | - | - | - | - | - | - | - | 未发现 >=0.25m 的重点 map->odom 单步跳变 |")
    for e in big_events[:20]:
        seg = next((s for s in segments if s["start"] <= e["t"] <= s["end"]), None)
        md.append(
            f"| {e['t']:.3f} | map->odom | {fmt_pose(e['before'])} | {fmt_pose(e['after'])} | "
            f"{e['dx']:.3f} | {e['dy']:.3f} | {e['dxy']:.3f} | {e['dyaw']:.3f} | "
            f"{seg['name'] if seg else '非导航段/未知'} |"
        )

    md.extend([
        "",
        "## 6. bridge 接受/拒绝/冻结情况",
        "",
        f"- 事件统计：`{dict(bridge_counts)}`",
        f"- Top reasons：`{dict(bridge_reason_counts.most_common(20))}`",
        "- 因缺少 `/prior_localization/odom`、`/prior_localization/confidence`、`/prior_localization/open3d_input_odom`，无法直接列完整候选位姿和 confidence，只能列 bridge 文本中的摘要字段。",
        "",
        "## 7. odom 兜底和导航暂停判断",
        "",
    ])
    if fallback:
        md.append("| 点位 | 开始 | 结束 | 持续s | odom位移m | 最大线速度 | 最大角速度 | 事件 |")
        md.append("|---|---:|---:|---:|---:|---:|---:|---|")
        for f in fallback[:30]:
            md.append(
                f"| {f['segment']} | {f['start']:.3f} | {f['end']:.3f} | {f['duration']:.1f} | "
                f"{f['odom_distance']:.2f} | {f['max_cmd_v']:.2f} | {f['max_cmd_w']:.2f} | `{f['events']}` |"
            )
    else:
        md.append("- 未识别到满足“bridge 未接受 + odom 移动 + cmd_vel 有运动 + 导航 active”的 odom 兜底窗口。")
    md.append("")
    md.append(f"- 点位暂停标记：{sum(1 for s in segments if s['paused'])} 个点位段出现 paused/pending/localization_auto_paused 标记。")

    md.extend([
        "",
        "## 8. SpinToPose 旋转保护分析",
        "",
        f"- BehaviorTree Spin 事件数：{len(data['bt_spin_events'])}。",
        f"- bridge 文本中包含 spin 的拒绝/冻结事件数：{sum(1 for b in data['bridge'] if 'spin' in (b['text'] + b['reason']).lower())}。",
        "- 判断：SpinToPose 已纳入统计；若表中 spin冻结拒绝较多但没有大跳/暂停，按旋转保护处理，不单独判为定位失败。",
        "",
        "## 9. 保护策略 shadow 模拟效果",
        "",
        f"- 离线 shadow 规则下会拦截 `dx>0.50m` 或已有 HOLD/DEGRADED 的候选数：{shadow['blocked_candidates']}。",
        f"- hold 聚类段数：{len(shadow['hold_segments'])}。",
        "- 这是离线 shadow，不是真正 Nav2 闭环重跑。",
        "",
        "## 10. 风险判断",
        "",
    ])
    if max_seg and max_seg["map_odom_max"] > 0.50:
        md.append(f"- 最高风险点位：{max_seg['name']}，最大 map->odom 跳变 {max_seg['map_odom_max']:.3f}m。")
    else:
        md.append("- 未见 >0.50m 的 map->odom 单步大跳，按记录话题看没有明显定位漂移。")
    if missing_required:
        md.append("- 风险：缺少 prior 候选和 confidence 话题，无法证明候选从未离谱，只能证明 bridge/TF 输出侧没有相应大跳。")

    md.extend([
        "",
        "## 11. 后续建议",
        "",
        "- 后续录包建议补上 `/prior_localization/odom`、`/prior_localization/confidence`、`/prior_localization/open3d_input_odom`，否则候选定位是否离谱无法精确复盘。",
        "- 如果要证明保护策略对 Nav2 闭环一定有效，需要过滤旧 TF 后重跑实验 bridge 和 Nav2；本报告不是闭环 Nav2 重跑。",
        "",
        "## 12. 生成的中间文件",
        "",
        f"- `{out_dir / 'analysis_summary.json'}`",
        f"- `{out_dir / 'segment_summary.csv'}`",
        f"- `{out_dir / 'major_map_odom_events.csv'}`",
        f"- `/home/ubuntu/software/Todesk/Files/humanoid_ws/debug_logs/nav_drift_trajectories/{name}/full_trajectory_on_map.png`",
        f"- `/home/ubuntu/software/Todesk/Files/humanoid_ws/debug_logs/nav_drift_trajectories/{name}/map_odom_correction_over_time.png`",
    ])
    report = out_dir / f"{name}_protection_analysis_cn.md"
    report.write_text("\n".join(md) + "\n", encoding="utf-8")
    print(report)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--name", required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()
    bag = Path(args.bag)
    data = read_bag(bag)
    write_outputs(args.name, bag, Path(args.out_dir), data)


if __name__ == "__main__":
    main()
