#!/usr/bin/env python3
import argparse
import json
import math
from bisect import bisect_left
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/cmd_vel",
    "/odom",
    "/robot_realpose",
    "/tf",
    "/localization/prior_map_odom_bridge_status",
    "/rosout",
    "/local_costmap/costmap",
    "/navigation/status",
}


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def dist(rows):
    total = 0.0
    for a, b in zip(rows, rows[1:]):
        if 0.0 < b["t"] - a["t"] < 1.5:
            total += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    return total


def nearest(rows, t):
    if not rows:
        return None
    times = [r["t"] for r in rows]
    idx = bisect_left(times, t)
    if idx <= 0:
        return rows[0]
    if idx >= len(rows):
        return rows[-1]
    return rows[idx] if abs(rows[idx]["t"] - t) < abs(rows[idx - 1]["t"] - t) else rows[idx - 1]


def group_bool(rows, pred, max_gap=0.45):
    groups = []
    cur = None
    for r in rows:
        ok = pred(r)
        if not ok:
            if cur is not None:
                cur["end"] = cur["last_t"]
                groups.append(cur)
                cur = None
            continue
        if cur is None or r["t"] - cur["last_t"] > max_gap:
            if cur is not None:
                cur["end"] = cur["last_t"]
                groups.append(cur)
            cur = {"start": r["t"], "last_t": r["t"], "rows": [r]}
        else:
            cur["last_t"] = r["t"]
            cur["rows"].append(r)
    if cur is not None:
        cur["end"] = cur["last_t"]
        groups.append(cur)
    return groups


def pose_from_odom(msg, t):
    p = msg.pose.pose.position
    return {"t": t, "x": p.x, "y": p.y, "yaw": yaw_from_q(msg.pose.pose.orientation)}


def parse_bridge(text, t):
    parts = text.split()
    event = parts[0] if parts else "UNKNOWN"
    reason = parts[1] if len(parts) > 1 else ""

    def val(name):
        marker = f"{name}="
        for item in parts:
            if item.startswith(marker):
                try:
                    return float(item[len(marker):])
                except ValueError:
                    return None
        return None

    return {"t": t, "event": event, "reason": reason, "dx": val("dx"), "yaw": val("yaw"), "text": text}


def map_odom_pose(tr, t):
    q = tr.transform.rotation
    return {
        "t": t,
        "x": tr.transform.translation.x,
        "y": tr.transform.translation.y,
        "yaw": yaw_from_q(q),
    }


def jump_rows(rows):
    out = []
    for a, b in zip(rows, rows[1:]):
        dt = b["t"] - a["t"]
        if dt <= 0.0 or dt > 0.5:
            continue
        dx = b["x"] - a["x"]
        dy = b["y"] - a["y"]
        dyaw = wrap(b["yaw"] - a["yaw"])
        out.append({"t": b["t"], "dx": dx, "dy": dy, "dxy": math.hypot(dx, dy), "dyaw": dyaw})
    return out


def costmap_front_stats(msg, t, odom_rows, realpose_rows):
    frame = msg.header.frame_id
    # In these bags local_costmap is published as odom_ground, but its XY values
    # align with the navigation/global pose stream rather than raw /odom.
    pose_source = odom_rows if frame == "odom" else realpose_rows
    pose = nearest(pose_source, t)
    if pose is None:
        return {"t": t, "frame": frame, "available": False}

    info = msg.info
    ox = info.origin.position.x
    oy = info.origin.position.y
    oyaw = yaw_from_q(info.origin.orientation)
    cos_o = math.cos(oyaw)
    sin_o = math.sin(oyaw)
    cos_r = math.cos(-pose["yaw"])
    sin_r = math.sin(-pose["yaw"])
    occupied = 0
    lethal = 0
    unknown = 0
    sampled = 0
    max_cost = -1
    front_min_x = 0.10
    front_max_x = 1.40
    front_half_y = 0.50
    data = msg.data
    for y in range(info.height):
        base = y * info.width
        cy = (y + 0.5) * info.resolution
        for x in range(info.width):
            cost = int(data[base + x])
            cx = (x + 0.5) * info.resolution
            wx = ox + cos_o * cx - sin_o * cy
            wy = oy + sin_o * cx + cos_o * cy
            dx = wx - pose["x"]
            dy = wy - pose["y"]
            rx = cos_r * dx - sin_r * dy
            ry = sin_r * dx + cos_r * dy
            if front_min_x <= rx <= front_max_x and abs(ry) <= front_half_y:
                sampled += 1
                max_cost = max(max_cost, cost)
                if cost < 0:
                    unknown += 1
                elif cost >= 90:
                    lethal += 1
                    occupied += 1
                elif cost >= 50:
                    occupied += 1
    return {
        "t": t,
        "frame": frame,
        "available": True,
        "sampled": sampled,
        "occupied": occupied,
        "lethal": lethal,
        "unknown": unknown,
        "max_cost": max_cost,
        "pose_x": pose["x"],
        "pose_y": pose["y"],
        "pose_yaw": pose["yaw"],
    }


def read_bag(bag, start, end):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    topics = sorted(TOPICS.intersection(type_map))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    msg_types = {topic: get_message(type_map[topic]) for topic in topics}
    rows = {k: [] for k in ["cmd", "odom", "realpose", "map_odom", "bridge", "rosout", "costmap", "nav_status"]}
    costmap_msgs = []
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns * 1e-9
        if t < start - 2.0:
            continue
        if t > end + 2.0:
            continue
        msg = deserialize_message(raw, msg_types[topic])
        if topic == "/cmd_vel":
            rows["cmd"].append({"t": t, "vx": msg.linear.x, "vy": msg.linear.y, "wz": msg.angular.z})
        elif topic == "/odom":
            rows["odom"].append(pose_from_odom(msg, t))
        elif topic == "/robot_realpose":
            rows["realpose"].append(pose_from_odom(msg, t))
        elif topic == "/tf":
            for tr in msg.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    rows["map_odom"].append(map_odom_pose(tr, t))
        elif topic == "/localization/prior_map_odom_bridge_status":
            rows["bridge"].append(parse_bridge(msg.data, t))
        elif topic == "/rosout":
            text = getattr(msg, "msg", "")
            keys = (
                "collision ahead",
                "Controller patience exceeded",
                "Running wait",
                "wait completed",
                "Passing new path",
                "Reached the goal",
                "navigation_obstacle_blocked",
                "机器人阻塞后已恢复运动",
                "Nav2确认到达路点",
                "开始导航到路点",
                "Goal succeeded",
            )
            if any(k in text for k in keys):
                rows["rosout"].append({"t": t, "text": text})
        elif topic == "/local_costmap/costmap":
            costmap_msgs.append((t, msg))
        elif topic == "/navigation/status":
            try:
                obj = json.loads(msg.data)
            except Exception:
                obj = {}
            rows["nav_status"].append({
                "t": t,
                "active": bool(obj.get("is_active")),
                "state": str(obj.get("current_state")),
                "detail": str(obj.get("detailed_state")),
                "paused": bool(obj.get("localization_auto_paused")) or str(obj.get("current_state")).lower() == "paused",
            })
    rows["costmap"] = [costmap_front_stats(msg, t, rows["odom"], rows["realpose"]) for t, msg in costmap_msgs]
    return rows, sorted(type_map)


def render(args, rows, topics):
    start = args.start
    end = args.end
    focus_start = args.focus_start
    focus_end = args.focus_end
    in_win = lambda r: start <= r["t"] <= end
    in_focus = lambda r: focus_start <= r["t"] <= focus_end
    cmd = [r for r in rows["cmd"] if in_win(r)]
    cmd_focus = [r for r in rows["cmd"] if in_focus(r)]
    nonzero = group_bool(cmd, lambda r: math.hypot(r["vx"], r["vy"]) > 0.03 or abs(r["wz"]) > 0.05)
    forward = group_bool(cmd, lambda r: r["vx"] > 0.03)
    zero = group_bool(cmd, lambda r: math.hypot(r["vx"], r["vy"]) <= 0.01 and abs(r["wz"]) <= 0.01)
    odom = [r for r in rows["odom"] if in_win(r)]
    realpose = [r for r in rows["realpose"] if in_win(r)]
    odom_focus = [r for r in rows["odom"] if in_focus(r)]
    real_focus = [r for r in rows["realpose"] if in_focus(r)]
    jumps = [j for j in jump_rows([r for r in rows["map_odom"] if in_win(r)]) if j["dxy"] >= 0.02 or abs(j["dyaw"]) >= 0.02]
    bridge = [r for r in rows["bridge"] if in_win(r)]
    cost = [r for r in rows["costmap"] if in_win(r) and r.get("available")]
    cost_focus = [r for r in rows["costmap"] if in_focus(r) and r.get("available")]
    rosout = [r for r in rows["rosout"] if in_win(r)]
    nav = [r for r in rows["nav_status"] if in_win(r)]
    active_samples = sum(1 for r in nav if r["active"])
    paused_samples = sum(1 for r in nav if r["paused"])
    max_v = max((math.hypot(r["vx"], r["vy"]) for r in cmd), default=0.0)
    max_v_focus = max((math.hypot(r["vx"], r["vy"]) for r in cmd_focus), default=0.0)
    max_forward_focus = max((r["vx"] for r in cmd_focus), default=0.0)
    max_w_focus = max((abs(r["wz"]) for r in cmd_focus), default=0.0)
    max_jump = max(jumps, key=lambda r: r["dxy"], default=None)
    cost_occ = [r for r in cost_focus if r["occupied"] > 0 or r["lethal"] > 0]
    cost_clear = [r for r in cost_focus if r["occupied"] == 0 and r["lethal"] == 0 and r["sampled"] > 0]
    max_cost = max((r["max_cost"] for r in cost_focus), default=-1)
    max_lethal = max((r["lethal"] for r in cost_focus), default=0)
    lines = [
        "# nav_drift_test33 第25点障碍窗口细查",
        "",
        "## 结论",
        "",
        f"- 分析窗口：`{start:.3f}-{end:.3f}`；重点障碍窗口：`{focus_start:.3f}-{focus_end:.3f}`。",
        f"- `/navigation/status` 在窗口内 active 样本 {active_samples}/{len(nav)}，paused 样本 {paused_samples}；日志显示 `navigation_obstacle_blocked` 后 Nav2 goal 没有被取消，BT 继续执行 wait recovery 并重试 FollowPath。",
        f"- 重点窗口内 `/cmd_vel` 最大平面速度 `{max_v_focus:.3f} m/s`，最大正向 vx `{max_forward_focus:.3f} m/s`，最大角速度 `{max_w_focus:.3f} rad/s`；存在多段非零速度，不是一直停住。",
        f"- 窗口内 `/odom` 位移 `{dist(odom):.2f} m`，`/robot_realpose` 位移 `{dist(realpose):.2f} m`；重点障碍窗口内 `/odom` 位移 `{dist(odom_focus):.2f} m`，`/robot_realpose` 位移 `{dist(real_focus):.2f} m`。",
        f"- local costmap 前方检测窗口样本 {len(cost_focus)} 帧，其中前方有 occupied/lethal 的帧 {len(cost_occ)}，前方完全 clear 的帧 {len(cost_clear)}，最大 cost {max_cost}，最大 lethal cell 数 {max_lethal}。",
        f"- bridge 事件统计：`{dict(Counter(r['event'] for r in bridge))}`；窗口内最大 `map->odom` 跳变 `{(max_jump or {'dxy': 0.0})['dxy']:.3f} m`。",
        "",
        "## 解释",
        "",
        "- 这次“先停、播报障碍、随后又走”的直接机制是 Nav2 BT 的恢复行为：RPP 连续检测 collision 后 FollowPath abort，BT 进入 wait；等待结束后重新计算/跟踪路径，状态管理器只上报 `navigation_obstacle_blocked`，没有取消 Nav2 当前 goal。",
        "- 从定位侧看，第 25 点窗口没有 REJECTED/HOLD，也没有 `map->odom` 大跳，因此不像是定位漂移导致机器人忽略障碍。",
        "- 从 costmap 侧看，重点窗口里如果出现前方 clear 帧，就说明局部代价地图在某些重试时刻已经不再认为正前方被占用；这会让 RPP 重新输出速度。该结论依赖 local costmap 的栅格坐标和 `/odom` 位姿反算，不能替代真实闭环复现。",
        "",
        "## `/cmd_vel` 非零段",
        "",
        "| start | end | duration | max_v | max_vx | max_wz | samples |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for g in nonzero[:40]:
        gr = g["rows"]
        lines.append(
            f"| {g['start']:.3f} | {g['end']:.3f} | {g['end'] - g['start']:.2f} | "
            f"{max(math.hypot(r['vx'], r['vy']) for r in gr):.3f} | {max(r['vx'] for r in gr):.3f} | "
            f"{max(abs(r['wz']) for r in gr):.3f} | {len(gr)} |"
        )
    lines.extend([
        "",
        "## `/cmd_vel` 正向 vx 段",
        "",
        "| start | end | duration | max_vx | samples |",
        "|---:|---:|---:|---:|---:|",
    ])
    for g in forward[:40]:
        gr = g["rows"]
        lines.append(f"| {g['start']:.3f} | {g['end']:.3f} | {g['end'] - g['start']:.2f} | {max(r['vx'] for r in gr):.3f} | {len(gr)} |")
    lines.extend([
        "",
        "## `/cmd_vel` 归零段",
        "",
        "| start | end | duration | samples |",
        "|---:|---:|---:|---:|",
    ])
    for g in zero[:40]:
        lines.append(f"| {g['start']:.3f} | {g['end']:.3f} | {g['end'] - g['start']:.2f} | {len(g['rows'])} |")
    lines.extend([
        "",
        "## local costmap 前方窗口",
        "",
        "| t | frame | sampled | occupied | lethal | unknown | max_cost |",
        "|---:|---|---:|---:|---:|---:|---:|",
    ])
    for r in cost_focus[:80]:
        lines.append(f"| {r['t']:.3f} | {r['frame']} | {r['sampled']} | {r['occupied']} | {r['lethal']} | {r['unknown']} | {r['max_cost']} |")
    lines.extend([
        "",
        "## bridge / map->odom",
        "",
        f"- bridge reasons：`{dict(Counter(r['reason'] for r in bridge))}`",
        "",
        "| t | dxy | dyaw |",
        "|---:|---:|---:|",
    ])
    if jumps:
        for r in jumps[:40]:
            lines.append(f"| {r['t']:.3f} | {r['dxy']:.3f} | {r['dyaw']:.3f} |")
    else:
        lines.append("| - | 0.000 | 0.000 |")
    lines.extend([
        "",
        "## ROS 日志时间线",
        "",
        "| t | event |",
        "|---:|---|",
    ])
    for r in rosout:
        text = r["text"].replace("|", "/")
        lines.append(f"| {r['t']:.3f} | {text} |")
    lines.extend([
        "",
        "## 数据限制",
        "",
        f"- bag 中包含 local costmap：`{'/local_costmap/costmap' in topics}`；包含 prior 候选位姿/置信度：`False`。",
        "- 前方 costmap 统计使用 `/local_costmap/costmap` 栅格、`/odom` 或 `/robot_realpose` 最近位姿，将机器人前方 0.10-1.40m、左右 0.50m 作为检测区域；本 bag 的 local costmap frame 为 `odom_ground`，其 XY 与 `/robot_realpose` 更一致，因此该 frame 使用 `/robot_realpose` 反算。",
        "- 该区域不是 Nav2 footprint/collision checker 的完整等价实现，只用于判断前方代价是否大体持续存在。",
        "- 本报告是离线分析，不是闭环 Nav2 回放。",
        "",
    ])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--start", type=float, default=1780389417.367)
    parser.add_argument("--end", type=float, default=1780389480.950)
    parser.add_argument("--focus-start", type=float, default=1780389438.017)
    parser.add_argument("--focus-end", type=float, default=1780389475.384)
    args = parser.parse_args()
    rows, topics = read_bag(Path(args.bag), args.start, args.end)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(render(args, rows, topics), encoding="utf-8")
    print(out)


if __name__ == "__main__":
    main()
