#!/usr/bin/env python3
import argparse
import math
import os
from bisect import bisect_left

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
import rosbag2_py
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def setup_chinese_font():
    """Pick an installed CJK font so Chinese labels render in saved figures."""
    preferred = [
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "WenQuanYi Micro Hei",
        "WenQuanYi Zen Hei",
        "SimHei",
        "Microsoft YaHei",
    ]
    available = {}
    for fname in font_manager.findSystemFonts():
        try:
            available[font_manager.FontProperties(fname=fname).get_name()] = fname
        except Exception:
            continue
    for name in preferred:
        if name in available:
            plt.rcParams["font.sans-serif"] = [name]
            break
    plt.rcParams["axes.unicode_minus"] = False


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_to_mat_xyzw(x, y, z, w):
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), 0.0],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), 0.0],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def transform_matrix(x, y, z, qx, qy, qz, qw):
    m = quat_to_mat_xyzw(qx, qy, qz, qw)
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


def yaw_from_matrix(m):
    return math.atan2(m[1, 0], m[0, 0])


def pose_matrix_from_msg_pose(pose):
    p = pose.position
    q = pose.orientation
    return transform_matrix(p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def nearest(rows, t):
    if not rows:
        return None
    times = [r[0] for r in rows]
    idx = bisect_left(times, t)
    if idx <= 0:
        return rows[0]
    if idx >= len(rows):
        return rows[-1]
    return rows[idx] if abs(rows[idx][0] - t) < abs(rows[idx - 1][0] - t) else rows[idx - 1]


def read_bag(uri):
    topics = ["/odom", "/robot_realpose", "/tf", "/localization/prior_map_odom_bridge_status"]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    msg_types = {topic: get_message(types[topic]) for topic in topics if topic in types}

    odom_body = []
    global_pose = []
    map_odom = []
    corrections = []

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns / 1e9
        msg = deserialize_message(raw, msg_types[topic])
        if topic == "/odom":
            odom_body.append((t, pose_matrix_from_msg_pose(msg.pose.pose)))
        elif topic == "/robot_realpose":
            p = msg.pose.pose.position
            yaw = quat_to_yaw(msg.pose.pose.orientation)
            global_pose.append((t, p.x, p.y, yaw))
        elif topic == "/tf":
            for tr in msg.transforms:
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    tf = tr.transform
                    q = tf.rotation
                    v = tf.translation
                    m = transform_matrix(v.x, v.y, v.z, q.x, q.y, q.z, q.w)
                    map_odom.append((t, m))
        elif topic == "/localization/prior_map_odom_bridge_status":
            text = msg.data
            key = "map_odom_xy_norm="
            if key in text:
                try:
                    value = float(text.split(key, 1)[1].split()[0])
                    corrections.append((t, value, text))
                except ValueError:
                    pass

    return odom_body, global_pose, map_odom, corrections


def load_waypoints(path):
    import json
    with open(path, "r") as f:
        data = json.load(f)
    points = []
    for item in data.get("waypoints", {}).get("navigation_target", {}).values():
        name = item.get("name", "")
        pos = item.get("position", [0.0, 0.0, 0.0])
        points.append((name, float(pos[0]), float(pos[1])))
    def key(row):
        digits = "".join(ch for ch in row[0] if ch.isdigit())
        return int(digits) if digits else 9999
    return sorted(points, key=key)


def decimate(rows, step=5):
    return rows[::step] if len(rows) > step else rows


def main():
    parser = argparse.ArgumentParser(
        description="把导航 bag 中的 FAST-LIO 原始里程计、全局定位轨迹和 map->odom 修正量画到地图上。"
    )
    parser.add_argument("--bag", default="/home/ubuntu/nav_drift_test/nav_drift_test30")
    parser.add_argument("--map-yaml", default="/home/ubuntu/software/Todesk/Files/humanoid_ws/install/humanoid_navigation2/share/humanoid_navigation2/maps/hall.yaml")
    parser.add_argument("--map-image", default="/home/ubuntu/software/Todesk/Files/humanoid_ws/install/humanoid_navigation2/share/humanoid_navigation2/maps/hall.pgm")
    parser.add_argument("--waypoints", default="/home/ubuntu/software/Todesk/Files/humanoid_ws/data/dynamic_waypoints.json")
    parser.add_argument("--out-dir", default="/home/ubuntu/software/Todesk/Files/humanoid_ws/debug_logs/nav_drift_trajectories")
    args = parser.parse_args()

    setup_chinese_font()
    os.makedirs(args.out_dir, exist_ok=True)
    odom_body, global_pose, map_odom, corrections = read_bag(args.bag)
    if not odom_body or not global_pose or not map_odom:
        raise RuntimeError("missing required odom/global_pose/map_odom data")

    # Static TFs from the launch log:
    # odom -> camera_init and body -> base_footprint.
    odom_to_camera = transform_matrix(0, 0, 0, -0.5, -0.5, 0.5, 0.5)
    body_to_base = transform_matrix(0.004, 1.215, 0.072, 0.5, 0.5, -0.5, 0.5)

    initial_map_odom = map_odom[0][1]
    fastlio_aligned = []
    fused_from_tf = []
    for t, camera_to_body in odom_body:
        odom_to_base = odom_to_camera @ camera_to_body @ body_to_base
        raw_map_to_base = initial_map_odom @ odom_to_base
        fastlio_aligned.append((t, raw_map_to_base[0, 3], raw_map_to_base[1, 3], yaw_from_matrix(raw_map_to_base)))
        mo = nearest(map_odom, t)
        if mo is not None and abs(mo[0] - t) < 0.25:
            fused = mo[1] @ odom_to_base
            fused_from_tf.append((t, fused[0, 3], fused[1, 3], yaw_from_matrix(fused)))

    # Parse yaml without depending on pyyaml.
    yaml_text = open(args.map_yaml).read().splitlines()
    resolution = float(next(line.split(":", 1)[1] for line in yaml_text if line.startswith("resolution:")).strip())
    origin_text = next(line.split(":", 1)[1] for line in yaml_text if line.startswith("origin:")).strip()
    origin = [float(x.strip()) for x in origin_text.strip("[]").split(",")]
    img = np.array(Image.open(args.map_image))
    height, width = img.shape[:2]
    extent = [origin[0], origin[0] + width * resolution, origin[1], origin[1] + height * resolution]

    waypoints = load_waypoints(args.waypoints)
    gps = decimate(global_pose, 3)
    fl = decimate(fastlio_aligned, 3)
    fused = decimate(fused_from_tf, 3)

    fig, ax = plt.subplots(figsize=(15, 14), dpi=180)
    ax.imshow(np.flipud(img), cmap="gray", extent=extent, origin="lower", alpha=0.78)
    ax.plot([r[1] for r in fl], [r[2] for r in fl], color="#d95f02", lw=1.8, label="FAST-LIO原始里程计轨迹(按初始map->odom对齐)")
    ax.plot([r[1] for r in fused], [r[2] for r in fused], color="#1b9e77", lw=1.2, alpha=0.55, label="TF融合轨迹(map->odom已纠偏)")
    ax.plot([r[1] for r in gps], [r[2] for r in gps], color="#1f4aff", lw=1.4, label="导航实际全局位姿 /robot_realpose")

    if corrections:
        corr_times = [c[0] for c in corrections]
        corr_values = [c[1] for c in corrections]
        corr_xy = []
        for t, value, _ in corrections:
            p = nearest(global_pose, t)
            if p is not None:
                corr_xy.append((p[1], p[2], value))
        if corr_xy:
            sc = ax.scatter(
                [r[0] for r in corr_xy],
                [r[1] for r in corr_xy],
                c=[r[2] for r in corr_xy],
                s=16,
                cmap="magma",
                alpha=0.65,
                label="全局定位纠偏量",
            )
            cbar = fig.colorbar(sc, ax=ax, fraction=0.035, pad=0.02)
            cbar.set_label("map->odom XY纠偏量 (m)")

    for name, x, y in waypoints:
        ax.scatter([x], [y], s=18, color="black", zorder=5)
        if name in {"点位1", "点位3", "点位14", "点位18", "点位19", "点位22"}:
            ax.text(x + 0.15, y + 0.15, name, fontsize=9, color="black")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(min(extent[0], min(r[1] for r in fl + gps)) - 1, max(extent[1], max(r[1] for r in fl + gps)) + 1)
    ax.set_ylim(min(extent[2], min(r[2] for r in fl + gps)) - 1, max(extent[3], max(r[2] for r in fl + gps)) + 1)
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("地图X坐标 (m)")
    ax.set_ylabel("地图Y坐标 (m)")
    ax.set_title("导航全程轨迹对比：FAST-LIO原始里程计 vs 全局定位纠偏后轨迹")
    ax.legend(loc="upper left")
    out_png = os.path.join(args.out_dir, "full_trajectory_on_map.png")
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(15, 5), dpi=180)
    if corrections:
        t0 = corrections[0][0]
        ax.plot([(c[0] - t0) for c in corrections], [c[1] for c in corrections], color="#d95f02", lw=1.2)
        ax.set_ylabel("map->odom XY纠偏量 (m)")
        ax.set_xlabel("距首次纠偏的时间 (s)")
        ax.set_title("全局定位纠偏量随时间变化")
        ax.grid(True, alpha=0.3)
    out_corr = os.path.join(args.out_dir, "map_odom_correction_over_time.png")
    fig.tight_layout()
    fig.savefig(out_corr)
    plt.close(fig)

    csv_path = os.path.join(args.out_dir, "trajectory_samples.csv")
    with open(csv_path, "w") as f:
        f.write("topic,t,x,y,yaw\n")
        for name, rows in [
            ("fastlio_raw_aligned", fastlio_aligned),
            ("global_robot_realpose", global_pose),
            ("tf_fused_map_base", fused_from_tf),
        ]:
            for t, x, y, yaw in decimate(rows, 10):
                f.write(f"{name},{t:.6f},{x:.6f},{y:.6f},{yaw:.6f}\n")

    print(out_png)
    print(out_corr)
    print(csv_path)
    print(f"counts fastlio={len(fastlio_aligned)} global={len(global_pose)} fused={len(fused_from_tf)} corrections={len(corrections)}")


if __name__ == "__main__":
    main()
