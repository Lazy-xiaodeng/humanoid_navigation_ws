#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from std_msgs.msg import String


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def pose_from_odom(msg: Odometry) -> tuple[float, float, float]:
    p = msg.pose.pose.position
    return float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation)


def pose_from_cov(msg: PoseWithCovarianceStamped) -> tuple[float, float, float]:
    p = msg.pose.pose.position
    return float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation)


def stamp_sec(msg, fallback_ns: int) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp and (stamp.sec or stamp.nanosec):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(fallback_ns) * 1e-9


def storage_id_for_bag(path: Path) -> str:
    if any(path.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def analyze_series(name: str, rows: list[dict], thresholds: argparse.Namespace) -> dict:
    events = []
    distances = []
    yaw_deltas = []
    speeds = []
    for prev, cur in zip(rows, rows[1:]):
        dt = cur["t"] - prev["t"]
        if dt <= 0.0 or dt > thresholds.max_dt_for_jump:
            continue
        dist = math.hypot(cur["x"] - prev["x"], cur["y"] - prev["y"])
        dyaw = abs(wrap_pi(cur["yaw"] - prev["yaw"]))
        speed = dist / dt
        distances.append(dist)
        yaw_deltas.append(dyaw)
        speeds.append(speed)
        if (
            dist >= thresholds.jump_distance
            or speed >= thresholds.jump_speed
            or dyaw >= math.radians(thresholds.jump_yaw_deg)
        ):
            events.append(
                {
                    "topic": name,
                    "prev_index": prev["index"],
                    "index": cur["index"],
                    "stamp": f"{cur['t']:.6f}",
                    "dt": f"{dt:.4f}",
                    "dx": f"{cur['x'] - prev['x']:.4f}",
                    "dy": f"{cur['y'] - prev['y']:.4f}",
                    "distance": f"{dist:.4f}",
                    "speed": f"{speed:.4f}",
                    "yaw_delta_deg": f"{math.degrees(dyaw):.3f}",
                    "x": f"{cur['x']:.4f}",
                    "y": f"{cur['y']:.4f}",
                    "yaw_deg": f"{math.degrees(cur['yaw']):.3f}",
                }
            )

    def percentile(values: list[float], q: float) -> float:
        if not values:
            return 0.0
        ordered = sorted(values)
        idx = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * q))))
        return ordered[idx]

    return {
        "topic": name,
        "samples": len(rows),
        "duration_sec": rows[-1]["t"] - rows[0]["t"] if len(rows) > 1 else 0.0,
        "jump_events": len(events),
        "max_step_distance": max(distances) if distances else 0.0,
        "p95_step_distance": percentile(distances, 0.95),
        "max_speed": max(speeds) if speeds else 0.0,
        "p95_speed": percentile(speeds, 0.95),
        "max_yaw_delta_deg": math.degrees(max(yaw_deltas)) if yaw_deltas else 0.0,
        "events": events,
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--max-dt-for-jump", type=float, default=0.5)
    parser.add_argument("--jump-distance", type=float, default=0.5)
    parser.add_argument("--jump-speed", type=float, default=2.0)
    parser.add_argument("--jump-yaw-deg", type=float, default=35.0)
    args = parser.parse_args()

    bag = Path(args.bag).expanduser()
    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag), storage_id=storage_id_for_bag(bag)),
        ConverterOptions("", ""),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    odom_rows = []
    realpose_rows = []
    initialpose_rows = []
    recovery_requests = []
    recovery_status = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == "/odom":
            msg = deserialize_message(data, Odometry)
            x, y, yaw = pose_from_odom(msg)
            odom_rows.append({"index": len(odom_rows), "t": stamp_sec(msg, timestamp), "x": x, "y": y, "yaw": yaw})
        elif topic == "/robot_realpose":
            msg = deserialize_message(data, PoseWithCovarianceStamped)
            x, y, yaw = pose_from_cov(msg)
            realpose_rows.append({"index": len(realpose_rows), "t": stamp_sec(msg, timestamp), "x": x, "y": y, "yaw": yaw})
        elif topic == "/initialpose":
            msg = deserialize_message(data, PoseWithCovarianceStamped)
            x, y, yaw = pose_from_cov(msg)
            initialpose_rows.append({"index": len(initialpose_rows), "t": stamp_sec(msg, timestamp), "x": x, "y": y, "yaw": yaw})
        elif topic == "/localization/recovery_requests":
            msg = deserialize_message(data, String)
            recovery_requests.append({"index": len(recovery_requests), "t": timestamp * 1e-9, "data": msg.data})
        elif topic == "/localization/recovery_status":
            msg = deserialize_message(data, String)
            recovery_status.append({"index": len(recovery_status), "t": timestamp * 1e-9, "data": msg.data})

    odom_summary = analyze_series("/odom", odom_rows, args)
    realpose_summary = analyze_series("/robot_realpose", realpose_rows, args)
    initialpose_summary = analyze_series("/initialpose", initialpose_rows, args)
    all_events = odom_summary["events"] + realpose_summary["events"] + initialpose_summary["events"]

    write_csv(out / "jump_events.csv", all_events)
    write_csv(out / "recovery_requests.csv", recovery_requests)
    write_csv(out / "recovery_status.csv", recovery_status)

    summary = {
        "bag": str(bag),
        "topics_present": sorted(type_map),
        "thresholds": {
            "max_dt_for_jump": args.max_dt_for_jump,
            "jump_distance": args.jump_distance,
            "jump_speed": args.jump_speed,
            "jump_yaw_deg": args.jump_yaw_deg,
        },
        "odom": {k: v for k, v in odom_summary.items() if k != "events"},
        "robot_realpose": {k: v for k, v in realpose_summary.items() if k != "events"},
        "initialpose": {k: v for k, v in initialpose_summary.items() if k != "events"},
        "recovery_requests": len(recovery_requests),
        "recovery_status": len(recovery_status),
    }
    (out / "continuity_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    md = [
        "# Nav Bag Continuity Analysis",
        "",
        f"- Bag: `{bag}`",
        f"- Jump thresholds: distance>={args.jump_distance:.2f} m, speed>={args.jump_speed:.2f} m/s, yaw>={args.jump_yaw_deg:.1f} deg, dt<={args.max_dt_for_jump:.2f} s",
        "",
        "## Summary",
        "",
        "| topic | samples | jump events | max step | p95 step | max speed | p95 speed | max yaw step |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in (summary["odom"], summary["robot_realpose"], summary["initialpose"]):
        md.append(
            f"| {item['topic']} | {item['samples']} | {item['jump_events']} | "
            f"{item['max_step_distance']:.4f} m | {item['p95_step_distance']:.4f} m | "
            f"{item['max_speed']:.4f} m/s | {item['p95_speed']:.4f} m/s | "
            f"{item['max_yaw_delta_deg']:.2f} deg |"
        )
    md.extend(
        [
            "",
            "## Recovery Topics",
            "",
            f"- Recovery requests: {len(recovery_requests)}",
            f"- Recovery status messages: {len(recovery_status)}",
            "",
            "Detailed jump events are in `jump_events.csv`.",
        ]
    )
    (out / "continuity_report.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
