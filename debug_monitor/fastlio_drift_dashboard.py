#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import signal
import threading
import time
import urllib.parse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from hdl_localization.msg import ScanMatchingStatus
except Exception:
    ScanMatchingStatus = None


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def pose_delta(now: Optional[Dict[str, Any]], base: Optional[Dict[str, Any]]) -> Optional[Dict[str, float]]:
    if not now or not base:
        return None
    dx = float(now["x"]) - float(base["x"])
    dy = float(now["y"]) - float(base["y"])
    dz = float(now["z"]) - float(base["z"])
    dyaw = angle_delta(float(now["yaw"]), float(base["yaw"]))
    return {
        "dx": dx,
        "dy": dy,
        "dz": dz,
        "dxy": math.hypot(dx, dy),
        "dyaw": dyaw,
        "dyaw_deg": math.degrees(dyaw),
    }


def make_pose_dict(stamp: float, frame: str, child: str, p, q) -> Dict[str, Any]:
    return {
        "stamp": stamp,
        "frame": frame,
        "child": child,
        "x": float(p.x),
        "y": float(p.y),
        "z": float(p.z),
        "yaw": yaw_from_quat(q),
        "yaw_deg": math.degrees(yaw_from_quat(q)),
    }


class FastLioDriftDashboard(Node):
    def __init__(self, args):
        super().__init__("fastlio_drift_dashboard")
        self.args = args
        self.started_wall = time.time()
        self.lock = threading.RLock()
        self.latest: Dict[str, Any] = {}
        self.baseline: Dict[str, Any] = {}
        self.markers = deque(maxlen=200)
        self.events = deque(maxlen=1000)
        self.last_snapshot: Optional[Dict[str, Any]] = None
        self.last_snapshot_wall = 0.0

        os.makedirs(args.out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.jsonl_path = os.path.abspath(os.path.join(args.out_dir, f"fastlio_drift_live_{ts}.jsonl"))
        self.meta_path = os.path.abspath(os.path.join(args.out_dir, f"fastlio_drift_live_{ts}_meta.txt"))
        self.jsonl_file = open(self.jsonl_path, "a", buffering=1)
        with open(self.meta_path, "w") as f:
            f.write(f"started_wall_epoch={self.started_wall:.6f}\n")
            f.write(f"jsonl_path={self.jsonl_path}\n")
            f.write(f"http_port={args.port}\n")
            f.write("tf_pairs=map->odom,camera_init->body,map->base_footprint,odom->base_footprint\n")

        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, "/odom", self.odom_cb, 50)
        self.create_subscription(PoseStamped, "/pcl_pose", self.pcl_pose_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 20)
        self.create_subscription(Imu, "/airy_imu", self.imu_cb, self.sensor_qos)
        self.create_subscription(PointCloud2, "/airy_points", lambda m: self.cloud_cb("/airy_points", m), self.sensor_qos)
        self.create_subscription(
            PointCloud2,
            "/fast_lio/cloud_registered",
            lambda m: self.cloud_cb("/fast_lio/cloud_registered", m),
            self.sensor_qos,
        )
        if ScanMatchingStatus is not None:
            self.create_subscription(ScanMatchingStatus, "/status", lambda m: self.ndt_cb("/status", m), 20)
            self.create_subscription(
                ScanMatchingStatus,
                "/localization/ndt_status",
                lambda m: self.ndt_cb("/localization/ndt_status", m),
                20,
            )

        self.freq_windows: Dict[str, deque] = {
            "/odom": deque(maxlen=100),
            "/pcl_pose": deque(maxlen=100),
            "/airy_imu": deque(maxlen=400),
            "/airy_points": deque(maxlen=100),
            "/fast_lio/cloud_registered": deque(maxlen=100),
            "/status": deque(maxlen=100),
            "/localization/ndt_status": deque(maxlen=100),
        }
        self.prev_pose_for_step: Dict[str, Dict[str, Any]] = {}

        self.create_timer(args.sample_period, self.sample_cb)
        self.add_event("monitor_start", "dashboard started")
        self.get_logger().info(f"dashboard jsonl: {self.jsonl_path}")

    def add_event(self, kind: str, text: str):
        event = {"wall": time.time(), "elapsed": time.time() - self.started_wall, "kind": kind, "text": text}
        with self.lock:
            self.events.append(event)
        try:
            self.jsonl_file.write(json.dumps({"type": "event", **event}, ensure_ascii=False) + "\n")
        except Exception:
            pass

    def mark(self, label: str):
        label = label.strip()[:80] or "marker"
        marker = {"wall": time.time(), "elapsed": time.time() - self.started_wall, "label": label}
        with self.lock:
            self.markers.append(marker)
        self.add_event("marker", label)

    def reset_baseline(self):
        with self.lock:
            self.baseline = {
                key: self.latest.get(key)
                for key in ["odom", "pcl_pose", "tf_map_odom", "tf_camera_init_body", "tf_map_base", "tf_odom_base"]
                if self.latest.get(key)
            }
        self.add_event("baseline_reset", "baseline reset from current values")

    def record_freq(self, topic: str):
        self.freq_windows.setdefault(topic, deque(maxlen=100)).append(time.time())

    def hz(self, topic: str) -> Optional[float]:
        xs = self.freq_windows.get(topic)
        if not xs or len(xs) < 2:
            return None
        dt = xs[-1] - xs[0]
        if dt <= 0.0:
            return None
        return (len(xs) - 1) / dt

    def update_pose_step(self, key: str, pose: Dict[str, Any]):
        prev = self.prev_pose_for_step.get(key)
        if prev:
            d = pose_delta(pose, prev)
            if d:
                pose["step_dxy"] = d["dxy"]
                pose["step_dz"] = d["dz"]
                pose["step_dyaw_deg"] = d["dyaw_deg"]
        self.prev_pose_for_step[key] = dict(pose)

    def odom_cb(self, msg: Odometry):
        self.record_freq("/odom")
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        pose = make_pose_dict(stamp_to_sec(msg.header.stamp), msg.header.frame_id, msg.child_frame_id, p, q)
        pose["vx"] = float(msg.twist.twist.linear.x)
        pose["vy"] = float(msg.twist.twist.linear.y)
        pose["vz"] = float(msg.twist.twist.linear.z)
        pose["wz"] = float(msg.twist.twist.angular.z)
        self.update_pose_step("odom", pose)
        with self.lock:
            self.latest["odom"] = pose

    def pcl_pose_cb(self, msg: PoseStamped):
        self.record_freq("/pcl_pose")
        p = msg.pose.position
        q = msg.pose.orientation
        pose = make_pose_dict(stamp_to_sec(msg.header.stamp), msg.header.frame_id, "", p, q)
        self.update_pose_step("pcl_pose", pose)
        with self.lock:
            self.latest["pcl_pose"] = pose

    def cmd_vel_cb(self, msg: Twist):
        with self.lock:
            self.latest["cmd_vel"] = {
                "wall": time.time(),
                "linear_x": float(msg.linear.x),
                "linear_y": float(msg.linear.y),
                "angular_z": float(msg.angular.z),
            }

    def imu_cb(self, msg: Imu):
        self.record_freq("/airy_imu")
        a = msg.linear_acceleration
        g = msg.angular_velocity
        with self.lock:
            self.latest["imu"] = {
                "stamp": stamp_to_sec(msg.header.stamp),
                "frame": msg.header.frame_id,
                "ax": float(a.x),
                "ay": float(a.y),
                "az": float(a.z),
                "acc_norm": math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z),
                "gx": float(g.x),
                "gy": float(g.y),
                "gz": float(g.z),
                "gyro_norm": math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z),
            }

    def cloud_cb(self, topic: str, msg: PointCloud2):
        self.record_freq(topic)
        with self.lock:
            self.latest[topic] = {
                "stamp": stamp_to_sec(msg.header.stamp),
                "frame": msg.header.frame_id,
                "width": int(msg.width),
                "height": int(msg.height),
                "points": int(msg.width) * int(msg.height),
            }

    def ndt_cb(self, topic: str, msg):
        self.record_freq(topic)
        rel = msg.relative_pose
        with self.lock:
            self.latest["ndt_status"] = {
                "topic": topic,
                "stamp": stamp_to_sec(msg.header.stamp),
                "converged": bool(msg.has_converged),
                "matching_error": float(msg.matching_error),
                "inlier_fraction": float(msg.inlier_fraction),
                "relative_x": float(rel.translation.x),
                "relative_y": float(rel.translation.y),
                "relative_z": float(rel.translation.z),
                "relative_yaw_deg": math.degrees(yaw_from_quat(rel.rotation)),
            }

    def tf_pose(self, parent: str, child: str) -> Optional[Dict[str, Any]]:
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return make_pose_dict(stamp_to_sec(tf.header.stamp), parent, child, t, r)

    def sample_cb(self):
        tf_values = {
            "tf_map_odom": self.tf_pose("map", "odom"),
            "tf_camera_init_body": self.tf_pose("camera_init", "body"),
            "tf_map_base": self.tf_pose("map", "base_footprint"),
            "tf_odom_base": self.tf_pose("odom", "base_footprint"),
        }
        with self.lock:
            for key, val in tf_values.items():
                if val:
                    self.update_pose_step(key, val)
                    self.latest[key] = val

            freq = {topic: self.hz(topic) for topic in self.freq_windows}
            ages = {}
            now_wall = time.time()
            for key, val in self.latest.items():
                if isinstance(val, dict) and "wall" in val:
                    ages[key] = now_wall - float(val["wall"])

            drift = {}
            for key in ["odom", "pcl_pose", "tf_map_odom", "tf_camera_init_body", "tf_map_base", "tf_odom_base"]:
                drift[key] = pose_delta(self.latest.get(key), self.baseline.get(key))

            alerts = self.make_alerts(drift)
            snapshot = {
                "type": "sample",
                "wall": now_wall,
                "elapsed": now_wall - self.started_wall,
                "latest": dict(self.latest),
                "baseline": dict(self.baseline),
                "drift": drift,
                "freq": freq,
                "ages": ages,
                "markers": list(self.markers)[-20:],
                "events": list(self.events)[-50:],
                "alerts": alerts,
                "log_path": self.jsonl_path,
            }
            self.last_snapshot = snapshot

        self.jsonl_file.write(json.dumps(snapshot, ensure_ascii=False, allow_nan=False) + "\n")

    def make_alerts(self, drift: Dict[str, Any]):
        alerts = []
        odom_d = drift.get("odom")
        body_d = drift.get("tf_camera_init_body")
        ndt = self.latest.get("ndt_status")
        if odom_d and abs(odom_d["dz"]) > 0.20:
            alerts.append(f"/odom z drift {odom_d['dz']:+.3f} m")
        if body_d and abs(body_d["dz"]) > 0.20:
            alerts.append(f"camera_init->body z drift {body_d['dz']:+.3f} m")
        if odom_d and odom_d["dxy"] > 0.30 and not self.is_cmd_active():
            alerts.append(f"/odom moved {odom_d['dxy']:.3f} m from baseline while cmd_vel is near zero")
        if ndt and (not ndt.get("converged", True)):
            alerts.append("NDT not converged")
        if ndt and ndt.get("matching_error", 0.0) > self.args.bad_fitness:
            alerts.append(f"NDT high matching_error {ndt['matching_error']:.3f}")
        if self.hz("/airy_imu") is not None and self.hz("/airy_imu") < 100.0:
            alerts.append(f"/airy_imu low rate {self.hz('/airy_imu'):.1f} Hz")
        if self.hz("/airy_points") is not None and self.hz("/airy_points") < 5.0:
            alerts.append(f"/airy_points low rate {self.hz('/airy_points'):.1f} Hz")
        return alerts

    def is_cmd_active(self) -> bool:
        cmd = self.latest.get("cmd_vel")
        if not cmd:
            return False
        return abs(cmd.get("linear_x", 0.0)) > 0.02 or abs(cmd.get("linear_y", 0.0)) > 0.02 or abs(cmd.get("angular_z", 0.0)) > 0.03

    def snapshot_json(self) -> str:
        with self.lock:
            data = self.last_snapshot or {
                "type": "sample",
                "wall": time.time(),
                "elapsed": time.time() - self.started_wall,
                "latest": {},
                "baseline": {},
                "drift": {},
                "freq": {},
                "markers": list(self.markers),
                "events": list(self.events),
                "alerts": ["waiting for first ROS samples"],
                "log_path": self.jsonl_path,
            }
        return json.dumps(data, ensure_ascii=False, allow_nan=False)

    def close_files(self):
        try:
            self.jsonl_file.close()
        except Exception:
            pass


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Fast-LIO Drift Dashboard</title>
<style>
:root { color-scheme: dark; --bg:#111418; --panel:#1b2027; --line:#303844; --text:#e9eef5; --muted:#9ca8b8; --good:#46d38c; --warn:#ffd166; --bad:#ff6b6b; --blue:#73b7ff; }
* { box-sizing: border-box; }
body { margin:0; background:var(--bg); color:var(--text); font:14px/1.45 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
header { position:sticky; top:0; z-index:2; display:flex; gap:12px; align-items:center; justify-content:space-between; padding:12px 16px; background:#151a20; border-bottom:1px solid var(--line); }
h1 { margin:0; font-size:18px; font-weight:650; letter-spacing:0; }
main { padding:14px; display:grid; grid-template-columns: repeat(12, minmax(0,1fr)); gap:12px; }
section { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:12px; min-width:0; }
.span12 { grid-column:span 12; } .span8 { grid-column:span 8; } .span6 { grid-column:span 6; } .span4 { grid-column:span 4; }
.toolbar { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
button, a.button { border:1px solid var(--line); color:var(--text); background:#242b35; border-radius:6px; padding:7px 10px; text-decoration:none; cursor:pointer; min-height:34px; }
button:hover, a.button:hover { background:#2e3744; }
.primary { border-color:#4477aa; background:#243a52; }
.danger { border-color:#7c3838; background:#42252a; }
.grid { display:grid; grid-template-columns: repeat(4, minmax(0,1fr)); gap:8px; }
.metric { border:1px solid #2b333f; background:#151a20; border-radius:6px; padding:8px; min-height:68px; }
.label { color:var(--muted); font-size:12px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.value { font-size:18px; margin-top:3px; font-variant-numeric:tabular-nums; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.sub { color:var(--muted); font-size:12px; margin-top:2px; font-variant-numeric:tabular-nums; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.ok { color:var(--good); } .warn { color:var(--warn); } .bad { color:var(--bad); } .blue { color:var(--blue); }
table { width:100%; border-collapse:collapse; font-variant-numeric:tabular-nums; }
th, td { text-align:left; border-bottom:1px solid #2a323d; padding:6px 5px; white-space:nowrap; }
th { color:var(--muted); font-weight:600; }
td.mono { font-family:ui-monospace,SFMono-Regular,Consolas,monospace; }
.alerts { display:flex; gap:8px; flex-wrap:wrap; }
.pill { border:1px solid #49363b; background:#2a1e22; color:#ffd6d6; border-radius:999px; padding:5px 9px; }
.empty { color:var(--muted); }
input { background:#12161b; border:1px solid var(--line); color:var(--text); border-radius:6px; padding:7px 9px; min-height:34px; width:160px; }
@media (max-width: 1000px) { .span8,.span6,.span4 { grid-column:span 12; } .grid { grid-template-columns: repeat(2, minmax(0,1fr)); } header { align-items:flex-start; flex-direction:column; } }
</style>
</head>
<body>
<header>
  <h1>Fast-LIO Drift Dashboard <span id="state" class="sub">connecting...</span></h1>
  <div class="toolbar">
    <button class="primary" onclick="post('/reset_baseline')">重置基准</button>
    <button onclick="mark('静止开始')">静止</button>
    <button onclick="mark('直走开始')">直走</button>
    <button onclick="mark('转向开始')">转向</button>
    <button onclick="mark('停止/测试结束')">停止</button>
    <input id="customMark" placeholder="自定义标记">
    <button onclick="mark(document.getElementById('customMark').value)">标记</button>
    <a class="button" href="/download.jsonl">JSONL</a>
    <a class="button" href="/download.csv">CSV</a>
  </div>
</header>
<main>
  <section class="span12"><div id="alerts" class="alerts"><span class="empty">等待数据...</span></div></section>
  <section class="span8">
    <h2>Pose / TF 数值</h2>
    <table><thead><tr><th>来源</th><th>x</th><th>y</th><th>z</th><th>yaw</th><th>相对基准 dxy</th><th>dz</th><th>dyaw</th><th>step</th><th>Hz</th></tr></thead><tbody id="poseRows"></tbody></table>
  </section>
  <section class="span4">
    <h2>传感器</h2>
    <div class="grid" id="sensorGrid"></div>
  </section>
  <section class="span6">
    <h2>NDT / 命令</h2>
    <div class="grid" id="ndtGrid"></div>
  </section>
  <section class="span6">
    <h2>标记 / 日志</h2>
    <div class="grid">
      <div class="metric"><div class="label">运行时间</div><div class="value" id="elapsed">--</div></div>
      <div class="metric"><div class="label">日志文件</div><div class="value" style="font-size:13px" id="logPath">--</div></div>
    </div>
    <table><thead><tr><th>t(s)</th><th>标记</th></tr></thead><tbody id="markerRows"></tbody></table>
  </section>
</main>
<script>
const fmt = (v, n=3) => (v === undefined || v === null || Number.isNaN(v)) ? '--' : Number(v).toFixed(n);
const hz = (v) => (v === undefined || v === null) ? '--' : Number(v).toFixed(1);
function clsAbs(v, warn, bad) { if (v === undefined || v === null) return ''; const a = Math.abs(v); return a >= bad ? 'bad' : (a >= warn ? 'warn' : 'ok'); }
async function post(path) { await fetch(path, {method:'POST'}); }
async function mark(label) { label = (label || '').trim(); if (!label) return; await fetch('/mark?label=' + encodeURIComponent(label), {method:'POST'}); document.getElementById('customMark').value=''; }
function poseRow(name, key, data, freqKey) {
  const p = data.latest[key] || {};
  const d = data.drift[key] || {};
  const f = data.freq[freqKey || key] || data.freq['/' + key] || null;
  const step = [fmt(p.step_dxy), fmt(p.step_dz), fmt(p.step_dyaw_deg,1)].join(' / ');
  return `<tr><td>${name}</td><td class="mono">${fmt(p.x)}</td><td class="mono">${fmt(p.y)}</td><td class="mono ${clsAbs(p.z,0.2,0.5)}">${fmt(p.z)}</td><td class="mono">${fmt(p.yaw_deg,1)}</td><td class="mono ${clsAbs(d.dxy,0.15,0.5)}">${fmt(d.dxy)}</td><td class="mono ${clsAbs(d.dz,0.1,0.3)}">${fmt(d.dz)}</td><td class="mono ${clsAbs(d.dyaw_deg,2,8)}">${fmt(d.dyaw_deg,1)}</td><td class="mono">${step}</td><td class="mono">${hz(f)}</td></tr>`;
}
function metric(label, value, sub='', klass='') { return `<div class="metric"><div class="label">${label}</div><div class="value ${klass}">${value}</div><div class="sub">${sub}</div></div>`; }
function render(data) {
  document.getElementById('state').textContent = ` t=${fmt(data.elapsed,1)}s`;
  document.getElementById('elapsed').textContent = `${fmt(data.elapsed,1)} s`;
  document.getElementById('logPath').textContent = data.log_path || '--';
  const alerts = data.alerts || [];
  document.getElementById('alerts').innerHTML = alerts.length ? alerts.map(a => `<span class="pill">${a}</span>`).join('') : '<span class="empty ok">当前无报警，继续观察数值</span>';
  document.getElementById('poseRows').innerHTML = [
    poseRow('/odom', 'odom', data, '/odom'),
    poseRow('/pcl_pose', 'pcl_pose', data, '/pcl_pose'),
    poseRow('TF map->odom', 'tf_map_odom', data),
    poseRow('TF camera_init->body', 'tf_camera_init_body', data),
    poseRow('TF map->base', 'tf_map_base', data),
    poseRow('TF odom->base', 'tf_odom_base', data),
  ].join('');
  const imu = data.latest.imu || {};
  const airy = data.latest['/airy_points'] || {};
  const reg = data.latest['/fast_lio/cloud_registered'] || {};
  document.getElementById('sensorGrid').innerHTML = [
    metric('/airy_imu Hz', hz(data.freq['/airy_imu']), `|a|=${fmt(imu.acc_norm)} |g|=${fmt(imu.gyro_norm)}`),
    metric('acc xyz', `${fmt(imu.ax)} ${fmt(imu.ay)} ${fmt(imu.az)}`, imu.frame || ''),
    metric('gyro xyz', `${fmt(imu.gx)} ${fmt(imu.gy)} ${fmt(imu.gz)}`, 'rad/s'),
    metric('/airy_points', `${hz(data.freq['/airy_points'])} Hz`, `${airy.points || '--'} pts ${airy.frame || ''}`),
    metric('/cloud_registered', `${hz(data.freq['/fast_lio/cloud_registered'])} Hz`, `${reg.points || '--'} pts ${reg.frame || ''}`),
  ].join('');
  const ndt = data.latest.ndt_status || {};
  const cmd = data.latest.cmd_vel || {};
  document.getElementById('ndtGrid').innerHTML = [
    metric('NDT converged', ndt.converged === undefined ? '--' : String(ndt.converged), ndt.topic || '', ndt.converged === false ? 'bad' : 'ok'),
    metric('matching_error', fmt(ndt.matching_error), `inlier=${fmt(ndt.inlier_fraction)}`, ndt.matching_error > 1.0 ? 'warn' : ''),
    metric('NDT relative', `${fmt(ndt.relative_x)} ${fmt(ndt.relative_y)} ${fmt(ndt.relative_z)}`, `yaw=${fmt(ndt.relative_yaw_deg,1)} deg`),
    metric('/cmd_vel', `${fmt(cmd.linear_x)} ${fmt(cmd.linear_y)} ${fmt(cmd.angular_z)}`, 'vx vy wz'),
  ].join('');
  const markers = data.markers || [];
  document.getElementById('markerRows').innerHTML = markers.slice().reverse().map(m => `<tr><td class="mono">${fmt(m.elapsed,1)}</td><td>${m.label}</td></tr>`).join('');
}
const es = new EventSource('/events');
es.onmessage = (ev) => render(JSON.parse(ev.data));
es.onerror = () => { document.getElementById('state').textContent = ' disconnected'; };
</script>
</body>
</html>
"""


def flatten(prefix: str, obj: Any, out: Dict[str, Any]):
    if isinstance(obj, dict):
        for k, v in obj.items():
            flatten(f"{prefix}_{k}" if prefix else k, v, out)
    elif isinstance(obj, (str, int, float, bool)) or obj is None:
        out[prefix] = obj


def make_handler(node: FastLioDriftDashboard):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def send_text(self, code: int, text: str, content_type: str = "text/plain; charset=utf-8"):
            body = text.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            parsed = urllib.parse.urlparse(self.path)
            if parsed.path == "/":
                self.send_text(200, HTML, "text/html; charset=utf-8")
            elif parsed.path == "/api/latest":
                self.send_text(200, node.snapshot_json(), "application/json; charset=utf-8")
            elif parsed.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                while rclpy.ok():
                    try:
                        self.wfile.write(f"data: {node.snapshot_json()}\n\n".encode("utf-8"))
                        self.wfile.flush()
                    except Exception:
                        break
                    time.sleep(0.5)
            elif parsed.path == "/download.jsonl":
                try:
                    with open(node.jsonl_path, "rb") as f:
                        body = f.read()
                except FileNotFoundError:
                    body = b""
                self.send_response(200)
                self.send_header("Content-Type", "application/jsonl")
                self.send_header("Content-Disposition", f'attachment; filename="{os.path.basename(node.jsonl_path)}"')
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif parsed.path == "/download.csv":
                rows = []
                fields = set()
                try:
                    with open(node.jsonl_path, "r") as f:
                        for line in f:
                            rec = json.loads(line)
                            if rec.get("type") != "sample":
                                continue
                            flat = {}
                            flatten("", rec, flat)
                            rows.append(flat)
                            fields.update(flat.keys())
                except Exception:
                    pass
                fieldnames = sorted(fields)
                from io import StringIO
                buf = StringIO()
                writer = csv.DictWriter(buf, fieldnames=fieldnames)
                writer.writeheader()
                for row in rows:
                    writer.writerow(row)
                self.send_response(200)
                self.send_header("Content-Type", "text/csv; charset=utf-8")
                self.send_header("Content-Disposition", 'attachment; filename="fastlio_drift_live.csv"')
                body = buf.getvalue().encode("utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_text(404, "not found")

        def do_POST(self):
            parsed = urllib.parse.urlparse(self.path)
            if parsed.path == "/mark":
                qs = urllib.parse.parse_qs(parsed.query)
                node.mark(qs.get("label", ["marker"])[0])
                self.send_text(200, "ok")
            elif parsed.path == "/reset_baseline":
                node.reset_baseline()
                self.send_text(200, "ok")
            else:
                self.send_text(404, "not found")

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--out-dir", default="/home/ubuntu/humanoid_ws/debug_monitor")
    parser.add_argument("--sample-period", type=float, default=0.5)
    parser.add_argument("--bad-fitness", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = FastLioDriftDashboard(args)
    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(node))
    http_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    http_thread.start()
    node.get_logger().info(f"open dashboard: http://{args.host}:{args.port}/")

    stop = threading.Event()

    def handle_signal(signum, frame):
        stop.set()
        httpd.shutdown()
        node.add_event("monitor_stop", f"signal {signum}")
        node.close_files()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        httpd.shutdown()
        node.close_files()
        node.destroy_node()


if __name__ == "__main__":
    main()
