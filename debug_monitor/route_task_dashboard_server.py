#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路线任务调试台本地服务。

这个脚本不参与机器人控制闭环，只负责：
1. 提供浏览器调试页面；
2. 读取 Todesk 工作区里的地图和点位 JSON；
3. 通过按钮启动/停止 ros2 bag record；
4. 保存调试台侧的命令/事件日志，方便和 bag 一起排查问题。
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import signal
import subprocess
import threading
import time
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import urlparse

import yaml
from PIL import Image


WORKSPACE = Path(__file__).resolve().parents[1]
DASHBOARD_DIR = Path(__file__).resolve().parent / "route_task_dashboard"
HTML_PATH = DASHBOARD_DIR / "index.html"
LOG_DIR = WORKSPACE / "debug_logs" / "route_task_dashboard"
BAG_DIR = WORKSPACE / "debug_bags" / "route_task_dashboard"
WAYPOINTS_JSON = WORKSPACE / "data" / "dynamic_waypoints.json"
MAP_YAML = WORKSPACE / "src" / "humanoid_navigation2" / "maps" / "hall.yaml"

DEFAULT_BAG_TOPICS = [
    "/tf",
    "/tf_static",
    "/map",
    "/odom",
    "/robot_realpose",
    "/cmd_vel",
    "/navigation/status",
    "/navigation/requests",
    "/navigation/acknowledgments",
    "/navigation/waypoints_data",
    "/app/navigation_command",
    "/app/waypoint_command",
    "/localization/prior_map_odom_bridge_status",
    "/prior_localization/confidence",
    "/robot_status_processed",
    "/system/exceptions",
]


class DashboardState:
    """保存本地服务运行态。"""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.bag_process: Optional[subprocess.Popen] = None
        self.bag_output_dir = ""
        self.bag_started_at = 0.0
        self.bag_topics: List[str] = []
        self.log_file = LOG_DIR / f"dashboard_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        BAG_DIR.mkdir(parents=True, exist_ok=True)

    def bag_status(self) -> Dict[str, Any]:
        with self.lock:
            running = self.bag_process is not None and self.bag_process.poll() is None
            return {
                "running": running,
                "pid": self.bag_process.pid if running and self.bag_process else None,
                "output_dir": self.bag_output_dir,
                "started_at": self.bag_started_at,
                "duration_sec": round(time.time() - self.bag_started_at, 1) if running else 0.0,
                "topics": self.bag_topics,
            }

    def append_log(self, event: Dict[str, Any]) -> None:
        event.setdefault("timestamp", time.time())
        event.setdefault("time_text", datetime.now().isoformat(timespec="milliseconds"))
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        with self.log_file.open("a", encoding="utf-8") as f:
            f.write(json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n")


STATE = DashboardState()


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def read_body(handler: BaseHTTPRequestHandler) -> Dict[str, Any]:
    length = int(handler.headers.get("Content-Length", "0") or "0")
    if length <= 0:
        return {}
    raw = handler.rfile.read(length).decode("utf-8")
    return json.loads(raw or "{}")


def load_map_payload() -> Dict[str, Any]:
    """读取 hall.yaml/hall.pgm，并把地图图片转成浏览器可直接显示的 PNG data URL。"""
    map_meta = yaml.safe_load(MAP_YAML.read_text(encoding="utf-8"))
    image_path = (MAP_YAML.parent / map_meta["image"]).resolve()
    with Image.open(image_path) as image:
        image = image.convert("L")
        width, height = image.size
        # Canvas 用 data URL 最省事，调试台不需要额外处理 PGM 兼容性。
        from io import BytesIO

        buffer = BytesIO()
        image.save(buffer, format="PNG")
        image_data = base64.b64encode(buffer.getvalue()).decode("ascii")

    return {
        "yaml_path": str(MAP_YAML),
        "image_path": str(image_path),
        "image_data_url": f"data:image/png;base64,{image_data}",
        "width": width,
        "height": height,
        "resolution": float(map_meta.get("resolution", 0.05)),
        "origin": map_meta.get("origin", [0.0, 0.0, 0.0]),
        "negate": int(map_meta.get("negate", 0)),
        "occupied_thresh": float(map_meta.get("occupied_thresh", 0.65)),
        "free_thresh": float(map_meta.get("free_thresh", 0.196)),
    }


def load_waypoints_payload() -> Dict[str, Any]:
    data = read_json(WAYPOINTS_JSON)
    nav = data.get("waypoints", {}).get("navigation_target", {})
    ordered = []
    for waypoint_id, waypoint in nav.items():
        properties = waypoint.get("properties", {}) if isinstance(waypoint.get("properties"), dict) else {}
        try:
            order = int(properties.get("route_order", waypoint_id))
        except Exception:
            order = 0
        ordered.append({
            "id": str(waypoint.get("id", waypoint_id)),
            "name": waypoint.get("name", waypoint_id),
            "position": waypoint.get("position", [0.0, 0.0, 0.0]),
            "orientation": waypoint.get("orientation", [0.0, 0.0, 0.0, 1.0]),
            "frame_id": waypoint.get("frame_id", "map"),
            "properties": properties,
            "route_order": order,
            "waypoint_role": properties.get("waypoint_role", "task"),
            "walk_direction": properties.get("walk_direction", "forward"),
            "need_broadcast": bool(properties.get("need_broadcast", False)),
            "stop_and_align": bool(properties.get("stop_and_align", True)),
        })
    ordered.sort(key=lambda item: item["route_order"])
    return {
        "waypoints_revision": str(data.get("waypoints_revision", "")),
        "timestamp": data.get("timestamp", 0),
        "waypoints": ordered,
        "route_waypoint_ids": [item["id"] for item in ordered],
    }


def safe_bag_name(raw_name: str) -> str:
    cleaned = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in raw_name.strip())
    return cleaned or f"route_task_{datetime.now().strftime('%Y%m%d_%H%M%S')}"


def start_bag_record(payload: Dict[str, Any]) -> Dict[str, Any]:
    with STATE.lock:
        if STATE.bag_process is not None and STATE.bag_process.poll() is None:
            return {"ok": False, "error": "bag_already_running", "status": STATE.bag_status()}

        name = safe_bag_name(payload.get("name", ""))
        output_dir = BAG_DIR / name
        if output_dir.exists():
            output_dir = BAG_DIR / f"{name}_{datetime.now().strftime('%H%M%S')}"

        topics = payload.get("topics") or DEFAULT_BAG_TOPICS
        if isinstance(topics, str):
            topics = [item.strip() for item in topics.splitlines() if item.strip()]
        topics = [str(topic).strip() for topic in topics if str(topic).strip()]
        if not topics:
            topics = DEFAULT_BAG_TOPICS

        cmd = ["ros2", "bag", "record", "-o", str(output_dir), *topics]
        env = os.environ.copy()
        env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

        proc = subprocess.Popen(
            cmd,
            cwd=str(WORKSPACE),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        STATE.bag_process = proc
        STATE.bag_output_dir = str(output_dir)
        STATE.bag_started_at = time.time()
        STATE.bag_topics = topics
        STATE.append_log({
            "kind": "bag",
            "direction": "local",
            "event": "bag_started",
            "command": cmd,
            "output_dir": str(output_dir),
            "topics": topics,
            "pid": proc.pid,
        })
        return {"ok": True, "status": STATE.bag_status()}


def stop_bag_record() -> Dict[str, Any]:
    with STATE.lock:
        proc = STATE.bag_process
        if proc is None or proc.poll() is not None:
            STATE.bag_process = None
            return {"ok": False, "error": "bag_not_running", "status": STATE.bag_status()}

        pid = proc.pid
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
        except ProcessLookupError:
            pass

    try:
        proc.wait(timeout=12)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=5)

    with STATE.lock:
        output_dir = STATE.bag_output_dir
        STATE.append_log({
            "kind": "bag",
            "direction": "local",
            "event": "bag_stopped",
            "output_dir": output_dir,
            "pid": pid,
            "returncode": proc.returncode,
        })
        STATE.bag_process = None
        return {"ok": True, "output_dir": output_dir, "returncode": proc.returncode, "status": STATE.bag_status()}


def response_json(handler: BaseHTTPRequestHandler, payload: Any, status: int = 200) -> None:
    raw = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(raw)))
    handler.send_header("Access-Control-Allow-Origin", "*")
    handler.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
    handler.send_header("Access-Control-Allow-Headers", "Content-Type")
    handler.end_headers()
    handler.wfile.write(raw)


def response_text(handler: BaseHTTPRequestHandler, text: str, content_type: str = "text/html; charset=utf-8") -> None:
    raw = text.encode("utf-8")
    handler.send_response(HTTPStatus.OK)
    handler.send_header("Content-Type", content_type)
    handler.send_header("Content-Length", str(len(raw)))
    handler.end_headers()
    handler.wfile.write(raw)


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "RouteTaskDashboard/1.0"

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[{datetime.now().isoformat(timespec='seconds')}] {self.address_string()} {fmt % args}")

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self) -> None:
        try:
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                response_text(self, HTML_PATH.read_text(encoding="utf-8"))
            elif path == "/api/config":
                waypoints = load_waypoints_payload()
                response_json(self, {
                    "workspace": str(WORKSPACE),
                    "websocket_url": "ws://127.0.0.1:8765",
                    "alternate_websocket_urls": [
                        "ws://127.0.0.1:8765",
                        "ws://localhost:8765",
                        "ws://10.192.1.2:8765",
                    ],
                    "waypoints_revision": waypoints["waypoints_revision"],
                    "route_waypoint_ids": waypoints["route_waypoint_ids"],
                    "default_bag_topics": DEFAULT_BAG_TOPICS,
                    "bag_status": STATE.bag_status(),
                    "log_file": str(STATE.log_file),
                })
            elif path == "/api/map":
                response_json(self, load_map_payload())
            elif path == "/api/waypoints":
                response_json(self, load_waypoints_payload())
            elif path == "/api/bag/status":
                response_json(self, {"ok": True, "status": STATE.bag_status()})
            elif path == "/api/logs/current":
                response_json(self, {
                    "ok": True,
                    "log_file": str(STATE.log_file),
                    "content": STATE.log_file.read_text(encoding="utf-8") if STATE.log_file.exists() else "",
                })
            else:
                response_json(self, {"ok": False, "error": "not_found", "path": path}, 404)
        except Exception as exc:
            response_json(self, {"ok": False, "error": str(exc)}, 500)

    def do_POST(self) -> None:
        try:
            path = urlparse(self.path).path
            payload = read_body(self)
            if path == "/api/bag/start":
                response_json(self, start_bag_record(payload))
            elif path == "/api/bag/stop":
                response_json(self, stop_bag_record())
            elif path == "/api/log":
                STATE.append_log(payload)
                response_json(self, {"ok": True, "log_file": str(STATE.log_file)})
            else:
                response_json(self, {"ok": False, "error": "not_found", "path": path}, 404)
        except Exception as exc:
            response_json(self, {"ok": False, "error": str(exc)}, 500)


def main() -> int:
    parser = argparse.ArgumentParser(description="路线任务调试台本地服务")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP 监听地址")
    parser.add_argument("--port", type=int, default=18080, help="HTTP 监听端口")
    args = parser.parse_args()

    if not HTML_PATH.exists():
        raise SystemExit(f"dashboard html not found: {HTML_PATH}")

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    print(f"路线任务调试台已启动: http://{args.host}:{args.port}")
    print(f"工作区: {WORKSPACE}")
    print(f"日志文件: {STATE.log_file}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("收到 Ctrl+C，正在关闭调试台...")
    finally:
        if STATE.bag_process is not None and STATE.bag_process.poll() is None:
            stop_bag_record()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
