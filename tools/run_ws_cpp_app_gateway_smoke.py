#!/usr/bin/env python3
"""C++ APP 网关 WebSocket smoke。

功能：
1. 自动选择本机空闲端口，避免和现场 8765 冲突。
2. 通过 humanoid_app_gateway_runtime/app_gateway_runtime.launch.py 启动 C++ APP 网关和 C++ 数据整合。
3. 连接 C++ APP WebSocket，验证 connection_ack、request 转发和 integration response 回发。
4. 使用独立 ROS_DOMAIN_ID，不影响当前导航/控制系统。
"""

import asyncio
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import websockets

WORKSPACE = Path(__file__).resolve().parents[1]
APP_GATEWAY_EXE = str(WORKSPACE / 'install' / 'humanoid_app_gateway_runtime' / 'lib' / 'humanoid_app_gateway_runtime' / 'app_gateway_node')
DATA_INTEGRATION_EXE = str(WORKSPACE / 'install' / 'humanoid_app_gateway_runtime' / 'lib' / 'humanoid_app_gateway_runtime' / 'data_integration_node')


class Probe(Node):
    def __init__(self):
        super().__init__('ws_cpp_app_gateway_smoke_probe')
        self.data_requests = []
        self.create_subscription(String, '/websocket/data_requests', self.data_requests.append, 10)
        self.data_response_pub = self.create_publisher(String, '/integration/data_responses', 10)


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def matching_pids(needles):
    pids = set()
    proc_root = Path('/proc')
    for item in proc_root.iterdir():
        if not item.name.isdigit():
            continue
        try:
            cmdline = (item / 'cmdline').read_bytes().replace(b'\x00', b' ').decode('utf-8', errors='ignore')
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if any(needle in cmdline for needle in needles):
            pids.add(int(item.name))
    return pids


def cleanup_new_runtime_processes(before_pids):
    after_pids = matching_pids([APP_GATEWAY_EXE, DATA_INTEGRATION_EXE])
    new_pids = sorted(pid for pid in after_pids - before_pids if pid != os.getpid())
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        still_running = []
        for pid in new_pids:
            try:
                os.kill(pid, sig)
                still_running.append(pid)
            except ProcessLookupError:
                pass
        if not still_running:
            return
        new_pids = still_running
        time.sleep(0.5)


def wait_for_port(port, proc, timeout_sec=6.0):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError('launch exited early')
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.2)
            if sock.connect_ex(('127.0.0.1', port)) == 0:
                return True
        time.sleep(0.1)
    return False


def spin_until(node, predicate, timeout_sec=3.0):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


async def recv_until(ws, predicate, timeout_sec=3.0):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        payload = json.loads(await asyncio.wait_for(ws.recv(), max(0.1, deadline - time.time())))
        if predicate(payload):
            return payload
    raise TimeoutError('expected websocket message was not received')


async def exercise(port, node):
    async with websockets.connect(f'ws://127.0.0.1:{port}') as ws:
        ack = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
        if ack.get('data_type') != 'connection_ack':
            raise RuntimeError(f'bad connection ack: {ack}')
        client_id = ack.get('data', {}).get('client_id')
        if not client_id:
            raise RuntimeError(f'missing client_id: {ack}')

        for _ in range(12):
            rclpy.spin_once(node, timeout_sec=0.05)
            await asyncio.sleep(0.05)

        request = {
            'protocol_version': '2.0',
            'message_id': 'ws-cpp-smoke-request',
            'timestamp': time.time(),
            'message_type': 'request',
            'data_type': 'system_status',
            'source': 'smoke-client',
            'destination': 'websocket_server',
            'data': {},
            'metadata': {},
        }
        await ws.send(json.dumps(request, ensure_ascii=False))
        await recv_until(ws, lambda item: item.get('data_type') == 'request_ack')
        if not spin_until(node, lambda: node.data_requests):
            raise RuntimeError('request was not forwarded to /websocket/data_requests')
        forwarded = json.loads(node.data_requests[-1].data)
        if forwarded.get('source') != client_id or forwarded.get('destination') != 'data_integration':
            raise RuntimeError(f'bad forwarded request: {forwarded}')

        response = {
            'protocol_version': '2.0',
            'message_id': 'ws-cpp-smoke-response',
            'timestamp': time.time(),
            'message_type': 'response',
            'data_type': 'system_status',
            'source': 'data_integration',
            'destination': client_id,
            'data': {'battery_level': 66},
            'metadata': {'status': 'success'},
        }
        msg = String()
        msg.data = json.dumps(response, ensure_ascii=False)
        for _ in range(3):
            node.data_response_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            await asyncio.sleep(0.05)
        routed = await recv_until(ws, lambda item: item.get('message_id') == 'ws-cpp-smoke-response')
        if routed.get('data', {}).get('battery_level') != 66:
            raise RuntimeError(f'bad routed response: {routed}')


def main():
    port = free_port()
    domain = os.environ.get('SMOKE_ROS_DOMAIN_ID', '228')
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = domain
    os.environ['ROS_DOMAIN_ID'] = domain
    cmd = (
        'unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH && '
        'source /opt/ros/jazzy/setup.bash && '
        f'source {WORKSPACE / "install" / "setup.bash"} && '
        'ros2 launch humanoid_app_gateway_runtime app_gateway_runtime.launch.py '
        'websocket_server_enable:=true '
        'data_integration_enable:=true '
        f'websocket_host:=127.0.0.1 websocket_port:={port}'
    )
    before_runtime_pids = matching_pids([APP_GATEWAY_EXE, DATA_INTEGRATION_EXE])
    proc = subprocess.Popen(
        ['bash', '-lc', cmd],
        cwd=str(WORKSPACE),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    rclpy.init(args=None)
    node = Probe()
    try:
        if not wait_for_port(port, proc):
            stdout, stderr = proc.communicate(timeout=1) if proc.poll() is not None else ('', '')
            print(f'APP gateway port did not open on {port}', file=sys.stderr)
            print(stdout, file=sys.stderr)
            print(stderr, file=sys.stderr)
            return 1
        asyncio.run(exercise(port, node))
        print(f'PASS ws_cpp_app_gateway_smoke port={port} domain={domain}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        cleanup_new_runtime_processes(before_runtime_pids)


if __name__ == '__main__':
    raise SystemExit(main())
