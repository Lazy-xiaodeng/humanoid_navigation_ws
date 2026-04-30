#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import asyncio
import websockets
import json
import time
from datetime import datetime

# TODO: 根据需要修改机器人的IP
ROBOT_WS_URL = "ws://127.0.0.1:8765"

def format_time(ts):
    return datetime.fromtimestamp(ts).strftime('%H:%M:%S.%f')[:-3]

async def monitor_navigation():
    print(f"[{format_time(time.time())}] 🔌 尝试连接机器人: {ROBOT_WS_URL} ...")
    try:
        async with websockets.connect(ROBOT_WS_URL, ping_interval=None) as websocket:
            print(f"[{format_time(time.time())}] ✅ WebSocket连接成功！")

            # 1. 模拟APP发起订阅 (订阅导航状态)
            sub_msg = {
                "protocol_version": "2.0",
                "message_id": f"sub_test_{int(time.time())}",
                "message_type": "subscription",
                "source": "test_monitor_script",
                "data": {
                    "action": "subscribe",
                    "data_types": ["navigation_status"],
                    "push_frequency": 5.0
                }
            }
            await websocket.send(json.dumps(sub_msg, ensure_ascii=False))
            print(f"[{format_time(time.time())}] 📡 已发送主动订阅请求...")

            print("\n=========================================================")
            print("  🎧 开始全方位监听！(已屏蔽刷屏进度，仅在同一行更新)")
            print("=========================================================\n")
            
            async for message in websocket:
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    continue

                msg_type = data.get("message_type")
                data_type = data.get("data_type")
                payload = data.get("data", {})

                # 排除连接确认和初始同步等杂音
                if msg_type != "push":
                    continue

                # ========================================================
                # 侦探逻辑 1：它会不会在 navigation_command_result 里？
                # ========================================================
                if data_type == "navigation_command_result":
                    ack_type = payload.get("ack_type", "unknown")
                    status = payload.get("status", "unknown")
                    msg_text = payload.get("message", "")
                    
                    print(f"\n[{format_time(time.time())}] 🚨 抓到了！事件出现在【navigation_command_result】里！")
                    print(f"   -> 动作类型 (ack_type): {ack_type}")
                    print(f"   -> 执行状态 (status)  : {status}")
                    print(f"   -> 附带信息 (message) : {msg_text}")
                    
                    if ack_type == "navigation_completed":
                        print("   🏆 结论：确实是这个字段！请让APP前端改监听 [navigation_command_result] 的 [ack_type]！\n")

                # ========================================================
                # 侦探逻辑 2：它会不会乖乖在 navigation_status 里？
                # ========================================================
                elif data_type == "navigation_status":
                    # 看看是不是我们之前截图里的进度刷屏事件？
                    if "navigation_time_sec" in payload or "distance_remaining" in payload:
                        # 这是进度条刷新，我们在同一行覆盖打印，不让它刷屏
                        dist = payload.get("distance_remaining", 0.0)
                        print(f"\r[{format_time(time.time())}] 🏃 实时进度 (navigation_status) -> 剩余距离: {dist:.2f}米...", end="", flush=True)
                        continue

                    # 不是进度刷屏，那是不是我们要找的 event_type 规范结构？
                    event_type = payload.get("event_type")
                    if event_type:
                        print(f"\n[{format_time(time.time())}] 🎯 抓到了！事件符合统一规范，在【navigation_status】里！")
                        print(f"   -> 事件类型 (event_type): {event_type}")
                        print(f"   -> 事件数据 (event_data): {payload.get('event_data', {})}")
                    else:
                        print(f"\n[{format_time(time.time())}] 🔔 收到未知的 navigation_status 格式: {payload}")

    except websockets.exceptions.ConnectionClosed:
        print(f"\n[{format_time(time.time())}] 🔌 连接断开。")
    except Exception as e:
        print(f"\n[{format_time(time.time())}] ❌ 发生异常: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(monitor_navigation())
    except KeyboardInterrupt:
        print("\n\n⏹️ 监听已结束。")