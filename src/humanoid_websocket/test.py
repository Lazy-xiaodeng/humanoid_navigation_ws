import asyncio
import websockets
import json

async def test_websocket():
    uri = "ws://192.168.175.128:8765"
    print(f"正在连接 {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("连接成功!")
            
            # 接收服务器的欢迎消息
            response = await websocket.recv()
            print(f"收到服务器消息: {response}")
            
            # 发送一条测试消息
            test_msg = {"type": "test", "data": "Hello from client"}
            await websocket.send(json.dumps(test_msg))
            
            # 接收回显
            echo = await websocket.recv()
            print(f"收到回显: {echo}")
            
            # 等待一段时间，看看连接是否保持
            await asyncio.sleep(5)
            print("连接保持正常！")
            
    except Exception as e:
        print(f"连接失败: {e}")

if __name__ == "__main__":
    asyncio.run(test_websocket())