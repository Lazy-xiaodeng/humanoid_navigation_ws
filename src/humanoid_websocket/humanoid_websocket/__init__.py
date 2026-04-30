"""
humanoid_websocket包初始化文件
WebSocket通信模块用于人形机器人
"""

__version__ = '1.0.0'
__author__ = 'Humanoid Robotics Team'
__email__ = 'support@humanoid-robot.com'
__description__ = 'WebSocket communication package for humanoid robot'

# 导出主要类
from .websocket_server import CompleteWebSocketServer
from .websocket_client import HumanoidWebSocketClient
from .message_bridge import MessageBridge

# 导出主要函数
__all__ = [
    'CompleteWebSocketServer',
    'HumanoidWebSocketClient',
    'MessageBridge',
    'start_websocket_server',
    'start_websocket_client',
    'start_message_bridge'
]

# 辅助函数
def start_websocket_server():
    """启动WebSocket服务器"""
    from .websocket_server import main
    main()

def start_websocket_client():
    """启动WebSocket客户端"""
    from .websocket_client import main
    main()

def start_message_bridge():
    """启动消息桥接"""
    from .message_bridge import main
    main()