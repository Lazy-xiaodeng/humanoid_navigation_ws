#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class TestPushNode(Node):
    def __init__(self):
        super().__init__('test_push_node')
        # 创建发布者，向 WebSocket 服务器的整合话题发布消息
        self.publisher_ = self.create_publisher(String, '/integration/push_messages', 10)
        time.sleep(1)  # 等待发布者建立连接
        
        # 构建要推送的完整 JSON 消息 (这里以【导航完成】为例)
        test_msg = {
            "protocol_version": "2.0",
            "message_id": f"test_push_{int(time.time()*1000)}",
            "timestamp": time.time(),
            "message_type": "push",
            "data_type": "navigation_status",
            "source": "test_script",
            "destination": "all",  
            "data": {
                "event_type": "navigation_completed",    
                "event_data": {
                    "completed_waypoints": 3,
                    "total_waypoints": 3,
                    "navigation_mode": "multi_point"
                }
            },
            "metadata": {
                "status": "success",
                "error_code": "",
                "error_message": "模拟测试推送数据"
            }
        }
        
        msg = String()
        msg.data = json.dumps(test_msg, ensure_ascii=False)
        self.publisher_.publish(msg)
        self.get_logger().info(f'✅ 已成功模拟推送了 {test_msg["data"]["event_type"]} 消息到 APP!')

def main(args=None):
    rclpy.init(args=args)
    node = TestPushNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()