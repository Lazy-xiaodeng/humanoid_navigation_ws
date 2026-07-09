#!/usr/bin/env python3
"""Subscribe to /localization/ndt_status and write each JSON string as one line."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os

class NdtRecorder(Node):
    def __init__(self, output_path):
        super().__init__('ndt_recorder')
        self.out = open(output_path, 'w')
        self.count = 0
        self.sub = self.create_subscription(
            String, '/localization/ndt_status', self.callback, 10)

    def callback(self, msg):
        self.out.write(msg.data + '\n')
        self.count += 1
        if self.count % 500 == 0:
            self.get_logger().info(f"Recorded {self.count} messages")

    def close(self):
        self.out.close()
        self.get_logger().info(f"Total: {self.count} messages")

def main():
    rclpy.init()
    output = sys.argv[1] if len(sys.argv) > 1 else '/tmp/ndt_record.jsonl'
    recorder = NdtRecorder(output)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
