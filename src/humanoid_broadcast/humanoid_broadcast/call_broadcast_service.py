#!/usr/bin/env python3
import os
import sys

import rclpy
from rclpy.node import Node

from humanoid_interfaces.srv import PlayBroadcast


class BroadcastServiceClient(Node):
    def __init__(self):
        super().__init__("xiaorui_broadcast_service_client")
        self.client = self.create_client(PlayBroadcast, "/xiaorui_broadcast/play")


def main(args=None):
    del args
    rclpy.init()
    node = BroadcastServiceClient()
    try:
        timeout_sec = float(os.getenv("XIAORUI_BROADCAST_SERVICE_TIMEOUT_SEC", "120"))
        if not node.client.wait_for_service(timeout_sec=timeout_sec):
            print("/xiaorui_broadcast/play service is not available", file=sys.stderr)
            return 3

        request = PlayBroadcast.Request()
        request.text = os.getenv("XIAORUI_BROADCAST_TEXT", "")
        request.broadcast_id = os.getenv("XIAORUI_BROADCAST_ID", "")
        request.waypoint_id = os.getenv("XIAORUI_BROADCAST_WAYPOINT_ID", "")
        request.volume_percent = int(os.getenv("XIAORUI_BROADCAST_VOLUME", "72"))
        request.use_request_volume = True

        if not request.text.strip():
            print("XIAORUI_BROADCAST_TEXT is empty", file=sys.stderr)
            return 2

        future = node.client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            print("broadcast service call timed out", file=sys.stderr)
            return 4

        response = future.result()
        if not response.success:
            print(response.message or "broadcast playback failed", file=sys.stderr)
            return 5

        print(
            f"broadcast completed: duration={response.duration_sec:.3f}s "
            f"backend={response.backend} device={response.selected_device}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
