#!/usr/bin/env python3
"""
精确测量 /airy_imu 和 /airy_points 时间戳差值
不需要机器人运动，静止即可
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu
import numpy as np

class TimestampMeasurer(Node):
    def __init__(self):
        super().__init__('timestamp_measurer')

        self.imu_times= []
        self.lidar_times = []
        self.diffs       = []

        self.last_imu_t= None
        self.last_lidar_t = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Imu, '/airy_imu',self.imu_cb, sensor_qos)
        self.create_subscription(PointCloud2, '/airy_points',
            self.lidar_cb, sensor_qos)

        # 每 5 秒打印一次统计
        self.create_timer(5.0, self.print_stats)
        self.get_logger().info("开始测量时间戳差值，机器人保持静止即可...")
        self.get_logger().info("格式：IMU时间戳 - LiDAR时间戳 = 差值")

    def imu_cb(self, msg):
        self.last_imu_t = msg.header.stamp.sec + \
                          msg.header.stamp.nanosec * 1e-9

    def lidar_cb(self, msg):
        self.last_lidar_t = msg.header.stamp.sec + \
                            msg.header.stamp.nanosec * 1e-9
        
        if self.last_imu_t is None:
            return

        # 找最近的 IMU 时间戳与当前 LiDAR 时间戳的差
        diff = self.last_imu_t - self.last_lidar_t
        self.diffs.append(diff)

        self.get_logger().info(
            f"IMU={self.last_imu_t:.6f}  "
            f"LiDAR={self.last_lidar_t:.6f}  "
            f"差值(IMU-LiDAR)={diff*1000:+.1f} ms"
        )

    def print_stats(self):
        if len(self.diffs) < 3:
            return

        arr = np.array(self.diffs)
        self.get_logger().info(
            f"\n{'='*55}\n"
            f"统计（共 {len(arr)} 次）\n"
            f"均值:{np.mean(arr)*1000:+.3f} ms\n"
            f"  中位数:{np.median(arr)*1000:+.3f} ms\n"
            f"  标准差:{np.std(arr)*1000:.3f} ms\n"
            f"  最小值:{np.min(arr)*1000:+.3f} ms\n"
            f"  最大值:{np.max(arr)*1000:+.3f} ms\n"
            f"{'='*55}\n"
            f"  → time_offset_lidar_to_imu 建议值: "
            f"{-np.median(arr):.6f} 秒\n"
            f"{'='*55}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TimestampMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()