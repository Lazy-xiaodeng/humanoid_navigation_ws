#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR-IMU 时间偏差精准自动校准脚本 v3.0

【修复说明】
  旧版用"点云点数"作为激光信号，点数几乎不随运动变化，导致相关性极低。
  新版改用"点云质心位移速度"作为激光信号，与IMU加速度高度相关。

【原理】
  - IMU 信号：加速度模长的变化量（去除重力后的动态加速度）
  - LiDAR 信号：相邻帧点云质心的位移速度（反映传感器运动）
  - 两者通过互相关找到时间对齐的偏移量

【使用方法】
  终端1: ros2 launch humanoid_navigation2 mapping.launch.py
  终端2: ros2 run humanoid_navigation2 lidar_imu_time_calibrator
  让机器人做明显运动（摇晃/行走），约60 秒后 Ctrl+C 查看结果
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile, ReliabilityPolicy,HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy import signal, interpolate
from scipy.signal import butter, filtfilt
import signal as sys_signal
import sys
import time
from collections import deque

class LidarImuTimeCalibratorV3(Node):
    """
    LiDAR-IMU 时间偏差校准节点 v3.0
    使用点云质心位移速度作为激光信号，大幅提升相关性
    """

    def __init__(self):
        super().__init__('lidar_imu_time_calibrator')

        # ===== 数据存储 =====
        # IMU：存储时间戳 + 动态加速度（去重力后）
        self.imu_timestamps= []
        self.imu_acc_dynamic = []   # 动态加速度模长（去除重力均值后）

        # LiDAR：存储时间戳 + 质心位移速度
        self.lidar_timestamps    = []
        self.lidar_centroid_vel= []   # 相邻帧质心位移速度（m/s）
        self._prev_centroid      = None
        self._prev_lidar_time    = None

        # ===== 重力估计（用于去除重力分量）=====
        self._gravity_buffer = deque(maxlen=500)   # 用最近 500 帧估计重力
        self._gravity_norm= 9.81                # 初始重力估计

        # ===== 校准结果 =====
        self.offset_estimates= []
        self.confidence_list= []

        # ===== 参数 =====
        self.max_imu_samples= 50000# IMU 最多保留样本数
        self.max_lidar_samples  = 3000    # LiDAR 最多保留样本数
        self.min_lidar_frames   = 30      # 最少需要多少帧 LiDAR 才开始计算
        self.estimate_interval  = 30      # 每30 帧 LiDAR 计算一次
        self.max_search_sec     = 0.5     # 最大搜索偏差 ±0.5 秒
        self.resample_freq      = 50.0    # 重采样频率（Hz）

        # ===== 统计 =====
        self.start_time  = time.time()
        self.frame_count = 0

        # ===== QoS 配置 =====
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ===== 订阅原始话题 =====
        self.imu_sub = self.create_subscription(
            Imu, '/airy_imu', self.imu_callback, sensor_qos
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/airy_points', self.lidar_callback, sensor_qos
        )

        # ===== 退出信号 =====
        sys_signal.signal(sys_signal.SIGINT, self.shutdown_handler)
        self._print_banner()

    # =========================================================
    #启动提示
    # =========================================================
    def _print_banner(self):
        self.get_logger().info("\n" + "=" * 62)
        self.get_logger().info("  LiDAR-IMU 时间偏差校准脚本 v3.0")
        self.get_logger().info("=" * 62)
        self.get_logger().info("  订阅话题:")
        self.get_logger().info("    IMU:/airy_imu    (原始 IMU)")
        self.get_logger().info("    LiDAR:  /airy_points (原始点云)")
        self.get_logger().info("  信号方案:")
        self.get_logger().info("    IMU→ 动态加速度模长（去重力）")
        self.get_logger().info("    LiDAR  → 点云质心位移速度")
        self.get_logger().info("-" * 62)
        self.get_logger().info("  请让机器人做以下运动（持续 60 秒）：")
        self.get_logger().info("    ✅ 推荐：原地左右快速摇晃躯干（幅度越大越好）")
        self.get_logger().info("    ✅ 推荐：机器人行走 10~20 步")
        self.get_logger().info("    ❌ 避免：完全静止不动")
        self.get_logger().info("-" * 62)
        self.get_logger().info("  Ctrl+C 停止并输出最终结果")
        self.get_logger().info("=" * 62 + "\n")

    # =========================================================
    #  IMU 回调
    # =========================================================
    def imu_callback(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        acc_norm = float(np.sqrt(ax*ax + ay*ay + az*az))

        # 用滑动窗口估计重力（静止时的均值）
        self._gravity_buffer.append(acc_norm)
        if len(self._gravity_buffer) >= 100:
            self._gravity_norm = float(np.median(self._gravity_buffer))

        # 动态加速度 = 总加速度模长 - 重力模长
        # 机器人运动时，动态加速度会有明显波动
        acc_dynamic = abs(acc_norm - self._gravity_norm)

        self.imu_timestamps.append(t)
        self.imu_acc_dynamic.append(acc_dynamic)

        # 限制样本量
        if len(self.imu_timestamps) > self.max_imu_samples:
            keep = self.max_imu_samples // 2
            self.imu_timestamps  = self.imu_timestamps[-keep:]
            self.imu_acc_dynamic = self.imu_acc_dynamic[-keep:]

    # =========================================================
    #  LiDAR 回调
    # =========================================================
    def lidar_callback(self, msg: PointCloud2):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        try:
            # 读取点云（只取xyz，跳过强度以提高速度）
            pts = pc2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )

            if pts is None or len(pts) < 10:
                return

            # 计算质心
            centroid = np.mean(pts[:, :3], axis=0)
        
        except Exception as e:
            self.get_logger().warn(
                f"点云读取失败: {e}", throttle_duration_sec=2.0
            )
            return

        # 计算质心位移速度（m/s）
        if self._prev_centroid is not None and self._prev_lidar_time is not None:
            dt = t - self._prev_lidar_time
            if dt > 1e-6:
                displacement = float(np.linalg.norm(centroid - self._prev_centroid))
                centroid_vel = displacement / dt# m/s

                self.lidar_timestamps.append(t)
                self.lidar_centroid_vel.append(centroid_vel)

        self._prev_centroid   = centroid
        self._prev_lidar_time = t

        self.frame_count += 1

        # 限制样本量
        if len(self.lidar_timestamps) > self.max_lidar_samples:
            keep = self.max_lidar_samples // 2
            self.lidar_timestamps   = self.lidar_timestamps[-keep:]
            self.lidar_centroid_vel = self.lidar_centroid_vel[-keep:]# 每隔 N 帧计算一次
        if (self.frame_count >= self.min_lidar_frames and
                self.frame_count % self.estimate_interval == 0):
            self.compute_time_offset()

    # =========================================================
    #  核心：互相关计算时间偏差
    # =========================================================
    def compute_time_offset(self):
        n_imu   = len(self.imu_timestamps)
        n_lidar = len(self.lidar_timestamps)

        if n_imu < 100 or n_lidar < 20:
            self.get_logger().info(
                f"数据不足，继续收集... "
                f"(IMU:{n_imu}, LiDAR:{n_lidar})"
            )
            return

        try:
            # ── 1. 确定公共时间范围 ──────────────────────────────
            t0 = max(self.imu_timestamps[0],self.lidar_timestamps[0])
            t1 = min(self.imu_timestamps[-1],  self.lidar_timestamps[-1])
            duration = t1 - t0

            if duration < 8.0:
                self.get_logger().info(
                    f"有效时间窗口太短({duration:.1f}s < 8s)，继续收集..."
                )
                return

            # ── 2. 建立统一时间轴 ────────────────────────────────
            t_uniform = np.arange(t0, t1, 1.0 / self.resample_freq)

            # ── 3. 重采样 IMU 信号 ───────────────────────────────
            imu_t = np.array(self.imu_timestamps)
            imu_s = np.array(self.imu_acc_dynamic)

            mask = (imu_t >= t0) & (imu_t <= t1)
            imu_t, imu_s = imu_t[mask], imu_s[mask]

            if len(imu_t) < 20:
                return

            # 去重
            _, uid = np.unique(imu_t, return_index=True)
            imu_t, imu_s = imu_t[uid], imu_s[uid]

            imu_f = interpolate.interp1d(
                imu_t, imu_s, kind='linear', fill_value='extrapolate'
            )
            imu_resampled = imu_f(t_uniform)

            # ── 4. 重采样 LiDAR 信号 ─────────────────────────────
            lidar_t = np.array(self.lidar_timestamps)
            lidar_s = np.array(self.lidar_centroid_vel)

            mask = (lidar_t >= t0) & (lidar_t <= t1)
            lidar_t, lidar_s = lidar_t[mask], lidar_s[mask]

            if len(lidar_t) < 10:
                return

            _, uid = np.unique(lidar_t, return_index=True)
            lidar_t, lidar_s = lidar_t[uid], lidar_s[uid]

            lidar_f = interpolate.interp1d(
                lidar_t, lidar_s, kind='linear', fill_value='extrapolate'
            )
            lidar_resampled = lidar_f(t_uniform)

            # ── 5. 低通滤波（去除高频噪声）──────────────────────
            # 截止频率 5Hz，采样率 50Hz
            b, a = butter(4, 5.0 / (self.resample_freq / 2), btype='low')
            imu_filtered= filtfilt(b, a, imu_resampled)
            lidar_filtered = filtfilt(b, a, lidar_resampled)

            # ── 6. 去均值 + 归一化 ───────────────────────────────
            imu_sig= imu_filtered   - np.mean(imu_filtered)
            lidar_sig = lidar_filtered - np.mean(lidar_filtered)

            imu_std   = np.std(imu_sig)
            lidar_std = np.std(lidar_sig)

            if imu_std < 1e-4 or lidar_std < 1e-4:
                self.get_logger().warn(
                    "⚠️  信号变化不足！请让机器人做更大幅度的运动...",
                    throttle_duration_sec=3.0
                )
                return

            imu_sig   /= imu_std
            lidar_sig /= lidar_std

            # ── 7. 互相关 ────────────────────────────────────────
            max_lag = int(self.max_search_sec * self.resample_freq)

            corr = signal.correlate(lidar_sig, imu_sig, mode='full')
            lags = signal.correlation_lags(
                len(lidar_sig), len(imu_sig), mode='full'
            )

            # 限制搜索范围
            valid = np.abs(lags) <= max_lag
            corr_v = corr[valid]
            lags_v = lags[valid]

            # 找峰值
            peak_idx = np.argmax(np.abs(corr_v))
            peak_lag = lags_v[peak_idx]
            time_offset = float(peak_lag) / self.resample_freq

            # ── 8. 置信度计算（改进版）──────────────────────────
            # 用峰值与次峰值之比衡量置信度（越大越好）
            peak_val = np.abs(corr_v[peak_idx])

            # 排除峰值附近 ±10 个点，找次峰值
            mask_near_peak = np.ones(len(corr_v), dtype=bool)
            lo = max(0, peak_idx - 10)
            hi = min(len(corr_v), peak_idx + 10)
            mask_near_peak[lo:hi] = False

            if mask_near_peak.any():
                second_peak = np.max(np.abs(corr_v[mask_near_peak]))
                confidence  = peak_val / (second_peak + 1e-6)
            else:
                confidence = 1.0

            # ── 9. 记录结果 ──────────────────────────────────────
            self.offset_estimates.append(time_offset)
            self.confidence_list.append(confidence)

            elapsed = time.time() - self.start_time

            # 置信度评级
            if confidence >= 3.0:
                conf_label = "✅ 高"
            elif confidence >= 1.5:
                conf_label = "⚠️中"
            else:
                conf_label = "❌ 低"

            self.get_logger().info(
                f"\n{'─'*62}\n"
                f"第 {len(self.offset_estimates)} 次估计结果：\n"
                f"  时间偏差:{time_offset*1000:+8.3f} ms\n"
                f"                ({time_offset:+.6f} 秒)\n"
                f"  峰次比置信度: {confidence:.3f}  [{conf_label}]\n"
                f"  有效数据范围: {duration:.1f} 秒\n"
                f"  运行时间:     {elapsed:.1f} 秒\n"
                f"{'─'*62}"
            )

            # 每 5 次输出统计
            if len(self.offset_estimates) >= 5:
                self._print_statistics()

        except Exception as e:
            self.get_logger().error(f"计算失败: {e}")
            import traceback
            traceback.print_exc()

    # =========================================================
    #  统计分析
    # =========================================================
    def _print_statistics(self):
        estimates= np.array(self.offset_estimates)
        confidences = np.array(self.confidence_list)

        # 只使用置信度 >= 1.5 的估计
        high_conf_mask = confidences >= 1.5
        if high_conf_mask.sum() >= 3:
            good_estimates = estimates[high_conf_mask]
            note = f"（仅使用置信度≥1.5的 {high_conf_mask.sum()} 次）"
        else:
            good_estimates = estimates
            note = "（置信度普遍偏低，建议增加运动幅度）"

        mean_v= np.mean(good_estimates)
        median_v = np.median(good_estimates)
        std_v    = np.std(good_estimates)

        # 去离群值
        mask= np.abs(good_estimates - mean_v) < 2* std_v
        final_v  = np.mean(good_estimates[mask]) if mask.sum() > 0 else mean_v

        self.get_logger().info(
            f"\n{'='*62}\n"
            f"统计分析（共 {len(estimates)} 次估计）{note}\n"
            f"{'='*62}\n"
            f"  平均值:   {mean_v*1000:+8.3f} ms\n"
            f"  中位数:   {median_v*1000:+8.3f} ms\n"
            f"  标准差:   {std_v*1000:8.3f} ms\n"
            f"{'─'*62}\n"
            f"  推荐设置值: {final_v:+.6f} 秒\n"
            f"{'='*62}\n"
            f"  time_sync_en: false\n"
            f"time_offset_lidar_to_imu: {final_v:.6f}\n"
            f"{'='*62}"
        )

    # =========================================================
    #  最终结果
    # =========================================================
    def print_final_result(self):
        if not self.offset_estimates:
            print("\n" + "=" * 70)
            print("  ❌ 没有收集到足够数据，无法估计时间偏差")
            print("  排查建议：")
            print("    1. 确认/airy_imu 和 /airy_points 话题正常发布")
            print("       ros2 topic hz /airy_imu")
            print("       ros2 topic hz /airy_points")
            print("    2. 确保机器人有明显运动（摇晃/行走）")
            print("    3. 运行时间至少 60 秒")
            print("=" * 70)
            return

        estimates   = np.array(self.offset_estimates)
        confidences = np.array(self.confidence_list)
        elapsed     = time.time() - self.start_time

        # 优先使用高置信度结果
        high_conf_mask = confidences >= 1.5
        if high_conf_mask.sum() >= 3:
            good_estimates = estimates[high_conf_mask]
        else:
            good_estimates = estimates

        mean_v   = np.mean(good_estimates)
        median_v = np.median(good_estimates)
        std_v    = np.std(good_estimates)

        # 去离群值
        mask    = np.abs(good_estimates - mean_v) < 2 * std_v
        final_v = np.mean(good_estimates[mask]) if mask.sum() > 0 else median_v

        print("\n")
        print("=" * 70)
        print("LiDAR-IMU 时间偏差校准 v3.0 - 最终结果")
        print("=" * 70)
        print(f"  总运行时间:{elapsed:.1f} 秒")
        print(f"  IMU 数据量:       {len(self.imu_timestamps)} 帧")
        print(f"  LiDAR 数据量:     {self.frame_count} 帧")
        print(f"  估计次数:         {len(estimates)} 次")
        print(f"  高置信度次数:     {high_conf_mask.sum()} 次")
        print("─" * 70)
        print(f"  各次估计值(ms):  "
              f"{', '.join([f'{v*1000:+.1f}' for v in estimates])}")
        print(f"  各次置信度:       "
              f"{', '.join([f'{c:.2f}' for c in confidences])}")
        print("─" * 70)
        print(f"  平均值:           {mean_v*1000:+8.3f} ms")
        print(f"  中位数:           {median_v*1000:+8.3f} ms")
        print(f"  标准差:           {std_v*1000:8.3f} ms")
        print("─" * 70)
        print(f"★ 最终推荐偏差值: {final_v*1000:+.3f} ms ({final_v:+.6f} 秒)")
        print("=" * 70)
        print("")
        print("  请将以下内容写入 robosenseAiry.yaml：")
        print("")
        print("    common:")
        print("      time_sync_en: false")
        print(f"      time_offset_lidar_to_imu: {final_v:.6f}")
        print("")# 综合评级
        if std_v * 1000 < 10.0 and high_conf_mask.sum() >= 3:
            print("  ✅ 置信度：高（结果稳定可信）")
        elif std_v * 1000 < 30.0:
            print("  ⚠️  置信度：中（建议增加运动幅度后重新校准）")
        else:
            print("  ❌ 置信度：低（结果不稳定，请参考下方建议）")
            print("")
            print("  【提升置信度的建议】")
            print("  1. 让机器人做更大幅度的快速运动（如快速左右摆动）")
            print("  2. 运动时间延长到 90秒以上")
            print("  3. 检查 /airy_imu 的linear_acceleration 是否有真实数据：")
            print("     ros2 topic echo /airy_imu --once")
            print("  4. 若IMU 数据全为 0，说明 IMU 驱动未正常工作")
            print("")
        print("  【偏差含义说明】")
        print("  正值：LiDAR 时间戳比 IMU 晚（LiDAR 超前于 IMU）")
        print("  负值：LiDAR 时间戳比 IMU 早（IMU 超前于 LiDAR）")
        print("=" * 70)

    # =========================================================
    #  退出处理
    # =========================================================
    def shutdown_handler(self, sig, frame):
        self.get_logger().info("收到退出信号，正在计算最终结果...")
        self.print_final_result()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = LidarImuTimeCalibratorV3()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.print_final_result()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()