/**
 * @file lidar_undistortion.hpp
 * @brief 激光雷达点云运动畸变校正模块
 * 
 * 本文件实现了基于IMU数据的点云运动畸变校正功能
 * 参考LeGO-LOAM的实现，使用IMU的角速度和加速度信息
 * 补偿激光雷达在扫描过程中由于机器人运动导致的点云畸变
 * 
 * 工作原理：
 * 1. 机械式激光雷达（如16线、32线、64线）是逐线扫描的
 * 2. 在一个扫描周期内（通常0.1秒），机器人可能发生了移动和旋转
 * 3. 这导致同一时刻扫描的点云对应的机器人位姿不同
 * 4. 使用高频IMU数据可以估计每个点的时刻机器人的位姿
 * 5. 将所有点补偿到同一参考位姿（通常是扫描起始时刻）
 * 
 * 依赖：
 * - LeGO-LOAM的畸变校正思想
 * - PCL点云库
 * - Eigen线性代数库
 */

#ifndef LIDAR_UNDISTORTION_HPP_
#define LIDAR_UNDISTORTION_HPP_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

/**
 * @brief 激光雷达点云去畸变类
 * 
 * 该类接收高频IMU数据（角速度、加速度、姿态），
 * 积分计算每个时刻的位姿变化，
 * 然后将点云中的每个点补偿到参考时刻的位姿
 */
class LidarUndistortion
{
public:
  /**
   * @brief 构造函数
   */
  LidarUndistortion() {}

  /**
   * @brief 接收IMU数据并积分计算位姿
   * 
   * 此函数将新的IMU数据加入队列，并基于加速度积分计算
   * 速度和位移，基于角速度积分计算姿态变化
   * 
   * 参考：LeGO-LOAM featureAssociation.cpp#L431-L459
   * https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/LeGO-LOAM/src/featureAssociation.cpp#L431-L459
   * 
   * @param angular_velo 角速度向量 [x, y, z] (rad/s)
   * @param acc 加速度向量 [x, y, z] (m/s^2)
   * @param quat 姿态四元数
   * @param imu_time IMU数据时间戳（秒）
   */
  void getImu(
    Eigen::Vector3f angular_velo, Eigen::Vector3f acc, const Eigen::Quaternionf quat,
    const double imu_time /*[sec]*/)
  {
    float roll, pitch, yaw;
    Eigen::Affine3f affine(quat);
    pcl::getEulerAngles(affine, roll, pitch, yaw);  // 四元数转欧拉角

    // 更新IMU数据队列指针（循环队列）
    imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_que_length_;

    // 如果队列满了，弹出最老的元素
    if ((imu_ptr_last_ + 1) % imu_que_length_ == imu_ptr_front_) {
      imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_que_length_;
    }

    // 将IMU数据存储到队列中
    imu_time_[imu_ptr_last_] = imu_time;
    imu_roll_[imu_ptr_last_] = roll;
    imu_pitch_[imu_ptr_last_] = pitch;
    imu_yaw_[imu_ptr_last_] = yaw;
    imu_acc_x_[imu_ptr_last_] = acc.x();
    imu_acc_y_[imu_ptr_last_] = acc.y();
    imu_acc_z_[imu_ptr_last_] = acc.z();
    imu_angular_velo_x_[imu_ptr_last_] = angular_velo.x();
    imu_angular_velo_y_[imu_ptr_last_] = angular_velo.y();
    imu_angular_velo_z_[imu_ptr_last_] = angular_velo.z();

    // 将加速度转换到世界坐标系
    Eigen::Matrix3f rot = quat.toRotationMatrix();
    acc = rot * acc;
    // angular_velo = rot * angular_velo;  // 角速度通常不需要转换

    // 基于加速度和角速度积分，计算速度、位移和姿态变化
    int imu_ptr_back = (imu_ptr_last_ - 1 + imu_que_length_) % imu_que_length_;
    double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
    
    // 只有当时间间隔小于扫描周期时才积分（避免异常数据）
    if (time_diff < scan_period_) {
      // 位移积分：s = s0 + v0*t + 0.5*a*t^2
      imu_shift_x_[imu_ptr_last_] =
        imu_shift_x_[imu_ptr_back] + imu_velo_x_[imu_ptr_back] * time_diff + acc(0) * time_diff *
        time_diff * 0.5;
      imu_shift_y_[imu_ptr_last_] =
        imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * time_diff + acc(1) * time_diff *
        time_diff * 0.5;
      imu_shift_z_[imu_ptr_last_] =
        imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * time_diff + acc(2) * time_diff *
        time_diff * 0.5;

      // 速度积分：v = v0 + a*t
      imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + acc(0) * time_diff;
      imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + acc(1) * time_diff;
      imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + acc(2) * time_diff;

      // 姿态积分：angle = angle0 + angular_velo*t
      imu_angular_rot_x_[imu_ptr_last_] = imu_angular_rot_x_[imu_ptr_back] + angular_velo(0) *
        time_diff;
      imu_angular_rot_y_[imu_ptr_last_] = imu_angular_rot_y_[imu_ptr_back] + angular_velo(1) *
        time_diff;
      imu_angular_rot_z_[imu_ptr_last_] = imu_angular_rot_z_[imu_ptr_back] + angular_velo(2) *
        time_diff;
    }
  }

  /**
   * @brief 对点云进行运动畸变校正
   * 
   * 此函数遍历点云中的每个点，根据点的角度估算其扫描时刻，
   * 然后使用IMU数据将该点补偿到参考位姿（扫描起始时刻）
   * 
   * 参考：LeGO-LOAM featureAssociation.cpp#L491-L619
   * https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/LeGO-LOAM/src/featureAssociation.cpp#L491-L619
   * 
   * @param cloud 输入点云（会被就地修改）
   * @param scan_time 扫描时间戳（秒）
   */
  void adjustDistortion(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const double scan_time /*[sec]*/)
  {
    bool half_passed = false;  // 标记是否经过角度跳变点
    int cloud_size = cloud->points.size();

    // 计算扫描起始和结束的角度
    float start_ori = -std::atan2(cloud->points[0].y, cloud->points[0].x);
    float end_ori = -std::atan2(cloud->points[cloud_size - 1].y, cloud->points[cloud_size - 1].x);
    
    // 处理角度跨越±π边界的情况
    if (end_ori - start_ori > 3 * M_PI) {
      end_ori -= 2 * M_PI;
    } else if (end_ori - start_ori < M_PI) {
      end_ori += 2 * M_PI;
    }
    float ori_diff = end_ori - start_ori;  // 总扫描角度范围

    // 临时变量
    Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
    Eigen::Vector3f shift_from_start;
    Eigen::Matrix3f r_s_i, r_c;
    Eigen::Vector3f adjusted_p;
    float ori_h;
    
    // 遍历每个点进行畸变校正
    for (int i = 0; i < cloud_size; ++i) {
      pcl::PointXYZI & p = cloud->points[i];
      ori_h = -std::atan2(p.y, p.x);  // 计算当前点的角度
      
      // ========== 角度连续性处理 ==========
      if (!half_passed) {
        // 处理角度跳变
        if (ori_h < start_ori - M_PI * 0.5) {
          ori_h += 2 * M_PI;
        } else if (ori_h > start_ori + M_PI * 1.5) {
          ori_h -= 2 * M_PI;
        }

        // 检查是否经过跳变点
        if (ori_h - start_ori > M_PI) {
          half_passed = true;
        }
      } else {
        ori_h += 2 * M_PI;
        if (ori_h < end_ori - 1.5 * M_PI) {
          ori_h += 2 * M_PI;
        } else if (ori_h > end_ori + 0.5 * M_PI) {
          ori_h -= 2 * M_PI;
        }
      }

      // ========== 计算相对时间 ==========
      // 根据角度线性插值，估算该点的扫描时刻
      float rel_time = (ori_h - start_ori) / ori_diff * scan_period_;

      // ========== 查找并插值IMU数据 ==========
      if (imu_ptr_last_ > 0) {
        imu_ptr_front_ = imu_ptr_last_iter_;
        
        // 在IMU队列中查找扫描时刻对应的IMU数据
        while (imu_ptr_front_ != imu_ptr_last_) {
          if (scan_time + rel_time > imu_time_[imu_ptr_front_]) {
            break;
          }
          imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_que_length_;
        }

        // 根据找到的IMU数据计算当前点的位姿
        if (scan_time + rel_time > imu_time_[imu_ptr_front_]) {
          // 使用当前IMU帧的数据
          rpy_cur(0) = imu_roll_[imu_ptr_front_];
          rpy_cur(1) = imu_pitch_[imu_ptr_front_];
          rpy_cur(2) = imu_yaw_[imu_ptr_front_];
          shift_cur(0) = imu_shift_x_[imu_ptr_front_];
          shift_cur(1) = imu_shift_y_[imu_ptr_front_];
          shift_cur(2) = imu_shift_z_[imu_ptr_front_];
          velo_cur(0) = imu_velo_x_[imu_ptr_front_];
          velo_cur(1) = imu_velo_y_[imu_ptr_front_];
          velo_cur(2) = imu_velo_z_[imu_ptr_front_];
        } else {
          // 在两个IMU帧之间线性插值
          int imu_ptr_back = (imu_ptr_front_ - 1 + imu_que_length_) % imu_que_length_;
          float ratio_front = (scan_time + rel_time - imu_time_[imu_ptr_back]) /
            (imu_time_[imu_ptr_front_] - imu_time_[imu_ptr_back]);
          float ratio_back = 1.0 - ratio_front;
          
          // 欧拉角插值
          rpy_cur(0) = imu_roll_[imu_ptr_front_] * ratio_front + imu_roll_[imu_ptr_back] *
            ratio_back;
          rpy_cur(1) = imu_pitch_[imu_ptr_front_] * ratio_front + imu_pitch_[imu_ptr_back] *
            ratio_back;
          rpy_cur(2) = imu_yaw_[imu_ptr_front_] * ratio_front + imu_yaw_[imu_ptr_back] * ratio_back;
          
          // 位移插值
          shift_cur(0) = imu_shift_x_[imu_ptr_front_] * ratio_front + imu_shift_x_[imu_ptr_back] *
            ratio_back;
          shift_cur(1) = imu_shift_y_[imu_ptr_front_] * ratio_front + imu_shift_y_[imu_ptr_back] *
            ratio_back;
          shift_cur(2) = imu_shift_z_[imu_ptr_front_] * ratio_front + imu_shift_z_[imu_ptr_back] *
            ratio_back;
          
          // 速度插值
          velo_cur(0) = imu_velo_x_[imu_ptr_front_] * ratio_front + imu_velo_x_[imu_ptr_back] *
            ratio_back;
          velo_cur(1) = imu_velo_y_[imu_ptr_front_] * ratio_front + imu_velo_y_[imu_ptr_back] *
            ratio_back;
          velo_cur(2) = imu_velo_z_[imu_ptr_front_] * ratio_front + imu_velo_z_[imu_ptr_back] *
            ratio_back;
        }

        // 根据欧拉角构造旋转矩阵
        r_c = (
          Eigen::AngleAxisf(rpy_cur(2), Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(rpy_cur(1), Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX())
          ).toRotationMatrix();

        // 对于第一个点，记录参考位姿
        if (i == 0) {
          rpy_start = rpy_cur;
          shift_start = shift_cur;
          velo_start = velo_cur;
          r_s_i = r_c.inverse();  // 参考位姿的逆旋转矩阵
        } else {
          // ========== 畸变校正 ==========
          // 计算从参考时刻到当前时刻的位移变化
          shift_from_start = shift_cur - shift_start - velo_start * rel_time;
          
          // 将点从当前位姿补偿到参考位姿
          // 公式：P_adjusted = R_s_inv * (R_c * P + shift_from_start)
          adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
          p.x = adjusted_p.x();
          p.y = adjusted_p.y();
          p.z = adjusted_p.z();
        }
      }
      imu_ptr_last_iter_ = imu_ptr_front_;
    }
  }

  /**
   * @brief 设置激光雷达扫描周期
   * 
   * @param scan_period 扫描周期（秒），如10Hz雷达为0.1
   */
  void setScanPeriod(const double scan_period /*[sec]*/)
  {
    scan_period_ = scan_period;
  }

private:
  double scan_period_{0.1};  ///< 雷达扫描周期（秒）
  static const int imu_que_length_{200};  ///< IMU队列最大长度
  int imu_ptr_front_{0}, imu_ptr_last_{-1}, imu_ptr_last_iter_{0};  ///< IMU队列指针

  // IMU数据存储数组（循环队列）
  std::array<double, imu_que_length_> imu_time_;         ///< IMU时间戳
  std::array<float, imu_que_length_> imu_roll_;          ///< IMU横滚角
  std::array<float, imu_que_length_> imu_pitch_;         ///< IMU俯仰角
  std::array<float, imu_que_length_> imu_yaw_;           ///< IMU偏航角

  std::array<float, imu_que_length_> imu_acc_x_;         ///< X轴加速度
  std::array<float, imu_que_length_> imu_acc_y_;         ///< Y轴加速度
  std::array<float, imu_que_length_> imu_acc_z_;         ///< Z轴加速度
  
  std::array<float, imu_que_length_> imu_velo_x_;        ///< X轴速度（积分得到）
  std::array<float, imu_que_length_> imu_velo_y_;        ///< Y轴速度（积分得到）
  std::array<float, imu_que_length_> imu_velo_z_;        ///< Z轴速度（积分得到）
  
  std::array<float, imu_que_length_> imu_shift_x_;       ///< X轴位移（积分得到）
  std::array<float, imu_que_length_> imu_shift_y_;       ///< Y轴位移（积分得到）
  std::array<float, imu_que_length_> imu_shift_z_;       ///< Z轴位移（积分得到）

  std::array<float, imu_que_length_> imu_angular_velo_x_;  ///< X轴角速度
  std::array<float, imu_que_length_> imu_angular_velo_y_;  ///< Y轴角速度
  std::array<float, imu_que_length_> imu_angular_velo_z_;  ///< Z轴角速度
  
  std::array<float, imu_que_length_> imu_angular_rot_x_;   ///< X轴累积积转角
  std::array<float, imu_que_length_> imu_angular_rot_y_;   ///< Y轴累积积转角
  std::array<float, imu_que_length_> imu_angular_rot_z_;   ///< Z轴累积积转角
};

#endif  // LIDAR_UNDISTORTION_HPP_
