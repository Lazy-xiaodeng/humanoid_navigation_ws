/*
 * pose_speed_adapter.hpp
 *
 * 文件用途：
 * 1. 负责 /robot_realpose 和 /odom 到 robot_pose、robot_speed 的转换。
 * 2. 保留速度死区、导航未激活时速度抑制、pose 差分估速等语义边界。
 * 3. 调试时需要重点对比导航页实时速度显示是否一致。
 * 4. 上游：定位链路发布的 /robot_realpose 和 FastLIO/Nav2 发布的 /odom。
 * 5. 下游：data_store 中的 robot_pose、robot_speed 和 APP 实时状态面板。
 */

#pragma once

#include <array>
#include <string>
#include <vector>

namespace humanoid_app_gateway_runtime
{

struct QuaternionValue
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct TwistValue
{
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
};

struct SpeedPayload
{
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
  double speed_mps{0.0};
  double turn_rate_radps{0.0};
  bool is_moving{false};
  std::string coordinate_frame{"base_link_standard"};
  std::string source_topic;
  std::string source;
  std::string source_coordinate_frame;
  double timestamp{0.0};
  double pose_timestamp{0.0};
  bool valid{true};
  std::string reject_reason;
};

class PoseSpeedAdapter
{
public:
  std::string name() const;

  double apply_deadzone(double value, double threshold = 0.01) const;
  double normalize_angle(double angle) const;
  double yaw_from_orientation(const QuaternionValue & orientation) const;

  std::string estimate_pose_quality(const std::vector<double> & covariance) const;
  double calculate_confidence(const std::vector<double> & covariance) const;

  bool has_nonzero_speed(const SpeedPayload & speed_data) const;

  SpeedPayload build_actual_speed_payload(
    double linear_x,
    double linear_y,
    double angular_z,
    double pose_timestamp,
    double timestamp,
    bool valid = true,
    const std::string & reject_reason = "") const;

  SpeedPayload convert_fastlio_velocity(
    const TwistValue & twist,
    double timestamp) const;
};

}  // namespace humanoid_app_gateway_runtime
