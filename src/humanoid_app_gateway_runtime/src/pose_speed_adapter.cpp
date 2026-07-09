/*
 * pose_speed_adapter.cpp
 *
 * 文件用途：
 * 1. 实现机器人位姿质量估计、速度死区、角度归一化和 FastLIO twist 速度坐标转换。
 * 2. 对齐现有速度死区、角度归一化、朝向角、定位质量、置信度和 FastLIO 速度转换基础语义。
 * 3. 上游：/robot_realpose、/odom。
 * 4. 下游：data_store 中的 robot_pose、robot_speed 和 APP 实时状态面板。
 */

#include "humanoid_app_gateway_runtime/pose_speed_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace humanoid_app_gateway_runtime
{

std::string PoseSpeedAdapter::name() const { return "pose_speed_adapter"; }

namespace
{

double round3(const double value)
{
  return std::round(value * 1000.0) / 1000.0;
}

}  // namespace

double PoseSpeedAdapter::apply_deadzone(
  const double value,
  const double threshold) const
{
  return std::abs(value) < threshold ? 0.0 : value;
}

double PoseSpeedAdapter::normalize_angle(const double angle) const
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double PoseSpeedAdapter::yaw_from_orientation(const QuaternionValue & orientation) const
{
  const double siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosy = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(siny, cosy);
}

std::string PoseSpeedAdapter::estimate_pose_quality(
  const std::vector<double> & covariance) const
{
  if (covariance.size() < 36) {
    return "unknown";
  }
  const double pos_variance = std::max({covariance[0], covariance[7], covariance[14]});
  if (pos_variance < 0.01) {
    return "excellent";
  }
  if (pos_variance < 0.1) {
    return "good";
  }
  if (pos_variance < 1.0) {
    return "fair";
  }
  return "poor";
}

double PoseSpeedAdapter::calculate_confidence(
  const std::vector<double> & covariance) const
{
  if (covariance.size() < 36) {
    return 0.0;
  }
  const double pos_variance = std::max({covariance[0], covariance[7], covariance[14]});
  const double confidence = 1.0 / (1.0 + pos_variance * 10.0);
  return std::max(0.0, std::min(1.0, confidence));
}

bool PoseSpeedAdapter::has_nonzero_speed(const SpeedPayload & speed_data) const
{
  return std::abs(speed_data.linear_x) > 0.0 ||
    std::abs(speed_data.linear_y) > 0.0 ||
    std::abs(speed_data.angular_z) > 0.0;
}

SpeedPayload PoseSpeedAdapter::build_actual_speed_payload(
  const double linear_x,
  const double linear_y,
  const double angular_z,
  const double pose_timestamp,
  const double timestamp,
  const bool valid,
  const std::string & reject_reason) const
{
  SpeedPayload payload;
  payload.linear_x = round3(apply_deadzone(linear_x, 0.05));
  payload.linear_y = round3(apply_deadzone(linear_y, 0.05));
  payload.angular_z = round3(apply_deadzone(angular_z, 0.05));
  payload.speed_mps = round3(std::sqrt(payload.linear_x * payload.linear_x + payload.linear_y * payload.linear_y));
  payload.turn_rate_radps = payload.angular_z;
  payload.is_moving = payload.speed_mps > 0.0 || std::abs(payload.angular_z) > 0.0;
  payload.source_topic = "/robot_realpose";
  payload.source = "pose_delta";
  payload.source_coordinate_frame = "map_pose_delta";
  payload.timestamp = timestamp;
  payload.pose_timestamp = pose_timestamp;
  payload.valid = valid;
  payload.reject_reason = reject_reason;
  return payload;
}

SpeedPayload PoseSpeedAdapter::convert_fastlio_velocity(
  const TwistValue & twist,
  const double timestamp) const
{
  SpeedPayload payload;
  payload.linear_x = round3(apply_deadzone(-twist.linear_z));
  payload.linear_y = round3(apply_deadzone(twist.linear_x));
  payload.linear_z = round3(apply_deadzone(-twist.linear_y));
  payload.angular_x = round3(apply_deadzone(-twist.angular_z));
  payload.angular_y = round3(apply_deadzone(twist.angular_x));
  payload.angular_z = round3(apply_deadzone(-twist.angular_y));
  payload.speed_mps = round3(std::sqrt(payload.linear_x * payload.linear_x + payload.linear_y * payload.linear_y));
  payload.turn_rate_radps = payload.angular_z;
  payload.is_moving = payload.speed_mps > 0.0 ||
    std::abs(payload.angular_x) > 0.0 ||
    std::abs(payload.angular_y) > 0.0 ||
    std::abs(payload.angular_z) > 0.0;
  payload.source_topic = "/odom";
  payload.source_coordinate_frame = "fast_lio_x_left_y_down_z_back";
  payload.timestamp = timestamp;
  return payload;
}

}  // namespace humanoid_app_gateway_runtime
