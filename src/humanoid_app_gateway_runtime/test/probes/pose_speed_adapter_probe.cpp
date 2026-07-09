/*
 * pose_speed_adapter_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期位姿/速度转换验证工具，构造固定 twist、covariance 和 quaternion 输入。
 * 2. 验证 FastLIO twist 坐标转换、实际速度 payload、定位质量、置信度和 yaw 计算。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本验证转换结果是否符合线上协议语义。
 */

#include <iostream>
#include <string>
#include <vector>

#include "humanoid_app_gateway_runtime/pose_speed_adapter.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

void add_string(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const std::string & value)
{
  object.AddMember(
    rapidjson::Value(key, allocator).Move(),
    rapidjson::Value(value.c_str(), allocator).Move(),
    allocator);
}

rapidjson::Value speed_to_json(
  const humanoid_app_gateway_runtime::SpeedPayload & speed,
  rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value value(rapidjson::kObjectType);
  value.AddMember("linear_x", speed.linear_x, allocator);
  value.AddMember("linear_y", speed.linear_y, allocator);
  value.AddMember("angular_z", speed.angular_z, allocator);

  rapidjson::Value linear(rapidjson::kObjectType);
  linear.AddMember("x", speed.linear_x, allocator);
  linear.AddMember("y", speed.linear_y, allocator);
  linear.AddMember("z", speed.linear_z, allocator);
  value.AddMember("linear", linear, allocator);

  rapidjson::Value angular(rapidjson::kObjectType);
  angular.AddMember("x", speed.angular_x, allocator);
  angular.AddMember("y", speed.angular_y, allocator);
  angular.AddMember("z", speed.angular_z, allocator);
  value.AddMember("angular", angular, allocator);

  value.AddMember("speed_mps", speed.speed_mps, allocator);
  value.AddMember("turn_rate_radps", speed.turn_rate_radps, allocator);
  value.AddMember("is_moving", speed.is_moving, allocator);
  add_string(value, allocator, "coordinate_frame", speed.coordinate_frame);
  add_string(value, allocator, "source_topic", speed.source_topic);
  if (!speed.source.empty()) {
    add_string(value, allocator, "source", speed.source);
  }
  add_string(value, allocator, "source_coordinate_frame", speed.source_coordinate_frame);
  value.AddMember("timestamp", speed.timestamp, allocator);
  if (speed.pose_timestamp > 0.0) {
    value.AddMember("pose_timestamp", speed.pose_timestamp, allocator);
  }
  value.AddMember("valid", speed.valid, allocator);
  if (!speed.reject_reason.empty()) {
    add_string(value, allocator, "reject_reason", speed.reject_reason);
  }
  return value;
}

}  // namespace

int main()
{
  humanoid_app_gateway_runtime::PoseSpeedAdapter adapter;

  humanoid_app_gateway_runtime::TwistValue twist;
  twist.linear_x = 0.2;
  twist.linear_y = -0.03;
  twist.linear_z = -0.4;
  twist.angular_x = 0.11;
  twist.angular_y = -0.2;
  twist.angular_z = -0.03;

  std::vector<double> covariance(36, 0.0);
  covariance[0] = 0.02;
  covariance[7] = 0.04;
  covariance[14] = 0.08;

  const humanoid_app_gateway_runtime::QuaternionValue orientation{
    0.0,
    0.0,
    0.382683432365,
    0.923879532511};

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  document.AddMember("fastlio", speed_to_json(adapter.convert_fastlio_velocity(twist, 123.0), allocator), allocator);
  document.AddMember(
    "actual",
    speed_to_json(adapter.build_actual_speed_payload(0.1234, 0.03, -0.2, 122.5, 123.0, false, "test_reason"), allocator),
    allocator);
  add_string(document, allocator, "pose_quality", adapter.estimate_pose_quality(covariance));
  document.AddMember("confidence", adapter.calculate_confidence(covariance), allocator);
  document.AddMember("yaw", adapter.yaw_from_orientation(orientation), allocator);
  document.AddMember("normalized_angle", adapter.normalize_angle(4.0), allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
