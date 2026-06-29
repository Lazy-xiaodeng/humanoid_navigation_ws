/*
 * walk_velocity_controller_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期 /cmd_vel 行走控制逻辑验证工具，构造固定速度输入和机器人状态。
 * 2. 验证速度缓存启动、Walk 状态发送、Menu 状态拦截、动作执行拦截、零速度停车退出。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本比较速度控制决策是否与 既有机器人网关协议一致。
 */

#include <iostream>

#include "humanoid_robot_gateway_runtime/walk_velocity_controller.hpp"
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

void add_decision(
  rapidjson::Value & array,
  rapidjson::Document::AllocatorType & allocator,
  const char * label,
  const humanoid_robot_gateway_runtime::WalkVelocityTickDecision & decision)
{
  rapidjson::Value object(rapidjson::kObjectType);
  add_string(object, allocator, "label", label);
  object.AddMember("timer_running", decision.timer_running, allocator);
  object.AddMember("should_send", decision.should_send, allocator);
  object.AddMember("should_stop_timer", decision.should_stop_timer, allocator);
  object.AddMember("blocked_by_motion", decision.blocked_by_motion, allocator);
  object.AddMember("blocked_by_robot_state", decision.blocked_by_robot_state, allocator);
  add_string(object, allocator, "title", decision.title);
  add_string(object, allocator, "reason", decision.reason);

  rapidjson::Document data_doc;
  if (!decision.data_json.empty()) {
    data_doc.Parse(decision.data_json.c_str());
  } else {
    data_doc.SetObject();
  }
  object.AddMember("data", rapidjson::Value(data_doc, allocator), allocator);
  array.PushBack(object, allocator);
}

}  // namespace

int main()
{
  humanoid_robot_gateway_runtime::RobotGatewayConfig config;
  config.walk_velocity_send_enable = true;
  config.walk_velocity_send_rate_hz = 50.0;

  humanoid_robot_gateway_runtime::WalkVelocityController controller(config);

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  document.AddMember("interval", controller.send_interval_sec(), allocator);

  rapidjson::Value decisions(rapidjson::kArrayType);

  document.AddMember("started", controller.on_cmd_vel(1.2345, -2.0, 0.12345), allocator);
  add_decision(decisions, allocator, "walk_send", controller.evaluate_tick("WALK", false));

  controller.on_cmd_vel(0.5, 0.0, 0.0);
  add_decision(decisions, allocator, "menu_block", controller.evaluate_tick("MENU", false));

  controller.on_cmd_vel(0.5, 0.0, 0.0);
  add_decision(decisions, allocator, "motion_block", controller.evaluate_tick("WALK", true));

  controller.on_cmd_vel(0.0, 0.0, 0.0);
  add_decision(decisions, allocator, "zero_stop", controller.evaluate_tick("WALK", false));
  document.AddMember("final_timer_running", controller.timer_running(), allocator);

  document.AddMember("decisions", decisions, allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
