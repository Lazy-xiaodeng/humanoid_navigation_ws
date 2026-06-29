/*
 * business_command_router.hpp
 *
 * 文件用途：
 * 1. 负责 APP business_command 到导航、地图、机器人控制、表情、系统命令 ROS topic 的路由。
 * 2. 对齐现有 APP business_command 入口和各类业务命令路由语义。
 * 3. 本模块只做命令分类和字段归一化，真实发布由 Node 外壳注入 publisher 完成。
 * 4. 上游：app_protocol 校验后的 APP business_command。
 * 5. 下游：/app/navigation_command、/app/map_command、/app/robot_control、/robot/facial_raw_cmd、/initialpose。
 */

#pragma once

#include <string>

namespace humanoid_app_gateway_runtime
{

enum class BusinessCommandTarget
{
  WaypointCommand,
  NavigationCommand,
  MapCommand,
  RobotControl,
  FacialRawCommand,
  InitialPose,
  SystemCommand,
  BroadcastVolumeService,
  Error
};

struct BusinessCommandRouteResult
{
  bool ok{false};
  BusinessCommandTarget target{BusinessCommandTarget::Error};
  std::string error;
  std::string payload_json;
  std::string ack_json;
  std::string ack_request_id;
};

class BusinessCommandRouter
{
public:
  std::string name() const;

  // 保持线上协议 handle_business_command() 入口路由和各类 handle_xxx 的字段整理。
  BusinessCommandRouteResult route_business_command(
    const std::string & message_json,
    const std::string & client_id,
    double timestamp_sec) const;
};

}  // namespace humanoid_app_gateway_runtime
