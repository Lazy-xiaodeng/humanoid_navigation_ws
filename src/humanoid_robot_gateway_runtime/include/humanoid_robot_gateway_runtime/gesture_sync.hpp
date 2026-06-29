/*
 * gesture_sync.hpp
 *
 * 文件用途：
 * 1. 负责从机器人本体获取动作库、更新 gestures.yaml、通知数据整合热重载。
 * 2. 对齐现有动作库拉取、gestures.yaml 内容生成和热重载通知语义。
 * 3. 本模块会写配置文件，接入前需要独立验证文件路径和权限。
 * 4. 上游：机器人本体动作库接口。
 * 5. 下游：humanoid_locomotion gestures.yaml、/system/gesture_list_updated 和 data_integration_node。
 */

#pragma once

#include <string>
#include <vector>

namespace humanoid_robot_gateway_runtime
{

struct GestureMotionItem
{
  std::string en_name;
  std::string cn_name;
  int index{0};
};

struct GestureSyncResult
{
  bool ok{false};
  std::string error;
  std::string text;
};

class GestureSync
{
public:
  std::string name() const;

  // 从机器人 response.data 中提取动作条目，兼容 线上协议支持的多个字段名。
  std::vector<GestureMotionItem> extract_motion_items(const std::string & data_json) const;

  // 生成 gestures.yaml 内容；保留已有动作元数据，只刷新机器人 OTA 返回的 id/name。
  GestureSyncResult build_gestures_yaml(
    const std::string & existing_yaml,
    const std::vector<GestureMotionItem> & motion_items) const;

  // 生成 /system/gesture_list_updated 热重载通知消息。
  GestureSyncResult build_reload_message(double timestamp_sec) const;
};

}  // namespace humanoid_robot_gateway_runtime
