/*
 * map_waypoint_adapter.hpp
 *
 * 文件用途：
 * 1. 负责地图状态、地图响应和点位库 waypoints_data 的缓存与 APP 推送转换。
 * 2. 对齐现有地图状态、地图响应和点位全量数据转换语义。
 * 3. 本模块应保持 map_id、revision、update_type 等字段兼容。
 * 4. 上游：map_context_manager、dynamic_waypoints_manager 发布的地图和点位话题。
 * 5. 下游：data_store、APP 初始快照、地图页和导航页路线点显示。
 */

#pragma once

#include <string>

namespace humanoid_app_gateway_runtime
{

struct MapWaypointBuildResult
{
  // 转换是否成功；输入 JSON 非法时为 false。
  bool ok{false};

  // 失败原因；成功时为空。
  std::string error;

  // 转换后的紧凑 JSON，包含 message_type、data_type、data、metadata。
  std::string json;
};

class MapWaypointAdapter
{
public:
  std::string name() const;

  // 保持线上协议 map_message_callback() 中 data_type/data/metadata 的整理逻辑。
  MapWaypointBuildResult build_map_message_payload(const std::string & payload_json) const;

  // 保持线上协议 waypoints_data_callback() 中 data_type/data/metadata 的整理逻辑。
  MapWaypointBuildResult build_waypoints_data_payload(const std::string & payload_json) const;
};

}  // namespace humanoid_app_gateway_runtime
