/*
 * dynamic_waypoints_manager.cpp
 *
 * 文件用途：
 * 1. 实现当前 Test 工作区单地图点位管理器的 C++ 版本。
 * 2. 订阅 APP 点位命令，处理 set/update/delete/get/clear。
 * 3. 订阅 APP 导航命令，按 Python 版格式转发到 /navigation/requests。
 * 4. 使用 data/dynamic_waypoints.json 单文件持久化，不引入多地图 registry。
 * 5. 发布 /navigation/waypoints_data，消息外层保持 protocol_version/message_type/data_type 等统一格式。
 *
 * 上游节点：
 * - humanoid_app_gateway_runtime/app_gateway_node 发布 /app/waypoint_command 与 /app/navigation_command。
 *
 * 下游节点：
 * - navigation_state_manager 订阅 /navigation/requests 与 /navigation/waypoints_data。
 * - websocket/data_integration 侧可读取 /navigation/waypoints_data 中的点位响应和点位全集。
 */

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"

namespace
{
using JsonValue = rapidjson::Value;

constexpr const char * kWaypointTypes[] = {
  "navigation_target",
  "exhibition_point",
  "obstacle_point",
  "charging_point",
  "rest_point",
  "landmark_point",
};

std::string default_data_path()
{
  const char * home = std::getenv("HOME");
  const std::string home_dir = home && *home ? home : ".";
  return home_dir + "/humanoid_ws/data/dynamic_waypoints.json";
}

std::string expand_user_path(const std::string & path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char * home = std::getenv("HOME");
  const std::string home_dir = home && *home ? home : ".";
  if (path.size() == 1) {
    return home_dir;
  }
  if (path[1] == '/') {
    return home_dir + path.substr(1);
  }
  return path;
}

double now_seconds()
{
  using Clock = std::chrono::system_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

std::string json_to_string(const rapidjson::Document & document, const bool pretty = false)
{
  rapidjson::StringBuffer buffer;
  if (pretty) {
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);
  } else {
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);
  }
  return buffer.GetString();
}

std::string json_to_string(const rapidjson::Value & value)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  value.Accept(writer);
  return buffer.GetString();
}

std::string read_string(const rapidjson::Value & object, const char * key, const std::string & fallback = "")
{
  if (object.IsObject() && object.HasMember(key) && object[key].IsString()) {
    return object[key].GetString();
  }
  return fallback;
}

bool has_waypoint_type(const std::string & type)
{
  return std::find(std::begin(kWaypointTypes), std::end(kWaypointTypes), type) != std::end(kWaypointTypes);
}

void copy_json_value(
  rapidjson::Value & dst,
  const rapidjson::Value & src,
  rapidjson::Document::AllocatorType & allocator)
{
  dst.CopyFrom(src, allocator);
}

}  // namespace

struct WaypointData
{
  std::string id;
  std::string name;
  std::string type;
  std::vector<double> position{0.0, 0.0, 0.0};
  std::vector<double> orientation{0.0, 0.0, 0.0, 1.0};
  std::string frame_id{"map"};
  std::string properties_json{"{}"};
  double created_time{0.0};
  double last_modified{0.0};
};

struct NavigationSequence
{
  std::string id;
  std::string name;
  std::vector<std::string> waypoint_ids;
  std::string sequence_type{"custom"};
  std::string properties_json{"{}"};
};

class DynamicWaypointsManagerCpp : public rclcpp::Node
{
public:
  DynamicWaypointsManagerCpp()
  : Node("dynamic_waypoints_manager_cpp")
  {
    declare_parameters();
    load_parameters();
    initialize_storage();
    setup_interfaces();

    initial_publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(initial_publish_interval_sec_),
      [this]() { publish_initial_waypoints_data(); });

    RCLCPP_INFO(get_logger(), "动态路点管理器 C++ 版启动成功，保持当前单地图协议");
  }

private:
  using WaypointBucket = std::unordered_map<std::string, WaypointData>;

  void declare_parameters()
  {
    declare_parameter<bool>("data_storage.enabled", true);
    declare_parameter<std::string>(
      "data_storage.file_path", default_data_path());
    declare_parameter<std::string>("app_waypoint_command_topic", "/app/waypoint_command");
    declare_parameter<std::string>("app_navigation_command_topic", "/app/navigation_command");
    declare_parameter<std::string>("navigation_requests_topic", "/navigation/requests");
    declare_parameter<std::string>("navigation_waypoints_data_topic", "/navigation/waypoints_data");
    declare_parameter<std::string>("navigation_acknowledgments_topic", "/navigation/acknowledgments");
    declare_parameter<int>("initial_waypoints_publish_max", 5);
    declare_parameter<double>("initial_waypoints_publish_interval_sec", 1.0);
  }

  void load_parameters()
  {
    data_storage_enabled_ = get_parameter("data_storage.enabled").as_bool();
    storage_file_path_ = expand_user_path(get_parameter("data_storage.file_path").as_string());
    app_waypoint_command_topic_ = get_parameter("app_waypoint_command_topic").as_string();
    app_navigation_command_topic_ = get_parameter("app_navigation_command_topic").as_string();
    navigation_requests_topic_ = get_parameter("navigation_requests_topic").as_string();
    navigation_waypoints_data_topic_ = get_parameter("navigation_waypoints_data_topic").as_string();
    navigation_acknowledgments_topic_ = get_parameter("navigation_acknowledgments_topic").as_string();
    initial_publish_max_ = get_parameter("initial_waypoints_publish_max").as_int();
    initial_publish_interval_sec_ = get_parameter("initial_waypoints_publish_interval_sec").as_double();
  }

  void initialize_storage()
  {
    for (const auto * type : kWaypointTypes) {
      waypoints_[type] = WaypointBucket{};
    }

    if (data_storage_enabled_) {
      std::filesystem::create_directories(std::filesystem::path(storage_file_path_).parent_path());
      RCLCPP_INFO(get_logger(), "数据持久化已启用，文件路径: %s", storage_file_path_.c_str());
      load_waypoints_data();
    }
  }

  void setup_interfaces()
  {
    waypoint_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      app_waypoint_command_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { app_waypoint_callback(msg->data); });
    navigation_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      app_navigation_command_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { app_navigation_callback(msg->data); });
    navigation_ack_sub_ = create_subscription<std_msgs::msg::String>(
      navigation_acknowledgments_topic_, 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { navigation_ack_callback(msg->data); });

    navigation_request_pub_ = create_publisher<std_msgs::msg::String>(navigation_requests_topic_, 10);
    waypoints_data_pub_ = create_publisher<std_msgs::msg::String>(navigation_waypoints_data_topic_, 10);
  }

  void app_waypoint_callback(const std::string & payload)
  {
    rapidjson::Document command;
    if (command.Parse(payload.c_str()).HasParseError() || !command.IsObject()) {
      send_app_response("error", "处理命令失败: JSON 格式错误");
      return;
    }

    const std::string command_type = read_string(command, "command_type");
    RCLCPP_INFO(get_logger(), "收到APP点位命令: %s", command_type.c_str());

    if (command_type == "set_waypoint") {
      handle_set_waypoint(command);
    } else if (command_type == "update_waypoint") {
      handle_update_waypoint(command);
    } else if (command_type == "delete_waypoint") {
      handle_delete_waypoint(command);
    } else if (command_type == "get_waypoints") {
      handle_get_waypoints(command);
    } else if (command_type == "clear_waypoints") {
      handle_clear_waypoints(command);
    } else {
      send_app_response("error", "未知命令: " + command_type);
    }
  }

  void app_navigation_callback(const std::string & payload)
  {
    rapidjson::Document command;
    if (command.Parse(payload.c_str()).HasParseError() || !command.IsObject()) {
      send_app_response("error", "处理导航命令失败: JSON 格式错误");
      return;
    }

    RCLCPP_INFO(get_logger(), "收到APP导航命令: %s - 转发给状态管理器", read_string(command, "command_type").c_str());
    send_navigation_request(command);
  }

  void navigation_ack_callback(const std::string & payload)
  {
    rapidjson::Document ack;
    if (!ack.Parse(payload.c_str()).HasParseError() && ack.IsObject()) {
      RCLCPP_INFO(
        get_logger(), "收到状态管理器确认: %s - %s",
        read_string(ack, "ack_type").c_str(), read_string(ack, "status").c_str());
    }
  }

  void handle_set_waypoint(const rapidjson::Document & command)
  {
    if (!command.HasMember("waypoint_data") || !command["waypoint_data"].IsObject()) {
      send_app_response("error", "缺少必要参数: waypoint_data");
      return;
    }
    const auto & waypoint_data = command["waypoint_data"];
    const auto parsed = parse_waypoint(waypoint_data, std::nullopt);
    if (!parsed) {
      return;
    }

    waypoints_[parsed->type][parsed->id] = *parsed;
    save_and_publish();
    send_app_response("success", "点位 '" + parsed->name + "' 设置成功");
  }

  void handle_update_waypoint(const rapidjson::Document & command)
  {
    if (!command.HasMember("waypoint_data") || !command["waypoint_data"].IsObject()) {
      send_app_response("error", "缺少必要参数: waypoint_data");
      return;
    }
    const auto & waypoint_data = command["waypoint_data"];
    const std::string id = read_string(waypoint_data, "id");
    const std::string type = read_string(waypoint_data, "type");
    if (id.empty() || type.empty()) {
      send_app_response("error", "缺少必要参数: id 或 type");
      return;
    }
    if (!has_waypoint_type(type) || waypoints_[type].find(id) == waypoints_[type].end()) {
      send_app_response("error", "点位不存在: " + id);
      return;
    }

    const auto parsed = parse_waypoint(waypoint_data, waypoints_[type][id]);
    if (!parsed) {
      return;
    }
    waypoints_[type][id] = *parsed;
    save_and_publish();
    send_app_response("success", "点位 '" + parsed->name + "' 更新成功");
  }

  void handle_delete_waypoint(const rapidjson::Document & command)
  {
    const std::string id = read_string(command, "waypoint_id");
    const std::string type = read_string(command, "waypoint_type");
    if (id.empty() || type.empty()) {
      send_app_response("error", "缺少必要参数: waypoint_id 或 waypoint_type");
      return;
    }
    if (!has_waypoint_type(type) || waypoints_[type].find(id) == waypoints_[type].end()) {
      send_app_response("error", "点位不存在: " + id);
      return;
    }

    const std::string name = waypoints_[type][id].name;
    waypoints_[type].erase(id);
    for (auto & item : sequences_) {
      auto & ids = item.second.waypoint_ids;
      ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
    }
    save_and_publish();
    send_app_response("success", "点位 '" + name + "' 删除成功");
  }

  void handle_get_waypoints(const rapidjson::Document & command)
  {
    const std::string type = read_string(command, "waypoint_type");
    const bool include_details =
      !command.HasMember("include_details") || !command["include_details"].IsBool() ||
      command["include_details"].GetBool();

    rapidjson::Document result;
    result.SetObject();
    auto & allocator = result.GetAllocator();

    if (!type.empty() && type != "all") {
      if (!has_waypoint_type(type)) {
        send_app_response("error", "无效的点位类型: " + type);
        return;
      }
      append_waypoint_type(result, type, include_details, allocator);
    } else {
      for (const auto * item_type : kWaypointTypes) {
        append_waypoint_type(result, item_type, include_details, allocator);
      }
    }

    if (include_details) {
      JsonValue seqs(rapidjson::kObjectType);
      for (const auto & item : sequences_) {
        JsonValue seq(rapidjson::kObjectType);
        append_sequence(seq, item.second, allocator);
        seqs.AddMember(JsonValue(item.first.c_str(), allocator).Move(), seq, allocator);
      }
      result.AddMember("sequences", seqs, allocator);
    }

    send_app_response("success", "获取点位列表成功", result);
  }

  void handle_clear_waypoints(const rapidjson::Document & command)
  {
    const std::string type = read_string(command, "waypoint_type");
    if (!type.empty()) {
      if (!has_waypoint_type(type)) {
        send_app_response("error", "无效的点位类型: " + type);
        return;
      }
      const auto cleared_count = waypoints_[type].size();
      waypoints_[type].clear();
      send_app_response("success", "清空 " + type + " 类型点位成功，共 " + std::to_string(cleared_count) + " 个");
    } else {
      const int total = total_waypoint_count();
      for (auto & item : waypoints_) {
        item.second.clear();
      }
      sequences_.clear();
      send_app_response("success", "清空所有点位成功，共 " + std::to_string(total) + " 个");
    }
    save_and_publish();
  }

  std::optional<WaypointData> parse_waypoint(
    const rapidjson::Value & data,
    const std::optional<WaypointData> & existing)
  {
    WaypointData waypoint = existing.value_or(WaypointData{});
    waypoint.id = read_string(data, "id", waypoint.id);
    waypoint.type = read_string(data, "type", waypoint.type);
    if (waypoint.id.empty() || waypoint.type.empty()) {
      send_app_response("error", "缺少必要参数: id 或 type");
      return std::nullopt;
    }
    if (!has_waypoint_type(waypoint.type)) {
      send_app_response("error", "无效的点位类型: " + waypoint.type);
      return std::nullopt;
    }

    waypoint.name = read_string(data, "name", waypoint.id);
    waypoint.frame_id = read_string(data, "frame_id", waypoint.frame_id);
    waypoint.position = read_number_array(data, "position", waypoint.position, 3);
    waypoint.orientation = read_number_array(data, "orientation", waypoint.orientation, 4);
    if (data.HasMember("properties") && data["properties"].IsObject()) {
      auto normalized_properties = normalize_waypoint_speed_properties(data["properties"]);
      if (!normalized_properties) {
        return std::nullopt;
      }
      waypoint.properties_json = json_to_string(*normalized_properties);
    }
    const double now = now_seconds();
    if (waypoint.created_time <= 0.0) {
      waypoint.created_time = now;
    }
    waypoint.last_modified = now;
    return waypoint;
  }

  std::optional<rapidjson::Document> normalize_waypoint_speed_properties(const rapidjson::Value & properties)
  {
    rapidjson::Document normalized;
    normalized.SetObject();
    normalized.CopyFrom(properties, normalized.GetAllocator());

    const char * speed_keys[] = {"speed", "target_speed", "navigation_speed"};
    const char * speed_key = nullptr;
    for (const char * key : speed_keys) {
      if (normalized.HasMember(key)) {
        speed_key = key;
        break;
      }
    }

    if (speed_key == nullptr) {
      return normalized;
    }

    const auto & raw_speed = normalized[speed_key];
    if (raw_speed.IsBool() || !raw_speed.IsNumber()) {
      send_app_response("error", "路点速度 speed 必须是数字，单位 m/s");
      return std::nullopt;
    }

    const double speed = raw_speed.GetDouble();
    constexpr double min_speed = 0.15;
    constexpr double max_speed = 1.0;
    if (speed < min_speed || speed > max_speed) {
      std::ostringstream oss;
      oss << "路点速度 speed 必须在 " << std::fixed << std::setprecision(2)
          << min_speed << "~" << max_speed << " m/s 范围内";
      send_app_response("error", oss.str());
      return std::nullopt;
    }

    auto & allocator = normalized.GetAllocator();
    if (normalized.HasMember("speed")) {
      normalized["speed"].SetDouble(speed);
    } else {
      normalized.AddMember("speed", speed, allocator);
    }
    return normalized;
  }

  std::vector<double> read_number_array(
    const rapidjson::Value & data,
    const char * key,
    const std::vector<double> & fallback,
    const std::size_t expected)
  {
    if (!data.HasMember(key) || !data[key].IsArray()) {
      return fallback;
    }
    std::vector<double> values;
    for (const auto & item : data[key].GetArray()) {
      if (item.IsNumber()) {
        values.push_back(item.GetDouble());
      }
    }
    return values.size() == expected ? values : fallback;
  }

  void send_navigation_request(const rapidjson::Document & command)
  {
    rapidjson::Document request;
    request.SetObject();
    auto & allocator = request.GetAllocator();
    request.AddMember("request_type", "navigation_command", allocator);
    JsonValue command_copy(rapidjson::kObjectType);
    copy_json_value(command_copy, command, allocator);
    request.AddMember("command_data", command_copy, allocator);
    request.AddMember("timestamp", now_seconds(), allocator);
    request.AddMember("source", "waypoints_manager", allocator);

    const std::string command_type = read_string(command, "command_type");
    if (command_type == "start_single_navigation") {
      const std::string waypoint_id = read_string(command, "waypoint_id");
      if (const auto waypoint = find_waypoint_by_id(waypoint_id)) {
        JsonValue wp(rapidjson::kObjectType);
        append_waypoint(wp, *waypoint, allocator);
        request.AddMember("waypoint_data", wp, allocator);
      }
    } else if (command_type == "start_multi_point_navigation") {
      if (!command.HasMember("waypoint_ids") || !command["waypoint_ids"].IsArray()) {
        send_app_response("error", "点位列表不能为空");
        return;
      }
      JsonValue multi(rapidjson::kObjectType);
      for (const auto & id_value : command["waypoint_ids"].GetArray()) {
        if (!id_value.IsString()) {
          continue;
        }
        const std::string waypoint_id = id_value.GetString();
        const auto waypoint = find_waypoint_by_id(waypoint_id);
        if (!waypoint) {
          send_app_response("error", "点位不存在: " + waypoint_id);
          return;
        }
        JsonValue wp(rapidjson::kObjectType);
        append_waypoint(wp, *waypoint, allocator);
        multi.AddMember(JsonValue(waypoint_id.c_str(), allocator).Move(), wp, allocator);
      }
      request.AddMember("waypoints_data", multi, allocator);
    } else if (command_type == "start_exhibition_navigation") {
      JsonValue exhibition(rapidjson::kObjectType);
      for (const auto & item : waypoints_["exhibition_point"]) {
        JsonValue wp(rapidjson::kObjectType);
        append_waypoint(wp, item.second, allocator);
        exhibition.AddMember(JsonValue(item.first.c_str(), allocator).Move(), wp, allocator);
      }
      request.AddMember("waypoints_data", exhibition, allocator);
    }

    std_msgs::msg::String msg;
    msg.data = json_to_string(request);
    navigation_request_pub_->publish(msg);
  }

  void publish_waypoints_data(const std::string & update_type = "full_update")
  {
    rapidjson::Document data;
    data.SetObject();
    auto & allocator = data.GetAllocator();
    data.AddMember("update_type", JsonValue(update_type.c_str(), allocator).Move(), allocator);
    data.AddMember("timestamp", now_seconds(), allocator);

    JsonValue payload(rapidjson::kObjectType);
    JsonValue waypoints_object(rapidjson::kObjectType);
    for (const auto * type : kWaypointTypes) {
      JsonValue bucket(rapidjson::kObjectType);
      for (const auto & item : waypoints_[type]) {
        JsonValue wp(rapidjson::kObjectType);
        append_waypoint(wp, item.second, allocator);
        bucket.AddMember(JsonValue(item.first.c_str(), allocator).Move(), wp, allocator);
      }
      waypoints_object.AddMember(JsonValue(type, allocator).Move(), bucket, allocator);
    }
    payload.AddMember("waypoints", waypoints_object, allocator);

    JsonValue sequences_object(rapidjson::kObjectType);
    for (const auto & item : sequences_) {
      JsonValue seq(rapidjson::kObjectType);
      append_sequence(seq, item.second, allocator);
      sequences_object.AddMember(JsonValue(item.first.c_str(), allocator).Move(), seq, allocator);
    }
    payload.AddMember("sequences", sequences_object, allocator);
    data.AddMember("data", payload, allocator);

    JsonValue metadata(rapidjson::kObjectType);
    metadata.AddMember("total_count", total_waypoint_count(), allocator);
    metadata.AddMember("sequence_count", static_cast<int>(sequences_.size()), allocator);
    data.AddMember("metadata", metadata, allocator);

    publish_unified_message("push", "waypoints_data", "waypoints_manager", "all", data);
  }

  void publish_initial_waypoints_data()
  {
    publish_waypoints_data("initial_load");
    initial_publish_count_++;
    if (initial_publish_count_ >= initial_publish_max_ && initial_publish_timer_) {
      initial_publish_timer_->cancel();
      initial_publish_timer_.reset();
    }
  }

  void send_app_response(
    const std::string & response_type,
    const std::string & message,
    const rapidjson::Value * result = nullptr)
  {
    rapidjson::Document data;
    data.SetObject();
    auto & allocator = data.GetAllocator();
    data.AddMember("response_type", JsonValue(response_type.c_str(), allocator).Move(), allocator);
    data.AddMember("message", JsonValue(message.c_str(), allocator).Move(), allocator);
    if (result) {
      JsonValue copied;
      copied.CopyFrom(*result, allocator);
      data.AddMember("result", copied, allocator);
    } else {
      data.AddMember("result", JsonValue(rapidjson::kObjectType), allocator);
    }
    publish_unified_message("response", "waypoint_response", "waypoints_manager", "all", data, response_type);
  }

  void send_app_response(
    const std::string & response_type,
    const std::string & message,
    const rapidjson::Document & result)
  {
    send_app_response(response_type, message, static_cast<const rapidjson::Value *>(&result));
  }

  void publish_unified_message(
    const std::string & message_type,
    const std::string & data_type,
    const std::string & source,
    const std::string & destination,
    const rapidjson::Value & data,
    const std::string & status = "success")
  {
    rapidjson::Document message;
    message.SetObject();
    auto & allocator = message.GetAllocator();
    message.AddMember("protocol_version", "2.0", allocator);
    const std::string message_id = message_type + "_" + std::to_string(static_cast<int64_t>(now_seconds()));
    message.AddMember("message_id", JsonValue(message_id.c_str(), allocator).Move(), allocator);
    message.AddMember("timestamp", now_seconds(), allocator);
    message.AddMember("message_type", JsonValue(message_type.c_str(), allocator).Move(), allocator);
    message.AddMember("data_type", JsonValue(data_type.c_str(), allocator).Move(), allocator);
    message.AddMember("source", JsonValue(source.c_str(), allocator).Move(), allocator);
    message.AddMember("destination", JsonValue(destination.c_str(), allocator).Move(), allocator);
    JsonValue copied;
    copied.CopyFrom(data, allocator);
    message.AddMember("data", copied, allocator);

    JsonValue metadata(rapidjson::kObjectType);
    metadata.AddMember("status", JsonValue(status == "error" ? "error" : "success", allocator).Move(), allocator);
    metadata.AddMember("error_code", "", allocator);
    metadata.AddMember("error_message", "", allocator);
    metadata.AddMember("request_id", "", allocator);
    if (status == "error" && data.HasMember("message") && data["message"].IsString()) {
      metadata["error_message"].SetString(data["message"].GetString(), allocator);
    }
    message.AddMember("metadata", metadata, allocator);

    std_msgs::msg::String msg;
    msg.data = json_to_string(message);
    waypoints_data_pub_->publish(msg);
  }

  void save_and_publish()
  {
    if (data_storage_enabled_) {
      save_waypoints_data();
    }
    publish_waypoints_data();
  }

  void save_waypoints_data()
  {
    rapidjson::Document data;
    data.SetObject();
    auto & allocator = data.GetAllocator();

    JsonValue waypoints_object(rapidjson::kObjectType);
    for (const auto * type : kWaypointTypes) {
      JsonValue bucket(rapidjson::kObjectType);
      for (const auto & item : waypoints_[type]) {
        JsonValue wp(rapidjson::kObjectType);
        append_waypoint(wp, item.second, allocator);
        bucket.AddMember(JsonValue(item.first.c_str(), allocator).Move(), wp, allocator);
      }
      waypoints_object.AddMember(JsonValue(type, allocator).Move(), bucket, allocator);
    }
    data.AddMember("waypoints", waypoints_object, allocator);

    JsonValue sequences_object(rapidjson::kObjectType);
    for (const auto & item : sequences_) {
      JsonValue seq(rapidjson::kObjectType);
      append_sequence(seq, item.second, allocator);
      sequences_object.AddMember(JsonValue(item.first.c_str(), allocator).Move(), seq, allocator);
    }
    data.AddMember("sequences", sequences_object, allocator);
    data.AddMember("timestamp", now_seconds(), allocator);

    std::ofstream ofs(storage_file_path_);
    rapidjson::OStreamWrapper osw(ofs);
    rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
    data.Accept(writer);
  }

  void load_waypoints_data()
  {
    if (!std::filesystem::exists(storage_file_path_)) {
      return;
    }
    std::ifstream ifs(storage_file_path_);
    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document data;
    if (data.ParseStream(isw).HasParseError() || !data.IsObject()) {
      RCLCPP_ERROR(get_logger(), "加载点位数据错误: JSON 格式错误");
      return;
    }

    if (data.HasMember("waypoints") && data["waypoints"].IsObject()) {
      for (const auto * type : kWaypointTypes) {
        if (!data["waypoints"].HasMember(type) || !data["waypoints"][type].IsObject()) {
          continue;
        }
        for (const auto & member : data["waypoints"][type].GetObject()) {
          const auto parsed = parse_waypoint(member.value, std::nullopt);
          if (parsed) {
            waypoints_[type][parsed->id] = *parsed;
          }
        }
      }
    }

    if (data.HasMember("sequences") && data["sequences"].IsObject()) {
      for (const auto & member : data["sequences"].GetObject()) {
        if (!member.value.IsObject()) {
          continue;
        }
        NavigationSequence seq;
        seq.id = read_string(member.value, "id", member.name.GetString());
        seq.name = read_string(member.value, "name", seq.id);
        seq.sequence_type = read_string(member.value, "sequence_type", "custom");
        if (member.value.HasMember("waypoint_ids") && member.value["waypoint_ids"].IsArray()) {
          for (const auto & id : member.value["waypoint_ids"].GetArray()) {
            if (id.IsString()) {
              seq.waypoint_ids.emplace_back(id.GetString());
            }
          }
        }
        if (member.value.HasMember("properties") && member.value["properties"].IsObject()) {
          seq.properties_json = json_to_string(member.value["properties"]);
        }
        sequences_[seq.id] = seq;
      }
    }

    RCLCPP_INFO(
      get_logger(), "从文件加载点位数据完成，共 %d 个点位，%zu 个序列",
      total_waypoint_count(), sequences_.size());
  }

  void append_waypoint_type(
    JsonValue & object,
    const std::string & type,
    const bool include_details,
    rapidjson::Document::AllocatorType & allocator)
  {
    if (include_details) {
      JsonValue bucket(rapidjson::kObjectType);
      for (const auto & item : waypoints_[type]) {
        JsonValue wp(rapidjson::kObjectType);
        append_waypoint(wp, item.second, allocator);
        bucket.AddMember(JsonValue(item.first.c_str(), allocator).Move(), wp, allocator);
      }
      object.AddMember(JsonValue(type.c_str(), allocator).Move(), bucket, allocator);
    } else {
      JsonValue ids(rapidjson::kArrayType);
      for (const auto & item : waypoints_[type]) {
        ids.PushBack(JsonValue(item.first.c_str(), allocator).Move(), allocator);
      }
      object.AddMember(JsonValue(type.c_str(), allocator).Move(), ids, allocator);
    }
  }

  void append_waypoint(
    JsonValue & object,
    const WaypointData & waypoint,
    rapidjson::Document::AllocatorType & allocator)
  {
    object.AddMember("id", JsonValue(waypoint.id.c_str(), allocator).Move(), allocator);
    object.AddMember("name", JsonValue(waypoint.name.c_str(), allocator).Move(), allocator);
    object.AddMember("type", JsonValue(waypoint.type.c_str(), allocator).Move(), allocator);
    JsonValue position(rapidjson::kArrayType);
    for (double value : waypoint.position) {
      position.PushBack(value, allocator);
    }
    object.AddMember("position", position, allocator);
    JsonValue orientation(rapidjson::kArrayType);
    for (double value : waypoint.orientation) {
      orientation.PushBack(value, allocator);
    }
    object.AddMember("orientation", orientation, allocator);
    object.AddMember("frame_id", JsonValue(waypoint.frame_id.c_str(), allocator).Move(), allocator);
    rapidjson::Document props;
    if (props.Parse(waypoint.properties_json.c_str()).HasParseError() || !props.IsObject()) {
      props.SetObject();
    }
    JsonValue properties;
    properties.CopyFrom(props, allocator);
    object.AddMember("properties", properties, allocator);
    object.AddMember("created_time", waypoint.created_time, allocator);
    object.AddMember("last_modified", waypoint.last_modified, allocator);
  }

  void append_sequence(
    JsonValue & object,
    const NavigationSequence & seq,
    rapidjson::Document::AllocatorType & allocator)
  {
    object.AddMember("id", JsonValue(seq.id.c_str(), allocator).Move(), allocator);
    object.AddMember("name", JsonValue(seq.name.c_str(), allocator).Move(), allocator);
    JsonValue ids(rapidjson::kArrayType);
    for (const auto & id : seq.waypoint_ids) {
      ids.PushBack(JsonValue(id.c_str(), allocator).Move(), allocator);
    }
    object.AddMember("waypoint_ids", ids, allocator);
    object.AddMember("sequence_type", JsonValue(seq.sequence_type.c_str(), allocator).Move(), allocator);
    rapidjson::Document props;
    if (props.Parse(seq.properties_json.c_str()).HasParseError() || !props.IsObject()) {
      props.SetObject();
    }
    JsonValue properties;
    properties.CopyFrom(props, allocator);
    object.AddMember("properties", properties, allocator);
  }

  const WaypointData * find_waypoint_by_id(const std::string & id) const
  {
    for (const auto & bucket : waypoints_) {
      const auto it = bucket.second.find(id);
      if (it != bucket.second.end()) {
        return &it->second;
      }
    }
    return nullptr;
  }

  int total_waypoint_count() const
  {
    int total = 0;
    for (const auto & item : waypoints_) {
      total += static_cast<int>(item.second.size());
    }
    return total;
  }

  bool data_storage_enabled_{true};
  std::string storage_file_path_;
  std::string app_waypoint_command_topic_;
  std::string app_navigation_command_topic_;
  std::string navigation_requests_topic_;
  std::string navigation_waypoints_data_topic_;
  std::string navigation_acknowledgments_topic_;
  int initial_publish_max_{5};
  double initial_publish_interval_sec_{1.0};
  int initial_publish_count_{0};

  std::map<std::string, WaypointBucket> waypoints_;
  std::unordered_map<std::string, NavigationSequence> sequences_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_ack_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoints_data_pub_;
  rclcpp::TimerBase::SharedPtr initial_publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicWaypointsManagerCpp>());
  rclcpp::shutdown();
  return 0;
}
