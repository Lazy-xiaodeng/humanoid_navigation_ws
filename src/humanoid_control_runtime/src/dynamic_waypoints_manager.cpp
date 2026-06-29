/*
 * dynamic_waypoints_manager.cpp
 *
 * 文件用途：
 * 1. 这个节点属于“常驻控制层”，负责连接 APP 与路线运行层，不直接执行导航、不发布导航状态。
 * 2. 节点职责包括：APP 点位命令处理、APP 导航命令轻校验与透传、多地图点位 JSON 持久化、
 *    点位库 revision 维护、启动后点位库同步。
 * 3. 点位库是路线任务的基础数据源，任何 set/update/delete/clear 都必须更新 revision，
 *    并通过 /navigation/waypoints_data 通知路线运行层重新读取。
 * 4. APP 导航命令只在这里做轻量格式检查和 ID 字符串归一化，真正的路线执行、暂停、
 *    跳点和播报状态由下游路线运行层负责。
 * 5. 文件路径指向现场运行数据，调试时要确认 data_storage.waypoints_dir 位于当前工作区。
 *
 * 本文件按真实数据链路排列代码块：
 * 1. 数据模型和配置结构。
 * 2. 节点初始化：参数、缓存、持久化、ROS 接口。
 * 3. ROS 输入入口：APP 点位命令、APP 导航命令、状态机 ack。
 * 4. APP 点位命令处理：set/update/delete/get/clear。
 * 5. APP 导航命令桥接：校验、ID 归一化、透传到 /navigation/requests。
 * 6. 输出链路：点位库同步和 APP response。
 * 7. 点位缓存工具：map_id、revision、统计和查找。
 * 8. 文件持久化：加载、兼容导入、保存。
 * 9. main 入口。
 */

#include <chrono>
#include <cctype>
#include <cstdio>
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

#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace humanoid_control_runtime
{

namespace fs = std::filesystem;

// =============================================================================
// 1. 数据模型和配置结构
// =============================================================================

enum class WaypointType
{
  Manual,
  NavigationTarget,
  ExhibitionPoint,
  ObstaclePoint,
  ChargingPoint,
  RestPoint,
  LandmarkPoint,
};

std::string waypoint_type_to_string(const WaypointType type)
{
  switch (type) {
    case WaypointType::Manual:
      return "manual";
    case WaypointType::NavigationTarget:
      return "navigation_target";
    case WaypointType::ExhibitionPoint:
      return "exhibition_point";
    case WaypointType::ObstaclePoint:
      return "obstacle_point";
    case WaypointType::ChargingPoint:
      return "charging_point";
    case WaypointType::RestPoint:
      return "rest_point";
    case WaypointType::LandmarkPoint:
      return "landmark_point";
  }
  return "navigation_target";
}

WaypointType waypoint_type_from_string(const std::string & value)
{
  if (value == "manual") {
    return WaypointType::Manual;
  }
  if (value == "exhibition_point") {
    return WaypointType::ExhibitionPoint;
  }
  if (value == "obstacle_point") {
    return WaypointType::ObstaclePoint;
  }
  if (value == "charging_point") {
    return WaypointType::ChargingPoint;
  }
  if (value == "rest_point") {
    return WaypointType::RestPoint;
  }
  if (value == "landmark_point") {
    return WaypointType::LandmarkPoint;
  }
  return WaypointType::NavigationTarget;
}

bool is_valid_waypoint_type(const std::string & value)
{
  return value == "manual" ||
         value == "navigation_target" ||
         value == "exhibition_point" ||
         value == "obstacle_point" ||
         value == "charging_point" ||
         value == "rest_point" ||
         value == "landmark_point";
}

struct WaypointData
{
  // 单个点位的运行时数据。字段名也是点位 JSON 和 APP 协议的一部分，不能随意改名。
  // properties_json 保存原始扩展属性，便于新增业务字段时不频繁改 C++ 数据结构。
  std::string id;
  std::string name;
  WaypointType type{WaypointType::NavigationTarget};
  std::vector<double> position{0.0, 0.0, 0.0};
  std::vector<double> orientation{0.0, 0.0, 0.0, 1.0};
  std::string frame_id{"map"};
  std::string map_id{"hall"};
  std::string properties_json{"{}"};
  double created_time{0.0};
  double last_modified{0.0};
};

double now_seconds()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string now_revision()
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << now_seconds();
  return stream.str();
}

std::string trim_copy(const std::string & input)
{
  const auto begin = input.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(begin, end - begin + 1);
}

std::string json_value_to_string(const rapidjson::Value & value)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  value.Accept(writer);
  return buffer.GetString();
}

rapidjson::Value json_string_to_value(
  const std::string & json_text,
  rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Document doc;
  if (json_text.empty() || doc.Parse(json_text.c_str()).HasParseError()) {
    rapidjson::Value empty_object(rapidjson::kObjectType);
    return empty_object;
  }
  rapidjson::Value value;
  value.CopyFrom(doc, allocator);
  return value;
}

std::vector<double> read_double_array(
  const rapidjson::Value & object,
  const char * key,
  const std::vector<double> & fallback)
{
  if (!object.HasMember(key) || !object[key].IsArray()) {
    return fallback;
  }

  std::vector<double> result;
  for (const auto & item : object[key].GetArray()) {
    if (item.IsNumber()) {
      result.push_back(item.GetDouble());
    }
  }
  return result.empty() ? fallback : result;
}

void write_double_array(
  rapidjson::Value & object,
  const char * key,
  const std::vector<double> & values,
  rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value array(rapidjson::kArrayType);
  for (const auto value : values) {
    array.PushBack(value, allocator);
  }
  object.AddMember(rapidjson::Value(key, allocator).Move(), array, allocator);
}

struct DynamicWaypointsConfig
{
  // 数据持久化参数：集中管理点位文件位置、默认地图和自动保存策略。
  // data_storage_enabled=false 时节点仍可收发命令，但点位变更不会写入磁盘，适合纯临时测试。
  bool data_storage_enabled{true};

  // 兼容单文件点位库路径。仅在多地图目录没有可用数据时作为导入来源。
  std::string storage_file_path{
    "/home/ubuntu/software/Todesk/Files/humanoid_ws/data/dynamic_waypoints.json"};

  // 多地图点位目录。每张地图一个 JSON 文件，文件名由 map_id 归一化后生成。
  std::string waypoints_dir{"/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints"};

  // 命令没有携带 map_id 时使用的默认地图，避免点位写入空地图桶。
  std::string default_map_id{"hall"};

  // 周期落盘间隔。即使命令处理时已保存，周期保存仍可作为异常恢复兜底。
  double auto_save_interval_sec{300.0};

  // 话题参数：保持与现有 APP、websocket、路线运行层完全一致。
  // APP 点位命令入口，承载 set/update/delete/get/clear。
  std::string app_waypoint_command_topic{"/app/waypoint_command"};

  // APP 导航控制入口，承载 start/pause/resume/stop/jump/broadcast_finished。
  std::string app_navigation_command_topic{"/app/navigation_command"};

  // 转发给路线运行层的请求话题，路线运行层只需要监听这个统一入口。
  std::string navigation_requests_topic{"/navigation/requests"};

  // 点位库同步和点位命令响应共用话题，APP 桥接层和路线运行层都依赖它。
  std::string navigation_waypoints_data_topic{"/navigation/waypoints_data"};

  // 路线运行层确认消息话题，当前主要用于日志与后续闭环扩展。
  std::string navigation_acknowledgments_topic{"/navigation/acknowledgments"};

  // 启动同步参数：用于启动后重复推送本地点位库。
  // 重复次数越多，越能照顾后启动订阅者；过大则会增加启动阶段日志和网络流量。
  int initial_waypoints_publish_max{5};

  // 重复推送间隔，单位秒。
  double initial_waypoints_publish_interval_sec{1.0};
};

using WaypointBucket = std::unordered_map<std::string, WaypointData>;
using WaypointsByType = std::map<std::string, WaypointBucket>;
using WaypointsByMap = std::map<std::string, WaypointsByType>;

class DynamicWaypointsManagerNode : public rclcpp::Node
{
public:
  explicit DynamicWaypointsManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dynamic_waypoints_manager_cpp", options)
  {
    declare_parameters();
    load_parameters();
    initialize_runtime_cache();
    setup_data_persistence();
    setup_interfaces();

    RCLCPP_WARN(
      get_logger(),
      "dynamic_waypoints_manager_cpp 已启动：负责点位持久化、点位命令处理和导航命令桥接；"
      "当前通过 use_cpp_control_runtime 运行开关启用，已接入点位管理运行链路。");
  }

private:
  // ===========================================================================
  // 2. 节点初始化：参数、缓存、持久化、ROS 接口
  // ===========================================================================

  void declare_parameters()
  {
    // 数据持久化参数：保持 data_storage.* 层级，方便现场通过 YAML 统一管理。
    declare_parameter<bool>("data_storage.enabled", true);
    declare_parameter<std::string>(
      "data_storage.file_path",
      "/home/ubuntu/software/Todesk/Files/humanoid_ws/data/dynamic_waypoints.json");
    declare_parameter<std::string>(
      "data_storage.waypoints_dir",
      "/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints");
    declare_parameter<std::string>("data_storage.default_map_id", "hall");
    declare_parameter<double>("data_storage.auto_save_interval", 300.0);

    // 话题参数：这些话题是 APP、WebSocket 桥和路线运行层之间的稳定接口。
    declare_parameter<std::string>("app_waypoint_command_topic", "/app/waypoint_command");
    declare_parameter<std::string>("app_navigation_command_topic", "/app/navigation_command");
    declare_parameter<std::string>("navigation_requests_topic", "/navigation/requests");
    declare_parameter<std::string>("navigation_waypoints_data_topic", "/navigation/waypoints_data");
    declare_parameter<std::string>("navigation_acknowledgments_topic", "/navigation/acknowledgments");

    // 启动同步参数：用于控制启动后点位库重复推送次数和间隔。
    declare_parameter<int>("initial_waypoints_publish_max", 5);
    declare_parameter<double>("initial_waypoints_publish_interval_sec", 1.0);
  }

  void load_parameters()
  {
    config_.data_storage_enabled = get_parameter("data_storage.enabled").as_bool();
    config_.storage_file_path = get_parameter("data_storage.file_path").as_string();
    config_.waypoints_dir = get_parameter("data_storage.waypoints_dir").as_string();
    config_.default_map_id = normalize_map_id(get_parameter("data_storage.default_map_id").as_string());
    config_.auto_save_interval_sec = get_parameter("data_storage.auto_save_interval").as_double();

    config_.app_waypoint_command_topic = get_parameter("app_waypoint_command_topic").as_string();
    config_.app_navigation_command_topic = get_parameter("app_navigation_command_topic").as_string();
    config_.navigation_requests_topic = get_parameter("navigation_requests_topic").as_string();
    config_.navigation_waypoints_data_topic =
      get_parameter("navigation_waypoints_data_topic").as_string();
    config_.navigation_acknowledgments_topic =
      get_parameter("navigation_acknowledgments_topic").as_string();
    config_.initial_waypoints_publish_max = get_parameter("initial_waypoints_publish_max").as_int();
    config_.initial_waypoints_publish_interval_sec =
      get_parameter("initial_waypoints_publish_interval_sec").as_double();
  }

  void initialize_runtime_cache()
  {
    // 当前地图和默认地图先对齐；后续收到 APP 命令时再按 command/map_id 切换目标缓存。
    current_map_id_ = config_.default_map_id;
    ensure_map_cache(current_map_id_);
  }

  void setup_data_persistence()
  {
    // 启动持久化链路：
    // 1. 展开 ~/ 路径；
    // 2. 创建 data/waypoints 目录；
    // 3. 优先加载多地图点位文件；
    // 4. 如果没有多地图文件，则从兼容单文件点位库导入到默认地图。
    config_.storage_file_path = expand_user_path(config_.storage_file_path);
    config_.waypoints_dir = expand_user_path(config_.waypoints_dir);

    try {
      if (!config_.storage_file_path.empty()) {
        const auto legacy_parent = fs::path(config_.storage_file_path).parent_path();
        if (!legacy_parent.empty()) {
          fs::create_directories(legacy_parent);
        }
      }
      fs::create_directories(config_.waypoints_dir);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "创建点位存储目录失败: %s", error.what());
    }

    ensure_map_cache(config_.default_map_id);

    if (!config_.data_storage_enabled) {
      refresh_waypoints_revision(config_.default_map_id);
      RCLCPP_INFO(get_logger(), "点位持久化已禁用，仅使用内存缓存");
      return;
    }

    const bool loaded_new_format = load_waypoints_data();
    if (!loaded_new_format && fs::exists(config_.storage_file_path)) {
      RCLCPP_WARN(
        get_logger(),
        "未发现多地图点位文件，开始从兼容单文件点位库导入: %s",
        config_.storage_file_path.c_str());
      load_legacy_waypoints_data(config_.default_map_id);
      save_waypoints_data(config_.default_map_id);
    } else if (!loaded_new_format) {
      refresh_waypoints_revision(config_.default_map_id);
      RCLCPP_INFO(get_logger(), "没有找到现有点位文件，将在首次保存时创建");
    }

    current_map_id_ = config_.default_map_id;
    ensure_map_cache(current_map_id_);
  }

  void setup_interfaces()
  {
    // ROS 接口分成三类：APP 点位命令、APP 导航命令、路线运行层 ack。
    // 点位命令会修改本地缓存和文件；导航命令只做协议整理后透传给路线运行层。
    navigation_request_pub_ =
      create_publisher<std_msgs::msg::String>(config_.navigation_requests_topic, rclcpp::QoS(10));
    waypoints_data_pub_ =
      create_publisher<std_msgs::msg::String>(
      config_.navigation_waypoints_data_topic, rclcpp::QoS(10));

    app_waypoint_sub_ = create_subscription<std_msgs::msg::String>(
      config_.app_waypoint_command_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        on_app_waypoint_command(msg);
      });

    app_navigation_sub_ = create_subscription<std_msgs::msg::String>(
      config_.app_navigation_command_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        on_app_navigation_command(msg);
      });

    navigation_ack_sub_ = create_subscription<std_msgs::msg::String>(
      config_.navigation_acknowledgments_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        on_navigation_ack(msg);
      });

    // 只有启动时已经从本地点位文件加载到点位，才重复发布 initial_load。
    // 这样后启动的路线运行层也能拿到完整点位库，但空库不会刷出无意义的同步消息。
    if (get_total_waypoints_count() > 0) {
      const auto interval_ms =
        std::chrono::milliseconds(
        static_cast<int64_t>(config_.initial_waypoints_publish_interval_sec * 1000.0));
      initial_publish_timer_ = create_wall_timer(
        interval_ms <= 0ms ? 1s : interval_ms,
        [this]() {
          publish_initial_waypoints_data();
        });
    }
  }

  // ===========================================================================
  // 3. ROS 输入入口：APP 点位命令、APP 导航命令、状态机 ack
  // ===========================================================================

  void on_app_waypoint_command(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    // APP 点位命令入口：处理新增、更新、删除、查询和清空点位。
    // 数据流：APP -> /app/waypoint_command -> 本节点 -> 点位缓存/文件 -> /navigation/waypoints_data。
    // 这里负责 JSON parse、command_type 分发和异常 response。
    RCLCPP_DEBUG(get_logger(), "收到 APP 点位命令，长度=%zu", msg->data.size());
    dispatch_waypoint_command(msg->data);
  }

  void on_app_navigation_command(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    // APP 导航命令入口：处理路线任务控制命令，并转发给路线运行层。
    // 数据流：APP -> /app/navigation_command -> 本节点轻校验/归一化 -> /navigation/requests。
    // 注意：route task 语义、jump 合法性、播报上下文匹配都应交给路线运行层判断。
    RCLCPP_DEBUG(get_logger(), "收到 APP 导航命令，长度=%zu", msg->data.size());
    bridge_navigation_command(msg->data);
  }

  void on_navigation_ack(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    // 路线运行层 ack 入口：当前主要用于兼容日志和后续状态闭环扩展。
    // 该链路主要兼容旧确认消息；新 route task 业务结果更多走 /navigation/status。
    RCLCPP_DEBUG(get_logger(), "收到路线运行层 ack，长度=%zu", msg->data.size());
  }

  // ===========================================================================
  // 4. APP 点位命令处理：set/update/delete/get/clear
  // ===========================================================================

  void dispatch_waypoint_command(const std::string & raw_json)
  {
    // 入口只负责解析 command_type 和异常兜底，具体业务由各 handle_* 保持单一职责。
    rapidjson::Document command;
    command.Parse(raw_json.c_str());
    if (command.HasParseError() || !command.IsObject()) {
      send_app_response("error", "点位命令 JSON 解析失败");
      return;
    }

    const std::string command_type = read_string_member(command, "command_type", "");
    active_waypoint_request_message_id_ = read_string_member(command, "request_message_id", "");
    RCLCPP_INFO(get_logger(), "收到 APP 点位命令: %s", command_type.c_str());

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

  void handle_set_waypoint(const rapidjson::Value & command)
  {
    // 新增点位：校验必要字段、归一化速度属性、写入地图点位缓存并触发落盘。
    // 成功后会发布 waypoint_response，并同步最新点位库给路线运行层。
    // 目标行为：校验 id/type -> 归一化 speed -> 写入地图点位桶 -> 刷新 revision -> 保存 -> 推送。
    const auto * waypoint_data = find_object_member(command, "waypoint_data");
    if (waypoint_data == nullptr) {
      send_app_response("error", "缺少必要参数: id 或 type");
      return;
    }

    const std::string waypoint_id =
      read_string_member(*waypoint_data, "id", read_string_member(*waypoint_data, "waypoint_id", ""));
    const std::string waypoint_type =
      read_string_member(*waypoint_data, "type", read_string_member(command, "waypoint_type", ""));
    const std::string map_id = extract_command_map_id(command, waypoint_data);
    if (waypoint_id.empty() || waypoint_type.empty()) {
      send_app_response("error", "缺少必要参数: id 或 type", rapidjson::Value(rapidjson::kObjectType), map_id);
      return;
    }
    if (!is_valid_waypoint_type(waypoint_type)) {
      send_app_response(
        "error",
        "设置点位失败: '" + waypoint_type + "' is not a valid WaypointType",
        rapidjson::Value(rapidjson::kObjectType),
        map_id);
      return;
    }

    std::string normalized_properties = "{}";
    std::string error_message;
    if (!normalize_waypoint_speed_properties(*waypoint_data, normalized_properties, error_message)) {
      send_app_response("error", error_message, rapidjson::Value(rapidjson::kObjectType), map_id);
      return;
    }

    WaypointData waypoint;
    waypoint.id = waypoint_id;
    waypoint.name = read_string_member(
      *waypoint_data, "name", read_string_member(*waypoint_data, "waypoint_name", waypoint_id));
    waypoint.type = waypoint_type_from_string(waypoint_type);
    waypoint.position = read_double_array(*waypoint_data, "position", {0.0, 0.0, 0.0});
    waypoint.orientation = read_double_array(*waypoint_data, "orientation", {0.0, 0.0, 0.0, 1.0});
    waypoint.frame_id = read_string_member(*waypoint_data, "frame_id", "map");
    waypoint.map_id = map_id;
    waypoint.properties_json = normalized_properties;
    waypoint.created_time = now_seconds();
    waypoint.last_modified = waypoint.created_time;

    auto & map_waypoints = waypoints_by_map_[ensure_map_cache(map_id)];
    map_waypoints[waypoint_type_to_string(waypoint.type)][waypoint.id] = waypoint;

    refresh_waypoints_revision(map_id);
    if (config_.data_storage_enabled) {
      save_waypoints_data(map_id);
    }
    current_map_id_ = normalize_map_id(map_id);
    publish_waypoints_data();

    rapidjson::Document response_doc;
    response_doc.SetObject();
    response_doc.AddMember(
      "map_id",
      rapidjson::Value(normalize_map_id(map_id).c_str(), response_doc.GetAllocator()).Move(),
      response_doc.GetAllocator());
    send_app_response("success", "点位 '" + waypoint.name + "' 设置成功", response_doc, map_id);
  }

  void handle_update_waypoint(const rapidjson::Value & command)
  {
    // 更新点位：只覆盖命令里显式携带的字段，未携带的字段保持原值。
    // properties 采用增量合并策略，方便 APP 只更新速度、播报词等局部属性。
    // 目标行为：查找原点位 -> 增量更新字段/properties -> 刷新 revision -> 保存 -> 推送。
    const auto * waypoint_data = find_object_member(command, "waypoint_data");
    if (waypoint_data == nullptr) {
      send_app_response("error", "缺少必要参数: id 或 type");
      return;
    }

    const std::string waypoint_id =
      read_string_member(*waypoint_data, "id", read_string_member(*waypoint_data, "waypoint_id", ""));
    const std::string waypoint_type =
      read_string_member(*waypoint_data, "type", read_string_member(command, "waypoint_type", ""));
    const std::string map_id = extract_command_map_id(command, waypoint_data);
    if (waypoint_id.empty() || waypoint_type.empty()) {
      send_app_response("error", "缺少必要参数: id 或 type", rapidjson::Value(rapidjson::kObjectType), map_id);
      return;
    }
    if (!is_valid_waypoint_type(waypoint_type)) {
      send_app_response(
        "error",
        "更新点位失败: '" + waypoint_type + "' is not a valid WaypointType",
        rapidjson::Value(rapidjson::kObjectType),
        map_id);
      return;
    }

    const std::string normalized_map_id = ensure_map_cache(map_id);
    auto & bucket = waypoints_by_map_[normalized_map_id][waypoint_type];
    auto waypoint_it = bucket.find(waypoint_id);
    if (waypoint_it == bucket.end()) {
      send_app_response("error", "地图 " + normalized_map_id + " 下点位不存在: " + waypoint_id);
      return;
    }

    std::string normalized_properties = "{}";
    std::string error_message;
    if (!normalize_waypoint_speed_properties(*waypoint_data, normalized_properties, error_message)) {
      send_app_response("error", error_message, rapidjson::Value(rapidjson::kObjectType), normalized_map_id);
      return;
    }

    auto & waypoint = waypoint_it->second;
    waypoint.name = read_string_member(
      *waypoint_data, "name", read_string_member(*waypoint_data, "waypoint_name", waypoint.name));
    waypoint.position = read_double_array(*waypoint_data, "position", waypoint.position);
    waypoint.orientation = read_double_array(*waypoint_data, "orientation", waypoint.orientation);
    waypoint.frame_id = read_string_member(*waypoint_data, "frame_id", waypoint.frame_id);
    waypoint.map_id = normalized_map_id;
    waypoint.properties_json = merge_properties_json(waypoint.properties_json, normalized_properties);
    waypoint.last_modified = now_seconds();

    refresh_waypoints_revision(normalized_map_id);
    if (config_.data_storage_enabled) {
      save_waypoints_data(normalized_map_id);
    }
    current_map_id_ = normalized_map_id;
    publish_waypoints_data();
    send_app_response("success", "点位 '" + waypoint.name + "' 更新成功", rapidjson::Value(rapidjson::kObjectType), normalized_map_id);
  }

  void handle_delete_waypoint(const rapidjson::Value & command)
  {
    // 删除点位：按 map_id + waypoint_id 精确删除。
    // 删除成功后刷新 revision，避免路线运行层继续使用旧点位缓存。
    // 目标行为：按 map_id/type/id 精确删除，避免不同地图同 ID 点位互相影响。
    const std::string waypoint_id = read_string_member(command, "waypoint_id", "");
    const std::string waypoint_type = read_string_member(command, "waypoint_type", "");
    const std::string map_id = extract_command_map_id(command, nullptr);
    if (waypoint_id.empty() || waypoint_type.empty()) {
      send_app_response("error", "缺少必要参数: waypoint_id 或 waypoint_type", rapidjson::Value(rapidjson::kObjectType), map_id);
      return;
    }
    if (!is_valid_waypoint_type(waypoint_type)) {
      send_app_response(
        "error",
        "删除点位失败: '" + waypoint_type + "' is not a valid WaypointType",
        rapidjson::Value(rapidjson::kObjectType),
        map_id);
      return;
    }

    const std::string normalized_map_id = ensure_map_cache(map_id);
    auto & bucket = waypoints_by_map_[normalized_map_id][waypoint_type];
    const auto waypoint_it = bucket.find(waypoint_id);
    if (waypoint_it == bucket.end()) {
      send_app_response("error", "地图 " + normalized_map_id + " 下点位不存在: " + waypoint_id);
      return;
    }

    const std::string waypoint_name = waypoint_it->second.name;
    bucket.erase(waypoint_it);

    refresh_waypoints_revision(normalized_map_id);
    if (config_.data_storage_enabled) {
      save_waypoints_data(normalized_map_id);
    }
    current_map_id_ = normalized_map_id;
    publish_waypoints_data();
    send_app_response("success", "点位 '" + waypoint_name + "' 删除成功", rapidjson::Value(rapidjson::kObjectType), normalized_map_id);
  }

  void handle_get_waypoints(const rapidjson::Value & command)
  {
    // 查询点位：支持按地图、点位类型和 include_details 筛选输出。
    // include_details=false 时只返回轻量字段，减少 APP 高频刷新时的 JSON 体积。
    // 目标行为：支持查询单类型或 all，include_details 控制返回完整点位或仅 ID 列表。
    const std::string waypoint_type = read_string_member(command, "waypoint_type", "");
    const bool include_details =
      command.HasMember("include_details") && command["include_details"].IsBool() ?
      command["include_details"].GetBool() :
      true;
    const std::string map_id = ensure_map_cache(extract_command_map_id(command, nullptr));

    rapidjson::Document response_doc;
    response_doc.SetObject();
    auto & allocator = response_doc.GetAllocator();
    response_doc.AddMember("map_id", rapidjson::Value(map_id.c_str(), allocator).Move(), allocator);

    const auto & map_waypoints = waypoints_by_map_[map_id];
    if (!waypoint_type.empty() && waypoint_type != "all") {
      const auto type_it = map_waypoints.find(waypoint_type);
      if (type_it == map_waypoints.end()) {
        send_app_response("error", "无效的点位类型: " + waypoint_type, rapidjson::Value(rapidjson::kObjectType), map_id);
        return;
      }
      response_doc.AddMember(
        rapidjson::Value(waypoint_type.c_str(), allocator).Move(),
        waypoint_bucket_response_to_json(type_it->second, include_details, allocator),
        allocator);
    } else {
      for (const auto & [type_name, bucket] : map_waypoints) {
        response_doc.AddMember(
          rapidjson::Value(type_name.c_str(), allocator).Move(),
          waypoint_bucket_response_to_json(bucket, include_details, allocator),
          allocator);
      }
    }

    response_doc.AddMember(
      "waypoints_revision",
      rapidjson::Value(waypoints_revisions_by_map_[map_id].c_str(), allocator).Move(),
      allocator);
    send_app_response("success", "获取地图 " + map_id + " 点位列表成功", response_doc, map_id);
  }

  void handle_clear_waypoints(const rapidjson::Value & command)
  {
    // 清空点位：支持清空单张地图、指定类型点位或全部地图点位。
    // 这是有破坏性的写文件操作，必须在响应中明确返回 affected_count。
    // 目标行为：多地图模式下默认必须指定 map_id；clear_scope=all_maps 才允许清空所有地图。
    const std::string waypoint_type = read_string_member(command, "waypoint_type", "");
    const std::string raw_map_id = read_string_member(command, "map_id", "");
    const std::string clear_scope = trim_copy(read_string_member(command, "clear_scope", ""));
    if (raw_map_id.empty() && clear_scope != "all_maps") {
      rapidjson::Document data;
      data.SetObject();
      data.AddMember("error_code", "missing_map_id", data.GetAllocator());
      send_app_response(
        "error",
        "多地图模式下 clear_waypoints 必须携带 map_id，避免误删其他地图点位",
        data,
        config_.default_map_id);
      return;
    }

    const std::string map_id = ensure_map_cache(extract_command_map_id(command, nullptr));
    size_t cleared_count = 0;

    if (clear_scope == "all_maps") {
      cleared_count = get_total_waypoints_count();
      for (auto & [target_map_id, map_waypoints] : waypoints_by_map_) {
        map_waypoints = make_empty_waypoint_bucket();
        refresh_waypoints_revision(target_map_id);
        if (config_.data_storage_enabled) {
          save_waypoints_data(target_map_id);
        }
      }
      current_map_id_ = map_id;
      publish_waypoints_data("clear_all_maps");
      send_clear_response("清空所有地图点位成功，共 " + std::to_string(cleared_count) + " 个", map_id, cleared_count, "all_maps");
      return;
    }

    auto & map_waypoints = waypoints_by_map_[map_id];
    if (!waypoint_type.empty()) {
      auto type_it = map_waypoints.find(waypoint_type);
      if (type_it == map_waypoints.end()) {
        send_app_response("error", "无效的点位类型: " + waypoint_type, rapidjson::Value(rapidjson::kObjectType), map_id);
        return;
      }
      cleared_count = type_it->second.size();
      type_it->second.clear();
    } else {
      cleared_count = get_total_waypoints_count(map_id);
      for (auto & [unused_type, bucket] : map_waypoints) {
        (void)unused_type;
        bucket.clear();
      }
    }

    refresh_waypoints_revision(map_id);
    if (config_.data_storage_enabled) {
      save_waypoints_data(map_id);
    }
    current_map_id_ = map_id;
    publish_waypoints_data();
    const std::string clear_message = waypoint_type.empty() ?
      "清空地图 " + map_id + " 所有点位成功，共 " + std::to_string(cleared_count) + " 个" :
      "清空地图 " + map_id + " 的 " + waypoint_type + " 类型点位成功，共 " +
        std::to_string(cleared_count) + " 个";
    send_clear_response(clear_message, map_id, cleared_count, "");
  }

  // ===========================================================================
  // 5. APP 导航命令桥接：校验、ID 归一化、透传到 /navigation/requests
  // ===========================================================================

  void bridge_navigation_command(const std::string & raw_json)
  {
    rapidjson::Document command;
    command.Parse(raw_json.c_str());
    if (command.HasParseError() || !command.IsObject()) {
      send_app_response("error", "导航命令 JSON 解析失败");
      return;
    }

    const std::string command_type = read_string_member(command, "command_type", "");
    RCLCPP_INFO(get_logger(), "收到 APP 导航命令: %s - 转发给路线运行层", command_type.c_str());

    if (!validate_navigation_command(command)) {
      send_app_response("error", "导航命令校验失败: " + command_type);
      return;
    }

    rapidjson::Document normalized_command = normalize_navigation_command(command);
    send_navigation_request(normalized_command);
  }

  bool validate_navigation_command(const rapidjson::Value & command)
  {
    // 导航命令轻校验：只检查命令类型、必要 ID 和路线任务数据结构。
    // 这里不判断目标是否可达，也不判断定位是否 ready，这些由下游运行层负责。
    // 这里必须保持“宽进严出”：字段缺失只记录 warning 并转发，让路线运行层返回精确错误码。
    const std::string command_type = read_string_member(command, "command_type", "");

    if (command_type == "start_route_task") {
      if (read_string_member(command, "task_session_id", "").empty()) {
        RCLCPP_WARN(get_logger(), "路线任务启动字段缺失: task_session_id，继续转发给路线运行层返回业务 ack");
      }
      if (read_string_member(command, "route_id", "").empty()) {
        RCLCPP_WARN(get_logger(), "路线任务启动字段缺失: route_id，继续转发给路线运行层返回业务 ack");
      }

      const bool has_route_waypoints =
        command.HasMember("route_waypoints") && command["route_waypoints"].IsArray() &&
        !command["route_waypoints"].Empty();
      const bool has_route_waypoint_ids =
        command.HasMember("route_waypoint_ids") && command["route_waypoint_ids"].IsArray() &&
        !command["route_waypoint_ids"].Empty();
      if (!has_route_waypoints && !has_route_waypoint_ids) {
        RCLCPP_WARN(
          get_logger(),
          "路线任务启动字段异常: route_waypoints/route_waypoint_ids 均为空或不是数组，继续转发给路线运行层");
      }
      if (has_route_waypoints && has_route_waypoint_ids) {
        RCLCPP_WARN(
          get_logger(),
          "路线任务启动字段异常: route_waypoints 和 route_waypoint_ids 同时存在，继续转发给路线运行层");
      }
      if (has_route_waypoint_ids && read_string_member(command, "waypoints_revision", "").empty()) {
        RCLCPP_WARN(
          get_logger(),
          "路线任务 ID 列表模式缺少 waypoints_revision，继续转发给路线运行层返回 missing_waypoints_revision");
      }
    } else if (command_type == "jump_to_waypoint") {
      if (read_string_member(command, "task_session_id", "").empty() ||
        read_string_member(command, "target_waypoint_id", "").empty())
      {
        RCLCPP_WARN(
          get_logger(),
          "路线任务跳转字段缺失: task_session_id 或 target_waypoint_id，继续转发给路线运行层返回业务 ack");
      }
    } else if (
      command_type == "pause_route_task" ||
      command_type == "resume_route_task" ||
      command_type == "stop_route_task")
    {
      if (read_string_member(command, "task_session_id", "").empty() ||
        read_string_member(command, "route_id", "").empty())
      {
        RCLCPP_WARN(
          get_logger(),
          "路线任务控制字段缺失: %s 需要 task_session_id 和 route_id，继续转发给路线运行层返回业务 ack",
          command_type.c_str());
      }
    } else if (command_type == "broadcast_finished") {
      const char * required_fields[] = {"task_session_id", "route_id", "waypoint_id", "broadcast_id"};
      for (const auto * field : required_fields) {
        if (read_string_member(command, field, "").empty()) {
          RCLCPP_WARN(get_logger(), "播报完成字段缺失: %s，继续转发给路线运行层返回业务 ack", field);
        }
      }
    }

    return true;
  }

  rapidjson::Document normalize_navigation_command(const rapidjson::Value & command) const
  {
    // 导航命令归一化：把 APP 可能传来的数字 ID 统一转成字符串。
    // 这样路线任务中 waypoint_id、route_id、task_id 的比较不会受 JSON 数字/字符串差异影响。
    // 顶层 ID、route_waypoint_ids、route_waypoints[].waypoint_id 都要字符串化，不能悄悄删除空 ID。
    rapidjson::Document normalized;
    normalized.SetObject();
    auto & allocator = normalized.GetAllocator();
    normalized.CopyFrom(command, allocator);

    const char * top_level_id_fields[] = {
      "waypoint_id",
      "target_waypoint_id",
      "task_session_id",
      "route_id",
      "map_id",
      "broadcast_id",
      "request_message_id",
      "waypoints_revision",
    };

    for (const auto * field : top_level_id_fields) {
      if (normalized.HasMember(field) && !normalized[field].IsNull()) {
        const std::string string_value = json_scalar_to_string(normalized[field]);
        normalized[field].SetString(string_value.c_str(), allocator);
      }
    }

    if (normalized.HasMember("route_waypoint_ids") && normalized["route_waypoint_ids"].IsArray()) {
      for (auto & item : normalized["route_waypoint_ids"].GetArray()) {
        const std::string string_value = item.IsNull() ? "" : trim_copy(json_scalar_to_string(item));
        item.SetString(string_value.c_str(), allocator);
      }
    }

    if (normalized.HasMember("route_waypoints") && normalized["route_waypoints"].IsArray()) {
      for (auto & waypoint : normalized["route_waypoints"].GetArray()) {
        if (!waypoint.IsObject()) {
          continue;
        }
        if (waypoint.HasMember("waypoint_id") && !waypoint["waypoint_id"].IsNull()) {
          const std::string waypoint_id = json_scalar_to_string(waypoint["waypoint_id"]);
          waypoint["waypoint_id"].SetString(waypoint_id.c_str(), allocator);
        }
        if (waypoint.HasMember("map_id") && !waypoint["map_id"].IsNull()) {
          const std::string map_id = normalize_map_id(json_scalar_to_string(waypoint["map_id"]));
          waypoint["map_id"].SetString(map_id.c_str(), allocator);
        }
      }
    }

    return normalized;
  }

  void send_navigation_request(const rapidjson::Value & command_data)
  {
    // 转发导航请求：保持原始 command_type，同时补齐 request_message_id 和时间戳。
    // 下游路线运行层只订阅 /navigation/requests，不直接接触 APP 原始命令话题。
    // 输出格式必须保持：
    // {
    //   "request_type": "navigation_command",
    //   "command_data": <归一化后的 APP 命令>,
    //   "timestamp": <当前时间>,
    //   "source": "waypoints_manager"
    // }
    rapidjson::Document request;
    request.SetObject();
    auto & allocator = request.GetAllocator();
    request.AddMember("request_type", "navigation_command", allocator);
    rapidjson::Value command_copy;
    command_copy.CopyFrom(command_data, allocator);
    request.AddMember("command_data", command_copy, allocator);
    request.AddMember("timestamp", now_seconds(), allocator);
    request.AddMember("source", "waypoints_manager", allocator);

    std_msgs::msg::String msg;
    msg.data = json_document_to_string(request, false);
    navigation_request_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "导航请求已发送: %s",
      read_string_member(command_data, "command_type", "").c_str());
  }

  // ===========================================================================
  // 6. 输出链路：点位库同步和 APP response
  // ===========================================================================

  void publish_waypoints_data(const std::string & update_type = "full_update")
  {
    // 发布点位库同步消息：路线运行层通过 revision 判断是否需要刷新缓存。
    // reason 用于区分 initial_load、set_waypoint、update_waypoint、delete_waypoint 等触发来源。
    // 这是状态机拿到点位库的关键输出，必须包含：
    // map_id、default_map_id、waypoints_revision、waypoints_revisions_by_map、
    // data.waypoints、data.waypoints_by_map、metadata.total_count。
    const std::string normalized_map_id = ensure_map_cache(current_map_id_);

    rapidjson::Document doc;
    doc.SetObject();
    auto & allocator = doc.GetAllocator();

    rapidjson::Value payload(rapidjson::kObjectType);
    payload.AddMember(
      "update_type", rapidjson::Value(update_type.c_str(), allocator).Move(), allocator);
    payload.AddMember("timestamp", now_seconds(), allocator);
    payload.AddMember(
      "map_id", rapidjson::Value(normalized_map_id.c_str(), allocator).Move(), allocator);
    payload.AddMember(
      "default_map_id", rapidjson::Value(config_.default_map_id.c_str(), allocator).Move(), allocator);
    payload.AddMember(
      "waypoints_revision",
      rapidjson::Value(waypoints_revisions_by_map_[normalized_map_id].c_str(), allocator).Move(),
      allocator);
    payload.AddMember(
      "waypoints_revisions_by_map",
      revisions_to_json(allocator),
      allocator);

    rapidjson::Value data(rapidjson::kObjectType);
    data.AddMember(
      "waypoints",
      waypoints_for_map_to_json(normalized_map_id, allocator),
      allocator);
    data.AddMember(
      "waypoints_by_map",
      waypoints_by_map_to_json(allocator),
      allocator);
    payload.AddMember("data", data, allocator);

    rapidjson::Value metadata(rapidjson::kObjectType);
    metadata.AddMember("total_count", static_cast<uint64_t>(get_total_waypoints_count(normalized_map_id)), allocator);
    metadata.AddMember("total_count_all_maps", static_cast<uint64_t>(get_total_waypoints_count()), allocator);
    metadata.AddMember(
      "map_id", rapidjson::Value(normalized_map_id.c_str(), allocator).Move(), allocator);
    metadata.AddMember(
      "waypoints_revision",
      rapidjson::Value(waypoints_revisions_by_map_[normalized_map_id].c_str(), allocator).Move(),
      allocator);
    metadata.AddMember("waypoints_revisions_by_map", revisions_to_json(allocator), allocator);
    payload.AddMember("metadata", metadata, allocator);

    rapidjson::Value unified = create_unified_message(
      "push", "waypoints_data", "waypoints_manager", "all", payload, allocator);
    doc.CopyFrom(unified, allocator);

    std_msgs::msg::String msg;
    msg.data = json_document_to_string(doc, false);
    waypoints_data_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "已发布地图 %s 点位数据: update_type=%s, total=%zu",
      normalized_map_id.c_str(),
      update_type.c_str(),
      get_total_waypoints_count(normalized_map_id));
  }

  void publish_initial_waypoints_data()
  {
    // 启动点位库同步：在节点启动后重复发布几次，照顾后启动的订阅者。
    // 当本地没有任何点位时不发布 initial_load，避免空数据覆盖下游已有缓存。
    // 启动后重复发布本地点位库，解决路线运行层后启动时错过初始数据的问题。
    if (initial_publish_timer_ == nullptr) {
      return;
    }
    if (initial_publish_count_ >= config_.initial_waypoints_publish_max) {
      initial_publish_timer_->cancel();
      return;
    }
    ++initial_publish_count_;
    publish_waypoints_data("initial_load");
  }

  void send_app_response(const std::string & response_type, const std::string & message)
  {
    rapidjson::Document empty;
    empty.SetObject();
    send_app_response(response_type, message, empty, current_map_id_);
  }

  void send_app_response(
    const std::string & response_type,
    const std::string & message,
    const rapidjson::Value & data,
    const std::string & map_id)
  {
    // APP 响应发布：waypoint_response 继续走 /navigation/waypoints_data。
    // 这是当前 APP 桥接链路使用的协议，不能随意换话题，否则前端会收不到点位命令响应。
    const std::string normalized_map_id = ensure_map_cache(map_id);

    rapidjson::Document doc;
    doc.SetObject();
    auto & allocator = doc.GetAllocator();

    rapidjson::Value result_data(rapidjson::kObjectType);
    if (data.IsObject()) {
      result_data.CopyFrom(data, allocator);
    }
    if (!result_data.HasMember("map_id")) {
      result_data.AddMember(
        "map_id", rapidjson::Value(normalized_map_id.c_str(), allocator).Move(), allocator);
    }
    if (!result_data.HasMember("waypoints_revision")) {
      result_data.AddMember(
        "waypoints_revision",
        rapidjson::Value(waypoints_revisions_by_map_[normalized_map_id].c_str(), allocator).Move(),
        allocator);
    }
    if (!result_data.HasMember("waypoints_revisions_by_map")) {
      result_data.AddMember("waypoints_revisions_by_map", revisions_to_json(allocator), allocator);
    }

    rapidjson::Value response_payload(rapidjson::kObjectType);
    response_payload.AddMember(
      "response_type", rapidjson::Value(response_type.c_str(), allocator).Move(), allocator);
    response_payload.AddMember(
      "message", rapidjson::Value(message.c_str(), allocator).Move(), allocator);
    response_payload.AddMember(
      "map_id", rapidjson::Value(normalized_map_id.c_str(), allocator).Move(), allocator);
    response_payload.AddMember(
      "waypoints_revision",
      rapidjson::Value(waypoints_revisions_by_map_[normalized_map_id].c_str(), allocator).Move(),
      allocator);
    response_payload.AddMember(
      "request_message_id",
      rapidjson::Value(active_waypoint_request_message_id_.c_str(), allocator).Move(),
      allocator);
    response_payload.AddMember("result", result_data, allocator);

    rapidjson::Value unified = create_unified_message(
      "response", "waypoint_response", "waypoints_manager", "all", response_payload, allocator);
    doc.CopyFrom(unified, allocator);
    if (response_type == "error") {
      doc["metadata"]["status"].SetString("error", allocator);
      doc["metadata"]["error_message"].SetString(message.c_str(), allocator);
    }
    doc["metadata"]["request_id"].SetString(active_waypoint_request_message_id_.c_str(), allocator);

    std_msgs::msg::String msg;
    msg.data = json_document_to_string(doc, false);
    waypoints_data_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "点位响应: [%s] %s", response_type.c_str(), message.c_str());
  }

  rapidjson::Value create_unified_message(
    const std::string & message_type,
    const std::string & data_type,
    const std::string & source,
    const std::string & destination,
    rapidjson::Value & data,
    rapidjson::Document::AllocatorType & allocator) const
  {
    // 统一消息外壳是 APP、websocket、data_integration 共同依赖的协议结构。
    // 新增业务字段应放在 data 内，外壳字段尽量保持稳定，避免破坏跨进程解析。
    rapidjson::Value message(rapidjson::kObjectType);
    message.AddMember("protocol_version", "2.0", allocator);
    const std::string message_id =
      message_type + "_" + std::to_string(static_cast<int64_t>(now_seconds()));
    message.AddMember("message_id", rapidjson::Value(message_id.c_str(), allocator).Move(), allocator);
    message.AddMember("timestamp", now_seconds(), allocator);
    message.AddMember("message_type", rapidjson::Value(message_type.c_str(), allocator).Move(), allocator);
    message.AddMember("data_type", rapidjson::Value(data_type.c_str(), allocator).Move(), allocator);
    message.AddMember("source", rapidjson::Value(source.c_str(), allocator).Move(), allocator);
    message.AddMember("destination", rapidjson::Value(destination.c_str(), allocator).Move(), allocator);
    message.AddMember("data", data, allocator);

    rapidjson::Value metadata(rapidjson::kObjectType);
    metadata.AddMember("status", "success", allocator);
    metadata.AddMember("error_code", "", allocator);
    metadata.AddMember("error_message", "", allocator);
    metadata.AddMember("request_id", "", allocator);
    message.AddMember("metadata", metadata, allocator);
    return message;
  }

  void send_clear_response(
    const std::string & message,
    const std::string & map_id,
    const size_t cleared_count,
    const std::string & clear_scope)
  {
    rapidjson::Document data;
    data.SetObject();
    auto & allocator = data.GetAllocator();
    data.AddMember("map_id", rapidjson::Value(map_id.c_str(), allocator).Move(), allocator);
    data.AddMember("cleared_count", static_cast<uint64_t>(cleared_count), allocator);
    if (!clear_scope.empty()) {
      data.AddMember(
        "clear_scope", rapidjson::Value(clear_scope.c_str(), allocator).Move(), allocator);
    }
    send_app_response("success", message, data, map_id);
  }

  // ===========================================================================
  // 7. 点位缓存工具：map_id、revision、统计和查找
  // ===========================================================================

  std::string normalize_map_id(const std::string & map_id) const
  {
    // 地图 ID 归一化：空值统一落到默认地图，避免无 map_id 命令写入不可预期文件。
    // 空 map_id 切换到默认地图，保留旧 APP 没传 map_id 时默认归入 hall 的语义。
    const std::string normalized = trim_copy(map_id);
    if (!normalized.empty()) {
      return normalized;
    }
    return config_.default_map_id.empty() ? "hall" : config_.default_map_id;
  }

  WaypointsByType make_empty_waypoint_bucket() const
  {
    // 创建空点位桶：每张地图独立维护 revision、waypoints 数组和 metadata。
    // 每张地图都按点位类型分桶，避免不同类型同 ID 点位互相覆盖。
    return {
      {"navigation_target", {}},
      {"exhibition_point", {}},
      {"obstacle_point", {}},
      {"charging_point", {}},
      {"rest_point", {}},
      {"landmark_point", {}},
    };
  }

  std::string ensure_map_cache(const std::string & map_id)
  {
    // 保证地图点位桶和 revision 都存在；current_map_id 对应的缓存是默认操作目标。
    const std::string normalized_map_id = normalize_map_id(map_id);
    if (waypoints_by_map_.find(normalized_map_id) == waypoints_by_map_.end()) {
      waypoints_by_map_[normalized_map_id] = make_empty_waypoint_bucket();
    }
    if (waypoints_revisions_by_map_.find(normalized_map_id) == waypoints_revisions_by_map_.end()) {
      waypoints_revisions_by_map_[normalized_map_id] = "0.000";
    }
    return normalized_map_id;
  }

  void refresh_waypoints_revision(const std::string & map_id)
  {
    // 刷新点位库 revision：任何会改变点位内容的操作都必须调用。
    // revision 是 APP 用 ID 列表启动路线时的一致性保护，必须在点位真实变化后刷新。
    const std::string normalized_map_id = ensure_map_cache(map_id);
    waypoints_revisions_by_map_[normalized_map_id] = now_revision();
  }

  std::string waypoint_file_path(const std::string & map_id) const
  {
    // 把 map_id 中非字母数字/-/_ 的字符替换成 _，避免非法路径。
    std::string safe_map_id;
    for (const char ch : normalize_map_id(map_id)) {
      const auto uch = static_cast<unsigned char>(ch);
      if (std::isalnum(uch) || ch == '-' || ch == '_') {
        safe_map_id.push_back(ch);
      } else {
        safe_map_id.push_back('_');
      }
    }
    return (fs::path(config_.waypoints_dir) / (safe_map_id + ".json")).string();
  }

  size_t get_total_waypoints_count(const std::optional<std::string> & map_id = std::nullopt) const
  {
    // 统计所有地图的点位总数，用于状态日志和同步消息 metadata。
    // map_id 为空时统计所有地图；有 map_id 时只统计指定地图。
    size_t total = 0;
    if (map_id.has_value()) {
      const auto map_it = waypoints_by_map_.find(normalize_map_id(map_id.value()));
      if (map_it == waypoints_by_map_.end()) {
        return 0;
      }
      for (const auto & [unused_type, bucket] : map_it->second) {
        (void)unused_type;
        total += bucket.size();
      }
      return total;
    }

    for (const auto & [unused_map_id, waypoints_by_type] : waypoints_by_map_) {
      (void)unused_map_id;
      for (const auto & [unused_type, bucket] : waypoints_by_type) {
        (void)unused_type;
        total += bucket.size();
      }
    }
    return total;
  }

  // ===========================================================================
  // 8. 文件持久化：加载、兼容导入、保存
  // ===========================================================================

  void save_waypoints_data(const std::string & map_id)
  {
    // 保存点位数据：按 map_id 分文件落盘，避免多地图点位互相覆盖。
    // 保存失败只记录错误，不直接退出节点，保证 APP 控制链路继续可用。
    // 输出 data/waypoints/<map_id>.json，包含 map_id、waypoints_revision、waypoints、timestamp。
    const std::string normalized_map_id = ensure_map_cache(map_id);
    if (waypoints_revisions_by_map_[normalized_map_id].empty()) {
      refresh_waypoints_revision(normalized_map_id);
    }

    rapidjson::Document doc;
    doc.SetObject();
    auto & allocator = doc.GetAllocator();

    doc.AddMember(
      "map_id",
      rapidjson::Value(normalized_map_id.c_str(), allocator).Move(),
      allocator);
    doc.AddMember(
      "waypoints_revision",
      rapidjson::Value(waypoints_revisions_by_map_[normalized_map_id].c_str(), allocator).Move(),
      allocator);

    rapidjson::Value waypoints_object(rapidjson::kObjectType);
    for (const auto & [type_name, bucket] : waypoints_by_map_[normalized_map_id]) {
      rapidjson::Value type_object(rapidjson::kObjectType);
      for (const auto & [waypoint_id, waypoint] : bucket) {
        rapidjson::Value waypoint_object = waypoint_to_json(waypoint, allocator);
        type_object.AddMember(
          rapidjson::Value(waypoint_id.c_str(), allocator).Move(),
          waypoint_object,
          allocator);
      }
      waypoints_object.AddMember(
        rapidjson::Value(type_name.c_str(), allocator).Move(),
        type_object,
        allocator);
    }
    doc.AddMember("waypoints", waypoints_object, allocator);
    doc.AddMember("timestamp", now_seconds(), allocator);

    const std::string file_path = waypoint_file_path(normalized_map_id);
    try {
      fs::create_directories(fs::path(file_path).parent_path());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "创建点位文件目录失败: %s", error.what());
      return;
    }

    FILE * file = std::fopen(file_path.c_str(), "wb");
    if (file == nullptr) {
      RCLCPP_ERROR(get_logger(), "打开点位文件写入失败: %s", file_path.c_str());
      return;
    }

    char buffer[65536];
    rapidjson::FileWriteStream stream(file, buffer, sizeof(buffer));
    rapidjson::PrettyWriter<rapidjson::FileWriteStream> writer(stream);
    writer.SetIndent(' ', 2);
    doc.Accept(writer);
    std::fclose(file);

    RCLCPP_INFO(get_logger(), "地图 %s 点位数据保存成功: %s", normalized_map_id.c_str(), file_path.c_str());
  }

  bool load_waypoints_data()
  {
    // 加载点位数据：优先扫描多地图目录，每个 JSON 文件对应一张地图。
    // 遍历 data/waypoints/*.json，加载每张地图自己的点位桶和 revision。
    if (!fs::is_directory(config_.waypoints_dir)) {
      return false;
    }

    bool loaded_any = false;
    for (const auto & entry : fs::directory_iterator(config_.waypoints_dir)) {
      if (!entry.is_regular_file() || entry.path().extension() != ".json") {
        continue;
      }

      auto doc = read_json_file(entry.path().string());
      if (!doc.has_value() || !doc->IsObject()) {
        RCLCPP_WARN(get_logger(), "加载地图点位文件失败: %s", entry.path().c_str());
        continue;
      }

      std::string map_id = entry.path().stem().string();
      if (doc->HasMember("map_id") && (*doc)["map_id"].IsString()) {
        map_id = (*doc)["map_id"].GetString();
      }
      load_waypoints_payload_into_map(*doc, map_id);
      loaded_any = true;
    }

    if (loaded_any) {
      RCLCPP_INFO(
        get_logger(),
        "多地图点位加载完成: maps=%zu, total_waypoints=%zu",
        waypoints_by_map_.size(),
        get_total_waypoints_count());
    }
    return loaded_any;
  }

  void load_legacy_waypoints_data(const std::string & map_id)
  {
    // 兼容导入单文件点位库：用于把早期单地图数据导入默认地图桶。
    // 如果没有多地图 JSON，则从旧 data/dynamic_waypoints.json 读入默认地图并保存成新格式。
    auto doc = read_json_file(config_.storage_file_path);
    if (!doc.has_value() || !doc->IsObject()) {
      RCLCPP_WARN(get_logger(), "旧点位文件读取失败: %s", config_.storage_file_path.c_str());
      return;
    }

    load_waypoints_payload_into_map(*doc, map_id);
    RCLCPP_INFO(
      get_logger(),
      "兼容单文件点位库导入到地图 %s 完成，共 %zu 个点位",
      normalize_map_id(map_id).c_str(),
      get_total_waypoints_count(normalize_map_id(map_id)));
  }

  void load_waypoints_payload_into_map(const rapidjson::Document & data, const std::string & map_id)
  {
    // 把 JSON payload 写入指定地图缓存：负责解析 revision、metadata 和 waypoints 数组。
    // 把一个 JSON payload 写入指定地图缓存，同时恢复 created_time/last_modified/properties。
    const std::string normalized_map_id = ensure_map_cache(map_id);
    auto & map_waypoints = waypoints_by_map_[normalized_map_id];

    if (data.HasMember("waypoints_revision") && data["waypoints_revision"].IsString()) {
      waypoints_revisions_by_map_[normalized_map_id] = data["waypoints_revision"].GetString();
    } else if (data.HasMember("timestamp") && data["timestamp"].IsNumber()) {
      std::ostringstream stream;
      stream.setf(std::ios::fixed);
      stream.precision(3);
      stream << data["timestamp"].GetDouble();
      waypoints_revisions_by_map_[normalized_map_id] = stream.str();
    } else {
      refresh_waypoints_revision(normalized_map_id);
    }

    if (!data.HasMember("waypoints") || !data["waypoints"].IsObject()) {
      return;
    }

    const auto & waypoints_object = data["waypoints"];
    for (auto type_it = waypoints_object.MemberBegin(); type_it != waypoints_object.MemberEnd(); ++type_it) {
      const std::string type_name = type_it->name.GetString();
      if (map_waypoints.find(type_name) == map_waypoints.end() || !type_it->value.IsObject()) {
        continue;
      }

      auto & bucket = map_waypoints[type_name];
      for (auto wp_it = type_it->value.MemberBegin(); wp_it != type_it->value.MemberEnd(); ++wp_it) {
        if (!wp_it->value.IsObject()) {
          continue;
        }
        try {
          WaypointData waypoint = waypoint_from_json(wp_it->value, normalized_map_id, type_name);
          bucket[waypoint.id.empty() ? std::string(wp_it->name.GetString()) : waypoint.id] = waypoint;
        } catch (const std::exception & error) {
          RCLCPP_WARN(
            get_logger(),
            "加载地图 %s 点位失败: %s - %s",
            normalized_map_id.c_str(),
            wp_it->name.GetString(),
            error.what());
        }
      }
    }
  }

  std::optional<rapidjson::Document> read_json_file(const std::string & file_path) const
  {
    std::ifstream input(file_path);
    if (!input.is_open()) {
      return std::nullopt;
    }

    std::stringstream buffer;
    buffer << input.rdbuf();

    rapidjson::Document doc;
    doc.Parse(buffer.str().c_str());
    if (doc.HasParseError()) {
      RCLCPP_WARN(
        get_logger(),
        "JSON 解析失败: %s, offset=%zu, error=%s",
        file_path.c_str(),
        doc.GetErrorOffset(),
        rapidjson::GetParseError_En(doc.GetParseError()));
      return std::nullopt;
    }
    return doc;
  }

  std::string json_document_to_string(const rapidjson::Document & doc, const bool pretty) const
  {
    rapidjson::StringBuffer buffer;
    if (pretty) {
      rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
      writer.SetIndent(' ', 2);
      doc.Accept(writer);
    } else {
      rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
      doc.Accept(writer);
    }
    return buffer.GetString();
  }

  rapidjson::Value revisions_to_json(rapidjson::Document::AllocatorType & allocator) const
  {
    rapidjson::Value object(rapidjson::kObjectType);
    for (const auto & [map_id, revision] : waypoints_revisions_by_map_) {
      object.AddMember(
        rapidjson::Value(map_id.c_str(), allocator).Move(),
        rapidjson::Value(revision.c_str(), allocator).Move(),
        allocator);
    }
    return object;
  }

  rapidjson::Value waypoint_bucket_response_to_json(
    const WaypointBucket & bucket,
    const bool include_details,
    rapidjson::Document::AllocatorType & allocator) const
  {
    if (!include_details) {
      rapidjson::Value ids(rapidjson::kArrayType);
      for (const auto & [waypoint_id, unused_waypoint] : bucket) {
        (void)unused_waypoint;
        ids.PushBack(rapidjson::Value(waypoint_id.c_str(), allocator).Move(), allocator);
      }
      return ids;
    }

    rapidjson::Value object(rapidjson::kObjectType);
    for (const auto & [waypoint_id, waypoint] : bucket) {
      object.AddMember(
        rapidjson::Value(waypoint_id.c_str(), allocator).Move(),
        waypoint_to_json(waypoint, allocator),
        allocator);
    }
    return object;
  }

  rapidjson::Value waypoints_for_map_to_json(
    const std::string & map_id,
    rapidjson::Document::AllocatorType & allocator) const
  {
    rapidjson::Value object(rapidjson::kObjectType);
    const auto map_it = waypoints_by_map_.find(normalize_map_id(map_id));
    if (map_it == waypoints_by_map_.end()) {
      return object;
    }

    for (const auto & [type_name, bucket] : map_it->second) {
      rapidjson::Value type_object(rapidjson::kObjectType);
      for (const auto & [waypoint_id, waypoint] : bucket) {
        rapidjson::Value waypoint_object = waypoint_to_json(waypoint, allocator);
        type_object.AddMember(
          rapidjson::Value(waypoint_id.c_str(), allocator).Move(),
          waypoint_object,
          allocator);
      }
      object.AddMember(
        rapidjson::Value(type_name.c_str(), allocator).Move(),
        type_object,
        allocator);
    }
    return object;
  }

  rapidjson::Value waypoints_by_map_to_json(rapidjson::Document::AllocatorType & allocator) const
  {
    rapidjson::Value object(rapidjson::kObjectType);
    for (const auto & [map_id, unused_waypoints_by_type] : waypoints_by_map_) {
      (void)unused_waypoints_by_type;
      object.AddMember(
        rapidjson::Value(map_id.c_str(), allocator).Move(),
        waypoints_for_map_to_json(map_id, allocator),
        allocator);
    }
    return object;
  }

  rapidjson::Value waypoint_to_json(
    const WaypointData & waypoint,
    rapidjson::Document::AllocatorType & allocator) const
  {
    // 点位输出字段属于外部协议，状态机和 APP 都依赖这些字段名与结构。
    rapidjson::Value object(rapidjson::kObjectType);
    object.AddMember("id", rapidjson::Value(waypoint.id.c_str(), allocator).Move(), allocator);
    object.AddMember("waypoint_id", rapidjson::Value(waypoint.id.c_str(), allocator).Move(), allocator);
    object.AddMember("name", rapidjson::Value(waypoint.name.c_str(), allocator).Move(), allocator);
    object.AddMember("waypoint_name", rapidjson::Value(waypoint.name.c_str(), allocator).Move(), allocator);
    const std::string type_name = waypoint_type_to_string(waypoint.type);
    object.AddMember("type", rapidjson::Value(type_name.c_str(), allocator).Move(), allocator);
    object.AddMember("map_id", rapidjson::Value(waypoint.map_id.c_str(), allocator).Move(), allocator);
    write_double_array(object, "position", waypoint.position, allocator);
    write_double_array(object, "orientation", waypoint.orientation, allocator);
    object.AddMember(
      "frame_id", rapidjson::Value(waypoint.frame_id.c_str(), allocator).Move(), allocator);
    rapidjson::Value properties = json_string_to_value(waypoint.properties_json, allocator);
    if (properties.IsObject()) {
      copy_property_to_top_level(object, properties, allocator, "waypoint_role");
      copy_property_to_top_level(object, properties, allocator, "need_broadcast");
      copy_property_to_top_level(object, properties, allocator, "broadcast_id");
      copy_property_to_top_level(object, properties, allocator, "broadcast_text");
      copy_property_to_top_level(object, properties, allocator, "broadcast_blocking");
      copy_property_to_top_level(object, properties, allocator, "stop_and_align");
      copy_property_to_top_level(object, properties, allocator, "walk_direction");
      copy_property_to_top_level(object, properties, allocator, "speed");
    }
    object.AddMember("properties", properties, allocator);
    object.AddMember("created_time", waypoint.created_time, allocator);
    object.AddMember("last_modified", waypoint.last_modified, allocator);
    return object;
  }

  WaypointData waypoint_from_json(
    const rapidjson::Value & object,
    const std::string & fallback_map_id,
    const std::string & fallback_type_name) const
  {
    WaypointData waypoint;
    waypoint.id = read_string_member(object, "id", read_string_member(object, "waypoint_id", ""));
    waypoint.name = read_string_member(object, "name", read_string_member(object, "waypoint_name", waypoint.id));
    const std::string type_name = read_string_member(object, "type", fallback_type_name);
    waypoint.type = waypoint_type_from_string(type_name);
    waypoint.map_id = normalize_map_id(read_string_member(object, "map_id", fallback_map_id));
    waypoint.position = read_double_array(object, "position", {0.0, 0.0, 0.0});
    waypoint.orientation = read_double_array(object, "orientation", {0.0, 0.0, 0.0, 1.0});
    waypoint.frame_id = read_string_member(object, "frame_id", "map");
    if (object.HasMember("properties") && object["properties"].IsObject()) {
      waypoint.properties_json = json_value_to_string(object["properties"]);
    } else {
      waypoint.properties_json = "{}";
    }
    waypoint.created_time =
      object.HasMember("created_time") && object["created_time"].IsNumber() ?
      object["created_time"].GetDouble() :
      now_seconds();
    waypoint.last_modified =
      object.HasMember("last_modified") && object["last_modified"].IsNumber() ?
      object["last_modified"].GetDouble() :
      now_seconds();
    return waypoint;
  }

  std::string read_string_member(
    const rapidjson::Value & object,
    const char * key,
    const std::string & fallback) const
  {
    if (!object.HasMember(key)) {
      return fallback;
    }
    if (object[key].IsString()) {
      return object[key].GetString();
    }
    if (object[key].IsInt64()) {
      return std::to_string(object[key].GetInt64());
    }
    if (object[key].IsUint64()) {
      return std::to_string(object[key].GetUint64());
    }
    return fallback;
  }

  std::string json_scalar_to_string(const rapidjson::Value & value) const
  {
    // 仅用于业务 ID 字段，把 JSON 数字、字符串统一成字符串，避免 ID 比较出现类型差异。
    if (value.IsString()) {
      return value.GetString();
    }
    if (value.IsInt64()) {
      return std::to_string(value.GetInt64());
    }
    if (value.IsUint64()) {
      return std::to_string(value.GetUint64());
    }
    if (value.IsDouble()) {
      std::ostringstream stream;
      stream << value.GetDouble();
      return stream.str();
    }
    if (value.IsBool()) {
      return value.GetBool() ? "true" : "false";
    }
    return "";
  }

  void copy_property_to_top_level(
    rapidjson::Value & object,
    const rapidjson::Value & properties,
    rapidjson::Document::AllocatorType & allocator,
    const char * key) const
  {
    if (!properties.IsObject() || !properties.HasMember(key) || object.HasMember(key)) {
      return;
    }
    object.AddMember(
      rapidjson::Value(key, allocator).Move(),
      rapidjson::Value(properties[key], allocator),
      allocator);
  }

  const rapidjson::Value * find_object_member(
    const rapidjson::Value & object,
    const char * key) const
  {
    if (!object.IsObject() || !object.HasMember(key) || !object[key].IsObject()) {
      return nullptr;
    }
    return &object[key];
  }

  std::string extract_command_map_id(
    const rapidjson::Value & command,
    const rapidjson::Value * waypoint_data) const
  {
    // 从命令中提取地图 ID：优先顶层 map_id，其次 waypoint_data.map_id，最后使用默认地图。
    // 优先级：command.map_id -> waypoint_data.map_id -> waypoint_data.properties.map_id -> default_map_id。
    const std::string command_map_id = read_string_member(command, "map_id", "");
    if (!command_map_id.empty()) {
      return normalize_map_id(command_map_id);
    }
    if (waypoint_data != nullptr) {
      const std::string waypoint_map_id = read_string_member(*waypoint_data, "map_id", "");
      if (!waypoint_map_id.empty()) {
        return normalize_map_id(waypoint_map_id);
      }
      if (waypoint_data->HasMember("properties") && (*waypoint_data)["properties"].IsObject()) {
        const std::string properties_map_id =
          read_string_member((*waypoint_data)["properties"], "map_id", "");
        if (!properties_map_id.empty()) {
          return normalize_map_id(properties_map_id);
        }
      }
    }
    return normalize_map_id("");
  }

  bool normalize_waypoint_speed_properties(
    const rapidjson::Value & waypoint_data,
    std::string & normalized_properties,
    std::string & error_message) const
  {
    // 归一化点位速度属性：支持 speed、target_speed、navigation_speed 三种入口字段。
    // 如果 APP 传入非法速度值，返回错误信息并拒绝写入，避免生成不可执行路线。
    // 保持三个别名兼容：speed / target_speed / navigation_speed，并统一补写 properties.speed。
    rapidjson::Document props_doc;
    props_doc.SetObject();
    auto & allocator = props_doc.GetAllocator();

    if (waypoint_data.HasMember("properties")) {
      if (!waypoint_data["properties"].IsObject()) {
        error_message = "properties 必须是对象";
        return false;
      }
      props_doc.CopyFrom(waypoint_data["properties"], allocator);
    }

    for (const auto * key : {
        "waypoint_role",
        "need_broadcast",
        "broadcast_id",
        "broadcast_text",
        "broadcast_blocking",
        "stop_and_align",
        "walk_direction",
        "speed",
        "target_speed",
        "navigation_speed",
      })
    {
      if (waypoint_data.HasMember(key) && !props_doc.HasMember(key)) {
        props_doc.AddMember(
          rapidjson::Value(key, allocator).Move(),
          rapidjson::Value(waypoint_data[key], allocator),
          allocator);
      }
    }

    const char * speed_keys[] = {"speed", "target_speed", "navigation_speed"};
    const rapidjson::Value * raw_speed = nullptr;
    for (const auto * key : speed_keys) {
      if (props_doc.HasMember(key)) {
        raw_speed = &props_doc[key];
        break;
      }
    }

    if (raw_speed != nullptr) {
      if (raw_speed->IsBool() || !raw_speed->IsNumber()) {
        error_message = "路点速度 speed 必须是数字，单位 m/s";
        return false;
      }
      const double speed = raw_speed->GetDouble();
      if (speed < 0.15 || speed > 1.0) {
        error_message = "路点速度 speed 必须在 0.15~1.00 m/s 范围内";
        return false;
      }
      if (props_doc.HasMember("speed")) {
        props_doc["speed"].SetDouble(speed);
      } else {
        props_doc.AddMember("speed", speed, allocator);
      }
    }

    normalized_properties = json_document_to_string(props_doc, false);
    return true;
  }

  std::string merge_properties_json(
    const std::string & existing_json,
    const std::string & incoming_json) const
  {
    // 更新 properties 时只覆盖传入字段，不清空旧字段。
    // 这样 APP 可以单独修改某个扩展属性，而不会误删播报、速度等其他配置。
    rapidjson::Document merged;
    merged.SetObject();
    auto & allocator = merged.GetAllocator();

    rapidjson::Document existing;
    if (!existing_json.empty() && !existing.Parse(existing_json.c_str()).HasParseError() && existing.IsObject()) {
      merged.CopyFrom(existing, allocator);
    }

    rapidjson::Document incoming;
    if (!incoming_json.empty() && !incoming.Parse(incoming_json.c_str()).HasParseError() && incoming.IsObject()) {
      for (auto it = incoming.MemberBegin(); it != incoming.MemberEnd(); ++it) {
        rapidjson::Value name;
        name.CopyFrom(it->name, allocator);
        rapidjson::Value value;
        value.CopyFrom(it->value, allocator);
        if (merged.HasMember(name.GetString())) {
          merged[name.GetString()] = value;
        } else {
          merged.AddMember(name, value, allocator);
        }
      }
    }

    return json_document_to_string(merged, false);
  }

  std::string expand_user_path(const std::string & path) const
  {
    if (path.rfind("~/", 0) != 0) {
      return path;
    }
    const char * home = std::getenv("HOME");
    if (home == nullptr) {
      return path;
    }
    return std::string(home) + path.substr(1);
  }

  // ===========================================================================
  // 9. 成员变量
  // ===========================================================================

  DynamicWaypointsConfig config_;
  std::string current_map_id_{"hall"};
  WaypointsByMap waypoints_by_map_;
  std::map<std::string, std::string> waypoints_revisions_by_map_;
  std::string active_waypoint_request_message_id_;
  int initial_publish_count_{0};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoints_data_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr app_waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr app_navigation_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_ack_sub_;
  rclcpp::TimerBase::SharedPtr initial_publish_timer_;
};

}  // namespace humanoid_control_runtime

// =============================================================================
// 10. main 入口
// =============================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_control_runtime::DynamicWaypointsManagerNode>());
  rclcpp::shutdown();
  return 0;
}
