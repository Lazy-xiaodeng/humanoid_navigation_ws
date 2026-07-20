/*
 * data_integration_node.cpp
 *
 * 文件用途：
 * 1. 数据整合 C++ 节点，负责把 ROS 业务 topic 转换为 integration 系列消息。
 * 2. 已接入机器人状态、导航、地图点位、定位速度、动作结果、动作/表情库和定位恢复异常链路。
 * 3. 真实接管由 data_integration_enable 控制，便于先验证转换逻辑，再接入正式数据链路。
 * 4. 上游：robot_gateway_node、navigation_state_manager、map_context_manager、定位/odom/plan 等系统话题。
 * 5. 下游：/integration/data_responses、/integration/push_messages、/integration/subscription_responses。
 *
 * 代码块顺序：
 * 1. 匿名工具函数：JSON 安全解析、字段类型转换、字符串清洗和 YAML 简易读取辅助。
 * 2. 节点初始化：加载参数、配置缓存 TTL、创建 integration publisher 和业务 subscriber。
 * 3. APP 请求链路：处理 /websocket/data_requests 并从 DataStore 生成 response。
 * 4. APP 订阅链路：处理 /websocket/data_subscriptions，维护订阅表并周期推送新鲜数据。
 * 5. 业务数据输入：机器人状态、导航状态、地图点位、动作结果、定位、速度、路径和 IMU。
 * 6. 异常与事件：导航失败、定位恢复、动作失败等统一转换成 system_exception push。
 * 7. 通用发布出口：response、push、subscription_response 的最终组包和 publisher。
 */

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "humanoid_app_gateway_runtime/app_gateway_config.hpp"
#include "humanoid_app_gateway_runtime/app_gateway_types.hpp"
#include "humanoid_app_gateway_runtime/data_store.hpp"
#include "humanoid_app_gateway_runtime/map_waypoint_adapter.hpp"
#include "humanoid_app_gateway_runtime/navigation_status_adapter.hpp"
#include "humanoid_app_gateway_runtime/path_metrics.hpp"
#include "humanoid_app_gateway_runtime/pose_speed_adapter.hpp"
#include "humanoid_app_gateway_runtime/protocol_builder.hpp"
#include "humanoid_app_gateway_runtime/robot_status_adapter.hpp"
#include "humanoid_app_gateway_runtime/subscription_manager.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

namespace humanoid_app_gateway_runtime
{

namespace
{

const rapidjson::Value * find_member(const rapidjson::Value & object, const char * key)
{
  if (!object.IsObject()) {
    return nullptr;
  }
  const auto it = object.FindMember(key);
  return it == object.MemberEnd() ? nullptr : &it->value;
}

std::string value_to_string(const rapidjson::Value * value, const std::string & fallback = "")
{
  if (value == nullptr || value->IsNull()) {
    return fallback;
  }
  if (value->IsString()) {
    return std::string(value->GetString(), value->GetStringLength());
  }
  if (value->IsBool()) {
    return value->GetBool() ? "True" : "False";
  }
  if (value->IsInt64()) {
    return std::to_string(value->GetInt64());
  }
  if (value->IsUint64()) {
    return std::to_string(value->GetUint64());
  }
  if (value->IsDouble()) {
    return std::to_string(value->GetDouble());
  }
  return fallback;
}

std::string document_to_json(const rapidjson::Value & value)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  value.Accept(writer);
  return buffer.GetString();
}

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

void set_string(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const std::string & value)
{
  if (!object.IsObject()) {
    return;
  }
  const auto it = object.FindMember(key);
  if (it == object.MemberEnd()) {
    add_string(object, allocator, key, value);
    return;
  }
  it->value.SetString(value.c_str(), allocator);
}

void add_speed_payload(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const SpeedPayload & speed)
{
  object.AddMember("linear_x", speed.linear_x, allocator);
  object.AddMember("linear_y", speed.linear_y, allocator);
  object.AddMember("angular_z", speed.angular_z, allocator);
  rapidjson::Value linear(rapidjson::kObjectType);
  linear.AddMember("x", speed.linear_x, allocator);
  linear.AddMember("y", speed.linear_y, allocator);
  linear.AddMember("z", speed.linear_z, allocator);
  object.AddMember("linear", linear, allocator);
  rapidjson::Value angular(rapidjson::kObjectType);
  angular.AddMember("x", speed.angular_x, allocator);
  angular.AddMember("y", speed.angular_y, allocator);
  angular.AddMember("z", speed.angular_z, allocator);
  object.AddMember("angular", angular, allocator);
  object.AddMember("speed_mps", speed.speed_mps, allocator);
  object.AddMember("turn_rate_radps", speed.turn_rate_radps, allocator);
  object.AddMember("is_moving", speed.is_moving, allocator);
  add_string(object, allocator, "coordinate_frame", speed.coordinate_frame);
  add_string(object, allocator, "source_topic", speed.source_topic);
  add_string(object, allocator, "source", speed.source);
  add_string(object, allocator, "source_coordinate_frame", speed.source_coordinate_frame);
  object.AddMember("timestamp", speed.timestamp, allocator);
  object.AddMember("valid", speed.valid, allocator);
  if (!speed.reject_reason.empty()) {
    add_string(object, allocator, "reject_reason", speed.reject_reason);
  }
}

std::vector<std::string> string_array_from_json(const rapidjson::Value * value)
{
  std::vector<std::string> items;
  if (value == nullptr || !value->IsArray()) {
    return items;
  }
  for (const auto & item : value->GetArray()) {
    items.push_back(value_to_string(&item));
  }
  return items;
}

double value_to_double(const rapidjson::Value * value, const double fallback)
{
  if (value == nullptr || value->IsNull()) {
    return fallback;
  }
  if (value->IsNumber()) {
    return value->GetDouble();
  }
  if (value->IsString()) {
    try {
      return std::stod(value->GetString());
    } catch (...) {
      return fallback;
    }
  }
  return fallback;
}

int value_to_int(const rapidjson::Value * value, const int fallback)
{
  if (value == nullptr || value->IsNull()) {
    return fallback;
  }
  if (value->IsInt()) {
    return value->GetInt();
  }
  if (value->IsUint()) {
    return static_cast<int>(value->GetUint());
  }
  if (value->IsNumber()) {
    return static_cast<int>(value->GetDouble());
  }
  if (value->IsString()) {
    try {
      return std::stoi(value->GetString());
    } catch (...) {
      return fallback;
    }
  }
  return fallback;
}

bool value_to_bool(const rapidjson::Value * value, const bool fallback)
{
  if (value == nullptr || value->IsNull()) {
    return fallback;
  }
  if (value->IsBool()) {
    return value->GetBool();
  }
  if (value->IsNumber()) {
    return value->GetDouble() != 0.0;
  }
  if (value->IsString()) {
    std::string text(value->GetString(), value->GetStringLength());
    for (auto & c : text) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return text == "true" || text == "1" || text == "yes" || text == "on";
  }
  return fallback;
}

bool parse_object(const std::string & json, rapidjson::Document & document)
{
  document.Parse(json.c_str());
  return !document.HasParseError() && document.IsObject();
}

bool truthy_json_value(const rapidjson::Value * value)
{
  if (value == nullptr || value->IsNull()) {
    return false;
  }
  if (value->IsBool()) {
    return value->GetBool();
  }
  if (value->IsNumber()) {
    return value->GetDouble() != 0.0;
  }
  if (value->IsString()) {
    std::string text(value->GetString(), value->GetStringLength());
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    return text == "1" || text == "true" || text == "yes" || text == "on" ||
           text == "busy" || text == "moving";
  }
  return false;
}

std::string trim_copy(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::string strip_quotes(const std::string & value)
{
  std::string result = trim_copy(value);
  if (!result.empty() && result.front() == '\'' && result.back() == '\'') {
    result = result.substr(1, result.size() - 2);
  } else if (!result.empty() && result.front() == '"' && result.back() == '"') {
    result = result.substr(1, result.size() - 2);
  }
  return result;
}

bool starts_with(const std::string & text, const std::string & prefix)
{
  return text.rfind(prefix, 0) == 0;
}

int leading_spaces(const std::string & text)
{
  int count = 0;
  for (const char ch : text) {
    if (ch != ' ') {
      break;
    }
    ++count;
  }
  return count;
}

}  // namespace

class DataIntegrationNode : public rclcpp::Node
{
public:
  explicit DataIntegrationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("data_integration_node_cpp", options)
  {
    declare_data_integration_parameters(*this, config_);
    config_ = load_data_integration_config(*this, config_);
    configure_data_store_ttl();

    // integration 三个 publisher 是 APP 数据出口：
    // data_responses 对应 APP 主动 request，push_messages 对应状态变化/订阅推送，
    // subscription_responses 对应订阅/退订结果。
    data_response_pub_ = create_publisher<std_msgs::msg::String>(
      config_.integration_data_responses_topic, 10);
    push_message_pub_ = create_publisher<std_msgs::msg::String>(
      config_.integration_push_messages_topic, 10);
    subscription_response_pub_ = create_publisher<std_msgs::msg::String>(
      config_.integration_subscription_responses_topic, 10);

    if (config_.data_integration_enable) {
      // 以下 subscriber 按数据流分组接入：
      // robot_status_raw 提供系统健康与电量状态；
      // data_requests/data_subscriptions 来自 APP 网关；
      // navigation/map/waypoints/action_result 负责导航页和任务事件；
      // pose/odom/imu/path 负责实时状态和路径指标。
      robot_status_raw_sub_ = create_subscription<std_msgs::msg::String>(
        config_.robot_status_raw_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          robot_status_raw_callback(*msg);
        });
      data_request_sub_ = create_subscription<std_msgs::msg::String>(
        config_.data_requests_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          data_request_callback(*msg);
        });
      subscription_sub_ = create_subscription<std_msgs::msg::String>(
        config_.data_subscriptions_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          subscription_callback(*msg);
        });
      navigation_status_sub_ = create_subscription<std_msgs::msg::String>(
        config_.navigation_status_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          navigation_status_callback(*msg);
        });
      navigation_ack_sub_ = create_subscription<std_msgs::msg::String>(
        config_.navigation_acknowledgments_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          navigation_ack_callback(*msg);
        });
      map_response_sub_ = create_subscription<std_msgs::msg::String>(
        config_.map_response_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          map_message_callback(*msg);
        });
      map_status_sub_ = create_subscription<std_msgs::msg::String>(
        config_.map_status_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          map_message_callback(*msg);
        });
      waypoints_data_sub_ = create_subscription<std_msgs::msg::String>(
        config_.waypoints_data_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          waypoints_data_callback(*msg);
        });
      action_result_sub_ = create_subscription<std_msgs::msg::String>(
        config_.robot_action_result_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          action_result_callback(*msg);
        });
      localization_recovery_status_sub_ = create_subscription<std_msgs::msg::String>(
        config_.localization_recovery_status_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          localization_recovery_status_callback(*msg);
        });
      gesture_update_sub_ = create_subscription<std_msgs::msg::String>(
        config_.gesture_list_updated_topic,
        10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          gesture_update_callback(*msg);
        });
      navigation_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        config_.navigation_path_topic,
        10,
        [this](const nav_msgs::msg::Path::SharedPtr msg) {
          path_callback(*msg);
        });
      robot_realpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        config_.robot_realpose_topic,
        10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          real_pose_callback(*msg);
        });
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic,
        10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_callback(*msg);
        });
      if (config_.enable_imu_input) {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          config_.imu_topic,
          10,
          [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imu_callback(*msg);
          });
      }
      cleanup_timer_ = create_wall_timer(
        std::chrono::duration<double>(config_.cleanup_interval_sec),
        [this]() { cleanup_expired_data(); });
      push_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / config_.push_update_rate_hz),
        [this]() { push_data_updates(); });

      // 动作/表情列表属于导航页初始回显数据，启动时先从 YAML 加载到缓存；
      // 机器人动作库热重载时会通过 gesture_update_callback() 再次刷新。
      load_static_business_lists();
      initial_sync_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        [this]() {
          trigger_initial_sync();
          initial_sync_timer_->cancel();
        });
    }

    RCLCPP_WARN(
      get_logger(),
      "data_integration_node_cpp 已启动：data_integration_enable=%s，enable_imu_input=%s，已接入 system/nav/map/waypoints/action_result 链路。",
      config_.data_integration_enable ? "true" : "false",
      config_.enable_imu_input ? "true" : "false");
  }

private:
  void configure_data_store_ttl()
  {
    // DataStore 只存 JSON 和更新时间，TTL 决定 APP 查询/订阅时数据是否仍然可信。
    // 不同业务数据实时性不同：速度和传感器很短，点位/动作库较长。
    data_store_.set_expiry("robot_pose", config_.robot_pose_ttl_sec);
    data_store_.set_expiry("robot_speed", config_.robot_speed_ttl_sec);
    data_store_.set_expiry("odom_raw", config_.odom_raw_ttl_sec);
    data_store_.set_expiry("navigation_status", config_.navigation_status_ttl_sec);
    data_store_.set_expiry("navigation_path", config_.navigation_path_ttl_sec);
    data_store_.set_expiry("system_status", config_.system_status_ttl_sec);
    data_store_.set_expiry("action_result", config_.action_result_ttl_sec);
    data_store_.set_expiry("map_status", config_.map_status_ttl_sec);
    data_store_.set_expiry("map_response", config_.map_response_ttl_sec);
    data_store_.set_expiry("waypoints_data", config_.waypoints_data_ttl_sec);
    data_store_.set_expiry("system_exception", config_.system_exception_ttl_sec);
    data_store_.set_expiry("sensor_data", config_.sensor_data_ttl_sec);
    data_store_.set_expiry("imu", config_.sensor_data_ttl_sec);
    data_store_.set_expiry("gesture_list", config_.waypoints_data_ttl_sec);
    data_store_.set_expiry("facial_gesture_list", config_.waypoints_data_ttl_sec);
  }

  void robot_status_raw_callback(const std_msgs::msg::String & msg)
  {
    // 机器人本体原始状态先转换成 APP 需要的 system_status，再写入缓存并立即推送。
    // 这里是机器人在线、Walk/Menu 状态、电量、通信质量等字段进入前端的主入口。
    const double timestamp = now_sec();
    const auto converted = robot_status_adapter_.convert_raw_to_system_status(msg.data, timestamp);
    if (!converted.ok) {
      RCLCPP_WARN(get_logger(), "robot_status_raw 转换失败: %s", converted.error.c_str());
      return;
    }

    data_store_.set_data("system_status", converted.system_status_json, timestamp);
    publish_push("system_status", converted.system_status_json, "all", timestamp, timestamp, 0);
  }

  void data_request_callback(const std_msgs::msg::String & msg)
  {
    // APP 主动查询入口：取 client_id/request_id/data_type 后，从 DataStore 读取最近缓存。
    // 查询不主动拉上游数据；如果缓存不存在，就返回 data_not_available。
    rapidjson::Document request;
    request.Parse(msg.data.c_str());
    if (request.HasParseError() || !request.IsObject()) {
      publish_error("unknown", "", "unknown", "invalid_request_json", "请求 JSON 格式错误");
      return;
    }

    const std::string client_id = value_to_string(find_member(request, "source"), "unknown");
    const std::string request_id = value_to_string(find_member(request, "message_id"));
    const std::string data_type = value_to_string(find_member(request, "data_type"));
    if (data_type.empty()) {
      publish_error(client_id, request_id, "unknown", "missing_data_type", "请求缺少 data_type");
      return;
    }

    const auto data = data_store_.get_data(data_type);
    if (!data.has_value()) {
      publish_error(client_id, request_id, data_type, "data_not_available", "请求的数据当前不可用");
      return;
    }

    const double timestamp = now_sec();
    const auto * request_params = find_member(request, "data");
    const std::string response_data_json = prepare_data_response_json(data_type, data->json, request_params);
    const auto response = protocol_builder_.create_data_response(
      client_id,
      request_id,
      data_type,
      response_data_json,
      timestamp,
      data->updated_at_sec);
    if (!response.ok) {
      publish_error(client_id, request_id, data_type, response.error, "数据响应组包失败");
      return;
    }

    std_msgs::msg::String out;
    out.data = response.json;
    data_response_pub_->publish(out);
  }

  void subscription_callback(const std_msgs::msg::String & msg)
  {
    // APP 订阅入口：维护 data_type -> client_id 的推送关系。
    // 订阅频率只影响主动 push 节流，不改变上游 topic 接收频率。
    rapidjson::Document request;
    if (!parse_object(msg.data, request)) {
      publish_subscription_response(msg.data, false, "处理失败: 请求 JSON 格式错误");
      return;
    }

    const std::string client_id = value_to_string(find_member(request, "source"), "unknown");
    const auto * data = find_member(request, "data");
    const std::string action = data != nullptr && data->IsObject() ?
      value_to_string(find_member(*data, "action"), "subscribe") : "subscribe";
    const std::vector<std::string> data_types = data != nullptr && data->IsObject() ?
      string_array_from_json(find_member(*data, "data_types")) : std::vector<std::string>{};
    const double frequency = data != nullptr && data->IsObject() ?
      value_to_double(find_member(*data, "push_frequency"), 1.0) : 1.0;

    bool success = false;
    std::string message;
    if (action == "subscribe") {
      success = subscription_manager_.subscribe(client_id, data_types, frequency, msg.data, now_sec());
      message = success ? "订阅成功" : "订阅失败";
    } else {
      success = subscription_manager_.unsubscribe(client_id, data_types);
      message = success ? "取消订阅成功" : "取消订阅失败";
    }
    publish_subscription_response(msg.data, success, message);
  }

  void navigation_status_callback(const std_msgs::msg::String & msg)
  {
    // 导航状态既要缓存给 APP 查询，也要在关键事件发生时立即 push。
    // 普通周期状态不每帧强推，避免导航页和 WebSocket 被高频状态刷屏。
    rapidjson::Document status;
    if (!parse_object(msg.data, status)) {
      RCLCPP_WARN(get_logger(), "navigation_status JSON 解析失败");
      return;
    }

    const double timestamp = now_sec();
    data_store_.set_data("navigation_status", msg.data, timestamp);
    const std::string event_type = value_to_string(find_member(status, "event_type"));
    if (navigation_status_adapter_.should_push_navigation_status_immediately(event_type)) {
      publish_push("navigation_status", msg.data, "all", timestamp, timestamp, 0);
    }
    maybe_publish_navigation_exception(status, event_type);
  }

  void navigation_ack_callback(const std_msgs::msg::String & msg)
  {
    // 导航 ack 是“命令执行结果/任务事件”的主要来源。
    // Adapter 负责整理字段，本函数负责推送和必要的 system_exception 事件补发。
    const double timestamp = now_sec();
    const auto built = navigation_status_adapter_.build_navigation_command_result_data(msg.data, timestamp);
    if (!built.ok) {
      RCLCPP_WARN(get_logger(), "navigation ack 转换失败: %s", built.error.c_str());
      return;
    }
    data_store_.set_data("navigation_status", built.json, timestamp);
    publish_navigation_ack_push(built.json, msg.data, timestamp);
  }

  void map_message_callback(const std_msgs::msg::String & msg)
  {
    // 地图状态和地图命令响应统一走 adapter，输出会保留 data_type/metadata，
    // 方便 APP 网关按 data_type 做初始快照和主动推送。
    const auto built = map_waypoint_adapter_.build_map_message_payload(msg.data);
    if (!built.ok) {
      RCLCPP_WARN(get_logger(), "map 消息转换失败: %s", built.error.c_str());
      return;
    }
    cache_and_publish_adapter_payload(built.json);
  }

  void waypoints_data_callback(const std_msgs::msg::String & msg)
  {
    // 点位全量数据是导航页路线/点位列表的权威来源。
    // revision、map_id、update_type 等字段用于页面判断是否需要刷新。
    const auto built = map_waypoint_adapter_.build_waypoints_data_payload(msg.data);
    if (!built.ok) {
      RCLCPP_WARN(get_logger(), "waypoints_data 转换失败: %s", built.error.c_str());
      return;
    }
    cache_and_publish_adapter_payload(built.json);
  }

  void action_result_callback(const std_msgs::msg::String & msg)
  {
    // 机器人动作结果既要更新最近动作状态，也要作为事件推送给 APP。
    // 失败、超时、拒绝等结果会额外转成 system_exception，便于事件日志和异常提示显示。
    rapidjson::Document action_result;
    if (!parse_object(msg.data, action_result)) {
      RCLCPP_WARN(get_logger(), "action_result JSON 解析失败");
      return;
    }
    const double timestamp = now_sec();
    data_store_.set_data("action_result", msg.data, timestamp);
    publish_action_result_push(action_result, msg.data, timestamp);
    const std::string status = value_to_string(find_member(action_result, "status"));
    if (!status.empty() && status != "success") {
      publish_system_exception(
        "action",
        "error",
        "动作异常",
        value_to_string(find_member(action_result, "message"), "动作执行失败"),
        value_to_string(find_member(action_result, "result"), "action_failed"),
        "action_result",
        action_result);
    }
  }

  void gesture_update_callback(const std_msgs::msg::String & msg)
  {
    // 动作库热重载通知：机器人侧同步 gestures.yaml 后会发布本消息。
    // 数据整合节点收到后重新读取配置，并把更新后的列表推送给 APP。
    rapidjson::Document update;
    parse_object(msg.data, update);
    RCLCPP_INFO(get_logger(), "收到动作库更新通知，重新加载 gesture_list");
    if (load_gesture_list(true)) {
      RCLCPP_INFO(get_logger(), "动作库热重载完成");
    }
  }

  void localization_recovery_status_callback(const std_msgs::msg::String & msg)
  {
    // 定位恢复状态只在恢复节点主动上报时进入这里。
    // 该链路用于 APP 弹出“定位恢复/异常”类提示，不直接驱动导航状态机。
    rapidjson::Document status;
    if (!parse_object(msg.data, status)) {
      RCLCPP_WARN(get_logger(), "localization_recovery_status JSON 解析失败");
      return;
    }

    const std::string event_type = value_to_string(find_member(status, "event_type"));
    const std::string reason = value_to_string(find_member(status, "reason"), event_type);
    const std::string result_code = value_to_string(find_member(status, "result_code"));
    std::string title;
    std::string severity = "info";
    bool popup = true;

    if (event_type == "localization_recovery_started") {
      title = "定位异常，正在重定位";
      severity = "warning";
    } else if (event_type == "localization_relocalize_requested") {
      title = "定位重定位请求已发出";
      severity = "info";
      popup = false;
    } else if (event_type == "localization_relocalize_attempt_deferred") {
      title = "定位重定位暂未接受";
      severity = (
        result_code == "globalmap_not_ready" ||
        result_code == "no_scan" ||
        result_code == "not_enough_accumulated_scans" ||
        result_code == "held_for_consistency") ? "info" : "warning";
    } else if (event_type == "localization_relocalize_accepted") {
      title = "定位重定位结果已接受";
    } else if (event_type == "localization_initialpose_published") {
      title = "定位初始位姿已更新";
    } else if (event_type == "localization_relocalize_failed") {
      title = "定位重定位失败";
      severity = "error";
    } else if (event_type == "localization_recovered") {
      title = "定位已恢复";
    } else if (event_type == "localization_manual_initialpose_override") {
      title = "定位已手动校正";
    } else {
      return;
    }

    publish_system_exception(
      "localization",
      severity,
      title,
      reason,
      result_code.empty() ? event_type : result_code,
      event_type,
      status,
      popup,
      event_type == "localization_relocalize_attempt_deferred" ? 2.0 : 0.5);
  }

  void load_static_business_lists()
  {
    // 静态业务列表包括机器人动作库和表情动作库。
    // 它们低频变化，适合启动时读取 YAML 后缓存到 DataStore。
    load_gesture_list(false);
    load_facial_gesture_list(false);
  }

  std::string resolve_expression_config_path(
    const std::string & configured_path,
    const std::string & file_name) const
  {
    if (!configured_path.empty()) {
      return configured_path;
    }
    try {
      return ament_index_cpp::get_package_share_directory("humanoid_expression_runtime") +
             "/config/" + file_name;
    } catch (const std::exception &) {
      return "src/humanoid_expression_runtime/config/" + file_name;
    }
  }

  bool load_gesture_list(const bool publish_after_load)
  {
    // 读取机器人 OTA gestures.yaml 并构造 gesture_list 数据。
    // publish_after_load=true 时说明来自热重载，需要立即通知 APP 刷新动作列表。
    const std::string path = resolve_expression_config_path(
      config_.gesture_list_yaml_path,
      "gestures.yaml");
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_WARN(get_logger(), "无法打开动作库 YAML: %s", path.c_str());
      return false;
    }

    rapidjson::Document document;
    document.SetObject();
    auto & allocator = document.GetAllocator();
    document.AddMember("timestamp", now_sec(), allocator);
    rapidjson::Value gestures(rapidjson::kArrayType);

    bool in_actions = false;
    std::string current_key;
    std::string current_name;
    std::string current_type;
    auto flush_current = [&]() {
        if (current_key.empty()) {
          return;
        }
        rapidjson::Value item(rapidjson::kObjectType);
        add_string(item, allocator, "id", current_key);
        add_string(item, allocator, "name", current_name.empty() ? current_key : current_name);
        add_string(item, allocator, "type", current_type.empty() ? "upper_body" : current_type);
        gestures.PushBack(item, allocator);
        current_key.clear();
        current_name.clear();
        current_type.clear();
      };

    std::string line;
    while (std::getline(file, line)) {
      const std::string stripped = trim_copy(line);
      if (stripped.empty() || starts_with(stripped, "#")) {
        continue;
      }
      if (stripped == "actions:") {
        in_actions = true;
        continue;
      }
      if (!in_actions) {
        continue;
      }
      const int indent = leading_spaces(line);
      if (indent == 2 && stripped.back() == ':') {
        flush_current();
        current_key = stripped.substr(0, stripped.size() - 1);
      } else if (indent >= 4 && !current_key.empty()) {
        const auto colon = stripped.find(':');
        if (colon == std::string::npos) {
          continue;
        }
        const std::string key = stripped.substr(0, colon);
        std::string value = strip_quotes(stripped.substr(colon + 1));
        const auto comment_pos = value.find(" #");
        if (comment_pos != std::string::npos) {
          value = trim_copy(value.substr(0, comment_pos));
        }
        if (key == "name") {
          current_name = value;
        } else if (key == "type") {
          current_type = value;
        }
      }
    }
    flush_current();

    document.AddMember("gestures", gestures, allocator);
    const double timestamp = now_sec();
    const std::string json = document_to_json(document);
    data_store_.set_data("gesture_list", json, timestamp);
    if (publish_after_load) {
      publish_push("gesture_list", json, "all", timestamp, timestamp, 0);
    }
    return true;
  }

  bool load_facial_gesture_list(const bool publish_after_load)
  {
    // 读取表情动作配置并构造 facial_gesture_list 数据。
    // 该数据只用于 APP 展示和下发表情命令，不参与导航控制。
    const std::string path = resolve_expression_config_path(
      config_.facial_gesture_list_yaml_path,
      "facial_gestures.yaml");
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_WARN(get_logger(), "无法打开表情库 YAML: %s", path.c_str());
      return false;
    }

    const std::unordered_map<std::string, std::string> name_map{
      {"idle", "待机"},
      {"surprised", "惊讶"},
      {"thinking", "思考"},
      {"speak_start", "说话开始"},
      {"speak_stop", "说话停止"},
      {"sleeping", "休眠"}};

    rapidjson::Document document;
    document.SetObject();
    auto & allocator = document.GetAllocator();
    document.AddMember("timestamp", now_sec(), allocator);
    rapidjson::Value gestures(rapidjson::kArrayType);
    rapidjson::Value metadata(rapidjson::kObjectType);

    bool in_metadata = false;
    bool in_facial = false;
    std::string line;
    while (std::getline(file, line)) {
      const std::string stripped = trim_copy(line);
      if (stripped.empty() || starts_with(stripped, "#")) {
        continue;
      }
      if (stripped == "metadata:") {
        in_metadata = true;
        in_facial = false;
        continue;
      }
      if (stripped == "facial_gestures:") {
        in_metadata = false;
        in_facial = true;
        continue;
      }
      const int indent = leading_spaces(line);
      if (in_metadata && indent == 2) {
        const auto colon = stripped.find(':');
        if (colon != std::string::npos) {
          add_string(
            metadata,
            allocator,
            stripped.substr(0, colon).c_str(),
            strip_quotes(stripped.substr(colon + 1)));
        }
      } else if (in_facial && indent == 2 && stripped.back() == ':') {
        const std::string action_id = stripped.substr(0, stripped.size() - 1);
        rapidjson::Value item(rapidjson::kObjectType);
        add_string(item, allocator, "id", action_id);
        const auto it = name_map.find(action_id);
        add_string(item, allocator, "name", it == name_map.end() ? action_id : it->second);
        gestures.PushBack(item, allocator);
      }
    }

    document.AddMember("gestures", gestures, allocator);
    document.AddMember("metadata", metadata, allocator);
    const double timestamp = now_sec();
    const std::string json = document_to_json(document);
    data_store_.set_data("facial_gesture_list", json, timestamp);
    if (publish_after_load) {
      publish_push("facial_gesture_list", json, "all", timestamp, timestamp, 0);
    }
    return true;
  }

  void trigger_initial_sync()
  {
    // 节点启动后延迟补发静态列表，给 APP 网关和订阅关系一点初始化时间。
    const double timestamp = now_sec();
    for (const char * data_type : {"gesture_list", "facial_gesture_list"}) {
      const auto stored = data_store_.get_data(data_type);
      if (stored.has_value()) {
        publish_push(data_type, stored->json, "all", timestamp, stored->updated_at_sec, 0);
      }
    }
  }

  void publish_system_exception(
    const std::string & category,
    const std::string & severity,
    const std::string & title,
    const std::string & message,
    const std::string & code,
    const std::string & source_event,
    const rapidjson::Value & details,
    const bool popup = true,
    const double dedupe_sec = 1.0)
  {
    // system_exception 是异常事件的统一结构。
    // dedupe_key 用于短时间内去重，避免同一个定位/导航异常疯狂刷屏。
    const double timestamp = now_sec();
    const std::string dedupe_key = category + ":" + code + ":" + source_event + ":" + message;
    const auto last_it = last_exception_push_times_.find(dedupe_key);
    if (dedupe_sec > 0.0 && last_it != last_exception_push_times_.end() &&
      timestamp - last_it->second < dedupe_sec)
    {
      return;
    }
    last_exception_push_times_[dedupe_key] = timestamp;

    rapidjson::Document document;
    document.SetObject();
    auto & allocator = document.GetAllocator();
    add_string(document, allocator, "exception_id", "exception_" + std::to_string(static_cast<long long>(timestamp * 1000.0)));
    add_string(document, allocator, "category", category);
    add_string(document, allocator, "severity", severity);
    add_string(document, allocator, "title", title);
    add_string(document, allocator, "message", message);
    add_string(document, allocator, "code", code);
    add_string(document, allocator, "source_event", source_event);
    document.AddMember("details", rapidjson::Value(details, allocator), allocator);
    rapidjson::Value display(rapidjson::kObjectType);
    display.AddMember("popup", popup, allocator);
    display.AddMember("modal", severity == "error" || severity == "critical", allocator);
    display.AddMember("auto_close", severity == "info" || severity == "warning", allocator);
    document.AddMember("display", display, allocator);
    document.AddMember("timestamp", timestamp, allocator);

    const std::string json = document_to_json(document);
    data_store_.set_data("system_exception", json, timestamp);
    publish_system_exception_push(json, severity, code, message, timestamp);
  }

  void path_callback(const nav_msgs::msg::Path & msg)
  {
    // 全局路径进入后计算路径长度、预计耗时、平滑度和复杂度。
    // 这些字段用于 APP 展示和诊断，不反向修改 Nav2 路径。
    rapidjson::Document document;
    document.SetObject();
    auto & allocator = document.GetAllocator();
    const double timestamp = now_sec();

    add_string(document, allocator, "path_id", "path_" + std::to_string(static_cast<long long>(timestamp)));
    add_string(document, allocator, "frame_id", msg.header.frame_id);
    document.AddMember("timestamp", timestamp, allocator);
    document.AddMember("waypoint_count", static_cast<int>(msg.poses.size()), allocator);

    rapidjson::Value path_poses(rapidjson::kArrayType);
    std::vector<PathPoint2D> points;
    double total_length = 0.0;
    bool has_previous = false;
    double prev_x = 0.0;
    double prev_y = 0.0;
    for (std::size_t i = 0; i < msg.poses.size(); ++i) {
      const auto & pose = msg.poses[i].pose;
      const double x = pose.position.x;
      const double y = pose.position.y;
      double segment_length = 0.0;
      if (has_previous) {
        const double dx = x - prev_x;
        const double dy = y - prev_y;
        segment_length = std::sqrt(dx * dx + dy * dy);
      }
      total_length += segment_length;
      prev_x = x;
      prev_y = y;
      has_previous = true;
      points.push_back(PathPoint2D{x, y});

      rapidjson::Value item(rapidjson::kObjectType);
      item.AddMember("sequence", static_cast<int>(i), allocator);
      rapidjson::Value position(rapidjson::kObjectType);
      position.AddMember("x", x, allocator);
      position.AddMember("y", y, allocator);
      position.AddMember("z", pose.position.z, allocator);
      item.AddMember("position", position, allocator);
      rapidjson::Value orientation(rapidjson::kObjectType);
      orientation.AddMember("x", pose.orientation.x, allocator);
      orientation.AddMember("y", pose.orientation.y, allocator);
      orientation.AddMember("z", pose.orientation.z, allocator);
      orientation.AddMember("w", pose.orientation.w, allocator);
      item.AddMember("orientation", orientation, allocator);
      item.AddMember("segment_length", segment_length, allocator);
      item.AddMember("cumulative_length", total_length, allocator);
      path_poses.PushBack(item, allocator);
    }

    const auto analysis = path_metrics_.analyze_path_properties(points);
    document.AddMember("total_length", total_length, allocator);
    document.AddMember("estimated_duration", path_metrics_.estimate_path_duration(total_length), allocator);
    document.AddMember("path_poses", path_poses, allocator);
    rapidjson::Value properties(rapidjson::kObjectType);
    add_string(properties, allocator, "smoothness", analysis.smoothness);
    add_string(properties, allocator, "safety_level", analysis.safety_level);
    add_string(properties, allocator, "complexity", analysis.complexity);
    properties.AddMember("turn_count", analysis.turn_count, allocator);
    properties.AddMember("total_distance", analysis.total_distance, allocator);
    properties.AddMember("avg_segment_length", analysis.avg_segment_length, allocator);
    properties.AddMember("max_angle_change_deg", analysis.max_angle_change_deg, allocator);
    properties.AddMember("segment_count", analysis.segment_count, allocator);
    document.AddMember("path_properties", properties, allocator);

    data_store_.set_data("navigation_path", document_to_json(document), timestamp);
  }

  void real_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    // 定位位姿主入口：把 pose、yaw、协方差质量和定位置信度转换成 robot_pose。
    // speed 字段会结合前后位姿估计，保证导航页实时状态显示更完整。
    rapidjson::Document document;
    document.SetObject();
    auto & allocator = document.GetAllocator();
    const double timestamp = now_sec();

    rapidjson::Value position(rapidjson::kObjectType);
    position.AddMember("x", msg.pose.pose.position.x, allocator);
    position.AddMember("y", msg.pose.pose.position.y, allocator);
    position.AddMember("z", msg.pose.pose.position.z, allocator);
    document.AddMember("position", position, allocator);
    rapidjson::Value orientation(rapidjson::kObjectType);
    orientation.AddMember("x", msg.pose.pose.orientation.x, allocator);
    orientation.AddMember("y", msg.pose.pose.orientation.y, allocator);
    orientation.AddMember("z", msg.pose.pose.orientation.z, allocator);
    orientation.AddMember("w", msg.pose.pose.orientation.w, allocator);
    document.AddMember("orientation", orientation, allocator);

    std::vector<double> covariance;
    rapidjson::Value covariance_json(rapidjson::kArrayType);
    for (const auto value : msg.pose.covariance) {
      covariance.push_back(value);
      covariance_json.PushBack(value, allocator);
    }
    document.AddMember("covariance", covariance_json, allocator);
    add_string(document, allocator, "frame_id", msg.header.frame_id);
    document.AddMember("timestamp", timestamp, allocator);
    add_string(document, allocator, "pose_quality", pose_speed_adapter_.estimate_pose_quality(covariance));
    document.AddMember("location_confidence", pose_speed_adapter_.calculate_confidence(covariance), allocator);
    add_string(document, allocator, "source", "tf_realpose");
    data_store_.set_data("robot_pose", document_to_json(document), timestamp);

    const double pose_timestamp = msg.header.stamp.sec > 0 || msg.header.stamp.nanosec > 0 ?
      static_cast<double>(msg.header.stamp.sec) + static_cast<double>(msg.header.stamp.nanosec) * 1e-9 :
      timestamp;
    const auto speed = estimate_actual_speed_from_pose(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      QuaternionValue{
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w},
      pose_timestamp,
      timestamp);
    if (speed.has_value()) {
      rapidjson::Document speed_doc;
      speed_doc.SetObject();
      add_speed_payload(speed_doc, speed_doc.GetAllocator(), speed.value());
      data_store_.set_data("robot_speed", document_to_json(speed_doc), timestamp);
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    // odom 提供 robot_speed 的另一条来源。
    // 当导航不活跃或机器人动作忙碌时，速度会被抑制为 0，避免 APP 显示误导性的残余速度。
    const double timestamp = now_sec();
    const std::string suppress_reason = speed_suppression_reason(timestamp);
    if (!suppress_reason.empty()) {
      filtered_speed_valid_ = false;
      auto speed = pose_speed_adapter_.build_actual_speed_payload(
        0.0,
        0.0,
        0.0,
        timestamp,
        timestamp,
        false,
        suppress_reason);
      rapidjson::Document document;
      document.SetObject();
      add_speed_payload(document, document.GetAllocator(), speed);
      data_store_.set_data("robot_speed", document_to_json(document), timestamp);
      return;
    }

    TwistValue twist;
    twist.linear_x = msg.twist.twist.linear.x;
    twist.linear_y = msg.twist.twist.linear.y;
    twist.linear_z = msg.twist.twist.linear.z;
    twist.angular_x = msg.twist.twist.angular.x;
    twist.angular_y = msg.twist.twist.angular.y;
    twist.angular_z = msg.twist.twist.angular.z;
    auto speed = pose_speed_adapter_.convert_fastlio_velocity(twist, timestamp);
    speed.source = "odom_twist";
    if (!pose_speed_adapter_.has_nonzero_speed(speed)) {
      return;
    }

    rapidjson::Document document;
    document.SetObject();
    add_speed_payload(document, document.GetAllocator(), speed);
    data_store_.set_data("robot_speed", document_to_json(document), timestamp);
  }

  std::optional<SpeedPayload> estimate_actual_speed_from_pose(
    const double x,
    const double y,
    const QuaternionValue & orientation,
    const double pose_timestamp,
    const double timestamp)
  {
    // 使用相邻定位位姿估计实际速度，并做跳变剔除和一阶滤波。
    // 这条链路主要服务 APP 展示，不参与底盘控制，所以宁可保守置 0，也不输出异常尖峰。
    const double yaw = pose_speed_adapter_.yaw_from_orientation(orientation);
    const std::string suppress_reason = speed_suppression_reason(timestamp);
    LastPose current{x, y, yaw, pose_timestamp};
    const bool had_previous = last_pose_valid_;
    const LastPose previous = last_pose_;
    last_pose_ = current;
    last_pose_valid_ = true;

    if (!suppress_reason.empty()) {
      filtered_speed_valid_ = false;
      return pose_speed_adapter_.build_actual_speed_payload(
        0.0, 0.0, 0.0, pose_timestamp, timestamp, false, suppress_reason);
    }
    if (!had_previous) {
      return pose_speed_adapter_.build_actual_speed_payload(
        0.0, 0.0, 0.0, pose_timestamp, timestamp, false, "initializing");
    }

    const double dt = current.time - previous.time;
    if (dt < 0.03) {
      return std::nullopt;
    }
    if (dt > 1.0) {
      filtered_speed_valid_ = false;
      return pose_speed_adapter_.build_actual_speed_payload(
        0.0, 0.0, 0.0, pose_timestamp, timestamp, false, "pose_gap_reset");
    }

    const double map_vx = (current.x - previous.x) / dt;
    const double map_vy = (current.y - previous.y) / dt;
    const double linear_x = std::cos(current.yaw) * map_vx + std::sin(current.yaw) * map_vy;
    const double linear_y = -std::sin(current.yaw) * map_vx + std::cos(current.yaw) * map_vy;
    const double angular_z = pose_speed_adapter_.normalize_angle(current.yaw - previous.yaw) / dt;
    if (std::hypot(linear_x, linear_y) > 2.0 || std::abs(angular_z) > 4.0) {
      filtered_speed_valid_ = false;
      return pose_speed_adapter_.build_actual_speed_payload(
        0.0, 0.0, 0.0, pose_timestamp, timestamp, false, "pose_jump_rejected");
    }

    if (!filtered_speed_valid_) {
      filtered_linear_x_ = linear_x;
      filtered_linear_y_ = linear_y;
      filtered_angular_z_ = angular_z;
      filtered_speed_valid_ = true;
    } else {
      constexpr double alpha = 0.35;
      filtered_linear_x_ += alpha * (linear_x - filtered_linear_x_);
      filtered_linear_y_ += alpha * (linear_y - filtered_linear_y_);
      filtered_angular_z_ += alpha * (angular_z - filtered_angular_z_);
    }
    return pose_speed_adapter_.build_actual_speed_payload(
      filtered_linear_x_,
      filtered_linear_y_,
      filtered_angular_z_,
      pose_timestamp,
      timestamp);
  }

  std::string speed_suppression_reason(const double timestamp)
  {
    // 速度抑制用于避免非导航状态或机器人正在执行上半身动作时显示“正在移动”。
    // 返回空字符串表示允许发布速度，非空字符串会进入 reject_reason 便于调试。
    const auto system_status = data_store_.get_data("system_status");
    if (system_status.has_value()) {
      rapidjson::Document status;
      if (parse_object(system_status->json, status)) {
        const auto * details = find_member(status, "details");
        const bool motion_busy =
          truthy_json_value(find_member(status, "motion_busy")) ||
          (details != nullptr && details->IsObject() && truthy_json_value(find_member(*details, "motion_busy")));
        if (motion_busy) {
          const std::string current_motion =
            value_to_string(find_member(status, "current_motion"),
            details != nullptr && details->IsObject() ?
            value_to_string(find_member(*details, "current_motion")) :
            "");
          return current_motion.empty() ? "robot_motion_busy" : "robot_motion_busy:" + current_motion;
        }
      }
    }

    const auto navigation_status = data_store_.get_data("navigation_status");
    if (!navigation_status.has_value() || timestamp - navigation_status->updated_at_sec > 2.0) {
      return "navigation_inactive";
    }
    rapidjson::Document nav;
    if (!parse_object(navigation_status->json, nav)) {
      return "navigation_inactive";
    }
    std::string current_state = value_to_string(find_member(nav, "current_state"));
    std::transform(current_state.begin(), current_state.end(), current_state.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
    const bool active = truthy_json_value(find_member(nav, "is_active")) ||
      current_state == "planning" || current_state == "executing";
    return active ? "" : "navigation_inactive";
  }

  void imu_callback(const sensor_msgs::msg::Imu & msg)
  {
    // IMU 数据只做轻量缓存，供传感器状态查询或后续诊断使用。
    // 当前不从这里推导导航状态，避免把高频 IMU 噪声引入 APP 状态判断。
    rapidjson::Document document;
    document.SetObject();
    auto & allocator = document.GetAllocator();
    const double timestamp = now_sec();

    rapidjson::Value orientation(rapidjson::kObjectType);
    orientation.AddMember("x", msg.orientation.x, allocator);
    orientation.AddMember("y", msg.orientation.y, allocator);
    orientation.AddMember("z", msg.orientation.z, allocator);
    orientation.AddMember("w", msg.orientation.w, allocator);
    document.AddMember("orientation", orientation, allocator);

    rapidjson::Value angular_velocity(rapidjson::kObjectType);
    angular_velocity.AddMember("x", msg.angular_velocity.x, allocator);
    angular_velocity.AddMember("y", msg.angular_velocity.y, allocator);
    angular_velocity.AddMember("z", msg.angular_velocity.z, allocator);
    document.AddMember("angular_velocity", angular_velocity, allocator);

    rapidjson::Value linear_acceleration(rapidjson::kObjectType);
    linear_acceleration.AddMember("x", msg.linear_acceleration.x, allocator);
    linear_acceleration.AddMember("y", msg.linear_acceleration.y, allocator);
    linear_acceleration.AddMember("z", msg.linear_acceleration.z, allocator);
    document.AddMember("linear_acceleration", linear_acceleration, allocator);

    document.AddMember("timestamp", timestamp, allocator);
    const std::string json = document_to_json(document);
    data_store_.set_data("imu", json, timestamp);
    data_store_.set_data("sensor_data", json, timestamp);
  }

  void maybe_publish_navigation_exception(
    const rapidjson::Document & status,
    const std::string & event_type)
  {
    // 导航异常事件统一入口：
    // 只有失败/取消/阻塞/定位恢复失败等关键事件会生成 system_exception。
    if (event_type != "navigation_failed" &&
      event_type != "navigation_aborted" &&
      event_type != "navigation_obstacle_blocked" &&
      event_type != "navigation_localization_recovery_failed" &&
      event_type != "navigation_localization_resume_failed")
    {
      return;
    }

    const rapidjson::Value * event_data = find_member(status, "event_data");
    const rapidjson::Value & details =
      (event_data != nullptr && event_data->IsObject()) ? *event_data : static_cast<const rapidjson::Value &>(status);

    const std::string message =
      value_to_string(find_member(details, "message"),
      value_to_string(find_member(details, "reason"),
      value_to_string(find_member(details, "error_message"),
      value_to_string(find_member(status, "message"),
      value_to_string(find_member(status, "reason"),
      value_to_string(find_member(status, "error_message"), event_type))))));

    std::string title = "导航异常";
    std::string category = "navigation";
    std::string severity = "error";
    if (event_type.find("localization") != std::string::npos) {
      title = "定位恢复异常";
      category = "localization";
    } else if (event_type == "navigation_obstacle_blocked") {
      title = "导航受阻";
      severity = "warning";
    }

    std::string final_message = message;
    if (event_type == "navigation_failed") {
      const std::string target_task_id = value_to_string(find_member(details, "current_target_task_id"));
      if (!target_task_id.empty()) {
        title = "路线任务导航失败";
        final_message += "，current_target_task_id=" + target_task_id;
      }
    }

    publish_system_exception(
      category,
      severity,
      title,
      final_message,
      value_to_string(find_member(details, "failure_code"), event_type),
      event_type,
      details);
  }

  void cache_and_publish_adapter_payload(const std::string & adapter_payload_json)
  {
    // Adapter 输出统一包含 data_type/data/metadata。
    // 本函数负责拆出 data 写缓存，并按 adapter 指定的 message_type 生成 APP 外壳：
    // - map_response/waypoint_response 这类命令结果保持 response 语义；
    // - map_status/waypoints_data 这类状态同步保持 push 语义。
    // 这样既能保留 request_id/status/error_code 等回执字段，也能让协议和 Excel 表定义一致。
    rapidjson::Document payload;
    if (!parse_object(adapter_payload_json, payload)) {
      RCLCPP_WARN(get_logger(), "adapter payload JSON 解析失败");
      return;
    }

    const std::string data_type = value_to_string(find_member(payload, "data_type"));
    std::string message_type = value_to_string(find_member(payload, "message_type"), "push");
    if (message_type != "response" && message_type != "push") {
      message_type = "push";
    }
    const auto * data = find_member(payload, "data");
    if (data_type.empty() || data == nullptr) {
      RCLCPP_WARN(get_logger(), "adapter payload 缺少 data_type 或 data");
      return;
    }

    const double timestamp = now_sec();
    const std::string data_json = document_to_json(*data);
    data_store_.set_data(data_type, data_json, timestamp);

    auto envelope = protocol_builder_.create_base_message(
      message_type,
      data_type,
      "data_integration",
      "all",
      timestamp);
    if (!envelope.ok) {
      RCLCPP_WARN(get_logger(), "adapter payload 外壳组包失败: %s", envelope.error.c_str());
      return;
    }

    rapidjson::Document out;
    if (!parse_object(envelope.json, out)) {
      RCLCPP_WARN(get_logger(), "adapter payload 外壳 JSON 解析失败");
      return;
    }

    auto & allocator = out.GetAllocator();
    out["data"].CopyFrom(*data, allocator);
    if (const auto * metadata = find_member(payload, "metadata");
      metadata != nullptr && metadata->IsObject() && out.HasMember("metadata"))
    {
      out["metadata"].CopyFrom(*metadata, allocator);
    }

    std_msgs::msg::String msg;
    msg.data = document_to_json(out);
    if (message_type == "response") {
      data_response_pub_->publish(msg);
    } else {
      push_message_pub_->publish(msg);
    }
  }

  std::string prepare_data_response_json(
    const std::string & data_type,
    const std::string & base_json,
    const rapidjson::Value * request_params)
  {
    // APP 查询可以带轻量参数，例如 robot_pose 的 detail_level、navigation_path 的简化倍数。
    // 这里只裁剪/增强返回体，不改变 DataStore 中保存的原始缓存。
    rapidjson::Document base;
    if (!parse_object(base_json, base)) {
      return base_json;
    }

    rapidjson::Document response;
    response.SetObject();
    auto & allocator = response.GetAllocator();

    if (data_type == "robot_pose") {
      const std::string detail_level =
        request_params != nullptr ? value_to_string(find_member(*request_params, "detail_level"), "standard") : "standard";
      if (detail_level == "minimal") {
        if (const auto * position = find_member(base, "position")) {
          response.AddMember("position", rapidjson::Value(*position, allocator), allocator);
        } else {
          response.AddMember("position", rapidjson::Value(rapidjson::kObjectType), allocator);
        }
        if (const auto * orientation = find_member(base, "orientation")) {
          response.AddMember("orientation", rapidjson::Value(*orientation, allocator), allocator);
        } else {
          response.AddMember("orientation", rapidjson::Value(rapidjson::kObjectType), allocator);
        }
        response.AddMember("timestamp", value_to_double(find_member(base, "timestamp"), 0.0), allocator);
        return document_to_json(response);
      }
      if (detail_level == "enhanced") {
        response.CopyFrom(base, allocator);
        rapidjson::Value info(rapidjson::kObjectType);
        info.AddMember("location_confidence", value_to_double(find_member(base, "location_confidence"), 0.0), allocator);
        add_string(info, allocator, "pose_quality", value_to_string(find_member(base, "pose_quality"), "unknown"));
        add_string(info, allocator, "coordinate_system", "map");
        response.AddMember("additional_info", info, allocator);
        return document_to_json(response);
      }
      return base_json;
    }

    if (data_type == "navigation_path") {
      response.CopyFrom(base, allocator);
      const int simplify_factor =
        request_params != nullptr ? value_to_int(find_member(*request_params, "simplify_factor"), 1) : 1;
      const bool include_waypoints =
        request_params != nullptr ? value_to_bool(find_member(*request_params, "include_waypoints"), true) : true;
      const auto * poses = find_member(base, "path_poses");
      if (simplify_factor > 1 && poses != nullptr && poses->IsArray() &&
        static_cast<int>(poses->Size()) > simplify_factor)
      {
        rapidjson::Value simplified(rapidjson::kArrayType);
        const int step = std::max(1, static_cast<int>(poses->Size()) / simplify_factor);
        for (rapidjson::SizeType i = 0; i < poses->Size(); i += static_cast<rapidjson::SizeType>(step)) {
          simplified.PushBack(rapidjson::Value((*poses)[i], allocator), allocator);
        }
        response.RemoveMember("path_poses");
        response.AddMember("path_poses", simplified, allocator);
        response.AddMember("simplified", true, allocator);
        response.AddMember("original_waypoint_count", static_cast<int>(poses->Size()), allocator);
      }
      if (!include_waypoints) {
        response.RemoveMember("path_poses");
        response.AddMember("summary_only", true, allocator);
      }
      return document_to_json(response);
    }

    if (data_type == "navigation_status") {
      const bool include_details =
        request_params != nullptr ? value_to_bool(find_member(*request_params, "include_details"), true) : true;
      const bool include_performance =
        request_params != nullptr ? value_to_bool(find_member(*request_params, "include_performance"), true) : true;
      if (!include_details) {
        add_string(
          response,
          allocator,
          "navigation_state",
          value_to_string(find_member(base, "navigation_state"), "unknown"));
        response.AddMember("progress_percentage", value_to_double(find_member(base, "progress_percentage"), 0.0), allocator);
        response.AddMember("timestamp", value_to_double(find_member(base, "system_timestamp"), 0.0), allocator);
        return document_to_json(response);
      }
      response.CopyFrom(base, allocator);
      if (!include_performance) {
        response.RemoveMember("performance_metrics");
      }
      return document_to_json(response);
    }

    return base_json;
  }

  void publish_navigation_ack_push(
    const std::string & navigation_status_json,
    const std::string & raw_ack_json,
    const double timestamp)
  {
    // 导航 ack push 会带上 raw ack 中的上下文字段，保证 APP 任务详情能显示路线、点位和播报状态。
    const auto push = protocol_builder_.create_push_message(
      "navigation_status",
      navigation_status_json,
      "all",
      timestamp,
      timestamp,
      0);
    if (!push.ok) {
      RCLCPP_WARN(get_logger(), "navigation_ack push 组包失败: %s", push.error.c_str());
      return;
    }

    rapidjson::Document ack;
    rapidjson::Document push_doc;
    if (!parse_object(raw_ack_json, ack) || !parse_object(push.json, push_doc)) {
      RCLCPP_WARN(get_logger(), "navigation_ack push JSON 修饰失败");
      return;
    }

    auto & allocator = push_doc.GetAllocator();
    rapidjson::Value & metadata = push_doc["metadata"];
    const std::string status = value_to_string(find_member(ack, "status"));
    const std::string ack_type = value_to_string(find_member(ack, "ack_type"));
    const std::string message = value_to_string(find_member(ack, "message"));
    set_string(metadata, allocator, "status", status);
    set_string(metadata, allocator, "push_reason", "navigation_ack");
    if (status == "error") {
      set_string(
        metadata,
        allocator,
        "error_code",
        value_to_string(find_member(ack, "error_code"), ack_type.empty() ? "nav_error" : ack_type));
      set_string(metadata, allocator, "error_message", message);
      publish_system_exception(
        "navigation",
        "error",
        "导航异常",
        message.empty() ? "导航命令执行失败" : message,
        ack_type.empty() ? "nav_error" : ack_type,
        "navigation_ack",
        ack);
    }

    std_msgs::msg::String out;
    out.data = document_to_json(push_doc);
    push_message_pub_->publish(out);
  }

  void publish_action_result_push(
    const rapidjson::Document & action_result,
    const std::string & action_result_json,
    const double timestamp)
  {
    // 动作结果 push 保留原始动作字段，同时补充 metadata 状态，
    // 让 APP 网关无需理解动作业务即可转发。
    const auto push = protocol_builder_.create_push_message(
      "action_result",
      action_result_json,
      "all",
      timestamp,
      timestamp,
      0);
    if (!push.ok) {
      RCLCPP_WARN(get_logger(), "action_result push 组包失败: %s", push.error.c_str());
      return;
    }

    rapidjson::Document push_doc;
    if (!parse_object(push.json, push_doc)) {
      RCLCPP_WARN(get_logger(), "action_result push JSON 修饰失败");
      return;
    }

    auto & allocator = push_doc.GetAllocator();
    rapidjson::Value & metadata = push_doc["metadata"];
    const std::string status = value_to_string(find_member(action_result, "status"));
    set_string(metadata, allocator, "push_reason", "action_result");
    set_string(metadata, allocator, "qos_level", "realtime");
    set_string(metadata, allocator, "status", status == "success" ? "success" : "error");
    if (status != "success") {
      set_string(
        metadata,
        allocator,
        "error_code",
        value_to_string(find_member(action_result, "result"), "action_failed"));
      set_string(
        metadata,
        allocator,
        "error_message",
        value_to_string(find_member(action_result, "message"), "动作执行失败"));
    }

    std_msgs::msg::String out;
    out.data = document_to_json(push_doc);
    push_message_pub_->publish(out);
  }

  void publish_system_exception_push(
    const std::string & exception_json,
    const std::string & severity,
    const std::string & code,
    const std::string & message,
    const double timestamp)
  {
    // system_exception 的主动推送出口，metadata 中会补充 severity/code/message，
    // 方便 APP 事件日志按异常级别做颜色和弹窗处理。
    const auto push = protocol_builder_.create_push_message(
      "system_exception",
      exception_json,
      "all",
      timestamp,
      timestamp,
      0);
    if (!push.ok) {
      RCLCPP_WARN(get_logger(), "system_exception push 组包失败: %s", push.error.c_str());
      return;
    }

    rapidjson::Document push_doc;
    if (!parse_object(push.json, push_doc)) {
      RCLCPP_WARN(get_logger(), "system_exception push JSON 修饰失败");
      return;
    }

    auto & allocator = push_doc.GetAllocator();
    rapidjson::Value & metadata = push_doc["metadata"];
    set_string(metadata, allocator, "push_reason", "exception_event");
    set_string(metadata, allocator, "qos_level", "realtime");
    set_string(metadata, allocator, "status", (severity == "error" || severity == "critical") ? "error" : severity);
    set_string(metadata, allocator, "error_code", code);
    set_string(metadata, allocator, "error_message", message);

    std_msgs::msg::String out;
    out.data = document_to_json(push_doc);
    push_message_pub_->publish(out);
  }

  void publish_push(
    const std::string & data_type,
    const std::string & data_json,
    const std::string & destination,
    const double timestamp,
    const double last_update_time,
    const int subscription_count)
  {
    // 通用 push 出口：所有主动推送最终都走 ProtocolBuilder，
    // 保证 message_type、data_type、timestamp、metadata 等协议字段格式一致。
    const auto push = protocol_builder_.create_push_message(
      data_type,
      data_json,
      destination,
      timestamp,
      last_update_time,
      subscription_count);
    if (!push.ok) {
      RCLCPP_WARN(get_logger(), "integration push 组包失败: %s", push.error.c_str());
      return;
    }

    std_msgs::msg::String out;
    out.data = push.json;
    push_message_pub_->publish(out);
  }

  void publish_error(
    const std::string & client_id,
    const std::string & request_id,
    const std::string & data_type,
    const std::string & error_code,
    const std::string & error_message)
  {
    // 通用错误响应出口：APP request 解析失败、数据不可用或组包失败时使用。
    const auto response = protocol_builder_.create_specific_error(
      client_id,
      request_id,
      data_type,
      error_code,
      error_message,
      now_sec());
    if (!response.ok) {
      RCLCPP_WARN(get_logger(), "integration error 组包失败: %s", response.error.c_str());
      return;
    }

    std_msgs::msg::String out;
    out.data = response.json;
    data_response_pub_->publish(out);
  }

  void cleanup_expired_data()
  {
    // 定时清理过期缓存，避免 APP 查询到长时间未更新的机器人/导航状态。
    const auto expired = data_store_.cleanup_expired(now_sec());
    if (!expired.empty()) {
      RCLCPP_DEBUG(get_logger(), "已清理 %zu 类过期数据", expired.size());
    }
  }

  void push_data_updates()
  {
    // 订阅主动推送主循环：按订阅频率检查 DataStore 中的新鲜数据。
    // 这里不会主动拉取上游数据，只负责把已有缓存按频率送给订阅客户端。
    const double timestamp = now_sec();
    for (const auto & data_type : subscribed_data_types_) {
      const auto stored = data_store_.get_fresh_data(data_type, timestamp);
      if (!stored.has_value()) {
        continue;
      }

      const auto subscribers = subscription_manager_.get_subscribers(data_type, timestamp);
      for (const auto & subscriber : subscribers) {
        publish_push(
          data_type,
          stored->json,
          subscriber.client_id,
          timestamp,
          stored->updated_at_sec,
          static_cast<int>(subscribers.size()));
        subscription_manager_.update_push_time(subscriber.client_id, data_type, timestamp);
      }
    }
  }

  void publish_subscription_response(
    const std::string & original_message_json,
    const bool success,
    const std::string & message)
  {
    // 订阅响应会引用原始 request 中的 client_id/request_id/data_types，
    // 方便 APP 把订阅结果和页面上的请求动作对应起来。
    const auto response = protocol_builder_.create_subscription_response(
      original_message_json,
      success,
      message,
      now_sec());
    if (!response.ok) {
      RCLCPP_WARN(get_logger(), "subscription response 组包失败: %s", response.error.c_str());
      return;
    }

    std_msgs::msg::String out;
    out.data = response.json;
    subscription_response_pub_->publish(out);
  }

  double now_sec() const
  {
    return static_cast<double>(now().nanoseconds()) / 1e9;
  }

  DataIntegrationConfig config_;
  DataStore data_store_;
  RobotStatusAdapter robot_status_adapter_;
  NavigationStatusAdapter navigation_status_adapter_;
  MapWaypointAdapter map_waypoint_adapter_;
  PoseSpeedAdapter pose_speed_adapter_;
  PathMetrics path_metrics_;
  ProtocolBuilder protocol_builder_;
  SubscriptionManager subscription_manager_;
  const std::vector<std::string> subscribed_data_types_{
    "system_status",
    "navigation_status",
    "navigation_path",
    "robot_pose",
    "robot_speed",
    "map_status",
    "map_response",
    "waypoints_data",
    "action_result",
    "system_exception",
    "sensor_data",
    "imu",
    "gesture_list",
    "facial_gesture_list"};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_raw_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_request_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_ack_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_response_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoints_data_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_result_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_recovery_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gesture_update_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr navigation_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_realpose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_response_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr push_message_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr subscription_response_pub_;
  rclcpp::TimerBase::SharedPtr cleanup_timer_;
  rclcpp::TimerBase::SharedPtr push_timer_;
  rclcpp::TimerBase::SharedPtr initial_sync_timer_;
  std::unordered_map<std::string, double> last_exception_push_times_;
  struct LastPose
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double time{0.0};
  };
  LastPose last_pose_;
  bool last_pose_valid_{false};
  bool filtered_speed_valid_{false};
  double filtered_linear_x_{0.0};
  double filtered_linear_y_{0.0};
  double filtered_angular_z_{0.0};
};

}  // namespace humanoid_app_gateway_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_app_gateway_runtime::DataIntegrationNode>());
  rclcpp::shutdown();
  return 0;
}
