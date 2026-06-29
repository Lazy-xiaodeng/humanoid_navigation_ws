/*
 * app_gateway_node.cpp
 *
 * 文件用途：
 * 1. APP 网关 C++ 节点，负责 APP WebSocket 连接管理、协议入口和 integration 消息回发。
 * 2. 支持 APP 数据请求、订阅请求、业务命令转 ROS topic，以及 integration 响应/推送/订阅响应转发。
 * 3. WebSocket server 受 websocket_server_enable 控制，便于在真实接管前先做离线和旁路验证。
 * 4. 上游：APP / 导航页 WebSocket 客户端，以及 data_integration_node 发布的 integration 系列话题。
 * 5. 下游：/websocket/data_requests、/websocket/data_subscriptions、app 系列 topic、/robot/facial_raw_cmd、/initialpose。
 *
 * 代码块顺序：
 * 1. 匿名工具函数：JSON 字段提取、类型转换、消息补源和 message_id 生成。
 * 2. WebSocket 连接管理：启动、停止、accept 循环、单客户端读写循环和断开清理。
 * 3. APP 入站处理：request、subscription、business_command 三类消息分发。
 * 4. 业务命令出口：导航、地图、点位、机器人控制、表情、初始位姿和播报音量服务。
 * 5. integration 回流：data_response、push、subscription_response 选择目标客户端并发送。
 * 6. 初始快照与健康检查：新连接回显、超时客户端清理和周期状态日志。
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "humanoid_interfaces/srv/set_broadcast_volume.hpp"
#include "humanoid_app_gateway_runtime/app_gateway_config.hpp"
#include "humanoid_app_gateway_runtime/app_gateway_types.hpp"
#include "humanoid_app_gateway_runtime/app_protocol.hpp"
#include "humanoid_app_gateway_runtime/business_command_router.hpp"
#include "humanoid_app_gateway_runtime/client_registry.hpp"
#include "humanoid_app_gateway_runtime/integration_forwarder.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace humanoid_app_gateway_runtime
{

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

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
    std::ostringstream stream;
    stream << value->GetDouble();
    return stream.str();
  }
  return fallback;
}

double value_to_double(const rapidjson::Value * value, const double fallback = 0.0)
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

std::string document_to_json(const rapidjson::Value & value)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  value.Accept(writer);
  return buffer.GetString();
}

std::string generate_message_id(const std::string & prefix, const double timestamp_sec)
{
  const auto timestamp_ms = static_cast<long long>(timestamp_sec * 1000.0);
  static thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<int> dist(0, 0xFFFFFF);
  std::ostringstream suffix;
  suffix << std::hex << std::setw(6) << std::setfill('0') << dist(rng);
  return prefix + "_" + std::to_string(timestamp_ms) + "_" + suffix.str();
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

std::string message_with_source(
  const std::string & message_json,
  const std::string & client_id,
  const std::string & default_destination)
{
  rapidjson::Document document;
  document.Parse(message_json.c_str());
  if (document.HasParseError() || !document.IsObject()) {
    return message_json;
  }
  auto & allocator = document.GetAllocator();
  if (document.HasMember("source")) {
    document["source"].SetString(client_id.c_str(), allocator);
  } else {
    add_string(document, allocator, "source", client_id);
  }
  if (!default_destination.empty()) {
    if (document.HasMember("destination")) {
      document["destination"].SetString(default_destination.c_str(), allocator);
    } else {
      add_string(document, allocator, "destination", default_destination);
    }
  }
  return document_to_json(document);
}

}  // namespace

struct ClientConnection
{
  std::string id;
  std::shared_ptr<websocket::stream<tcp::socket>> ws;
  std::mutex write_mutex;
};

class AppGatewayNode : public rclcpp::Node
{
public:
  explicit AppGatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("app_gateway_node", options), acceptor_(ioc_)
  {
    declare_app_gateway_parameters(*this, config_);
    config_ = load_app_gateway_config(*this, config_);
    setup_publishers_and_subscribers();
    setup_runtime_timers();

    if (config_.websocket_server_enable) {
      start_websocket_server();
    }

    RCLCPP_WARN(
      get_logger(),
      "app_gateway_node 已启动：websocket_server_enable=%s，integration 转发和业务命令路由已接入。",
      config_.websocket_server_enable ? "true" : "false");
  }

  ~AppGatewayNode() override
  {
    stop_websocket_server();
  }

private:
  // ROS 边界初始化：所有 publisher/subscriber 在这里集中创建，便于核对 topic 名称和数据方向。
  // APP 入站消息经 /websocket/* 或 /app/* topic 进入下游；integration 出站消息从
  // /integration/* topic 回到这里，再由 WebSocket 写给 APP 客户端。
  void setup_publishers_and_subscribers()
  {
    data_request_pub_ = create_publisher<std_msgs::msg::String>(config_.data_requests_topic, 10);
    data_subscription_pub_ = create_publisher<std_msgs::msg::String>(config_.data_subscriptions_topic, 10);
    waypoint_command_pub_ = create_publisher<std_msgs::msg::String>(config_.app_waypoint_command_topic, 10);
    navigation_command_pub_ = create_publisher<std_msgs::msg::String>(config_.app_navigation_command_topic, 10);
    navigation_ack_pub_ = create_publisher<std_msgs::msg::String>("/navigation/acknowledgments", 10);
    map_command_pub_ = create_publisher<std_msgs::msg::String>(config_.app_map_command_topic, 10);
    robot_control_pub_ = create_publisher<std_msgs::msg::String>(config_.app_robot_control_topic, 10);
    system_command_pub_ = create_publisher<std_msgs::msg::String>(config_.app_system_command_topic, 10);
    facial_raw_command_pub_ = create_publisher<std_msgs::msg::String>(config_.facial_raw_command_topic, 10);
    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(config_.initial_pose_topic, 10);
    broadcast_volume_client_ = create_client<humanoid_interfaces::srv::SetBroadcastVolume>(
      "/xiaorui_broadcast/set_volume");

    data_response_sub_ = create_subscription<std_msgs::msg::String>(
      config_.integration_data_responses_topic,
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) { handle_integration_data_response(msg->data); });
    push_message_sub_ = create_subscription<std_msgs::msg::String>(
      config_.integration_push_messages_topic,
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) { handle_integration_push(msg->data); });
    subscription_response_sub_ = create_subscription<std_msgs::msg::String>(
      config_.integration_subscription_responses_topic,
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) { handle_integration_subscription_response(msg->data); });
  }

  // 运行态定时器只做维护工作，不承载业务状态机：
  // 1. 客户端健康检查负责清理长时间无活动连接；
  // 2. 状态报告只输出连接数和订阅数，方便现场确认网关是否在正常工作。
  void setup_runtime_timers()
  {
    if (config_.client_health_check_interval_sec > 0.0) {
      client_health_timer_ = create_wall_timer(
        std::chrono::duration<double>(config_.client_health_check_interval_sec),
        [this]() { client_health_check_tick(); });
    }
    if (config_.status_report_interval_sec > 0.0) {
      status_report_timer_ = create_wall_timer(
        std::chrono::duration<double>(config_.status_report_interval_sec),
        [this]() { status_report_tick(); });
    }
  }

  // WebSocket server 独立线程运行，避免阻塞 rclcpp executor。
  // 是否真正监听端口由 websocket_server_enable 控制；正式导航默认监听，离线验证可关闭。
  void start_websocket_server()
  {
    running_ = true;
    server_thread_ = std::thread([this]() { accept_loop(); });
  }

  // 节点退出时关闭 acceptor、逐个关闭客户端连接并等待 server 线程退出。
  // 这里不发布业务消息，避免 shutdown 期间产生额外状态扰动。
  void stop_websocket_server()
  {
    running_ = false;
    beast::error_code ec;
    acceptor_.close(ec);
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      for (auto & [client_id, client] : clients_) {
        (void)client_id;
        if (client && client->ws) {
          std::lock_guard<std::mutex> write_lock(client->write_mutex);
          client->ws->close(websocket::close_code::normal, ec);
        }
      }
      clients_.clear();
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

  // 接收新 APP 连接：
  // 1. 完成 TCP/WebSocket 握手；
  // 2. 分配内部 client_id 并登记会话；
  // 3. 发送 connection_ack；
  // 4. 可选补发导航页需要的初始业务快照；
  // 5. 为每个客户端启动独立读循环。
  void accept_loop()
  {
    try {
      const auto address = net::ip::make_address(config_.websocket_host);
      tcp::endpoint endpoint(address, static_cast<unsigned short>(config_.websocket_port));
      acceptor_.open(endpoint.protocol());
      acceptor_.set_option(net::socket_base::reuse_address(true));
      acceptor_.bind(endpoint);
      acceptor_.listen(net::socket_base::max_listen_connections);

      while (running_) {
        tcp::socket socket(ioc_);
        beast::error_code ec;
        acceptor_.accept(socket, ec);
        if (ec) {
          if (running_) {
            RCLCPP_WARN(get_logger(), "APP WebSocket accept 失败: %s", ec.message().c_str());
          }
          continue;
        }
        auto ws = std::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
        ws->accept(ec);
        if (ec) {
          RCLCPP_WARN(get_logger(), "APP WebSocket handshake 失败: %s", ec.message().c_str());
          continue;
        }
        auto client = std::make_shared<ClientConnection>();
        client->id = generate_client_id();
        client->ws = ws;
        {
          std::lock_guard<std::mutex> lock(clients_mutex_);
          clients_[client->id] = client;
        }
        client_registry_.register_client(client->id, "unknown", now_sec());
        send_to_client(client->id, build_connection_ack(client->id));
        if (config_.send_initial_snapshot_on_connect) {
          send_initial_business_data(client->id);
        }
        std::thread([this, client]() { client_read_loop(client); }).detach();
      }
    } catch (const std::exception & ex) {
      if (running_) {
        RCLCPP_ERROR(get_logger(), "APP WebSocket server 异常: %s", ex.what());
      }
    }
  }

  // 单客户端读循环：持续读取 WebSocket 文本消息，更新活跃时间后交给协议入口处理。
  // 发生异常或对端断开时，统一清理连接和订阅关系，防止 data_integration 继续向失效客户端推送。
  void client_read_loop(const std::shared_ptr<ClientConnection> & client)
  {
    try {
      while (running_) {
        beast::flat_buffer buffer;
        client->ws->read(buffer);
        const std::string message = beast::buffers_to_string(buffer.data());
        client_registry_.update_activity(client->id, now_sec());
        handle_client_message(message, client->id);
      }
    } catch (const std::exception &) {
      cleanup_client(client->id);
    }
  }

  void cleanup_client(const std::string & client_id)
  {
    const auto subscriptions = client_registry_.cleanup_client(client_id);
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      clients_.erase(client_id);
    }
    if (!subscriptions.empty()) {
      publish_unsubscribe_for_client(client_id, subscriptions);
    }
  }

  std::string generate_client_id() const
  {
    const auto timestamp_ms = static_cast<long long>(now_sec() * 1000.0);
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> dist(1000, 9999);
    return "client_" + std::to_string(timestamp_ms) + "_" + std::to_string(dist(rng));
  }

  void handle_client_message(const std::string & message, const std::string & client_id)
  {
    rapidjson::Document document;
    document.Parse(message.c_str());
    if (document.HasParseError() || !document.IsObject()) {
      send_to_client(client_id, build_error("error", "invalid_json", "消息 JSON 格式错误", client_id));
      return;
    }

    const std::string message_type = value_to_string(find_member(document, "message_type"));
    const std::string request_id = value_to_string(find_member(document, "message_id"));
    if (message_type == "request") {
      data_request_pub_->publish(make_string_msg(message_with_source(message, client_id, "data_integration")));
      send_to_client(client_id, build_simple_ack(
        "request_ack", "request", "received", "数据请求已接收，正在处理", client_id, request_id));
    } else if (message_type == "subscription") {
      handle_subscription_message(message, document, client_id);
    } else if (message_type == "command" || message_type == "business_command") {
      handle_business_command(message, client_id);
    } else {
      send_to_client(client_id, build_error("error", "unsupported_message_type", "不支持的消息类型: " + message_type, client_id));
    }
  }

  // APP 订阅入口：只解析订阅/退订意图，然后同步两处状态：
  // 1. 本节点 client_registry_ 保存“哪些客户端订阅了哪些数据”，用于 push 路由；
  // 2. /websocket/data_subscriptions 通知 data_integration_node 建立真实数据推送节流。
  void handle_subscription_message(
    const std::string & message,
    const rapidjson::Document & document,
    const std::string & client_id)
  {
    const auto * data = find_member(document, "data");
    const std::string action = data != nullptr && data->IsObject() ?
      value_to_string(find_member(*data, "action"), "subscribe") : "subscribe";
    const std::vector<std::string> data_types = data != nullptr && data->IsObject() ?
      string_array_from_json(find_member(*data, "data_types")) : std::vector<std::string>{};
    const double frequency = data != nullptr && data->IsObject() ?
      value_to_double(find_member(*data, "push_frequency"), 1.0) : 1.0;
    client_registry_.apply_subscription_request(client_id, action, data_types, frequency, now_sec());
    data_subscription_pub_->publish(make_string_msg(message_with_source(message, client_id, "data_integration")));
    send_to_client(client_id, build_simple_ack(
      "subscription_ack",
      "subscription",
      "forwarded",
      "订阅请求已转发到数据服务",
      client_id,
      value_to_string(find_member(document, "message_id"))));
  }

  // APP 业务命令入口：所有 command 先由 BusinessCommandRouter 做协议归一化。
  // 本函数只负责错误回包、异常事件补发和把路由结果交给 publish_business_payload()。
  void handle_business_command(const std::string & message, const std::string & client_id)
  {
    const auto routed = business_router_.route_business_command(message, client_id, now_sec());
    if (!routed.ok) {
      maybe_publish_business_exception(message, client_id, routed.error);
      send_to_client(client_id, build_error("error", "PROCESSING_ERROR", routed.error, client_id));
      return;
    }

    publish_business_payload(routed);
    if (!routed.ack_json.empty()) {
      send_to_client(client_id, wrap_ack_payload("command_ack", routed.ack_json, client_id, routed.ack_request_id));
    }
  }

  // 业务命令出口：根据 router 判定的目标，把 payload 发布到对应 ROS topic 或调用服务。
  // 这样 APP 协议和 ROS 发布细节解耦，后续新增命令时优先扩展 router 和这个出口表。
  void publish_business_payload(const BusinessCommandRouteResult & routed)
  {
    if (routed.target == BusinessCommandTarget::InitialPose) {
      publish_initial_pose(routed.payload_json);
      return;
    }
    if (routed.target == BusinessCommandTarget::BroadcastVolumeService) {
      handle_broadcast_volume_service(routed.payload_json);
      return;
    }
    if (routed.target == BusinessCommandTarget::FacialRawCommand) {
      publish_facial_raw_command(routed.payload_json);
      return;
    }

    auto msg = make_string_msg(routed.payload_json);
    switch (routed.target) {
      case BusinessCommandTarget::WaypointCommand:
        waypoint_command_pub_->publish(msg);
        break;
      case BusinessCommandTarget::NavigationCommand:
        navigation_command_pub_->publish(msg);
        break;
      case BusinessCommandTarget::MapCommand:
        map_command_pub_->publish(msg);
        break;
      case BusinessCommandTarget::RobotControl:
        robot_control_pub_->publish(msg);
        break;
      case BusinessCommandTarget::FacialRawCommand:
        break;
      case BusinessCommandTarget::SystemCommand:
        system_command_pub_->publish(msg);
        break;
      default:
        break;
    }
  }

  // initial_pose 是 geometry_msgs 消息，不能直接透传 JSON。
  // 这里把 APP 的 pose/yaw 字段转换成 /initialpose，并使用 map 坐标系给定位链路消费。
  void publish_initial_pose(const std::string & payload_json)
  {
    rapidjson::Document payload;
    payload.Parse(payload_json.c_str());
    if (payload.HasParseError() || !payload.IsObject()) {
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = value_to_string(find_member(payload, "frame_id"), "map");
    msg.pose.pose.position.x = value_to_double(find_member(payload, "x"));
    msg.pose.pose.position.y = value_to_double(find_member(payload, "y"));
    msg.pose.pose.orientation.w = value_to_double(find_member(payload, "qw"), 1.0);
    msg.pose.pose.orientation.z = value_to_double(find_member(payload, "qz"), 0.0);
    msg.pose.covariance = {
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685};
    initial_pose_pub_->publish(msg);
  }

  // 表情命令保持 JSON 原样下发到机器人表情链路，避免网关层理解具体表情枚举。
  void publish_facial_raw_command(const std::string & payload_json)
  {
    rapidjson::Document payload;
    payload.Parse(payload_json.c_str());
    if (payload.HasParseError() || !payload.IsObject()) {
      return;
    }
    const std::string action = value_to_string(find_member(payload, "action"));
    if (action.empty()) {
      return;
    }
    // facial_driver 订阅的是纯动作 ID 字符串，不能发布 JSON，否则旧链路会解析失败。
    facial_raw_command_pub_->publish(make_string_msg(action));
  }

  // 播报音量是服务调用，不是普通 topic。
  // 这里负责把 APP command 转成 SetBroadcastVolume 请求，并把服务结果包装成导航 ack 事件返回。
  void handle_broadcast_volume_service(const std::string & payload_json)
  {
    rapidjson::Document payload;
    payload.Parse(payload_json.c_str());
    if (payload.HasParseError() || !payload.IsObject()) {
      publish_navigation_ack(build_broadcast_volume_ack("", "error", "broadcast_volume_invalid_payload", "播报音量命令格式错误"));
      return;
    }

    const std::string request_id = value_to_string(find_member(payload, "request_message_id"));
    const int volume = static_cast<int>(value_to_double(find_member(payload, "volume_percent"), 72.0));
    if (!broadcast_volume_client_->service_is_ready()) {
      publish_navigation_ack(build_broadcast_volume_ack(
        request_id,
        "error",
        "broadcast_service_unavailable",
        "播报服务未启动，无法设置 ROS 音量"));
      return;
    }

    auto request = std::make_shared<humanoid_interfaces::srv::SetBroadcastVolume::Request>();
    request->volume_percent = volume;
    broadcast_volume_client_->async_send_request(
      request,
      [this, request_id](rclcpp::Client<humanoid_interfaces::srv::SetBroadcastVolume>::SharedFuture future) {
        try {
          const auto response = future.get();
          auto ack = build_broadcast_volume_ack(
            request_id,
            response->success ? "success" : "error",
            response->success ? "" : "broadcast_volume_failed",
            response->message.empty() ?
            (response->success ? "播报音量已设置" : "播报音量设置失败") :
            response->message);
          rapidjson::Document document;
          document.Parse(ack.c_str());
          auto & allocator = document.GetAllocator();
          if (response->success) {
            add_string(document, allocator, "result_reason", "broadcast_volume_applied");
            document.AddMember("applied_volume_percent", response->applied_volume_percent, allocator);
            add_string(document, allocator, "selected_device", response->selected_device);
            add_string(document, allocator, "backend", response->backend);
          }
          publish_navigation_ack(document_to_json(document));
        } catch (const std::exception & ex) {
          publish_navigation_ack(build_broadcast_volume_ack(
            request_id,
            "error",
            "broadcast_volume_exception",
            std::string("播报音量设置异常: ") + ex.what()));
        }
      });
  }

  std::string build_broadcast_volume_ack(
    const std::string & request_id,
    const std::string & status,
    const std::string & error_code,
    const std::string & message) const
  {
    rapidjson::Document document;
    document.SetObject();
    auto & allocator = document.GetAllocator();
    add_string(document, allocator, "ack_type", "navigation_command_result");
    add_string(document, allocator, "command_type", "set_broadcast_volume");
    add_string(document, allocator, "request_message_id", request_id);
    add_string(document, allocator, "status", status);
    add_string(document, allocator, "message", message);
    if (!error_code.empty()) {
      add_string(document, allocator, "error_code", error_code);
    }
    document.AddMember("timestamp", now_sec(), allocator);
    return document_to_json(document);
  }

  void publish_navigation_ack(const std::string & ack_json)
  {
    navigation_ack_pub_->publish(make_string_msg(ack_json));
  }

  void maybe_publish_business_exception(
    const std::string & message_json,
    const std::string & client_id,
    const std::string & error)
  {
    rapidjson::Document message;
    message.Parse(message_json.c_str());
    if (message.HasParseError() || !message.IsObject()) {
      return;
    }
    const std::string data_type = value_to_string(find_member(message, "data_type"));
    if (data_type != "facial_control") {
      return;
    }
    publish_system_exception(
      "expression",
      "error",
      "表情控制异常",
      error,
      "facial_control_error",
      client_id);
  }

  void publish_system_exception(
    const std::string & category,
    const std::string & severity,
    const std::string & title,
    const std::string & message,
    const std::string & code,
    const std::string & client_id)
  {
    rapidjson::Document document;
    document.Parse(build_base_message("push", "system_exception", "all").c_str());
    auto & allocator = document.GetAllocator();
    document["data"].AddMember(
      "exception_id",
      rapidjson::Value(generate_message_id("exception", now_sec()).c_str(), allocator).Move(),
      allocator);
    add_string(document["data"], allocator, "category", category);
    add_string(document["data"], allocator, "severity", severity);
    add_string(document["data"], allocator, "title", title);
    add_string(document["data"], allocator, "message", message);
    add_string(document["data"], allocator, "code", code);
    rapidjson::Value details(rapidjson::kObjectType);
    add_string(details, allocator, "client_id", client_id);
    document["data"].AddMember("details", details, allocator);
    rapidjson::Value display(rapidjson::kObjectType);
    display.AddMember("popup", true, allocator);
    display.AddMember("modal", severity == "error" || severity == "critical", allocator);
    display.AddMember("auto_close", severity == "info" || severity == "warning", allocator);
    document["data"].AddMember("display", display, allocator);
    document["data"].AddMember("timestamp", now_sec(), allocator);
    add_string(document["metadata"], allocator, "push_reason", "exception_event");
    add_string(document["metadata"], allocator, "qos_level", "realtime");
    document["metadata"]["status"].SetString((severity == "error" || severity == "critical") ? "error" : severity.c_str(), allocator);
    document["metadata"]["error_code"].SetString(code.c_str(), allocator);
    document["metadata"]["error_message"].SetString(message.c_str(), allocator);
    broadcast(document_to_json(document));
  }

  // integration data_response：通常是 APP 主动 request 的结果，优先按 destination/client_id 定向回发。
  void handle_integration_data_response(const std::string & json)
  {
    const auto connected = connected_clients();
    const auto decision = integration_forwarder_.route_data_response(json, connected);
    route_decision(json, decision);
  }

  // integration push：通常是系统主动状态变化，可能广播给全部客户端，也可能只发给订阅者。
  void handle_integration_push(const std::string & json)
  {
    rapidjson::Document push;
    push.Parse(json.c_str());
    if (!push.HasParseError() && push.IsObject()) {
      update_business_state_from_push(push);
    }
    const std::string data_type = !push.HasParseError() && push.IsObject() ?
      value_to_string(find_member(push, "data_type")) : "";
    const auto connected = connected_clients();
    const auto subscribed = client_registry_.get_subscribed_clients(data_type);
    const auto decision = integration_forwarder_.route_push_message(json, connected, subscribed);
    route_decision(json, decision);
  }

  // integration subscription_response：订阅处理结果回给发起订阅的客户端。
  void handle_integration_subscription_response(const std::string & json)
  {
    const auto connected = connected_clients();
    const auto decision = integration_forwarder_.route_subscription_response(json, connected);
    route_decision(json, decision);
  }

  // 根据 IntegrationForwarder 的决策执行最终 WebSocket 发送。
  // BroadcastAll、DirectClient、SubscribedClients、Drop 四种路径集中在这里，避免各回调重复写发送逻辑。
  void route_decision(const std::string & json, const ForwardRouteDecision & decision)
  {
    if (decision.kind == ForwardRouteKind::BroadcastAll) {
      broadcast(json);
    } else if (decision.kind == ForwardRouteKind::DirectClient ||
      decision.kind == ForwardRouteKind::SubscribedClients)
    {
      for (const auto & client_id : decision.target_clients) {
        send_to_client(client_id, json);
      }
    }
  }

  void update_business_state_from_push(const rapidjson::Document & push)
  {
    const std::string data_type = value_to_string(find_member(push, "data_type"));
    const auto * data = find_member(push, "data");
    if (data == nullptr) {
      return;
    }
    std::lock_guard<std::mutex> lock(business_state_mutex_);
    if (data_type == "gesture_list" || data_type == "facial_gesture_list" || data_type == "waypoints_data") {
      business_state_[data_type] = document_to_json(*data);
    } else if (data_type == "navigation_status") {
      const std::string event_type = value_to_string(find_member(*data, "event_type"));
      if (event_type == "navigation_started") {
        current_navigation_mode_ = value_to_string(find_member(*data, "navigation_mode"));
        current_waypoint_index_ = 0;
      } else if (event_type == "waypoint_reached") {
        const auto * event_data = find_member(*data, "event_data");
        current_waypoint_index_ = static_cast<int>(
          event_data != nullptr && event_data->IsObject() ?
          value_to_double(find_member(*event_data, "waypoint_index"), 0.0) : 0.0);
      } else if (event_type == "navigation_completed" || event_type == "navigation_stopped") {
        current_navigation_mode_.clear();
        current_waypoint_index_ = 0;
      }
    }
  }

  void send_initial_business_data(const std::string & client_id)
  {
    // 保持线上协议 send_initial_business_data()：新 APP 连接后补发已有业务快照，
    // 避免刷新页面后必须等待下一次 push 才能看到点位、动作库和当前导航状态。
    std::vector<std::string> messages;
    {
      std::lock_guard<std::mutex> lock(business_state_mutex_);
      append_initial_snapshot_message(messages, client_id, "waypoints_data", "initial_data");
      append_initial_snapshot_message(messages, client_id, "gesture_list", "initial_data_sync");
      append_initial_snapshot_message(messages, client_id, "facial_gesture_list", "initial_data_sync");
      if (!current_navigation_mode_.empty()) {
        messages.push_back(build_initial_navigation_status_message(client_id));
      }
    }
    for (const auto & message : messages) {
      send_to_client(client_id, message);
    }
  }

  void append_initial_snapshot_message(
    std::vector<std::string> & messages,
    const std::string & client_id,
    const std::string & data_type,
    const std::string & push_reason) const
  {
    const auto it = business_state_.find(data_type);
    if (it == business_state_.end() || it->second.empty()) {
      return;
    }
    messages.push_back(build_push_with_data(client_id, data_type, it->second, push_reason));
  }

  std::string build_push_with_data(
    const std::string & client_id,
    const std::string & data_type,
    const std::string & data_json,
    const std::string & push_reason) const
  {
    rapidjson::Document document;
    document.Parse(build_base_message("push", data_type, client_id).c_str());
    auto & allocator = document.GetAllocator();

    rapidjson::Document data;
    data.Parse(data_json.c_str());
    if (!data.HasParseError()) {
      document["data"] = rapidjson::Value(data, allocator);
    } else {
      add_string(document["data"], allocator, "raw", data_json);
    }
    add_string(document["metadata"], allocator, "push_reason", push_reason);
    return document_to_json(document);
  }

  std::string build_initial_navigation_status_message(const std::string & client_id) const
  {
    rapidjson::Document document;
    document.Parse(build_base_message("push", "navigation_status", client_id).c_str());
    auto & allocator = document.GetAllocator();
    add_string(document["data"], allocator, "navigation_mode", current_navigation_mode_);
    document["data"].AddMember("current_waypoint_index", current_waypoint_index_, allocator);
    add_string(document["metadata"], allocator, "push_reason", "initial_data");
    return document_to_json(document);
  }

  void publish_unsubscribe_for_client(const std::string & client_id, const std::vector<std::string> & data_types)
  {
    rapidjson::Document document;
    document.SetObject();
    auto & allocator = document.GetAllocator();
    add_string(document, allocator, "protocol_version", "2.0");
    add_string(document, allocator, "message_id", generate_message_id("unsubscribe", now_sec()));
    document.AddMember("timestamp", now_sec(), allocator);
    add_string(document, allocator, "message_type", "subscription");
    add_string(document, allocator, "data_type", "subscription_manage");
    add_string(document, allocator, "source", client_id);
    add_string(document, allocator, "destination", "data_integration");
    rapidjson::Value data(rapidjson::kObjectType);
    add_string(data, allocator, "action", "unsubscribe");
    rapidjson::Value types(rapidjson::kArrayType);
    for (const auto & data_type : data_types) {
      types.PushBack(rapidjson::Value(data_type.c_str(), allocator).Move(), allocator);
    }
    data.AddMember("data_types", types, allocator);
    document.AddMember("data", data, allocator);
    data_subscription_pub_->publish(make_string_msg(document_to_json(document)));
  }

  std::vector<std::string> connected_clients() const
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    std::vector<std::string> clients;
    for (const auto & [client_id, client] : clients_) {
      (void)client;
      clients.push_back(client_id);
    }
    return clients;
  }

  bool send_to_client(const std::string & client_id, const std::string & json)
  {
    std::shared_ptr<ClientConnection> client;
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      const auto it = clients_.find(client_id);
      if (it == clients_.end()) {
        return false;
      }
      client = it->second;
    }
    try {
      std::lock_guard<std::mutex> write_lock(client->write_mutex);
      client->ws->write(net::buffer(json));
      return true;
    } catch (const std::exception &) {
      cleanup_client(client_id);
      return false;
    }
  }

  void broadcast(const std::string & json)
  {
    const auto clients = connected_clients();
    for (const auto & client_id : clients) {
      send_to_client(client_id, json);
    }
  }

  void client_health_check_tick()
  {
    if (!config_.websocket_server_enable) {
      return;
    }
    const double max_idle_sec = config_.client_health_check_interval_sec * 3.0;
    const auto inactive_clients = client_registry_.find_inactive_clients(now_sec(), max_idle_sec);
    for (const auto & client_id : inactive_clients) {
      RCLCPP_WARN(get_logger(), "APP 客户端长时间无活动，清理连接: %s", client_id.c_str());
      cleanup_client(client_id);
    }
  }

  void status_report_tick()
  {
    if (!config_.websocket_server_enable) {
      return;
    }
    std::size_t cached_business_count = 0;
    {
      std::lock_guard<std::mutex> lock(business_state_mutex_);
      cached_business_count = business_state_.size();
    }
    RCLCPP_INFO(
      get_logger(),
      "APP 网关状态：clients=%zu subscriptions=%zu cached_business=%zu",
      client_registry_.client_count(),
      client_registry_.subscription_count(),
      cached_business_count);
  }

  std_msgs::msg::String make_string_msg(const std::string & data) const
  {
    std_msgs::msg::String msg;
    msg.data = data;
    return msg;
  }

  std::string build_base_message(
    const std::string & message_type,
    const std::string & data_type,
    const std::string & destination) const
  {
    rapidjson::Document document;
    document.SetObject();
    auto & allocator = document.GetAllocator();
    const double timestamp = now_sec();
    add_string(document, allocator, "protocol_version", "2.0");
    add_string(document, allocator, "message_id", generate_message_id(message_type, timestamp));
    document.AddMember("timestamp", timestamp, allocator);
    add_string(document, allocator, "message_type", message_type);
    add_string(document, allocator, "data_type", data_type);
    add_string(document, allocator, "source", "websocket_server");
    add_string(document, allocator, "destination", destination);
    rapidjson::Value data(rapidjson::kObjectType);
    document.AddMember("data", data, allocator);
    rapidjson::Value metadata(rapidjson::kObjectType);
    add_string(metadata, allocator, "status", "success");
    add_string(metadata, allocator, "error_code", "");
    add_string(metadata, allocator, "error_message", "");
    add_string(metadata, allocator, "request_id", "");
    metadata.AddMember("data_freshness", 0.0, allocator);
    add_string(metadata, allocator, "qos_level", "standard");
    document.AddMember("metadata", metadata, allocator);
    return document_to_json(document);
  }

  std::string build_connection_ack(const std::string & client_id) const
  {
    rapidjson::Document document;
    document.Parse(build_base_message("response", "connection_ack", client_id).c_str());
    auto & allocator = document.GetAllocator();
    add_string(document["data"], allocator, "status", "connected");
    document["data"].AddMember("client_id", rapidjson::Value(client_id.c_str(), allocator).Move(), allocator);
    document["data"].AddMember("server_time", now_sec(), allocator);
    add_string(document["data"], allocator, "protocol_version", "2.0");
    add_string(document["data"], allocator, "supported_protocol_version", "2.0");

    rapidjson::Value supported_data_types(rapidjson::kArrayType);
    for (const char * item : {
        "robot_pose",
        "robot_speed",
        "navigation_path",
        "navigation_status",
        "system_status",
        "waypoints_data",
        "waypoint_response",
        "map_status",
        "map_response",
        "gesture_list",
        "facial_gesture_list",
        "action_result",
      })
    {
      supported_data_types.PushBack(rapidjson::Value(item, allocator).Move(), allocator);
    }
    document["data"].AddMember("supported_data_types", supported_data_types, allocator);

    rapidjson::Value supported_commands(rapidjson::kArrayType);
    for (const char * item : {
        "waypoint_management",
        "navigation_control",
        "map_management",
        "robot_control",
        "facial_control",
        "system_command",
        "initial_pose",
      })
    {
      supported_commands.PushBack(rapidjson::Value(item, allocator).Move(), allocator);
    }
    document["data"].AddMember("supported_commands", supported_commands, allocator);
    document["data"].AddMember("subscription_supported", true, allocator);
    document["data"].AddMember("heartbeat_interval", 30.0, allocator);
    add_string(document["metadata"], allocator, "message", "WebSocket连接成功");
    return document_to_json(document);
  }

  std::string build_simple_ack(
    const std::string & data_type,
    const std::string & command_type,
    const std::string & status,
    const std::string & message,
    const std::string & client_id,
    const std::string & request_id = "") const
  {
    rapidjson::Document document;
    document.Parse(build_base_message("response", data_type, client_id).c_str());
    auto & allocator = document.GetAllocator();
    add_string(document["data"], allocator, "command_type", command_type);
    add_string(document["data"], allocator, "status", status);
    add_string(document["data"], allocator, "message", message);
    if (!request_id.empty()) {
      add_string(document["data"], allocator, "request_id", request_id);
      document["metadata"]["request_id"].SetString(request_id.c_str(), allocator);
    }
    return document_to_json(document);
  }

  std::string wrap_ack_payload(
    const std::string & data_type,
    const std::string & payload_json,
    const std::string & client_id,
    const std::string & request_id = "") const
  {
    rapidjson::Document payload;
    payload.Parse(payload_json.c_str());
    rapidjson::Document document;
    document.Parse(build_base_message("response", data_type, client_id).c_str());
    if (!payload.HasParseError() && payload.IsObject()) {
      document["data"] = rapidjson::Value(payload, document.GetAllocator());
    }
    if (!request_id.empty()) {
      document["metadata"]["request_id"].SetString(request_id.c_str(), document.GetAllocator());
    }
    return document_to_json(document);
  }

  std::string build_error(
    const std::string & data_type,
    const std::string & error_code,
    const std::string & error_message,
    const std::string & client_id) const
  {
    rapidjson::Document document;
    document.Parse(build_base_message("response", data_type, client_id).c_str());
    auto & allocator = document.GetAllocator();
    document["metadata"]["status"].SetString("error", allocator);
    document["metadata"]["error_code"].SetString(error_code.c_str(), allocator);
    document["metadata"]["error_message"].SetString(error_message.c_str(), allocator);
    add_string(document["data"], allocator, "error_code", error_code);
    add_string(document["data"], allocator, "error_message", error_message);
    add_string(document["data"], allocator, "message", error_message);
    document["data"].AddMember("timestamp", now_sec(), allocator);
    return document_to_json(document);
  }

  double now_sec() const
  {
    return static_cast<double>(now().nanoseconds()) / 1e9;
  }

  AppGatewayConfig config_;
  AppProtocol app_protocol_;
  ClientRegistry client_registry_;
  IntegrationForwarder integration_forwarder_;
  BusinessCommandRouter business_router_;

  net::io_context ioc_;
  tcp::acceptor acceptor_;
  std::atomic_bool running_{false};
  std::thread server_thread_;
  mutable std::mutex clients_mutex_;
  std::map<std::string, std::shared_ptr<ClientConnection>> clients_;
  std::mutex business_state_mutex_;
  std::map<std::string, std::string> business_state_;
  std::string current_navigation_mode_;
  int current_waypoint_index_{0};
  rclcpp::TimerBase::SharedPtr client_health_timer_;
  rclcpp::TimerBase::SharedPtr status_report_timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr data_subscription_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_ack_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_control_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr facial_raw_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Client<humanoid_interfaces::srv::SetBroadcastVolume>::SharedPtr broadcast_volume_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr data_response_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr push_message_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_response_sub_;
};

}  // namespace humanoid_app_gateway_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_app_gateway_runtime::AppGatewayNode>());
  rclcpp::shutdown();
  return 0;
}
