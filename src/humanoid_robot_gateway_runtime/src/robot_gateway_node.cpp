/*
 * robot_gateway_node.cpp
 *
 * 文件用途：
 * 1. 机器人本体网关 C++ 节点外壳，负责把内部模块串成 ROS 输入输出链路。
 * 2. 当前已接入低风险状态链路：机器人 WebSocket -> notify_robot_info/notify_joy_data -> ROS topic。
 * 3. 速度发送、动作执行、动作库真实写入仍受 YAML 开关保护，默认不会控制机器人。
 * 4. 上游：机器人本体 WebSocket 服务，以及后续的 /cmd_vel、/app/robot_control。
 * 5. 下游：/robot_status_raw、/joy_raw、/robot/action_result、/system/gesture_list_updated。
 *
 * 代码块顺序：
 * 1. 匿名工具函数：机器人状态字段提取、Walk/Menu 判断和动作命令 JSON 生成。
 * 2. 节点初始化：加载参数、动作时长、publisher/subscriber、WebSocket 客户端和定时器。
 * 3. 机器人消息入口：按 title/route 分发状态、摇杆、动作通知和速度错误。
 * 4. 行走速度链路：/cmd_vel 缓存、周期 tick、状态门控、WebSocket 发送。
 * 5. 动作库同步链路：连接稳定后拉取机器人动作库、写 YAML、通知数据整合热重载。
 * 6. APP 动作命令链路：解析 execute_gesture、Menu/Walk 切换、等待动作完成、发布结果。
 * 7. 通用辅助：文件读写、动作时长加载、机器人状态等待和动作通知等待。
 */

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "humanoid_robot_gateway_runtime/action_result_builder.hpp"
#include "humanoid_robot_gateway_runtime/gesture_sync.hpp"
#include "humanoid_robot_gateway_runtime/motion_controller.hpp"
#include "humanoid_robot_gateway_runtime/robot_gateway_config.hpp"
#include "humanoid_robot_gateway_runtime/robot_gateway_types.hpp"
#include "humanoid_robot_gateway_runtime/robot_status_parser.hpp"
#include "humanoid_robot_gateway_runtime/robot_ws_client.hpp"
#include "humanoid_robot_gateway_runtime/walk_velocity_controller.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "yaml-cpp/yaml.h"

namespace humanoid_robot_gateway_runtime
{

namespace
{

std::string json_value_to_string(const rapidjson::Value & value)
{
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
    return std::to_string(value.GetDouble());
  }
  return "";
}

std::string uppercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  return value;
}

bool robot_state_is(const std::string & robot_state, const std::string & expected)
{
  return uppercase(robot_state) == uppercase(expected);
}

bool is_walk_state(const std::string & robot_state)
{
  return robot_state_is(robot_state, "Walk");
}

bool is_menu_state(const std::string & robot_state)
{
  return robot_state_is(robot_state, "Menu");
}

std::string extract_robot_state_from_notify_info(
  const std::string & data_json,
  const std::string & fallback)
{
  rapidjson::Document data;
  data.Parse(data_json.c_str());
  if (data.HasParseError() || !data.IsObject() ||
    !data.HasMember("result") || !data["result"].IsArray())
  {
    return fallback;
  }

  const auto & results = data["result"];
  for (rapidjson::SizeType i = 0; i < results.Size(); ++i) {
    const auto & component = results[i];
    if (!component.IsObject() || !component.HasMember("values") || !component["values"].IsArray()) {
      continue;
    }
    const auto & values = component["values"];
    for (rapidjson::SizeType j = 0; j < values.Size(); ++j) {
      const auto & item = values[j];
      if (!item.IsObject() || !item.HasMember("key") || !item.HasMember("value")) {
        continue;
      }
      if (json_value_to_string(item["key"]) == "robot_status") {
        const auto state = json_value_to_string(item["value"]);
        return state.empty() ? fallback : state;
      }
    }
  }
  return fallback;
}

std::string get_json_field_string(const std::string & json, const char * key)
{
  rapidjson::Document document;
  document.Parse(json.c_str());
  if (document.HasParseError()) {
    return "";
  }
  if (document.IsObject() && document.HasMember(key)) {
    return json_value_to_string(document[key]);
  }
  if (document.IsObject() && document.HasMember("data") && document["data"].IsObject() &&
    document["data"].HasMember(key))
  {
    return json_value_to_string(document["data"][key]);
  }
  return "";
}

std::string rapidjson_value_to_json(const rapidjson::Value & value)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  value.Accept(writer);
  return buffer.GetString();
}

std::string motion_engine_mode_json(const int mode)
{
  return std::string("{\"mode\":") + std::to_string(mode) + "}";
}

std::string execute_motion_json(const std::string & motion_name)
{
  return std::string("{\"motion_name\":\"") + motion_name + "\"}";
}

}  // namespace

class RobotGatewayNode : public rclcpp::Node
{
public:
  explicit RobotGatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_gateway_node", options),
    walk_controller_(config_),
    motion_controller_(config_)
  {
    declare_robot_gateway_parameters(*this, config_);
    config_ = load_robot_gateway_config(*this, config_);
    walk_controller_ = WalkVelocityController(config_);
    motion_controller_ = MotionController(config_);

    // 动作时长用于计算“动作完成等待超时”和“整条任务超时”。
    // 找不到配置时仍可使用默认超时，避免动作库缺失导致节点无法启动。
    load_motion_expected_durations(resolve_gestures_yaml_path());

    // ROS 输出边界：机器人状态、摇杆、动作结果、动作库热重载通知都从这里发布。
    robot_status_pub_ = create_publisher<std_msgs::msg::String>(config_.robot_status_raw_topic, 10);
    joy_pub_ = create_publisher<sensor_msgs::msg::Joy>(config_.joy_raw_topic, 10);
    action_result_pub_ = create_publisher<std_msgs::msg::String>(config_.robot_action_result_topic, 10);
    gesture_update_pub_ = create_publisher<std_msgs::msg::String>(config_.gesture_list_updated_topic, 10);
    action_interlock_client_ = create_client<std_srvs::srv::SetBool>(
      config_.navigation_action_interlock_service);

    // WebSocket 客户端只负责连接和收发原始机器人协议。
    // 所有业务分发都回到 handle_robot_ws_event()，避免网络层直接发布 ROS topic。
    ws_client_ = std::make_unique<RobotWsClient>(config_);
    ws_client_->set_message_callback([this](const RobotWsIncomingEvent & event) {
      handle_robot_ws_event(event);
    });

    // /cmd_vel 和 /app/robot_control 是机器人控制入口。
    // 真实发送/真实动作执行仍受 walk_velocity_send_enable 和 motion_execution_enable 保护。
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      config_.cmd_vel_topic,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_callback(*msg);
      });
    app_robot_control_sub_ = create_subscription<std_msgs::msg::String>(
      config_.app_robot_control_topic,
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        robot_control_callback(*msg);
      });
    walk_timer_ = create_wall_timer(
      std::chrono::duration<double>(walk_controller_.send_interval_sec()),
      [this]() { walk_velocity_tick(); });
    gesture_sync_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { maybe_start_gesture_sync(); });

    // 只有 robot_ws_enable=true 才连接真实机器人；正式导航默认连接，离线验证可关闭。
    if (config_.robot_ws_enable) {
      ws_client_->start();
    }

    RCLCPP_WARN(
      get_logger(),
      "robot_gateway_node 已启动：robot_ws_enable=%s，walk_velocity_send_enable=%s，motion_execution_enable=%s。",
      config_.robot_ws_enable ? "true" : "false",
      config_.walk_velocity_send_enable ? "true" : "false",
      config_.motion_execution_enable ? "true" : "false");
  }

  ~RobotGatewayNode() override
  {
    if (ws_client_) {
      ws_client_->stop();
    }
  }

private:
  void handle_robot_ws_event(const RobotWsIncomingEvent & event)
  {
    // 机器人 WebSocket 统一入口：
    // notify_robot_info -> /robot_status_raw；
    // notify_joy_data -> /joy_raw；
    // notify_execute_atomic_motion -> 唤醒动作等待；
    // response_set_walk_vel_sync -> 输出速度错误告警。
    if (!event.ok) {
      RCLCPP_WARN(get_logger(), "机器人 WebSocket 消息解析失败: %s", event.error.c_str());
      return;
    }

    if (event.route == "notify_robot_info") {
      publish_robot_status(event.data_json);
    } else if (event.route == "notify_joy_data") {
      publish_joy(event.data_json);
    } else if (event.route == "notify_execute_atomic_motion") {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_motion_notify_json_ = event.data_json;
      current_motion_result_ = get_json_field_string(event.data_json, "result");
      motion_cv_.notify_all();
    } else if (event.route == "response_set_walk_vel_sync" && !event.walk_velocity_message.empty()) {
      RCLCPP_WARN(get_logger(), "%s", event.walk_velocity_message.c_str());
    }
  }

  void publish_robot_status(const std::string & data_json)
  {
    // 状态发布链路会更新 robot_state_、消息间隔统计和动作等待条件。
    // parse_notify_robot_info() 负责把机器人原始 result/values 转成 /robot_status_raw JSON。
    const double current_time = now_sec();
    double previous_time = current_time;
    std::vector<double> previous_intervals;
    std::string robot_state;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      robot_state_ = extract_robot_state_from_notify_info(data_json, robot_state_);
      robot_state = robot_state_;
      previous_time = last_robot_status_time_sec_ > 0.0 ? last_robot_status_time_sec_ : current_time;
      previous_intervals = msg_intervals_ms_;
    }

    const auto parsed = status_parser_.parse_notify_robot_info(
      data_json,
      ws_client_->accid(),
      ws_client_->sn(),
      ws_client_->identity_source(),
      robot_state,
      is_executing_motion_,
      current_motion_name_,
      current_time,
      previous_time,
      previous_intervals);

    if (!parsed.ok) {
      RCLCPP_WARN(get_logger(), "notify_robot_info 解析失败: %s", parsed.error.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const double interval_ms = (current_time - previous_time) * 1000.0;
      msg_intervals_ms_.push_back(interval_ms);
      if (msg_intervals_ms_.size() > 20) {
        msg_intervals_ms_.erase(msg_intervals_ms_.begin());
      }
      last_robot_status_time_sec_ = current_time;
      motion_cv_.notify_all();
    }

    std_msgs::msg::String msg;
    msg.data = parsed.robot_status_raw_json;
    robot_status_pub_->publish(msg);
  }

  void publish_joy(const std::string & data_json)
  {
    // 摇杆数据只做格式转换，不参与导航控制决策。
    // 下游如果需要遥控接管，可订阅 /joy_raw 自行判断。
    const auto parsed = status_parser_.parse_notify_joy_data(data_json);
    if (!parsed.ok) {
      RCLCPP_WARN(get_logger(), "notify_joy_data 解析失败: %s", parsed.error.c_str());
      return;
    }

    sensor_msgs::msg::Joy msg;
    msg.header.stamp = now();
    msg.header.frame_id = parsed.frame_id;
    msg.axes.reserve(parsed.axes.size());
    for (const auto axis : parsed.axes) {
      msg.axes.push_back(static_cast<float>(axis));
    }
    msg.buttons = parsed.buttons;
    joy_pub_->publish(msg);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    // /cmd_vel 回调只更新目标速度缓存，不在回调线程里直接发 WebSocket。
    // 真实发送由 walk_velocity_tick() 定频执行，保证速度发送频率稳定。
    walk_controller_.on_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z);
  }

  void walk_velocity_tick()
  {
    // 定频速度发送周期：
    // WalkVelocityController 会检查机器人是否 Walk、是否正在执行动作、是否需要超时停车。
    // 如果开关未打开或门控不通过，本周期不会发任何底层速度命令。
    std::string robot_state;
    bool is_executing_motion = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      robot_state = robot_state_;
      is_executing_motion = is_executing_motion_;
    }

    const auto decision = walk_controller_.evaluate_tick(robot_state, is_executing_motion);
    if (!decision.should_send) {
      return;
    }
    const auto sent = ws_client_->send_command_no_response(decision.title, decision.data_json);
    if (!sent.ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "行走速度发送失败: %s",
        sent.error.c_str());
    }
  }

  void maybe_start_gesture_sync()
  {
    // 动作库同步是一次性任务：等待 WebSocket 连接稳定一小段时间后再拉取。
    // 这样可以避开连接刚建立时机器人本体服务尚未准备好的窗口。
    if (!config_.gesture_sync_enable || !config_.robot_ws_enable || gesture_sync_started_) {
      if (gesture_sync_timer_) {
        gesture_sync_timer_->cancel();
      }
      return;
    }
    if (!ws_client_ || !ws_client_->connected()) {
      first_connected_time_sec_ = 0.0;
      return;
    }
    if (first_connected_time_sec_ <= 0.0) {
      first_connected_time_sec_ = now_sec();
      return;
    }
    if (now_sec() - first_connected_time_sec_ < std::max(0.0, config_.gesture_sync_delay_sec)) {
      return;
    }

    gesture_sync_started_ = true;
    if (gesture_sync_timer_) {
      gesture_sync_timer_->cancel();
    }
    std::thread([this]() { run_gesture_sync_once(); }).detach();
  }

  void run_gesture_sync_once()
  {
    // 动作库同步执行流：
    // 1. 请求机器人动作列表；
    // 2. 解析动作条目；
    // 3. 生成并写入 gestures.yaml；
    // 4. 重新加载动作时长；
    // 5. 通知 data_integration_node 热重载 APP 动作列表。
    RCLCPP_INFO(get_logger(), "正在从机器人拉取动作库列表...");
    const auto response = ws_client_->send_command(
      "request_get_atomic_motion_list",
      "{}",
      config_.command_timeout_sec);
    if (!response.ok) {
      RCLCPP_WARN(get_logger(), "获取动作库列表失败: %s，保留现有动作库文件。", response.error.c_str());
      return;
    }

    const std::string data_json = extract_response_data_json(response.response_json);
    const std::string result = get_json_field_string(data_json, "result");
    if (!result.empty() && result != "success") {
      RCLCPP_WARN(get_logger(), "获取动作库列表返回异常 result=%s，保留现有动作库文件。", result.c_str());
      return;
    }

    const auto motion_items = gesture_sync_.extract_motion_items(data_json);
    if (motion_items.empty()) {
      RCLCPP_WARN(get_logger(), "未能从机器人响应中解析出动作列表，保留现有动作库文件。");
      return;
    }

    const std::string yaml_path = resolve_gestures_yaml_path();
    const auto yaml_result = gesture_sync_.build_gestures_yaml(read_text_file(yaml_path), motion_items);
    if (!yaml_result.ok) {
      RCLCPP_WARN(get_logger(), "生成动作库 YAML 失败: %s，保留现有动作库文件。", yaml_result.error.c_str());
      return;
    }
    if (!write_text_file(yaml_path, yaml_result.text)) {
      RCLCPP_ERROR(get_logger(), "写入动作库 YAML 失败: %s", yaml_path.c_str());
      return;
    }

    load_motion_expected_durations(yaml_path);
    const auto reload = gesture_sync_.build_reload_message(now_sec());
    if (reload.ok) {
      std_msgs::msg::String msg;
      msg.data = reload.text;
      gesture_update_pub_->publish(msg);
    }
    RCLCPP_INFO(get_logger(), "已同步机器人动作库 %zu 项，并通知数据整合节点热重载。", motion_items.size());
  }

  void robot_control_callback(const std_msgs::msg::String & msg)
  {
    // APP 动作命令入口目前只处理 execute_gesture。
    // parse/evaluate 阶段会先做参数、忙碌、机器人状态和功能开关门控。
    const auto parsed = motion_controller_.parse_robot_control_command(msg.data, now_sec());
    if (!parsed.ok) {
      RCLCPP_WARN(get_logger(), "机器人控制命令解析失败: %s", parsed.reason.c_str());
      return;
    }
    std::string robot_state;
    bool executing = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      robot_state = robot_state_;
      executing = is_executing_motion_;
    }
    const auto decision = motion_controller_.evaluate_start_request(parsed, executing, robot_state);
    if (decision.should_publish_result) {
      publish_action_result_from_decision(decision, now_sec(), now_sec());
      return;
    }
    if (!decision.accepted) {
      return;
    }
    if (!config_.motion_execution_enable) {
      MotionStartDecision disabled = decision;
      disabled.should_publish_result = true;
      disabled.status = "rejected";
      disabled.result_code = "motion_execution_disabled";
      disabled.message = "机器人动作真实执行开关未开启，已安全拒绝";
      disabled.walk_ready = is_walk_state(robot_state);
      publish_action_result_from_decision(disabled, now_sec(), now_sec());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      is_executing_motion_ = true;
      current_motion_name_ = decision.motion_name;
      current_motion_result_.clear();
      current_motion_notify_json_.clear();
    }
    std::thread([this, decision]() { execute_motion_task(decision); }).detach();
  }

  void execute_motion_task(const MotionStartDecision decision)
  {
    // 动作真实执行在线程中完成，避免阻塞 ROS 回调线程。
    // 执行顺序是：必要时进 Menu -> 下发动作 -> 等待动作完成通知 -> 必要时回 Walk -> 发布结果。
    const double started_at = now_sec();
    std::string final_status = "failed";
    std::string result_code = "unknown";
    std::string result_message = "动作未完成";
    std::string notify_json = "{}";
    std::string interlock_message;
    const bool interlock_acquired = set_navigation_action_interlock(true, &interlock_message);
    if (!interlock_acquired) {
      std::string rollback_message;
      set_navigation_action_interlock(false, &rollback_message);
      finish_motion_result(
        decision,
        started_at,
        "rejected",
        "navigation_interlock_failed",
        interlock_message.empty() ? "导航动作互锁服务不可用" : interlock_message,
        notify_json,
        is_walk_state(current_robot_state()));
      return;
    }

    auto finish_and_release =
      [this, &decision, started_at, &notify_json, interlock_acquired](
      const std::string & status,
      const std::string & code,
      const std::string & message) {
        bool ready = ensure_walk_mode_after_motion();
        finish_motion_result(
          decision, started_at, status, code, message, notify_json, ready);
        if (interlock_acquired && ready) {
          std::string release_message;
          if (!set_navigation_action_interlock(false, &release_message)) {
            RCLCPP_ERROR(
              get_logger(), "动作完成后释放导航互锁失败: %s", release_message.c_str());
          }
        } else if (interlock_acquired) {
          RCLCPP_ERROR(
            get_logger(), "动作结束但机器人未确认回到 Walk，保持导航动作互锁");
        }
      };

    try {
      std::string state = current_robot_state();
      if (!is_menu_state(state)) {
        if (!config_.motion_allow_enter_menu) {
          result_code = "enter_menu_disabled";
          result_message = "动作执行需要切换 Menu，但 motion_allow_enter_menu 未开启";
          finish_and_release(final_status, result_code, result_message);
          return;
        }
        const auto mode_result = ws_client_->send_command(
          "request_set_motion_engine",
          motion_engine_mode_json(1),
          config_.command_timeout_sec);
        if (!mode_result.ok || get_json_field_string(mode_result.response_json, "result") != "success") {
          result_code = mode_result.ok ? "enter_menu_failed" : mode_result.error;
          result_message = "切换动作库模式失败";
          finish_and_release(final_status, result_code, result_message);
          return;
        }
        if (!wait_for_robot_state("Menu", 3.0)) {
          result_code = "enter_menu_timeout";
          result_message = "未成功进入动作库模式";
          finish_and_release(final_status, result_code, result_message);
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      }

      auto exec_result = ws_client_->send_command(
        "request_execute_atomic_motion",
        execute_motion_json(decision.motion_name),
        config_.command_timeout_sec);
      std::string result = get_json_field_string(exec_result.response_json, "result");
      if (exec_result.ok && result == "fail_invalid_mode") {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        exec_result = ws_client_->send_command(
          "request_execute_atomic_motion",
          execute_motion_json(decision.motion_name),
          config_.command_timeout_sec);
        result = get_json_field_string(exec_result.response_json, "result");
      }
      if (!exec_result.ok || result != "success") {
        result_code = exec_result.ok ? (result.empty() ? "dispatch_failed" : result) : exec_result.error;
        result_message = "动作下发失败: " + result_code;
        finish_and_release(final_status, result_code, result_message);
        return;
      }

      if (wait_for_motion_notify(decision.motion_timeout_sec, &notify_json, &result)) {
        if (result.empty() || result == "success") {
          final_status = "success";
          result_code = "success";
          result_message = "动作执行完成";
        } else {
          final_status = "failed";
          result_code = result;
          result_message = "动作底层反馈异常: " + result;
        }
      } else {
        final_status = "timeout";
        result_code = "motion_completion_timeout";
        result_message = "等待动作完成通知超时";
      }

    } catch (const std::exception & ex) {
      final_status = "failed";
      result_code = "motion_task_exception";
      result_message = ex.what();
    }

    finish_and_release(final_status, result_code, result_message);
  }

  bool ensure_walk_mode_after_motion()
  {
    if (is_walk_state(current_robot_state())) {
      return true;
    }
    if (!config_.motion_allow_return_walk) {
      return false;
    }
    const auto walk_result = ws_client_->send_command(
      "request_set_motion_engine",
      motion_engine_mode_json(0),
      config_.command_timeout_sec);
    if (!walk_result.ok || get_json_field_string(walk_result.response_json, "result") != "success") {
      return false;
    }
    return wait_for_robot_state("Walk", 3.0);
  }

  bool set_navigation_action_interlock(const bool enabled, std::string * message)
  {
    if (!action_interlock_client_->wait_for_service(std::chrono::seconds(1))) {
      if (message != nullptr) {
        *message = "导航动作互锁服务不可用";
      }
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto future = action_interlock_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(4)) != std::future_status::ready) {
      if (message != nullptr) {
        *message = "等待导航动作互锁响应超时";
      }
      return false;
    }
    try {
      const auto response = future.get();
      if (message != nullptr) {
        *message = response ? response->message : "导航动作互锁无响应";
      }
      return response && response->success;
    } catch (const std::exception & ex) {
      if (message != nullptr) {
        *message = std::string("导航动作互锁调用异常: ") + ex.what();
      }
      return false;
    }
  }

  void finish_motion_result(
    const MotionStartDecision & decision,
    const double started_at,
    const std::string & status,
    const std::string & result_code,
    const std::string & message,
    const std::string & notify_json,
    bool walk_ready)
  {
    // 动作结束统一收口：先清理“正在执行动作”的内部状态，再发布 /robot/action_result。
    // walk_ready 会结合最新 robot_state_ 二次确认，避免误报已经可行走。
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      is_executing_motion_ = false;
      current_motion_name_.clear();
      walk_ready = walk_ready && is_walk_state(robot_state_);
    }
    publish_action_result(
      decision.motion_name,
      status,
      result_code,
      message,
      started_at,
      now_sec(),
      decision.client_id,
      decision.command_timestamp_json,
      notify_json,
      walk_ready);
  }

  void publish_action_result_from_decision(
    const MotionStartDecision & decision,
    const double started_at,
    const double completed_at)
  {
    // 入口门控阶段直接拒绝/失败的命令会走这里，不需要启动动作线程。
    publish_action_result(
      decision.motion_name,
      decision.status,
      decision.result_code,
      decision.message,
      started_at,
      completed_at,
      decision.client_id,
      decision.command_timestamp_json,
      "{}",
      decision.walk_ready);
  }

  void publish_action_result(
    const std::string & motion_name,
    const std::string & status,
    const std::string & result_code,
    const std::string & message,
    const double started_at,
    const double completed_at,
    const std::string & client_id,
    const std::string & command_timestamp_json,
    const std::string & notify_json,
    const bool walk_ready)
  {
    // 统一动作结果发布出口，字段组包交给 ActionResultBuilder。
    // data_integration_node 会订阅该 topic 并转换成 APP 事件和可能的 system_exception。
    const auto built = action_result_builder_.build_action_result(
      motion_name,
      status,
      result_code,
      message,
      started_at,
      completed_at,
      client_id,
      command_timestamp_json,
      notify_json,
      current_robot_state(),
      is_motion_executing(),
      walk_ready);
    if (!built.ok) {
      RCLCPP_WARN(get_logger(), "动作结果组包失败: %s", built.error.c_str());
      return;
    }
    std_msgs::msg::String out;
    out.data = built.json;
    action_result_pub_->publish(out);
  }

  std::string current_robot_state()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return robot_state_;
  }

  bool is_motion_executing()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return is_executing_motion_;
  }

  std::string resolve_gestures_yaml_path() const
  {
    // 优先使用 YAML 配置的动作库路径；未配置时定位表情/动作运行包内默认路径。
    if (!config_.gestures_yaml_path.empty()) {
      return config_.gestures_yaml_path;
    }
    try {
      return ament_index_cpp::get_package_share_directory("humanoid_expression_runtime") + "/config/gestures.yaml";
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "无法自动定位 gestures.yaml: %s", ex.what());
      return "";
    }
  }

  std::string extract_response_data_json(const std::string & response_json) const
  {
    rapidjson::Document response;
    response.Parse(response_json.c_str());
    if (response.HasParseError() || !response.IsObject() || !response.HasMember("data")) {
      return "{}";
    }
    return rapidjson_value_to_json(response["data"]);
  }

  std::string read_text_file(const std::string & path) const
  {
    if (path.empty()) {
      return "";
    }
    std::ifstream input(path);
    if (!input.is_open()) {
      return "";
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
  }

  bool write_text_file(const std::string & path, const std::string & text) const
  {
    if (path.empty()) {
      return false;
    }
    std::ofstream output(path, std::ios::trunc);
    if (!output.is_open()) {
      return false;
    }
    output << text;
    return output.good();
  }

  void load_motion_expected_durations(const std::string & yaml_path)
  {
    // 动作时长是可选增强信息：用于更准确地设置等待完成超时。
    // 解析失败不会影响节点运行，只会回到默认超时参数。
    motion_controller_.clear_motion_expected_durations();
    motion_controller_.replace_available_motion_names({});
    if (yaml_path.empty()) {
      return;
    }
    try {
      const YAML::Node config = YAML::LoadFile(yaml_path);
      const YAML::Node actions = config["actions"];
      if (!actions || !actions.IsMap()) {
        return;
      }
      std::size_t loaded = 0;
      std::set<std::string> available_motion_names;
      for (const auto & action : actions) {
        const std::string motion_name = action.first.as<std::string>();
        if (!motion_name.empty()) {
          available_motion_names.insert(motion_name);
        }
        const auto duration = extract_motion_duration(motion_name, action.second);
        if (duration > 0.0) {
          motion_controller_.set_motion_expected_duration(motion_name, duration);
          ++loaded;
        }
      }
      motion_controller_.replace_available_motion_names(available_motion_names);
      if (loaded > 0) {
        RCLCPP_INFO(get_logger(), "已加载 %zu 个动作时长提示。", loaded);
      }
      RCLCPP_INFO(
        get_logger(), "已加载机器人 OTA 动作库，共 %zu 个有效动作。",
        motion_controller_.available_motion_count());
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "解析动作库时长失败: %s", ex.what());
    }
  }

  double extract_motion_duration(const std::string & motion_name, const YAML::Node & entry) const
  {
    // 支持显式 duration 字段，也支持从动作名/描述中提取“3s/3秒”这类时长提示。
    if (!entry || !entry.IsMap()) {
      return 0.0;
    }
    for (const auto & key : {"duration_sec", "duration", "expected_duration_sec"}) {
      const YAML::Node value = entry[key];
      if (value && value.IsScalar()) {
        try {
          const double duration = value.as<double>();
          if (duration > 0.0) {
            return duration;
          }
        } catch (const std::exception &) {
        }
      }
    }

    const std::vector<std::string> texts = {
      motion_name,
      entry["name"] ? entry["name"].as<std::string>() : "",
      entry["description"] ? entry["description"].as<std::string>() : ""};
    const std::regex duration_regex(R"((\d+(?:\.\d+)?)\s*(?:s|秒))", std::regex::icase);
    for (const auto & text : texts) {
      std::smatch match;
      if (std::regex_search(text, match, duration_regex) && match.size() > 1) {
        try {
          return std::stod(match[1].str());
        } catch (const std::exception &) {
        }
      }
    }
    return 0.0;
  }

  bool wait_for_robot_state(const std::string & target_state, const double timeout_sec)
  {
    // 等待状态切换依赖 notify_robot_info 更新 robot_state_。
    // 条件变量由状态回调唤醒，超时后再做一次最终状态检查。
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    std::unique_lock<std::mutex> lock(state_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (robot_state_is(robot_state_, target_state)) {
        return true;
      }
      motion_cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    return robot_state_is(robot_state_, target_state);
  }

  bool wait_for_motion_notify(
    const double timeout_sec,
    std::string * notify_json,
    std::string * result)
  {
    // 等待底层 notify_execute_atomic_motion，确认动作真的完成或返回异常结果。
    // 如果超时未收到通知，上层会发布 timeout 结果，避免 APP 一直等待。
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    std::unique_lock<std::mutex> lock(state_mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!current_motion_notify_json_.empty()) {
        if (notify_json != nullptr) {
          *notify_json = current_motion_notify_json_;
        }
        if (result != nullptr) {
          *result = current_motion_result_;
        }
        return true;
      }
      motion_cv_.wait_for(lock, std::chrono::milliseconds(100));
    }
    return false;
  }

  double now_sec() const
  {
    return static_cast<double>(now().nanoseconds()) / 1e9;
  }

  RobotGatewayConfig config_;
  RobotStatusParser status_parser_;
  WalkVelocityController walk_controller_;
  MotionController motion_controller_;
  GestureSync gesture_sync_;
  ActionResultBuilder action_result_builder_;
  std::unique_ptr<RobotWsClient> ws_client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_result_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gesture_update_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr action_interlock_client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr app_robot_control_sub_;
  rclcpp::TimerBase::SharedPtr walk_timer_;
  rclcpp::TimerBase::SharedPtr gesture_sync_timer_;

  std::mutex state_mutex_;
  std::condition_variable motion_cv_;
  std::string robot_state_{"Unknown"};
  bool is_executing_motion_{false};
  std::string current_motion_name_;
  std::string current_motion_notify_json_;
  std::string current_motion_result_;
  double last_robot_status_time_sec_{0.0};
  double first_connected_time_sec_{0.0};
  std::atomic_bool gesture_sync_started_{false};
  std::vector<double> msg_intervals_ms_;
};

}  // namespace humanoid_robot_gateway_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_robot_gateway_runtime::RobotGatewayNode>());
  rclcpp::shutdown();
  return 0;
}
