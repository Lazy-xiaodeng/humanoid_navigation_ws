/*
 * map_context_manager.cpp
 *
 * 文件用途：
 * 1. 这个节点属于“常驻控制层”，负责维护当前地图上下文、响应 APP 地图命令、
 *    发布地图状态、协调切图过程中的初始位姿与定位稳定检查。
 * 2. 节点不直接加载地图、不直接执行导航，也不直接控制底盘；它负责把 APP 的地图意图
 *    整理成稳定的 ROS 状态机流程，并把切图进度持续回传给上层。
 * 3. 对外接口包括：/app/map_command、/map/response、/map/status、
 *    /initialpose、/navigation/status、/localization/prior_map_odom_bridge_status。
 * 4. 切图时控制层必须保持在线，真正会被重启的是导航定位层；因此本节点只调用切图脚本，
 *    不在脚本里终止自身所在的控制层进程。
 * 5. 地图 registry 是现场运行数据，保存 current_map_id 时要保证写回路径指向当前工作区。
 *
 * 本文件按真实数据链路排列代码块：
 * 1. 工具函数、数据结构和配置。
 * 2. 节点初始化：参数、地图注册表、ROS 接口和定时器。
 * 3. APP 地图命令入口：get_map_list、get_current_map、switch_map。
 * 4. 切图状态机：校验、脚本启动、watchdog、定位稳定和超时。
 * 5. 状态输入：导航状态、定位健康状态。
 * 6. 输出链路：/map/response、/map/status、/initialpose。
 * 7. 地图注册表读写和 JSON 工具。
 * 8. main 入口。
 */

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
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

double now_seconds()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
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

std::string json_to_string(const rapidjson::Value & value, const bool pretty = false)
{
  rapidjson::StringBuffer buffer;
  if (pretty) {
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    writer.SetIndent(' ', 2);
    value.Accept(writer);
  } else {
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    value.Accept(writer);
  }
  return buffer.GetString();
}

std::vector<double> read_double_array(
  const rapidjson::Value & object,
  const char * key,
  const std::vector<double> & fallback)
{
  if (!object.HasMember(key) || !object[key].IsArray()) {
    return fallback;
  }
  std::vector<double> values;
  for (const auto & item : object[key].GetArray()) {
    if (item.IsNumber()) {
      values.push_back(item.GetDouble());
    }
  }
  return values.empty() ? fallback : values;
}

struct MapContextConfig
{
  // 地图注册表路径。注册表记录默认地图、当前地图、2D 栅格地图和 3D prior 地图路径。
  std::string map_registry_path{"/home/ubuntu/software/Todesk/Files/humanoid_ws/data/maps/map_registry.json"};

  // 命令没有携带目标地图时使用的默认地图 ID。
  std::string default_map_id{"hall"};

  // 定位健康状态话题。切图后必须连续收到健康状态，才允许进入 ready。
  std::string localization_health_status_topic{"/localization/prior_map_odom_bridge_status"};

  // 初始位姿话题。切图或同地图 reset 时会重复发布 registry 中的 initial_pose。
  std::string initialpose_topic{"/initialpose"};

  // 切图后等待定位恢复的最长时间，单位秒。包含脚本重启和定位稳定耗时。
  double switch_localization_timeout_sec{45.0};

  // 连续多少帧定位健康才认为地图 ready。数值越高越稳，但 ready 响应会更慢。
  int switch_localization_stable_frames{2};

  // 初始位姿重复发布次数，用于降低订阅者刚启动时漏收的概率。
  int initialpose_repeat_count{3};

  // 初始位姿重复发布间隔，单位秒。
  double initialpose_repeat_interval_sec{0.5};

  // 导航状态话题。导航执行、暂停、等待播报期间都禁止切图。
  std::string navigation_status_topic{"/navigation/status"};

  // 切图脚本路径。脚本只允许重启导航定位层，不应该停止控制层本身。
  std::string map_switch_script{"/home/ubuntu/software/Todesk/Files/humanoid_ws/switch_navigation_map.sh"};

  // APP 地图命令入口，承载 get_map_list/get_current_map/switch_map。
  std::string app_map_command_topic{"/app/map_command"};

  // 地图命令响应输出话题。
  std::string map_response_topic{"/map/response"};

  // 地图状态周期推送话题，供 APP 状态栏和调试面板使用。
  std::string map_status_topic{"/map/status"};

  // 地图状态周期推送间隔，单位秒。
  double map_status_publish_interval_sec{5.0};

  // 切图状态机检查间隔，负责推进脚本状态、初始位姿发布、定位 ready 和超时。
  double switch_watchdog_interval_sec{0.5};
};

struct SwitchContext
{
  // APP 请求 ID。最终 map_ready、timeout、failed 响应会沿用该 ID，方便前端关联一次切图。
  std::string request_message_id;

  // 本次切图目标地图。
  std::string target_map_id;

  // 目标地图 registry 条目快照。切图过程中即使 registry 文件变化，也使用启动时快照。
  rapidjson::Document target_map;

  // 切图开始时间，用于 timeout 判定。
  double started_at{0.0};

  // 已发布初始位姿次数。
  int initialpose_sent{0};

  // 上一次发布初始位姿的时间。
  double last_initialpose_time{0.0};

  // 跨地图切换脚本进程 ID；同地图 reset 不启动脚本，保持 -1。
  pid_t switch_pid{-1};
};

class MapContextManagerNode : public rclcpp::Node
{
public:
  explicit MapContextManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("map_context_manager_cpp", options)
  {
    declare_parameters();
    load_parameters();
    maps_.SetArray();
    load_map_registry();
    setup_interfaces();

    RCLCPP_WARN(
      get_logger(),
      "map_context_manager_cpp 已启动：负责地图查询、切图状态机、初始位姿和地图状态发布；"
      "当前作为正式 C++ 地图上下文链路运行。");
  }

private:
  // ===========================================================================
  // 2. 节点初始化：参数、地图注册表、ROS 接口和定时器
  // ===========================================================================

  void declare_parameters()
  {
    declare_parameter<std::string>("map_registry_path", config_.map_registry_path);
    declare_parameter<std::string>("default_map_id", config_.default_map_id);
    declare_parameter<std::string>(
      "localization_health_status_topic", config_.localization_health_status_topic);
    declare_parameter<std::string>("initialpose_topic", config_.initialpose_topic);
    declare_parameter<double>("switch_localization_timeout_sec", config_.switch_localization_timeout_sec);
    declare_parameter<int>("switch_localization_stable_frames", config_.switch_localization_stable_frames);
    declare_parameter<int>("initialpose_repeat_count", config_.initialpose_repeat_count);
    declare_parameter<double>("initialpose_repeat_interval_sec", config_.initialpose_repeat_interval_sec);
    declare_parameter<std::string>("navigation_status_topic", config_.navigation_status_topic);
    declare_parameter<std::string>("map_switch_script", config_.map_switch_script);
    declare_parameter<std::string>("app_map_command_topic", config_.app_map_command_topic);
    declare_parameter<std::string>("map_response_topic", config_.map_response_topic);
    declare_parameter<std::string>("map_status_topic", config_.map_status_topic);
    declare_parameter<double>("map_status_publish_interval_sec", config_.map_status_publish_interval_sec);
    declare_parameter<double>("switch_watchdog_interval_sec", config_.switch_watchdog_interval_sec);
  }

  void load_parameters()
  {
    config_.map_registry_path = expand_user_path(get_parameter("map_registry_path").as_string());
    config_.default_map_id = normalize_map_id(get_parameter("default_map_id").as_string());
    config_.localization_health_status_topic = get_parameter("localization_health_status_topic").as_string();
    config_.initialpose_topic = get_parameter("initialpose_topic").as_string();
    config_.switch_localization_timeout_sec = get_parameter("switch_localization_timeout_sec").as_double();
    config_.switch_localization_stable_frames =
      std::max(1, static_cast<int>(get_parameter("switch_localization_stable_frames").as_int()));
    config_.initialpose_repeat_count =
      std::max(1, static_cast<int>(get_parameter("initialpose_repeat_count").as_int()));
    config_.initialpose_repeat_interval_sec =
      std::max(0.1, get_parameter("initialpose_repeat_interval_sec").as_double());
    config_.navigation_status_topic = get_parameter("navigation_status_topic").as_string();
    config_.map_switch_script = expand_user_path(get_parameter("map_switch_script").as_string());
    config_.app_map_command_topic = get_parameter("app_map_command_topic").as_string();
    config_.map_response_topic = get_parameter("map_response_topic").as_string();
    config_.map_status_topic = get_parameter("map_status_topic").as_string();
    config_.map_status_publish_interval_sec = get_parameter("map_status_publish_interval_sec").as_double();
    config_.switch_watchdog_interval_sec = get_parameter("switch_watchdog_interval_sec").as_double();
    current_map_id_ = config_.default_map_id;
  }

  void setup_interfaces()
  {
    initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      config_.initialpose_topic, rclcpp::QoS(10));
    map_response_pub_ = create_publisher<std_msgs::msg::String>(config_.map_response_topic, rclcpp::QoS(10));
    map_status_pub_ = create_publisher<std_msgs::msg::String>(config_.map_status_topic, rclcpp::QoS(10));

    map_command_sub_ = create_subscription<std_msgs::msg::String>(
      config_.app_map_command_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { on_map_command(msg); });
    navigation_status_sub_ = create_subscription<std_msgs::msg::String>(
      config_.navigation_status_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { on_navigation_status(msg); });
    localization_status_sub_ = create_subscription<std_msgs::msg::String>(
      config_.localization_health_status_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { on_localization_status(msg); });

    switch_watchdog_timer_ = create_wall_timer(
      to_duration(config_.switch_watchdog_interval_sec, 500ms),
      [this]() { switch_watchdog_tick(); });
    map_status_timer_ = create_wall_timer(
      to_duration(config_.map_status_publish_interval_sec, 5s),
      [this]() { publish_map_status(); });
    initial_status_timer_ = create_wall_timer(
      200ms,
      [this]() {
        publish_map_status();
        if (initial_status_timer_) {
          initial_status_timer_->cancel();
          initial_status_timer_.reset();
        }
      });
  }

  // ===========================================================================
  // 3. APP 地图命令入口：get_map_list、get_current_map、switch_map
  // ===========================================================================

  void on_map_command(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    rapidjson::Document command;
    command.Parse(msg->data.c_str());
    if (command.HasParseError() || !command.IsObject()) {
      send_map_response("unknown", "", make_error_payload(
        "invalid_map_command", "地图命令 JSON 解析失败", current_map_id_));
      return;
    }

    const std::string command_type = trim_copy(read_string_member(command, "command_type", ""));
    const std::string request_message_id = read_string_member(command, "request_message_id", "");
    if (command_type == "get_map_list") {
      rapidjson::Document data;
      data.SetObject();
      auto & allocator = data.GetAllocator();
      data.AddMember("status", "success", allocator);
      data.AddMember("current_map_id", rapidjson::Value(current_map_id_.c_str(), allocator).Move(), allocator);
      data.AddMember("default_map_id", rapidjson::Value(config_.default_map_id.c_str(), allocator).Move(), allocator);
      rapidjson::Value maps_copy;
      maps_copy.CopyFrom(maps_, allocator);
      data.AddMember("maps", maps_copy, allocator);
      send_map_response(command_type, request_message_id, data);
    } else if (command_type == "get_current_map") {
      rapidjson::Document data;
      data.SetObject();
      auto & allocator = data.GetAllocator();
      data.AddMember("status", "success", allocator);
      data.AddMember("current_map_id", rapidjson::Value(current_map_id_.c_str(), allocator).Move(), allocator);
      data.AddMember("default_map_id", rapidjson::Value(config_.default_map_id.c_str(), allocator).Move(), allocator);
      data.AddMember("current_map", find_map_as_value(current_map_id_, allocator), allocator);
      send_map_response(command_type, request_message_id, data);
    } else if (command_type == "switch_map") {
      handle_switch_map(command, request_message_id);
    } else {
      send_map_response(command_type.empty() ? "unknown" : command_type, request_message_id, make_error_payload(
        "unknown_map_command", "未知地图命令: " + command_type, current_map_id_));
    }
  }

  // ===========================================================================
  // 4. 切图状态机：校验、脚本启动、watchdog、定位稳定和超时
  // ===========================================================================

  void handle_switch_map(const rapidjson::Value & command, const std::string & request_message_id)
  {
    const std::string target_map_id = normalize_map_id(read_string_member(command, "target_map_id", ""));
    auto target_map = find_map_document(target_map_id);
    if (!target_map.has_value()) {
      send_map_response("switch_map", request_message_id, make_error_payload(
        "map_not_registered", "地图未注册: " + target_map_id, current_map_id_, target_map_id));
      return;
    }
    if (navigation_active_) {
      auto data = make_error_payload(
        "map_switch_rejected_route_task_active",
        "当前导航状态不允许切图: " + navigation_detailed_state_,
        current_map_id_,
        target_map_id);
      data.AddMember("map_state", rapidjson::Value(map_state_.c_str(), data.GetAllocator()).Move(), data.GetAllocator());
      send_map_response("switch_map", request_message_id, data);
      return;
    }
    if (target_map->HasMember("enabled") && (*target_map)["enabled"].IsBool() && !(*target_map)["enabled"].GetBool()) {
      send_map_response("switch_map", request_message_id, make_error_payload(
        "map_disabled", "地图已禁用: " + target_map_id, current_map_id_, target_map_id));
      return;
    }
    const std::string map_yaml_file = read_string_member(*target_map, "map_yaml_file", "");
    if (!map_yaml_file.empty() && !fs::exists(map_yaml_file)) {
      auto data = make_error_payload("map_file_missing", "地图文件不存在: " + map_yaml_file, current_map_id_, target_map_id);
      data.AddMember("map_yaml_file", rapidjson::Value(map_yaml_file.c_str(), data.GetAllocator()).Move(), data.GetAllocator());
      send_map_response("switch_map", request_message_id, data);
      return;
    }
    const std::string prior_map_file = read_string_member(*target_map, "open3d_prior_map_file", "");
    if (!prior_map_file.empty() && !fs::exists(prior_map_file)) {
      auto data = make_error_payload("prior_map_file_missing", "3D 定位地图不存在: " + prior_map_file, current_map_id_, target_map_id);
      data.AddMember("open3d_prior_map_file", rapidjson::Value(prior_map_file.c_str(), data.GetAllocator()).Move(), data.GetAllocator());
      send_map_response("switch_map", request_message_id, data);
      return;
    }
    if (map_state_ == "switching" || map_state_ == "localization_resetting" || map_state_ == "waiting_localization") {
      const std::string active_target = switch_context_ ? switch_context_->target_map_id : target_map_id;
      send_map_response("switch_map", request_message_id, make_error_payload(
        "map_switch_in_progress", "已有地图切换正在进行", current_map_id_, active_target));
      return;
    }

    if (target_map_id != current_map_id_) {
      if (!fs::exists(config_.map_switch_script)) {
        auto data = make_error_payload(
          "map_switch_script_missing",
          "切图重启脚本不存在: " + config_.map_switch_script,
          current_map_id_,
          target_map_id);
        data.AddMember("map_switch_script", rapidjson::Value(config_.map_switch_script.c_str(), data.GetAllocator()).Move(), data.GetAllocator());
        send_map_response("switch_map", request_message_id, data);
        return;
      }
      start_cross_map_switch(request_message_id, target_map_id, *target_map, map_yaml_file, prior_map_file);
      return;
    }

    if (target_map_id == current_map_id_ && map_state_ == "ready") {
      rapidjson::Document data;
      data.SetObject();
      auto & allocator = data.GetAllocator();
      data.AddMember("status", "success", allocator);
      data.AddMember("result_reason", "already_active", allocator);
      data.AddMember("message", "目标地图已经处于 ready 状态", allocator);
      data.AddMember("current_map_id", rapidjson::Value(current_map_id_.c_str(), allocator).Move(), allocator);
      data.AddMember("target_map_id", rapidjson::Value(target_map_id.c_str(), allocator).Move(), allocator);
      data.AddMember("map_state", rapidjson::Value(map_state_.c_str(), allocator).Move(), allocator);
      data.AddMember("localization_state", rapidjson::Value(localization_state_.c_str(), allocator).Move(), allocator);
      send_map_response("switch_map", request_message_id, data);
      return;
    }

    start_same_map_localization_reset(request_message_id, target_map_id, *target_map);
  }

  void start_cross_map_switch(
    const std::string & request_message_id,
    const std::string & target_map_id,
    const rapidjson::Document & target_map,
    const std::string & map_yaml_file,
    const std::string & prior_map_file)
  {
    map_state_ = "switching";
    localization_state_ = "restarting_navigation_stack";
    localization_stable_count_ = 0;
    const std::string old_map_id = current_map_id_;
    current_map_id_ = target_map_id;
    persist_current_map_id(target_map_id);

    const pid_t pid = fork();
    if (pid < 0) {
      current_map_id_ = old_map_id;
      persist_current_map_id(old_map_id);
      map_state_ = "failed";
      localization_state_ = "restart_failed";
      auto data = make_error_payload(
        "map_switch_restart_failed", "启动切图重启脚本失败", current_map_id_, target_map_id);
      data.AddMember("map_switch_script", rapidjson::Value(config_.map_switch_script.c_str(), data.GetAllocator()).Move(), data.GetAllocator());
      send_map_response("switch_map", request_message_id, data);
      publish_map_status();
      return;
    }
    if (pid == 0) {
      setsid();
      const fs::path script_path(config_.map_switch_script);
      setenv("WORKSPACE", script_path.parent_path().c_str(), 1);
      setenv("MAP_ID", target_map_id.c_str(), 1);
      if (chdir(script_path.parent_path().c_str()) != 0) {
        _exit(126);
      }
      execl("/bin/bash", "/bin/bash", config_.map_switch_script.c_str(), target_map_id.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }

    switch_context_ = std::make_unique<SwitchContext>();
    switch_context_->request_message_id = request_message_id;
    switch_context_->target_map_id = target_map_id;
    switch_context_->target_map.CopyFrom(target_map, switch_context_->target_map.GetAllocator());
    switch_context_->started_at = now_seconds();
    switch_context_->switch_pid = pid;

    rapidjson::Document data;
    data.SetObject();
    auto & allocator = data.GetAllocator();
    data.AddMember("status", "success", allocator);
    data.AddMember("result_reason", "map_switch_restart_started", allocator);
    data.AddMember("message", "切图已开始：控制层保持在线，导航定位层将按目标地图重启", allocator);
    data.AddMember("previous_map_id", rapidjson::Value(old_map_id.c_str(), allocator).Move(), allocator);
    data.AddMember("current_map_id", rapidjson::Value(current_map_id_.c_str(), allocator).Move(), allocator);
    data.AddMember("target_map_id", rapidjson::Value(target_map_id.c_str(), allocator).Move(), allocator);
    data.AddMember("map_yaml_file", rapidjson::Value(map_yaml_file.c_str(), allocator).Move(), allocator);
    data.AddMember("open3d_prior_map_file", rapidjson::Value(prior_map_file.c_str(), allocator).Move(), allocator);
    data.AddMember("map_switch_script", rapidjson::Value(config_.map_switch_script.c_str(), allocator).Move(), allocator);
    send_map_response("switch_map", request_message_id, data);
    publish_map_status();
  }

  void start_same_map_localization_reset(
    const std::string & request_message_id,
    const std::string & target_map_id,
    const rapidjson::Document & target_map)
  {
    map_state_ = "switching";
    localization_state_ = "resetting";
    localization_stable_count_ = 0;
    current_map_id_ = target_map_id;
    persist_current_map_id(target_map_id);

    switch_context_ = std::make_unique<SwitchContext>();
    switch_context_->request_message_id = request_message_id;
    switch_context_->target_map_id = target_map_id;
    switch_context_->target_map.CopyFrom(target_map, switch_context_->target_map.GetAllocator());
    switch_context_->started_at = now_seconds();

    rapidjson::Document data;
    data.SetObject();
    auto & allocator = data.GetAllocator();
    data.AddMember("status", "success", allocator);
    data.AddMember("result_reason", "map_switch_started", allocator);
    data.AddMember("message", "地图切换已开始，等待定位稳定", allocator);
    data.AddMember("current_map_id", rapidjson::Value(current_map_id_.c_str(), allocator).Move(), allocator);
    data.AddMember("target_map_id", rapidjson::Value(target_map_id.c_str(), allocator).Move(), allocator);
    data.AddMember("map_state", rapidjson::Value(map_state_.c_str(), allocator).Move(), allocator);
    data.AddMember("localization_state", rapidjson::Value(localization_state_.c_str(), allocator).Move(), allocator);
    send_map_response("switch_map", request_message_id, data);
    publish_map_status();
    switch_watchdog_tick();
  }

  void switch_watchdog_tick()
  {
    if (!switch_context_) {
      return;
    }

    const double now = now_seconds();
    const std::string target_map_id = switch_context_->target_map_id;
    const std::string request_message_id = switch_context_->request_message_id;

    if (switch_context_->switch_pid > 0) {
      int status = 0;
      const pid_t result = waitpid(switch_context_->switch_pid, &status, WNOHANG);
      if (result == switch_context_->switch_pid) {
        const int exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
        if (exit_code != 0) {
          map_state_ = "failed";
          localization_state_ = "navigation_stack_restart_failed";
          auto data = make_error_payload(
            "map_switch_restart_failed",
            "导航定位层重启脚本异常退出: " + std::to_string(exit_code),
            current_map_id_,
            target_map_id);
          data.AddMember("map_state", rapidjson::Value(map_state_.c_str(), data.GetAllocator()).Move(), data.GetAllocator());
          data.AddMember("localization_state", rapidjson::Value(localization_state_.c_str(), data.GetAllocator()).Move(), data.GetAllocator());
          send_map_response("switch_map", request_message_id, data);
          switch_context_.reset();
          publish_map_status();
          return;
        }
        if (localization_state_ == "restarting_navigation_stack") {
          localization_state_ = "waiting_localization";
        }
        switch_context_->switch_pid = -1;
      }
    }

    if (now - switch_context_->started_at > config_.switch_localization_timeout_sec) {
      map_state_ = "failed";
      auto data = make_error_payload(
        "map_switch_localization_timeout", "切图后等待定位稳定超时", current_map_id_, target_map_id);
      data.AddMember("map_state", rapidjson::Value(map_state_.c_str(), data.GetAllocator()).Move(), data.GetAllocator());
      data.AddMember("localization_state", rapidjson::Value(localization_state_.c_str(), data.GetAllocator()).Move(), data.GetAllocator());
      send_map_response("switch_map", request_message_id, data);
      switch_context_.reset();
      publish_map_status();
      return;
    }

    if (switch_context_->initialpose_sent < config_.initialpose_repeat_count) {
      if (now - switch_context_->last_initialpose_time >= config_.initialpose_repeat_interval_sec) {
        map_state_ = "localization_resetting";
        publish_initial_pose(switch_context_->target_map);
        switch_context_->initialpose_sent += 1;
        switch_context_->last_initialpose_time = now;
        publish_map_status();
      }
      return;
    }

    map_state_ = "waiting_localization";
    if (localization_stable_count_ >= config_.switch_localization_stable_frames) {
      map_state_ = "ready";
      localization_state_ = "stable";
      rapidjson::Document data;
      data.SetObject();
      auto & allocator = data.GetAllocator();
      data.AddMember("status", "success", allocator);
      data.AddMember("result_reason", "map_ready", allocator);
      data.AddMember("message", "地图切换完成，定位已稳定，可以开始导航", allocator);
      data.AddMember("current_map_id", rapidjson::Value(current_map_id_.c_str(), allocator).Move(), allocator);
      data.AddMember("target_map_id", rapidjson::Value(target_map_id.c_str(), allocator).Move(), allocator);
      data.AddMember("map_state", rapidjson::Value(map_state_.c_str(), allocator).Move(), allocator);
      data.AddMember("localization_state", rapidjson::Value(localization_state_.c_str(), allocator).Move(), allocator);
      send_map_response("switch_map", request_message_id, data);
      switch_context_.reset();
    }
    publish_map_status();
  }

  // ===========================================================================
  // 5. 状态输入：导航状态、定位健康状态
  // ===========================================================================

  void on_localization_status(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    rapidjson::Document data;
    bool healthy = false;
    std::string state = "UNKNOWN";
    if (!data.Parse(msg->data.c_str()).HasParseError() && data.IsObject()) {
      state = uppercase(read_string_member(data, "state", read_string_member(data, "status", "")));
      healthy =
        (data.HasMember("healthy") && data["healthy"].IsBool() && data["healthy"].GetBool()) ||
        state == "ACCEPTED" || state == "LOCALIZED" || state == "HEALTHY" || state == "OK" || state == "READY";
    } else {
      const std::string text = trim_copy(msg->data);
      state = uppercase(text.substr(0, text.find(' ')));
      healthy = state == "ACCEPTED";
    }
    if (healthy) {
      ++localization_stable_count_;
      localization_state_ = "stable";
    } else {
      localization_stable_count_ = 0;
      localization_state_ = lowercase(state.empty() ? "unstable" : state);
    }
  }

  void on_navigation_status(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    rapidjson::Document payload;
    if (payload.Parse(msg->data.c_str()).HasParseError() || !payload.IsObject()) {
      return;
    }
    if (payload.HasMember("event_type")) {
      return;
    }
    navigation_active_ = payload.HasMember("is_active") && payload["is_active"].IsBool() && payload["is_active"].GetBool();
    const std::string detailed_state = read_string_member(payload, "detailed_state", "");
    const std::string current_state = read_string_member(payload, "current_state", "");
    if (payload.HasMember("route_task") && payload["route_task"].IsObject()) {
      const auto & route_task = payload["route_task"];
      if (route_task.HasMember("awaiting_broadcast") && route_task["awaiting_broadcast"].IsBool() &&
        route_task["awaiting_broadcast"].GetBool())
      {
        navigation_active_ = true;
      }
    }
    if (current_state == "executing" || current_state == "paused" || current_state == "reached_waypoint") {
      navigation_active_ = true;
    }
    navigation_detailed_state_ = detailed_state.empty() ? current_state : detailed_state;
  }

  // ===========================================================================
  // 6. 输出链路：/map/response、/map/status、/initialpose
  // ===========================================================================

  void publish_initial_pose(const rapidjson::Value & map_info)
  {
    const rapidjson::Value * initial_pose = nullptr;
    if (map_info.HasMember("initial_pose") && map_info["initial_pose"].IsObject()) {
      initial_pose = &map_info["initial_pose"];
    }
    std::vector<double> position{0.0, 0.0, 0.0};
    std::vector<double> orientation{0.0, 0.0, 0.0, 1.0};
    std::string frame_id = "map";
    if (initial_pose != nullptr) {
      position = read_double_array(*initial_pose, "position", position);
      orientation = read_double_array(*initial_pose, "orientation", orientation);
      frame_id = read_string_member(*initial_pose, "frame_id", "map");
    }
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id;
    msg.pose.pose.position.x = position.size() > 0 ? position[0] : 0.0;
    msg.pose.pose.position.y = position.size() > 1 ? position[1] : 0.0;
    msg.pose.pose.position.z = position.size() > 2 ? position[2] : 0.0;
    msg.pose.pose.orientation.x = orientation.size() > 0 ? orientation[0] : 0.0;
    msg.pose.pose.orientation.y = orientation.size() > 1 ? orientation[1] : 0.0;
    msg.pose.pose.orientation.z = orientation.size() > 2 ? orientation[2] : 0.0;
    msg.pose.pose.orientation.w = orientation.size() > 3 ? orientation[3] : 1.0;
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.0685;
    initialpose_pub_->publish(msg);
  }

  void send_map_response(
    const std::string & command_type,
    const std::string & request_message_id,
    const rapidjson::Value & data)
  {
    rapidjson::Document payload;
    payload.SetObject();
    auto & allocator = payload.GetAllocator();
    payload.CopyFrom(data, allocator);
    payload.AddMember("command_type", rapidjson::Value(command_type.c_str(), allocator).Move(), allocator);
    payload.AddMember("request_message_id", rapidjson::Value(request_message_id.c_str(), allocator).Move(), allocator);
    payload.AddMember("timestamp", now_seconds(), allocator);
    rapidjson::Value base = build_base_message("response", "map_response", payload, allocator);
    rapidjson::Document out;
    out.SetObject();
    out.CopyFrom(base, out.GetAllocator());
    std_msgs::msg::String msg;
    msg.data = json_to_string(out);
    map_response_pub_->publish(msg);
  }

  void publish_map_status()
  {
    rapidjson::Document data;
    data.SetObject();
    auto & allocator = data.GetAllocator();
    data.AddMember("status", "success", allocator);
    data.AddMember("current_map_id", rapidjson::Value(current_map_id_.c_str(), allocator).Move(), allocator);
    data.AddMember("default_map_id", rapidjson::Value(config_.default_map_id.c_str(), allocator).Move(), allocator);
    data.AddMember("map_state", rapidjson::Value(map_state_.c_str(), allocator).Move(), allocator);
    data.AddMember("localization_state", rapidjson::Value(localization_state_.c_str(), allocator).Move(), allocator);
    data.AddMember("localization_stable_count", localization_stable_count_, allocator);
    const std::string target = switch_context_ ? switch_context_->target_map_id : "";
    data.AddMember("switch_target_map_id", rapidjson::Value(target.c_str(), allocator).Move(), allocator);
    data.AddMember("navigation_active", navigation_active_, allocator);
    data.AddMember("navigation_detailed_state", rapidjson::Value(navigation_detailed_state_.c_str(), allocator).Move(), allocator);
    data.AddMember("maps_count", static_cast<int>(maps_.Size()), allocator);
    data.AddMember("timestamp", now_seconds(), allocator);
    rapidjson::Value base = build_base_message("push", "map_status", data, allocator);
    rapidjson::Document out;
    out.SetObject();
    out.CopyFrom(base, out.GetAllocator());
    std_msgs::msg::String msg;
    msg.data = json_to_string(out);
    map_status_pub_->publish(msg);
  }

  rapidjson::Value build_base_message(
    const std::string & message_type,
    const std::string & data_type,
    const rapidjson::Value & data,
    rapidjson::Document::AllocatorType & allocator) const
  {
    rapidjson::Value message(rapidjson::kObjectType);
    message.AddMember("protocol_version", "2.0", allocator);
    const std::string message_id = data_type + "_" + std::to_string(static_cast<int64_t>(now_seconds() * 1000.0));
    message.AddMember("message_id", rapidjson::Value(message_id.c_str(), allocator).Move(), allocator);
    message.AddMember("timestamp", now_seconds(), allocator);
    message.AddMember("message_type", rapidjson::Value(message_type.c_str(), allocator).Move(), allocator);
    message.AddMember("data_type", rapidjson::Value(data_type.c_str(), allocator).Move(), allocator);
    message.AddMember("source", "map_context_manager", allocator);
    message.AddMember("destination", "all", allocator);
    rapidjson::Value data_copy;
    data_copy.CopyFrom(data, allocator);
    message.AddMember("data", data_copy, allocator);
    rapidjson::Value metadata(rapidjson::kObjectType);
    const std::string status = read_string_member(message["data"], "status", "success");
    const std::string error_message = status == "error" ? read_string_member(message["data"], "message", "") : "";
    metadata.AddMember("status", rapidjson::Value(status.c_str(), allocator).Move(), allocator);
    metadata.AddMember("error_code", rapidjson::Value(read_string_member(message["data"], "error_code", "").c_str(), allocator).Move(), allocator);
    metadata.AddMember("error_message", rapidjson::Value(error_message.c_str(), allocator).Move(), allocator);
    metadata.AddMember("request_id", rapidjson::Value(read_string_member(message["data"], "request_message_id", "").c_str(), allocator).Move(), allocator);
    message.AddMember("metadata", metadata, allocator);
    return message;
  }

  // ===========================================================================
  // 7. 地图注册表读写和 JSON 工具
  // ===========================================================================

  void load_map_registry()
  {
    const fs::path path(config_.map_registry_path);
    if (!fs::exists(path)) {
      fs::create_directories(path.parent_path());
      rapidjson::Document registry;
      registry.SetObject();
      auto & allocator = registry.GetAllocator();
      registry.AddMember("default_map_id", rapidjson::Value(config_.default_map_id.c_str(), allocator).Move(), allocator);
      registry.AddMember("current_map_id", rapidjson::Value(current_map_id_.c_str(), allocator).Move(), allocator);
      rapidjson::Value maps(rapidjson::kArrayType);
      rapidjson::Value item(rapidjson::kObjectType);
      item.AddMember("map_id", rapidjson::Value(config_.default_map_id.c_str(), allocator).Move(), allocator);
      item.AddMember("display_name", "默认地图", allocator);
      item.AddMember("enabled", true, allocator);
      item.AddMember("description", "由多地图一期自动创建的默认地图", allocator);
      item.AddMember("initial_pose", default_initial_pose_json(allocator), allocator);
      maps.PushBack(item, allocator);
      registry.AddMember("maps", maps, allocator);
      write_json_file(path.string(), registry);
      maps_.CopyFrom(registry["maps"], maps_.GetAllocator());
      return;
    }

    auto registry = read_json_file(path.string());
    if (registry.has_value() && registry->IsObject()) {
      config_.default_map_id = normalize_map_id(read_string_member(*registry, "default_map_id", config_.default_map_id));
      current_map_id_ = normalize_map_id(read_string_member(*registry, "current_map_id", config_.default_map_id));
      if (registry->HasMember("maps") && (*registry)["maps"].IsArray() && !(*registry)["maps"].Empty()) {
        maps_.CopyFrom((*registry)["maps"], maps_.GetAllocator());
        return;
      }
    }

    maps_.SetArray();
    rapidjson::Value fallback(rapidjson::kObjectType);
    auto & allocator = maps_.GetAllocator();
    fallback.AddMember("map_id", rapidjson::Value(config_.default_map_id.c_str(), allocator).Move(), allocator);
    fallback.AddMember("display_name", "默认地图", allocator);
    fallback.AddMember("enabled", true, allocator);
    fallback.AddMember("description", "地图注册表异常时的兜底地图", allocator);
    maps_.PushBack(fallback, allocator);
  }

  void persist_current_map_id(const std::string & map_id)
  {
    rapidjson::Document registry;
    registry.SetObject();
    auto & allocator = registry.GetAllocator();
    registry.AddMember("default_map_id", rapidjson::Value(config_.default_map_id.c_str(), allocator).Move(), allocator);
    const std::string normalized = normalize_map_id(map_id);
    registry.AddMember("current_map_id", rapidjson::Value(normalized.c_str(), allocator).Move(), allocator);
    rapidjson::Value maps_copy;
    maps_copy.CopyFrom(maps_, allocator);
    registry.AddMember("maps", maps_copy, allocator);
    write_json_file(config_.map_registry_path, registry);
  }

  std::optional<rapidjson::Document> find_map_document(const std::string & map_id) const
  {
    const std::string normalized = normalize_map_id(map_id);
    for (const auto & item : maps_.GetArray()) {
      if (item.IsObject() && normalize_map_id(read_string_member(item, "map_id", "")) == normalized) {
        rapidjson::Document doc;
        doc.SetObject();
        doc.CopyFrom(item, doc.GetAllocator());
        return doc;
      }
    }
    return std::nullopt;
  }

  rapidjson::Value find_map_as_value(
    const std::string & map_id,
    rapidjson::Document::AllocatorType & allocator) const
  {
    const auto item = find_map_document(map_id);
    rapidjson::Value value(rapidjson::kObjectType);
    if (item.has_value()) {
      value.CopyFrom(*item, allocator);
    }
    return value;
  }

  rapidjson::Document make_error_payload(
    const std::string & error_code,
    const std::string & message,
    const std::string & current_map_id,
    const std::string & target_map_id = "") const
  {
    rapidjson::Document data;
    data.SetObject();
    auto & allocator = data.GetAllocator();
    data.AddMember("status", "error", allocator);
    data.AddMember("error_code", rapidjson::Value(error_code.c_str(), allocator).Move(), allocator);
    data.AddMember("message", rapidjson::Value(message.c_str(), allocator).Move(), allocator);
    data.AddMember("current_map_id", rapidjson::Value(current_map_id.c_str(), allocator).Move(), allocator);
    if (!target_map_id.empty()) {
      data.AddMember("target_map_id", rapidjson::Value(target_map_id.c_str(), allocator).Move(), allocator);
    }
    return data;
  }

  rapidjson::Value default_initial_pose_json(rapidjson::Document::AllocatorType & allocator) const
  {
    rapidjson::Value pose(rapidjson::kObjectType);
    pose.AddMember("frame_id", "map", allocator);
    rapidjson::Value position(rapidjson::kArrayType);
    position.PushBack(0.0, allocator).PushBack(0.0, allocator).PushBack(0.0, allocator);
    pose.AddMember("position", position, allocator);
    rapidjson::Value orientation(rapidjson::kArrayType);
    orientation.PushBack(0.0, allocator).PushBack(0.0, allocator).PushBack(0.0, allocator).PushBack(1.0, allocator);
    pose.AddMember("orientation", orientation, allocator);
    return pose;
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
      RCLCPP_ERROR(
        get_logger(),
        "JSON 解析失败: %s offset=%zu error=%s",
        file_path.c_str(),
        doc.GetErrorOffset(),
        rapidjson::GetParseError_En(doc.GetParseError()));
      return std::nullopt;
    }
    return doc;
  }

  void write_json_file(const std::string & file_path, const rapidjson::Document & doc) const
  {
    const fs::path path(file_path);
    fs::create_directories(path.parent_path());
    FILE * file = std::fopen(file_path.c_str(), "wb");
    if (file == nullptr) {
      RCLCPP_WARN(get_logger(), "写入 JSON 失败: %s", file_path.c_str());
      return;
    }
    char buffer[65536];
    rapidjson::FileWriteStream stream(file, buffer, sizeof(buffer));
    rapidjson::PrettyWriter<rapidjson::FileWriteStream> writer(stream);
    writer.SetIndent(' ', 2);
    doc.Accept(writer);
    std::fclose(file);
  }

  std::string normalize_map_id(const std::string & map_id) const
  {
    const std::string normalized = trim_copy(map_id);
    return normalized.empty() ? "hall" : normalized;
  }

  std::string expand_user_path(const std::string & path) const
  {
    if (path.rfind("~/", 0) != 0) {
      return path;
    }
    const char * home = std::getenv("HOME");
    return home == nullptr ? path : std::string(home) + path.substr(1);
  }

  std::string read_string_member(
    const rapidjson::Value & object,
    const char * key,
    const std::string & fallback) const
  {
    if (!object.IsObject() || !object.HasMember(key)) {
      return fallback;
    }
    const auto & value = object[key];
    if (value.IsString()) {
      return value.GetString();
    }
    if (value.IsInt64()) {
      return std::to_string(value.GetInt64());
    }
    if (value.IsUint64()) {
      return std::to_string(value.GetUint64());
    }
    return fallback;
  }

  std::string uppercase(std::string text) const
  {
    for (auto & ch : text) {
      ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }
    return text;
  }

  std::string lowercase(std::string text) const
  {
    for (auto & ch : text) {
      ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return text;
  }

  static std::chrono::milliseconds to_duration(
    const double seconds,
    const std::chrono::milliseconds fallback)
  {
    const auto millis = static_cast<int64_t>(seconds * 1000.0);
    return millis > 0 ? std::chrono::milliseconds(millis) : fallback;
  }

  MapContextConfig config_;
  std::string current_map_id_{"hall"};
  rapidjson::Document maps_;
  std::string map_state_{"ready"};
  std::string localization_state_{"unknown"};
  int localization_stable_count_{0};
  std::unique_ptr<SwitchContext> switch_context_;
  bool navigation_active_{false};
  std::string navigation_detailed_state_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_response_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;
  rclcpp::TimerBase::SharedPtr map_status_timer_;
  rclcpp::TimerBase::SharedPtr switch_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr initial_status_timer_;
};

}  // namespace humanoid_control_runtime

// =============================================================================
// 8. main 入口
// =============================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_control_runtime::MapContextManagerNode>());
  rclcpp::shutdown();
  return 0;
}
