/*
 * 文件功能：C++ 表情串口驱动节点。
 *
 * 主要职责：
 * 1. 从参数文件读取表情动作库路径、串口设备、波特率和动作执行时序参数。
 * 2. 解析 humanoid_expression_runtime/config/facial_gestures.yaml 中的 facial_gestures 表情序列。
 * 3. 订阅 /robot/facial_raw_cmd，接收 APP 网关或调试工具发来的表情动作名。
 * 4. 按动作序列向仿生头串口控制板发送原始串口指令，并支持 delay:x 延时标记。
 * 5. 串口断开或发送失败时关闭句柄，下次动作自动尝试重连。
 * 6. 节点启动时执行 idle，退出时尝试执行 sleeping，与旧运行逻辑保持一致。
 *
 * 上游节点：
 * - humanoid_app_gateway_runtime/app_gateway_node：发布 /robot/facial_raw_cmd。
 * - facial_control_ui 或 ros2 topic pub：人工调试时也可直接发布 /robot/facial_raw_cmd。
 *
 * 下游硬件：
 * - 仿生头串口控制板，默认 /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0。
 * - 如果 by-id 不存在，会按 fallback_ports 依次尝试 /dev/ttyUSB0、/dev/ttyUSB1 等候选串口。
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

using namespace std::chrono_literals;

// 表情动作库：key 是动作名，例如 idle/talk；value 是串口指令和 delay 标记的有序列表。
using GestureLibrary = std::map<std::string, std::vector<std::string>>;

std::string trim_copy(const std::string & input)
{
  const auto begin = input.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(begin, end - begin + 1);
}

bool starts_with(const std::string & text, const std::string & prefix)
{
  return text.rfind(prefix, 0) == 0;
}

speed_t baudrate_to_termios(const int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}

}  // namespace

class FacialDriverCpp final : public rclcpp::Node
{
public:
  FacialDriverCpp()
  : Node("facial_driver_node")
  {
    declare_and_load_parameters();
    resolve_default_config_path();
    connect_serial();
    load_gesture_config();

    // 命令入口保持与旧 Python 节点一致：只接收纯动作名字符串，不接收 JSON。
    command_sub_ = create_subscription<std_msgs::msg::String>(
      command_topic_,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        handle_command(msg->data);
      });

    if (!trim_copy(startup_gesture_).empty()) {
      current_gesture_ = startup_gesture_;
      execute_gesture(startup_gesture_);
    }
  }

  ~FacialDriverCpp() override
  {
    shutdown_driver();
  }

private:
  // ==================== 参数加载 ====================

  void declare_and_load_parameters()
  {
    declare_parameter<std::string>("config_path", "");
    declare_parameter<std::string>("command_topic", "/robot/facial_raw_cmd");
    declare_parameter<std::string>("port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
    declare_parameter<std::vector<std::string>>(
      "fallback_ports",
      std::vector<std::string>{"/dev/ttyUSB0", "/dev/ttyUSB1"});
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<int>("serial_timeout_ms", 100);
    declare_parameter<double>("connect_settle_sec", 2.0);
    declare_parameter<double>("interrupt_wait_sec", 0.5);
    declare_parameter<std::string>("startup_gesture", "idle");
    declare_parameter<std::string>("shutdown_gesture", "sleeping");
    declare_parameter<bool>("pre_stop_eye_enable", true);
    declare_parameter<int>("pre_stop_eye_repeat", 2);
    declare_parameter<double>("pre_stop_eye_command_delay_sec", 0.1);
    declare_parameter<bool>("pre_stop_mouth_enable", true);
    declare_parameter<int>("pre_stop_mouth_repeat", 1);
    declare_parameter<double>("pre_stop_mouth_command_delay_sec", 0.1);
    declare_parameter<bool>("eye_reset_enable", true);
    declare_parameter<std::vector<std::string>>("eye_reset_commands", std::vector<std::string>{"$DGL:1!"});
    declare_parameter<double>("eye_reset_command_delay_sec", 0.1);
    declare_parameter<double>("idle_normal_start_pre_delay_sec", 0.5);
    declare_parameter<double>("idle_normal_start_post_delay_sec", 0.8);
    declare_parameter<double>("stop_command_delay_sec", 0.6);
    declare_parameter<double>("loop_start_command_delay_sec", 0.8);
    declare_parameter<double>("default_command_delay_sec", 0.1);
    declare_parameter<double>("shutdown_wait_sec", 1.0);
    declare_parameter<bool>("log_serial_output", true);

    get_parameter("config_path", config_path_);
    get_parameter("command_topic", command_topic_);
    get_parameter("port", port_);
    get_parameter("fallback_ports", fallback_ports_);
    get_parameter("baudrate", baudrate_);
    get_parameter("serial_timeout_ms", serial_timeout_ms_);
    get_parameter("connect_settle_sec", connect_settle_sec_);
    get_parameter("interrupt_wait_sec", interrupt_wait_sec_);
    get_parameter("startup_gesture", startup_gesture_);
    get_parameter("shutdown_gesture", shutdown_gesture_);
    get_parameter("pre_stop_eye_enable", pre_stop_eye_enable_);
    get_parameter("pre_stop_eye_repeat", pre_stop_eye_repeat_);
    get_parameter("pre_stop_eye_command_delay_sec", pre_stop_eye_command_delay_sec_);
    get_parameter("pre_stop_mouth_enable", pre_stop_mouth_enable_);
    get_parameter("pre_stop_mouth_repeat", pre_stop_mouth_repeat_);
    get_parameter("pre_stop_mouth_command_delay_sec", pre_stop_mouth_command_delay_sec_);
    get_parameter("eye_reset_enable", eye_reset_enable_);
    get_parameter("eye_reset_commands", eye_reset_commands_);
    get_parameter("eye_reset_command_delay_sec", eye_reset_command_delay_sec_);
    get_parameter("idle_normal_start_pre_delay_sec", idle_normal_start_pre_delay_sec_);
    get_parameter("idle_normal_start_post_delay_sec", idle_normal_start_post_delay_sec_);
    get_parameter("stop_command_delay_sec", stop_command_delay_sec_);
    get_parameter("loop_start_command_delay_sec", loop_start_command_delay_sec_);
    get_parameter("default_command_delay_sec", default_command_delay_sec_);
    get_parameter("shutdown_wait_sec", shutdown_wait_sec_);
    get_parameter("log_serial_output", log_serial_output_);
  }

  void resolve_default_config_path()
  {
    config_path_ = trim_copy(config_path_);
    if (!config_path_.empty()) {
      return;
    }

    try {
      config_path_ = ament_index_cpp::get_package_share_directory("humanoid_expression_runtime") +
        "/config/facial_gestures.yaml";
    } catch (const std::exception & e) {
      // 未 source install 或包未安装时，保留源码树兜底路径，方便开发期直接验证。
      (void)e;
      config_path_ = "src/humanoid_expression_runtime/config/facial_gestures.yaml";
    }
  }

  // ==================== YAML 表情库加载 ====================

  void load_gesture_config()
  {
    gestures_.clear();

    try {
      const YAML::Node root = YAML::LoadFile(config_path_);
      const YAML::Node facial_gestures = root["facial_gestures"];
      if (!facial_gestures || !facial_gestures.IsMap()) {
        RCLCPP_ERROR(get_logger(), "表情库格式错误，缺少 facial_gestures: %s", config_path_.c_str());
        return;
      }

      for (const auto & item : facial_gestures) {
        const std::string name = item.first.as<std::string>();
        const YAML::Node sequence = item.second;
        std::vector<std::string> commands;

        if (sequence && sequence.IsSequence()) {
          for (const auto & command : sequence) {
            commands.push_back(command.as<std::string>());
          }
        }
        gestures_[name] = commands;
      }

      RCLCPP_INFO(
        get_logger(),
        "成功加载表情库: path=%s, gestures=%zu",
        config_path_.c_str(),
        gestures_.size());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "表情库加载失败: path=%s, error=%s", config_path_.c_str(), e.what());
    }
  }

  // ==================== 串口生命周期 ====================

  bool connect_serial()
  {
    if (serial_fd_ >= 0) {
      return true;
    }

    const auto candidates = build_serial_candidates();
    std::string last_error;
    for (const auto & candidate : candidates) {
      if (open_and_configure_serial(candidate, last_error)) {
        active_port_ = candidate;
        RCLCPP_INFO(get_logger(), "已连接到仿生头模块: %s, baudrate=%d", active_port_.c_str(), baudrate_);
        sleep_seconds(connect_settle_sec_);
        return true;
      }
    }

    RCLCPP_ERROR(
      get_logger(),
      "头部串口连接失败，已尝试 %zu 个候选设备，最后错误: %s",
      candidates.size(),
      last_error.c_str());
    return false;
  }

  std::vector<std::string> build_serial_candidates() const
  {
    std::vector<std::string> candidates;
    auto append_unique = [&candidates](const std::string & port) {
      const std::string trimmed = trim_copy(port);
      if (trimmed.empty()) {
        return;
      }
      for (const auto & existing : candidates) {
        if (existing == trimmed) {
          return;
        }
      }
      candidates.push_back(trimmed);
    };

    append_unique(port_);
    for (const auto & fallback : fallback_ports_) {
      append_unique(fallback);
    }
    return candidates;
  }

  bool open_and_configure_serial(const std::string & candidate, std::string & last_error)
  {
    serial_fd_ = ::open(candidate.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      last_error = candidate + ": " + std::strerror(errno);
      RCLCPP_WARN(get_logger(), "候选串口打开失败: %s", last_error.c_str());
      return false;
    }

    termios tty {};
    if (::tcgetattr(serial_fd_, &tty) != 0) {
      last_error = candidate + ": 读取串口参数失败: " + std::strerror(errno);
      RCLCPP_WARN(get_logger(), "%s", last_error.c_str());
      close_serial();
      return false;
    }

    const speed_t speed = baudrate_to_termios(baudrate_);
    ::cfsetospeed(&tty, speed);
    ::cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = std::max(1, serial_timeout_ms_ / 100);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (::tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      last_error = candidate + ": 设置串口参数失败: " + std::strerror(errno);
      RCLCPP_WARN(get_logger(), "%s", last_error.c_str());
      close_serial();
      return false;
    }

    return true;
  }

  void close_serial()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void flush_serial_buffers()
  {
    if (serial_fd_ < 0) {
      return;
    }

    if (::tcflush(serial_fd_, TCIOFLUSH) != 0) {
      RCLCPP_ERROR(get_logger(), "清空串口缓冲区失败，将尝试重连: %s", std::strerror(errno));
      close_serial();
    }
  }

  bool send_raw(const std::string & raw_command)
  {
    const std::string command = trim_copy(raw_command);
    if (command.empty()) {
      RCLCPP_WARN(get_logger(), "空指令，忽略发送");
      return false;
    }

    if (!connect_serial()) {
      RCLCPP_ERROR(get_logger(), "串口未连接，无法发送指令");
      return false;
    }

    const std::string payload = command + "\r\n";
    const char * data = payload.data();
    std::size_t remaining = payload.size();

    while (remaining > 0) {
      const ssize_t written = ::write(serial_fd_, data, remaining);
      if (written < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "串口发送失败，将关闭串口并等待下次指令重连: %s", std::strerror(errno));
        close_serial();
        return false;
      }
      data += written;
      remaining -= static_cast<std::size_t>(written);
    }

    if (log_serial_output_) {
      RCLCPP_INFO(get_logger(), "Serial Out: %s", command.c_str());
    }
    return true;
  }

  // ==================== ROS 命令入口 ====================

  void handle_command(const std::string & action)
  {
    const std::string action_name = trim_copy(action);
    if (action_name.empty()) {
      RCLCPP_WARN(get_logger(), "收到空指令，忽略");
      return;
    }

    RCLCPP_INFO(get_logger(), "收到新指令，中断当前动作: %s", action_name.c_str());
    is_interrupted_ = true;
    sleep_seconds(interrupt_wait_sec_);

    flush_serial_buffers();
    is_interrupted_ = false;
    current_gesture_ = action_name;
    execute_gesture(action_name);
  }

  // ==================== 动作执行 ====================

  void execute_gesture(const std::string & name)
  {
    const auto gesture_it = gestures_.find(name);
    const bool known_gesture = gesture_it != gestures_.end();

    // 对齐旧节点：普通表情执行前，先停眼部循环、停嘴部说话循环，再恢复睁眼。
    if (known_gesture && name != "eye_stop" && name != "mouth_speak_stop") {
      run_pre_stop_sequence();
    }

    if (!known_gesture) {
      RCLCPP_WARN(get_logger(), "表情库中无 %s，尝试作为原始指令发送", name.c_str());
      send_raw(name);
      return;
    }

    RCLCPP_INFO(get_logger(), "执行表情: %s", name.c_str());
    flush_serial_buffers();

    for (const auto & command : gesture_it->second) {
      if (is_interrupted_) {
        RCLCPP_INFO(get_logger(), "动作被中断，停止发送 %s 的剩余指令", name.c_str());
        break;
      }

      const std::string trimmed = trim_copy(command);
      if (starts_with(trimmed, "delay:")) {
        handle_delay_command(trimmed);
        continue;
      }

      if (name == "idle" && trimmed.find("!normal_start") != std::string::npos) {
        sleep_seconds(idle_normal_start_pre_delay_sec_);
        send_raw(trimmed);
        sleep_seconds(idle_normal_start_post_delay_sec_);
        continue;
      }

      send_raw(trimmed);

      if (name == "stop") {
        sleep_seconds(stop_command_delay_sec_);
      } else if (
        trimmed.find("!silent_start") != std::string::npos ||
        trimmed.find("!normal_start") != std::string::npos)
      {
        sleep_seconds(loop_start_command_delay_sec_);
      } else {
        sleep_seconds(default_command_delay_sec_);
      }
    }
  }

  void run_pre_stop_sequence()
  {
    if (pre_stop_eye_enable_) {
      run_named_sequence("eye_stop", pre_stop_eye_repeat_, pre_stop_eye_command_delay_sec_, "强制终止眼部循环");
    }

    if (pre_stop_mouth_enable_) {
      run_named_sequence(
        "mouth_speak_stop",
        pre_stop_mouth_repeat_,
        pre_stop_mouth_command_delay_sec_,
        "强制终止嘴部循环");
    }

    if (eye_reset_enable_) {
      for (const auto & command : eye_reset_commands_) {
        send_raw(command);
        sleep_seconds(eye_reset_command_delay_sec_);
      }
      RCLCPP_INFO(get_logger(), "眼部强制复位");
    }
  }

  void run_named_sequence(
    const std::string & name,
    const int repeat,
    const double command_delay_sec,
    const std::string & log_text)
  {
    const auto it = gestures_.find(name);
    if (it == gestures_.end()) {
      return;
    }

    const int safe_repeat = std::max(0, repeat);
    for (int round = 0; round < safe_repeat; ++round) {
      for (const auto & command : it->second) {
        send_raw(command);
        sleep_seconds(command_delay_sec);
      }
    }
    RCLCPP_INFO(get_logger(), "%s", log_text.c_str());
  }

  void handle_delay_command(const std::string & command)
  {
    try {
      const double delay_sec = std::stod(command.substr(std::string("delay:").size()));
      sleep_seconds(delay_sec);
      RCLCPP_INFO(get_logger(), "延迟 %.3f 秒", delay_sec);
    } catch (const std::exception &) {
      RCLCPP_WARN(get_logger(), "无效延迟指令: %s", command.c_str());
    }
  }

  void sleep_seconds(const double seconds)
  {
    if (seconds <= 0.0) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }

  // ==================== 关闭清理 ====================

  void shutdown_driver()
  {
    if (shutdown_done_) {
      return;
    }
    shutdown_done_ = true;

    RCLCPP_INFO(get_logger(), "开始优雅关闭表情驱动");
    if (serial_fd_ >= 0) {
      if (!trim_copy(shutdown_gesture_).empty()) {
        execute_gesture(shutdown_gesture_);
      }
      sleep_seconds(shutdown_wait_sec_);
      close_serial();
      RCLCPP_INFO(get_logger(), "串口已安全关闭");
    } else {
      RCLCPP_INFO(get_logger(), "串口未连接，无需关闭");
    }
  }

  // ==================== 成员变量 ====================

  std::string config_path_;
  std::string command_topic_;
  std::string port_;
  std::vector<std::string> fallback_ports_{"/dev/ttyUSB0", "/dev/ttyUSB1"};
  std::string active_port_;
  int baudrate_{115200};
  int serial_timeout_ms_{100};
  double connect_settle_sec_{2.0};
  double interrupt_wait_sec_{0.5};
  std::string startup_gesture_{"idle"};
  std::string shutdown_gesture_{"sleeping"};
  bool pre_stop_eye_enable_{true};
  int pre_stop_eye_repeat_{2};
  double pre_stop_eye_command_delay_sec_{0.1};
  bool pre_stop_mouth_enable_{true};
  int pre_stop_mouth_repeat_{1};
  double pre_stop_mouth_command_delay_sec_{0.1};
  bool eye_reset_enable_{true};
  std::vector<std::string> eye_reset_commands_{"$DGL:1!"};
  double eye_reset_command_delay_sec_{0.1};
  double idle_normal_start_pre_delay_sec_{0.5};
  double idle_normal_start_post_delay_sec_{0.8};
  double stop_command_delay_sec_{0.6};
  double loop_start_command_delay_sec_{0.8};
  double default_command_delay_sec_{0.1};
  double shutdown_wait_sec_{1.0};
  bool log_serial_output_{true};

  int serial_fd_{-1};
  bool is_interrupted_{false};
  bool shutdown_done_{false};
  std::string current_gesture_{"idle"};
  GestureLibrary gestures_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FacialDriverCpp>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "节点运行异常: %s", e.what());
  }
  node.reset();
  rclcpp::shutdown();
  return 0;
}
