/*
 * broadcast_service_node.cpp
 *
 * 文件作用：
 * 1. 提供机器人本机播报服务，保持 /xiaorui_broadcast/play、/set_volume、/health 三个服务接口不变。
 * 2. 自动选择音频输出设备，优先 PipeWire/Pulse sink，必要时回退 ALSA 设备。
 * 3. 按请求设置播报音量，并根据 dry_run、command、beep 三种模式执行播报。
 * 4. 保留旧节点使用的环境变量语义，方便现场脚本和产品配置继续复用。
 *
 * 上游节点：
 * - app_gateway_node：把 APP 的 set_broadcast_volume 命令转换成 /xiaorui_broadcast/set_volume 服务调用。
 * - 调试脚本或其他业务节点：可直接调用 /xiaorui_broadcast/play 和 /health。
 *
 * 下游对象：
 * - PipeWire/Pulse：通过 pactl 查询 sink、设置音量。
 * - ALSA：通过 aplay/amixer 发现设备、设置音量和播放测试音。
 * - 外部 TTS 播放脚本：command 模式下可继续调用 edge-tts + ffplay 脚本。
 */

#include <rclcpp/rclcpp.hpp>

#include <humanoid_interfaces/srv/get_broadcast_health.hpp>
#include <humanoid_interfaces/srv/play_broadcast.hpp>
#include <humanoid_interfaces/srv/set_broadcast_volume.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

struct CommandResult
{
  int return_code{0};
  std::string stdout_text;
  std::string stderr_text;
};

struct AudioDevice
{
  std::string backend;
  std::string name;
  std::string description;
  std::string alsa_device;
};

std::string trim(const std::string & text)
{
  const auto begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::string lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

std::vector<std::string> split_lines(const std::string & text)
{
  std::vector<std::string> lines;
  std::stringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    lines.push_back(line);
  }
  return lines;
}

std::vector<std::string> split_tab(const std::string & text)
{
  std::vector<std::string> parts;
  std::stringstream stream(text);
  std::string part;
  while (std::getline(stream, part, '\t')) {
    parts.push_back(part);
  }
  return parts;
}

std::optional<std::string> getenv_optional(const char * name)
{
  const char * value = std::getenv(name);
  if (value == nullptr) {
    return std::nullopt;
  }
  return trim(value);
}

double getenv_double_or(const char * name, double fallback)
{
  const char * value = std::getenv(name);
  if (value == nullptr) {
    return fallback;
  }
  try {
    return std::stod(trim(value));
  } catch (...) {
    return fallback;
  }
}

int getenv_int_or(const char * name, int fallback)
{
  const char * value = std::getenv(name);
  if (value == nullptr) {
    return fallback;
  }
  try {
    return std::stoi(trim(value));
  } catch (...) {
    return fallback;
  }
}

bool getenv_bool_or(const char * name, bool fallback)
{
  const char * value = std::getenv(name);
  if (value == nullptr) {
    return fallback;
  }
  const std::string normalized = lower_copy(trim(value));
  if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on") {
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off") {
    return false;
  }
  return fallback;
}

int normalize_volume(int value)
{
  return std::max(0, std::min(100, value));
}

std::string shell_quote(const std::string & value)
{
  std::string quoted = "'";
  for (const char ch : value) {
    if (ch == '\'') {
      quoted += "'\\''";
    } else {
      quoted.push_back(ch);
    }
  }
  quoted.push_back('\'');
  return quoted;
}

std::string replace_all(std::string text, const std::string & from, const std::string & to)
{
  if (from.empty()) {
    return text;
  }
  std::size_t pos = 0;
  while ((pos = text.find(from, pos)) != std::string::npos) {
    text.replace(pos, from.size(), to);
    pos += to.size();
  }
  return text;
}

CommandResult run_shell_command(const std::string & command)
{
  // 用 shell 执行是为了兼容旧 Python 节点的 shell=True 语义，以及现场传入的复合播放器命令。
  const std::string wrapped = command + " 2>&1";
  std::array<char, 4096> buffer{};
  CommandResult result;

  FILE * pipe = popen(wrapped.c_str(), "r");
  if (pipe == nullptr) {
    result.return_code = -1;
    result.stderr_text = "popen failed";
    return result;
  }

  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    result.stdout_text += buffer.data();
  }

  const int status = pclose(pipe);
  if (status == -1) {
    result.return_code = -1;
  } else if (WIFEXITED(status)) {
    result.return_code = WEXITSTATUS(status);
  } else {
    result.return_code = status;
  }
  return result;
}

CommandResult run_command(const std::vector<std::string> & args)
{
  if (args.empty()) {
    return CommandResult{-1, "", "empty command"};
  }
  std::string command;
  for (const auto & arg : args) {
    if (!command.empty()) {
      command.push_back(' ');
    }
    command += shell_quote(arg);
  }
  return run_shell_command(command);
}

void write_u16_le(std::ofstream & file, std::uint16_t value)
{
  file.put(static_cast<char>(value & 0xff));
  file.put(static_cast<char>((value >> 8) & 0xff));
}

void write_u32_le(std::ofstream & file, std::uint32_t value)
{
  file.put(static_cast<char>(value & 0xff));
  file.put(static_cast<char>((value >> 8) & 0xff));
  file.put(static_cast<char>((value >> 16) & 0xff));
  file.put(static_cast<char>((value >> 24) & 0xff));
}

void write_test_wav(const std::filesystem::path & path)
{
  constexpr int sample_rate = 16000;
  constexpr double duration_sec = 0.35;
  constexpr int sample_count = static_cast<int>(sample_rate * duration_sec);
  constexpr int bits_per_sample = 16;
  constexpr int channels = 1;
  constexpr int byte_rate = sample_rate * channels * bits_per_sample / 8;
  constexpr int block_align = channels * bits_per_sample / 8;
  constexpr int data_size = sample_count * block_align;

  std::ofstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to create test wav: " + path.string());
  }

  file.write("RIFF", 4);
  write_u32_le(file, 36 + data_size);
  file.write("WAVE", 4);
  file.write("fmt ", 4);
  write_u32_le(file, 16);
  write_u16_le(file, 1);
  write_u16_le(file, channels);
  write_u32_le(file, sample_rate);
  write_u32_le(file, byte_rate);
  write_u16_le(file, block_align);
  write_u16_le(file, bits_per_sample);
  file.write("data", 4);
  write_u32_le(file, data_size);

  for (int i = 0; i < sample_count; ++i) {
    const double phase = 2.0 * M_PI * 880.0 * static_cast<double>(i) / sample_rate;
    const auto sample = static_cast<std::int16_t>(12000.0 * std::sin(phase));
    write_u16_le(file, static_cast<std::uint16_t>(sample));
  }
}

}  // namespace

class SpeakerSelector
{
public:
  explicit SpeakerSelector(rclcpp::Logger logger)
  : logger_(logger)
  {
  }

  AudioDevice select(
    const std::string & forced_backend,
    const std::string & forced_sink,
    const std::string & forced_alsa)
  {
    const std::string backend = lower_copy(trim(forced_backend.empty() ? "auto" : forced_backend));
    const std::string sink = trim(forced_sink);
    const std::string alsa = trim(forced_alsa);

    // 1. 明确指定 ALSA 设备时直接使用，保持旧节点的环境变量优先级。
    if ((backend == "alsa" || backend == "auto") && !alsa.empty() && lower_copy(alsa) != "auto") {
      return AudioDevice{"alsa", alsa, "forced ALSA device " + alsa, alsa};
    }

    // 2. PipeWire/Pulse 优先：先匹配指定 sink，再使用默认 sink，最后按名称评分。
    if (backend == "pipewire" || backend == "pulse" || backend == "auto") {
      const auto sinks = list_pulse_sinks();
      if (!sink.empty() && lower_copy(sink) != "auto") {
        const auto it = std::find_if(sinks.begin(), sinks.end(), [&](const AudioDevice & item) {
          return item.name.find(sink) != std::string::npos ||
                 item.description.find(sink) != std::string::npos;
        });
        if (it != sinks.end()) {
          return *it;
        }
        return AudioDevice{"pipewire", sink, "forced sink " + sink, ""};
      }

      const std::string default_sink = pulse_default_sink();
      if (!default_sink.empty()) {
        const auto it = std::find_if(sinks.begin(), sinks.end(), [&](const AudioDevice & item) {
          return item.name == default_sink;
        });
        if (it != sinks.end()) {
          return *it;
        }
      }

      const auto preferred = choose_preferred(sinks);
      if (preferred.has_value()) {
        return preferred.value();
      }
    }

    // 3. Pulse 不可用时回退 ALSA。
    const auto alsa_devices = list_alsa_devices();
    const auto preferred_alsa = choose_preferred(alsa_devices);
    if (preferred_alsa.has_value()) {
      return preferred_alsa.value();
    }

    return AudioDevice{"none", "", "no playback device detected", ""};
  }

  void set_volume(const AudioDevice & device, int volume_percent)
  {
    const int volume = normalize_volume(volume_percent);
    if ((device.backend == "pipewire" || device.backend == "pulse") && !device.name.empty()) {
      const auto result = run_command({"pactl", "set-sink-volume", device.name, std::to_string(volume) + "%"});
      if (result.return_code != 0) {
        throw std::runtime_error(trim(result.stdout_text).empty() ? "pactl set-sink-volume failed" : trim(result.stdout_text));
      }
      return;
    }

    if (device.backend == "alsa" && !device.alsa_device.empty()) {
      // 对齐旧 Python 节点：ALSA 音量设置失败不抛错，因为部分 USB 声卡没有 Master mixer。
      (void)run_command({"amixer", "-D", device.alsa_device, "sset", "Master", std::to_string(volume) + "%"});
    }
  }

private:
  std::vector<AudioDevice> list_pulse_sinks()
  {
    const auto result = run_command({"pactl", "list", "short", "sinks"});
    if (result.return_code != 0) {
      return {};
    }

    std::vector<AudioDevice> sinks;
    for (const auto & line : split_lines(result.stdout_text)) {
      const auto parts = split_tab(line);
      if (parts.size() < 2) {
        continue;
      }
      const std::string name = trim(parts[1]);
      std::string description;
      for (std::size_t i = 1; i < parts.size(); ++i) {
        if (!description.empty()) {
          description.push_back(' ');
        }
        description += trim(parts[i]);
      }
      sinks.push_back(AudioDevice{"pipewire", name, description, ""});
    }
    return sinks;
  }

  std::string pulse_default_sink()
  {
    const auto result = run_command({"pactl", "get-default-sink"});
    if (result.return_code != 0) {
      return "";
    }
    return trim(result.stdout_text);
  }

  std::vector<AudioDevice> list_alsa_devices()
  {
    const auto result = run_command({"aplay", "-l"});
    if (result.return_code != 0) {
      return {};
    }

    std::vector<AudioDevice> devices;
    for (const auto & line : split_lines(result.stdout_text)) {
      const std::string stripped = trim(line);
      if (stripped.rfind("card ", 0) != 0 || stripped.find("device ") == std::string::npos) {
        continue;
      }

      try {
        const auto colon_pos = stripped.find(':');
        const auto device_pos = stripped.find("device ");
        const auto device_colon = stripped.find(':', device_pos);
        if (colon_pos == std::string::npos || device_pos == std::string::npos ||
          device_colon == std::string::npos)
        {
          continue;
        }
        const std::string card_id = trim(stripped.substr(5, colon_pos - 5));
        const std::string device_id = trim(stripped.substr(device_pos + 7, device_colon - (device_pos + 7)));
        const std::string name = "plughw:" + card_id + "," + device_id;
        devices.push_back(AudioDevice{"alsa", name, stripped, name});
      } catch (const std::exception & ex) {
        RCLCPP_DEBUG(logger_, "忽略 ALSA 设备行: %s (%s)", stripped.c_str(), ex.what());
      }
    }
    return devices;
  }

  std::optional<AudioDevice> choose_preferred(const std::vector<AudioDevice> & devices)
  {
    if (devices.empty()) {
      return std::nullopt;
    }
    return *std::max_element(devices.begin(), devices.end(), [](const AudioDevice & lhs, const AudioDevice & rhs) {
      return score_audio_name(lhs.description) < score_audio_name(rhs.description);
    });
  }

  static int score_audio_name(const std::string & value)
  {
    const std::string text = lower_copy(value);
    int score = 0;
    if (text.find("usb") != std::string::npos) {
      score += 50;
    }
    if (text.find("analog") != std::string::npos || text.find("模拟") != std::string::npos) {
      score += 30;
    }
    if (text.find("speaker") != std::string::npos || text.find("audio") != std::string::npos ||
      text.find("声") != std::string::npos)
    {
      score += 10;
    }
    if (text.find("hdmi") != std::string::npos) {
      score -= 40;
    }
    return score;
  }

  rclcpp::Logger logger_;
};

class BroadcastPlayer
{
public:
  explicit BroadcastPlayer(rclcpp::Logger logger)
  : logger_(logger)
  {
  }

  std::string play(
    const std::string & text,
    int volume_percent,
    const AudioDevice & device,
    const std::string & broadcast_id,
    const std::string & waypoint_id,
    const std::string & mode,
    const std::string & command_template,
    double dry_run_sec,
    bool log_command_error)
  {
    const std::string normalized_mode = lower_copy(trim(mode));
    if (normalized_mode.empty() || normalized_mode == "dry_run" || normalized_mode == "none") {
      std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, dry_run_sec)));
      return "dry-run playback completed";
    }

    if (normalized_mode == "command" && !trim(command_template).empty()) {
      run_external_command(
        command_template, text, volume_percent, device, broadcast_id, waypoint_id, log_command_error);
      return "external playback command completed";
    }

    if (normalized_mode == "beep") {
      play_beep(device);
      return "test beep completed";
    }

    throw std::runtime_error("unsupported broadcast player mode: " + normalized_mode);
  }

private:
  void run_external_command(
    const std::string & command_template,
    const std::string & text,
    int volume_percent,
    const AudioDevice & device,
    const std::string & broadcast_id,
    const std::string & waypoint_id,
    bool log_command_error)
  {
    // 先做模板替换，再额外注入环境变量，兼容旧脚本和未来自定义播放器两种写法。
    std::string command = command_template;
    command = replace_all(command, "{{text}}", text);
    command = replace_all(command, "{{volumePercent}}", std::to_string(volume_percent));
    command = replace_all(command, "{{broadcastId}}", broadcast_id);
    command = replace_all(command, "{{waypointId}}", waypoint_id);
    command = replace_all(command, "{{selectedDevice}}", device.name);
    command = replace_all(command, "{{backend}}", device.backend);

    std::ostringstream env_prefix;
    env_prefix
      << "XIAORUI_BROADCAST_TEXT=" << shell_quote(text) << " "
      << "XIAORUI_BROADCAST_VOLUME=" << shell_quote(std::to_string(volume_percent)) << " "
      << "XIAORUI_BROADCAST_ID=" << shell_quote(broadcast_id) << " "
      << "XIAORUI_BROADCAST_WAYPOINT_ID=" << shell_quote(waypoint_id) << " "
      << "XIAORUI_AUDIO_SELECTED_DEVICE=" << shell_quote(device.name) << " "
      << "XIAORUI_AUDIO_BACKEND_SELECTED=" << shell_quote(device.backend) << " ";

    const auto result = run_shell_command(env_prefix.str() + command);
    if (result.return_code != 0) {
      const std::string reason = trim(result.stdout_text).empty() ?
        ("exit code " + std::to_string(result.return_code)) :
        trim(result.stdout_text);
      if (log_command_error) {
        RCLCPP_WARN(logger_, "播报外部命令失败: %s", reason.c_str());
      }
      throw std::runtime_error(reason);
    }
  }

  void play_beep(const AudioDevice & device)
  {
    const auto tmp_base = std::filesystem::temp_directory_path();
    const auto unique = "xiaorui_broadcast_" + std::to_string(::getpid()) + "_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const auto wav_path = tmp_base / (unique + ".wav");

    try {
      write_test_wav(wav_path);
      CommandResult result;
      if (device.backend == "alsa" && !device.alsa_device.empty()) {
        result = run_command({"aplay", "-D", device.alsa_device, wav_path.string()});
      } else {
        result = run_command({"ffplay", "-nodisp", "-autoexit", "-loglevel", "error", wav_path.string()});
      }
      std::filesystem::remove(wav_path);
      if (result.return_code != 0) {
        throw std::runtime_error(trim(result.stdout_text).empty() ? "beep playback failed" : trim(result.stdout_text));
      }
    } catch (...) {
      std::error_code ec;
      std::filesystem::remove(wav_path, ec);
      throw;
    }
  }

  rclcpp::Logger logger_;
};

class BroadcastServiceNode : public rclcpp::Node
{
public:
  BroadcastServiceNode()
  : Node("xiaorui_broadcast_service"),
    selector_(get_logger()),
    player_(get_logger())
  {
    load_parameters();
    current_volume_percent_ = normalize_volume(default_volume_percent_);
    selected_device_ = selector_.select(audio_backend_, audio_sink_, alsa_device_);

    // 三个服务名保持不变，这是上下游协议兼容的关键点。
    play_srv_ = create_service<humanoid_interfaces::srv::PlayBroadcast>(
      "/xiaorui_broadcast/play",
      std::bind(&BroadcastServiceNode::handle_play, this, std::placeholders::_1, std::placeholders::_2));
    set_volume_srv_ = create_service<humanoid_interfaces::srv::SetBroadcastVolume>(
      "/xiaorui_broadcast/set_volume",
      std::bind(&BroadcastServiceNode::handle_set_volume, this, std::placeholders::_1, std::placeholders::_2));
    health_srv_ = create_service<humanoid_interfaces::srv::GetBroadcastHealth>(
      "/xiaorui_broadcast/health",
      std::bind(&BroadcastServiceNode::handle_health, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "broadcast service ready: backend=%s, device=%s",
      selected_device_.backend.c_str(),
      selected_device_.description.c_str());
  }

private:
  void load_parameters()
  {
    // YAML 用于产品默认配置；环境变量用于现场临时覆盖。旧 Python 节点完全依赖环境变量，
    // 因此这里在声明参数后再次读取环境变量，让启动脚本的覆盖语义保持一致。
    audio_backend_ = declare_parameter<std::string>("audio_backend", "auto");
    audio_sink_ = declare_parameter<std::string>("audio_sink", "auto");
    alsa_device_ = declare_parameter<std::string>("alsa_device", "auto");
    player_mode_ = declare_parameter<std::string>("player_mode", "dry_run");
    player_command_ = declare_parameter<std::string>("player_command", "");
    dry_run_sec_ = declare_parameter<double>("dry_run_sec", 0.2);
    default_volume_percent_ = declare_parameter<int>("default_volume_percent", 72);
    reselect_each_request_ = declare_parameter<bool>("reselect_each_request", false);
    log_command_error_ = declare_parameter<bool>("log_command_error", true);

    if (const auto value = getenv_optional("XIAORUI_AUDIO_BACKEND"); value.has_value() && !value->empty()) {
      audio_backend_ = value.value();
    }
    if (const auto value = getenv_optional("XIAORUI_AUDIO_SINK"); value.has_value() && !value->empty()) {
      audio_sink_ = value.value();
    }
    if (const auto value = getenv_optional("XIAORUI_ALSA_DEVICE"); value.has_value() && !value->empty()) {
      alsa_device_ = value.value();
    }
    if (const auto value = getenv_optional("XIAORUI_BROADCAST_PLAYER"); value.has_value() && !value->empty()) {
      player_mode_ = value.value();
    }
    if (const auto value = getenv_optional("XIAORUI_BROADCAST_PLAYER_COMMAND"); value.has_value()) {
      player_command_ = value.value();
    }
    dry_run_sec_ = getenv_double_or("XIAORUI_BROADCAST_DRY_RUN_SEC", dry_run_sec_);
    default_volume_percent_ = getenv_int_or("XIAORUI_BROADCAST_DEFAULT_VOLUME", default_volume_percent_);
    reselect_each_request_ = getenv_bool_or("XIAORUI_AUDIO_RESELECT_EACH_REQUEST", reselect_each_request_);
  }

  void refresh_device_if_needed()
  {
    if (selected_device_.backend == "none" || reselect_each_request_) {
      selected_device_ = selector_.select(audio_backend_, audio_sink_, alsa_device_);
    }
  }

  void handle_play(
    const std::shared_ptr<humanoid_interfaces::srv::PlayBroadcast::Request> request,
    std::shared_ptr<humanoid_interfaces::srv::PlayBroadcast::Response> response)
  {
    const auto started_at = std::chrono::steady_clock::now();
    const int volume = normalize_volume(
      request->use_request_volume ? request->volume_percent : current_volume_percent_);

    try {
      refresh_device_if_needed();
      selector_.set_volume(selected_device_, volume);
      response->message = player_.play(
        request->text,
        volume,
        selected_device_,
        request->broadcast_id,
        request->waypoint_id,
        player_mode_,
        player_command_,
        dry_run_sec_,
        log_command_error_);
      response->success = true;
    } catch (const std::exception & ex) {
      response->success = false;
      response->message = ex.what();
    }

    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at);
    response->duration_sec = std::max(0.0, elapsed.count());
    response->selected_device = selected_device_.description;
    response->backend = selected_device_.backend;
  }

  void handle_set_volume(
    const std::shared_ptr<humanoid_interfaces::srv::SetBroadcastVolume::Request> request,
    std::shared_ptr<humanoid_interfaces::srv::SetBroadcastVolume::Response> response)
  {
    const int volume = normalize_volume(request->volume_percent);
    try {
      refresh_device_if_needed();
      selector_.set_volume(selected_device_, volume);
      current_volume_percent_ = volume;
      response->success = true;
      response->message = "broadcast volume applied";
    } catch (const std::exception & ex) {
      response->success = false;
      response->message = ex.what();
    }

    response->applied_volume_percent = current_volume_percent_;
    response->selected_device = selected_device_.description;
    response->backend = selected_device_.backend;
  }

  void handle_health(
    const std::shared_ptr<humanoid_interfaces::srv::GetBroadcastHealth::Request> request,
    std::shared_ptr<humanoid_interfaces::srv::GetBroadcastHealth::Response> response)
  {
    (void)request;
    refresh_device_if_needed();
    response->ready = selected_device_.backend != "none";
    response->selected_device = selected_device_.description;
    response->backend = selected_device_.backend;
    response->message = response->ready ? "ready" : "no playback device detected";
    response->current_volume_percent = current_volume_percent_;
  }

  std::string audio_backend_;
  std::string audio_sink_;
  std::string alsa_device_;
  std::string player_mode_;
  std::string player_command_;
  double dry_run_sec_{0.2};
  int default_volume_percent_{72};
  int current_volume_percent_{72};
  bool reselect_each_request_{false};
  bool log_command_error_{true};
  AudioDevice selected_device_;

  SpeakerSelector selector_;
  BroadcastPlayer player_;

  rclcpp::Service<humanoid_interfaces::srv::PlayBroadcast>::SharedPtr play_srv_;
  rclcpp::Service<humanoid_interfaces::srv::SetBroadcastVolume>::SharedPtr set_volume_srv_;
  rclcpp::Service<humanoid_interfaces::srv::GetBroadcastHealth>::SharedPtr health_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BroadcastServiceNode>());
  rclcpp::shutdown();
  return 0;
}
