/*
 * resource_monitor.cpp
 *
 * 文件作用：
 *   1. 实现进程 CPU/内存/线程数采样。
 *   2. 为离线评估 CSV 提供资源消耗证据。
 *
 * 说明：
 *   - getrusage 的 ru_maxrss 在 Linux 上单位是 KB。
 *   - /proc/self/status 中 VmRSS/VmHWM/VmSize 也是 KB，本文件统一转换为 MB。
 */

#include "humanoid_global_relocalization_runtime/resource_monitor.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <sys/resource.h>

namespace humanoid_global_relocalization
{
namespace
{

double timeval_to_ms(const timeval & value)
{
  return static_cast<double>(value.tv_sec) * 1000.0 + static_cast<double>(value.tv_usec) / 1000.0;
}

double kb_to_mb(double kb)
{
  return kb / 1024.0;
}

void parse_status_line(const std::string & line, ResourceSnapshot & snapshot)
{
  // /proc/self/status 的行格式类似 "VmRSS:\t  12345 kB"。这里只解析需要写入 CSV 的几项。
  std::istringstream input(line);
  std::string key;
  double value_kb = 0.0;
  std::string unit;
  input >> key >> value_kb >> unit;

  if (key == "VmRSS:") {
    snapshot.rss_mb = kb_to_mb(value_kb);
  } else if (key == "VmHWM:") {
    snapshot.peak_rss_mb = kb_to_mb(value_kb);
  } else if (key == "VmSize:") {
    snapshot.virtual_mem_mb = kb_to_mb(value_kb);
  } else if (key == "Threads:") {
    snapshot.thread_count = static_cast<int>(value_kb);
  }
}

}  // namespace

ResourceSnapshot sample_resource_snapshot()
{
  ResourceSnapshot snapshot;

  rusage usage {};
  if (getrusage(RUSAGE_SELF, &usage) == 0) {
    snapshot.user_cpu_ms = timeval_to_ms(usage.ru_utime);
    snapshot.system_cpu_ms = timeval_to_ms(usage.ru_stime);
    snapshot.peak_rss_mb = kb_to_mb(static_cast<double>(usage.ru_maxrss));
  }

  std::ifstream status_file("/proc/self/status");
  std::string line;
  while (std::getline(status_file, line)) {
    parse_status_line(line, snapshot);
  }

  return snapshot;
}

}  // namespace humanoid_global_relocalization
