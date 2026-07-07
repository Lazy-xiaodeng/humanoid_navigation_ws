#pragma once

/*
 * resource_monitor.hpp
 *
 * 文件作用：
 *   1. 定义进程资源快照结构，用于记录全局重定位验证时的 CPU、内存和线程数。
 *   2. 暴露轻量级采样函数，离线评估在关键阶段前后调用。
 *
 * 设计说明：
 *   - CPU 时间来自 getrusage(RUSAGE_SELF)，表示当前进程累计 user/system CPU 时间。
 *   - RSS、峰值 RSS、线程数来自 /proc/self/status，比只看 top 更方便写入 CSV。
 *   - 回归验证会比较不同输入话题、不同参数和不同 bag 的资源消耗，所以资源采样是核心指标之一。
 */

#include <cstdint>

namespace humanoid_global_relocalization
{

struct ResourceSnapshot
{
  double user_cpu_ms{0.0};
  double system_cpu_ms{0.0};
  double rss_mb{0.0};
  double peak_rss_mb{0.0};
  double virtual_mem_mb{0.0};
  int thread_count{0};
};

ResourceSnapshot sample_resource_snapshot();

}  // namespace humanoid_global_relocalization
