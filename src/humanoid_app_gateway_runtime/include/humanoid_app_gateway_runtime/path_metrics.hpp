/*
 * path_metrics.hpp
 *
 * 文件用途：
 * 1. 负责路径长度、平滑度、复杂度、安全等级和预计耗时等纯计算逻辑。
 * 2. 对齐现有路径长度、平滑度、复杂度和安全等级分析语义。
 * 3. 本模块不依赖 ROS Node，适合做独立单元测试。
 * 4. 上游：Nav2 全局路径 /plan。
 * 5. 下游：navigation_path 响应、导航页路径摘要和调试指标。
 */

#pragma once

#include <vector>
#include <string>

namespace humanoid_app_gateway_runtime
{

struct PathPoint2D
{
  double x{0.0};
  double y{0.0};
};

struct PathAnalysisResult
{
  std::string smoothness{"unknown"};
  std::string safety_level{"unknown"};
  std::string complexity{"unknown"};
  int turn_count{0};
  double total_distance{0.0};
  double avg_segment_length{0.0};
  double max_angle_change_deg{0.0};
  int segment_count{0};
};

class PathMetrics
{
public:
  std::string name() const;

  // 保持线上协议 estimate_path_duration()：默认平均速度 0.5m/s。
  double estimate_path_duration(double path_length, double avg_speed = 0.5) const;

  // 保持线上协议 analyze_path_properties()：根据路径点计算平滑度、复杂度、安全等级。
  PathAnalysisResult analyze_path_properties(const std::vector<PathPoint2D> & points) const;

  // 保持线上协议 calculate_std()。
  double calculate_std(const std::vector<double> & values) const;

  std::string calculate_smoothness(
    const std::vector<double> & segment_lengths,
    const std::vector<double> & angles) const;

  std::string calculate_complexity(
    int point_count,
    int turn_count,
    double total_distance,
    double avg_segment_length) const;

  std::string calculate_safety_level(
    const std::vector<double> & segment_lengths,
    int turn_count,
    double avg_segment_length) const;
};

}  // namespace humanoid_app_gateway_runtime
