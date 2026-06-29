/*
 * path_metrics.cpp
 *
 * 文件用途：
 * 1. 实现路径长度、平滑度、复杂度、安全等级和预计耗时等纯计算逻辑。
 * 2. 对齐现有路径耗时估算、路径属性分析、平滑度、复杂度、安全等级和标准差核心公式。
 * 3. 上游：Nav2 全局路径 /plan 转换出的二维路径点。
 * 4. 下游：navigation_path 响应、导航页路径摘要和调试指标。
 */

#include "humanoid_app_gateway_runtime/path_metrics.hpp"

#include <cmath>
#include <numeric>

namespace humanoid_app_gateway_runtime
{

std::string PathMetrics::name() const { return "path_metrics"; }

double PathMetrics::estimate_path_duration(
  const double path_length,
  const double avg_speed) const
{
  return avg_speed > 0.0 ? path_length / avg_speed : 0.0;
}

PathAnalysisResult PathMetrics::analyze_path_properties(
  const std::vector<PathPoint2D> & points) const
{
  PathAnalysisResult result;
  if (points.size() < 2) {
    result.smoothness = "unknown";
    result.safety_level = "unknown";
    result.complexity = "simple";
    return result;
  }

  std::vector<double> segment_lengths;
  std::vector<double> angles;
  double max_angle_change = 0.0;

  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double dx = points[i + 1].x - points[i].x;
    const double dy = points[i + 1].y - points[i].y;
    const double segment_length = std::sqrt(dx * dx + dy * dy);

    if (segment_length > 0.0) {
      segment_lengths.push_back(segment_length);
      result.total_distance += segment_length;

      const double angle = std::atan2(dy, dx);
      angles.push_back(angle);

      double angle_diff = 0.0;
      if (angles.size() >= 2) {
        angle_diff = std::abs(angles[angles.size() - 1] - angles[angles.size() - 2]);
        while (angle_diff > M_PI) {
          angle_diff -= 2.0 * M_PI;
        }
        while (angle_diff < -M_PI) {
          angle_diff += 2.0 * M_PI;
        }
        angle_diff = std::abs(angle_diff);
      }

      if (angle_diff > M_PI / 6.0) {
        result.turn_count += 1;
      }
      max_angle_change = std::max(max_angle_change, angle_diff);
    }
  }

  result.avg_segment_length = segment_lengths.empty() ?
    0.0 :
    std::accumulate(segment_lengths.begin(), segment_lengths.end(), 0.0) /
      static_cast<double>(segment_lengths.size());
  result.smoothness = calculate_smoothness(segment_lengths, angles);
  result.complexity = calculate_complexity(
    static_cast<int>(points.size()),
    result.turn_count,
    result.total_distance,
    result.avg_segment_length);
  result.safety_level = calculate_safety_level(
    segment_lengths,
    result.turn_count,
    result.avg_segment_length);
  result.max_angle_change_deg = max_angle_change > 0.0 ? max_angle_change * 180.0 / M_PI : 0.0;
  result.segment_count = static_cast<int>(segment_lengths.size());
  return result;
}

double PathMetrics::calculate_std(const std::vector<double> & values) const
{
  if (values.empty()) {
    return 0.0;
  }
  const double mean =
    std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  double variance = 0.0;
  for (const auto value : values) {
    const double diff = value - mean;
    variance += diff * diff;
  }
  variance /= static_cast<double>(values.size());
  return std::sqrt(variance);
}

std::string PathMetrics::calculate_smoothness(
  const std::vector<double> & segment_lengths,
  const std::vector<double> & angles) const
{
  if (segment_lengths.empty() || segment_lengths.size() < 2) {
    return "unknown";
  }

  const double length_std = calculate_std(segment_lengths);
  std::vector<double> angle_changes;
  for (std::size_t i = 1; i < angles.size(); ++i) {
    double angle_diff = std::abs(angles[i] - angles[i - 1]);
    while (angle_diff > M_PI) {
      angle_diff -= 2.0 * M_PI;
    }
    while (angle_diff < -M_PI) {
      angle_diff += 2.0 * M_PI;
    }
    angle_changes.push_back(std::abs(angle_diff));
  }

  const double angle_std = angle_changes.empty() ? 0.0 : calculate_std(angle_changes);
  const double length_threshold = 0.5;
  const double angle_threshold = 20.0 * M_PI / 180.0;

  if (length_std < length_threshold && (angle_changes.empty() || angle_std < angle_threshold)) {
    return "smooth";
  }
  if (length_std < length_threshold * 2.0 &&
    (angle_changes.empty() || angle_std < angle_threshold * 2.0))
  {
    return "moderate";
  }
  return "rough";
}

std::string PathMetrics::calculate_complexity(
  const int point_count,
  const int turn_count,
  const double total_distance,
  const double avg_segment_length) const
{
  int complexity_score = 0;
  if (point_count > 50) {
    complexity_score += 3;
  } else if (point_count > 20) {
    complexity_score += 2;
  } else if (point_count > 10) {
    complexity_score += 1;
  }

  if (turn_count > 10) {
    complexity_score += 3;
  } else if (turn_count > 5) {
    complexity_score += 2;
  } else if (turn_count > 2) {
    complexity_score += 1;
  }

  if (total_distance > 50.0) {
    complexity_score += 2;
  } else if (total_distance > 20.0) {
    complexity_score += 1;
  }

  if (avg_segment_length < 0.5) {
    complexity_score += 2;
  } else if (avg_segment_length < 1.0) {
    complexity_score += 1;
  }

  if (complexity_score >= 5) {
    return "complex";
  }
  if (complexity_score >= 3) {
    return "moderate";
  }
  return "simple";
}

std::string PathMetrics::calculate_safety_level(
  const std::vector<double> & segment_lengths,
  const int turn_count,
  const double avg_segment_length) const
{
  if (segment_lengths.empty()) {
    return "unknown";
  }

  double coeff_variation = 0.0;
  if (segment_lengths.size() > 1) {
    const double mean_length =
      std::accumulate(segment_lengths.begin(), segment_lengths.end(), 0.0) /
      static_cast<double>(segment_lengths.size());
    const double std_dev = calculate_std(segment_lengths);
    coeff_variation = mean_length > 0.0 ? std_dev / mean_length : 0.0;
  }

  int safety_score = 0;
  if (coeff_variation < 0.3) {
    safety_score += 2;
  } else if (coeff_variation < 0.6) {
    safety_score += 1;
  }

  const double turn_density =
    segment_lengths.empty() ? 0.0 : static_cast<double>(turn_count) / static_cast<double>(segment_lengths.size());
  if (turn_density < 0.1) {
    safety_score += 2;
  } else if (turn_density < 0.3) {
    safety_score += 1;
  }

  if (avg_segment_length >= 1.0 && avg_segment_length <= 5.0) {
    safety_score += 2;
  } else if ((avg_segment_length >= 0.5 && avg_segment_length < 1.0) || avg_segment_length > 5.0) {
    safety_score += 1;
  }

  if (safety_score >= 5) {
    return "safe";
  }
  if (safety_score >= 3) {
    return "caution";
  }
  return "danger";
}

}  // namespace humanoid_app_gateway_runtime
