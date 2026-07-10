/*
 * offline_eval_main.cpp
 *
 * 文件作用：
 *   1. 提供 global_relocalization_offline_eval 命令行入口。
 *   2. 从 --config 指定的 YAML 加载参数，执行单帧 PCD 离线评估。
 *   3. 将关键结果打印到终端，详细指标写入 output_dir 下的 CSV。
 *
 * 使用方式：
 *   ros2 run humanoid_global_relocalization_runtime global_relocalization_offline_eval \
 *     --config src/humanoid_global_relocalization_runtime/config/relocalization_validation.yaml
 */

#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "humanoid_global_relocalization_runtime/config.hpp"
#include "humanoid_global_relocalization_runtime/evaluator.hpp"

namespace
{

std::string parse_config_path(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
      return argv[i + 1];
    }
  }
  return "src/humanoid_global_relocalization_runtime/config/relocalization_validation.yaml";
}

}  // namespace

void write_summary_csv(
  const humanoid_global_relocalization::RuntimeConfig & config,
  const std::vector<humanoid_global_relocalization::EvaluationSummary> & summaries)
{
  // 汇总 CSV 记录整次离线运行的成功率。逐帧 metrics CSV 适合排查单帧问题；
  // summary CSV 适合快速比较不同 bag、不同输入话题、不同参数组合的整体效果。
  std::filesystem::create_directories(config.input.output_dir);
  const std::filesystem::path csv_path =
    std::filesystem::path(config.input.output_dir) / config.evaluation.summary_csv_name;
  const bool write_header = !std::filesystem::exists(csv_path);

  std::ofstream out(csv_path, std::ios::app);
  if (!out) {
    throw std::runtime_error("failed to open summary csv: " + csv_path.string());
  }

  std::size_t localized_count = 0;
  std::size_t success_count = 0;
  std::size_t reference_count = 0;
  for (const auto & summary : summaries) {
    if (summary.bbs_result.localized) {
      ++localized_count;
    }
    if (summary.success) {
      ++success_count;
    }
    if (summary.has_reference_pose) {
      ++reference_count;
    }
  }

  const double total = static_cast<double>(summaries.size());
  const double localized_rate = total > 0.0 ? static_cast<double>(localized_count) / total : 0.0;
  const double success_rate = total > 0.0 ? static_cast<double>(success_count) / total : 0.0;
  const double reference_rate = total > 0.0 ? static_cast<double>(reference_count) / total : 0.0;
  const double success_over_reference_rate =
    reference_count > 0 ? static_cast<double>(success_count) / static_cast<double>(reference_count) : 0.0;

  if (write_header) {
    out
      << "map_path,refine_method,input_mode,total_frames,localized_frames,success_frames,reference_frames,"
      << "localized_rate,success_rate,reference_rate,success_over_reference_rate,"
      << "scenario_count,metrics_csv,candidates_csv\n";
  }

  out << "\"ALL\"" << ","
      << "ALL" << ","
      << humanoid_global_relocalization::to_string(config.input.mode) << ","
      << summaries.size() << ","
      << localized_count << ","
      << success_count << ","
      << reference_count << ","
      << std::fixed << std::setprecision(6) << localized_rate << ","
      << success_rate << ","
      << reference_rate << ","
      << success_over_reference_rate << ","
      << config.scenarios.size() << ","
      << config.evaluation.metrics_csv_name << ","
      << config.evaluation.candidates_csv_name << "\n";

  std::map<std::pair<std::string, std::string>, std::vector<humanoid_global_relocalization::EvaluationSummary>> groups;
  for (const auto & summary : summaries) {
    groups[{summary.map_path, humanoid_global_relocalization::to_string(summary.refine_method)}].push_back(summary);
  }

  for (const auto & item : groups) {
    const auto & group = item.second;
    std::size_t group_localized = 0;
    std::size_t group_success = 0;
    std::size_t group_reference = 0;
    for (const auto & summary : group) {
      if (summary.bbs_result.localized) {
        ++group_localized;
      }
      if (summary.success) {
        ++group_success;
      }
      if (summary.has_reference_pose) {
        ++group_reference;
      }
    }
    const double group_total = static_cast<double>(group.size());
    const double group_reference_success_rate =
      group_reference > 0 ? static_cast<double>(group_success) / static_cast<double>(group_reference) : 0.0;
    out << '"' << item.first.first << '"' << ","
        << item.first.second << ","
        << humanoid_global_relocalization::to_string(config.input.mode) << ","
        << group.size() << ","
        << group_localized << ","
        << group_success << ","
        << group_reference << ","
        << std::fixed << std::setprecision(6)
        << (group_total > 0.0 ? static_cast<double>(group_localized) / group_total : 0.0) << ","
        << (group_total > 0.0 ? static_cast<double>(group_success) / group_total : 0.0) << ","
        << (group_total > 0.0 ? static_cast<double>(group_reference) / group_total : 0.0) << ","
        << group_reference_success_rate << ","
        << config.scenarios.size() << ","
        << config.evaluation.metrics_csv_name << ","
        << config.evaluation.candidates_csv_name << "\n";
  }
}

int main(int argc, char ** argv)
{
  const std::string config_path = parse_config_path(argc, argv);

  try {
    const auto config = humanoid_global_relocalization::load_config_file(config_path);
    std::cout << "[global_relocalization_offline_eval] config: " << config_path << std::endl;
    std::cout << "[global_relocalization_offline_eval] input_mode: "
              << humanoid_global_relocalization::to_string(config.input.mode) << std::endl;

    if (config.input.single_scan_pcd_path.empty() && !config.input.bag_paths.empty()) {
      const auto summaries = humanoid_global_relocalization::run_bag_evaluation(config);
      std::size_t localized_count = 0;
      for (const auto & item : summaries) {
        if (item.bbs_result.localized) {
          ++localized_count;
        }
      }
      write_summary_csv(config, summaries);
      std::cout << "[global_relocalization_offline_eval] bag_frames: " << summaries.size()
                << " localized: " << localized_count << std::endl;
      return localized_count > 0 ? 0 : 2;
    }

    const auto summary = humanoid_global_relocalization::run_single_pcd_evaluation(config);
    if (!config.input.single_scan_pcd_path.empty()) {
      write_summary_csv(config, {summary});
    }
    std::cout << "[global_relocalization_offline_eval] message: " << summary.message << std::endl;
    std::cout << "[global_relocalization_offline_eval] localized: "
              << (summary.bbs_result.localized ? "true" : "false") << std::endl;
    if (!summary.bbs_result.candidates.empty()) {
      const auto & best = summary.bbs_result.candidates.front();
      std::cout << "[global_relocalization_offline_eval] best_score: " << best.score
                << " score_ratio: " << best.score_ratio << std::endl;
      std::cout << "[global_relocalization_offline_eval] final_pose:\n"
                << summary.final_pose << std::endl;
    }
    std::cout << "[global_relocalization_offline_eval] map_points: " << summary.map_points
              << " scan_points: " << summary.scan_points << std::endl;
    std::cout << "[global_relocalization_offline_eval] build_index_ms: "
              << summary.bbs_result.build_index_ms
              << " search_ms: " << summary.bbs_result.search_ms
              << " refine_ms: " << summary.refine_ms
              << " total_ms: " << summary.total_ms << std::endl;
    std::cout << "[global_relocalization_offline_eval] rss_mb: " << summary.resources.rss_mb
              << " peak_rss_mb: " << summary.resources.peak_rss_mb
              << " threads: " << summary.resources.thread_count << std::endl;
    std::cout << "[global_relocalization_offline_eval] delta_user_cpu_ms: "
              << summary.delta_user_cpu_ms
              << " delta_system_cpu_ms: " << summary.delta_system_cpu_ms << std::endl;
    return summary.ok ? 0 : 2;
  } catch (const std::exception & exc) {
    std::cerr << "[global_relocalization_offline_eval] failed: " << exc.what() << std::endl;
    return 1;
  }
}
