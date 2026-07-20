#pragma once

/*
 * evaluator.hpp
 *
 * 文件作用：
 *   1. 定义离线评估流程的输入输出结构。
 *   2. 将地图加载、scan 预处理、3D-BBS 搜索、GICP 精配准、资源统计和 CSV 输出串成完整闭环。
 *   3. 为单帧 PCD 和多 bag 抽帧评估提供统一入口。
 */

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "humanoid_global_relocalization_runtime/config.hpp"
#include "humanoid_global_relocalization_runtime/refiner.hpp"
#include "humanoid_global_relocalization_runtime/resource_monitor.hpp"
#include "humanoid_global_relocalization_runtime/simple_bbs3d.hpp"

namespace humanoid_global_relocalization
{

struct EvaluationSummary
{
  bool ok{false};
  std::string message;
  std::string map_path;
  std::string bag_path;
  int bag_frame_index{-1};
  double stamp_sec{0.0};
  RefineMethod refine_method{RefineMethod::Gicp};
  BbsResult bbs_result;
  std::vector<RefineOutput> candidate_refinements;
  Eigen::Matrix4d final_pose{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d reference_pose{Eigen::Matrix4d::Identity()};
  int refined_candidate_rank{0};
  double refine_fitness_score{-1.0};
  double refine_ms{0.0};
  double total_ms{0.0};
  double translation_error_m{-1.0};
  double yaw_error_deg{-1.0};
  bool has_reference_pose{false};
  std::string reference_source;
  std::string reference_rejected_reason;
  bool success{false};
  bool has_odom_pose{false};
  Eigen::Matrix4d odom_to_base_pose{Eigen::Matrix4d::Identity()};
  int map_points{0};
  int scan_points{0};
  ResourceSnapshot resources;
  double delta_user_cpu_ms{0.0};
  double delta_system_cpu_ms{0.0};
};

EvaluationSummary run_single_pcd_evaluation(const RuntimeConfig & config);
std::vector<EvaluationSummary> run_bag_evaluation(const RuntimeConfig & config);

}  // namespace humanoid_global_relocalization
