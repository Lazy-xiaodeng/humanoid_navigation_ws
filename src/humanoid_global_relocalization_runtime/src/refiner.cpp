/*
 * refiner.cpp
 *
 * 文件作用：
 *   1. 实现 ICP/GICP/NDT 三种精配准方法。
 *   2. 对 3D-BBS top-K 粗候选逐个局部优化，并按 fitness 选择最终位姿。
 *   3. 为离线评估和在线 recovery 候选提供同一套精配准能力。
 */

#include "humanoid_global_relocalization_runtime/refiner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

namespace humanoid_global_relocalization
{
namespace
{

double elapsed_ms(
  const std::chrono::steady_clock::time_point & start,
  const std::chrono::steady_clock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

}  // namespace

RefineOutput refine_single_candidate(
  const CloudPtr & map_cloud,
  const CloudPtr & scan_cloud,
  const BbsCandidate & candidate,
  int rank,
  const RefineConfig & config)
{
  RefineOutput output;
  output.pose = candidate.pose;
  output.candidate_rank = rank;

  if (config.method == RefineMethod::None) {
    output.fitness_score = 0.0;
    output.converged = true;
    return output;
  }

  const auto start = std::chrono::steady_clock::now();
  const Eigen::Matrix4f initial_pose = candidate.pose.cast<float>();
  Cloud aligned;

  // PCL 三种精配准后端都使用同一个输入/目标定义：
  // source=当前 scan，target=预建地图，initial guess=3D-BBS 给出的 map->base 粗位姿。
  if (config.method == RefineMethod::Icp) {
    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setInputTarget(map_cloud);
    icp.setInputSource(scan_cloud);
    icp.setMaximumIterations(config.max_iterations);
    icp.setMaxCorrespondenceDistance(config.max_correspondence_distance);
    icp.setTransformationEpsilon(config.transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);
    icp.align(aligned, initial_pose);
    output.converged = icp.hasConverged();
    output.fitness_score = output.converged ?
      icp.getFitnessScore(config.max_correspondence_distance) :
      std::numeric_limits<double>::infinity();
    output.pose = output.converged ? icp.getFinalTransformation().cast<double>() : candidate.pose;
  } else if (config.method == RefineMethod::Gicp) {
    pcl::GeneralizedIterativeClosestPoint<Point, Point> gicp;
    gicp.setInputTarget(map_cloud);
    gicp.setInputSource(scan_cloud);
    gicp.setMaximumIterations(config.max_iterations);
    gicp.setMaxCorrespondenceDistance(config.max_correspondence_distance);
    gicp.setTransformationEpsilon(config.transformation_epsilon);
    gicp.setEuclideanFitnessEpsilon(config.euclidean_fitness_epsilon);
    gicp.align(aligned, initial_pose);
    output.converged = gicp.hasConverged();
    output.fitness_score = output.converged ?
      gicp.getFitnessScore(config.max_correspondence_distance) :
      std::numeric_limits<double>::infinity();
    output.pose = output.converged ? gicp.getFinalTransformation().cast<double>() : candidate.pose;
  } else if (config.method == RefineMethod::Ndt) {
    pcl::NormalDistributionsTransform<Point, Point> ndt;
    ndt.setInputTarget(map_cloud);
    ndt.setInputSource(scan_cloud);
    ndt.setMaximumIterations(config.max_iterations);
    ndt.setStepSize(config.ndt_step_size);
    ndt.setResolution(config.ndt_resolution);
    ndt.setOulierRatio(config.ndt_outlier_ratio);
    ndt.setTransformationEpsilon(config.transformation_epsilon);
    ndt.align(aligned, initial_pose);
    output.converged = ndt.hasConverged();
    output.fitness_score = output.converged ?
      ndt.getFitnessScore(config.max_correspondence_distance) :
      std::numeric_limits<double>::infinity();
    output.pose = output.converged ? ndt.getFinalTransformation().cast<double>() : candidate.pose;
  }

  output.elapsed_ms = elapsed_ms(start, std::chrono::steady_clock::now());
  return output;
}

RefineOutput refine_candidates(
  const CloudPtr & map_cloud,
  const CloudPtr & scan_cloud,
  const std::vector<BbsCandidate> & candidates,
  const RefineConfig & config)
{
  if (candidates.empty()) {
    return RefineOutput{};
  }

  const int max_candidates = config.refine_top_k ?
    std::min<int>(std::max(1, config.max_refine_candidates), static_cast<int>(candidates.size())) :
    1;

  RefineOutput best;
  best.pose = candidates.front().pose;
  best.candidate_rank = 1;
  best.fitness_score = std::numeric_limits<double>::infinity();

  // 对 top-K 前若干候选分别 refine，然后按 fitness score 重新挑最终位姿。
  // 这一步能暴露“BBS top-1 被重复结构骗了，但 top-K 中有正确解”的情况。
  double total_refine_ms = 0.0;
  for (int i = 0; i < max_candidates; ++i) {
    RefineOutput current =
      refine_single_candidate(map_cloud, scan_cloud, candidates[static_cast<std::size_t>(i)], i + 1, config);
    total_refine_ms += current.elapsed_ms;

    if (config.method == RefineMethod::None) {
      best = current;
      break;
    }
    if (current.converged && current.fitness_score < best.fitness_score) {
      best = current;
    }
  }

  if (!std::isfinite(best.fitness_score)) {
    // 所有精配准后端都未收敛时，保留 3D-BBS top-1，避免错误的未初始化位姿进入评估。
    best.pose = candidates.front().pose;
    best.candidate_rank = 1;
  }
  best.elapsed_ms = total_refine_ms;
  return best;
}

}  // namespace humanoid_global_relocalization
