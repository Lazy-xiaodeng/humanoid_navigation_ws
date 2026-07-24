/*
 * multi_seed_verifier.cpp
 *
 * 文件作用：实现多初值候选的初始化、连续跟踪和双轨一致性裁决。
 *
 * 该实现对应离线验证中的 same-source multi-seed rolling tracker：
 * 同帧候选建立多条独立轨迹，后续帧按各自 map->odom 预测做细 GICP，
 * 最终要求两条同来源、同初始簇轨迹在滚动窗口内独立稳定并相互一致。
 */

#include "humanoid_global_relocalization_runtime/multi_seed_verifier.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <thread>
#include <tuple>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

namespace humanoid_global_relocalization
{
namespace
{

double normalized_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double xy_distance(const Eigen::Matrix4d & lhs, const Eigen::Matrix4d & rhs)
{
  return std::hypot(lhs(0, 3) - rhs(0, 3), lhs(1, 3) - rhs(1, 3));
}

double yaw_distance_deg(const Eigen::Matrix4d & lhs, const Eigen::Matrix4d & rhs)
{
  return std::abs(normalized_angle(yaw_from_matrix(lhs) - yaw_from_matrix(rhs))) * 180.0 / M_PI;
}

bool pose_close(
  const Eigen::Matrix4d & lhs,
  const Eigen::Matrix4d & rhs,
  double xy_gate,
  double yaw_gate_deg)
{
  return xy_distance(lhs, rhs) <= xy_gate && yaw_distance_deg(lhs, rhs) <= yaw_gate_deg;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  if (values.size() % 2U == 0U) {
    return 0.5 * (values[middle - 1U] + values[middle]);
  }
  return values[middle];
}

double circular_center(const std::vector<double> & angles)
{
  double sum_sin = 0.0;
  double sum_cos = 0.0;
  for (const double angle : angles) {
    sum_sin += std::sin(angle);
    sum_cos += std::cos(angle);
  }
  return std::atan2(sum_sin, sum_cos);
}

Eigen::Matrix4d planar_pose(double x, double y, double yaw)
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 0) = std::cos(yaw);
  pose(0, 1) = -std::sin(yaw);
  pose(1, 0) = std::sin(yaw);
  pose(1, 1) = std::cos(yaw);
  pose(0, 3) = x;
  pose(1, 3) = y;
  return pose;
}

Eigen::Matrix4d planarize(const Eigen::Matrix4d & value)
{
  return planar_pose(value(0, 3), value(1, 3), yaw_from_matrix(value));
}

struct RegistrationQuality
{
  double fitness{0.0};
  double rmse_m{std::numeric_limits<double>::infinity()};
};

RegistrationQuality registration_quality(
  pcl::KdTreeFLANN<Point> & tree,
  const CloudPtr & scan_cloud,
  const Eigen::Matrix4d & pose,
  double max_distance)
{
  RegistrationQuality quality;
  if (!scan_cloud || scan_cloud->empty()) {
    return quality;
  }
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  int inliers = 0;
  double squared_error = 0.0;
  const double max_squared = max_distance * max_distance;
  for (const auto & point : scan_cloud->points) {
    const Eigen::Vector4d mapped = pose * Eigen::Vector4d(point.x, point.y, point.z, 1.0);
    Point query;
    query.x = static_cast<float>(mapped.x());
    query.y = static_cast<float>(mapped.y());
    query.z = static_cast<float>(mapped.z());
    if (tree.nearestKSearch(query, 1, indices, squared_distances) > 0 &&
      squared_distances[0] <= max_squared)
    {
      ++inliers;
      squared_error += squared_distances[0];
    }
  }
  quality.fitness = static_cast<double>(inliers) / static_cast<double>(scan_cloud->size());
  quality.rmse_m = inliers > 0 ? std::sqrt(squared_error / static_cast<double>(inliers)) :
    std::numeric_limits<double>::infinity();
  return quality;
}

CloudPtr downsample(const CloudPtr & input, double leaf_size)
{
  if (!input || leaf_size <= 0.0) {
    return input;
  }
  CloudPtr output(new Cloud);
  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  filter.filter(*output);
  return output;
}

struct LocalHypothesis
{
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
  double support{0.0};
  double median_distance{std::numeric_limits<double>::infinity()};
};

LocalHypothesis score_local_hypothesis(
  pcl::KdTreeFLANN<Point> & tree,
  const CloudPtr & scan,
  const Eigen::Matrix4d & pose,
  double max_distance)
{
  LocalHypothesis hypothesis;
  hypothesis.pose = pose;
  if (!scan || scan->empty()) {
    return hypothesis;
  }
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  std::vector<double> distances;
  distances.reserve(scan->size());
  const double max_squared = max_distance * max_distance;
  for (const auto & point : scan->points) {
    const Eigen::Vector4d mapped = pose * Eigen::Vector4d(point.x, point.y, point.z, 1.0);
    Point query;
    query.x = static_cast<float>(mapped.x());
    query.y = static_cast<float>(mapped.y());
    query.z = static_cast<float>(mapped.z());
    if (tree.nearestKSearch(query, 1, indices, squared_distances) > 0 &&
      squared_distances[0] <= max_squared)
    {
      distances.push_back(std::sqrt(squared_distances[0]));
    }
  }
  hypothesis.support = static_cast<double>(distances.size()) / static_cast<double>(scan->size());
  hypothesis.median_distance = median(std::move(distances));
  return hypothesis;
}

Eigen::Matrix4d local_search_and_refine(
  pcl::KdTreeFLANN<Point> & tree,
  pcl::GeneralizedIterativeClosestPoint<Point, Point> & coarse_gicp,
  const CloudPtr & coarse_scan,
  const Eigen::Matrix4d & predicted_pose,
  const MultiSeedRecoveryConfig & config)
{
  std::vector<LocalHypothesis> hypotheses;
  const int xy_steps = std::max(
    0, static_cast<int>(std::floor(
      config.local_search_xy_radius_m / std::max(1e-6, config.local_search_xy_step_m))));
  const int yaw_steps = std::max(
    0, static_cast<int>(std::floor(
      config.local_search_yaw_radius_deg / std::max(1e-6, config.local_search_yaw_step_deg))));
  const double predicted_yaw = yaw_from_matrix(predicted_pose);
  hypotheses.reserve(static_cast<std::size_t>(
      (2 * xy_steps + 1) * (2 * xy_steps + 1) * (2 * yaw_steps + 1)));
  for (int x_index = -xy_steps; x_index <= xy_steps; ++x_index) {
    for (int y_index = -xy_steps; y_index <= xy_steps; ++y_index) {
      for (int yaw_index = -yaw_steps; yaw_index <= yaw_steps; ++yaw_index) {
        const Eigen::Matrix4d hypothesis_pose = planar_pose(
          predicted_pose(0, 3) + x_index * config.local_search_xy_step_m,
          predicted_pose(1, 3) + y_index * config.local_search_xy_step_m,
          predicted_yaw + yaw_index * config.local_search_yaw_step_deg * M_PI / 180.0);
        hypotheses.push_back(score_local_hypothesis(
            tree,
            coarse_scan,
            hypothesis_pose,
            config.local_search_score_distance_m));
      }
    }
  }
  std::stable_sort(
    hypotheses.begin(), hypotheses.end(),
    [](const LocalHypothesis & lhs, const LocalHypothesis & rhs) {
      if (lhs.support != rhs.support) {
        return lhs.support > rhs.support;
      }
      return lhs.median_distance < rhs.median_distance;
    });

  std::vector<LocalHypothesis> selected;
  for (const auto & hypothesis : hypotheses) {
    const bool duplicate = std::any_of(
      selected.begin(), selected.end(),
      [&](const LocalHypothesis & item) {
        return pose_close(
          item.pose,
          hypothesis.pose,
          config.local_search_nms_xy_m,
          config.local_search_nms_yaw_deg);
      });
    if (!duplicate) {
      selected.push_back(hypothesis);
    }
    if (static_cast<int>(selected.size()) >= std::max(1, config.local_search_refine_top_k)) {
      break;
    }
  }
  if (selected.empty()) {
    return predicted_pose;
  }

  Eigen::Matrix4d best_pose = selected.front().pose;
  RegistrationQuality best_quality;
  for (const auto & hypothesis : selected) {
    Cloud aligned;
    coarse_gicp.align(aligned, hypothesis.pose.cast<float>());
    if (!coarse_gicp.hasConverged()) {
      continue;
    }
    const Eigen::Matrix4d refined =
      planarize(coarse_gicp.getFinalTransformation().cast<double>());
    const RegistrationQuality quality = registration_quality(
      tree, coarse_scan, refined, config.local_search_refine_distance_m);
    if (quality.fitness > best_quality.fitness ||
      (quality.fitness == best_quality.fitness && quality.rmse_m < best_quality.rmse_m))
    {
      best_quality = quality;
      best_pose = refined;
    }
  }
  return best_pose;
}

struct InitialCandidate
{
  RefineOutput refined;
  std::string source;
};

struct InitialCluster
{
  std::string source;
  Eigen::Matrix4d representative{Eigen::Matrix4d::Identity()};
  std::vector<InitialCandidate> candidates;
};

struct TrackQuality
{
  bool qualified{false};
  int valid_frames{0};
  double correction_median_m{0.0};
  double correction_max_m{0.0};
  double fitness_median{0.0};
  double rmse_median_m{0.0};
  double spread_m{0.0};
  Eigen::Matrix4d map_to_odom{Eigen::Matrix4d::Identity()};
};

TrackQuality qualify_track(
  const MultiSeedTrack & track,
  const MultiSeedRecoveryConfig & config)
{
  TrackQuality quality;
  const int window = std::max(1, config.tracking_window_frames);
  if (static_cast<int>(track.samples.size()) < window) {
    return quality;
  }
  const auto begin = track.samples.end() - window;
  std::vector<double> corrections;
  std::vector<double> fitness;
  std::vector<double> rmse;
  std::vector<double> map_x;
  std::vector<double> map_y;
  std::vector<double> map_yaw;
  for (auto sample = begin; sample != track.samples.end(); ++sample) {
    if (!sample->valid || !std::isfinite(sample->fitness) || !std::isfinite(sample->rmse_m)) {
      continue;
    }
    corrections.push_back(sample->correction_xy_m);
    fitness.push_back(sample->fitness);
    rmse.push_back(sample->rmse_m);
    map_x.push_back(sample->map_to_odom(0, 3));
    map_y.push_back(sample->map_to_odom(1, 3));
    map_yaw.push_back(yaw_from_matrix(sample->map_to_odom));
  }
  quality.valid_frames = static_cast<int>(fitness.size());
  if (quality.valid_frames < std::max(1, config.tracking_min_valid_frames)) {
    return quality;
  }
  quality.correction_median_m = median(corrections);
  quality.correction_max_m = *std::max_element(corrections.begin(), corrections.end());
  quality.fitness_median = median(fitness);
  quality.rmse_median_m = median(rmse);
  const double center_x = median(map_x);
  const double center_y = median(map_y);
  const double center_yaw = circular_center(map_yaw);
  double variance_x = 0.0;
  double variance_y = 0.0;
  for (std::size_t index = 0; index < map_x.size(); ++index) {
    variance_x += (map_x[index] - center_x) * (map_x[index] - center_x);
    variance_y += (map_y[index] - center_y) * (map_y[index] - center_y);
  }
  variance_x /= static_cast<double>(map_x.size());
  variance_y /= static_cast<double>(map_y.size());
  quality.spread_m = std::hypot(std::sqrt(variance_x), std::sqrt(variance_y));
  quality.map_to_odom = planar_pose(center_x, center_y, center_yaw);
  quality.qualified =
    quality.correction_median_m <= config.max_correction_median_m &&
    quality.correction_max_m <= config.max_correction_m &&
    quality.fitness_median >= config.min_fitness_median &&
    quality.rmse_median_m <= config.max_rmse_median_m &&
    quality.spread_m <= config.max_map_odom_spread_m;
  return quality;
}

}  // namespace

MultiSeedInitializationOutput initialize_multi_seed_tracks(
  const MultiSeedInitializationInput & input,
  const MultiSeedRecoveryConfig & config)
{
  MultiSeedInitializationOutput output;
  const auto started = std::chrono::steady_clock::now();
  if (!config.enable) {
    output.reason = "disabled";
    return output;
  }
  if (!input.map_cloud || !input.scan_cloud || input.candidates.empty()) {
    output.reason = "invalid_input";
    return output;
  }

  const int candidate_count = std::min<int>(
    std::max(1, config.candidate_pool_size), static_cast<int>(input.candidates.size()));
  std::vector<InitialCandidate> refined(static_cast<std::size_t>(candidate_count));
  std::atomic<int> next_index{0};
  std::atomic<int> completed_candidates{0};
  const auto deadline = started + std::chrono::duration<double>(
    std::max(0.001, config.batch_timeout_sec));
  const int worker_count = std::min(std::max(1, config.worker_threads), candidate_count);
  std::vector<std::thread> workers;
  for (int worker = 0; worker < worker_count; ++worker) {
    workers.emplace_back([&]() {
      RefineSession session(input.map_cloud, input.scan_cloud, input.refine);
      while (true) {
        if (std::chrono::steady_clock::now() >= deadline) {
          return;
        }
        const int index = next_index.fetch_add(1);
        if (index >= candidate_count) {
          return;
        }
        refined[static_cast<std::size_t>(index)].refined =
          session.refine(input.candidates[static_cast<std::size_t>(index)], index + 1);
        refined[static_cast<std::size_t>(index)].source =
          input.candidates[static_cast<std::size_t>(index)].source;
        ++completed_candidates;
      }
    });
  }
  for (auto & worker : workers) {
    worker.join();
  }
  const auto finished = std::chrono::steady_clock::now();
  output.timed_out =
    completed_candidates.load() < candidate_count || finished >= deadline;
  if (output.timed_out) {
    output.reason = "timeout";
    output.elapsed_ms = std::chrono::duration<double, std::milli>(
      finished - started).count();
    return output;
  }

  std::vector<InitialCluster> clusters;
  for (const auto & candidate : refined) {
    if (!candidate.refined.converged || !std::isfinite(candidate.refined.fitness_score) ||
      (candidate.source != "bbs3d" && candidate.source != "solid"))
    {
      continue;
    }
    ++output.refined_candidates;
    auto cluster = std::find_if(
      clusters.begin(), clusters.end(),
      [&](const InitialCluster & item) {
        return item.source == candidate.source &&
               pose_close(
          item.representative,
          candidate.refined.pose,
          config.initial_cluster_xy_m,
          config.initial_cluster_yaw_deg);
      });
    if (cluster == clusters.end()) {
      InitialCluster item;
      item.source = candidate.source;
      item.representative = candidate.refined.pose;
      item.candidates.push_back(candidate);
      clusters.push_back(std::move(item));
    } else {
      cluster->candidates.push_back(candidate);
    }
  }
  clusters.erase(
    std::remove_if(
      clusters.begin(), clusters.end(),
      [](const InitialCluster & cluster) {return cluster.candidates.size() < 2U;}),
    clusters.end());
  std::stable_sort(
    clusters.begin(), clusters.end(),
    [](const InitialCluster & lhs, const InitialCluster & rhs) {
      if (lhs.candidates.size() != rhs.candidates.size()) {
        return lhs.candidates.size() > rhs.candidates.size();
      }
      return lhs.candidates.front().refined.candidate_rank <
             rhs.candidates.front().refined.candidate_rank;
    });
  if (static_cast<int>(clusters.size()) > std::max(1, config.max_active_clusters)) {
    clusters.resize(static_cast<std::size_t>(std::max(1, config.max_active_clusters)));
  }

  int track_id = 0;
  int cluster_id = 0;
  for (auto & cluster : clusters) {
    ++cluster_id;
    std::stable_sort(
      cluster.candidates.begin(), cluster.candidates.end(),
      [](const InitialCandidate & lhs, const InitialCandidate & rhs) {
        return lhs.refined.candidate_rank < rhs.refined.candidate_rank;
      });
    const int seeds = std::min<int>(
      std::max(1, config.max_seeds_per_cluster), static_cast<int>(cluster.candidates.size()));
    for (int index = 0; index < seeds; ++index) {
      const auto & candidate = cluster.candidates[static_cast<std::size_t>(index)];
      MultiSeedTrack track;
      track.track_id = ++track_id;
      track.cluster_id = cluster_id;
      track.candidate_rank = candidate.refined.candidate_rank;
      track.source = cluster.source;
      track.anchor_map_to_odom =
        planarize(candidate.refined.pose * input.odom_to_base.inverse());
      output.tracks.push_back(std::move(track));
    }
  }
  output.retained_clusters = cluster_id;
  output.reason = output.tracks.size() >= 2U ? "initialized" : "no_multi_seed_cluster";
  output.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return output;
}

MultiSeedTrackingOutput update_multi_seed_tracks(
  const MultiSeedTrackingInput & input,
  const MultiSeedRecoveryConfig & config)
{
  MultiSeedTrackingOutput output;
  const auto started = std::chrono::steady_clock::now();
  output.tracks = input.tracks;
  if (!input.map_cloud || !input.scan_cloud || input.tracks.empty()) {
    output.reason = "invalid_input";
    return output;
  }

  std::atomic<std::size_t> next_index{0};
  const auto deadline = started + std::chrono::duration<double>(
    std::max(0.001, config.batch_timeout_sec));
  const int worker_count = std::min<int>(
    std::max(1, config.worker_threads), static_cast<int>(output.tracks.size()));
  std::atomic<int> completed_tracks{0};
  const CloudPtr coarse_scan = downsample(input.scan_cloud, config.local_search_scan_leaf_size);
  const CloudPtr local_refine_scan =
    downsample(input.scan_cloud, config.local_search_refine_scan_leaf_size);
  std::vector<std::thread> workers;
  for (int worker = 0; worker < worker_count; ++worker) {
    workers.emplace_back([&]() {
      pcl::GeneralizedIterativeClosestPoint<Point, Point> gicp;
      gicp.setInputTarget(input.map_cloud);
      gicp.setInputSource(input.scan_cloud);
      gicp.setMaximumIterations(std::max(1, config.fine_max_iterations));
      gicp.setMaxCorrespondenceDistance(config.fine_max_correspondence_distance);
      gicp.setTransformationEpsilon(1e-7);
      gicp.setEuclideanFitnessEpsilon(1e-7);
      pcl::GeneralizedIterativeClosestPoint<Point, Point> coarse_gicp;
      coarse_gicp.setInputTarget(input.map_cloud);
      coarse_gicp.setInputSource(local_refine_scan);
      coarse_gicp.setMaximumIterations(std::max(1, config.local_search_refine_iterations));
      coarse_gicp.setMaxCorrespondenceDistance(config.local_search_refine_distance_m);
      coarse_gicp.setTransformationEpsilon(1e-5);
      coarse_gicp.setEuclideanFitnessEpsilon(1e-5);
      pcl::KdTreeFLANN<Point> quality_tree;
      quality_tree.setInputCloud(input.map_cloud);
      while (true) {
        if (std::chrono::steady_clock::now() >= deadline) {
          return;
        }
        const std::size_t index = next_index.fetch_add(1);
        if (index >= output.tracks.size()) {
          return;
        }
        auto & track = output.tracks[index];
        const Eigen::Matrix4d prediction_map_to_odom =
          track.has_tracked_map_to_odom ? track.tracked_map_to_odom : track.anchor_map_to_odom;
        const Eigen::Matrix4d predicted_pose =
          planarize(prediction_map_to_odom * input.odom_to_base);
        const Eigen::Matrix4d fine_initial = track.has_tracked_map_to_odom ?
          predicted_pose :
          local_search_and_refine(
          quality_tree, coarse_gicp, coarse_scan, predicted_pose, config);
        Cloud aligned;
        gicp.align(aligned, fine_initial.cast<float>());

        MultiSeedTrackSample sample;
        sample.pose = gicp.hasConverged() ?
          planarize(gicp.getFinalTransformation().cast<double>()) : predicted_pose;
        const RegistrationQuality quality = registration_quality(
          quality_tree,
          input.scan_cloud,
          sample.pose,
          config.fine_max_correspondence_distance);
        sample.fitness = quality.fitness;
        sample.rmse_m = quality.rmse_m;
        sample.correction_xy_m = xy_distance(sample.pose, predicted_pose);
        sample.correction_yaw_deg = yaw_distance_deg(sample.pose, predicted_pose);
        sample.map_to_odom = planarize(sample.pose * input.odom_to_base.inverse());
        sample.valid = gicp.hasConverged() &&
          sample.fitness >= config.sample_min_fitness &&
          sample.rmse_m <= config.sample_max_rmse_m;
        if (gicp.hasConverged() &&
          sample.fitness >= config.update_min_fitness &&
          sample.rmse_m <= config.update_max_rmse_m)
        {
          track.tracked_map_to_odom = sample.map_to_odom;
          track.has_tracked_map_to_odom = true;
        }
        track.samples.push_back(sample);
        while (static_cast<int>(track.samples.size()) > std::max(1, config.tracking_window_frames)) {
          track.samples.pop_front();
        }
        ++completed_tracks;
      }
    });
  }
  for (auto & worker : workers) {
    worker.join();
  }
  const auto finished = std::chrono::steady_clock::now();
  output.completed_tracks = completed_tracks.load();
  output.timed_out =
    output.completed_tracks < static_cast<int>(output.tracks.size()) || finished >= deadline;
  output.reason = output.timed_out ? "timeout" : "tracked";
  output.elapsed_ms = std::chrono::duration<double, std::milli>(
    finished - started).count();
  return output;
}

MultiSeedEvaluationOutput evaluate_multi_seed_tracks(
  const std::vector<MultiSeedTrack> & tracks,
  const Eigen::Matrix4d & current_odom_to_base,
  const MultiSeedRecoveryConfig & config)
{
  MultiSeedEvaluationOutput output;
  struct Qualified
  {
    const MultiSeedTrack * track{nullptr};
    TrackQuality quality;
  };
  std::map<std::tuple<std::string, int>, std::vector<Qualified>> groups;
  for (const auto & track : tracks) {
    TrackQuality quality = qualify_track(track, config);
    if (quality.qualified) {
      groups[{track.source, track.cluster_id}].push_back(Qualified{&track, quality});
    }
  }

  std::vector<Qualified> agreeing;
  for (const auto & [key, group] : groups) {
    (void)key;
    for (const auto & first : group) {
      std::vector<Qualified> support;
      for (const auto & second : group) {
        if (pose_close(
            first.quality.map_to_odom,
            second.quality.map_to_odom,
            config.track_agreement_xy_m,
            config.track_agreement_yaw_deg))
        {
          support.push_back(second);
        }
      }
      if (support.size() >= 2U) {
        agreeing = std::move(support);
        break;
      }
    }
    if (!agreeing.empty()) {
      break;
    }
  }
  if (agreeing.empty()) {
    output.reason = "collecting_or_no_agreement";
    return output;
  }

  const auto winner = std::min_element(
    agreeing.begin(), agreeing.end(),
    [](const Qualified & lhs, const Qualified & rhs) {
      return std::tie(
        lhs.quality.rmse_median_m,
        lhs.quality.correction_median_m,
        lhs.track->candidate_rank) <
             std::tie(
        rhs.quality.rmse_median_m,
        rhs.quality.correction_median_m,
        rhs.track->candidate_rank);
    });
  output.accepted = true;
  output.reason = "accepted";
  output.pose = planarize(winner->quality.map_to_odom * current_odom_to_base);
  output.candidate_rank = winner->track->candidate_rank;
  output.cluster_id = winner->track->cluster_id;
  output.qualified_tracks = static_cast<int>(agreeing.size());
  output.valid_frames = winner->quality.valid_frames;
  output.fitness_score = winner->quality.fitness_median;
  output.rmse_m = winner->quality.rmse_median_m;
  output.map_odom_spread_m = winner->quality.spread_m;
  return output;
}

}  // namespace humanoid_global_relocalization
