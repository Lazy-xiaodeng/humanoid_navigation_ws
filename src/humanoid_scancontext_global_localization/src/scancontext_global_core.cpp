#include "humanoid_scancontext_global_localization/scancontext_global_core.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace humanoid_scancontext_global_localization {
namespace {
constexpr float kEmptyCell = 0.0f;
constexpr float kEpsilon = 1e-6f;

float axisValue(const pcl::PointXYZI& point, int axis)
{
  switch (axis) {
    case 0:
      return point.x;
    case 1:
      return point.y;
    case 2:
      return point.z;
    default:
      return point.x;
  }
}
}  // namespace

ScanContextGlobal::ScanContextGlobal()
: config_(Config()) {}

ScanContextGlobal::ScanContextGlobal(Config config)
: config_(config) {}

Eigen::MatrixXf ScanContextGlobal::computeDescriptor(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const
{
  Eigen::MatrixXf descriptor = Eigen::MatrixXf::Constant(
    config_.num_rings, config_.num_sectors, kEmptyCell);

  if (!cloud || cloud->empty()) {
    return descriptor;
  }

  float min_z = std::numeric_limits<float>::max();
  for (const auto& point : cloud->points) {
    const float h1 = axisValue(point, config_.horizontal_axis_1);
    const float h2 = axisValue(point, config_.horizontal_axis_2);
    const float vertical = static_cast<float>(config_.vertical_sign) *
      axisValue(point, config_.vertical_axis);
    const double range = std::hypot(h1, h2);
    if (range < config_.min_range || range > config_.max_range || !std::isfinite(vertical)) {
      continue;
    }
    min_z = std::min(min_z, vertical);
  }

  if (!std::isfinite(min_z) || min_z == std::numeric_limits<float>::max()) {
    return descriptor;
  }

  const double ring_step = config_.max_range / static_cast<double>(config_.num_rings);
  const double sector_step = 2.0 * M_PI / static_cast<double>(config_.num_sectors);

  for (const auto& point : cloud->points) {
    const float h1 = axisValue(point, config_.horizontal_axis_1);
    const float h2 = axisValue(point, config_.horizontal_axis_2);
    const float vertical = static_cast<float>(config_.vertical_sign) *
      axisValue(point, config_.vertical_axis);
    if (!std::isfinite(h1) || !std::isfinite(h2) || !std::isfinite(vertical)) {
      continue;
    }

    const double range = std::hypot(h1, h2);
    if (range < config_.min_range || range > config_.max_range) {
      continue;
    }

    double angle = std::atan2(h2, h1);
    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }

    int ring = static_cast<int>(range / ring_step);
    int sector = static_cast<int>(angle / sector_step);
    ring = std::clamp(ring, 0, config_.num_rings - 1);
    sector = std::clamp(sector, 0, config_.num_sectors - 1);

    const float relative_height = vertical - min_z;
    descriptor(ring, sector) = std::max(descriptor(ring, sector), relative_height);
  }

  return descriptor;
}

Eigen::VectorXf ScanContextGlobal::computeRingKey(const Eigen::MatrixXf& descriptor) const
{
  Eigen::VectorXf key(descriptor.rows());
  for (int row = 0; row < descriptor.rows(); ++row) {
    key(row) = descriptor.row(row).mean();
  }
  return key;
}

Eigen::VectorXf ScanContextGlobal::computeSectorKey(const Eigen::MatrixXf& descriptor) const
{
  Eigen::VectorXf key(descriptor.cols());
  for (int col = 0; col < descriptor.cols(); ++col) {
    key(col) = descriptor.col(col).mean();
  }
  return key;
}

Eigen::MatrixXf ScanContextGlobal::circularShiftColumns(
  const Eigen::MatrixXf& matrix, int shift) const
{
  const int cols = matrix.cols();
  shift = ((shift % cols) + cols) % cols;
  if (shift == 0) {
    return matrix;
  }

  Eigen::MatrixXf shifted(matrix.rows(), cols);
  shifted.leftCols(cols - shift) = matrix.rightCols(cols - shift);
  shifted.rightCols(shift) = matrix.leftCols(shift);
  return shifted;
}

float ScanContextGlobal::columnCosineDistance(
  const Eigen::MatrixXf& query, const Eigen::MatrixXf& reference) const
{
  float total_distance = 0.0f;
  int valid_columns = 0;

  for (int col = 0; col < query.cols(); ++col) {
    const Eigen::VectorXf q = query.col(col);
    const Eigen::VectorXf r = reference.col(col);
    const float q_norm = q.norm();
    const float r_norm = r.norm();
    if (q_norm < kEpsilon || r_norm < kEpsilon) {
      continue;
    }

    float cosine = q.dot(r) / (q_norm * r_norm);
    cosine = std::clamp(cosine, -1.0f, 1.0f);
    total_distance += 1.0f - cosine;
    ++valid_columns;
  }

  return valid_columns > 0 ? total_distance / static_cast<float>(valid_columns) : 1.0f;
}

std::pair<float, int> ScanContextGlobal::distanceAndYaw(
  const Eigen::MatrixXf& query, const Eigen::MatrixXf& reference) const
{
  float best_distance = std::numeric_limits<float>::max();
  int best_shift = 0;

  const int max_lateral_shift = std::max(
    0, static_cast<int>(std::round(config_.num_sectors * config_.lateral_search_ratio)));

  for (int shift = 0; shift < config_.num_sectors; ++shift) {
    const Eigen::MatrixXf shifted = circularShiftColumns(reference, shift);
    float distance = columnCosineDistance(query, shifted);

    // Lightweight Scan Context++ style lateral tolerance: check a small sector window around
    // each yaw hypothesis and keep the best structural match.
    for (int lateral = 1; lateral <= max_lateral_shift; ++lateral) {
      distance = std::min(distance, columnCosineDistance(query, circularShiftColumns(reference, shift + lateral)));
      distance = std::min(distance, columnCosineDistance(query, circularShiftColumns(reference, shift - lateral)));
    }

    if (distance < best_distance) {
      best_distance = distance;
      best_shift = shift;
    }
  }

  return {best_distance, best_shift};
}

Eigen::Matrix4f ScanContextGlobal::yawCorrection(float yaw_rad) const
{
  Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
  const int a = config_.horizontal_axis_1;
  const int b = config_.horizontal_axis_2;
  if (a < 0 || a > 2 || b < 0 || b > 2 || a == b) {
    return correction;
  }
  correction(a, a) = std::cos(yaw_rad);
  correction(a, b) = -std::sin(yaw_rad);
  correction(b, a) = std::sin(yaw_rad);
  correction(b, b) = std::cos(yaw_rad);
  return correction;
}

void ScanContextGlobal::addKeyFrame(const KeyFrame& keyframe)
{
  keyframes_.push_back(keyframe);
}

std::vector<ScanContextGlobal::Candidate> ScanContextGlobal::search(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
  int top_k,
  int ring_candidates,
  float distance_threshold) const
{
  std::vector<Candidate> results;
  if (keyframes_.empty()) {
    return results;
  }

  const Eigen::MatrixXf query_descriptor = computeDescriptor(cloud);
  const Eigen::VectorXf query_ring_key = computeRingKey(query_descriptor);

  std::vector<std::pair<float, int>> ring_scores;
  ring_scores.reserve(keyframes_.size());
  for (size_t idx = 0; idx < keyframes_.size(); ++idx) {
    ring_scores.emplace_back((query_ring_key - keyframes_[idx].ring_key).norm(), static_cast<int>(idx));
  }
  std::sort(ring_scores.begin(), ring_scores.end());

  const int num_candidates = std::min(
    static_cast<int>(ring_scores.size()), std::max(top_k, ring_candidates));
  std::vector<Candidate> scored;
  scored.reserve(num_candidates);

  for (int i = 0; i < num_candidates; ++i) {
    const int idx = ring_scores[i].second;
    const auto [distance, yaw_offset] = distanceAndYaw(query_descriptor, keyframes_[idx].descriptor);
    if (distance > distance_threshold) {
      continue;
    }

    const float yaw_rad = static_cast<float>(yaw_offset) * 2.0f * static_cast<float>(M_PI) /
      static_cast<float>(config_.num_sectors);

    Candidate candidate;
    candidate.keyframe_index = idx;
    candidate.keyframe_id = keyframes_[idx].id;
    candidate.distance = distance;
    candidate.yaw_offset = yaw_offset;
    candidate.yaw_rad = yaw_rad;
    candidate.initial_pose = keyframes_[idx].pose * yawCorrection(yaw_rad);
    scored.push_back(candidate);
  }

  std::sort(scored.begin(), scored.end(), [](const Candidate& lhs, const Candidate& rhs) {
    return lhs.distance < rhs.distance;
  });

  for (int i = 0; i < std::min(top_k, static_cast<int>(scored.size())); ++i) {
    results.push_back(scored[i]);
  }
  return results;
}

bool ScanContextGlobal::saveDatabase(const std::string& path) const
{
  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    std::cerr << "failed to open Scan Context database for writing: " << path << std::endl;
    return false;
  }

  out.write(reinterpret_cast<const char*>(&config_.num_sectors), sizeof(int));
  out.write(reinterpret_cast<const char*>(&config_.num_rings), sizeof(int));
  out.write(reinterpret_cast<const char*>(&config_.max_range), sizeof(double));
  const double reserved_lidar_height = 0.0;
  out.write(reinterpret_cast<const char*>(&reserved_lidar_height), sizeof(double));

  const size_t count = keyframes_.size();
  out.write(reinterpret_cast<const char*>(&count), sizeof(size_t));
  for (const auto& keyframe : keyframes_) {
    out.write(reinterpret_cast<const char*>(&keyframe.id), sizeof(int));
    out.write(reinterpret_cast<const char*>(keyframe.pose.data()), sizeof(float) * 16);

    const int rows = keyframe.descriptor.rows();
    const int cols = keyframe.descriptor.cols();
    out.write(reinterpret_cast<const char*>(&rows), sizeof(int));
    out.write(reinterpret_cast<const char*>(&cols), sizeof(int));
    out.write(reinterpret_cast<const char*>(keyframe.descriptor.data()), sizeof(float) * rows * cols);

    const int ring_size = keyframe.ring_key.size();
    out.write(reinterpret_cast<const char*>(&ring_size), sizeof(int));
    out.write(reinterpret_cast<const char*>(keyframe.ring_key.data()), sizeof(float) * ring_size);
  }

  return true;
}

bool ScanContextGlobal::loadDatabase(const std::string& path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    std::cerr << "failed to open Scan Context database for reading: " << path << std::endl;
    return false;
  }

  keyframes_.clear();

  double reserved_lidar_height = 0.0;
  in.read(reinterpret_cast<char*>(&config_.num_sectors), sizeof(int));
  in.read(reinterpret_cast<char*>(&config_.num_rings), sizeof(int));
  in.read(reinterpret_cast<char*>(&config_.max_range), sizeof(double));
  in.read(reinterpret_cast<char*>(&reserved_lidar_height), sizeof(double));

  size_t count = 0;
  in.read(reinterpret_cast<char*>(&count), sizeof(size_t));
  for (size_t i = 0; i < count; ++i) {
    KeyFrame keyframe;
    in.read(reinterpret_cast<char*>(&keyframe.id), sizeof(int));
    in.read(reinterpret_cast<char*>(keyframe.pose.data()), sizeof(float) * 16);

    int rows = 0;
    int cols = 0;
    in.read(reinterpret_cast<char*>(&rows), sizeof(int));
    in.read(reinterpret_cast<char*>(&cols), sizeof(int));
    keyframe.descriptor.resize(rows, cols);
    in.read(reinterpret_cast<char*>(keyframe.descriptor.data()), sizeof(float) * rows * cols);

    int ring_size = 0;
    in.read(reinterpret_cast<char*>(&ring_size), sizeof(int));
    keyframe.ring_key.resize(ring_size);
    in.read(reinterpret_cast<char*>(keyframe.ring_key.data()), sizeof(float) * ring_size);
    keyframe.sector_key = computeSectorKey(keyframe.descriptor);

    if (!in.good()) {
      std::cerr << "Scan Context database is truncated or corrupt: " << path << std::endl;
      keyframes_.clear();
      return false;
    }
    keyframes_.push_back(keyframe);
  }

  return true;
}

}  // namespace humanoid_scancontext_global_localization
