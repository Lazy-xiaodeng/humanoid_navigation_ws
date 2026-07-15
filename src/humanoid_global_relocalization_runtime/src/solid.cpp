/*
 * solid.cpp
 *
 * 文件作用：
 *   1. 实现完整 SOLiD RAH 距离/方位描述子，与离线官方算法验证保持一致。
 *   2. 使用距离描述子余弦相似度召回关键帧，并用方位描述子循环移位估计航向。
 *   3. 保存和加载建图阶段生成的 SOLiD 数据库，不参与恢复位姿的最终发布决策。
 */

#include "humanoid_global_relocalization_runtime/solid.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include <pcl/filters/voxel_grid.h>

namespace humanoid_global_relocalization
{
namespace
{

std::vector<std::string> split(const std::string & text, char delimiter)
{
  std::vector<std::string> values;
  std::stringstream stream(text);
  std::string value;
  while (std::getline(stream, value, delimiter)) {
    while (!value.empty() && (value.back() == '\r' || value.back() == '\n')) {
      value.pop_back();
    }
    values.push_back(value);
  }
  return values;
}

std::vector<float> parse_values(const std::string & text)
{
  std::vector<float> values;
  for (const auto & value : split(text, ';')) {
    if (!value.empty()) {
      values.push_back(std::stof(value));
    }
  }
  return values;
}

void write_values(std::ostream & output, const std::vector<float> & values)
{
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      output << ';';
    }
    output << values[i];
  }
}

double cosine_similarity(const std::vector<float> & lhs, const std::vector<float> & rhs)
{
  if (lhs.size() != rhs.size() || lhs.empty()) {
    return -1.0;
  }
  double dot = 0.0;
  double lhs_norm = 0.0;
  double rhs_norm = 0.0;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    dot += lhs[i] * rhs[i];
    lhs_norm += lhs[i] * lhs[i];
    rhs_norm += rhs[i] * rhs[i];
  }
  const double denominator = std::sqrt(lhs_norm * rhs_norm);
  return denominator > 1e-12 ? dot / denominator : -1.0;
}

double estimate_heading_deg(const std::vector<float> & query, const std::vector<float> & candidate)
{
  if (query.size() != candidate.size() || query.empty()) {
    return 0.0;
  }
  int best_shift = 0;
  double best_error = std::numeric_limits<double>::infinity();
  for (int shift = 0; shift < static_cast<int>(query.size()); ++shift) {
    double error = 0.0;
    for (int index = 0; index < static_cast<int>(query.size()); ++index) {
      const int shifted = (index - shift + static_cast<int>(query.size())) %
        static_cast<int>(query.size());
      error += std::abs(candidate[static_cast<std::size_t>(index)] -
        query[static_cast<std::size_t>(shifted)]);
    }
    if (error < best_error) {
      best_error = error;
      best_shift = shift;
    }
  }
  // Keep the official SOLiD implementation's one-bin convention used by validation.
  return static_cast<double>(best_shift + 1) * 360.0 / static_cast<double>(query.size());
}

}  // namespace

SolidDescriptor compute_solid_descriptor(const CloudPtr & scan_cloud, const SolidConfig & config)
{
  SolidDescriptor descriptor;
  const int ranges = std::max(1, config.ranges);
  const int angles = std::max(1, config.angles);
  const int heights = std::max(1, config.heights);
  descriptor.range.assign(static_cast<std::size_t>(ranges), 0.0F);
  descriptor.angle.assign(static_cast<std::size_t>(angles), 0.0F);
  if (!scan_cloud || scan_cloud->empty() || config.max_range <= config.min_range) {
    return descriptor;
  }

  CloudPtr cropped(new Cloud);
  cropped->reserve(scan_cloud->size());
  for (const auto & source : scan_cloud->points) {
    Point point = source;
    point.z -= static_cast<float>(config.sensor_height);
    const double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (std::isfinite(distance) && distance > config.min_range && distance < config.max_range) {
      cropped->push_back(point);
    }
  }
  Cloud cloud;
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(cropped);
  const float leaf = static_cast<float>(std::max(0.01, config.voxel_size));
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(cloud);

  std::vector<double> range_height(static_cast<std::size_t>(ranges * heights), 0.0);
  std::vector<double> angle_height(static_cast<std::size_t>(angles * heights), 0.0);
  const double range_gap = config.max_range / static_cast<double>(ranges);
  const double angle_gap = 360.0 / static_cast<double>(angles);
  const double height_gap = (config.fov_up_deg - config.fov_down_deg) / static_cast<double>(heights);
  for (const auto & point : cloud.points) {
    double theta = std::atan2(point.y, point.x) * 180.0 / M_PI;
    if (theta < 0.0) {
      theta += 360.0;
    }
    const double planar = std::hypot(point.x, point.y);
    const double elevation = std::atan2(point.z, planar) * 180.0 / M_PI;
    const int ri = std::clamp(static_cast<int>(planar / range_gap), 0, ranges - 1);
    const int ai = std::clamp(static_cast<int>(theta / angle_gap), 0, angles - 1);
    const int hi = std::clamp(
      static_cast<int>((elevation - config.fov_down_deg) / std::max(1e-6, height_gap)), 0, heights - 1);
    range_height[static_cast<std::size_t>(ri * heights + hi)] += 1.0;
    angle_height[static_cast<std::size_t>(ai * heights + hi)] += 1.0;
  }

  std::vector<double> weights(static_cast<std::size_t>(heights), 0.0);
  for (int height = 0; height < heights; ++height) {
    for (int range = 0; range < ranges; ++range) {
      weights[static_cast<std::size_t>(height)] +=
        range_height[static_cast<std::size_t>(range * heights + height)];
    }
  }
  const auto [minimum, maximum] = std::minmax_element(weights.begin(), weights.end());
  const double span = *maximum - *minimum;
  for (double & weight : weights) {
    weight = span > 0.0 ? (weight - *minimum) / span : 1.0;
  }
  for (int range = 0; range < ranges; ++range) {
    for (int height = 0; height < heights; ++height) {
      descriptor.range[static_cast<std::size_t>(range)] += static_cast<float>(
        range_height[static_cast<std::size_t>(range * heights + height)] *
        weights[static_cast<std::size_t>(height)]);
    }
  }
  for (int angle = 0; angle < angles; ++angle) {
    for (int height = 0; height < heights; ++height) {
      descriptor.angle[static_cast<std::size_t>(angle)] += static_cast<float>(
        angle_height[static_cast<std::size_t>(angle * heights + height)] *
        weights[static_cast<std::size_t>(height)]);
    }
  }
  return descriptor;
}

CloudPtr prepare_solid_registration_cloud(const CloudPtr & scan_cloud, const SolidConfig & config)
{
  CloudPtr selected(new Cloud);
  if (!scan_cloud) {
    return selected;
  }
  selected->reserve(scan_cloud->size());
  for (const auto & point : scan_cloud->points) {
    const double radius = std::hypot(point.x, point.y);
    if (std::isfinite(radius) && radius >= config.min_range && radius <= config.max_range &&
      point.z >= 0.2F && point.z <= 2.5F)
    {
      selected->push_back(point);
    }
  }
  CloudPtr output(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(selected);
  const float leaf = static_cast<float>(std::max(0.01, config.registration_voxel_size));
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*output);
  return output;
}

std::vector<SolidMatch> query_solid_database(
  const SolidDescriptor & query,
  const std::vector<SolidEntry> & database,
  int current_cloud_index,
  const SolidConfig & config)
{
  std::vector<SolidMatch> matches;
  for (const auto & entry : database) {
    if (current_cloud_index >= 0 &&
      std::abs(entry.cloud_index - current_cloud_index) <= std::max(0, config.exclude_index_radius))
    {
      continue;
    }
    const double similarity = cosine_similarity(query.range, entry.descriptor.range);
    if (similarity < config.similarity_min) {
      continue;
    }
    matches.push_back(SolidMatch{
      0, entry.cloud_index, entry.source_id, similarity,
      estimate_heading_deg(query.angle, entry.descriptor.angle), entry.map_to_base});
  }
  std::sort(matches.begin(), matches.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.similarity > rhs.similarity;
  });
  std::unordered_map<std::string, int> source_counts;
  for (const auto & match : matches) {
    source_counts.emplace(match.source_id, 0);
  }
  const bool balance_sources = config.top_k_per_source > 0 && source_counts.size() > 1U;
  if (balance_sources) {
    std::vector<SolidMatch> balanced;
    balanced.reserve(matches.size());
    for (const auto & match : matches) {
      int & count = source_counts[match.source_id];
      if (count >= config.top_k_per_source) {
        continue;
      }
      balanced.push_back(match);
      ++count;
    }
    matches = std::move(balanced);
  }
  if (!balance_sources) {
    matches.resize(std::min<std::size_t>(
      matches.size(), static_cast<std::size_t>(std::max(1, config.top_k))));
  }
  for (std::size_t i = 0; i < matches.size(); ++i) {
    matches[i].rank = static_cast<int>(i) + 1;
  }
  return matches;
}

std::vector<SolidEntry> load_solid_database(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open SOLiD database: " + path);
  }
  std::vector<SolidEntry> database;
  std::string line;
  bool header = true;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    if (header) {
      header = false;
      if (line.find("cloud_index") != std::string::npos) {
        continue;
      }
    }
    const auto fields = split(line, ',');
    if (fields.size() < 8) {
      throw std::runtime_error("invalid SOLiD database row: " + line);
    }
    SolidEntry entry;
    entry.cloud_index = std::stoi(fields[0]);
    entry.map_to_base(0, 3) = std::stod(fields[1]);
    entry.map_to_base(1, 3) = std::stod(fields[2]);
    entry.map_to_base(2, 3) = std::stod(fields[3]);
    const double yaw = std::stod(fields[4]) * M_PI / 180.0;
    entry.map_to_base(0, 0) = std::cos(yaw);
    entry.map_to_base(0, 1) = -std::sin(yaw);
    entry.map_to_base(1, 0) = std::sin(yaw);
    entry.map_to_base(1, 1) = std::cos(yaw);
    entry.descriptor.range = parse_values(fields[5]);
    entry.descriptor.angle = parse_values(fields[6]);
    entry.source_id = fields.size() >= 9 ? fields[7] : "default";
    entry.cloud_path = fields.size() >= 9 ? fields[8] : fields[7];
    std::filesystem::path cloud_path(entry.cloud_path);
    if (cloud_path.is_relative()) {
      cloud_path = std::filesystem::path(path).parent_path() / cloud_path;
    }
    entry.cloud = load_pcd_xyz(cloud_path.string());
    database.push_back(std::move(entry));
  }
  return database;
}

void save_solid_database(const std::string & path, const std::vector<SolidEntry> & database)
{
  if (path.empty()) {
    return;
  }
  const std::filesystem::path output_path(path);
  if (output_path.has_parent_path()) {
    std::filesystem::create_directories(output_path.parent_path());
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("failed to write SOLiD database: " + path);
  }
  output << std::setprecision(9);
  const std::filesystem::path cloud_directory =
    output_path.parent_path() / (output_path.stem().string() + "_clouds");
  std::filesystem::create_directories(cloud_directory);
  output << "cloud_index,x_m,y_m,z_m,yaw_deg,range_descriptor,angle_descriptor,source_id,cloud_path\n";
  for (std::size_t index = 0; index < database.size(); ++index) {
    const auto & entry = database[index];
    const std::string cloud_name = "keyframe_" + std::to_string(index) + ".pcd";
    const std::filesystem::path cloud_path = cloud_directory / cloud_name;
    if (entry.cloud && !entry.cloud->empty()) {
      save_pcd_xyz(cloud_path.string(), *entry.cloud);
    }
    output << entry.cloud_index << ',' << entry.map_to_base(0, 3) << ',' << entry.map_to_base(1, 3) << ','
           << entry.map_to_base(2, 3) << ',' << yaw_from_matrix(entry.map_to_base) * 180.0 / M_PI << ',';
    write_values(output, entry.descriptor.range);
    output << ',';
    write_values(output, entry.descriptor.angle);
    output << ',' << entry.source_id << ',' <<
      output_path.stem().string() + "_clouds" + "/" + cloud_name << '\n';
  }
}

Eigen::Matrix4d apply_solid_heading(const Eigen::Matrix4d & keyframe_pose, double relative_yaw_deg)
{
  Eigen::Matrix4d relative = Eigen::Matrix4d::Identity();
  const double yaw = relative_yaw_deg * M_PI / 180.0;
  relative(0, 0) = std::cos(yaw);
  relative(0, 1) = -std::sin(yaw);
  relative(1, 0) = std::sin(yaw);
  relative(1, 1) = std::cos(yaw);
  return keyframe_pose * relative;
}

}  // namespace humanoid_global_relocalization
