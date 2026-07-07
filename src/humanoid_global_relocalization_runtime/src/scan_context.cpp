/*
 * scan_context.cpp
 *
 * 文件作用：
 *   1. 实现 Scan Context 描述子计算、循环移位匹配和 keyframe 查询。
 *   2. 支持从当前 scan 检索历史/地图 keyframe，输出可交给 GICP 的粗位姿 seed。
 *   3. 通过 shift 估计 query 与 keyframe 的相对 yaw，为任意朝向冷启动提供更合理的角度初值。
 */

#include "humanoid_global_relocalization_runtime/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace humanoid_global_relocalization
{
namespace
{

double deg_to_rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

int descriptor_index(int ring, int sector, int sectors)
{
  return ring * sectors + sector;
}

bool descriptor_has_same_shape(
  const ScanContextDescriptor & lhs,
  const ScanContextDescriptor & rhs)
{
  return lhs.rings == rhs.rings &&
    lhs.sectors == rhs.sectors &&
    lhs.values.size() == rhs.values.size() &&
    lhs.rings > 0 &&
    lhs.sectors > 0;
}

std::vector<std::string> split_csv_line(const std::string & line)
{
  // 当前 DB CSV 不允许字段内出现逗号，因此用轻量 split 即可。
  // descriptor_values 内部使用分号分隔，避免和 CSV 主字段冲突。
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    fields.push_back(item);
  }
  return fields;
}

std::vector<float> parse_descriptor_values(const std::string & text)
{
  std::vector<float> values;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, ';')) {
    if (!item.empty()) {
      values.push_back(std::stof(item));
    }
  }
  return values;
}

std::string descriptor_values_to_string(const std::vector<float> & values)
{
  std::ostringstream out;
  out.precision(6);
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ';';
    }
    out << values[i];
  }
  return out.str();
}

}  // namespace

ScanContextDescriptor compute_scan_context_descriptor(
  const CloudPtr & scan_cloud,
  const ScanContextConfig & config)
{
  // 描述子矩阵按 [ring, sector] 展平保存。每个 bin 记录“占据 + 少量高度信息”，
  // 这样既能保留 Scan Context 的形状召回能力，又不会让高墙/天花板高度完全主导匹配。
  ScanContextDescriptor descriptor;
  descriptor.rings = std::max(1, config.rings);
  descriptor.sectors = std::max(1, config.sectors);
  descriptor.values.assign(
    static_cast<std::size_t>(descriptor.rings * descriptor.sectors),
    0.0F);

  if (!scan_cloud || scan_cloud->empty() || config.max_radius <= 1e-6) {
    return descriptor;
  }

  for (const auto & point : scan_cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const double radius = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    if (radius < config.min_range || radius > config.max_radius ||
      point.z < config.min_z || point.z > config.max_z)
    {
      continue;
    }

    const double theta = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x)) + M_PI;
    const int ring = std::min(
      descriptor.rings - 1,
      std::max(0, static_cast<int>(std::floor(radius / config.max_radius * descriptor.rings))));
    const int sector = std::min(
      descriptor.sectors - 1,
      std::max(0, static_cast<int>(std::floor(theta / (2.0 * M_PI) * descriptor.sectors))));
    const int index = descriptor_index(ring, sector, descriptor.sectors);
    const float encoded = static_cast<float>(1.0 + 0.1 * point.z);
    descriptor.values[static_cast<std::size_t>(index)] =
      std::max(descriptor.values[static_cast<std::size_t>(index)], encoded);
  }

  return descriptor;
}

std::pair<double, int> scan_context_distance(
  const ScanContextDescriptor & query,
  const ScanContextDescriptor & candidate)
{
  if (!descriptor_has_same_shape(query, candidate)) {
    return {std::numeric_limits<double>::infinity(), 0};
  }

  double best_distance = std::numeric_limits<double>::infinity();
  int best_shift = 0;
  for (int shift = 0; shift < query.sectors; ++shift) {
    int intersection = 0;
    int uni = 0;
    double height_sum = 0.0;
    int occupied_count = 0;

    for (int ring = 0; ring < query.rings; ++ring) {
      for (int sector = 0; sector < query.sectors; ++sector) {
        const int shifted_sector = (sector + shift) % query.sectors;
        const float q = query.values[static_cast<std::size_t>(
          descriptor_index(ring, sector, query.sectors))];
        const float c = candidate.values[static_cast<std::size_t>(
          descriptor_index(ring, shifted_sector, query.sectors))];
        const bool q_occ = q > 0.0F;
        const bool c_occ = c > 0.0F;
        if (q_occ && c_occ) {
          ++intersection;
        }
        if (q_occ || c_occ) {
          ++uni;
          ++occupied_count;
          height_sum += std::abs(static_cast<double>(q) - static_cast<double>(c));
        }
      }
    }

    if (uni <= 0) {
      continue;
    }
    const double occupancy_distance = 1.0 - static_cast<double>(intersection) / static_cast<double>(uni);
    const double height_distance = height_sum / static_cast<double>(std::max(1, occupied_count));
    const double distance = occupancy_distance + 0.05 * height_distance;
    if (distance < best_distance) {
      best_distance = distance;
      best_shift = shift;
    }
  }

  return {best_distance, best_shift};
}

std::vector<ScanContextMatch> query_scan_context_database(
  const ScanContextDescriptor & query,
  const std::vector<ScanContextEntry> & database,
  int current_cloud_index,
  const ScanContextConfig & config)
{
  std::vector<ScanContextMatch> matches;
  matches.reserve(database.size());
  for (const auto & entry : database) {
    // 离线 bag 验证时必须排除当前帧附近的 keyframe，避免把几乎同一帧 scan 放进数据库造成数据泄漏。
    if (current_cloud_index >= 0 &&
      std::abs(entry.cloud_index - current_cloud_index) <= std::max(0, config.exclude_index_radius))
    {
      continue;
    }
    const auto [distance, shift] = scan_context_distance(query, entry.descriptor);
    if (!std::isfinite(distance)) {
      continue;
    }
    ScanContextMatch match;
    match.cloud_index = entry.cloud_index;
    match.distance = distance;
    match.shift = shift;
    match.map_to_base = entry.map_to_base;
    matches.push_back(match);
  }

  std::sort(
    matches.begin(),
    matches.end(),
    [](const ScanContextMatch & lhs, const ScanContextMatch & rhs) {
      return lhs.distance < rhs.distance;
    });

  const int keep = std::min<int>(std::max(1, config.top_k), static_cast<int>(matches.size()));
  matches.resize(static_cast<std::size_t>(keep));
  for (std::size_t i = 0; i < matches.size(); ++i) {
    matches[i].rank = static_cast<int>(i) + 1;
  }
  return matches;
}

std::vector<ScanContextEntry> load_scan_context_database(const std::string & path)
{
  std::vector<ScanContextEntry> database;
  if (path.empty()) {
    return database;
  }

  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("failed to open scan context database: " + path);
  }

  std::string line;
  bool first_line = true;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    if (first_line) {
      first_line = false;
      if (line.find("cloud_index") != std::string::npos) {
        continue;
      }
    }

    const auto fields = split_csv_line(line);
    if (fields.size() < 8) {
      throw std::runtime_error("invalid scan context database row in " + path + ": " + line);
    }

    ScanContextEntry entry;
    entry.cloud_index = std::stoi(fields[0]);
    entry.map_to_base = Eigen::Matrix4d::Identity();
    entry.map_to_base(0, 3) = std::stod(fields[1]);
    entry.map_to_base(1, 3) = std::stod(fields[2]);
    entry.map_to_base(2, 3) = std::stod(fields[3]);
    const double yaw = std::stod(fields[4]) * M_PI / 180.0;
    entry.map_to_base(0, 0) = std::cos(yaw);
    entry.map_to_base(0, 1) = -std::sin(yaw);
    entry.map_to_base(1, 0) = std::sin(yaw);
    entry.map_to_base(1, 1) = std::cos(yaw);
    entry.descriptor.rings = std::stoi(fields[5]);
    entry.descriptor.sectors = std::stoi(fields[6]);
    entry.descriptor.values = parse_descriptor_values(fields[7]);
    const int expected = entry.descriptor.rings * entry.descriptor.sectors;
    if (expected <= 0 || static_cast<int>(entry.descriptor.values.size()) != expected) {
      throw std::runtime_error("scan context descriptor size mismatch in " + path);
    }
    database.push_back(std::move(entry));
  }

  return database;
}

void save_scan_context_database(
  const std::string & path,
  const std::vector<ScanContextEntry> & database)
{
  if (path.empty()) {
    return;
  }
  const std::filesystem::path output_path(path);
  if (output_path.has_parent_path()) {
    std::filesystem::create_directories(output_path.parent_path());
  }

  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("failed to write scan context database: " + path);
  }
  out << "cloud_index,x_m,y_m,z_m,yaw_deg,rings,sectors,descriptor_values\n";
  out.precision(9);
  for (const auto & entry : database) {
    out << entry.cloud_index << ","
        << entry.map_to_base(0, 3) << ","
        << entry.map_to_base(1, 3) << ","
        << entry.map_to_base(2, 3) << ","
        << yaw_from_matrix(entry.map_to_base) * 180.0 / M_PI << ","
        << entry.descriptor.rings << ","
        << entry.descriptor.sectors << ","
        << descriptor_values_to_string(entry.descriptor.values) << "\n";
  }
}

Eigen::Matrix4d apply_scan_context_yaw_shift(
  const Eigen::Matrix4d & keyframe_pose,
  int shift,
  double yaw_offset_deg,
  const ScanContextConfig & config)
{
  // shift 表示将候选描述子循环移位后最接近 query。符号在不同雷达安装和描述子定义下可能有差异，
  // 因此调用方会额外配置 yaw_offsets_deg 做小范围补偿；这里默认按“query yaw = keyframe yaw + shift_angle”构造 seed。
  const double base_yaw = yaw_from_matrix(keyframe_pose);
  const double shift_yaw = config.use_shift_yaw ?
    (2.0 * M_PI * static_cast<double>(shift) / static_cast<double>(std::max(1, config.sectors))) :
    0.0;
  const double yaw = normalize_angle(base_yaw + shift_yaw + deg_to_rad(yaw_offset_deg));

  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 0) = std::cos(yaw);
  pose(0, 1) = -std::sin(yaw);
  pose(1, 0) = std::sin(yaw);
  pose(1, 1) = std::cos(yaw);
  pose(0, 3) = keyframe_pose(0, 3);
  pose(1, 3) = keyframe_pose(1, 3);
  pose(2, 3) = keyframe_pose(2, 3);
  return pose;
}

}  // namespace humanoid_global_relocalization
