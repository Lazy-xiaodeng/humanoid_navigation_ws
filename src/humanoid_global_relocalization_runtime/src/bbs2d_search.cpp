/*
 * bbs2d_search.cpp
 *
 * 文件作用：
 *   1. 实现 2D/2.5D BBS 全图候选召回器。
 *   2. 构建多高度层二维距离场，用 scan 点到地图占据的距离作为候选得分。
 *   3. 通过粗到细多分辨率搜索和 NMS 输出一组分散的高分候选，交给后续 GICP/轨迹门控确认。
 *
 * 算法流程：
 *   - 地图按 z 高度层分成若干 2D 栅格，每个栅格保存到最近占据点的近似距离。
 *   - 搜索从最粗 factor 开始全图枚举 x/y/yaw，只保留分数最高的一批 seed。
 *   - 后续更细 factor 只在上一层 seed 周围展开，避免完整细网格枚举带来过高 CPU。
 *   - 最后一层结果转成 map->base 位姿，并用平移/yaw NMS 保留分散候选。
 */

#include "humanoid_global_relocalization_runtime/bbs2d_search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "humanoid_global_relocalization_runtime/point_cloud_adapter.hpp"

namespace humanoid_global_relocalization
{
namespace
{

double deg_to_rad(double deg)
{
  return deg * M_PI / 180.0;
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

double pose_yaw(const Eigen::Matrix4d & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}

std::uint16_t saturated_distance_cell(double distance_m, double resolution)
{
  // distance field 使用 uint16_t 保存“格子距离”，既能降低内存，也足够覆盖本算法只关心的近邻范围。
  const double cells = std::max(0.0, distance_m / std::max(1e-6, resolution));
  return static_cast<std::uint16_t>(
    std::min<double>(cells, static_cast<double>(std::numeric_limits<std::uint16_t>::max())));
}

}  // namespace

Bbs2dSearch::Bbs2dSearch(Bbs2dConfig config)
: config_(std::move(config))
{
}

std::vector<Bbs2dSearch::HeightBand> Bbs2dSearch::parse_height_bands() const
{
  // YAML 用扁平数组表达高度层：[min0, max0, min1, max1, ...]。
  // 如果用户写错成奇数个数，这里只使用完整 pair，避免越界；空配置则回退到一个常用室内高度层。
  std::vector<HeightBand> bands;
  for (std::size_t i = 0; i + 1 < config_.height_bands.size(); i += 2) {
    const double min_z = std::min(config_.height_bands[i], config_.height_bands[i + 1]);
    const double max_z = std::max(config_.height_bands[i], config_.height_bands[i + 1]);
    if (max_z > min_z) {
      bands.push_back(HeightBand{min_z, max_z});
    }
  }
  if (bands.empty()) {
    bands.push_back(HeightBand{0.2, 2.2});
  }
  return bands;
}

void Bbs2dSearch::build_map_index(
  const std::vector<Eigen::Vector3d> & map_points,
  const BbsConfig & search_config)
{
  grids_.clear();
  if (!config_.enable || map_points.empty()) {
    return;
  }

  const auto bands = parse_height_bands();
  const double resolution = std::max(0.05, config_.base_resolution);
  const double min_x = search_config.search_min_xyz.x();
  const double min_y = search_config.search_min_xyz.y();
  const double max_x = search_config.search_max_xyz.x();
  const double max_y = search_config.search_max_xyz.y();
  const int width = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / resolution)) + 1);
  const int height = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / resolution)) + 1);

  grids_.reserve(bands.size());
  for (const auto & band : bands) {
    GridBand grid;
    grid.band = band;
    grid.width = width;
    grid.height = height;
    grid.origin_x = min_x;
    grid.origin_y = min_y;
    grid.resolution = resolution;
    fill_distance_field(grid, map_points);
    grids_.push_back(std::move(grid));
  }
}

void Bbs2dSearch::fill_distance_field(GridBand & grid, const std::vector<Eigen::Vector3d> & map_points) const
{
  // 这里构建的是“有限半径距离场”：只关心 distance_tolerance_m 内的地图占据。
  // 先把每个地图占据点附近 radius_cells 范围的格子写入最小平方距离，避免做全图 EDT 的复杂依赖。
  const int cell_count = std::max(0, grid.width * grid.height);
  const std::uint16_t far_value = saturated_distance_cell(
    std::max(config_.distance_tolerance_m, grid.resolution * 2.0) * 4.0,
    grid.resolution);
  grid.distance_cells.assign(static_cast<std::size_t>(cell_count), far_value);

  const int radius_cells = std::max(1, static_cast<int>(std::ceil(config_.distance_tolerance_m / grid.resolution)));
  for (const auto & point : map_points) {
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
      continue;
    }
    if (point.z() < grid.band.min_z || point.z() > grid.band.max_z) {
      continue;
    }
    const int cx = static_cast<int>(std::floor((point.x() - grid.origin_x) / grid.resolution));
    const int cy = static_cast<int>(std::floor((point.y() - grid.origin_y) / grid.resolution));
    if (cx < 0 || cy < 0 || cx >= grid.width || cy >= grid.height) {
      continue;
    }
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      const int y = cy + dy;
      if (y < 0 || y >= grid.height) {
        continue;
      }
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int x = cx + dx;
        if (x < 0 || x >= grid.width) {
          continue;
        }
        const double distance = std::hypot(
          static_cast<double>(dx) * grid.resolution,
          static_cast<double>(dy) * grid.resolution);
        if (distance > config_.distance_tolerance_m) {
          continue;
        }
        const auto encoded = saturated_distance_cell(distance, grid.resolution);
        auto & slot = grid.distance_cells[static_cast<std::size_t>(y * grid.width + x)];
        slot = std::min(slot, encoded);
      }
    }
  }
}

double Bbs2dSearch::score_pose(
  const std::vector<Eigen::Vector3d> & scan_points,
  const SearchSeed & seed) const
{
  if (scan_points.empty() || grids_.empty()) {
    return 0.0;
  }

  const double cos_yaw = std::cos(seed.yaw);
  const double sin_yaw = std::sin(seed.yaw);
  const int step = std::max(1, config_.scan_point_step);
  double score = 0.0;
  int used_points = 0;

  for (std::size_t i = 0; i < scan_points.size(); i += static_cast<std::size_t>(step)) {
    const auto & point = scan_points[i];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
      continue;
    }
    const double x = seed.x + cos_yaw * point.x() - sin_yaw * point.y();
    const double y = seed.y + sin_yaw * point.x() + cos_yaw * point.y();

    for (const auto & grid : grids_) {
      if (point.z() < grid.band.min_z || point.z() > grid.band.max_z) {
        continue;
      }
      const int gx = static_cast<int>(std::floor((x - grid.origin_x) / grid.resolution));
      const int gy = static_cast<int>(std::floor((y - grid.origin_y) / grid.resolution));
      if (gx < 0 || gy < 0 || gx >= grid.width || gy >= grid.height) {
        break;
      }
      const auto encoded = grid.distance_cells[static_cast<std::size_t>(gy * grid.width + gx)];
      const double distance = static_cast<double>(encoded) * grid.resolution;
      const double normalized = distance / std::max(config_.distance_tolerance_m, grid.resolution);
      score += std::max(0.0, 1.0 - normalized);
      ++used_points;
      break;
    }
  }

  return used_points > 0 ? score / static_cast<double>(used_points) : 0.0;
}

std::vector<Bbs2dSearch::SearchSeed> Bbs2dSearch::search_level(
  const std::vector<Eigen::Vector3d> & scan_points,
  const std::vector<SearchSeed> & parent_seeds,
  int factor,
  const BbsConfig & search_config) const
{
  const double xy_step = std::max(0.05, config_.base_resolution) * static_cast<double>(std::max(1, factor));
  const double yaw_step = deg_to_rad(std::max(0.5, config_.yaw_step_deg));
  const int keep = std::max(1, config_.level_keep_candidates);
  std::vector<SearchSeed> level_seeds;

  if (parent_seeds.empty()) {
    // 第一层是全图粗搜索。这里使用 search_min/max_xyz 约束可达地图范围，避免在地图外浪费枚举。
    for (double yaw = search_config.search_min_rpy.z(); yaw < search_config.search_max_rpy.z(); yaw += yaw_step) {
      for (double y = search_config.search_min_xyz.y(); y <= search_config.search_max_xyz.y(); y += xy_step) {
        for (double x = search_config.search_min_xyz.x(); x <= search_config.search_max_xyz.x(); x += xy_step) {
          SearchSeed seed{x, y, normalize_angle(yaw), 0.0};
          seed.score = score_pose(scan_points, seed);
          level_seeds.push_back(seed);
        }
      }
    }
  } else {
    // 细层只在上一层高分 seed 周围展开 3x3 平移邻域和 3 个 yaw 邻域。
    // 这不是严格数学 B&B 上界，但工程效果等价于“候选逐层加密”，并且能保持运行时可控。
    level_seeds.reserve(parent_seeds.size() * 27U);
    for (const auto & parent : parent_seeds) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          for (int da = -1; da <= 1; ++da) {
            SearchSeed seed;
            seed.x = parent.x + static_cast<double>(dx) * xy_step;
            seed.y = parent.y + static_cast<double>(dy) * xy_step;
            seed.yaw = normalize_angle(parent.yaw + static_cast<double>(da) * yaw_step);
            if (seed.x < search_config.search_min_xyz.x() || seed.x > search_config.search_max_xyz.x() ||
              seed.y < search_config.search_min_xyz.y() || seed.y > search_config.search_max_xyz.y())
            {
              continue;
            }
            seed.score = score_pose(scan_points, seed);
            level_seeds.push_back(seed);
          }
        }
      }
    }
  }

  std::sort(level_seeds.begin(), level_seeds.end(), [](const SearchSeed & lhs, const SearchSeed & rhs) {
    return lhs.score > rhs.score;
  });
  if (static_cast<int>(level_seeds.size()) > keep) {
    level_seeds.resize(static_cast<std::size_t>(keep));
  }
  return level_seeds;
}

Eigen::Matrix4d Bbs2dSearch::seed_to_pose(const SearchSeed & seed) const
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 0) = std::cos(seed.yaw);
  pose(0, 1) = -std::sin(seed.yaw);
  pose(1, 0) = std::sin(seed.yaw);
  pose(1, 1) = std::cos(seed.yaw);
  pose(0, 3) = seed.x;
  pose(1, 3) = seed.y;
  return pose;
}

bool Bbs2dSearch::is_duplicate_pose(
  const std::vector<BbsCandidate> & candidates,
  const Eigen::Matrix4d & pose) const
{
  for (const auto & candidate : candidates) {
    const double xy = std::hypot(candidate.pose(0, 3) - pose(0, 3), candidate.pose(1, 3) - pose(1, 3));
    const double yaw_deg = std::abs(normalize_angle(pose_yaw(candidate.pose) - pose_yaw(pose))) * 180.0 / M_PI;
    if (xy <= config_.nms_xy_m && yaw_deg <= config_.nms_yaw_deg) {
      return true;
    }
  }
  return false;
}

BbsResult Bbs2dSearch::localize(
  const std::vector<Eigen::Vector3d> & scan_points,
  const BbsConfig & search_config) const
{
  BbsResult result;
  if (!config_.enable || grids_.empty() || scan_points.empty()) {
    return result;
  }

  const auto start = std::chrono::steady_clock::now();
  std::vector<SearchSeed> seeds;
  const std::vector<int> factors = config_.pyramid_factors.empty() ?
    std::vector<int>{8, 4, 2, 1} :
    config_.pyramid_factors;
  for (const int factor : factors) {
    seeds = search_level(scan_points, seeds, factor, search_config);
    if (seeds.empty()) {
      break;
    }
  }

  for (const auto & seed : seeds) {
    const Eigen::Matrix4d pose = seed_to_pose(seed);
    if (is_duplicate_pose(result.candidates, pose)) {
      continue;
    }
    BbsCandidate candidate;
    candidate.source = "bbs2d";
    candidate.pose = pose;
    candidate.score = static_cast<int>(std::round(seed.score * 100000.0));
    candidate.score_ratio = seed.score;
    result.candidates.push_back(candidate);
    if (static_cast<int>(result.candidates.size()) >= std::max(1, config_.top_k)) {
      break;
    }
  }

  result.localized = !result.candidates.empty();
  result.search_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  return result;
}

}  // namespace humanoid_global_relocalization
