#pragma once

/*
 * bbs2d_search.hpp
 *
 * 文件作用：
 *   1. 定义 2D/2.5D BBS 全图候选召回器的 C++ 运行态接口。
 *   2. 将全局 PCD 地图投影成多高度层二维占据距离场，再对当前 scan 做 x/y/yaw 全图搜索。
 *   3. 输出 top-K 粗位姿候选，作为 3D-BBS、Scan Context 之外的深搜兜底候选源。
 *
 * 设计边界：
 *   - 该类只负责“召回候选”，不直接决定恢复是否可信。
 *   - 候选会继续进入现有 GICP、temporal consistency 和 trajectory likelihood 门控。
 *   - 运行态默认关闭，建议只在恢复状态机触发深搜或离线验证时打开。
 */

#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "humanoid_global_relocalization_runtime/config.hpp"
#include "humanoid_global_relocalization_runtime/simple_bbs3d.hpp"

namespace humanoid_global_relocalization
{

class Bbs2dSearch
{
public:
  explicit Bbs2dSearch(Bbs2dConfig config);

  void build_map_index(const std::vector<Eigen::Vector3d> & map_points, const BbsConfig & search_config);
  BbsResult localize(const std::vector<Eigen::Vector3d> & scan_points, const BbsConfig & search_config) const;

private:
  struct HeightBand
  {
    double min_z{0.0};
    double max_z{0.0};
  };

  struct GridBand
  {
    HeightBand band;
    int width{0};
    int height{0};
    double origin_x{0.0};
    double origin_y{0.0};
    double resolution{0.20};
    std::vector<std::uint16_t> distance_cells;
  };

  struct SearchSeed
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    double score{0.0};
  };

  std::vector<HeightBand> parse_height_bands() const;
  void fill_distance_field(GridBand & grid, const std::vector<Eigen::Vector3d> & map_points) const;
  double score_pose(const std::vector<Eigen::Vector3d> & scan_points, const SearchSeed & seed) const;
  std::vector<SearchSeed> search_level(
    const std::vector<Eigen::Vector3d> & scan_points,
    const std::vector<SearchSeed> & parent_seeds,
    int factor,
    const BbsConfig & search_config) const;
  bool is_duplicate_pose(const std::vector<BbsCandidate> & candidates, const Eigen::Matrix4d & pose) const;
  Eigen::Matrix4d seed_to_pose(const SearchSeed & seed) const;

  Bbs2dConfig config_;
  std::vector<GridBand> grids_;
};

}  // namespace humanoid_global_relocalization
