#ifndef GRID_MAP_COSTMAP_PLUGIN__GRID_MAP_LAYER_HPP_
#define GRID_MAP_COSTMAP_PLUGIN__GRID_MAP_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

namespace grid_map_costmap_plugin
{

class GridMapLayer : public nav2_costmap_2d::Layer
{
public:
  GridMapLayer();
  virtual ~GridMapLayer() = default;

  // Layer 接口
  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j,
    int max_i, int max_j) override;
  virtual void reset() override;
  virtual bool isClearable() override { return false; }

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

  // 将 traversability 值 [0, 1] 转换为 costmap 代价 [0, 254]
  unsigned char traversabilityToCost(float traversability) const;

  // 参数
  std::string grid_map_topic_;
  std::string layer_name_;
  float lethal_threshold_;
  float inscribed_threshold_;
  bool has_data_;

  // ★ 本地缓存的 logger 和 clock ★
  rclcpp::Logger logger_{rclcpp::get_logger("grid_map_layer")};
  rclcpp::Clock::SharedPtr clock_;

  // 数据缓存
  std::mutex data_mutex_;
  grid_map_msgs::msg::GridMap::SharedPtr latest_grid_map_;

  // ROS 接口
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
};

}  // namespace grid_map_costmap_plugin

#endif  // GRID_MAP_COSTMAP_PLUGIN__GRID_MAP_LAYER_HPP_
