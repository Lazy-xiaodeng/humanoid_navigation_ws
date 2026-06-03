#ifndef HUMANOID_DECAY_OBSTACLE_LAYER__DECAY_OBSTACLE_LAYER_HPP_
#define HUMANOID_DECAY_OBSTACLE_LAYER__DECAY_OBSTACLE_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

namespace humanoid_decay_obstacle_layer
{

class DecayObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  DecayObstacleLayer();
  ~DecayObstacleLayer() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return true;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void matchSize() override;

private:
  struct CellStamp
  {
    rclcpp::Time stamp;
    unsigned int mx;
    unsigned int my;
  };

  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void removeExpiredLocked(const rclcpp::Time & now);
  void touchCellBounds(unsigned int mx, unsigned int my);
  unsigned int cellIndex(unsigned int mx, unsigned int my) const;
  void clearLayer();

  std::recursive_mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::unordered_map<unsigned int, CellStamp> active_cells_;
  std::vector<CellStamp> expired_cells_;

  std::string topic_;
  std::string global_frame_;
  double decay_time_sec_;
  double obstacle_min_range_;
  double obstacle_max_range_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  int min_hits_per_cell_;
  unsigned char obstacle_cost_;
};

}  // namespace humanoid_decay_obstacle_layer

#endif  // HUMANOID_DECAY_OBSTACLE_LAYER__DECAY_OBSTACLE_LAYER_HPP_
