#include "humanoid_decay_obstacle_layer/decay_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <utility>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace humanoid_decay_obstacle_layer
{

DecayObstacleLayer::DecayObstacleLayer()
: decay_time_sec_(3.0),
  obstacle_min_range_(0.5),
  obstacle_max_range_(10.0),
  min_obstacle_height_(0.1),
  max_obstacle_height_(1.8),
  min_hits_per_cell_(1),
  obstacle_cost_(nav2_costmap_2d::LETHAL_OBSTACLE)
{
}

void DecayObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("DecayObstacleLayer node expired");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/airy_points_filtered")));
  declareParameter("decay_time", rclcpp::ParameterValue(3.0));
  declareParameter("obstacle_min_range", rclcpp::ParameterValue(0.5));
  declareParameter("obstacle_max_range", rclcpp::ParameterValue(10.0));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.1));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(1.8));
  declareParameter("min_hits_per_cell", rclcpp::ParameterValue(1));
  declareParameter("obstacle_cost", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".decay_time", decay_time_sec_);
  node->get_parameter(name_ + ".obstacle_min_range", obstacle_min_range_);
  node->get_parameter(name_ + ".obstacle_max_range", obstacle_max_range_);
  node->get_parameter(name_ + ".min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + ".min_hits_per_cell", min_hits_per_cell_);
  int obstacle_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
  node->get_parameter(name_ + ".obstacle_cost", obstacle_cost);
  obstacle_cost_ = static_cast<unsigned char>(
    std::clamp(obstacle_cost, 0, static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));

  if (decay_time_sec_ <= 0.0) {
    throw std::runtime_error("decay_time must be > 0");
  }
  if (obstacle_max_range_ <= obstacle_min_range_) {
    throw std::runtime_error("obstacle_max_range must be > obstacle_min_range");
  }
  if (max_obstacle_height_ <= min_obstacle_height_) {
    throw std::runtime_error("max_obstacle_height must be > min_obstacle_height");
  }
  min_hits_per_cell_ = std::max(1, min_hits_per_cell_);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  matchSize();
  current_ = true;

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&DecayObstacleLayer::cloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "DecayObstacleLayer initialized: topic=%s decay=%.2fs range=[%.2f, %.2f] "
    "height=[%.2f, %.2f] min_hits=%d enabled=%s",
    topic_.c_str(), decay_time_sec_, obstacle_min_range_, obstacle_max_range_,
    min_obstacle_height_, max_obstacle_height_, min_hits_per_cell_,
    enabled_ ? "true" : "false");
}

void DecayObstacleLayer::activate()
{
}

void DecayObstacleLayer::deactivate()
{
}

void DecayObstacleLayer::reset()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  clearLayer();
  current_ = true;
}

void DecayObstacleLayer::matchSize()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(),
    master->getSizeInCellsY(),
    master->getResolution(),
    master->getOriginX(),
    master->getOriginY());
  setDefaultValue(nav2_costmap_2d::NO_INFORMATION);
  clearLayer();
}

void DecayObstacleLayer::clearLayer()
{
  resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());
  active_cells_.clear();
  expired_cells_.clear();
}

unsigned int DecayObstacleLayer::cellIndex(unsigned int mx, unsigned int my) const
{
  return my * getSizeInCellsX() + mx;
}

void DecayObstacleLayer::touchCellBounds(unsigned int mx, unsigned int my)
{
  const double wx = getOriginX() + (static_cast<double>(mx) + 0.5) * getResolution();
  const double wy = getOriginY() + (static_cast<double>(my) + 0.5) * getResolution();
  const double half = 0.5 * getResolution();
  addExtraBounds(wx - half, wy - half, wx + half, wy + half);
}

void DecayObstacleLayer::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!enabled_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_->lookupTransform(
      global_frame_,
      msg->header.frame_id,
      msg->header.stamp,
      tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "DecayObstacleLayer TF failed from %s to %s: %s",
      msg->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    current_ = false;
    return;
  }

  tf2::Transform transform;
  tf2::fromMsg(transform_stamped.transform, transform);

  std::map<unsigned int, unsigned int> hit_counts;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const double local_x = *iter_x;
    const double local_y = *iter_y;
    const double local_z = *iter_z;
    if (!std::isfinite(local_x) || !std::isfinite(local_y) || !std::isfinite(local_z)) {
      continue;
    }

    const double range = std::hypot(local_x, local_y);
    if (range < obstacle_min_range_ || range > obstacle_max_range_) {
      continue;
    }
    if (local_z < min_obstacle_height_ || local_z > max_obstacle_height_) {
      continue;
    }

    const tf2::Vector3 p_global = transform * tf2::Vector3(local_x, local_y, local_z);
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!worldToMap(p_global.x(), p_global.y(), mx, my)) {
      continue;
    }

    const unsigned int index = cellIndex(mx, my);
    hit_counts[index] += 1;
  }

  const rclcpp::Time stamp = clock_->now();
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  removeExpiredLocked(stamp);

  for (const auto & item : hit_counts) {
    const unsigned int hits = item.second;
    if (static_cast<int>(hits) < min_hits_per_cell_) {
      continue;
    }

    const unsigned int index = item.first;
    const unsigned int mx = index % getSizeInCellsX();
    const unsigned int my = index / getSizeInCellsX();

    setCost(mx, my, obstacle_cost_);
    active_cells_[index] = CellStamp{stamp, mx, my};
    touchCellBounds(mx, my);
  }

  current_ = true;
}

void DecayObstacleLayer::removeExpiredLocked(const rclcpp::Time & now)
{
  const rclcpp::Duration max_age = rclcpp::Duration::from_seconds(decay_time_sec_);
  std::vector<unsigned int> expired_indexes;

  for (const auto & item : active_cells_) {
    if (now - item.second.stamp > max_age) {
      expired_indexes.push_back(item.first);
      expired_cells_.push_back(item.second);
    }
  }

  for (const auto index : expired_indexes) {
    const auto it = active_cells_.find(index);
    if (it == active_cells_.end()) {
      continue;
    }
    setCost(it->second.mx, it->second.my, nav2_costmap_2d::NO_INFORMATION);
    touchCellBounds(it->second.mx, it->second.my);
    active_cells_.erase(it);
  }
}

void DecayObstacleLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  removeExpiredLocked(clock_->now());

  for (const auto & item : active_cells_) {
    const double wx = getOriginX() + (static_cast<double>(item.second.mx) + 0.5) * getResolution();
    const double wy = getOriginY() + (static_cast<double>(item.second.my) + 0.5) * getResolution();
    const double half = 0.5 * getResolution();
    touch(wx - half, wy - half, min_x, min_y, max_x, max_y);
    touch(wx + half, wy + half, min_x, min_y, max_x, max_y);
  }
  for (const auto & cell : expired_cells_) {
    const double wx = getOriginX() + (static_cast<double>(cell.mx) + 0.5) * getResolution();
    const double wy = getOriginY() + (static_cast<double>(cell.my) + 0.5) * getResolution();
    const double half = 0.5 * getResolution();
    touch(wx - half, wy - half, min_x, min_y, max_x, max_y);
    touch(wx + half, wy + half, min_x, min_y, max_x, max_y);
  }
  expired_cells_.clear();
  useExtraBounds(min_x, min_y, max_x, max_y);
}

void DecayObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace humanoid_decay_obstacle_layer

PLUGINLIB_EXPORT_CLASS(
  humanoid_decay_obstacle_layer::DecayObstacleLayer,
  nav2_costmap_2d::Layer)
