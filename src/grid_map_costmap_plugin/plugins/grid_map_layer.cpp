#include "grid_map_costmap_plugin/grid_map_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include <cmath>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(grid_map_costmap_plugin::GridMapLayer, nav2_costmap_2d::Layer)

namespace grid_map_costmap_plugin
{

GridMapLayer::GridMapLayer()
: has_data_(false)
{
}

void GridMapLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in GridMapLayer");
  }

  // 保存 logger 和 clock 供后续使用
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 声明并获取参数
  declareParameter("grid_map_topic", rclcpp::ParameterValue(std::string("/elevation_mapping/output")));
  declareParameter("layer_name", rclcpp::ParameterValue(std::string("traversability")));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(0.3));
  declareParameter("inscribed_threshold", rclcpp::ParameterValue(0.5));
  declareParameter("enabled", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".grid_map_topic", grid_map_topic_);
  node->get_parameter(name_ + ".layer_name", layer_name_);
  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold_);
  node->get_parameter(name_ + ".inscribed_threshold", inscribed_threshold_);
  node->get_parameter(name_ + ".enabled", enabled_);

  RCLCPP_INFO(
    logger_,
    "GridMapLayer: topic='%s', layer='%s', lethal_thresh=%.2f, inscribed_thresh=%.2f",
    grid_map_topic_.c_str(), layer_name_.c_str(), lethal_threshold_, inscribed_threshold_);

  // 订阅 GridMap
  grid_map_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
    grid_map_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&GridMapLayer::gridMapCallback, this, std::placeholders::_1));

  current_ = true;
}

void GridMapLayer::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_grid_map_ = msg;
  has_data_ = true;
}

void GridMapLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_ || !has_data_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!latest_grid_map_) {
    return;
  }

  auto & info = latest_grid_map_->info;

  // GridMap 的中心位置
  double cx = info.pose.position.x;
  double cy = info.pose.position.y;
  double half_x = info.length_x / 2.0;
  double half_y = info.length_y / 2.0;

  // 扩展更新边界
  *min_x = std::min(*min_x, cx - half_x);
  *min_y = std::min(*min_y, cy - half_y);
  *max_x = std::max(*max_x, cx + half_x);
  *max_y = std::max(*max_y, cy + half_y);
}

void GridMapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  if (!enabled_ || !has_data_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!latest_grid_map_) {
    return;
  }

  auto & msg = *latest_grid_map_;
  auto & info = msg.info;

  // 查找目标图层索引
  int layer_index = -1;
  for (size_t i = 0; i < msg.layers.size(); ++i) {
    if (msg.layers[i] == layer_name_) {
      layer_index = static_cast<int>(i);
      break;
    }
  }

  if (layer_index < 0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "GridMapLayer: layer '%s' not found in GridMap message. Available layers:",
      layer_name_.c_str());
    for (auto & l : msg.layers) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "  - %s", l.c_str());
    }
    return;
  }

  // 获取层数据
  auto & array_msg = msg.data[layer_index];
  int gm_cols, gm_rows;

  if (array_msg.layout.dim.size() >= 2) {
    gm_cols = array_msg.layout.dim[0].size;
    gm_rows = array_msg.layout.dim[1].size;
  } else {
    gm_cols = static_cast<int>(info.length_x / info.resolution);
    gm_rows = static_cast<int>(info.length_y / info.resolution);
  }

  if (static_cast<int>(array_msg.data.size()) < gm_rows * gm_cols) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "GridMapLayer: data size mismatch. Expected %d, got %zu",
      gm_rows * gm_cols, array_msg.data.size());
    return;
  }

  // GridMap 中心 (世界坐标)
  double cx = info.pose.position.x;
  double cy = info.pose.position.y;
  double gm_res = info.resolution;

  // 遍历 costmap 的像元
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      // costmap 像元 → 世界坐标
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      // 世界坐标 → GridMap 像元索引
      double dx = cx + info.length_x / 2.0 - wx;
      double dy = cy + info.length_y / 2.0 - wy;

      int gm_row = static_cast<int>(dx / gm_res);
      int gm_col = static_cast<int>(dy / gm_res);

      // 边界检查
      if (gm_row < 0 || gm_row >= gm_rows || gm_col < 0 || gm_col >= gm_cols) {
        continue;
      }

      // 获取 traversability 值
      int data_index = gm_row * gm_cols + gm_col;
      float trav = array_msg.data[data_index];

      // 跳过 NaN
      if (std::isnan(trav)) {
        continue;
      }

      // 转换为代价值
      unsigned char cost = traversabilityToCost(trav);

      // 只在代价更高时覆盖（取最大值策略）
      unsigned char old_cost = master_grid.getCost(i, j);
      if (cost > old_cost) {
        master_grid.setCost(i, j, cost);
      }
    }
  }
}

unsigned char GridMapLayer::traversabilityToCost(float traversability) const
{
  if (traversability < lethal_threshold_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;  // 254
  }

  if (traversability < inscribed_threshold_) {
    float ratio = (traversability - lethal_threshold_) / (inscribed_threshold_ - lethal_threshold_);
    return static_cast<unsigned char>(253 - ratio * 125);
  }

  if (traversability < 0.8) {
    float ratio = (traversability - inscribed_threshold_) / (0.8 - inscribed_threshold_);
    return static_cast<unsigned char>(128 - ratio * 127);
  }

  return nav2_costmap_2d::FREE_SPACE;  // 0
}

void GridMapLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  has_data_ = false;
  latest_grid_map_.reset();
  current_ = false;
}

}  // namespace grid_map_costmap_plugin
