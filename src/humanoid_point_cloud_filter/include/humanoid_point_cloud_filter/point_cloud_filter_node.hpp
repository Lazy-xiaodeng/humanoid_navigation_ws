#ifndef HUMANOID_POINT_CLOUD_FILTER__POINT_CLOUD_FILTER_NODE_HPP_
#define HUMANOID_POINT_CLOUD_FILTER__POINT_CLOUD_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <deque>
#include <memory>

#include "humanoid_point_cloud_filter/point_cloud_filter_core.hpp"  

namespace humanoid_point_cloud_filter  
{

class PointCloudFilterNode : public rclcpp::Node
{
public:
  explicit PointCloudFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void logPerformanceStats();

  // 订阅和发布
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_elevation_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_nav_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 核心滤波器
  std::unique_ptr<PointCloudFilterCore> filter_core_;

  // 参数
  std::string input_topic_;
  std::string output_elevation_topic_;
  std::string output_nav_topic_;

  double min_range_;
  double max_range_;
  double elev_min_z_;
  double elev_max_z_;
  double nav_min_z_;
  double nav_max_z_;

  //===== 手臂包络盒过滤参数 =====
  bool enable_body_box_filter_;
  float arm_box_x_min_;
  float arm_box_x_max_;
  float arm_box_y_min_;
  float arm_box_y_max_;
  float arm_box_z_min_;
  float arm_box_z_max_;

  // 性能监控
  bool enable_performance_log_;
  int performance_log_interval_;
  int frame_count_;
  std::deque<double> timing_total_;
};

}  // namespace humanoid_point_cloud_filter  

#endif  // HUMANOID_POINT_CLOUD_FILTER__POINT_CLOUD_FILTER_NODE_HPP_