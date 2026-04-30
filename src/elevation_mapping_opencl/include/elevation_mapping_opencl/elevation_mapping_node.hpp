/**
 * @file elevation_mapping_node.hpp
 * @brief ROS2 高程图节点声明
 * 
 * 订阅点云，调用 OpenCL 高程图，发布 grid_map
 */

#ifndef ELEVATION_MAPPING_OPENCL__ELEVATION_MAPPING_NODE_HPP_
#define ELEVATION_MAPPING_OPENCL__ELEVATION_MAPPING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <memory>

#include "elevation_mapping_opencl/elevation_map_opencl.hpp"

namespace elevation_mapping_opencl
{

class ElevationMappingNode : public rclcpp::Node
{
public:
  explicit ElevationMappingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ElevationMappingNode() = default;

private:
  // ---- 回调函数 ----
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ---- 内部函数 ----
  void publishGridMap(const rclcpp::Time & stamp);
  bool getTransform(
    const std::string & target_frame,
    const std::string & source_frame,
    Eigen::Isometry3d & transform_out);
 
  // ===== 后处理开关 =====
  bool enable_post_processing_;
  bool enable_min_filter_;
  bool enable_smooth_;
  bool enable_inpaint_;
  bool enable_visibility_cleanup_;
  bool enable_erosion_;
  bool enable_drift_compensation_;
  bool drift_initialized_ = false;

  // ===== 漂移估计 =====
  float estimateGroundHeight();
  float drift_ema_ = 0.0f;

  // ---- ROS2 接口 ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- 高程图核心 ----
  std::unique_ptr<ElevationMapOpenCL> elevation_map_;

  // ---- 状态 ----
  Eigen::Isometry3d robot_pose_;
  bool has_pose_ = false;
  std::string map_frame_;
  std::string base_frame_;

  // ---- 参数 ----
  double publish_rate_;
  bool enable_traversability_;
};

}  // namespace elevation_mapping_opencl

#endif  // ELEVATION_MAPPING_OPENCL__ELEVATION_MAPPING_NODE_HPP_