/**
 * @file point_cloud_filter_node.cpp
 * @brief 高性能点云滤波节点 - ROS2 接口层
 * 
 * 功能：
 * 1. 订阅 FAST-LIO 输出的点云
 * 2. 进行坐标变换（通过 TF）
 * 3. 调用核心滤波器进行多级滤波
 * 4. 分离高程图点云和导航点云
 * 5. 发布两路点云
 * 6. 性能监控和日志
 */

#include "humanoid_point_cloud_filter/point_cloud_filter_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <rclcpp/executors.hpp>

namespace humanoid_point_cloud_filter
{

namespace
{

void pushTimingSample(std::deque<double> & samples, double value, size_t max_size = 100)
{
  samples.push_back(value);
  if (samples.size() > max_size) {
    samples.pop_front();
  }
}

double averageTiming(const std::deque<double> & samples)
{
  if (samples.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (double value : samples) {
    sum += value;
  }
  return sum / samples.size();
}

}  // namespace

/**
 * @brief 构造函数 - 初始化节点、参数、订阅和发布
 */
PointCloudFilterNode::PointCloudFilterNode(const rclcpp::NodeOptions & options)
: Node("high_performance_filter_node", options),
  frame_count_(0)
{
  // ===== 声明并获取参数 =====
  
  // 话题配置
  this->declare_parameter("input_topic", "/fast_lio/cloud_registered");
  this->declare_parameter("output_elevation_topic", "/airy_points_for_elevation");
  this->declare_parameter("output_nav_topic", "/airy_points_filtered");
  
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_elevation_topic_ = this->get_parameter("output_elevation_topic").as_string();
  output_nav_topic_ = this->get_parameter("output_nav_topic").as_string();
  this->declare_parameter("enable_elevation_output", true);
  enable_elevation_output_ = this->get_parameter("enable_elevation_output").as_bool();
  this->declare_parameter("scan_queue_size", 10);
  scan_queue_size_ = std::max(1, static_cast<int>(this->get_parameter("scan_queue_size").as_int()));
  
  // 距离过滤参数
  this->declare_parameter("min_range", 0.6);
  this->declare_parameter("max_range", 12.0);
  min_range_ = this->get_parameter("min_range").as_double();
  max_range_ = this->get_parameter("max_range").as_double();
  
  // 高度过滤参数
  this->declare_parameter("elev_min_z", -0.235);
  this->declare_parameter("elev_max_z", 1.7);
  this->declare_parameter("nav_min_z", 0.1);
  this->declare_parameter("nav_max_z", 1.4);
  
  elev_min_z_ = this->get_parameter("elev_min_z").as_double();
  elev_max_z_ = this->get_parameter("elev_max_z").as_double();
  nav_min_z_ = this->get_parameter("nav_min_z").as_double();
  nav_max_z_ = this->get_parameter("nav_max_z").as_double();
  
  // 性能监控参数
  this->declare_parameter("enable_performance_log", true);
  this->declare_parameter("performance_log_interval", 100);
  enable_performance_log_ = this->get_parameter("enable_performance_log").as_bool();
  performance_log_interval_ = this->get_parameter("performance_log_interval").as_int();
  
  // ===== 构建滤波器配置 =====
  FilterConfig config;
  
  // 体素下采样
  this->declare_parameter("voxel_leaf_size", 0.05);
  this->declare_parameter("enable_pre_voxel_for_filter", false);
  this->declare_parameter("pre_voxel_leaf_size", 0.05);
  this->declare_parameter("voxel_before_filters", false);
  this->declare_parameter("filter_mode", "sor");
  config.voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
  config.enable_pre_voxel_for_filter =
    this->get_parameter("enable_pre_voxel_for_filter").as_bool();
  config.pre_voxel_leaf_size = this->get_parameter("pre_voxel_leaf_size").as_double();
  config.voxel_before_filters = this->get_parameter("voxel_before_filters").as_bool();
  config.filter_mode = this->get_parameter("filter_mode").as_string();
  
  // SOR 参数
  this->declare_parameter("sor_k", 20);
  this->declare_parameter("sor_std_ratio", 1.0);
  this->declare_parameter("sor_std_ratio_moving", 0.8);
  config.sor_k = this->get_parameter("sor_k").as_int();
  config.sor_std_ratio = this->get_parameter("sor_std_ratio").as_double();
  config.sor_std_ratio_moving = this->get_parameter("sor_std_ratio_moving").as_double();
  
  // 高度连续性参数
  this->declare_parameter("height_diff_threshold", 0.05);
  this->declare_parameter("height_diff_threshold_moving", 0.03);
  config.height_diff_threshold = this->get_parameter("height_diff_threshold").as_double();
  config.height_diff_threshold_moving = this->get_parameter("height_diff_threshold_moving").as_double();
  
  // 密度检查参数
  this->declare_parameter("density_radius", 0.1);
  this->declare_parameter("min_density_points", 3);
  this->declare_parameter("min_density_points_moving", 4);
  config.density_radius = this->get_parameter("density_radius").as_double();
  config.min_density_points = this->get_parameter("min_density_points").as_int();
  config.min_density_points_moving = this->get_parameter("min_density_points_moving").as_int();
  
  // 运动检测参数
  this->declare_parameter("motion_threshold", 0.2);
  this->declare_parameter("motion_history_size", 5);
  config.motion_threshold = this->get_parameter("motion_threshold").as_double();
  config.motion_history_size = this->get_parameter("motion_history_size").as_int();

  // ===== 手臂包络盒过滤参数 =====
  this->declare_parameter("enable_body_box_filter", false);
  this->declare_parameter("arm_box_x_min", -0.10);
  this->declare_parameter("arm_box_x_max",  0.60);
  this->declare_parameter("arm_box_y_min", -0.45);
  this->declare_parameter("arm_box_y_max",  0.45);
  this->declare_parameter("arm_box_z_min",  0.70);
  this->declare_parameter("arm_box_z_max",  1.20);

  enable_body_box_filter_ = this->get_parameter("enable_body_box_filter").as_bool();
  arm_box_x_min_ = static_cast<float>(this->get_parameter("arm_box_x_min").as_double());
  arm_box_x_max_ = static_cast<float>(this->get_parameter("arm_box_x_max").as_double());
  arm_box_y_min_ = static_cast<float>(this->get_parameter("arm_box_y_min").as_double());
  arm_box_y_max_ = static_cast<float>(this->get_parameter("arm_box_y_max").as_double());
  arm_box_z_min_ = static_cast<float>(this->get_parameter("arm_box_z_min").as_double());
  arm_box_z_max_ = static_cast<float>(this->get_parameter("arm_box_z_max").as_double());

  // ===== 吊架包络盒过滤参数 =====
  this->declare_parameter("enable_mount_filter", true);
  this->declare_parameter("mount_box_x_min", -0.20);
  this->declare_parameter("mount_box_x_max",  0.20);
  this->declare_parameter("mount_box_y_min", -0.20);
  this->declare_parameter("mount_box_y_max",  0.20);
  this->declare_parameter("mount_box_z_min", -0.05);
  this->declare_parameter("mount_box_z_max",  0.25);

  enable_mount_filter_ = this->get_parameter("enable_mount_filter").as_bool();
  mount_box_x_min_ = static_cast<float>(this->get_parameter("mount_box_x_min").as_double());
  mount_box_x_max_ = static_cast<float>(this->get_parameter("mount_box_x_max").as_double());
  mount_box_y_min_ = static_cast<float>(this->get_parameter("mount_box_y_min").as_double());
  mount_box_y_max_ = static_cast<float>(this->get_parameter("mount_box_y_max").as_double());
  mount_box_z_min_ = static_cast<float>(this->get_parameter("mount_box_z_min").as_double());
  mount_box_z_max_ = static_cast<float>(this->get_parameter("mount_box_z_max").as_double());
  
  // 滤波开关
  this->declare_parameter("enable_sor", true);
  this->declare_parameter("enable_height_continuity", false);
  this->declare_parameter("enable_density", false);
  this->declare_parameter("enable_motion_detection", true);
  this->declare_parameter("combine_sor_height_kdtree", false);
  config.enable_sor = this->get_parameter("enable_sor").as_bool();
  config.enable_height_continuity = this->get_parameter("enable_height_continuity").as_bool();
  config.enable_density = this->get_parameter("enable_density").as_bool();
  config.enable_motion_detection = this->get_parameter("enable_motion_detection").as_bool();
  config.combine_sor_height_kdtree = this->get_parameter("combine_sor_height_kdtree").as_bool();

  // 多线程配置
  this->declare_parameter("num_threads", 4);  // 默认8线程，适合16核CPU
  config.num_threads = this->get_parameter("num_threads").as_int();

  RCLCPP_INFO(this->get_logger(), "点云滤波线程数: %d", config.num_threads);
  
  // ===== 创建核心滤波器 =====
  filter_core_ = std::make_unique<PointCloudFilterCore>(config);
  
  // ===== 初始化 TF =====
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // ===== 创建订阅和发布 =====
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_,
    rclcpp::SensorDataQoS().keep_last(scan_queue_size_),
    std::bind(&PointCloudFilterNode::cloudCallback, this, std::placeholders::_1)
  );
  
  if (enable_elevation_output_) {
    pub_elevation_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_elevation_topic_,
      10
    );
  }
  
  pub_nav_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_nav_topic_,
    10
  );
  
  // ===== 启动日志 =====
  RCLCPP_INFO(this->get_logger(),
    "高性能点云滤波节点已启动\n"
    "  输入话题: %s\n"
    "  高程图输出: %s\n"
    "  导航输出: %s\n"
    "  高程图输出启用: %s\n"
    "  滤波模式: %s\n"
    "  启用滤波: SOR=%s, 高度连续=%s, 密度=%s, 运动检测=%s, 合并KDTree=%s",
    input_topic_.c_str(),
    output_elevation_topic_.c_str(),
    output_nav_topic_.c_str(),
    enable_elevation_output_ ? "是" : "否",
    config.filter_mode.c_str(),
    config.enable_sor ? "是" : "否",
    config.enable_height_continuity ? "是" : "否",
    config.enable_density ? "是" : "否",
    config.enable_motion_detection ? "是" : "否",
    config.combine_sor_height_kdtree ? "是" : "否"
  );
  RCLCPP_INFO(
    this->get_logger(),
    "Pre-voxel滤波前降采样: %s, leaf=%.3fm, voxel提前: %s, 输出voxel leaf=%.3fm",
    config.enable_pre_voxel_for_filter ? "是" : "否",
    config.pre_voxel_leaf_size,
    config.voxel_before_filters ? "是" : "否",
    config.voxel_leaf_size);
}

/**
 * @brief 点云回调函数 - 主处理流程
 * 
 * 处理流程：
 * 1. 将 ROS PointCloud2 转换为 PCL 点云
 * 2. 坐标变换到 body 坐标系
 * 3. 距离过滤
 * 4. 调用核心滤波器（SOR、高度连续、密度检查、体素下采样）
 * 5. 坐标变换到 base_footprint
 * 6. 根据高度分离高程图点云和导航点云
 * 7. 发布两路点云
 * 8. 性能统计
 */
void PointCloudFilterNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto t_total_start = std::chrono::high_resolution_clock::now();
  
  // ===== 第 1 步：ROS 消息转 PCL 点云 =====
  auto t_ros_to_pcl_start = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_input);
  auto t_ros_to_pcl_end = std::chrono::high_resolution_clock::now();
  const double ros_to_pcl_ms =
    std::chrono::duration<double, std::milli>(t_ros_to_pcl_end - t_ros_to_pcl_start).count();
  
  if (cloud_input->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "收到空点云，跳过处理");
    return;
  }
  const rclcpp::Time cloud_stamp(msg->header.stamp, this->get_clock()->get_clock_type());
  
  // ===== 第 2 步：坐标变换到 body =====
  geometry_msgs::msg::TransformStamped transform_to_body;
  try {
    auto t_tf_lookup_body_start = std::chrono::high_resolution_clock::now();
    transform_to_body = tf_buffer_->lookupTransform(
      "body",
      msg->header.frame_id,
      cloud_stamp,
      rclcpp::Duration::from_seconds(0.1)
    );
    auto t_tf_lookup_body_end = std::chrono::high_resolution_clock::now();
    pushTimingSample(
      timing_tf_lookup_body_,
      std::chrono::duration<double, std::milli>(t_tf_lookup_body_end - t_tf_lookup_body_start).count());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "TF 查询失败 (%s -> body): %s", msg->header.frame_id.c_str(), ex.what());
    return;
  }
  
  // 转换为 Eigen 变换矩阵
  Eigen::Isometry3d T_to_body = tf2::transformToEigen(transform_to_body);
  
  // 应用变换
  auto t_transform_body_start = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_body(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud_input, *cloud_body, T_to_body.matrix().cast<float>());
  auto t_transform_body_end = std::chrono::high_resolution_clock::now();
  const double transform_body_ms =
    std::chrono::duration<double, std::milli>(t_transform_body_end - t_transform_body_start).count();
  
  // ===== 第 3 步：距离过滤 =====
  auto t_range_filter_start = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_range_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  cloud_range_filtered->reserve(cloud_body->size());
  
  for (const auto & point : cloud_body->points) {
    float dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (dist > min_range_ && dist < max_range_) {
      cloud_range_filtered->push_back(point);
    }
  }
  auto t_range_filter_end = std::chrono::high_resolution_clock::now();
  const double range_filter_ms =
    std::chrono::duration<double, std::milli>(t_range_filter_end - t_range_filter_start).count();
  
  if (cloud_range_filtered->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "距离过滤后点云为空，跳过处理");
    return;
  }

  // ===== 第 4 步：核心滤波 =====
  FilterTimings timings;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered = 
    filter_core_->filter(cloud_range_filtered, timings);
  
  if (cloud_filtered->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "滤波后点云为空，跳过处理");
    return;
  }
  
  // ===== 第 5 步：坐标变换到 base_footprint =====
  geometry_msgs::msg::TransformStamped transform_to_bf;
  try {
    auto t_tf_lookup_bf_start = std::chrono::high_resolution_clock::now();
    transform_to_bf = tf_buffer_->lookupTransform(
      "base_footprint",
      "body",
      cloud_stamp,
      rclcpp::Duration::from_seconds(0.1)
    );
    auto t_tf_lookup_bf_end = std::chrono::high_resolution_clock::now();
    pushTimingSample(
      timing_tf_lookup_bf_,
      std::chrono::duration<double, std::milli>(t_tf_lookup_bf_end - t_tf_lookup_bf_start).count());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "TF 查询失败 (body -> base_footprint): %s", ex.what());
    return;
  }
  
  Eigen::Isometry3d T_to_bf = tf2::transformToEigen(transform_to_bf);
  
  // 应用变换（用于高度判断）
  auto t_transform_bf_start = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bf(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud_filtered, *cloud_bf, T_to_bf.matrix().cast<float>());
  auto t_transform_bf_end = std::chrono::high_resolution_clock::now();
  const double transform_bf_ms =
    std::chrono::duration<double, std::milli>(t_transform_bf_end - t_transform_bf_start).count();
  
  // ===== 第 6 步：根据高度分离点云 =====
  auto t_split_clouds_start = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_elevation(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_nav(new pcl::PointCloud<pcl::PointXYZI>);
  
  if (enable_elevation_output_) {
    cloud_elevation->reserve(cloud_filtered->size());
  }
  cloud_nav->reserve(cloud_filtered->size());

  float bf_z_min = std::numeric_limits<float>::infinity();
  float bf_z_max = -std::numeric_limits<float>::infinity();
  size_t mount_filtered_count = 0;
  size_t body_filtered_count = 0;

  for (size_t i = 0; i < cloud_filtered->size(); ++i) {

    // 使用 base_footprint 坐标系（标准ROS，已经变换好了）
    float x = cloud_bf->points[i].x;
    float y = cloud_bf->points[i].y;
    float z = cloud_bf->points[i].z;

    bf_z_min = std::min(bf_z_min, z);
    bf_z_max = std::max(bf_z_max, z);

    // ★★★ 吊架过滤：过滤 LiDAR 正下方的支架点云
    bool in_mount_box = enable_mount_filter_ &&
                        (x > mount_box_x_min_ && x < mount_box_x_max_ &&
                         y > mount_box_y_min_ && y < mount_box_y_max_ &&
                         z > mount_box_z_min_ && z < mount_box_z_max_);
    if (in_mount_box) {
      mount_filtered_count++;
      continue;  // 丢弃吊架上的点
    }

    // 判断是否在手臂包络盒内（是则丢弃）
    bool in_arm_box = enable_body_box_filter_ &&
                  (x > arm_box_x_min_ && x < arm_box_x_max_ &&
                   y > arm_box_y_min_ && y < arm_box_y_max_ &&
                   z > arm_box_z_min_ && z < arm_box_z_max_);
    if (in_arm_box) {
      body_filtered_count++;
      continue;
    }
    
    // 高程图点云：用于建立高程地图
    if (enable_elevation_output_ && z > elev_min_z_ && z < elev_max_z_) {
        cloud_elevation->push_back(cloud_bf->points[i]);
    }
    
    // 导航点云：用于障碍物检测
    if (z > nav_min_z_ && z < nav_max_z_) {
        cloud_nav->push_back(cloud_bf->points[i]);
    }
  }
  auto t_split_clouds_end = std::chrono::high_resolution_clock::now();
  const double split_clouds_ms =
    std::chrono::duration<double, std::milli>(t_split_clouds_end - t_split_clouds_start).count();
  
  // ===== 第 7 步：发布点云 =====
  std_msgs::msg::Header header;
  // Keep the output stamp tied to the scan-time TF used above. Stamping this
  // with wall time can put the cloud in the future of Fast-LIO's dynamic TF.
  header.stamp = msg->header.stamp;
  header.frame_id = "base_footprint";
  
  if (enable_elevation_output_ && !cloud_elevation->empty()) {
    auto t_publish_elevation_start = std::chrono::high_resolution_clock::now();
    sensor_msgs::msg::PointCloud2 msg_elevation;
    pcl::toROSMsg(*cloud_elevation, msg_elevation);
    msg_elevation.header = header;
    pub_elevation_->publish(msg_elevation);
    auto t_publish_elevation_end = std::chrono::high_resolution_clock::now();
    pushTimingSample(
      timing_publish_elevation_,
      std::chrono::duration<double, std::milli>(
        t_publish_elevation_end - t_publish_elevation_start).count());
  } else {
    pushTimingSample(timing_publish_elevation_, 0.0);
  }
  
  if (!cloud_nav->empty()) {
    auto t_publish_nav_start = std::chrono::high_resolution_clock::now();
    sensor_msgs::msg::PointCloud2 msg_nav;
    pcl::toROSMsg(*cloud_nav, msg_nav);
    msg_nav.header = header;
    pub_nav_->publish(msg_nav);
    auto t_publish_nav_end = std::chrono::high_resolution_clock::now();
    pushTimingSample(
      timing_publish_nav_,
      std::chrono::duration<double, std::milli>(t_publish_nav_end - t_publish_nav_start).count());
  } else {
    pushTimingSample(timing_publish_nav_, 0.0);
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "导航点云为空，未发布 %s: input=%zu range=%zu filtered=%zu elevation=%zu "
      "mount_filtered=%zu body_filtered=%zu bf_z=[%.3f, %.3f] "
      "nav_z_filter=(%.3f, %.3f), body_box=%s mount_box=%s",
      output_nav_topic_.c_str(),
      cloud_input->size(),
      cloud_range_filtered->size(),
      cloud_filtered->size(),
      cloud_elevation->size(),
      mount_filtered_count,
      body_filtered_count,
      std::isfinite(bf_z_min) ? bf_z_min : 0.0f,
      std::isfinite(bf_z_max) ? bf_z_max : 0.0f,
      nav_min_z_,
      nav_max_z_,
      enable_body_box_filter_ ? "on" : "off",
      enable_mount_filter_ ? "on" : "off");
  }
  
  // ===== 第 8 步：性能统计 =====
  auto t_total_end = std::chrono::high_resolution_clock::now();
  double total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
  
  pushTimingSample(timing_total_, total_ms);
  pushTimingSample(timing_ros_to_pcl_, ros_to_pcl_ms);
  pushTimingSample(timing_transform_body_, transform_body_ms);
  pushTimingSample(timing_range_filter_, range_filter_ms);
  pushTimingSample(timing_transform_bf_, transform_bf_ms);
  pushTimingSample(timing_split_clouds_, split_clouds_ms);
  pushTimingSample(timing_motion_detect_, timings.motion_ms);
  pushTimingSample(timing_sor_, timings.sor_ms);
  pushTimingSample(timing_height_, timings.height_ms);
  pushTimingSample(timing_density_, timings.density_ms);
  pushTimingSample(timing_pre_voxel_, timings.pre_voxel_ms);
  pushTimingSample(timing_voxel_, timings.voxel_ms);
  
  frame_count_++;
  
  // 定期输出性能日志
  if (enable_performance_log_ && frame_count_ % performance_log_interval_ == 0) {
    logPerformanceStats();
  }
  
  // 每帧输出简要信息（调试级别）
  RCLCPP_DEBUG(this->get_logger(),
    "处理完成: 输入 %zu, 滤波后 %zu, 高程 %zu, 导航 %zu, 总耗时 %.2f ms",
    cloud_input->size(),
    cloud_filtered->size(),
    cloud_elevation->size(),
    cloud_nav->size(),
    total_ms
  );
}

/**
 * @brief 输出性能统计日志
 */
void PointCloudFilterNode::logPerformanceStats()
{
  if (timing_total_.empty()) {
    return;
  }
  
  const double avg_total = averageTiming(timing_total_);
  const double avg_ros_to_pcl = averageTiming(timing_ros_to_pcl_);
  const double avg_tf_lookup_body = averageTiming(timing_tf_lookup_body_);
  const double avg_transform_body = averageTiming(timing_transform_body_);
  const double avg_range_filter = averageTiming(timing_range_filter_);
  const double avg_tf_lookup_bf = averageTiming(timing_tf_lookup_bf_);
  const double avg_transform_bf = averageTiming(timing_transform_bf_);
  const double avg_split_clouds = averageTiming(timing_split_clouds_);
  const double avg_motion_detect = averageTiming(timing_motion_detect_);
  const double avg_sor = averageTiming(timing_sor_);
  const double avg_height = averageTiming(timing_height_);
  const double avg_density = averageTiming(timing_density_);
  const double avg_pre_voxel = averageTiming(timing_pre_voxel_);
  const double avg_voxel = averageTiming(timing_voxel_);
  const double avg_publish_elevation = averageTiming(timing_publish_elevation_);
  const double avg_publish_nav = averageTiming(timing_publish_nav_);
  
  RCLCPP_INFO(this->get_logger(),
    "\n========================================\n"
    "性能统计 (第 %d 帧)\n"
    "========================================\n"
    "平均总耗时: %.2f ms\n"
    "ROS->PCL: %.2f ms\n"
    "TF lookup to body: %.2f ms\n"
    "Transform to body: %.2f ms\n"
    "Range filter: %.2f ms\n"
    "Motion detect: %.2f ms\n"
    "SOR: %.2f ms\n"
    "Height continuity: %.2f ms\n"
    "Density: %.2f ms\n"
    "Pre-voxel downsample: %.2f ms\n"
    "Voxel downsample: %.2f ms\n"
    "TF lookup to base_footprint: %.2f ms\n"
    "Transform to base_footprint: %.2f ms\n"
    "Split elevation/nav: %.2f ms\n"
    "Publish elevation: %.2f ms\n"
    "Publish nav: %.2f ms\n"
    "运动状态: %s\n"
    "========================================",
    frame_count_,
    avg_total,
    avg_ros_to_pcl,
    avg_tf_lookup_body,
    avg_transform_body,
    avg_range_filter,
    avg_motion_detect,
    avg_sor,
    avg_height,
    avg_density,
    avg_pre_voxel,
    avg_voxel,
    avg_tf_lookup_bf,
    avg_transform_bf,
    avg_split_clouds,
    avg_publish_elevation,
    avg_publish_nav,
    filter_core_->isMoving() ? "运动中" : "静止"
  );
}

}  // namespace humanoid_point_cloud_filter

/**
 * @brief 主函数入口 - 使用多线程executor提高性能
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 使用多线程executor spin节点
  rclcpp::executors::MultiThreadedExecutor executor;
  
  // 创建节点（节点构造函数中会声明所有参数）
  auto node = std::make_shared<humanoid_point_cloud_filter::PointCloudFilterNode>();
  
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
