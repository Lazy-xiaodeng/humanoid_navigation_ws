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
#include <chrono>
#include <rclcpp/executors.hpp>

namespace humanoid_point_cloud_filter
{

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
  config.voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
  
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
  
  // 滤波开关
  this->declare_parameter("enable_sor", true);
  this->declare_parameter("enable_height_continuity", false);
  this->declare_parameter("enable_density", false);
  this->declare_parameter("enable_motion_detection", true);
  config.enable_sor = this->get_parameter("enable_sor").as_bool();
  config.enable_height_continuity = this->get_parameter("enable_height_continuity").as_bool();
  config.enable_density = this->get_parameter("enable_density").as_bool();
  config.enable_motion_detection = this->get_parameter("enable_motion_detection").as_bool();

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
    rclcpp::SensorDataQoS(),
    std::bind(&PointCloudFilterNode::cloudCallback, this, std::placeholders::_1)
  );
  
  pub_elevation_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_elevation_topic_,
    10
  );
  
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
    "  启用滤波: SOR=%s, 高度连续=%s, 密度=%s, 运动检测=%s",
    input_topic_.c_str(),
    output_elevation_topic_.c_str(),
    output_nav_topic_.c_str(),
    config.enable_sor ? "是" : "否",
    config.enable_height_continuity ? "是" : "否",
    config.enable_density ? "是" : "否",
    config.enable_motion_detection ? "是" : "否"
  );
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_input);
  
  if (cloud_input->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "收到空点云，跳过处理");
    return;
  }
  
  // ===== 第 2 步：坐标变换到 body =====
  geometry_msgs::msg::TransformStamped transform_to_body;
  try {
    transform_to_body = tf_buffer_->lookupTransform(
      "body",
      msg->header.frame_id,
      tf2::TimePointZero
    );
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "TF 查询失败 (%s -> body): %s", msg->header.frame_id.c_str(), ex.what());
    return;
  }
  
  // 转换为 Eigen 变换矩阵
  Eigen::Isometry3d T_to_body = tf2::transformToEigen(transform_to_body);
  
  // 应用变换
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_body(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud_input, *cloud_body, T_to_body.matrix().cast<float>());
  
  // ===== 第 3 步：距离过滤 =====
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_range_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  cloud_range_filtered->reserve(cloud_body->size());
  
  for (const auto & point : cloud_body->points) {
    float dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (dist > min_range_ && dist < max_range_) {
      cloud_range_filtered->push_back(point);
    }
  }
  
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
    transform_to_bf = tf_buffer_->lookupTransform(
      "base_footprint",
      "body",
      tf2::TimePointZero
    );
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "TF 查询失败 (body -> base_footprint): %s", ex.what());
    return;
  }
  
  Eigen::Isometry3d T_to_bf = tf2::transformToEigen(transform_to_bf);
  
  // 应用变换（用于高度判断）
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bf(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud_filtered, *cloud_bf, T_to_bf.matrix().cast<float>());
  
  // ===== 第 6 步：根据高度分离点云 =====
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_elevation(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_nav(new pcl::PointCloud<pcl::PointXYZI>);
  
  cloud_elevation->reserve(cloud_filtered->size());
  cloud_nav->reserve(cloud_filtered->size());

  // 手臂包络盒参数（base_footprint 坐标系，标准ROS，原点在脚底）
  // X前、Y左、Z上，坐标含义直觉化，实测后调整
  const float arm_x_min = -0.1f;   // 身体后方10cm（手臂后摆）
  const float arm_x_max =  0.6f;   // 前方60cm
  const float arm_y_min = -0.45f;  // 右侧45cm
  const float arm_y_max =  0.45f;  // 左侧45cm
  const float arm_z_min =  0.7f;   // 离地70cm（腰部以上）
  const float arm_z_max =  1.2f;   // 离地120cm（胸部以下）

  // ★★★ 吊架包络盒参数（base_footprint 坐标系）
  // 注意：吊架在 LiDAR 下方约1.2m处（因为 body 在 base_footprint 上方 1.215m）
  // 所以吊架在 base_footprint 坐标系中大约在 Z=0.0~0.2m 范围
  const float mount_x_min = -0.2f;   // 吊架后方20cm
  const float mount_x_max =  0.2f;   // 吊架前方20cm
  const float mount_y_min = -0.2f;   // 吊架右侧20cm
  const float mount_y_max =  0.2f;   // 吊架左侧20cm
  const float mount_z_min = -0.05f;  // 吊架下方（地面附近）
  const float mount_z_max =  0.25f;  // 吊架上方25cm（LiDAR 正下方区域）

  for (size_t i = 0; i < cloud_filtered->size(); ++i) {

    // 使用 base_footprint 坐标系（标准ROS，已经变换好了）
    float x = cloud_bf->points[i].x;
    float y = cloud_bf->points[i].y;
    float z = cloud_bf->points[i].z;

    // ★★★ 吊架过滤：过滤 LiDAR 正下方的支架点云
    bool in_mount_box = (x > mount_x_min && x < mount_x_max &&
                         y > mount_y_min && y < mount_y_max &&
                         z > mount_z_min && z < mount_z_max);
    if (in_mount_box) continue;  // 丢弃吊架上的点

    // 判断是否在手臂包络盒内（是则丢弃）
    bool in_arm_box = enable_body_box_filter_ &&
                  (x > arm_box_x_min_ && x < arm_box_x_max_ &&
                   y > arm_box_y_min_ && y < arm_box_y_max_ &&
                   z > arm_box_z_min_ && z < arm_box_z_max_);
    if (in_arm_box) continue;
    
    // 高程图点云：用于建立高程地图
    if (z > elev_min_z_ && z < elev_max_z_) {
        cloud_elevation->push_back(cloud_filtered->points[i]);
    }
    
    // 导航点云：用于障碍物检测
    if (z > nav_min_z_ && z < nav_max_z_) {
        cloud_nav->push_back(cloud_filtered->points[i]);
    }
  }
  
  // ===== 第 7 步：发布点云 =====
  std_msgs::msg::Header header;
  header.stamp = msg->header.stamp;
  header.frame_id = "body";
  
  if (!cloud_elevation->empty()) {
    sensor_msgs::msg::PointCloud2 msg_elevation;
    pcl::toROSMsg(*cloud_elevation, msg_elevation);
    msg_elevation.header = header;
    pub_elevation_->publish(msg_elevation);
  }
  
  if (!cloud_nav->empty()) {
    sensor_msgs::msg::PointCloud2 msg_nav;
    pcl::toROSMsg(*cloud_nav, msg_nav);
    msg_nav.header = header;
    pub_nav_->publish(msg_nav);
  }
  
  // ===== 第 8 步：性能统计 =====
  auto t_total_end = std::chrono::high_resolution_clock::now();
  double total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();
  
  timing_total_.push_back(total_ms);
  if (timing_total_.size() > 100) {
    timing_total_.pop_front();
  }
  
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
  
  // 计算平均耗时
  double avg_total = 0.0;
  for (double t : timing_total_) {
    avg_total += t;
  }
  avg_total /= timing_total_.size();
  
  RCLCPP_INFO(this->get_logger(),
    "\n========================================\n"
    "性能统计 (第 %d 帧)\n"
    "========================================\n"
    "平均总耗时: %.2f ms\n"
    "运动状态: %s\n"
    "========================================",
    frame_count_,
    avg_total,
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