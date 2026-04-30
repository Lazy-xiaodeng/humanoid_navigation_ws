/**
 * @file elevation_mapping_node.cpp
 * @brief ROS2 高程图节点实现
 */

#include "elevation_mapping_opencl/elevation_mapping_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <chrono>
#include <algorithm>
#include <limits>
#include <rclcpp/logging.hpp> 

namespace elevation_mapping_opencl
{

ElevationMappingNode::ElevationMappingNode(const rclcpp::NodeOptions & options)
: Node("elevation_mapping_opencl_node", options)
{
  // ===== 声明参数 =====
  // 话题
  this->declare_parameter("input_cloud_topic", "/airy_points_for_elevation");
  this->declare_parameter("odom_topic", "/Odometry");
  this->declare_parameter("output_map_topic", "/elevation_mapping/output");
  this->declare_parameter("map_frame", "odom");
  this->declare_parameter("base_frame", "base_footprint");

  // 地图参数
  this->declare_parameter("resolution", 0.05);
  this->declare_parameter("map_length", 10.0);
  this->declare_parameter("publish_rate", 10.0);
  this->declare_parameter("enable_traversability", true);

  // 传感器参数
  this->declare_parameter("sensor_noise_factor", 0.0025);
  this->declare_parameter("mahalanobis_thresh", 3.0);
  this->declare_parameter("outlier_variance", 0.01);
  this->declare_parameter("time_variance", 0.0001);
  this->declare_parameter("max_variance", 1.0);
  this->declare_parameter("min_valid_distance", 0.3);
  this->declare_parameter("max_height_range", 5.0);

  // 可通行性参数
  this->declare_parameter("max_slope", 0.5);
  this->declare_parameter("max_step_height", 0.15);
  this->declare_parameter("slope_weight", 0.5);
  this->declare_parameter("step_weight", 0.5);

  // ===== 后处理参数 =====
  this->declare_parameter("enable_post_processing", true);
  this->declare_parameter("enable_min_filter", true);
  this->declare_parameter("enable_smooth", true);
  this->declare_parameter("enable_inpaint", true);
  this->declare_parameter("enable_visibility_cleanup", false);  // 默认关闭，性能开销较大
  this->declare_parameter("enable_erosion", true);
  this->declare_parameter("enable_drift_compensation", false);

  // ===== 侵蚀参数 =====
  this->declare_parameter("erosion_radius", 3);
  this->declare_parameter("erosion_safety_threshold", 0.3);
  this->declare_parameter("erosion_strength", 0.7);

  // ===== 空洞参数 =====
  this->declare_parameter("inpaint_max_iters", 4);
  this->declare_parameter("inpaint_max_distance", 1.0);
  this->declare_parameter("inpaint_kernel_radius", 2);


  // ===== 构建高程图配置 =====
  ElevationMapConfig config;
  config.resolution          = this->get_parameter("resolution").as_double();
  config.map_length          = this->get_parameter("map_length").as_double();
  config.sensor_noise_factor = this->get_parameter("sensor_noise_factor").as_double();
  config.mahalanobis_thresh  = this->get_parameter("mahalanobis_thresh").as_double();
  config.outlier_variance    = this->get_parameter("outlier_variance").as_double();
  config.time_variance       = this->get_parameter("time_variance").as_double();
  config.max_variance        = this->get_parameter("max_variance").as_double();
  config.min_valid_distance  = this->get_parameter("min_valid_distance").as_double();
  config.max_height_range    = this->get_parameter("max_height_range").as_double();
  config.max_slope           = this->get_parameter("max_slope").as_double();
  config.max_step_height     = this->get_parameter("max_step_height").as_double();
  config.slope_weight        = this->get_parameter("slope_weight").as_double();
  config.step_weight         = this->get_parameter("step_weight").as_double();

  // ===== 创建高程图核心 =====
  elevation_map_ = std::make_unique<ElevationMapOpenCL>(config);

  // ===== 读取参数 =====
  std::string input_topic  = this->get_parameter("input_cloud_topic").as_string();
  std::string odom_topic   = this->get_parameter("odom_topic").as_string();
  std::string output_topic = this->get_parameter("output_map_topic").as_string();
  map_frame_               = this->get_parameter("map_frame").as_string();
  base_frame_              = this->get_parameter("base_frame").as_string();
  publish_rate_            = this->get_parameter("publish_rate").as_double();
  enable_traversability_   = this->get_parameter("enable_traversability").as_bool();

  enable_post_processing_     = this->get_parameter("enable_post_processing").as_bool();
  enable_min_filter_          = this->get_parameter("enable_min_filter").as_bool();
  enable_smooth_              = this->get_parameter("enable_smooth").as_bool();
  enable_inpaint_             = this->get_parameter("enable_inpaint").as_bool();
  enable_visibility_cleanup_  = this->get_parameter("enable_visibility_cleanup").as_bool();
  enable_erosion_             = this->get_parameter("enable_erosion").as_bool();
  enable_drift_compensation_ = this->get_parameter("enable_drift_compensation").as_bool();
  
  elevation_map_->erosion_config_.erosion_radius = this->get_parameter("erosion_radius").as_int();
  elevation_map_->erosion_config_.safety_threshold = this->get_parameter("erosion_safety_threshold").as_double();
  elevation_map_->erosion_config_.erosion_strength = this->get_parameter("erosion_strength").as_double();
  elevation_map_->inpaint_config_.max_iters = this->get_parameter("inpaint_max_iters").as_int();
  elevation_map_->inpaint_config_.max_inpaint_distance = this->get_parameter("inpaint_max_distance").as_double();
  elevation_map_->inpaint_config_.kernel_radius = this->get_parameter("inpaint_kernel_radius").as_int();
  

 

  // ===== 初始化 TF =====
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ===== 初始化位姿 =====
  robot_pose_ = Eigen::Isometry3d::Identity();

  // ===== 创建订阅和发布 =====
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&ElevationMappingNode::pointCloudCallback, this, std::placeholders::_1));

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&ElevationMappingNode::odomCallback, this, std::placeholders::_1));

  pub_grid_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    output_topic, 1);

  // ===== 定时发布 =====
  auto period_ms = std::chrono::milliseconds(
    static_cast<int>(1000.0 / publish_rate_));
  publish_timer_ = this->create_wall_timer(
    period_ms,
    [this]() {
      publishGridMap(this->now());
    });

  RCLCPP_INFO(this->get_logger(),
    "✅ ElevationMappingOpenCL 节点已启动\n"
    "  输入点云: %s\n"
    "  里程计: %s\n"
    "  输出: %s\n"
    "  分辨率: %.3f m, 地图边长: %.1f m\n"
    "  发布频率: %.1f Hz",
    input_topic.c_str(), odom_topic.c_str(), output_topic.c_str(),
    config.resolution, config.map_length, publish_rate_);
}

// =====================================================================
// 点云回调
// =====================================================================
void ElevationMappingNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  elevation_map_->resetPostProcessState();
  if (!has_pose_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "⚠️  尚未收到位姿，跳过点云处理");
    return;
  }

  // ===== 步骤 1：转换 ROS 消息 → PCL 点云 =====
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty()) return;

  // ===== 步骤 2：获取传感器到地图坐标系的变换 =====
  Eigen::Isometry3d T_map_sensor;
  if (!getTransform(map_frame_, msg->header.frame_id, T_map_sensor)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "⚠️  TF 查询失败: %s → %s",
      msg->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }

  // 旋转矩阵和平移向量
  Eigen::Matrix3f R = T_map_sensor.rotation().cast<float>();
  Eigen::Vector3f t = T_map_sensor.translation().cast<float>();

  // ===== 步骤 3：拍平点云数据 =====
  std::vector<Eigen::Vector3f> points;
  points.reserve(cloud->size());
  for (const auto & p : cloud->points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      points.emplace_back(p.x, p.y, p.z);
    }
  }

  if (points.empty()) return;

  // ===== 步骤 4：更新地图中心（跟随机器人）=====
  Eigen::Vector3f robot_pos = robot_pose_.translation().cast<float>();
  Eigen::Matrix3f robot_rot = robot_pose_.rotation().cast<float>();
  elevation_map_->moveTo(robot_pos, robot_rot);

  // ===== 步骤 5：融合点云 =====
  auto t_start = std::chrono::high_resolution_clock::now();
  elevation_map_->inputPointCloud(points, R, t);
  auto t_end = std::chrono::high_resolution_clock::now();

  double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  RCLCPP_DEBUG(this->get_logger(),
    "点云融合完成: %zu 点, 耗时 %.2f ms", points.size(), elapsed_ms);

  // ===== 步骤 6：更新方差 =====
  elevation_map_->updateVariance();

  // ===== 步骤 6.5：漂移补偿（可选）=====
  if (enable_drift_compensation_) {
    float measured = estimateGroundHeight();
    float drift_raw = measured - 0.0f;   // 期望地面高度 = 0

    // 指数移动平均，平滑漂移估计
    float alpha = 0.1f;   // 平滑系数，越小越稳
    if (!drift_initialized_) {
        drift_ema_ = drift_raw;
        drift_initialized_ = true;
    } else {
        drift_ema_ = alpha * drift_raw + (1.0f - alpha) * drift_ema_;
    }

    // 死区：漂移绝对值超过 0.05m 才修正
    // 修正量：只修正超出死区的部分，避免过修正
    float dead_band = 0.05f;
    if (std::abs(drift_ema_) > dead_band) {
      // 每帧只修正一小步，而不是全量修正
      float correction_step = 0.005f;   // 每帧最多修正 5mm
      float correction = std::copysign(
        std::min(std::abs(drift_ema_), correction_step),
        drift_ema_);

      elevation_map_->compensateDrift(correction);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Z 轴漂移估计 %.3f m，本帧修正 %.3f m",
        drift_ema_, correction);
     }
  }

  

  // ===== 步骤 7：后处理流程（新增）=====
  if (enable_post_processing_) {
    // 7.1 最小值滤波（抑制孤立高点）
    if (enable_min_filter_) {
      elevation_map_->applyMinFilter();
    }

    // 7.2 高斯平滑（降噪）
    if (enable_smooth_) {
      elevation_map_->smooth();
    }

    // 7.3 空洞填补（填补盲区）
    if (enable_inpaint_) {
      elevation_map_->inpaint();
    }

    // 7.4 视野清理（清除动态障碍残影）
    if (enable_visibility_cleanup_) {
       Eigen::Vector3f sensor_pos_world = T_map_sensor.translation().cast<float>();
       // 转换为相对于地图中心的坐标
       Eigen::Vector3f map_center = elevation_map_->getCenter();
       Eigen::Vector3f sensor_pos_relative = sensor_pos_world - map_center;
       elevation_map_->cleanupVisibility(sensor_pos_relative);
    }
  }

  // ===== 步骤 8：计算法线和可通行性 =====
  if (enable_traversability_) {
    elevation_map_->computeNormals();
    elevation_map_->computeTraversability();
    
    // 8.1 形态学侵蚀（生成真正的 erosion 层）
    if (enable_erosion_) {
      elevation_map_->computeErosion();
    }
  }
}

// =====================================================================
// 漂移补偿 “平面拟合后再修正”
// 在中心 ROI 内收集有效高程点
// 拟合一个平面 z = ax + by + c
// 计算平面在中心的偏移
// 只对整体偏移做微小修正，或者只修正高置信区域
// =====================================================================
float ElevationMappingNode::estimateGroundHeight()
{
  std::vector<float> elevation_data;
  std::vector<float> valid_data;      // ← 新增：同时读取有效性层
 
  elevation_map_->getLayer("elevation", elevation_data);
  elevation_map_->getLayer("is_valid", valid_data);   // ← 新增
 
  int width  = elevation_map_->getWidth();
  int height = elevation_map_->getHeight();
 
  int center_row = height / 2;
  int center_col = width  / 2;
  int roi_radius = 20;
 
  std::vector<float> valid_heights;
  valid_heights.reserve((2 * roi_radius + 1) * (2 * roi_radius + 1));
 
  for (int dr = -roi_radius; dr <= roi_radius; ++dr) {
    for (int dc = -roi_radius; dc <= roi_radius; ++dc) {
      int r = center_row + dr;
      int c = center_col + dc;
      if (r < 0 || r >= height || c < 0 || c >= width) continue;
 
      int idx = r * width + c;
 
      // ← 必须同时检查 is_valid，避免把初始化的 0.0f 当地面
      if (valid_data[idx] < 0.5f) continue;
 
      float h = elevation_data[idx];
      if (std::isfinite(h)) {
        valid_heights.push_back(h);
      }
    }
  }
 
  if (valid_heights.empty()) return 0.0f;
 
  // 取中位数
  std::nth_element(
    valid_heights.begin(),
    valid_heights.begin() + valid_heights.size() / 2,
    valid_heights.end());
 
  return valid_heights[valid_heights.size() / 2];
}

// =====================================================================
// 里程计回调（更新机器人位姿）
// =====================================================================
void ElevationMappingNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  Eigen::Quaterniond q(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);

  // 归一化四元数，防止里程计消息中有轻微数值误差
  q.normalize();   

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear()      = q.toRotationMatrix();   // ← 直接设置旋转矩阵
  pose.translation() = Eigen::Vector3d(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z);

  robot_pose_ = pose;
  has_pose_   = true;
}

// =====================================================================
// 发布 grid_map
// =====================================================================
void ElevationMappingNode::publishGridMap(const rclcpp::Time & stamp)
{
  if (!has_pose_) return;
  if (pub_grid_map_->get_subscription_count() == 0) return;

  // ===== 创建 grid_map =====
  grid_map::GridMap grid_map;
  grid_map.setFrameId(map_frame_);

  double resolution = elevation_map_->getResolution();
  int width  = elevation_map_->getWidth();
  int height = elevation_map_->getHeight();
  Eigen::Vector3f center = elevation_map_->getCenter();

  grid_map.setGeometry(
    grid_map::Length(width * resolution, height * resolution),
    resolution,
    grid_map::Position(center.x(), center.y()));

  // ===== 读取高程图各图层 =====
  auto fillLayer = [&](const std::string & layer_name) {
    std::vector<float> data;
    elevation_map_->getLayer(layer_name, data);

    grid_map.add(layer_name);
    auto & mat = grid_map[layer_name];

    for (int r = 0; r < height; ++r) {
      for (int c = 0; c < width; ++c) {
        float val = data[r * width + c];
        mat(r, c) = std::isfinite(val) ? val : std::numeric_limits<float>::quiet_NaN();
      }
    }
  };
   
  // ===== 发布基础层 =====
  fillLayer("elevation");
  fillLayer("variance");
  fillLayer("is_valid");
 
  // ===== 发布可通行性层和 erosion 层 =====
  if (enable_traversability_) {
    std::vector<float> trav_data;
    elevation_map_->getLayer("traversability", trav_data);

    grid_map.add("traversability");
    auto & mat = grid_map["traversability"];
    for (int r = 0; r < height; ++r) {
      for (int c = 0; c < width; ++c) {
        mat(r, c) = trav_data[r * width + c];
      }
    }

    // ===== 生成 erosion 层（真正的形态学侵蚀）=====
    if (enable_erosion_) {
      std::vector<float> erosion_data;
      elevation_map_->getLayer("erosion", erosion_data);
      grid_map.add("erosion");
      auto & erosion_mat = grid_map["erosion"];
      for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
          erosion_mat(r, c) = erosion_data[r * width + c];
        }
      }
    } 
    else {
     // 复制 traversability 数据，不用 mat 引用
      std::vector<float> trav_copy = trav_data;  // trav_data 已读取
      grid_map.add("erosion");
      auto & erosion_mat = grid_map["erosion"];
      for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
           erosion_mat(r, c) = trav_copy[r * width + c];
        }
      }
    }
  }
  // ===== 发布 =====
  auto msg = grid_map::GridMapRosConverter::toMessage(grid_map); // 返回 unique_ptr
  msg->header.stamp = stamp;
  pub_grid_map_->publish(std::move(msg));

  RCLCPP_DEBUG(this->get_logger(), "已发布 GridMap: %d×%d @ %.3f m/px",
    width, height, resolution);
}

// =====================================================================
// TF 查询辅助函数
// =====================================================================
bool ElevationMappingNode::getTransform(
  const std::string & target_frame,
  const std::string & source_frame,
  Eigen::Isometry3d & transform_out)
{
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, tf2::TimePointZero);
    transform_out = tf2::transformToEigen(tf_stamped);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF 错误: %s", ex.what());
    return false;
  }
}

}  // namespace elevation_mapping_opencl

// =====================================================================
// main 函数入口
// =====================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<elevation_mapping_opencl::ElevationMappingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}