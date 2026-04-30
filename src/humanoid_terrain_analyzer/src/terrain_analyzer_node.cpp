/**
 * @file terrain_analyzer_node.cpp
 * @brief 地形分析 ROS2 节点实现
 *
 * 这个文件负责：
 *   - ROS2 参数读取
 *   - GridMap 订阅与解析
 *   - ROI 提取
 *   - 调用 TerrainAnalyzerCore 分析
 *   - 发布 TerrainInfo 和虚拟墙点云
 */

#include "humanoid_terrain_analyzer/terrain_analyzer_node.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>
#include <array>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace humanoid_terrain_analyzer
{

// ===========================================================================
// 构造函数：读参数 → 创建核心 → 创建 ROS 接口
// ===========================================================================

TerrainAnalyzerNode::TerrainAnalyzerNode(const rclcpp::NodeOptions & options)
: Node("terrain_analyzer", options)
{
  // ===== 1. 声明所有参数（名称、默认值）=====
  this->declare_parameter<std::string>("map_topic",          "/elevation_mapping/output");
  this->declare_parameter<std::string>("terrain_topic",      "/perception/terrain_info");
  this->declare_parameter<std::string>("safety_cloud_topic", "/perception/safety_obstacles");

  this->declare_parameter<double>("roi_x_min",              0.3);
  this->declare_parameter<double>("roi_x_max",              1.0);
  this->declare_parameter<double>("roi_y_width",            0.6);
  this->declare_parameter<double>("fatal_obstacle_height",  0.35);
  this->declare_parameter<double>("min_valid_data_ratio",   0.4);
  this->declare_parameter<double>("safe_slope_threshold",   0.35);
  this->declare_parameter<double>("roughness_threshold",    0.08);
  this->declare_parameter<bool>  ("publish_virtual_wall",   true);

  // ===== 2. 读取参数 =====
  map_topic_            = this->get_parameter("map_topic").as_string();
  terrain_topic_        = this->get_parameter("terrain_topic").as_string();
  safety_cloud_topic_   = this->get_parameter("safety_cloud_topic").as_string();
  publish_virtual_wall_ = this->get_parameter("publish_virtual_wall").as_bool();

  roi_x_min_   = static_cast<float>(this->get_parameter("roi_x_min").as_double());
  roi_x_max_   = static_cast<float>(this->get_parameter("roi_x_max").as_double());
  roi_y_width_ = static_cast<float>(this->get_parameter("roi_y_width").as_double());

  config_.roi_x_min               = roi_x_min_;
  config_.roi_x_max               = roi_x_max_;
  config_.roi_y_width             = roi_y_width_;
  config_.fatal_obstacle_height   = static_cast<float>(
    this->get_parameter("fatal_obstacle_height").as_double());
  config_.min_valid_data_ratio    = static_cast<float>(
    this->get_parameter("min_valid_data_ratio").as_double());
  config_.safe_slope_threshold    = static_cast<float>(
    this->get_parameter("safe_slope_threshold").as_double());
  config_.roughness_threshold     = static_cast<float>(
    this->get_parameter("roughness_threshold").as_double());

  
  // 初始化 TF Buffer和 Listener（3行）
  tf_buffer_= std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ===== 3. 创建 OpenCL 核心 =====
  core_ = std::make_unique<TerrainAnalyzerCore>(config_);

  // ===== 4. 创建 ROS 订阅与发布 =====
  map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
    map_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&TerrainAnalyzerNode::mapCallback, this, std::placeholders::_1));

  terrain_pub_ = this->create_publisher<humanoid_interfaces::msg::TerrainInfo>(
    terrain_topic_, 10);

  safety_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    safety_cloud_topic_, 10);

  RCLCPP_INFO(this->get_logger(),
    "TerrainAnalyzerNode 已启动\n"
    "  订阅: %s\n"
    "  发布(地形): %s\n"
    "  发布(虚拟墙): %s\n"
    "  ROI: x=[%.2f, %.2f]m  y_width=%.2f m",
    map_topic_.c_str(),
    terrain_topic_.c_str(),
    safety_cloud_topic_.c_str(),
    roi_x_min_, roi_x_max_, roi_y_width_);
}

// ===========================================================================
// ROI 提取
// ===========================================================================
bool TerrainAnalyzerNode::extractROI(
  const grid_map_msgs::msg::GridMap & msg,
  std::vector<float> & roi,
  int & rows,
  int & cols)
{
  // =========================================================
  // 第一步：找到 elevation 层（与原来一致）
  // =========================================================
  auto layer_it = std::find(msg.layers.begin(), msg.layers.end(), "elevation");
  if (layer_it == msg.layers.end()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "GridMap 中未找到 'elevation' 层");
    return false;
  }
  size_t layer_idx =
    static_cast<size_t>(std::distance(msg.layers.begin(), layer_it));
  const auto & array_msg = msg.data[layer_idx];

  // =========================================================
  // 第二步：确定全图尺寸（与原来一致）
  // =========================================================
  int full_rows = 0, full_cols = 0;
  if (array_msg.layout.dim.size() >= 2) {
    full_cols = static_cast<int>(array_msg.layout.dim[0].size);
    full_rows = static_cast<int>(array_msg.layout.dim[1].size);
  } else {
    full_cols = static_cast<int>(msg.info.length_x / msg.info.resolution);
    full_rows = static_cast<int>(msg.info.length_y / msg.info.resolution);
  }
  if (full_rows <= 0 || full_cols <= 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "GridMap尺寸异常: rows=%d cols=%d", full_rows, full_cols);
    return false;
  }
  int expected_size = full_rows * full_cols;
  if (static_cast<int>(array_msg.data.size()) < expected_size) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "GridMap 数据长度不足: 期望 %d，实际 %zu",
      expected_size, array_msg.data.size());
    return false;
  }

  // =========================================================
  // ★ 第三步：查询机器人当前朝向（新增）
  //
  // 高程图的坐标系是 msg.header.frame_id（通常是 odom）
  // 我们需要知道 base_footprint 在 odom 中的朝向（yaw角）
  // 这样才能把"机器人前方"旋转到 odom 坐标系下
  // =========================================================
  const std::string map_frame  = msg.header.frame_id;  // 通常是 "odom"
  const std::string robot_frame = "base_footprint";

  // 机器人在高程图坐标系中的 yaw 角（弧度）
  double robot_yaw = 0.0;

  // 机器人在高程图坐标系中的位置（用于辅助验证，主要用msg.info.pose）
  // 高程图中心就是机器人位置，直接从 msg.info.pose 读取
  // 但朝向需要从 TF 查询
  try {
    // 查询 map_frame → robot_frame 的变换，获取机器人朝向
    geometry_msgs::msg::TransformStamped tf_stamped =
      tf_buffer_->lookupTransform(
        map_frame,// 目标坐标系（odom）
        robot_frame,      // 源坐标系（base_footprint）
        tf2::TimePointZero// 使用最新可用的变换
      );

    // 从四元数提取 yaw 角
    tf2::Quaternion q(
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z,
      tf_stamped.transform.rotation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    robot_yaw = yaw;
    last_robot_yaw_ = static_cast<float>(yaw);  // 保存成功的yaw值供下次使用
  } catch (const tf2::TransformException & ex) {
    // TF 查询失败时降级处理：
    // 使用 yaw=0（等同于原来的行为），并打印警告
    // 不直接返回 false，避免机器人启动初期 TF 未就绪时频繁失败
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "TF 查询失败 (%s→%s): %s，使用 yaw=0 降级处理",
      map_frame.c_str(), robot_frame.c_str(), ex.what());
      robot_yaw = last_robot_yaw_;
  }

  // =========================================================
  // ★ 第四步：在 base_footprint 坐标系下定义采样网格，
  //           旋转到 odom 坐标系，查询高程图格子值（新增）
  //
  // 坐标系约定：
  //   base_footprint: x=机器人前方, y=机器人左方
  //   odom:x=东/初始前方, y=北/初始左方
  //
  // 旋转公式（base_footprint → odom）：
  //   x_odom = x_robot * cos(yaw) - y_robot * sin(yaw)
  //   y_odom = x_robot * sin(yaw) + y_robot * cos(yaw)
  //
  // 高程图格子索引（grid_map 约定）：
  //   高程图中心对应像素 (center_r, center_c)
  //   odom 中x 增大 → 行索引减小（row = center_r - dx/res）
  //   odom 中 y 增大 → 列索引减小（col = center_c - dy/res）
  // =========================================================
  float res = static_cast<float>(msg.info.resolution);
  int center_r = full_rows / 2;
  int center_c = full_cols / 2;

  // 预计算旋转系数
  float cos_yaw = static_cast<float>(std::cos(robot_yaw));
  float sin_yaw = static_cast<float>(std::sin(robot_yaw));

  // 确定采样网格的行列数（在 base_footprint 坐标系下）
  // x方向：从 roi_x_min 到 roi_x_max，步长 = res
  // y 方向：从 -roi_y_width/2 到 +roi_y_width/2，步长 = res
  int sample_rows = static_cast<int>((roi_x_max_ - roi_x_min_) / res) + 1;
  int sample_cols = static_cast<int>(roi_y_width_ / res) + 1;

  if (sample_rows <= 0 || sample_cols <= 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "ROI 采样网格为空，请检查 roi_x_min/max/y_width 参数");
    return false;
  }

  roi.clear();
  roi.resize(static_cast<size_t>(sample_rows) * static_cast<size_t>(sample_cols),
             std::numeric_limits<float>::quiet_NaN());  // 默认填NaN（无效）

  // 逐格子采样
  for (int ri = 0; ri < sample_rows; ++ri) {
    // 在 base_footprint 坐标系下，x 从 roi_x_min 到 roi_x_max
    float x_robot = roi_x_min_ + static_cast<float>(ri) * res;

    for (int ci = 0; ci < sample_cols; ++ci) {
      // 在 base_footprint 坐标系下，y 从 -width/2 到 +width/2
      float y_robot = -roi_y_width_ * 0.5f + static_cast<float>(ci) * res;

      // ★ 旋转到 odom 坐标系
      float x_odom = x_robot * cos_yaw - y_robot * sin_yaw;
      float y_odom = x_robot * sin_yaw + y_robot * cos_yaw;

      // 转换为高程图像素坐标
      // grid_map 约定：x 增大 → row 减小，y 增大 → col 减小
      int map_r = center_r - static_cast<int>(std::round(x_odom / res));
      int map_c = center_c - static_cast<int>(std::round(y_odom / res));

      // 边界检查
      if (map_r < 0 || map_r >= full_rows || map_c < 0 || map_c >= full_cols) {
        // 超出地图范围，保持 NaN（已在 resize 时初始化）
        continue;
      }

      // 读取高程值
      float val = array_msg.data[map_r * full_cols + map_c];
      roi[static_cast<size_t>(ri) * static_cast<size_t>(sample_cols) + ci] = val;
    }
  }

  rows = sample_rows;
  cols = sample_cols;
  return true;
}


 
// ===========================================================================
// 虚拟墙发布
// ===========================================================================

void TerrainAnalyzerNode::publishVirtualWall(const std_msgs::msg::Header & header)
{
  // 虚拟墙几何参数（按机器人实际尺寸调整）
  const float wall_distance  = 0.3f;   // 机器人前方 0.3m（减小距离，减少误阻挡）
  const float wall_width     = 0.6f;   // 宽度 0.6m（匹配机器人肩宽）
  const float wall_height    = 0.3f;   // 高度 0.3m（降低高度，只阻挡脚部）
  const float point_spacing  = 0.1f;   // 点间距 0.1m（稀疏点云，减少计算量）

  // 生成墙上所有点（base_footprint 坐标系）
  std::vector<std::array<float, 3>> points;
  for (float y = -wall_width * 0.5f; y <= wall_width * 0.5f; y += point_spacing) {
    for (float z = 0.0f; z <= wall_height; z += point_spacing) {
      points.push_back({wall_distance, y, z});
    }
  }

  if (points.empty()) {
    return;
  }

  // 构建 PointCloud2 消息
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header          = header;
  cloud.header.frame_id = "base_footprint";
  cloud.height          = 1;
  cloud.width           = static_cast<uint32_t>(points.size());
  cloud.is_bigendian    = false;
  cloud.is_dense        = true;
  cloud.point_step      = 12;  // 3 * sizeof(float)
  cloud.row_step        = cloud.point_step * cloud.width;

  // 设置字段描述
  cloud.fields.resize(3);
  cloud.fields[0].name     = "x";
  cloud.fields[0].offset   = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count    = 1;

  cloud.fields[1].name     = "y";
  cloud.fields[1].offset   = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count    = 1;

  cloud.fields[2].name     = "z";
  cloud.fields[2].offset   = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count    = 1;

  // 序列化点数据
  cloud.data.resize(points.size() * 12);
  for (size_t i = 0; i < points.size(); ++i) {
    std::memcpy(&cloud.data[i * 12 + 0], &points[i][0], sizeof(float));
    std::memcpy(&cloud.data[i * 12 + 4], &points[i][1], sizeof(float));
    std::memcpy(&cloud.data[i * 12 + 8], &points[i][2], sizeof(float));
  }

  safety_cloud_pub_->publish(cloud);

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "[TerrainAnalyzer] 发布虚拟墙: %zu 点, 距机器人 %.2fm",
    points.size(), wall_distance);
}

// ===========================================================================
// 主回调
// ===========================================================================

void TerrainAnalyzerNode::mapCallback(
  const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  // ----- 1. 提取 ROI -----
  std::vector<float> roi;
  int rows = 0, cols = 0;

  if (!extractROI(*msg, roi, rows, cols)) {
    // ROI 提取失败：发布"不安全"保护信号
    humanoid_interfaces::msg::TerrainInfo out;
    out.is_safe_to_step = false;
    out.max_step_height = 0.0f;
    out.avg_slope       = 0.0f;
    out.roughness       = 99.0f;  // 极大粗糙度表示"数据缺失"
    terrain_pub_->publish(out);
    return;
  }

  // ----- 2. 调用 OpenCL 核心分析 -----
  TerrainAnalysisResult result = core_->analyze(
    roi, rows, cols,
    static_cast<float>(msg->info.resolution));

  // ----- 3. 封装并发布 TerrainInfo -----
  humanoid_interfaces::msg::TerrainInfo terrain_msg;
  terrain_msg.max_step_height = result.max_step_height;
  terrain_msg.avg_slope       = result.avg_slope;
  terrain_msg.roughness       = result.roughness;
  terrain_msg.is_safe_to_step = result.is_safe_to_step;
  terrain_pub_->publish(terrain_msg);

  // ----- 4. 不安全时发布虚拟墙 -----
  if (!result.is_safe_to_step && publish_virtual_wall_) {
    publishVirtualWall(msg->header);
  }

  // ----- 5. Debug 日志 -----
  RCLCPP_DEBUG(this->get_logger(),
    "地形分析: valid=%.2f step=%.3fm slope=%.3frad rough=%.4f safe=%s",
    result.valid_ratio,
    result.max_step_height,
    result.avg_slope,
    result.roughness,
    result.is_safe_to_step ? "YES" : "NO");
}

}  // namespace humanoid_terrain_analyzer

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<humanoid_terrain_analyzer::TerrainAnalyzerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}