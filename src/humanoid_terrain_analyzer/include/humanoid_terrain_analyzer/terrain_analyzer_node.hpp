#pragma once

/**
 * @file terrain_analyzer_node.hpp
 * @brief 地形分析 ROS2 节点头文件
 *
 * 职责：
 *   - 订阅 GridMap（/elevation_mapping/output）
 *   - 提取 elevation 层
 *   - 裁剪机器人前方 ROI
 *   - 调用 TerrainAnalyzerCore 进行分析
 *   - 发布 TerrainInfo 到 /perception/terrain_info
 *   - 不安全时发布虚拟墙点云到 /perception/safety_obstacles
 */

#include <grid_map_msgs/msg/grid_map.hpp>
#include <humanoid_interfaces/msg/terrain_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <string>
#include <vector>

// 引用本包内的核心类
#include "humanoid_terrain_analyzer/terrain_analyzer_core.hpp"

namespace humanoid_terrain_analyzer
{

/**
 * @brief 地形分析 ROS2 节点
 *
 * 纯粹负责 ROS 层面的通信和数据转换：
 *   - 不做任何数学计算
 *   - 所有计算全部委托给 TerrainAnalyzerCore
 */
class TerrainAnalyzerNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   * @param options ROS2 节点选项
   */
  explicit TerrainAnalyzerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief GridMap 回调函数
   *
   * 每次收到高程图时触发，完成一次地形分析周期。
   */
  void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

  /**
   * @brief 从 GridMap 消息中提取 elevation 层 ROI
   *
   * @param msg    输入 GridMap 消息
   * @param roi    输出 ROI 数据（行优先 float 数组）
   * @param rows   输出 ROI 行数
   * @param cols   输出 ROI 列数
   * @return true  提取成功
   * @return false 提取失败（层不存在、数据量不足等）
   */
  bool extractROI(
    const grid_map_msgs::msg::GridMap & msg,
    std::vector<float> & roi,
    int & rows,
    int & cols);

  /**
   * @brief 发布虚拟墙点云
   *
   * 当地形不安全时，在机器人前方发布一堵障碍物点云，
   * 通知 Nav2 代价地图该区域不可通行。
   *
   * @param header 原始地图消息的 header（复用时间戳）
   */
  void publishVirtualWall(const std_msgs::msg::Header & header);

  // ===== ROS 参数 =====
  std::string map_topic_;             ///< 订阅的 GridMap 话题
  std::string terrain_topic_;         ///< 发布的 TerrainInfo 话题
  std::string safety_cloud_topic_;    ///< 发布的虚拟墙点云话题
  bool        publish_virtual_wall_;  ///< 是否发布虚拟墙

  // ===== ROI 提取参数（冗余存一份，方便 extractROI 使用）=====
  float roi_x_min_;    ///< ROI 前方最近距离（米）
  float roi_x_max_;    ///< ROI 前方最远距离（米）
  float roi_y_width_;  ///< ROI 横向宽度（米）

  // ===== 虚拟墙参数（新增）=====
  float wall_distance_;       ///< 虚拟墙距离机器人前方（米）
  float wall_width_;          ///< 虚拟墙宽度（米）
  float wall_height_;         ///< 虚拟墙高度（米）
  float wall_point_spacing_;  ///< 虚拟墙点间距（米）

  // ===== ROS 接口 =====
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Publisher<humanoid_interfaces::msg::TerrainInfo>::SharedPtr terrain_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr safety_cloud_pub_;

  // TF2 成员变量
  std::shared_ptr<tf2_ros::Buffer>tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  float last_robot_yaw_{0.0f};  ///< 上次成功查询的机器人yaw角（TF失败时复用）

  // ===== 核心分析器 =====
  TerrainAnalyzerConfig                config_;  ///< 配置参数
  std::unique_ptr<TerrainAnalyzerCore> core_;    ///< OpenCL 分析核心
};

}  // namespace humanoid_terrain_analyzer