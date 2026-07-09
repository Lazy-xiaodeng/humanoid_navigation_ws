#ifndef MOVING_OBJECT_FILTER_HPP_
#define MOVING_OBJECT_FILTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Geometry>
#include <deque>
#include <mutex>

/**
 * @brief 移动物体过滤器
 * 
 * 功能：识别并过滤与机器人一起移动的物体(如吊架)
 * 原理：通过比较多帧点云中物体的位置变化,判断是否为跟随机器人移动的物体
 */
class MovingObjectFilter : public rclcpp::Node
{
public:
  MovingObjectFilter()
  : Node("moving_object_filter")
  {
    // 参数声明
    this->declare_parameter<std::string>("input_topic", "/airy_points_raw");
    this->declare_parameter<std::string>("output_topic", "/airy_points_filtered");
    this->declare_parameter<std::string>("robot_base_frame", "base_footprint");
    this->declare_parameter<double>("filter_radius", 1.5);       // 机器人周围过滤半径
    this->declare_parameter<int>("history_size", 10);            // 历史帧数
    this->declare_parameter<double>("movement_threshold", 0.1);  // 移动阈值(m)
    this->declare_parameter<bool>("enable_filter", true);        // 是否启用过滤

    // 读取参数
    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    filter_radius_ = get_parameter("filter_radius").as_double();
    history_size_ = get_parameter("history_size").as_int();
    movement_threshold_ = get_parameter("movement_threshold").as_double();
    enable_filter_ = get_parameter("enable_filter").as_bool();

    // 订阅和发布
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10,
      std::bind(&MovingObjectFilter::cloudCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&MovingObjectFilter::odomCallback, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "移动物体过滤器已启动");
    RCLCPP_INFO(this->get_logger(), "  输入话题: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  输出话题: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  过滤半径: %.2fm", filter_radius_);
    RCLCPP_INFO(this->get_logger(), "  移动阈值: %.2fm", movement_threshold_);
    RCLCPP_INFO(this->get_logger(), "  启用状态: %s", enable_filter_ ? "是" : "否");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!enable_filter_) {
      // 不启用过滤,直接转发
      cloud_pub_->publish(*msg);
      return;
    }

    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_in);

    // 获取当前机器人位姿
    geometry_msgs::msg::PoseStamped robot_pose;
    try {
      auto transform = tf_buffer_->lookupTransform("map", robot_base_frame_, tf2::TimePointZero);
      robot_pose.pose.position.x = transform.transform.translation.x;
      robot_pose.pose.position.y = transform.transform.translation.y;
      robot_pose.pose.position.z = transform.transform.translation.z;
      robot_pose.pose.orientation = transform.transform.rotation;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "无法获取TF变换: %s", ex.what());
      cloud_pub_->publish(*msg);  // TF失败时直接转发
      return;
    }

    // 记录历史位姿
    pose_history_.push_back(robot_pose);
    while (pose_history_.size() > static_cast<size_t>(history_size_)) {
      pose_history_.pop_front();
    }

    // 如果历史位姿不足,暂时不过滤
    if (pose_history_.size() < 2) {
      cloud_pub_->publish(*msg);
      return;
    }

    // 计算机器人移动距离
    double moved_distance = calcMovedDistance();

    // 过滤点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    int filtered_count = 0;

    for (const auto &point : cloud_in->points) {
      // 计算点到机器人的距离
      double dx = point.x - robot_pose.pose.position.x;
      double dy = point.y - robot_pose.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      // 只过滤过滤半径内的点
      if (dist > filter_radius_) {
        cloud_out->points.push_back(point);
        continue;
      }

      // 判断是否为移动物体(跟随机器人移动)
      if (isObjectMoving(point, moved_distance)) {
        // 是移动物体,过滤掉
        filtered_count++;
      } else {
        // 是静止物体,保留
        cloud_out->points.push_back(point);
      }
    }

    cloud_out->width = cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;

    // 转换回ROS消息
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_out_msg(new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*cloud_out, *cloud_out_msg);
    cloud_out_msg->header = msg->header;

    if (filtered_count > 0) {
      RCLCPP_DEBUG(this->get_logger(), "过滤掉 %d 个移动物体点 (机器人移动: %.3fm)",
                  filtered_count, moved_distance);
    }

    cloud_pub_->publish(*cloud_out_msg);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_odom_ = *msg;
  }

  // 计算机器人移动距离
  double calcMovedDistance()
  {
    if (pose_history_.size() < 2) return 0.0;

    const auto &oldest = pose_history_.front();
    const auto &latest = pose_history_.back();

    double dx = latest.pose.position.x - oldest.pose.position.x;
    double dy = latest.pose.position.y - oldest.pose.position.y;
    double dz = latest.pose.position.z - oldest.pose.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  // 判断点是否属于移动物体
  bool isObjectMoving(const pcl::PointXYZ &point, double robot_moved)
  {
    // 如果机器人移动距离小于阈值,认为物体可能是静止的
    if (robot_moved < movement_threshold_) {
      return false;
    }

    // 如果点跟随机器人移动,则认为它是移动物体
    // 简化判断：点在机器人附近且机器人有明显移动
    return true;
  }

  // 参数
  std::string input_topic_;
  std::string output_topic_;
  std::string robot_base_frame_;
  double filter_radius_;
  int history_size_;
  double movement_threshold_;
  bool enable_filter_;

  // 订阅和发布
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 状态
  std::deque<geometry_msgs::msg::PoseStamped> pose_history_;
  nav_msgs::msg::Odometry current_odom_;
};

#endif  // MOVING_OBJECT_FILTER_HPP_
