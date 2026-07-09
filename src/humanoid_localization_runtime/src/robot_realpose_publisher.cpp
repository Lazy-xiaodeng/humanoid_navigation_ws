/*
 * robot_realpose_publisher.cpp
 *
 * 文件作用：
 * 1. 周期性查询 global_frame -> base_frame 的 TF。
 * 2. 将机器人当前全局位姿发布为 /robot_realpose。
 * 3. 给 APP 展示、数据整合和调试工具提供统一真实位姿输入。
 *
 * 上游：定位桥和 TF 树。
 * 下游：APP 数据整合层、导航页面实时位置显示和调试工具。
 */

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class RobotRealposePublisher : public rclcpp::Node
{
public:
  RobotRealposePublisher()
  : Node("robot_realpose_publisher"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    auto publish_rate = declare_parameter<double>("publish_rate", 10.0);
    if (publish_rate <= 0.0) {
      publish_rate = 10.0;
    }

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/robot_realpose", 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&RobotRealposePublisher::timer_callback, this));

    RCLCPP_INFO(
      get_logger(), "robot_realpose_publisher started: %s -> %s, publish to /robot_realpose",
      global_frame_.c_str(), base_frame_.c_str());
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to get TF %s->%s: %s", global_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = global_frame_;
    msg.pose.pose.position.x = transform.transform.translation.x;
    msg.pose.pose.position.y = transform.transform.translation.y;
    msg.pose.pose.position.z = transform.transform.translation.z;
    msg.pose.pose.orientation = transform.transform.rotation;
    msg.pose.covariance.fill(0.0);
    pub_->publish(msg);

    if (!tf_ready_logged_) {
      RCLCPP_INFO(get_logger(), "TF ready: %s->%s", global_frame_.c_str(), base_frame_.c_str());
      tf_ready_logged_ = true;
    }
  }

  std::string global_frame_;
  std::string base_frame_;
  bool tf_ready_logged_{false};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotRealposePublisher>());
  rclcpp::shutdown();
  return 0;
}
