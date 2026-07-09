/*
 * dynamic_odom_ground_publisher.cpp
 *
 * 文件作用：
 * 1. 根据 parent_frame -> base_frame 的实时 TF，动态发布 parent_frame -> child_frame 的地面投影 TF。
 * 2. 正式导航中会启动两份：一份生成 map_ground，一份生成 odom_ground。
 * 3. 该节点让 Nav2 costmap 使用贴地坐标系，减少机器人高度变化对 2D costmap 的影响。
 *
 * 上游：TF 树中的 map/odom 到 base_footprint。
 * 下游：Nav2 global/local costmap、map_server 和依赖 map_ground/odom_ground 的定位展示链路。
 */

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class DynamicOdomGroundPublisher : public rclcpp::Node
{
public:
  DynamicOdomGroundPublisher()
  : Node("dynamic_odom_ground_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    parent_frame_ = declare_parameter<std::string>("parent_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    child_frame_ = declare_parameter<std::string>("child_frame", "odom_ground");
    z_offset_ = declare_parameter<double>("z_offset", 0.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    auto publish_rate = declare_parameter<double>("publish_rate", 30.0);
    if (publish_rate <= 0.0) {
      publish_rate = 30.0;
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&DynamicOdomGroundPublisher::publish_ground_tf, this));

    RCLCPP_INFO(
      get_logger(), "publishing %s->%s from %s->%s height",
      parent_frame_.c_str(), child_frame_.c_str(), parent_frame_.c_str(), base_frame_.c_str());
  }

private:
  void publish_ground_tf()
  {
    geometry_msgs::msg::TransformStamped parent_to_base;
    try {
      parent_to_base = tf_buffer_.lookupTransform(
        parent_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "waiting for %s->%s: %s", parent_frame_.c_str(), base_frame_.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = parent_frame_;
    msg.child_frame_id = child_frame_;
    msg.transform.translation.x = 0.0;
    msg.transform.translation.y = 0.0;
    msg.transform.translation.z = parent_to_base.transform.translation.z + z_offset_;
    msg.transform.rotation.x = 0.0;
    msg.transform.rotation.y = 0.0;
    msg.transform.rotation.z = 0.0;
    msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(msg);
  }

  std::string parent_frame_;
  std::string base_frame_;
  std::string child_frame_;
  double z_offset_{0.0};
  double tf_timeout_sec_{0.05};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicOdomGroundPublisher>());
  rclcpp::shutdown();
  return 0;
}
