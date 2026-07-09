/*
 * 文件功能：导航启动前的 TF 就绪门控节点。
 *
 * 主要职责：
 * 1. 按参数等待 target_frame -> source_frame 这条 TF 可查询。
 * 2. 要求连续 stable_count 次查询成功后主动退出，供 launch 的 OnProcessExit 继续启动下游节点。
 * 3. 支持 timeout_sec 超时失败退出，timeout_sec=0 表示一直等待。
 * 4. 以 C++ 实现短生命周期启动门控，减少 Python 运行时依赖和启动开销。
 *
 * 上游节点：
 * - prior_map_odom_bridge / dynamic_odom_ground_publisher / robot_state_publisher 等 TF 发布者。
 *
 * 下游节点：
 * - Nav2 planner/controller/behavior/bt_navigator 等依赖定位 TF 的导航核心节点。
 */

#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class WaitForTfNode final : public rclcpp::Node
{
public:
  WaitForTfNode()
  : Node("wait_for_tf"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    source_frame_ = declare_parameter<std::string>("source_frame", "base_footprint");
    timeout_sec_ = declare_parameter<double>("timeout_sec", 0.0);
    poll_period_sec_ = std::max(0.05, declare_parameter<double>("poll_period", 0.2));
    stable_count_required_ = std::max<int>(
      1,
      static_cast<int>(declare_parameter<int>("stable_count", 3)));

    start_time_ = now();
    last_warn_time_ = start_time_;

    RCLCPP_INFO(
      get_logger(),
      "等待 TF %s -> %s 后再启动依赖节点",
      target_frame_.c_str(),
      source_frame_.c_str());

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(poll_period_sec_)),
      [this]() { on_timer(); });
  }

private:
  void on_timer()
  {
    try {
      (void)tf_buffer_.lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
      ++stable_count_;
      if (stable_count_ >= stable_count_required_) {
        RCLCPP_INFO(
          get_logger(),
          "TF 已就绪: %s -> %s，下游节点可以启动",
          target_frame_.c_str(),
          source_frame_.c_str());
        rclcpp::shutdown();
      }
      return;
    } catch (const tf2::TransformException & ex) {
      stable_count_ = 0;
      const auto current_time = now();
      if ((current_time - last_warn_time_).seconds() > 5.0) {
        RCLCPP_WARN(
          get_logger(),
          "TF 尚未就绪: %s -> %s: %s",
          target_frame_.c_str(),
          source_frame_.c_str(),
          ex.what());
        last_warn_time_ = current_time;
      }
    }

    if (timeout_sec_ > 0.0 && (now() - start_time_).seconds() > timeout_sec_) {
      RCLCPP_ERROR(
        get_logger(),
        "等待 TF 超时: %s -> %s",
        target_frame_.c_str(),
        source_frame_.c_str());
      rclcpp::shutdown();
      std::exit(1);
    }
  }

  std::string target_frame_;
  std::string source_frame_;
  double timeout_sec_{0.0};
  double poll_period_sec_{0.2};
  int stable_count_required_{3};
  int stable_count_{0};
  rclcpp::Time start_time_;
  rclcpp::Time last_warn_time_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaitForTfNode>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
