// 自定义控制器：Regulated Pure Pursuit + 渐进式角速度增强
// 作用：当角速度过低时，采用渐进增强的方式，避免阶跃式跳变
// 逻辑：
//   1. 如果角速度 >= min_angular_vel_，保持原值
//   2. 如果 0 < |角速度| < min_angular_vel_，记录累计偏差
//   3. 逐步增强角速度，从原始值平滑过渡到 min_angular_vel_
//   4. 如果机器人实际没有转动（角速度接近0），则逐步加大增强力度

#include <memory>
#include <algorithm>
#include <cmath>
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace humanoid_navigation2
{

class RegulatedPurePursuitWithMinAngular
  : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  RegulatedPurePursuitWithMinAngular()
  : min_angular_vel_(0.15),
    enable_min_angular_(true),
    enhancement_gain_(0.3),      // 渐进增强增益（每次增强的步长）
    max_enhancement_(0.5),       // 最大增强角速度（防止过大）
    prev_angular_z_(0.0),        // 上一次角速度
    enhancement_accum_(0.0)      // 增强累计值
  {
  }

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    // 先调用父类配置
    RegulatedPurePursuitController::configure(parent, name, tf, costmap_ros);

    // 获取父类 node 指针用于日志
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("Failed to lock node");
    }

    // 声明并读取参数
    node->declare_parameter(name + ".min_angular_velocity", 0.15);
    node->get_parameter(name + ".min_angular_velocity", min_angular_vel_);

    node->declare_parameter(name + ".enable_min_angular_limit", true);
    node->get_parameter(name + ".enable_min_angular_limit", enable_min_angular_);

    node->declare_parameter(name + ".enhancement_gain", 0.3);
    node->get_parameter(name + ".enhancement_gain", enhancement_gain_);

    node->declare_parameter(name + ".max_enhancement", 0.5);
    node->get_parameter(name + ".max_enhancement", max_enhancement_);

    RCLCPP_INFO(
      node->get_logger(),
      "[RegulatedPurePursuitWithMinAngular] 渐进式角速度增强 - 最小: %.3f, 增益: %.3f, 最大: %.3f, 启用: %s",
      min_angular_vel_, enhancement_gain_, max_enhancement_, enable_min_angular_ ? "是" : "否");
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    // 调用父类（RPP）计算速度
    auto cmd = RegulatedPurePursuitController::computeVelocityCommands(
      pose, velocity, goal_checker);

    // 渐进式角速度增强
    if (enable_min_angular_) {
      double & angular_z = cmd.twist.angular.z;
      double abs_angular = std::abs(angular_z);

      if (abs_angular > 0.0 && abs_angular < min_angular_vel_) {
        // 情况1：角速度低于最小值，需要渐进增强
        // 计算增强累计值：如果当前角速度持续偏低，逐步加大增强
        if (abs_angular < prev_angular_z_ || abs_angular < 0.05) {
          // 角速度在减小或接近0，说明机器人可能转不动，增加增强力度
          enhancement_accum_ += enhancement_gain_ * 0.1;  // 缓慢增加
        } else {
          // 角速度在增大，说明机器人正在响应，减少增强力度
          enhancement_accum_ = std::max(0.0, enhancement_accum_ - enhancement_gain_ * 0.2);
        }

        // 计算增强后的角速度：原始值 + 增强累计值
        double enhanced = abs_angular + enhancement_accum_;

        // 限制范围：[min_angular_vel_, max_enhancement_]
        enhanced = std::clamp(enhanced, min_angular_vel_, max_enhancement_);

        // 应用增强（保持原始方向）
        angular_z = enhanced * (angular_z > 0 ? 1.0 : -1.0);

      } else if (abs_angular >= min_angular_vel_) {
        // 情况2：角速度已经足够大，重置增强累计值
        enhancement_accum_ = 0.0;
      } else {
        // 情况3：角速度为0（不需要转），重置增强累计值
        enhancement_accum_ = 0.0;
      }

      prev_angular_z_ = abs_angular;
    }

    return cmd;
  }

protected:
  double min_angular_vel_;   // 最小角速度 (rad/s)
  bool enable_min_angular_;  // 是否启用限制
  double enhancement_gain_;  // 渐进增强增益
  double max_enhancement_;   // 最大增强角速度
  double prev_angular_z_;    // 上一次角速度绝对值
  double enhancement_accum_; // 增强累计值
};

}  // namespace humanoid_navigation2

// 注册插件
PLUGINLIB_EXPORT_CLASS(
  humanoid_navigation2::RegulatedPurePursuitWithMinAngular,
  nav2_core::Controller)
