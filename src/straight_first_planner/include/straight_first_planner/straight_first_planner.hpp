#ifndef STRAIGHT_FIRST_PLANNER__STRAIGHT_FIRST_PLANNER_HPP_
#define STRAIGHT_FIRST_PLANNER__STRAIGHT_FIRST_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_theta_star_planner/theta_star_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace straight_first_planner
{

class StraightFirstPlanner : public nav2_core::GlobalPlanner
{
public:
  StraightFirstPlanner() = default;
  ~StraightFirstPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  bool straightLineIsSafe(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) const;

  nav_msgs::msg::Path makeStraightPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

  bool isCostSafe(unsigned char cost) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::string global_frame_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Logger logger_{rclcpp::get_logger("StraightFirstPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  nav2_theta_star_planner::ThetaStarPlanner fallback_;

  double interpolation_resolution_{0.05};
  double straight_check_resolution_{0.025};
  unsigned char max_straight_cost_{20};
  bool allow_unknown_{false};
  bool use_goal_orientation_{false};
};

}  // namespace straight_first_planner

#endif  // STRAIGHT_FIRST_PLANNER__STRAIGHT_FIRST_PLANNER_HPP_
