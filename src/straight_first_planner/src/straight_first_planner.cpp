#include "straight_first_planner/straight_first_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace straight_first_planner
{

void StraightFirstPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_node_ = parent;
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock parent node");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  clock_ = node_->get_clock();
  logger_ = node_->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".straight_check_resolution", rclcpp::ParameterValue(0.025));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".max_straight_cost", rclcpp::ParameterValue(20));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_goal_orientation", rclcpp::ParameterValue(false));

  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node_->get_parameter(name_ + ".straight_check_resolution", straight_check_resolution_);
  int max_straight_cost = 20;
  node_->get_parameter(name_ + ".max_straight_cost", max_straight_cost);
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node_->get_parameter(name_ + ".use_goal_orientation", use_goal_orientation_);

  max_straight_cost = std::clamp(max_straight_cost, 0, 252);
  max_straight_cost_ = static_cast<unsigned char>(max_straight_cost);
  interpolation_resolution_ = std::max(interpolation_resolution_, costmap_->getResolution());
  straight_check_resolution_ = std::max(straight_check_resolution_, costmap_->getResolution() * 0.5);

  fallback_.configure(parent, name, tf, costmap_ros);

  RCLCPP_INFO(
    logger_,
    "Configured StraightFirstPlanner: max_straight_cost=%u, allow_unknown=%s, "
    "interpolation_resolution=%.3f, straight_check_resolution=%.3f",
    max_straight_cost_, allow_unknown_ ? "true" : "false",
    interpolation_resolution_, straight_check_resolution_);
}

void StraightFirstPlanner::cleanup()
{
  fallback_.cleanup();
  RCLCPP_INFO(logger_, "Cleaning up StraightFirstPlanner");
}

void StraightFirstPlanner::activate()
{
  fallback_.activate();
  RCLCPP_INFO(logger_, "Activating StraightFirstPlanner");
}

void StraightFirstPlanner::deactivate()
{
  fallback_.deactivate();
  RCLCPP_INFO(logger_, "Deactivating StraightFirstPlanner");
}

nav_msgs::msg::Path StraightFirstPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  if (straightLineIsSafe(start, goal, cancel_checker)) {
    return makeStraightPlan(start, goal);
  }

  RCLCPP_DEBUG(logger_, "Straight path is not safe, falling back to ThetaStar");
  return fallback_.createPlan(start, goal, cancel_checker);
}

bool StraightFirstPlanner::straightLineIsSafe(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker) const
{
  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double distance = std::hypot(dx, dy);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / straight_check_resolution_)));

  for (int i = 0; i <= steps; ++i) {
    if (i % 64 == 0 && cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("StraightFirstPlanner was cancelled");
    }

    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double wx = start.pose.position.x + t * dx;
    const double wy = start.pose.position.y + t * dy;

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      return false;
    }

    if (!isCostSafe(costmap_->getCost(mx, my))) {
      return false;
    }
  }

  return true;
}

nav_msgs::msg::Path StraightFirstPlanner::makeStraightPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double distance = std::hypot(dx, dy);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / interpolation_resolution_)));
  const double path_yaw = std::atan2(dy, dx);

  tf2::Quaternion heading;
  heading.setRPY(0.0, 0.0, path_yaw);
  const auto heading_msg = tf2::toMsg(heading);

  path.poses.reserve(static_cast<size_t>(steps + 1));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = start.pose.position.x + t * dx;
    pose.pose.position.y = start.pose.position.y + t * dy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = heading_msg;
    path.poses.push_back(pose);
  }

  if (!path.poses.empty()) {
    path.poses.front().pose.orientation = heading_msg;
    path.poses.back().pose.orientation = use_goal_orientation_ ? goal.pose.orientation : heading_msg;
  }

  return path;
}

bool StraightFirstPlanner::isCostSafe(unsigned char cost) const
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }

  if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    return false;
  }

  return cost <= max_straight_cost_;
}

}  // namespace straight_first_planner

PLUGINLIB_EXPORT_CLASS(straight_first_planner::StraightFirstPlanner, nav2_core::GlobalPlanner)
