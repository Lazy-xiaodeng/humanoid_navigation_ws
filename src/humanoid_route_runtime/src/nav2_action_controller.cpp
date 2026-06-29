/*
 * nav2_action_controller.cpp
 *
 * 文件用途：
 * 1. 创建并持有 NavigateToPose / NavigateThroughPoses action client。
 * 2. 保存当前 route task 的 through/final goal handle。
 * 3. 提供统一发送、等待、取消和清理入口，让 ROS 节点外壳专注业务状态和事件发布。
 */

#include "humanoid_route_runtime/nav2_action_controller.hpp"

#include <utility>

namespace humanoid_route_runtime
{

void Nav2ActionController::configure(
  std::string navigate_to_pose_action,
  std::string navigate_through_poses_action)
{
  navigate_to_pose_action_ = std::move(navigate_to_pose_action);
  navigate_through_poses_action_ = std::move(navigate_through_poses_action);
}

void Nav2ActionController::create_clients(rclcpp::Node * node)
{
  nav_to_pose_client_ =
    rclcpp_action::create_client<NavigateToPose>(node, navigate_to_pose_action_);
  nav_through_poses_client_ =
    rclcpp_action::create_client<NavigateThroughPoses>(node, navigate_through_poses_action_);
}

const std::string & Nav2ActionController::navigate_to_pose_action() const
{
  return navigate_to_pose_action_;
}

const std::string & Nav2ActionController::navigate_through_poses_action() const
{
  return navigate_through_poses_action_;
}

bool Nav2ActionController::wait_for_navigate_to_pose_server(std::chrono::nanoseconds timeout)
{
  return nav_to_pose_client_ && nav_to_pose_client_->wait_for_action_server(timeout);
}

bool Nav2ActionController::wait_for_navigate_through_poses_server(std::chrono::nanoseconds timeout)
{
  return nav_through_poses_client_ && nav_through_poses_client_->wait_for_action_server(timeout);
}

void Nav2ActionController::async_send_navigate_to_pose_goal(
  const NavigateToPose::Goal & goal,
  const rclcpp_action::Client<NavigateToPose>::SendGoalOptions & options)
{
  nav_to_pose_client_->async_send_goal(goal, options);
}

void Nav2ActionController::async_send_navigate_through_poses_goal(
  const NavigateThroughPoses::Goal & goal,
  const rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions & options)
{
  nav_through_poses_client_->async_send_goal(goal, options);
}

void Nav2ActionController::set_final_pose_goal_handle(
  const GoalHandleNavigateToPose::SharedPtr & goal_handle)
{
  final_pose_goal_handle_ = goal_handle;
}

void Nav2ActionController::set_through_goal_handle(
  const GoalHandleNavigateThroughPoses::SharedPtr & goal_handle)
{
  through_goal_handle_ = goal_handle;
}

void Nav2ActionController::clear_final_pose_goal_handle()
{
  final_pose_goal_handle_.reset();
}

void Nav2ActionController::clear_through_goal_handle()
{
  through_goal_handle_.reset();
}

void Nav2ActionController::clear_goal_handles()
{
  through_goal_handle_.reset();
  final_pose_goal_handle_.reset();
}

bool Nav2ActionController::has_final_pose_goal() const
{
  return static_cast<bool>(final_pose_goal_handle_);
}

bool Nav2ActionController::has_through_goal() const
{
  return static_cast<bool>(through_goal_handle_);
}

bool Nav2ActionController::has_active_goal() const
{
  return has_through_goal() || has_final_pose_goal();
}

void Nav2ActionController::cancel_active_goals()
{
  if (through_goal_handle_ && nav_through_poses_client_) {
    nav_through_poses_client_->async_cancel_goal(through_goal_handle_);
    through_goal_handle_.reset();
  }
  if (final_pose_goal_handle_ && nav_to_pose_client_) {
    nav_to_pose_client_->async_cancel_goal(final_pose_goal_handle_);
    final_pose_goal_handle_.reset();
  }
}

}  // namespace humanoid_route_runtime
