/*
 * nav2_action_controller.hpp
 *
 * 文件用途：
 * 1. 定义 Nav2 action 控制器模块边界。
 * 2. 集中创建并持有 NavigateToPose / NavigateThroughPoses action client。
 * 3. 集中保存当前 route task 的 goal handle，统一处理取消和清理。
 * 4. 节点外壳仍负责路线任务状态机、事件发布和 generation/version 校验；
 *    本模块只管理 Nav2 action 通道，避免业务状态散落到 action 资源层。
 *
 * 当前设计：
 * 1. action client / goal handle 放在本模块，减少 navigation_state_manager 的资源持有职责。
 * 2. goal response / feedback / result 回调仍由节点外壳提供 lambda，确保旧 goal 隔离和事件发布逻辑不变。
 * 3. cancel_active_goals() 只发送取消并清空本地 handle，不改变路线状态；路线状态由调用方维护。
 */

#pragma once

#include <chrono>
#include <string>

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace humanoid_route_runtime
{

class Nav2ActionController
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  void configure(std::string navigate_to_pose_action, std::string navigate_through_poses_action);
  void create_clients(rclcpp::Node * node);

  const std::string & navigate_to_pose_action() const;
  const std::string & navigate_through_poses_action() const;

  bool wait_for_navigate_to_pose_server(std::chrono::nanoseconds timeout);
  bool wait_for_navigate_through_poses_server(std::chrono::nanoseconds timeout);

  void async_send_navigate_to_pose_goal(
    const NavigateToPose::Goal & goal,
    const rclcpp_action::Client<NavigateToPose>::SendGoalOptions & options);
  void async_send_navigate_through_poses_goal(
    const NavigateThroughPoses::Goal & goal,
    const rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions & options);

  void set_final_pose_goal_handle(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  void set_through_goal_handle(const GoalHandleNavigateThroughPoses::SharedPtr & goal_handle);
  void clear_final_pose_goal_handle();
  void clear_through_goal_handle();
  void clear_goal_handles();
  bool has_final_pose_goal() const;
  bool has_through_goal() const;
  bool has_active_goal() const;

  void cancel_active_goals();

private:
  std::string navigate_to_pose_action_;
  std::string navigate_through_poses_action_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_through_poses_client_;
  GoalHandleNavigateToPose::SharedPtr final_pose_goal_handle_;
  GoalHandleNavigateThroughPoses::SharedPtr through_goal_handle_;
};

}  // namespace humanoid_route_runtime
