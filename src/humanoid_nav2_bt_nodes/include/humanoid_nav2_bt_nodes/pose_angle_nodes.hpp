#ifndef HUMANOID_NAV2_BT_NODES__POSE_ANGLE_NODES_HPP_
#define HUMANOID_NAV2_BT_NODES__POSE_ANGLE_NODES_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace humanoid_nav2_bt_nodes
{

class MakePoseTowardGoal : public BT::SyncActionNode
{
public:
  MakePoseTowardGoal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();

    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Original navigation goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "output_goal", "Goal with yaw pointing from the robot toward the goal position"),
      BT::InputPort<std::string>(
        "global_frame", "", "Frame used for angle calculation. Empty means goal frame."),
      BT::InputPort<std::string>(
        "robot_base_frame", "base_footprint", "Robot base frame"),
      BT::InputPort<double>(
        "min_heading_distance", 0.15,
        "If robot is closer than this distance, preserve current yaw"),
      BT::InputPort<std::string>(
        "heading_mode", "toward",
        "toward: final path yaw points toward goal, away: final path yaw points away from goal"),
      BT::InputPort<double>("transform_tolerance", 0.5, "TF lookup timeout in seconds")
    };
  }

  BT::NodeStatus tick() override;

private:
  bool getInputData(
    geometry_msgs::msg::PoseStamped & goal,
    std::string & global_frame,
    std::string & robot_base_frame,
    double & min_heading_distance,
    std::string & heading_mode,
    double & transform_tolerance);

  geometry_msgs::msg::PoseStamped transformGoal(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & global_frame,
    double transform_tolerance) const;

  geometry_msgs::msg::PoseStamped getRobotPose(
    const std::string & global_frame,
    const std::string & robot_base_frame,
    double transform_tolerance) const;

  std::string resolveGlobalFrame(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & configured_frame) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

class SpinToPose : public BT::StatefulActionNode
{
public:
  using Action = nav2_msgs::action::Spin;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using WrappedResult = GoalHandle::WrappedResult;

  SpinToPose(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();

    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Original navigation goal"),
      BT::InputPort<std::string>(
        "mode", "goal_yaw",
        "goal_position: face goal XY, goal_position_reverse: face away from goal XY, "
        "goal_yaw: face final goal yaw"),
      BT::InputPort<std::string>(
        "global_frame", "", "Frame used for angle calculation. Empty means goal frame."),
      BT::InputPort<std::string>(
        "robot_base_frame", "base_footprint", "Robot base frame"),
      BT::InputPort<double>("yaw_tolerance", 0.25, "Yaw tolerance in radians"),
      BT::InputPort<double>(
        "position_tolerance", 0.15,
        "Position distance below which goal_position mode is already satisfied"),
      BT::InputPort<double>("transform_tolerance", 0.5, "TF lookup timeout in seconds"),
      BT::InputPort<double>("time_allowance", 20.0, "Spin action time allowance in seconds"),
      BT::InputPort<int>("max_attempts", 3, "Maximum spin attempts before failure"),
      BT::InputPort<std::string>("server_name", "spin", "Spin action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout"),
      BT::OutputPort<Action::Result::_error_code_type>(
        "error_code_id", "Spin action error code")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    IDLE,
    WAITING_FOR_GOAL_HANDLE,
    WAITING_FOR_RESULT
  };

  struct AngleCheck
  {
    bool aligned{false};
    double angle{0.0};
  };

  bool readRuntimeInputs();
  BT::NodeStatus sendSpinIfNeeded();
  AngleCheck computeAngleToTarget() const;

  geometry_msgs::msg::PoseStamped transformGoal(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & global_frame,
    double transform_tolerance) const;

  geometry_msgs::msg::PoseStamped getRobotPose(
    const std::string & global_frame,
    const std::string & robot_base_frame,
    double transform_tolerance) const;

  std::string resolveGlobalFrame(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & configured_frame) const;

  void resetActionState();
  void setErrorCode(Action::Result::_error_code_type error_code);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp_action::Client<Action>::SharedPtr action_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped goal_;
  std::string mode_;
  std::string configured_global_frame_;
  std::string robot_base_frame_;
  double yaw_tolerance_{0.25};
  double position_tolerance_{0.15};
  double transform_tolerance_{0.5};
  double time_allowance_{20.0};
  int max_attempts_{3};
  int attempts_{0};

  Phase phase_{Phase::IDLE};
  std::chrono::steady_clock::time_point goal_sent_steady_time_;
  std::chrono::milliseconds server_timeout_{1000};
  std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;
  std::shared_future<WrappedResult> future_result_;
  GoalHandle::SharedPtr goal_handle_;
};

}  // namespace humanoid_nav2_bt_nodes

#endif  // HUMANOID_NAV2_BT_NODES__POSE_ANGLE_NODES_HPP_
