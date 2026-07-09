#include "humanoid_nav2_bt_nodes/pose_angle_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "angles/angles.h"
#if __has_include("behaviortree_cpp/bt_factory.h")
#include "behaviortree_cpp/bt_factory.h"
#elif __has_include("behaviortree_cpp_v3/bt_factory.h")
#include "behaviortree_cpp_v3/bt_factory.h"
#else
#error "BehaviorTree.CPP bt_factory.h not found"
#endif
#include "builtin_interfaces/msg/duration.hpp"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace humanoid_nav2_bt_nodes
{
namespace
{

constexpr double kReadyPollSeconds = 0.0;
constexpr double kPi = 3.14159265358979323846;
constexpr auto kMinActionResponseTimeout = std::chrono::milliseconds(2000);

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

double normalizeRelativeYaw(double yaw)
{
  return angles::normalize_angle(yaw);
}

builtin_interfaces::msg::Duration secondsToDurationMsg(double seconds)
{
  seconds = std::max(0.0, seconds);
  builtin_interfaces::msg::Duration msg;
  msg.sec = static_cast<int32_t>(std::floor(seconds));
  msg.nanosec = static_cast<uint32_t>(
    std::round((seconds - static_cast<double>(msg.sec)) * 1.0e9));
  if (msg.nanosec >= 1000000000u) {
    msg.sec += 1;
    msg.nanosec -= 1000000000u;
  }
  return msg;
}

template<typename FutureT>
bool isFutureReady(const FutureT & future)
{
  return future.valid() &&
         future.wait_for(std::chrono::duration<double>(kReadyPollSeconds)) ==
         std::future_status::ready;
}

}  // namespace

MakePoseTowardGoal::MakePoseTowardGoal(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
}

BT::NodeStatus MakePoseTowardGoal::tick()
{
  geometry_msgs::msg::PoseStamped goal;
  std::string configured_global_frame;
  std::string robot_base_frame;
  double min_heading_distance;
  std::string heading_mode;
  double transform_tolerance;

  if (!getInputData(
      goal, configured_global_frame, robot_base_frame, min_heading_distance, heading_mode,
      transform_tolerance))
  {
    return BT::NodeStatus::FAILURE;
  }

  try {
    const auto global_frame = resolveGlobalFrame(goal, configured_global_frame);
    const auto transformed_goal = transformGoal(goal, global_frame, transform_tolerance);
    const auto robot_pose = getRobotPose(global_frame, robot_base_frame, transform_tolerance);

    const double dx = transformed_goal.pose.position.x - robot_pose.pose.position.x;
    const double dy = transformed_goal.pose.position.y - robot_pose.pose.position.y;
    const double distance = std::hypot(dx, dy);

    auto output_goal = transformed_goal;
    output_goal.header.stamp = node_->now();

    if (distance > min_heading_distance) {
      double heading = std::atan2(dy, dx);
      if (heading_mode == "away") {
        heading = normalizeRelativeYaw(heading + kPi);
      } else if (heading_mode != "toward") {
        throw std::runtime_error(
                "MakePoseTowardGoal heading_mode must be 'toward' or 'away', got '" +
                heading_mode + "'");
      }
      output_goal.pose.orientation = yawToQuaternion(heading);
    } else {
      output_goal.pose.orientation = robot_pose.pose.orientation;
    }

    setOutput("output_goal", output_goal);
    return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "MakePoseTowardGoal failed to get transform: %s", ex.what());
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "MakePoseTowardGoal failed: %s", ex.what());
  }

  return BT::NodeStatus::FAILURE;
}

bool MakePoseTowardGoal::getInputData(
  geometry_msgs::msg::PoseStamped & goal,
  std::string & global_frame,
  std::string & robot_base_frame,
  double & min_heading_distance,
  std::string & heading_mode,
  double & transform_tolerance)
{
  if (!getInput("goal", goal)) {
    RCLCPP_ERROR(node_->get_logger(), "MakePoseTowardGoal requires input port [goal]");
    return false;
  }

  getInput("global_frame", global_frame);
  getInput("robot_base_frame", robot_base_frame);
  getInput("min_heading_distance", min_heading_distance);
  getInput("heading_mode", heading_mode);
  getInput("transform_tolerance", transform_tolerance);

  if (robot_base_frame.empty()) {
    robot_base_frame = "base_footprint";
  }
  if (heading_mode.empty()) {
    heading_mode = "toward";
  }

  min_heading_distance = std::max(0.0, min_heading_distance);
  transform_tolerance = std::max(0.0, transform_tolerance);
  return true;
}

geometry_msgs::msg::PoseStamped MakePoseTowardGoal::transformGoal(
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & global_frame,
  double transform_tolerance) const
{
  if (goal.header.frame_id.empty() || goal.header.frame_id == global_frame) {
    auto transformed_goal = goal;
    transformed_goal.header.frame_id = global_frame;
    return transformed_goal;
  }

  return tf_buffer_->transform(
    goal, global_frame, tf2::durationFromSec(transform_tolerance));
}

geometry_msgs::msg::PoseStamped MakePoseTowardGoal::getRobotPose(
  const std::string & global_frame,
  const std::string & robot_base_frame,
  double transform_tolerance) const
{
  const auto transform = tf_buffer_->lookupTransform(
    global_frame, robot_base_frame, tf2::TimePointZero,
    tf2::durationFromSec(transform_tolerance));

  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

std::string MakePoseTowardGoal::resolveGlobalFrame(
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & configured_frame) const
{
  if (!configured_frame.empty()) {
    return configured_frame;
  }
  if (!goal.header.frame_id.empty()) {
    return goal.header.frame_id;
  }
  return "map_ground";
}

SpinToPose::SpinToPose(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  std::string action_name = "spin";
  getInput("server_name", action_name);
  if (action_name.empty()) {
    action_name = "spin";
  }

  action_client_ = rclcpp_action::create_client<Action>(node_, action_name, callback_group_);

  auto wait_for_service_timeout =
    config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");
  if (!action_client_->wait_for_action_server(wait_for_service_timeout)) {
    throw std::runtime_error("Spin action server is not available: " + action_name);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
}

BT::NodeStatus SpinToPose::onStart()
{
  resetActionState();
  attempts_ = 0;

  if (!readRuntimeInputs()) {
    return BT::NodeStatus::FAILURE;
  }

  return sendSpinIfNeeded();
}

BT::NodeStatus SpinToPose::onRunning()
{
  callback_group_executor_.spin_some();

  if (phase_ == Phase::WAITING_FOR_GOAL_HANDLE) {
    if (isFutureReady(future_goal_handle_)) {
      goal_handle_ = future_goal_handle_.get();
      future_goal_handle_ = {};

      if (!goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "SpinToPose goal was rejected by behavior_server");
        setErrorCode(ResultTraits::UNKNOWN);
        resetActionState();
        return BT::NodeStatus::FAILURE;
      }

      future_result_ = action_client_->async_get_result(goal_handle_);
      phase_ = Phase::WAITING_FOR_RESULT;
      return BT::NodeStatus::RUNNING;
    }

    if (std::chrono::steady_clock::now() - goal_sent_steady_time_ > server_timeout_)
    {
      RCLCPP_WARN(node_->get_logger(), "SpinToPose timed out waiting for goal acceptance");
      setErrorCode(ResultTraits::TIMEOUT);
      resetActionState();
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::WAITING_FOR_RESULT) {
    if (!isFutureReady(future_result_)) {
      return BT::NodeStatus::RUNNING;
    }

    const auto result = future_result_.get();
    const auto error_code = ResultTraits::errorCode(result.result);
    setErrorCode(error_code);

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      error_code != ResultTraits::NONE)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "SpinToPose spin failed, result_code=%d, error_code=%u",
        static_cast<int>(result.code), error_code);
      resetActionState();
      return BT::NodeStatus::FAILURE;
    }

    resetActionState();

    if (!readRuntimeInputs()) {
      return BT::NodeStatus::FAILURE;
    }
    return sendSpinIfNeeded();
  }

  return sendSpinIfNeeded();
}

void SpinToPose::onHalted()
{
  if (goal_handle_) {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_);
  }
  resetActionState();
}

bool SpinToPose::readRuntimeInputs()
{
  if (!getInput("goal", goal_)) {
    RCLCPP_ERROR(node_->get_logger(), "SpinToPose requires input port [goal]");
    return false;
  }

  getInput("mode", mode_);
  getInput("global_frame", configured_global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("yaw_tolerance", yaw_tolerance_);
  getInput("position_tolerance", position_tolerance_);
  getInput("transform_tolerance", transform_tolerance_);
  getInput("time_allowance", time_allowance_);
  getInput("max_attempts", max_attempts_);

  auto blackboard_timeout =
    config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  server_timeout_ = std::max(blackboard_timeout, kMinActionResponseTimeout);
  getInput("server_timeout", server_timeout_);
  server_timeout_ = std::max(server_timeout_, kMinActionResponseTimeout);

  if (robot_base_frame_.empty()) {
    robot_base_frame_ = "base_footprint";
  }
  if (mode_.empty()) {
    mode_ = "goal_yaw";
  }

  yaw_tolerance_ = std::max(0.0, yaw_tolerance_);
  position_tolerance_ = std::max(0.0, position_tolerance_);
  transform_tolerance_ = std::max(0.0, transform_tolerance_);
  time_allowance_ = std::max(0.1, time_allowance_);
  max_attempts_ = std::max(1, max_attempts_);
  return true;
}

BT::NodeStatus SpinToPose::sendSpinIfNeeded()
{
  try {
    const auto check = computeAngleToTarget();
    if (check.aligned) {
      setErrorCode(ResultTraits::NONE);
      return BT::NodeStatus::SUCCESS;
    }

    if (attempts_ >= max_attempts_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "SpinToPose failed to converge after %d attempts", attempts_);
      setErrorCode(ResultTraits::TIMEOUT);
      return BT::NodeStatus::FAILURE;
    }

    Action::Goal goal;
    goal.target_yaw = static_cast<float>(check.angle);
    goal.time_allowance = secondsToDurationMsg(time_allowance_);

    future_goal_handle_ = action_client_->async_send_goal(goal);
    goal_sent_steady_time_ = std::chrono::steady_clock::now();
    phase_ = Phase::WAITING_FOR_GOAL_HANDLE;
    attempts_++;

    RCLCPP_INFO(
      node_->get_logger(),
      "SpinToPose attempt %d/%d, mode=%s, target_yaw=%.3f rad",
      attempts_, max_attempts_, mode_.c_str(), check.angle);

    return BT::NodeStatus::RUNNING;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "SpinToPose TF failure: %s", ex.what());
    setErrorCode(ResultTraits::TF_ERROR);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(node_->get_logger(), "SpinToPose failure: %s", ex.what());
    setErrorCode(ResultTraits::UNKNOWN);
  }

  resetActionState();
  return BT::NodeStatus::FAILURE;
}

SpinToPose::AngleCheck SpinToPose::computeAngleToTarget() const
{
  const auto global_frame = resolveGlobalFrame(goal_, configured_global_frame_);
  const auto transformed_goal = transformGoal(goal_, global_frame, transform_tolerance_);
  const auto robot_pose = getRobotPose(global_frame, robot_base_frame_, transform_tolerance_);

  const double current_yaw = tf2::getYaw(robot_pose.pose.orientation);
  double target_yaw = 0.0;

  if (mode_ == "goal_position" || mode_ == "goal_position_reverse") {
    const double dx = transformed_goal.pose.position.x - robot_pose.pose.position.x;
    const double dy = transformed_goal.pose.position.y - robot_pose.pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance <= position_tolerance_) {
      return {true, 0.0};
    }
    target_yaw = std::atan2(dy, dx);
    if (mode_ == "goal_position_reverse") {
      target_yaw = normalizeRelativeYaw(target_yaw + kPi);
    }
  } else if (mode_ == "goal_yaw") {
    target_yaw = tf2::getYaw(transformed_goal.pose.orientation);
  } else {
    throw std::runtime_error(
            "SpinToPose mode must be 'goal_position', 'goal_position_reverse', or 'goal_yaw', got '" +
            mode_ + "'");
  }

  const double angle = normalizeRelativeYaw(
    angles::shortest_angular_distance(current_yaw, target_yaw));
  RCLCPP_INFO(
    node_->get_logger(),
    "SpinToPose angle check mode=%s current_yaw=%.3f target_yaw=%.3f shortest=%.3f",
    mode_.c_str(), current_yaw, target_yaw, angle);
  return {std::abs(angle) <= yaw_tolerance_, angle};
}

geometry_msgs::msg::PoseStamped SpinToPose::transformGoal(
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & global_frame,
  double transform_tolerance) const
{
  if (goal.header.frame_id.empty() || goal.header.frame_id == global_frame) {
    auto transformed_goal = goal;
    transformed_goal.header.frame_id = global_frame;
    return transformed_goal;
  }

  return tf_buffer_->transform(
    goal, global_frame, tf2::durationFromSec(transform_tolerance));
}

geometry_msgs::msg::PoseStamped SpinToPose::getRobotPose(
  const std::string & global_frame,
  const std::string & robot_base_frame,
  double transform_tolerance) const
{
  const auto transform = tf_buffer_->lookupTransform(
    global_frame, robot_base_frame, tf2::TimePointZero,
    tf2::durationFromSec(transform_tolerance));

  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

std::string SpinToPose::resolveGlobalFrame(
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & configured_frame) const
{
  if (!configured_frame.empty()) {
    return configured_frame;
  }
  if (!goal.header.frame_id.empty()) {
    return goal.header.frame_id;
  }
  return "map_ground";
}

void SpinToPose::resetActionState()
{
  phase_ = Phase::IDLE;
  future_goal_handle_ = {};
  future_result_ = {};
  goal_handle_.reset();
}

void SpinToPose::setErrorCode(ErrorCode error_code)
{
  setOutput("error_code_id", error_code);
}

}  // namespace humanoid_nav2_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<humanoid_nav2_bt_nodes::MakePoseTowardGoal>("MakePoseTowardGoal");
  factory.registerNodeType<humanoid_nav2_bt_nodes::SpinToPose>("SpinToPose");
}
