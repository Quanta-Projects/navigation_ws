#include "smrr_navigation/bt_nodes/stop_robot_action.hpp"

namespace smrr_navigation
{

StopRobotAction::StopRobotAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf)
{
  // Get node from blackboard (set by executor)
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("StopRobotAction"), "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }
}

void StopRobotAction::onHalted()
{
  // Publish one final zero Twist to ensure robot stops immediately
  if (cmd_vel_publisher_) {
    publishZeroTwist();
    RCLCPP_DEBUG(node_->get_logger(), "StopRobot halted - published final zero Twist");
  }
}

BT::NodeStatus StopRobotAction::onStart()
{
  // Get input parameters
  auto topic_result = getInput<std::string>("topic");
  auto repeat_ms_result = getInput<int>("repeat_ms");
  auto duration_ms_result = getInput<int>("duration_ms");

  if (!topic_result || !repeat_ms_result || !duration_ms_result) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "StopRobot: Missing required inputs");
    return BT::NodeStatus::FAILURE;
  }

  topic_name_ = topic_result.value();
  int repeat_ms = repeat_ms_result.value();
  int duration_ms = duration_ms_result.value();

  // Validate inputs
  if (topic_name_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "StopRobot: topic cannot be empty");
    return BT::NodeStatus::FAILURE;
  }

  if (repeat_ms <= 0) {
    repeat_ms = 200;  // Default fallback
    RCLCPP_WARN(
      node_->get_logger(),
      "StopRobot: invalid repeat_ms, using default 200ms");
  }

  if (duration_ms <= 0) {
    // If duration is zero or negative, publish once and return SUCCESS
    RCLCPP_INFO(
      node_->get_logger(),
      "StopRobot: duration_ms <= 0, publishing single zero Twist and returning SUCCESS");
    
    if (!cmd_vel_publisher_) {
      cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        topic_name_, rclcpp::QoS(10));
    }
    
    publishZeroTwist();
    return BT::NodeStatus::SUCCESS;
  }

  repeat_interval_ = std::chrono::milliseconds(repeat_ms);
  total_duration_ = std::chrono::milliseconds(duration_ms);

  // Create publisher if not exists
  if (!cmd_vel_publisher_) {
    cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      topic_name_, rclcpp::QoS(10));
    
    if (!cmd_vel_publisher_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "StopRobot: Failed to create publisher on topic '%s'",
        topic_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "StopRobot: Publishing zero Twist on '%s' every %dms for %dms",
    topic_name_.c_str(), repeat_ms, duration_ms);

  // Initialize timing
  start_time_ = std::chrono::steady_clock::now();
  last_publish_time_ = start_time_;

  // Publish first zero Twist immediately
  publishZeroTwist();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StopRobotAction::onRunning()
{
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_);

  // Check if total duration has elapsed
  if (elapsed >= total_duration_) {
    // Publish one final zero Twist before completing
    publishZeroTwist();
    
    RCLCPP_INFO(
      node_->get_logger(),
      "StopRobot: Completed after %ld ms",
      elapsed.count());
    
    return BT::NodeStatus::SUCCESS;
  }

  // Check if it's time to publish again
  auto time_since_last_publish = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_publish_time_);

  if (time_since_last_publish >= repeat_interval_) {
    publishZeroTwist();
    last_publish_time_ = now;
  }

  return BT::NodeStatus::RUNNING;
}

void StopRobotAction::publishZeroTwist()
{
  if (!cmd_vel_publisher_) {
    RCLCPP_ERROR(node_->get_logger(), "StopRobot: Publisher not initialized");
    return;
  }

  geometry_msgs::msg::Twist zero_twist;
  zero_twist.linear.x = 0.0;
  zero_twist.linear.y = 0.0;
  zero_twist.linear.z = 0.0;
  zero_twist.angular.x = 0.0;
  zero_twist.angular.y = 0.0;
  zero_twist.angular.z = 0.0;

  cmd_vel_publisher_->publish(zero_twist);
  
  RCLCPP_DEBUG(
    node_->get_logger(),
    "StopRobot: Published zero Twist on '%s'",
    topic_name_.c_str());
}

}  // namespace smrr_navigation
