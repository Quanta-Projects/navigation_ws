#ifndef SMRR_NAVIGATION__BT_NODES__STOP_ROBOT_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__STOP_ROBOT_ACTION_HPP_

#include <string>
#include <memory>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace smrr_navigation
{

/**
 * @brief BT StatefulActionNode that publishes zero Twist commands to stop robot motion.
 * 
 * Publishes zero velocity (linear.x=0, angular.z=0) repeatedly to ensure robot stops completely.
 * Useful after Spin or other motion commands to prevent residual rotation/movement.
 * 
 * Usage example:
 *   <StopRobot topic="/cmd_vel" repeat_ms="200" duration_ms="800"/>
 *   
 * This publishes zero velocity every 200ms for a total of 800ms (4 messages).
 */
class StopRobotAction : public BT::StatefulActionNode
{
public:
  StopRobotAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/cmd_vel", "Velocity command topic"),
      BT::InputPort<int>("repeat_ms", 200, "Interval between zero Twist publishes (milliseconds)"),
      BT::InputPort<int>("duration_ms", 800, "Total duration to publish zero Twist (milliseconds)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_publish_time_;
  
  std::chrono::milliseconds repeat_interval_;
  std::chrono::milliseconds total_duration_;
  
  std::string topic_name_;
  
  /**
   * @brief Publish a zero Twist message
   */
  void publishZeroTwist();
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__STOP_ROBOT_ACTION_HPP_
