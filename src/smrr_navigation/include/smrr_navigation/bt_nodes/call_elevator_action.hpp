#ifndef SMRR_NAVIGATION__BT_NODES__CALL_ELEVATOR_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__CALL_ELEVATOR_ACTION_HPP_

#include <string>
#include <memory>
#include <unordered_map>
#include <array>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that calls elevator to a specified floor via Gazebo CLI.
 * 
 * Uses subprocess execution to publish to Gazebo elevator topic.
 * Command: gz_cli topic -p gz_elevator_topic -m 'data: "<floor_num>"'
 */
class CallElevatorAction : public BT::SyncActionNode
{
public:
  CallElevatorAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor_id", "Floor to call elevator to (e.g., 'floor0')"),
      BT::InputPort<std::string>("gz_cli", "gz-11.14.0", "Gazebo CLI executable name"),
      BT::InputPort<std::string>("gz_elevator_topic", "/gazebo/default/elevator", "Gazebo elevator topic"),
      BT::InputPort<double>("timeout_sec", 5.0, "Command execution timeout in seconds")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  
  // Floor ID to floor number mapping
  std::unordered_map<std::string, std::string> floor_map_;
  
  /**
   * @brief Execute a shell command and capture output.
   * @param cmd Command to execute
   * @param timeout_sec Timeout in seconds
   * @param stdout_output Output string for stdout
   * @param stderr_output Output string for stderr
   * @return Exit code of the command (-1 on error/timeout)
   */
  int executeCommand(
    const std::string & cmd,
    double timeout_sec,
    std::string & stdout_output,
    std::string & stderr_output);
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__CALL_ELEVATOR_ACTION_HPP_
