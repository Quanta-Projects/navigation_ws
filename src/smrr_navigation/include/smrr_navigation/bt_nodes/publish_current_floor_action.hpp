#ifndef SMRR_NAVIGATION__BT_NODES__PUBLISH_CURRENT_FLOOR_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__PUBLISH_CURRENT_FLOOR_ACTION_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that publishes the current floor ID to /current_floor_id.
 *
 * Place this node in the BT right after the robot exits the elevator so that
 * named_goal_server (and any other subscriber) immediately knows the robot is
 * on the target floor — without waiting for the full mission to succeed.
 *
 * The floor ID is read from the blackboard key "target_floor_id" which is set
 * by smrr_bt_mission_executor before the tree is ticked.
 *
 * Usage in BT XML:
 *   <PublishCurrentFloor/>
 */
class PublishCurrentFloorAction : public BT::SyncActionNode
{
public:
  PublishCurrentFloorAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__PUBLISH_CURRENT_FLOOR_ACTION_HPP_
