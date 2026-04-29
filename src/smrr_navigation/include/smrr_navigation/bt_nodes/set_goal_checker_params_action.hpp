#ifndef SMRR_NAVIGATION__BT_NODES__SET_GOAL_CHECKER_PARAMS_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__SET_GOAL_CHECKER_PARAMS_ACTION_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that dynamically updates the controller_server's
 *        goal checker tolerances via /controller_server/set_parameters.
 *
 * Useful for loosening tolerances when entering/exiting the elevator where
 * precise yaw alignment is not required.
 *
 * Usage example:
 *   <SetGoalCheckerParams xy_goal_tolerance="0.30" yaw_goal_tolerance="0.50"/>
 *   <NavigateThroughPoses .../>
 *   <SetGoalCheckerParams xy_goal_tolerance="0.15" yaw_goal_tolerance="0.20"/>
 */
class SetGoalCheckerParamsAction : public BT::SyncActionNode
{
public:
  SetGoalCheckerParamsAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "xy_goal_tolerance",
        "XY distance tolerance (metres) for goal checker"),
      BT::InputPort<double>(
        "yaw_goal_tolerance",
        "Yaw angular tolerance (radians) for goal checker")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr helper_node_;
  std::shared_ptr<rclcpp::SyncParametersClient> params_client_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__SET_GOAL_CHECKER_PARAMS_ACTION_HPP_
