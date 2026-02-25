#ifndef SMRR_NAVIGATION__BT_NODES__SET_CONTROLLER_PARAMS_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__SET_CONTROLLER_PARAMS_ACTION_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that updates DWB local planner velocity/acceleration
 *        limits on the fly by calling /controller_server/set_parameters.
 *
 * Use this node to temporarily boost velocity and acceleration while the robot
 * crosses the physical gap between the floor and an elevator car, then reset
 * to normal values once inside.
 *
 * Usage example:
 *   <SetControllerParams max_vel_x="0.6" acc_lim_x="4.0"/>
 *   <NavigateToPose goal="{inside_pose}"/>
 *   <SetControllerParams max_vel_x="0.35" acc_lim_x="2.5"/>
 */
class SetControllerParamsAction : public BT::SyncActionNode
{
public:
  SetControllerParamsAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_vel_x", "Maximum linear velocity in x (m/s)"),
      BT::InputPort<double>("acc_lim_x", "Maximum linear acceleration in x (m/s^2)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  // Dedicated node used exclusively by SyncParametersClient.
  // SyncParametersClient::set_parameters() internally calls
  // rclcpp::spin_until_future_complete(node, ...) which tries to add the node
  // to a temporary executor.  If we passed node_ (already owned by the BT
  // mission executor) that would throw "already added to executor".  Using a
  // separate helper node that is never added to any other executor avoids
  // the conflict entirely.
  rclcpp::Node::SharedPtr helper_node_;
  std::shared_ptr<rclcpp::SyncParametersClient> params_client_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__SET_CONTROLLER_PARAMS_ACTION_HPP_
