#ifndef SMRR_NAVIGATION__BT_NODES__SET_AMCL_PARAMS_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__SET_AMCL_PARAMS_ACTION_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that dynamically updates AMCL's motion-model
 *        thresholds via /amcl/set_parameters.
 *
 * Setting `update_min_d` and `update_min_a` to very large values (e.g. 1e9)
 * effectively pauses AMCL particle-filter updates, preventing the symmetric
 * elevator walls from corrupting the pose estimate during an in-elevator
 * 180-degree turn.  Calling the node again with the normal values (e.g.
 * 0.2 m / 0.5 rad) re-enables updates once the robot is on the target floor.
 *
 * Usage example:
 *   <SetAMCLParams update_min_d="1e9" update_min_a="1e9"/>   <!-- freeze -->
 *   <Spin spin_dist="-3.5416" .../>
 *   <SetAMCLParams update_min_d="0.2" update_min_a="0.5"/>   <!-- restore -->
 *
 * NOTE: uses the same helper_node_ pattern as SetControllerParamsAction to
 * avoid the "Node already added to executor" crash when SyncParametersClient
 * internally calls rclcpp::spin_until_future_complete.
 */
class SetAMCLParamsAction : public BT::SyncActionNode
{
public:
  SetAMCLParamsAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "update_min_d",
        "Translational movement (metres) required before performing a filter update"),
      BT::InputPort<double>(
        "update_min_a",
        "Rotational movement (radians) required before performing a filter update")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  // Dedicated helper node passed to SyncParametersClient.
  // SyncParametersClient::set_parameters() spins this node internally; keeping
  // it separate from node_ (which is owned by the BT executor) avoids the
  // "already added to executor" runtime error.
  rclcpp::Node::SharedPtr helper_node_;
  std::shared_ptr<rclcpp::SyncParametersClient> params_client_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__SET_AMCL_PARAMS_ACTION_HPP_
