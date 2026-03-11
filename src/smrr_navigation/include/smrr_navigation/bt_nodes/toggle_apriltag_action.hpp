#ifndef SMRR_NAVIGATION__BT_NODES__TOGGLE_APRILTAG_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__TOGGLE_APRILTAG_ACTION_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that calls /toggle_apriltag (std_srvs/SetBool)
 *        to dynamically start or stop the apriltag_ros node, saving CPU
 *        when AprilTag detection is not needed.
 *
 * A standalone helper_node_ is created (never added to any executor) so that
 * rclcpp::spin_until_future_complete() inside the synchronous service call
 * does not conflict with the BT mission executor that owns the main node.
 *
 * Usage examples:
 *   <ToggleAprilTag turn_on="true"/>   <!-- start apriltag_node  -->
 *   <Spin spin_dist="-3.1416" .../>
 *   <ToggleAprilTag turn_on="false"/>  <!-- stop  apriltag_node  -->
 */
class ToggleAprilTagAction : public BT::SyncActionNode
{
public:
  ToggleAprilTagAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("turn_on", "true = start AprilTag node, false = stop it")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  // Dedicated lightweight node used exclusively for the service client.
  // Using the main BT node would throw "already added to executor".
  rclcpp::Node::SharedPtr helper_node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__TOGGLE_APRILTAG_ACTION_HPP_
