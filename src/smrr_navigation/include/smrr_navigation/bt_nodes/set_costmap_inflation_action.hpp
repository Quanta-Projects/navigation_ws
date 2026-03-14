#ifndef SMRR_NAVIGATION__BT_NODES__SET_COSTMAP_INFLATION_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__SET_COSTMAP_INFLATION_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that updates inflation radius for both local and
 *        global Nav2 costmaps at runtime via set_parameters services.
 *
 * Typical use in elevator missions:
 *   <SetCostmapInflation inflation_radius="0.325"/>  <!-- before entering -->
 *   <SetCostmapInflation inflation_radius="0.5"/>    <!-- after exiting -->
 */
class SetCostmapInflationAction : public BT::SyncActionNode
{
public:
  SetCostmapInflationAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("inflation_radius", "Costmap inflation radius (m)"),
      BT::InputPort<std::string>(
        "local_costmap_node", "/local_costmap/local_costmap",
        "Node name for local costmap parameter service"),
      BT::InputPort<std::string>(
        "global_costmap_node", "/global_costmap/global_costmap",
        "Node name for global costmap parameter service")
    };
  }

  BT::NodeStatus tick() override;

private:
  bool setInflationForNode(
    const std::string & node_name,
    double inflation_radius,
    const std::string & scope_label);

  rclcpp::Node::SharedPtr node_;

  // Dedicated helper node to avoid executor ownership conflicts when
  // SyncParametersClient spins internally.
  rclcpp::Node::SharedPtr helper_node_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__SET_COSTMAP_INFLATION_ACTION_HPP_
