#ifndef SMRR_NAVIGATION__BT_NODES__CLEAR_COSTMAPS_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__CLEAR_COSTMAPS_ACTION_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

namespace smrr_navigation
{

class ClearCostmapsAction : public BT::SyncActionNode
{
public:
  ClearCostmapsAction(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_clear_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_clear_client_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__CLEAR_COSTMAPS_ACTION_HPP_
