#ifndef SMRR_NAVIGATION__BT_NODES__SWITCH_MAP_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__SWITCH_MAP_ACTION_HPP_

#include <string>
#include <memory>
#include <chrono>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/load_map.hpp"

namespace smrr_navigation
{

/**
 * @brief BT StatefulActionNode that calls map_server to load a new map
 * 
 * Calls nav2_msgs/srv/LoadMap service to switch the active map.
 * Uses async pattern with RUNNING status to avoid blocking executor.
 */
class SwitchMapAction : public BT::StatefulActionNode
{
public:
  SwitchMapAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("map_yaml", "Full path to map YAML file"),
      BT::InputPort<std::string>("service_name", "/map_server/load_map", "LoadMap service name"),
      BT::InputPort<int>("timeout_ms", 10000, "Service call timeout in milliseconds (default 10s for map loading)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future_;
  std::chrono::steady_clock::time_point request_start_time_;
  bool request_sent_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__SWITCH_MAP_ACTION_HPP_
