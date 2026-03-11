#ifndef SMRR_NAVIGATION__BT_NODES__CHECK_FLOOR_ARRIVAL_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__CHECK_FLOOR_ARRIVAL_ACTION_HPP_

#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "smrr_interfaces/action/check_floor_arrival.hpp"

namespace smrr_navigation
{

class CheckFloorArrivalAction
  : public nav2_behavior_tree::BtActionNode<smrr_interfaces::action::CheckFloorArrival>
{
public:
  CheckFloorArrivalAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<smrr_interfaces::action::CheckFloorArrival>(
      xml_tag_name, action_name, conf)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("target_floor", "Target floor id, e.g. '3' or 'G'"),
    });
  }

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__CHECK_FLOOR_ARRIVAL_ACTION_HPP_
