#ifndef SMRR_NAVIGATION__BT_NODES__DETECT_CALL_BUTTON_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__DETECT_CALL_BUTTON_ACTION_HPP_

#include <string>
#include <memory>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "smrr_interfaces/action/detect_call_button.hpp"

namespace smrr_navigation
{

class DetectCallButtonAction
  : public nav2_behavior_tree::BtActionNode<
      smrr_interfaces::action::DetectCallButton>
{
public:
  DetectCallButtonAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<smrr_interfaces::action::DetectCallButton>(
      xml_tag_name, action_name, conf)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>(
        "current_floor", "Current floor id, e.g. 'floor0'"),
      BT::InputPort<std::string>(
        "target_floor",  "Target floor id, e.g. 'floor2'"),
    });
  }

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__DETECT_CALL_BUTTON_ACTION_HPP_
