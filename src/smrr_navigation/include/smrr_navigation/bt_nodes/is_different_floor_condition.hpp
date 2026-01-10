#ifndef SMRR_NAVIGATION__BT_NODES__IS_DIFFERENT_FLOOR_CONDITION_HPP_
#define SMRR_NAVIGATION__BT_NODES__IS_DIFFERENT_FLOOR_CONDITION_HPP_

#include <string>
#include "behaviortree_cpp_v3/condition_node.h"

namespace smrr_navigation
{

/**
 * @brief BT Condition node that checks if current floor differs from target floor.
 * 
 * Returns SUCCESS if floors are different (cross-floor navigation required).
 * Returns FAILURE if floors are the same or if inputs are invalid.
 */
class IsDifferentFloorCondition : public BT::ConditionNode
{
public:
  IsDifferentFloorCondition(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("current_floor", "Current floor identifier"),
      BT::InputPort<std::string>("target_floor", "Target floor identifier")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__IS_DIFFERENT_FLOOR_CONDITION_HPP_
