#ifndef SMRR_NAVIGATION__BT_NODES__IS_SAME_FLOOR_CONDITION_HPP_
#define SMRR_NAVIGATION__BT_NODES__IS_SAME_FLOOR_CONDITION_HPP_

#include <string>
#include "behaviortree_cpp_v3/condition_node.h"

namespace smrr_navigation
{

/**
 * @brief BT Condition node that checks if current and target floors are the same.
 * 
 * Returns SUCCESS if floors are equal, FAILURE otherwise.
 * Used to determine if navigation can proceed on same floor.
 */
class IsSameFloorCondition : public BT::ConditionNode
{
public:
  IsSameFloorCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("current_floor", "Current floor ID"),
      BT::InputPort<std::string>("target_floor", "Target floor ID")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__IS_SAME_FLOOR_CONDITION_HPP_
