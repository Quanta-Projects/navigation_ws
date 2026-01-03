#include "behaviortree_cpp_v3/bt_factory.h"
#include "smrr_navigation/bt_nodes/is_same_floor_condition.hpp"
#include "smrr_navigation/bt_nodes/is_different_floor_condition.hpp"
#include "smrr_navigation/bt_nodes/get_named_pose_action.hpp"

// Register all custom BT nodes for BehaviorTree.CPP
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<smrr_navigation::IsSameFloorCondition>("IsSameFloor");
  factory.registerNodeType<smrr_navigation::IsDifferentFloorCondition>("IsDifferentFloor");
  factory.registerNodeType<smrr_navigation::GetNamedPoseAction>("GetNamedPose");
}
