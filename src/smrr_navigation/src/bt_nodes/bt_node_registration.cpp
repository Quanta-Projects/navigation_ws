#include "behaviortree_cpp_v3/bt_factory.h"
#include "smrr_navigation/bt_nodes/is_same_floor_condition.hpp"
#include "smrr_navigation/bt_nodes/is_different_floor_condition.hpp"
#include "smrr_navigation/bt_nodes/get_named_pose_action.hpp"
#include "smrr_navigation/bt_nodes/get_named_map_action.hpp"
#include "smrr_navigation/bt_nodes/switch_map_action.hpp"
#include "smrr_navigation/bt_nodes/publish_initial_pose_action.hpp"
#include "smrr_navigation/bt_nodes/call_elevator_action.hpp"
#include "smrr_navigation/bt_nodes/wait_for_door_open_action.hpp"
#include "smrr_navigation/bt_nodes/wait_for_door_open_depth_action.hpp"
#include "smrr_navigation/bt_nodes/wait_for_door_open_model_action.hpp"
#include "smrr_navigation/bt_nodes/update_pose_timestamp_action.hpp"

// Register all custom BT nodes for BehaviorTree.CPP
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<smrr_navigation::IsSameFloorCondition>("IsSameFloor");
  factory.registerNodeType<smrr_navigation::IsDifferentFloorCondition>("IsDifferentFloor");
  factory.registerNodeType<smrr_navigation::GetNamedPoseAction>("GetNamedPose");
  factory.registerNodeType<smrr_navigation::GetNamedMapAction>("GetNamedMap");
  factory.registerNodeType<smrr_navigation::SwitchMapAction>("SwitchMap");
  factory.registerNodeType<smrr_navigation::PublishInitialPoseAction>("PublishInitialPose");
  factory.registerNodeType<smrr_navigation::CallElevatorAction>("CallElevator");
  factory.registerNodeType<smrr_navigation::WaitForDoorOpenAction>("WaitForDoorOpen");
  factory.registerNodeType<smrr_navigation::WaitForDoorOpenDepthAction>("WaitForDoorOpenDepth");
  factory.registerNodeType<smrr_navigation::WaitForDoorOpenModelAction>("WaitForDoorOpenModel");
  factory.registerNodeType<smrr_navigation::UpdatePoseTimestampAction>("UpdatePoseTimestamp");
}
