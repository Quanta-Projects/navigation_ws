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
#include "smrr_navigation/bt_nodes/stop_robot_action.hpp"
#include "smrr_navigation/bt_nodes/set_controller_params_action.hpp"
#include "smrr_navigation/bt_nodes/set_amcl_params_action.hpp"
#include "smrr_navigation/bt_nodes/check_floor_arrival_action.hpp"
#include "smrr_navigation/bt_nodes/toggle_apriltag_action.hpp"
#include "smrr_navigation/bt_nodes/build_pose_vector_action.hpp"

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
  factory.registerNodeType<smrr_navigation::StopRobotAction>("StopRobot");
  factory.registerNodeType<smrr_navigation::SetControllerParamsAction>("SetControllerParams");
  factory.registerNodeType<smrr_navigation::SetAMCLParamsAction>("SetAMCLParams");
  factory.registerNodeType<smrr_navigation::ToggleAprilTagAction>("ToggleAprilTag");
  factory.registerNodeType<smrr_navigation::BuildPoseVectorAction>("BuildPoseVector");

  // BtActionNode-derived nodes need a builder (3-arg constructor)
  BT::NodeBuilder check_floor_builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<smrr_navigation::CheckFloorArrivalAction>(
        name, "check_floor_arrival", config);
    };
  factory.registerBuilder<smrr_navigation::CheckFloorArrivalAction>(
    "CheckFloorArrival", check_floor_builder);
}
