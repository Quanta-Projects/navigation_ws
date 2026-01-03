#include "smrr_navigation/bt_nodes/is_same_floor_condition.hpp"
#include <iostream>

namespace smrr_navigation
{

BT::NodeStatus IsSameFloorCondition::tick()
{
  // Read input ports
  auto current_floor_result = getInput<std::string>("current_floor");
  auto target_floor_result = getInput<std::string>("target_floor");

  // Check if both inputs are available
  if (!current_floor_result) {
    std::cerr << "[IsSameFloor] ERROR: current_floor input not available" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
  if (!target_floor_result) {
    std::cerr << "[IsSameFloor] ERROR: target_floor input not available" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::string current_floor = current_floor_result.value();
  std::string target_floor = target_floor_result.value();

  // Check if either is empty
  if (current_floor.empty()) {
    std::cerr << "[IsSameFloor] ERROR: current_floor is empty" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
  if (target_floor.empty()) {
    std::cerr << "[IsSameFloor] ERROR: target_floor is empty" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  // Compare floors
  if (current_floor == target_floor) {
    std::cout << "[IsSameFloor] SUCCESS: Both floors are '" << current_floor << "'" << std::endl;
    return BT::NodeStatus::SUCCESS;
  } else {
    std::cout << "[IsSameFloor] FAILURE: Current floor '" << current_floor 
              << "' != target floor '" << target_floor << "'" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace smrr_navigation
