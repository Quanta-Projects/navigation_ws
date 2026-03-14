#include "smrr_navigation/bt_nodes/check_elevator_direction_action.hpp"

namespace smrr_navigation
{

void CheckElevatorDirectionAction::on_tick()
{
  std::string current_floor;
  if (!getInput("current_floor", current_floor)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[CheckElevatorDirection] Missing required input port 'current_floor'");
    return;
  }

  std::string target_floor;
  if (!getInput("target_floor", target_floor)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[CheckElevatorDirection] Missing required input port 'target_floor'");
    return;
  }

  goal_.current_floor = current_floor;
  goal_.target_floor  = target_floor;

  RCLCPP_INFO(
    node_->get_logger(),
    "[CheckElevatorDirection] Sending goal – current_floor: %s, target_floor: %s",
    current_floor.c_str(), target_floor.c_str());
}

BT::NodeStatus CheckElevatorDirectionAction::on_success()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "[CheckElevatorDirection] Action SUCCEEDED – correct elevator direction confirmed.");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckElevatorDirectionAction::on_aborted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[CheckElevatorDirection] Action ABORTED by server.");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckElevatorDirectionAction::on_cancelled()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[CheckElevatorDirection] Action CANCELLED.");
  return BT::NodeStatus::FAILURE;
}

}  // namespace smrr_navigation
