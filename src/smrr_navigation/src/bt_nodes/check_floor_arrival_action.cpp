#include "smrr_navigation/bt_nodes/check_floor_arrival_action.hpp"

namespace smrr_navigation
{

void CheckFloorArrivalAction::on_tick()
{
  std::string target_floor;
  if (!getInput("target_floor", target_floor)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[CheckFloorArrival] Missing required input port 'target_floor'");
    return;
  }
  goal_.target_floor = target_floor;
  RCLCPP_INFO(
    node_->get_logger(),
    "[CheckFloorArrival] Sending goal – target_floor: %s",
    target_floor.c_str());
}

BT::NodeStatus CheckFloorArrivalAction::on_success()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "[CheckFloorArrival] Action SUCCEEDED – floor reached.");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckFloorArrivalAction::on_aborted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[CheckFloorArrival] Action ABORTED by server.");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckFloorArrivalAction::on_cancelled()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[CheckFloorArrival] Action CANCELLED.");
  return BT::NodeStatus::FAILURE;
}

}  // namespace smrr_navigation
