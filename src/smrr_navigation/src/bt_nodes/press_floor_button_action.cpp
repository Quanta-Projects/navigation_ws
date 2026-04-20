#include "smrr_navigation/bt_nodes/press_floor_button_action.hpp"

namespace smrr_navigation
{

void PressFloorButtonAction::on_tick()
{
  std::string target_floor;
  if (!getInput("target_floor", target_floor)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[PressFloorButton] Missing required input port 'target_floor'");
    return;
  }

  goal_.target_floor = target_floor;

  RCLCPP_INFO(
    node_->get_logger(),
    "[PressFloorButton] Sending goal — target_floor: %s",
    target_floor.c_str());
}

BT::NodeStatus PressFloorButtonAction::on_success()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "[PressFloorButton] SUCCEEDED — floor button pressed and lit. %s",
    result_.result->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PressFloorButtonAction::on_aborted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[PressFloorButton] ABORTED by server.");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PressFloorButtonAction::on_cancelled()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[PressFloorButton] CANCELLED.");
  return BT::NodeStatus::FAILURE;
}

}  // namespace smrr_navigation
