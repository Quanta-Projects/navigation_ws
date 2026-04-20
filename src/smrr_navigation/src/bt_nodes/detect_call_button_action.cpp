#include "smrr_navigation/bt_nodes/detect_call_button_action.hpp"

namespace smrr_navigation
{

void DetectCallButtonAction::on_tick()
{
  std::string current_floor;
  if (!getInput("current_floor", current_floor)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[DetectCallButton] Missing required input port 'current_floor'");
    return;
  }

  std::string target_floor;
  if (!getInput("target_floor", target_floor)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[DetectCallButton] Missing required input port 'target_floor'");
    return;
  }

  goal_.current_floor = current_floor;
  goal_.target_floor  = target_floor;

  // inside_pose is optional — server skips door-open check if frame_id is empty
  geometry_msgs::msg::PoseStamped inside_pose;
  if (getInput("inside_pose", inside_pose)) {
    goal_.inside_pose = inside_pose;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[DetectCallButton] Sending goal — current: %s, target: %s",
    current_floor.c_str(), target_floor.c_str());
}

BT::NodeStatus DetectCallButtonAction::on_success()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "[DetectCallButton] SUCCEEDED — button localised, waypoints published. %s",
    result_.result->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DetectCallButtonAction::on_aborted()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[DetectCallButton] ABORTED by server.");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DetectCallButtonAction::on_cancelled()
{
  RCLCPP_WARN(
    node_->get_logger(),
    "[DetectCallButton] CANCELLED.");
  return BT::NodeStatus::FAILURE;
}

}  // namespace smrr_navigation
