#include "smrr_navigation/bt_nodes/update_pose_timestamp_action.hpp"

namespace smrr_navigation
{

UpdatePoseTimestampAction::UpdatePoseTimestampAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  // Get node from blackboard (set by executor)
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("UpdatePoseTimestampAction"), "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }
}

BT::NodeStatus UpdatePoseTimestampAction::tick()
{
  auto input_pose = getInput<geometry_msgs::msg::PoseStamped>("input_pose");
  
  if (!input_pose) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[UpdatePoseTimestampAction] Missing required input: input_pose");
    return BT::NodeStatus::FAILURE;
  }

  // Create output pose with updated timestamp
  geometry_msgs::msg::PoseStamped output_pose = input_pose.value();
  output_pose.header.stamp = node_->get_clock()->now();

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[UpdatePoseTimestampAction] Updated pose timestamp: frame='%s', x=%.3f, y=%.3f",
    output_pose.header.frame_id.c_str(),
    output_pose.pose.position.x,
    output_pose.pose.position.y);

  setOutput("output_pose", output_pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace smrr_navigation
