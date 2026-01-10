#include "smrr_navigation/bt_nodes/publish_initial_pose_action.hpp"

namespace smrr_navigation
{

PublishInitialPoseAction::PublishInitialPoseAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  // Get node from blackboard (set by executor)
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("PublishInitialPoseAction"), "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }
}

BT::NodeStatus PublishInitialPoseAction::tick()
{
  auto initial_pose = getInput<geometry_msgs::msg::PoseStamped>("initial_pose");
  
  // Use value_or for optional inputs with defaults
  std::string topic_name = "/initialpose";
  std::string frame_id_override = "";
  
  auto topic_input = getInput<std::string>("topic_name");
  if (topic_input.has_value()) {
    topic_name = topic_input.value();
  }
  
  auto frame_input = getInput<std::string>("frame_id");
  if (frame_input.has_value()) {
    frame_id_override = frame_input.value();
  }

  if (!initial_pose) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PublishInitialPoseAction"),
      "Missing required input: initial_pose");
    return BT::NodeStatus::FAILURE;
  }

  // Create publisher if not exists
  if (!initial_pose_pub_) {
    initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      topic_name, 10);
  }

  // Prepare PoseWithCovarianceStamped message
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = node_->get_clock()->now();
  
  // Use override frame_id if provided, otherwise use pose's frame_id
  if (!frame_id_override.empty()) {
    pose_msg.header.frame_id = frame_id_override;
  } else {
    pose_msg.header.frame_id = initial_pose.value().header.frame_id.empty() 
      ? "map" : initial_pose.value().header.frame_id;
  }

  pose_msg.pose.pose = initial_pose.value().pose;

  // Set reasonable default covariance (diagonal)
  // Position uncertainty: 0.25 m^2 (0.5m std dev)
  // Orientation uncertainty: 0.0685 rad^2 (~15 deg std dev)
  pose_msg.pose.covariance[0] = 0.25;   // x
  pose_msg.pose.covariance[7] = 0.25;   // y
  pose_msg.pose.covariance[14] = 0.0;   // z (not used in 2D)
  pose_msg.pose.covariance[21] = 0.0;   // roll (not used in 2D)
  pose_msg.pose.covariance[28] = 0.0;   // pitch (not used in 2D)
  pose_msg.pose.covariance[35] = 0.0685; // yaw

  RCLCPP_INFO(
    rclcpp::get_logger("PublishInitialPoseAction"),
    "Publishing initial pose: frame='%s', x=%.3f, y=%.3f, yaw=%.3f rad",
    pose_msg.header.frame_id.c_str(),
    pose_msg.pose.pose.position.x,
    pose_msg.pose.pose.position.y,
    2.0 * atan2(pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w));

  // Publish
  initial_pose_pub_->publish(pose_msg);

  RCLCPP_INFO(
    rclcpp::get_logger("PublishInitialPoseAction"),
    "Initial pose published successfully on topic: %s", topic_name.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace smrr_navigation
