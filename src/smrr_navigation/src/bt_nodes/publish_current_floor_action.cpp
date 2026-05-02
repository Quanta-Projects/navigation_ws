#include "smrr_navigation/bt_nodes/publish_current_floor_action.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace smrr_navigation
{

PublishCurrentFloorAction::PublishCurrentFloorAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PublishCurrentFloorAction"),
      "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }

  // Use a latched (transient-local) QoS so subscribers that come up after
  // this publish (e.g. after a restart) still receive the last floor value.
  rclcpp::QoS qos(1);
  qos.transient_local();
  pub_ = node_->create_publisher<std_msgs::msg::String>("/current_floor_id", qos);
}

BT::NodeStatus PublishCurrentFloorAction::tick()
{
  std::string target_floor_id;
  if (!config().blackboard->get("target_floor_id", target_floor_id)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "PublishCurrentFloor: 'target_floor_id' not found on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  std_msgs::msg::String msg;
  msg.data = target_floor_id;
  pub_->publish(msg);

  RCLCPP_INFO(
    node_->get_logger(),
    "[PublishCurrentFloor] Published current floor: %s",
    target_floor_id.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace smrr_navigation
