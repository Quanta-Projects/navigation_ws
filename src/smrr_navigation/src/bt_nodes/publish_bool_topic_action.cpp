#include "smrr_navigation/bt_nodes/publish_bool_topic_action.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace smrr_navigation
{

PublishBoolTopicAction::PublishBoolTopicAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PublishBoolTopicAction"),
      "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }
}

BT::NodeStatus PublishBoolTopicAction::tick()
{
  std::string topic;
  if (!getInput<std::string>("topic", topic) || topic.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "PublishBoolTopic: missing 'topic' port");
    return BT::NodeStatus::FAILURE;
  }

  bool value = true;
  getInput<bool>("value", value);

  // Create a new publisher if topic changed or not yet created.
  if (!pub_ || topic_ != topic) {
    topic_ = topic;
    pub_ = node_->create_publisher<std_msgs::msg::Bool>(topic_, 10);
  }

  std_msgs::msg::Bool msg;
  msg.data = value;
  pub_->publish(msg);

  RCLCPP_INFO(
    node_->get_logger(),
    "[PublishBoolTopic] Published %s = %s",
    topic_.c_str(), value ? "true" : "false");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace smrr_navigation
