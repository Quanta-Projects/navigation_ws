#ifndef SMRR_NAVIGATION__BT_NODES__PUBLISH_BOOL_TOPIC_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__PUBLISH_BOOL_TOPIC_ACTION_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that publishes std_msgs/Bool true to a given topic.
 *
 * Input ports:
 *   topic  — ROS 2 topic name  (e.g. "/going_in")
 *   value  — bool to publish   (default: true)
 *
 * Usage in BT XML:
 *   <PublishBoolTopic topic="/going_in" value="true"/>
 */
class PublishBoolTopicAction : public BT::SyncActionNode
{
public:
  PublishBoolTopicAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic"),
      BT::InputPort<bool>("value", true, "Value to publish (default true)"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  std::string topic_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__PUBLISH_BOOL_TOPIC_ACTION_HPP_
