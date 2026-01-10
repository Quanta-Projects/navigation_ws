#ifndef SMRR_NAVIGATION__BT_NODES__PUBLISH_INITIAL_POSE_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__PUBLISH_INITIAL_POSE_ACTION_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that publishes initial pose for AMCL relocalization
 * 
 * Publishes geometry_msgs/msg/PoseWithCovarianceStamped to /initialpose topic.
 */
class PublishInitialPoseAction : public BT::SyncActionNode
{
public:
  PublishInitialPoseAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("initial_pose", "Initial pose to publish"),
      BT::InputPort<std::string>("topic_name", "/initialpose", "Topic to publish initial pose"),
      BT::InputPort<std::string>("frame_id", "", "Override frame_id (default: use pose.header.frame_id)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__PUBLISH_INITIAL_POSE_ACTION_HPP_
