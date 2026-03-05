#ifndef SMRR_NAVIGATION__BT_NODES__PUBLISH_INITIAL_POSE_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__PUBLISH_INITIAL_POSE_ACTION_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that publishes initial pose for AMCL relocalization.
 *
 * When the optional `current_expected_pose` port is provided, the node looks up
 * the robot's actual TF pose on the departure floor, computes the parking error
 * relative to the expected pose, and applies that error to the target floor's
 * nominal pose before publishing.  This corrects "Transition Misalignment" caused
 * by the robot not parking at exactly the expected elevator-inside position.
 *
 * If `current_expected_pose` is NOT provided the node falls back to directly
 * publishing `initial_pose` unchanged (original behaviour).
 *
 * Ports:
 *   initial_pose          (required)  – target floor's nominal AMCL initial pose
 *   current_expected_pose (optional)  – departure floor's amcl_initial_pose_open
 *   topic_name            (optional)  – default "/initialpose"
 *   frame_id              (optional)  – override header frame_id
 */
class PublishInitialPoseAction : public BT::SyncActionNode
{
public:
  PublishInitialPoseAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "initial_pose",
        "Target floor's nominal AMCL initial pose to publish"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "current_expected_pose",
        "Departure floor's amcl_initial_pose_open used to compute parking error"),
      BT::InputPort<std::string>("topic_name", "/initialpose", "Topic to publish initial pose"),
      BT::InputPort<std::string>(
        "frame_id", "",
        "Override frame_id (default: use pose.header.frame_id)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__PUBLISH_INITIAL_POSE_ACTION_HPP_
