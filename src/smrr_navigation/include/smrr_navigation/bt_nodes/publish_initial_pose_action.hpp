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
 * Three operating modes (evaluated in priority order):
 *
 * 1. **AprilTag mode** (`use_apriltag = true`):
 *    Uses a live TF lookup of the tag relative to the robot to correct the
 *    ideal initial pose for elevator parking error.
 *    Formula:
 *      T_map_to_actual_base = T_map_to_ideal_base * T_ideal_base_to_tag
 *                             * T_actual_base_to_tag.inverse()
 *    The result is flattened to strict 2D (z=0, roll=0, pitch=0) before
 *    publishing.  Falls back to direct publish if the TF lookup fails.
 *
 * 2. **Relative-error mode** (`current_expected_pose` port provided):
 *    Looks up the actual robot TF in the map frame, computes the parking
 *    error relative to the departure floor's expected pose, and applies that
 *    error to the target floor's nominal pose.
 *
 * 3. **Direct publish** (default):
 *    Publishes `initial_pose` unchanged.
 *
 * Ports:
 *   initial_pose            (required) – target floor nominal AMCL initial pose
 *   current_expected_pose   (optional) – departure floor amcl_initial_pose_open
 *   topic_name              (optional) – default "/initialpose"
 *   frame_id                (optional) – override header frame_id
 *   use_apriltag            (optional, bool)   – default false
 *   tag_frame               (optional, string) – default "tag36h11:0"
 *   base_frame              (optional, string) – default "base_link"
 *   expected_tag_x/y/z      (optional, double) – expected tag position in base_frame
 *   expected_tag_roll/pitch/yaw (optional, double) – expected tag orientation (rad)
 */
class PublishInitialPoseAction : public BT::SyncActionNode
{
public:
  PublishInitialPoseAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      // ── Existing ports ────────────────────────────────────────────────────
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "initial_pose",
        "Target floor's nominal AMCL initial pose to publish"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "current_expected_pose",
        "Departure floor's amcl_initial_pose_open used to compute parking error"),
      BT::InputPort<std::string>("topic_name", "/initialpose", "Topic to publish initial pose"),
      BT::InputPort<std::string>(
        "frame_id", "",
        "Override frame_id (default: use pose.header.frame_id)"),

      // ── AprilTag relocalization ports ─────────────────────────────────────
      BT::InputPort<bool>(
        "use_apriltag", false,
        "If true, use AprilTag TF to correct parking error"),
      BT::InputPort<std::string>(
        "tag_frame", "tag36h11:0",
        "TF frame of the AprilTag (published by apriltag_ros)"),
      BT::InputPort<std::string>(
        "base_frame", "base_link",
        "Robot base frame for live TF lookup"),
      BT::InputPort<double>("expected_tag_x",     0.0, "Expected tag x in base_frame (m)"),
      BT::InputPort<double>("expected_tag_y",     0.0, "Expected tag y in base_frame (m)"),
      BT::InputPort<double>("expected_tag_z",     0.0, "Expected tag z in base_frame (m)"),
      BT::InputPort<double>("expected_tag_roll",  0.0, "Expected tag roll  in base_frame (rad)"),
      BT::InputPort<double>("expected_tag_pitch", 0.0, "Expected tag pitch in base_frame (rad)"),
      BT::InputPort<double>("expected_tag_yaw",   0.0, "Expected tag yaw   in base_frame (rad)")
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
