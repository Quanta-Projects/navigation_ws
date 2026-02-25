#ifndef SMRR_NAVIGATION__BT_NODES__UPDATE_POSE_TIMESTAMP_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__UPDATE_POSE_TIMESTAMP_ACTION_HPP_

#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that updates the timestamp of a PoseStamped to current time
 * 
 * Useful when a pose was created early in a mission and needs fresh timestamp for Nav2.
 */
class UpdatePoseTimestampAction : public BT::SyncActionNode
{
public:
  UpdatePoseTimestampAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("input_pose", "Pose with old timestamp"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_pose", "Pose with updated timestamp")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__UPDATE_POSE_TIMESTAMP_ACTION_HPP_
