#ifndef SMRR_NAVIGATION__BT_NODES__BUILD_POSE_VECTOR_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__BUILD_POSE_VECTOR_ACTION_HPP_

#include <vector>
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace smrr_navigation
{

/**
 * @brief BT Sync action that combines two PoseStamped inputs into a vector.
 *
 * Used to build the `goals` input for NavigateThroughPoses.
 *
 * Input Ports:
 *   - pose1: First waypoint (passed through)
 *   - pose2: Final destination
 * Output Ports:
 *   - poses: vector<PoseStamped> ordered [pose1, pose2]
 */
class BuildPoseVectorAction : public BT::SyncActionNode
{
public:
  BuildPoseVectorAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose1", "First waypoint pose"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose2", "Final destination pose"),
      BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("poses", "Ordered pose vector")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__BUILD_POSE_VECTOR_ACTION_HPP_
