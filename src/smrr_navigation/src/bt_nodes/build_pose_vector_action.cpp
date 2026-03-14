#include "smrr_navigation/bt_nodes/build_pose_vector_action.hpp"
#include <iostream>

namespace smrr_navigation
{

BT::NodeStatus BuildPoseVectorAction::tick()
{
  auto pose1_result = getInput<geometry_msgs::msg::PoseStamped>("pose1");
  auto pose2_result = getInput<geometry_msgs::msg::PoseStamped>("pose2");

  if (!pose1_result || !pose2_result) {
    std::cerr << "[BuildPoseVector] ERROR: Required inputs (pose1, pose2) not available" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.push_back(pose1_result.value());
  poses.push_back(pose2_result.value());

  setOutput("poses", poses);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace smrr_navigation
