#include "smrr_navigation/bt_nodes/set_controller_params_action.hpp"

#include <chrono>
#include <vector>
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"

namespace smrr_navigation
{

SetControllerParamsAction::SetControllerParamsAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SetControllerParamsAction"),
      "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }

  // Create a lightweight helper node that is never added to any executor.
  // SyncParametersClient will spin this node internally, which is safe
  // because it has no other executor ownership.
  helper_node_ = rclcpp::Node::make_shared("set_controller_params_helper");
  params_client_ = std::make_shared<rclcpp::SyncParametersClient>(
    helper_node_, "/controller_server");
}

BT::NodeStatus SetControllerParamsAction::tick()
{
  auto max_vel_x_res = getInput<double>("max_vel_x");

  if (!max_vel_x_res) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetControllerParams: missing required input port (max_vel_x)");
    return BT::NodeStatus::FAILURE;
  }

  const double max_vel_x = max_vel_x_res.value();

  if (!params_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetControllerParams: /controller_server set_parameters service not available "
      "within 2 s — is controller_server running?");
    return BT::NodeStatus::FAILURE;
  }

  // Only vx_max is a declared ROS 2 parameter in Nav2 Humble's MPPI.
  // Acceleration limits (ax_max etc.) are internal to the optimizer and
  // are NOT exposed via the parameter server.
  const std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("FollowPath.vx_max", max_vel_x)
  };

  const auto results = params_client_->set_parameters(params);

  for (const auto & result : results) {
    if (!result.successful) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "SetControllerParams: parameter set rejected — %s",
        result.reason.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "SetControllerParams: FollowPath.vx_max=%.3f",
    max_vel_x);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace smrr_navigation
