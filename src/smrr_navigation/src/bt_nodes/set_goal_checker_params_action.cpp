#include "smrr_navigation/bt_nodes/set_goal_checker_params_action.hpp"

#include <chrono>
#include <vector>
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"

namespace smrr_navigation
{

SetGoalCheckerParamsAction::SetGoalCheckerParamsAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SetGoalCheckerParamsAction"),
      "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }

  helper_node_ = rclcpp::Node::make_shared("set_goal_checker_params_helper");
  params_client_ = std::make_shared<rclcpp::SyncParametersClient>(
    helper_node_, "/controller_server");
}

BT::NodeStatus SetGoalCheckerParamsAction::tick()
{
  auto xy_res  = getInput<double>("xy_goal_tolerance");
  auto yaw_res = getInput<double>("yaw_goal_tolerance");

  if (!xy_res) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetGoalCheckerParams: missing required input port (xy_goal_tolerance)");
    return BT::NodeStatus::FAILURE;
  }
  if (!yaw_res) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetGoalCheckerParams: missing required input port (yaw_goal_tolerance)");
    return BT::NodeStatus::FAILURE;
  }

  const double xy_tol  = xy_res.value();
  const double yaw_tol = yaw_res.value();

  if (!params_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetGoalCheckerParams: /controller_server set_parameters service not available "
      "within 2 s — is controller_server running?");
    return BT::NodeStatus::FAILURE;
  }

  const std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("general_goal_checker.xy_goal_tolerance",  xy_tol),
    rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance", yaw_tol)
  };

  const auto results = params_client_->set_parameters(params);

  for (const auto & result : results) {
    if (!result.successful) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "SetGoalCheckerParams: parameter set rejected — %s",
        result.reason.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "SetGoalCheckerParams: xy_goal_tolerance=%.3f m, yaw_goal_tolerance=%.3f rad",
    xy_tol, yaw_tol);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace smrr_navigation
