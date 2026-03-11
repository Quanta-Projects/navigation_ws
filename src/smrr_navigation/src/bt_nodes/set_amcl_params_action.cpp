#include "smrr_navigation/bt_nodes/set_amcl_params_action.hpp"

#include <chrono>
#include <vector>
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"

namespace smrr_navigation
{

SetAMCLParamsAction::SetAMCLParamsAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SetAMCLParamsAction"),
      "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }

  // Create a lightweight helper node that is never added to any executor.
  // SyncParametersClient will spin this node internally, which is safe
  // because it has no other executor ownership.
  helper_node_ = rclcpp::Node::make_shared("set_amcl_params_helper");
  params_client_ = std::make_shared<rclcpp::SyncParametersClient>(
    helper_node_, "/amcl");
}

BT::NodeStatus SetAMCLParamsAction::tick()
{
  auto update_min_d_res = getInput<double>("update_min_d");
  auto update_min_a_res = getInput<double>("update_min_a");

  if (!update_min_d_res) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetAMCLParams: missing required input port (update_min_d)");
    return BT::NodeStatus::FAILURE;
  }
  if (!update_min_a_res) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetAMCLParams: missing required input port (update_min_a)");
    return BT::NodeStatus::FAILURE;
  }

  const double update_min_d = update_min_d_res.value();
  const double update_min_a = update_min_a_res.value();

  if (!params_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetAMCLParams: /amcl set_parameters service not available within 2 s "
      "— is amcl running?");
    return BT::NodeStatus::FAILURE;
  }

  const std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("update_min_d", update_min_d),
    rclcpp::Parameter("update_min_a", update_min_a)
  };

  const auto results = params_client_->set_parameters(params);

  for (const auto & result : results) {
    if (!result.successful) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "SetAMCLParams: parameter set rejected — %s",
        result.reason.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "SetAMCLParams: update_min_d=%.4f m, update_min_a=%.4f rad",
    update_min_d, update_min_a);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace smrr_navigation
