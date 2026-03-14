#include "smrr_navigation/bt_nodes/set_costmap_inflation_action.hpp"

#include <chrono>
#include <vector>

#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"

namespace smrr_navigation
{

SetCostmapInflationAction::SetCostmapInflationAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SetCostmapInflationAction"),
      "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }

  helper_node_ = rclcpp::Node::make_shared("set_costmap_inflation_helper");
}

bool SetCostmapInflationAction::setInflationForNode(
  const std::string & node_name,
  double inflation_radius,
  const std::string & scope_label)
{
  auto params_client = std::make_shared<rclcpp::SyncParametersClient>(
    helper_node_, node_name);

  if (!params_client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetCostmapInflation: %s set_parameters service not available within 2 s (node: %s)",
      scope_label.c_str(), node_name.c_str());
    return false;
  }

  const std::vector<rclcpp::Parameter> params = {
    rclcpp::Parameter("inflation_layer.inflation_radius", inflation_radius)
  };

  const auto results = params_client->set_parameters(params);

  for (const auto & result : results) {
    if (!result.successful) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "SetCostmapInflation: %s parameter set rejected on %s - %s",
        scope_label.c_str(), node_name.c_str(), result.reason.c_str());
      return false;
    }
  }

  return true;
}

BT::NodeStatus SetCostmapInflationAction::tick()
{
  auto inflation_radius_res = getInput<double>("inflation_radius");
  auto local_costmap_node_res = getInput<std::string>("local_costmap_node");
  auto global_costmap_node_res = getInput<std::string>("global_costmap_node");

  if (!inflation_radius_res) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetCostmapInflation: missing required input port (inflation_radius)");
    return BT::NodeStatus::FAILURE;
  }

  const double inflation_radius = inflation_radius_res.value();
  const std::string local_costmap_node =
    local_costmap_node_res.value_or("/local_costmap/local_costmap");
  const std::string global_costmap_node =
    global_costmap_node_res.value_or("/global_costmap/global_costmap");

  if (inflation_radius <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "SetCostmapInflation: inflation_radius must be > 0, got %.4f",
      inflation_radius);
    return BT::NodeStatus::FAILURE;
  }

  if (!setInflationForNode(local_costmap_node, inflation_radius, "local_costmap")) {
    return BT::NodeStatus::FAILURE;
  }

  if (!setInflationForNode(global_costmap_node, inflation_radius, "global_costmap")) {
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "SetCostmapInflation: inflation_layer.inflation_radius=%.3f (local + global)",
    inflation_radius);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace smrr_navigation
