#include "smrr_navigation/bt_nodes/toggle_apriltag_action.hpp"

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace smrr_navigation
{

ToggleAprilTagAction::ToggleAprilTagAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ToggleAprilTagAction"),
      "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }

  // Create a lightweight helper node that is never added to any executor.
  // rclcpp::spin_until_future_complete() used by the synchronous service call
  // will internally add a node to a temporary single-threaded executor.
  // If we used node_ (already owned by the BT mission executor) that would
  // throw "node already added to executor".  The helper node avoids this.
  helper_node_ = rclcpp::Node::make_shared("toggle_apriltag_helper");
  client_ = helper_node_->create_client<std_srvs::srv::SetBool>("/toggle_apriltag");
}

BT::NodeStatus ToggleAprilTagAction::tick()
{
  auto turn_on_res = getInput<bool>("turn_on");
  if (!turn_on_res) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ToggleAprilTag: missing required input port (turn_on)");
    return BT::NodeStatus::FAILURE;
  }

  const bool turn_on = turn_on_res.value();

  // Wait up to 2 seconds for the service to be available
  if (!client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ToggleAprilTag: /toggle_apriltag service not available within 2 s — "
      "is apriltag_manager_server running?");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = turn_on;

  auto future = client_->async_send_request(request);

  // Spin the helper node until the response arrives
  if (rclcpp::spin_until_future_complete(helper_node_, future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ToggleAprilTag: service call to /toggle_apriltag failed (future error)");
    return BT::NodeStatus::FAILURE;
  }

  auto response = future.get();
  RCLCPP_INFO(
    node_->get_logger(),
    "ToggleAprilTag: turn_on=%s — %s",
    turn_on ? "true" : "false",
    response->message.c_str());

  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace smrr_navigation
