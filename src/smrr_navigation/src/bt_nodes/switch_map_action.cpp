#include "smrr_navigation/bt_nodes/switch_map_action.hpp"
#include <thread>

namespace smrr_navigation
{

SwitchMapAction::SwitchMapAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf), request_sent_(false)
{
  // Get node from blackboard (set by executor)
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("SwitchMapAction"), "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }
}

void SwitchMapAction::onHalted()
{
  request_sent_ = false;
}

BT::NodeStatus SwitchMapAction::onStart()
{
  auto map_yaml = getInput<std::string>("map_yaml");
  auto service_name = getInput<std::string>("service_name").value();

  if (!map_yaml) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SwitchMapAction"),
      "Missing required input: map_yaml");
    return BT::NodeStatus::FAILURE;
  }

  if (request_sent_) {
    // Already sent, this shouldn't happen in onStart
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SwitchMapAction"),
    "Switching to map: %s (service: %s)",
    map_yaml.value().c_str(), service_name.c_str());

  // Create service client if not exists
  if (!load_map_client_) {
    load_map_client_ = node_->create_client<nav2_msgs::srv::LoadMap>(service_name);
  }

  // Wait for service
  if (!load_map_client_->wait_for_service(std::chrono::milliseconds(1000))) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SwitchMapAction"),
      "Service '%s' not available",
      service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Prepare and send request
  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = map_yaml.value();
  
  // Use .future.share() as recommended (not deprecated)
  future_ = load_map_client_->async_send_request(request).future.share();
  request_start_time_ = std::chrono::steady_clock::now();
  request_sent_ = true;
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SwitchMapAction::onRunning()
{
  auto map_yaml = getInput<std::string>("map_yaml").value();
  auto timeout_ms = getInput<int>("timeout_ms").value();

  // Check if future is valid
  if (!future_.valid()) {
    request_sent_ = false;
    RCLCPP_ERROR(
      rclcpp::get_logger("SwitchMapAction"),
      "Future is not valid");
    return BT::NodeStatus::FAILURE;
  }

  // Check if response ready
  auto status = future_.wait_for(std::chrono::milliseconds(0));
  if (status == std::future_status::ready) {
    request_sent_ = false;
    auto response = future_.get();
    
    if (response->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SwitchMapAction"),
        "Failed to load map: %s (result code: %d)",
        map_yaml.c_str(), response->result);
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("SwitchMapAction"),
      "Successfully loaded map: %s", map_yaml.c_str());

    return BT::NodeStatus::SUCCESS;
  }

  // Check timeout
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - request_start_time_).count();
  if (elapsed >= timeout_ms) {
    request_sent_ = false;
    RCLCPP_ERROR(
      rclcpp::get_logger("SwitchMapAction"),
      "Service call timed out after %ld ms", elapsed);
    return BT::NodeStatus::FAILURE;
  }

  // Still waiting
  return BT::NodeStatus::RUNNING;
}

}  // namespace smrr_navigation
