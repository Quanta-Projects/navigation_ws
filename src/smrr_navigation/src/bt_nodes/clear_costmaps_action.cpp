#include "smrr_navigation/bt_nodes/clear_costmaps_action.hpp"
#include <chrono>

namespace smrr_navigation
{

ClearCostmapsAction::ClearCostmapsAction(
  const std::string& name,
  const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  // Get ROS 2 node from blackboard
  config.blackboard->get<rclcpp::Node::SharedPtr>("node", node_);
  
  if (!node_) {
    throw BT::RuntimeError("ClearCostmapsAction: Failed to get node from blackboard");
  }

  global_clear_client_ = nullptr;
  local_clear_client_ = nullptr;
}

BT::PortsList ClearCostmapsAction::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "global_service",
      "/global_costmap/clear_entirely_global_costmap",
      "Global costmap clear service"
    ),
    BT::InputPort<std::string>(
      "local_service",
      "/local_costmap/clear_entirely_local_costmap",
      "Local costmap clear service"
    ),
    BT::InputPort<double>("timeout_sec", 3.0, "Service call timeout in seconds")
  };
}

BT::NodeStatus ClearCostmapsAction::tick()
{
  auto global_service = getInput<std::string>("global_service");
  auto local_service = getInput<std::string>("local_service");
  auto timeout_sec = getInput<double>("timeout_sec");

  if (!global_service || !local_service || !timeout_sec) {
    RCLCPP_ERROR(node_->get_logger(), "[ClearCostmaps] Missing required input ports");
    return BT::NodeStatus::FAILURE;
  }

  std::string global_service_str = global_service.value();
  std::string local_service_str = local_service.value();
  double timeout = timeout_sec.value();

  RCLCPP_INFO(
    node_->get_logger(),
    "[ClearCostmaps] Clearing global and local costmaps (timeout: %.1fs)", timeout
  );

  bool global_success = false;
  bool local_success = false;

  // Clear global costmap
  {
    if (!global_clear_client_ || 
        global_clear_client_->get_service_name() != global_service_str) {
      global_clear_client_ = 
        node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(global_service_str);
    }

    if (!global_clear_client_->wait_for_service(std::chrono::duration<double>(timeout))) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[ClearCostmaps] Global costmap clear service unavailable: %s",
        global_service_str.c_str()
      );
    } else {
      auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
      auto future = global_clear_client_->async_send_request(request);

      auto start_time = std::chrono::steady_clock::now();
      while (rclcpp::ok()) {
        // Spin to process service response callbacks
        rclcpp::spin_some(node_);
        
        auto status = future.wait_for(std::chrono::milliseconds(100));
        
        if (status == std::future_status::ready) {
          global_success = true;
          RCLCPP_INFO(node_->get_logger(), "[ClearCostmaps] Global costmap cleared");
          break;
        }

        auto elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start_time
        ).count();
        
        if (elapsed > timeout) {
          RCLCPP_WARN(
            node_->get_logger(),
            "[ClearCostmaps] Global costmap clear timed out"
          );
          break;
        }
      }
    }
  }

  // Clear local costmap
  {
    if (!local_clear_client_ || 
        local_clear_client_->get_service_name() != local_service_str) {
      local_clear_client_ = 
        node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(local_service_str);
    }

    if (!local_clear_client_->wait_for_service(std::chrono::duration<double>(timeout))) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[ClearCostmaps] Local costmap clear service unavailable: %s",
        local_service_str.c_str()
      );
    } else {
      auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
      auto future = local_clear_client_->async_send_request(request);

      auto spin_result = rclcpp::spin_until_future_complete(
        node_,
        future,
        std::chrono::duration<double>(timeout)
      );
      
      if (spin_result == rclcpp::FutureReturnCode::SUCCESS) {
        local_success = true;
        RCLCPP_INFO(node_->get_logger(), "[ClearCostmaps] Local costmap cleared");
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "[ClearCostmaps] Local costmap clear failed or timed out"
        );
      }
    }
  }

  // Require both to succeed
  if (global_success && local_success) {
    RCLCPP_INFO(node_->get_logger(), "[ClearCostmaps] SUCCESS: Both costmaps cleared");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[ClearCostmaps] FAILURE: global=%s, local=%s",
      global_success ? "OK" : "FAILED",
      local_success ? "OK" : "FAILED"
    );
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace smrr_navigation
