#include "smrr_navigation/bt_nodes/call_elevator_action.hpp"

#include <cstdio>
#include <array>
#include <memory>
#include <chrono>
#include <thread>
#include <sstream>

namespace smrr_navigation
{

CallElevatorAction::CallElevatorAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  // Get node from blackboard (set by executor)
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("CallElevatorAction"), "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }
  
  // Initialize floor mapping (matches Python implementation exactly)
  floor_map_ = {
    {"floor0", "0"},
    {"floor1", "1"},
    {"floor2", "2"},
    {"floor3", "3"}
  };
}

int CallElevatorAction::executeCommand(
  const std::string & cmd,
  double timeout_sec,
  std::string & stdout_output,
  std::string & stderr_output)
{
  stdout_output.clear();
  stderr_output.clear();
  
  // Create command with stderr redirect to capture both streams
  // Using popen for simplicity - redirects stderr to stdout
  std::string full_cmd = cmd + " 2>&1";
  
  RCLCPP_DEBUG(node_->get_logger(), "Executing command: %s", cmd.c_str());
  
  // Use popen to execute command
  FILE* pipe = popen(full_cmd.c_str(), "r");
  if (!pipe) {
    stderr_output = "Failed to create pipe for command execution";
    RCLCPP_ERROR(node_->get_logger(), "%s", stderr_output.c_str());
    return -1;
  }
  
  // Set up timeout tracking
  auto start_time = std::chrono::steady_clock::now();
  auto timeout_duration = std::chrono::duration<double>(timeout_sec);
  
  // Read output with timeout check
  std::array<char, 256> buffer;
  std::stringstream output_stream;
  
  while (true) {
    // Check timeout
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed > timeout_duration) {
      pclose(pipe);
      stderr_output = "Command timed out after " + std::to_string(timeout_sec) + " seconds";
      RCLCPP_ERROR(node_->get_logger(), "%s", stderr_output.c_str());
      return -1;
    }
    
    // Try to read from pipe
    if (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
      output_stream << buffer.data();
    } else {
      // EOF or error
      break;
    }
  }
  
  stdout_output = output_stream.str();
  
  // Get exit status
  int status = pclose(pipe);
  int exit_code = -1;
  
  if (WIFEXITED(status)) {
    exit_code = WEXITSTATUS(status);
  }
  
  return exit_code;
}

BT::NodeStatus CallElevatorAction::tick()
{
  // Get input ports
  std::string floor_id;
  auto floor_input = getInput<std::string>("floor_id");
  if (!floor_input.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "[CallElevator] Missing required input: floor_id");
    return BT::NodeStatus::FAILURE;
  }
  floor_id = floor_input.value();
  
  // Get optional inputs with defaults
  std::string gz_cli = "gz-11.14.0";
  std::string gz_elevator_topic = "/gazebo/default/elevator";
  double timeout_sec = 5.0;
  
  auto gz_cli_input = getInput<std::string>("gz_cli");
  if (gz_cli_input.has_value()) {
    gz_cli = gz_cli_input.value();
  }
  
  auto topic_input = getInput<std::string>("gz_elevator_topic");
  if (topic_input.has_value()) {
    gz_elevator_topic = topic_input.value();
  }
  
  auto timeout_input = getInput<double>("timeout_sec");
  if (timeout_input.has_value()) {
    timeout_sec = timeout_input.value();
  }
  
  // Map floor_id to floor number
  auto it = floor_map_.find(floor_id);
  if (it == floor_map_.end()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[CallElevator] Unknown floor_id: '%s'. Valid values: floor0, floor1, floor2, floor3",
      floor_id.c_str());
    return BT::NodeStatus::FAILURE;
  }
  std::string floor_num = it->second;
  
  // Build command string matching Python exactly:
  // [gz_cli, "topic", "-p", gz_elevator_topic, "-m", "data: \"<floor_num>\""]
  // Shell: gz-11.14.0 topic -p /gazebo/default/elevator -m 'data: "0"'
  std::stringstream cmd_ss;
  cmd_ss << gz_cli 
         << " topic -p " << gz_elevator_topic 
         << " -m 'data: \"" << floor_num << "\"'";
  std::string cmd = cmd_ss.str();
  
  RCLCPP_INFO(
    node_->get_logger(),
    "[CallElevator] Calling elevator to %s (floor number: %s)",
    floor_id.c_str(), floor_num.c_str());
  RCLCPP_INFO(node_->get_logger(), "[CallElevator] Executing: %s", cmd.c_str());
  
  // Execute command
  std::string stdout_output, stderr_output;
  int exit_code = executeCommand(cmd, timeout_sec, stdout_output, stderr_output);
  
  if (exit_code == 0) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[CallElevator] SUCCESS: Elevator commanded to floor %s",
      floor_num.c_str());
    if (!stdout_output.empty()) {
      RCLCPP_DEBUG(node_->get_logger(), "[CallElevator] stdout: %s", stdout_output.c_str());
    }
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[CallElevator] FAILURE: Command failed with exit code %d",
      exit_code);
    if (!stdout_output.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "[CallElevator] output: %s", stdout_output.c_str());
    }
    if (!stderr_output.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "[CallElevator] stderr: %s", stderr_output.c_str());
    }
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace smrr_navigation
