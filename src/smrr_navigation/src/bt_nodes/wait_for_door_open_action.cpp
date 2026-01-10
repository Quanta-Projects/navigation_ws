#include "smrr_navigation/bt_nodes/wait_for_door_open_action.hpp"

#include <cmath>
#include <algorithm>

namespace smrr_navigation
{

WaitForDoorOpenAction::WaitForDoorOpenAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf),
  scan_received_(false),
  door_open_timer_active_(false)
{
  // Get node from blackboard (set by executor)
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("WaitForDoorOpenAction"), "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }
}

void WaitForDoorOpenAction::loadParameters()
{
  // Load all parameters with defaults
  scan_topic_ = "/scan";
  timeout_sec_ = 30.0;
  window_center_deg_ = 0.0;
  window_width_deg_ = 30.0;
  range_threshold_m_ = 2.0;
  fraction_threshold_ = 0.6;
  stable_time_sec_ = 1.0;
  poll_rate_hz_ = 10.0;
  max_scan_stale_sec_ = 1.0;
  
  auto scan_topic_input = getInput<std::string>("scan_topic");
  if (scan_topic_input.has_value()) scan_topic_ = scan_topic_input.value();
  
  auto timeout_input = getInput<double>("timeout_sec");
  if (timeout_input.has_value()) timeout_sec_ = timeout_input.value();
  
  auto center_input = getInput<double>("window_center_deg");
  if (center_input.has_value()) window_center_deg_ = center_input.value();
  
  auto width_input = getInput<double>("window_width_deg");
  if (width_input.has_value()) window_width_deg_ = width_input.value();
  
  auto range_input = getInput<double>("range_threshold_m");
  if (range_input.has_value()) range_threshold_m_ = range_input.value();
  
  auto fraction_input = getInput<double>("fraction_threshold");
  if (fraction_input.has_value()) fraction_threshold_ = fraction_input.value();
  
  auto stable_input = getInput<double>("stable_time_sec");
  if (stable_input.has_value()) stable_time_sec_ = stable_input.value();
  
  auto poll_input = getInput<double>("poll_rate_hz");
  if (poll_input.has_value()) poll_rate_hz_ = poll_input.value();
  
  auto stale_input = getInput<double>("max_scan_stale_sec");
  if (stale_input.has_value()) max_scan_stale_sec_ = stale_input.value();
}

void WaitForDoorOpenAction::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
  latest_scan_time_ = node_->get_clock()->now();
  scan_received_ = true;
}

bool WaitForDoorOpenAction::isDoorOpen()
{
  if (!latest_scan_) {
    return false;
  }
  
  const auto & scan = *latest_scan_;
  
  // Convert window parameters to radians
  double center_rad = window_center_deg_ * M_PI / 180.0;
  double width_rad = window_width_deg_ * M_PI / 180.0;
  
  double window_min = center_rad - width_rad / 2.0;
  double window_max = center_rad + width_rad / 2.0;
  
  // Clamp window to scan range
  double window_min_clamped = std::max(window_min, static_cast<double>(scan.angle_min));
  double window_max_clamped = std::min(window_max, static_cast<double>(scan.angle_max));
  
  if (window_min_clamped >= window_max_clamped) {
    RCLCPP_WARN_ONCE(
      node_->get_logger(),
      "[WaitForDoorOpen] Door detection window outside scan range");
    return false;
  }
  
  // Compute index range
  int num_rays = static_cast<int>(scan.ranges.size());
  double angle_inc = scan.angle_increment;
  double angle_min = scan.angle_min;
  
  int idx_min = static_cast<int>((window_min_clamped - angle_min) / angle_inc);
  int idx_max = static_cast<int>((window_max_clamped - angle_min) / angle_inc);
  
  // Clamp indices to valid range
  idx_min = std::max(0, idx_min);
  idx_max = std::min(num_rays - 1, idx_max);
  
  if (idx_min >= idx_max) {
    return false;
  }
  
  // Count valid and open rays
  int valid_count = 0;
  int open_count = 0;
  
  for (int i = idx_min; i <= idx_max; ++i) {
    float r = scan.ranges[i];
    
    // Skip non-finite ranges
    if (!std::isfinite(r)) {
      continue;
    }
    
    // Clamp to valid range
    r = std::max(scan.range_min, std::min(scan.range_max, r));
    
    valid_count++;
    if (r > range_threshold_m_) {
      open_count++;
    }
  }
  
  // Need at least 5 valid rays (matching Python)
  if (valid_count < 5) {
    return false;
  }
  
  double fraction = static_cast<double>(open_count) / static_cast<double>(valid_count);
  return fraction >= fraction_threshold_;
}

BT::NodeStatus WaitForDoorOpenAction::onStart()
{
  // Load parameters from ports
  loadParameters();
  
  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForDoorOpen] Starting door open detection on topic '%s' "
    "(timeout=%.1fs, window=%.1f±%.1f°, threshold=%.2fm, fraction=%.0f%%, stable=%.1fs)",
    scan_topic_.c_str(), timeout_sec_, window_center_deg_, window_width_deg_ / 2.0,
    range_threshold_m_, fraction_threshold_ * 100.0, stable_time_sec_);
  
  // Reset state
  scan_received_ = false;
  latest_scan_.reset();
  door_open_timer_active_ = false;
  
  // Record start time
  start_time_ = node_->get_clock()->now();
  last_check_time_ = start_time_;
  
  // Create subscription if not already created (or if topic changed)
  // For simplicity, always recreate the subscription
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&WaitForDoorOpenAction::scanCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(node_->get_logger(), "[WaitForDoorOpen] Subscribed to %s, waiting for door...", scan_topic_.c_str());
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDoorOpenAction::onRunning()
{
  rclcpp::Time now = node_->get_clock()->now();
  
  // Check overall timeout
  double elapsed = (now - start_time_).seconds();
  if (elapsed > timeout_sec_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[WaitForDoorOpen] FAILURE: Timeout after %.1f seconds waiting for door to open",
      elapsed);
    return BT::NodeStatus::FAILURE;
  }
  
  // Rate limiting - only evaluate at poll_rate_hz
  double time_since_last_check = (now - last_check_time_).seconds();
  double poll_period = 1.0 / poll_rate_hz_;
  if (time_since_last_check < poll_period) {
    // Not time to check yet, keep running
    return BT::NodeStatus::RUNNING;
  }
  last_check_time_ = now;
  
  // Check if we have received any scan
  if (!scan_received_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,  // Warn every 2 seconds
      "[WaitForDoorOpen] No scan received yet on topic '%s'",
      scan_topic_.c_str());
    return BT::NodeStatus::RUNNING;
  }
  
  // Check scan age
  double scan_age = (now - latest_scan_time_).seconds();
  if (scan_age > max_scan_stale_sec_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[WaitForDoorOpen] FAILURE: Scan data is stale (age: %.2fs > %.2fs)",
      scan_age, max_scan_stale_sec_);
    return BT::NodeStatus::FAILURE;
  }
  
  // Check door open condition
  bool door_open = isDoorOpen();
  
  if (door_open) {
    if (!door_open_timer_active_) {
      // Door just opened, start stability timer
      door_open_start_time_ = now;
      door_open_timer_active_ = true;
      RCLCPP_INFO(
        node_->get_logger(),
        "[WaitForDoorOpen] Door open detected, waiting for stability (%.1fs)...",
        stable_time_sec_);
    } else {
      // Door still open, check stability
      double stable_duration = (now - door_open_start_time_).seconds();
      if (stable_duration >= stable_time_sec_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "[WaitForDoorOpen] SUCCESS: Door open stable for %.2fs",
          stable_duration);
        return BT::NodeStatus::SUCCESS;
      }
    }
  } else {
    // Door not open (or closed again)
    if (door_open_timer_active_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[WaitForDoorOpen] Door open condition lost, resetting stability timer");
      door_open_timer_active_ = false;
    }
  }
  
  return BT::NodeStatus::RUNNING;
}

void WaitForDoorOpenAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[WaitForDoorOpen] Halted");
  
  // Clean up subscription
  scan_sub_.reset();
  scan_received_ = false;
  door_open_timer_active_ = false;
}

}  // namespace smrr_navigation
