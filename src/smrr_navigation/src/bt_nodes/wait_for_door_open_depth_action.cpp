#include "smrr_navigation/bt_nodes/wait_for_door_open_depth_action.hpp"

#include <cmath>
#include <algorithm>
#include <numeric>
#include <cstring>

namespace smrr_navigation
{

WaitForDoorOpenDepthAction::WaitForDoorOpenDepthAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf),
  depth_received_(false),
  baseline_mean_depth_(0.0),
  baseline_std_depth_(0.0),
  baseline_ready_(false),
  require_both_conditions_effective_(true)
{
  // Get node from blackboard (set by executor)
  config().blackboard->get("node", node_);
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("WaitForDoorOpenDepthAction"), "Node not found in blackboard");
    throw BT::RuntimeError("Missing 'node' in blackboard");
  }
}

void WaitForDoorOpenDepthAction::loadParameters()
{
  // Set defaults
  depth_topic_ = "/camera/depth/image_raw";
  timeout_sec_ = 30.0;
  poll_rate_hz_ = 10.0;
  max_depth_stale_sec_ = 1.0;
  stable_time_sec_ = 1.0;
  roi_x_min_ = 0.30;
  roi_x_max_ = 0.70;
  roi_y_min_ = 0.20;
  roi_y_max_ = 0.85;
  min_depth_m_ = 0.20;
  max_depth_m_ = 5.00;
  sample_step_ = 2;
  baseline_frames_ = 15;
  plane_std_closed_max_ = 0.08;
  plane_std_drop_ratio_ = 2.0;
  free_space_delta_m_ = 0.50;
  free_space_fraction_threshold_ = 0.45;
  require_both_conditions_ = true;
  debug_log_ = false;
  
  // Load from ports with has_value pattern
  auto depth_topic_input = getInput<std::string>("depth_topic");
  if (depth_topic_input.has_value()) depth_topic_ = depth_topic_input.value();
  
  auto timeout_input = getInput<double>("timeout_sec");
  if (timeout_input.has_value()) timeout_sec_ = timeout_input.value();
  
  auto poll_input = getInput<double>("poll_rate_hz");
  if (poll_input.has_value()) poll_rate_hz_ = poll_input.value();
  
  auto stale_input = getInput<double>("max_depth_stale_sec");
  if (stale_input.has_value()) max_depth_stale_sec_ = stale_input.value();
  
  auto stable_input = getInput<double>("stable_time_sec");
  if (stable_input.has_value()) stable_time_sec_ = stable_input.value();
  
  auto roi_x_min_input = getInput<double>("roi_x_min");
  if (roi_x_min_input.has_value()) roi_x_min_ = roi_x_min_input.value();
  
  auto roi_x_max_input = getInput<double>("roi_x_max");
  if (roi_x_max_input.has_value()) roi_x_max_ = roi_x_max_input.value();
  
  auto roi_y_min_input = getInput<double>("roi_y_min");
  if (roi_y_min_input.has_value()) roi_y_min_ = roi_y_min_input.value();
  
  auto roi_y_max_input = getInput<double>("roi_y_max");
  if (roi_y_max_input.has_value()) roi_y_max_ = roi_y_max_input.value();
  
  auto min_depth_input = getInput<double>("min_depth_m");
  if (min_depth_input.has_value()) min_depth_m_ = min_depth_input.value();
  
  auto max_depth_input = getInput<double>("max_depth_m");
  if (max_depth_input.has_value()) max_depth_m_ = max_depth_input.value();
  
  auto sample_step_input = getInput<int>("sample_step");
  if (sample_step_input.has_value()) sample_step_ = sample_step_input.value();
  
  auto baseline_frames_input = getInput<int>("baseline_frames");
  if (baseline_frames_input.has_value()) baseline_frames_ = baseline_frames_input.value();
  
  auto plane_std_max_input = getInput<double>("plane_std_closed_max");
  if (plane_std_max_input.has_value()) plane_std_closed_max_ = plane_std_max_input.value();
  
  auto plane_ratio_input = getInput<double>("plane_std_drop_ratio");
  if (plane_ratio_input.has_value()) plane_std_drop_ratio_ = plane_ratio_input.value();
  
  auto free_delta_input = getInput<double>("free_space_delta_m");
  if (free_delta_input.has_value()) free_space_delta_m_ = free_delta_input.value();
  
  auto free_fraction_input = getInput<double>("free_space_fraction_threshold");
  if (free_fraction_input.has_value()) free_space_fraction_threshold_ = free_fraction_input.value();
  
  auto require_both_input = getInput<bool>("require_both_conditions");
  if (require_both_input.has_value()) require_both_conditions_ = require_both_input.value();
  
  auto debug_input = getInput<bool>("debug_log");
  if (debug_input.has_value()) debug_log_ = debug_input.value();
  
  // Validate sample_step
  if (sample_step_ < 1) sample_step_ = 1;
  
  // Initialize effective require_both from parameter
  require_both_conditions_effective_ = require_both_conditions_;
}

void WaitForDoorOpenDepthAction::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  latest_depth_ = msg;
  last_depth_time_ = node_->get_clock()->now();
  depth_received_ = true;
}

double WaitForDoorOpenDepthAction::getDepthAt(
  const sensor_msgs::msg::Image::SharedPtr & msg,
  int row, int col)
{
  if (!msg) return std::nan("");
  
  const int width = static_cast<int>(msg->width);
  const int height = static_cast<int>(msg->height);
  
  if (row < 0 || row >= height || col < 0 || col >= width) {
    return std::nan("");
  }
  
  double depth_m = std::nan("");
  
  if (msg->encoding == "16UC1") {
    // 16-bit unsigned, depth in millimeters
    const uint16_t* data = reinterpret_cast<const uint16_t*>(msg->data.data());
    size_t idx = static_cast<size_t>(row) * width + col;
    uint16_t depth_mm = data[idx];
    if (depth_mm > 0) {
      depth_m = static_cast<double>(depth_mm) / 1000.0;
    }
  } else if (msg->encoding == "32FC1") {
    // 32-bit float, depth in meters
    const float* data = reinterpret_cast<const float*>(msg->data.data());
    size_t idx = static_cast<size_t>(row) * width + col;
    float depth_val = data[idx];
    if (std::isfinite(depth_val) && depth_val > 0.0f) {
      depth_m = static_cast<double>(depth_val);
    }
  } else {
    // Unsupported encoding
    RCLCPP_WARN_ONCE(node_->get_logger(),
      "[WaitForDoorOpenDepth] Unsupported depth encoding: %s", msg->encoding.c_str());
  }
  
  return depth_m;
}

double WaitForDoorOpenDepthAction::computeMedian(std::vector<double> values)
{
  if (values.empty()) return 0.0;
  
  size_t n = values.size();
  std::sort(values.begin(), values.end());
  
  if (n % 2 == 0) {
    return (values[n/2 - 1] + values[n/2]) / 2.0;
  } else {
    return values[n/2];
  }
}

bool WaitForDoorOpenDepthAction::computeDepthMetrics(
  int & valid_count,
  double & mean_depth,
  double & std_depth,
  double & free_space_fraction)
{
  sensor_msgs::msg::Image::SharedPtr depth_copy;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (!latest_depth_) return false;
    depth_copy = latest_depth_;
  }
  
  const int width = static_cast<int>(depth_copy->width);
  const int height = static_cast<int>(depth_copy->height);
  
  // Compute ROI pixel bounds
  int x_min = static_cast<int>(roi_x_min_ * width);
  int x_max = static_cast<int>(roi_x_max_ * width);
  int y_min = static_cast<int>(roi_y_min_ * height);
  int y_max = static_cast<int>(roi_y_max_ * height);
  
  // Clamp to valid range
  x_min = std::max(0, std::min(x_min, width - 1));
  x_max = std::max(0, std::min(x_max, width - 1));
  y_min = std::max(0, std::min(y_min, height - 1));
  y_max = std::max(0, std::min(y_max, height - 1));
  
  if (x_min >= x_max || y_min >= y_max) {
    RCLCPP_WARN_ONCE(node_->get_logger(),
      "[WaitForDoorOpenDepth] Invalid ROI: x=[%d,%d] y=[%d,%d]", x_min, x_max, y_min, y_max);
    return false;
  }
  
  // Collect valid depth values with subsampling
  std::vector<double> depths;
  depths.reserve(((x_max - x_min) / sample_step_) * ((y_max - y_min) / sample_step_));
  
  for (int row = y_min; row < y_max; row += sample_step_) {
    for (int col = x_min; col < x_max; col += sample_step_) {
      double d = getDepthAt(depth_copy, row, col);
      if (std::isfinite(d) && d >= min_depth_m_ && d <= max_depth_m_) {
        depths.push_back(d);
      }
    }
  }
  
  valid_count = static_cast<int>(depths.size());
  
  if (valid_count < 50) {
    // Insufficient valid pixels
    mean_depth = 0.0;
    std_depth = 0.0;
    free_space_fraction = 0.0;
    return false;
  }
  
  // Compute mean
  double sum = std::accumulate(depths.begin(), depths.end(), 0.0);
  mean_depth = sum / static_cast<double>(valid_count);
  
  // Compute standard deviation
  double sq_sum = 0.0;
  for (double d : depths) {
    double diff = d - mean_depth;
    sq_sum += diff * diff;
  }
  std_depth = std::sqrt(sq_sum / static_cast<double>(valid_count));
  
  // Compute free space fraction (only if baseline is ready)
  if (baseline_ready_) {
    double free_threshold = baseline_mean_depth_ + free_space_delta_m_;
    int free_count = 0;
    for (double d : depths) {
      if (d > free_threshold) {
        free_count++;
      }
    }
    free_space_fraction = static_cast<double>(free_count) / static_cast<double>(valid_count);
  } else {
    free_space_fraction = 0.0;
  }
  
  return true;
}

BT::NodeStatus WaitForDoorOpenDepthAction::onStart()
{
  // Load parameters from ports
  loadParameters();
  
  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForDoorOpenDepth] Starting depth-based door detection on topic '%s' "
    "(timeout=%.1fs, ROI=[%.2f-%.2f, %.2f-%.2f], baseline_frames=%d, stable=%.1fs)",
    depth_topic_.c_str(), timeout_sec_, roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_,
    baseline_frames_, stable_time_sec_);
  
  // Reset state
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth_received_ = false;
    latest_depth_.reset();
  }
  stable_start_time_.reset();
  baseline_means_.clear();
  baseline_stds_.clear();
  baseline_mean_depth_ = 0.0;
  baseline_std_depth_ = 0.0;
  baseline_ready_ = false;
  require_both_conditions_effective_ = require_both_conditions_;
  
  // Record start time
  start_time_ = node_->get_clock()->now();
  last_check_time_ = start_time_;
  
  // Create subscription
  depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&WaitForDoorOpenDepthAction::depthCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(node_->get_logger(),
    "[WaitForDoorOpenDepth] Subscribed to %s, collecting baseline...", depth_topic_.c_str());
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDoorOpenDepthAction::onRunning()
{
  rclcpp::Time now = node_->get_clock()->now();
  
  // Check overall timeout
  double elapsed = (now - start_time_).seconds();
  if (elapsed > timeout_sec_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[WaitForDoorOpenDepth] FAILURE: Timeout after %.1f seconds waiting for door to open",
      elapsed);
    return BT::NodeStatus::FAILURE;
  }
  
  // Rate limiting - only evaluate at poll_rate_hz
  double time_since_last_check = (now - last_check_time_).seconds();
  double poll_period = 1.0 / poll_rate_hz_;
  if (time_since_last_check < poll_period) {
    return BT::NodeStatus::RUNNING;
  }
  last_check_time_ = now;
  
  // Check if we have received any depth data
  rclcpp::Time depth_time;
  bool have_depth = false;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    have_depth = depth_received_;
    if (have_depth) {
      depth_time = last_depth_time_;
    }
  }
  
  if (!have_depth) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "[WaitForDoorOpenDepth] No depth data received yet on topic '%s'",
      depth_topic_.c_str());
    return BT::NodeStatus::RUNNING;
  }
  
  // Check depth data age
  double depth_age = (now - depth_time).seconds();
  if (depth_age > max_depth_stale_sec_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[WaitForDoorOpenDepth] FAILURE: Depth data is stale (age: %.2fs > %.2fs)",
      depth_age, max_depth_stale_sec_);
    return BT::NodeStatus::FAILURE;
  }
  
  // Compute current depth metrics
  int valid_count = 0;
  double roi_mean = 0.0;
  double roi_std = 0.0;
  double free_space_fraction = 0.0;
  
  if (!computeDepthMetrics(valid_count, roi_mean, roi_std, free_space_fraction)) {
    if (debug_log_) {
      RCLCPP_INFO(node_->get_logger(),
        "[WaitForDoorOpenDepth] DEBUG: Insufficient valid pixels (%d)", valid_count);
    }
    return BT::NodeStatus::RUNNING;
  }
  
  // Baseline collection phase
  if (!baseline_ready_) {
    baseline_means_.push_back(roi_mean);
    baseline_stds_.push_back(roi_std);
    
    if (static_cast<int>(baseline_means_.size()) >= baseline_frames_) {
      // Compute baseline as median of collected values (robust)
      baseline_mean_depth_ = computeMedian(baseline_means_);
      baseline_std_depth_ = computeMedian(baseline_stds_);
      baseline_ready_ = true;
      
      RCLCPP_INFO(node_->get_logger(),
        "[WaitForDoorOpenDepth] Baseline collected: mean=%.3fm, std=%.4fm (%d frames)",
        baseline_mean_depth_, baseline_std_depth_, baseline_frames_);
      
      // Check if baseline std is unusually high (door might already be open or noisy scene)
      if (baseline_std_depth_ > plane_std_closed_max_) {
        RCLCPP_WARN(node_->get_logger(),
          "[WaitForDoorOpenDepth] Baseline std (%.4f) exceeds expected closed-door max (%.4f). "
          "Switching to free-space-only detection mode.",
          baseline_std_depth_, plane_std_closed_max_);
        require_both_conditions_effective_ = false;
      }
    } else {
      if (debug_log_) {
        RCLCPP_INFO(node_->get_logger(),
          "[WaitForDoorOpenDepth] DEBUG: Collecting baseline %zu/%d (mean=%.3f, std=%.4f)",
          baseline_means_.size(), baseline_frames_, roi_mean, roi_std);
      }
    }
    return BT::NodeStatus::RUNNING;
  }
  
  // Detection phase - check door open conditions
  bool plane_condition = (roi_std >= baseline_std_depth_ * plane_std_drop_ratio_);
  bool free_space_condition = (free_space_fraction >= free_space_fraction_threshold_);
  
  bool condition_met;
  if (require_both_conditions_effective_) {
    condition_met = plane_condition && free_space_condition;
  } else {
    condition_met = plane_condition || free_space_condition;
  }
  
  if (debug_log_) {
    RCLCPP_INFO(node_->get_logger(),
      "[WaitForDoorOpenDepth] DEBUG: roi_mean=%.3f, roi_std=%.4f, free_frac=%.1f%%, "
      "baseline_mean=%.3f, baseline_std=%.4f, plane_cond=%s, free_cond=%s, met=%s",
      roi_mean, roi_std, free_space_fraction * 100.0,
      baseline_mean_depth_, baseline_std_depth_,
      plane_condition ? "Y" : "N",
      free_space_condition ? "Y" : "N",
      condition_met ? "Y" : "N");
  }
  
  // Stability check
  if (condition_met) {
    if (!stable_start_time_.has_value()) {
      // Condition just became true, start stability timer
      stable_start_time_ = now;
      RCLCPP_INFO(node_->get_logger(),
        "[WaitForDoorOpenDepth] Door open detected (std=%.4f, free=%.1f%%), "
        "starting stability timer (%.1fs)...",
        roi_std, free_space_fraction * 100.0, stable_time_sec_);
    } else {
      // Condition still true, check stability duration
      double stable_duration = (now - stable_start_time_.value()).seconds();
      if (stable_duration >= stable_time_sec_) {
        RCLCPP_INFO(node_->get_logger(),
          "[WaitForDoorOpenDepth] SUCCESS: Door open stable for %.2fs "
          "(std=%.4f vs baseline %.4f, free=%.1f%%)",
          stable_duration, roi_std, baseline_std_depth_, free_space_fraction * 100.0);
        return BT::NodeStatus::SUCCESS;
      }
    }
  } else {
    // Condition not met
    if (stable_start_time_.has_value()) {
      RCLCPP_WARN(node_->get_logger(),
        "[WaitForDoorOpenDepth] Door open condition lost (std=%.4f, free=%.1f%%), "
        "resetting stability timer",
        roi_std, free_space_fraction * 100.0);
      stable_start_time_.reset();
    }
  }
  
  return BT::NodeStatus::RUNNING;
}

void WaitForDoorOpenDepthAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[WaitForDoorOpenDepth] Halted");
  
  // Clean up subscription and state
  depth_sub_.reset();
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth_received_ = false;
    latest_depth_.reset();
  }
  stable_start_time_.reset();
  baseline_means_.clear();
  baseline_stds_.clear();
  baseline_ready_ = false;
}

}  // namespace smrr_navigation
