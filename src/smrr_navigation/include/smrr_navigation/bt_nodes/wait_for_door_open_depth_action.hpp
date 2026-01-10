#ifndef SMRR_NAVIGATION__BT_NODES__WAIT_FOR_DOOR_OPEN_DEPTH_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__WAIT_FOR_DOOR_OPEN_DEPTH_ACTION_HPP_

#include <string>
#include <memory>
#include <cmath>
#include <vector>
#include <optional>
#include <mutex>
#include <algorithm>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace smrr_navigation
{

/**
 * @brief BT StatefulActionNode that waits for elevator door to open using depth camera.
 * 
 * Uses depth image data to detect when the door ahead has opened by analyzing
 * the depth distribution in a configurable ROI. The algorithm:
 * 
 * 1. Baseline Collection Phase:
 *    - Collects N frames with door closed to establish baseline mean/std of depth in ROI
 *    - Uses median of collected values for robustness
 * 
 * 2. Detection Phase:
 *    - Computes current ROI mean and std deviation of depth
 *    - Computes fraction of pixels that are "free space" (depth > baseline_mean + delta)
 *    - Door open when: std increases significantly OR significant free space appears
 * 
 * 3. Stability Phase:
 *    - Condition must remain true for stable_time_sec before SUCCESS
 */
class WaitForDoorOpenDepthAction : public BT::StatefulActionNode
{
public:
  WaitForDoorOpenDepthAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      // Topic configuration
      BT::InputPort<std::string>("depth_topic", "/camera/depth/image_raw", "Depth image topic"),
      
      // Timing parameters
      BT::InputPort<double>("timeout_sec", 30.0, "Timeout for door open detection"),
      BT::InputPort<double>("poll_rate_hz", 10.0, "Rate limit for depth evaluation"),
      BT::InputPort<double>("max_depth_stale_sec", 1.0, "Max age for depth data before failure"),
      BT::InputPort<double>("stable_time_sec", 1.0, "Time door must remain open for success"),
      
      // ROI definition (fractions 0..1)
      BT::InputPort<double>("roi_x_min", 0.30, "ROI left edge (fraction of image width)"),
      BT::InputPort<double>("roi_x_max", 0.70, "ROI right edge (fraction of image width)"),
      BT::InputPort<double>("roi_y_min", 0.20, "ROI top edge (fraction of image height)"),
      BT::InputPort<double>("roi_y_max", 0.85, "ROI bottom edge (fraction of image height)"),
      
      // Depth filtering
      BT::InputPort<double>("min_depth_m", 0.20, "Minimum valid depth in meters"),
      BT::InputPort<double>("max_depth_m", 5.00, "Maximum valid depth in meters"),
      BT::InputPort<int>("sample_step", 2, "Pixel subsampling step in ROI"),
      
      // Baseline / detection thresholds
      BT::InputPort<int>("baseline_frames", 15, "Number of frames to collect for baseline"),
      BT::InputPort<double>("plane_std_closed_max", 0.08, "Max expected std when door closed"),
      BT::InputPort<double>("plane_std_drop_ratio", 2.0, "Std multiplier to detect door open"),
      BT::InputPort<double>("free_space_delta_m", 0.50, "Depth increase threshold for free space"),
      BT::InputPort<double>("free_space_fraction_threshold", 0.45, "Fraction of free space pixels required"),
      BT::InputPort<bool>("require_both_conditions", true, "Require both plane AND free-space conditions"),
      BT::InputPort<bool>("debug_log", false, "Enable verbose debug logging")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  
  // Latest depth data (thread-safe access)
  std::mutex depth_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  rclcpp::Time last_depth_time_;
  bool depth_received_;
  
  // Timing
  rclcpp::Time start_time_;
  rclcpp::Time last_check_time_;
  std::optional<rclcpp::Time> stable_start_time_;
  
  // Baseline collection
  std::vector<double> baseline_means_;
  std::vector<double> baseline_stds_;
  double baseline_mean_depth_;
  double baseline_std_depth_;
  bool baseline_ready_;
  bool require_both_conditions_effective_;  // May be modified if baseline std is too high
  
  // Parameters (loaded from ports)
  std::string depth_topic_;
  double timeout_sec_;
  double poll_rate_hz_;
  double max_depth_stale_sec_;
  double stable_time_sec_;
  double roi_x_min_;
  double roi_x_max_;
  double roi_y_min_;
  double roi_y_max_;
  double min_depth_m_;
  double max_depth_m_;
  int sample_step_;
  int baseline_frames_;
  double plane_std_closed_max_;
  double plane_std_drop_ratio_;
  double free_space_delta_m_;
  double free_space_fraction_threshold_;
  bool require_both_conditions_;
  bool debug_log_;
  
  /**
   * @brief Callback for depth image messages.
   */
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  /**
   * @brief Load parameters from input ports.
   */
  void loadParameters();
  
  /**
   * @brief Compute depth metrics from current ROI.
   * @param[out] valid_count Number of valid depth pixels
   * @param[out] mean_depth Mean depth of valid pixels
   * @param[out] std_depth Standard deviation of depth
   * @param[out] free_space_fraction Fraction of pixels exceeding baseline + delta
   * @return true if computation succeeded, false otherwise
   */
  bool computeDepthMetrics(
    int & valid_count,
    double & mean_depth,
    double & std_depth,
    double & free_space_fraction);
  
  /**
   * @brief Get depth value at pixel position from current image.
   * @param msg The depth image message
   * @param row Pixel row (y)
   * @param col Pixel column (x)
   * @return Depth in meters, or NaN if invalid
   */
  double getDepthAt(const sensor_msgs::msg::Image::SharedPtr & msg, int row, int col);
  
  /**
   * @brief Compute median of a vector.
   */
  static double computeMedian(std::vector<double> values);
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__WAIT_FOR_DOOR_OPEN_DEPTH_ACTION_HPP_
