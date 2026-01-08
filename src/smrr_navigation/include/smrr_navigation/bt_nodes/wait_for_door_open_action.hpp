#ifndef SMRR_NAVIGATION__BT_NODES__WAIT_FOR_DOOR_OPEN_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__WAIT_FOR_DOOR_OPEN_ACTION_HPP_

#include <string>
#include <memory>
#include <cmath>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace smrr_navigation
{

/**
 * @brief BT StatefulActionNode that waits for elevator door to open.
 * 
 * Uses LaserScan data to detect when the door ahead has opened by checking
 * if a sufficient fraction of rays in a forward-facing window exceed a range threshold.
 * 
 * Door is considered open when:
 * - At least 5 valid rays in the detection window
 * - fraction_threshold (default 60%) of rays exceed range_threshold_m (default 2.0m)
 * - This condition is stable for stable_time_sec (default 1.0s)
 */
class WaitForDoorOpenAction : public BT::StatefulActionNode
{
public:
  WaitForDoorOpenAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("scan_topic", "/scan", "LaserScan topic to subscribe to"),
      BT::InputPort<double>("timeout_sec", 30.0, "Timeout for door open detection"),
      BT::InputPort<double>("window_center_deg", 0.0, "Center of detection window (degrees from forward)"),
      BT::InputPort<double>("window_width_deg", 30.0, "Width of detection window in degrees"),
      BT::InputPort<double>("range_threshold_m", 2.0, "Range threshold for 'open' ray"),
      BT::InputPort<double>("fraction_threshold", 0.6, "Fraction of open rays required"),
      BT::InputPort<double>("stable_time_sec", 1.0, "Time door must remain open for success"),
      BT::InputPort<double>("poll_rate_hz", 10.0, "Rate limit for scan evaluation"),
      BT::InputPort<double>("max_scan_stale_sec", 1.0, "Max age for scan data before failure")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  
  // Latest scan data
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_scan_time_;
  bool scan_received_;
  
  // Timing
  rclcpp::Time start_time_;
  rclcpp::Time door_open_start_time_;
  rclcpp::Time last_check_time_;
  bool door_open_timer_active_;
  
  // Parameters (loaded from ports)
  std::string scan_topic_;
  double timeout_sec_;
  double window_center_deg_;
  double window_width_deg_;
  double range_threshold_m_;
  double fraction_threshold_;
  double stable_time_sec_;
  double poll_rate_hz_;
  double max_scan_stale_sec_;
  
  /**
   * @brief Callback for LaserScan messages.
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
  /**
   * @brief Check if door is open based on latest scan.
   * @return true if door appears open, false otherwise
   */
  bool isDoorOpen();
  
  /**
   * @brief Load parameters from input ports.
   */
  void loadParameters();
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__WAIT_FOR_DOOR_OPEN_ACTION_HPP_
