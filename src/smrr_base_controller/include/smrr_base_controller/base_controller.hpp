#ifndef BASE_CONTROLLER_HPP
#define BASE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>


namespace smrr_base_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BaseController : public hardware_interface::SystemInterface
{
public:
  BaseController();
  virtual ~BaseController();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // Implementing hardware_interface::SystemInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  LibSerial::SerialPort arduino_;
  std::string port_;
  std::vector<double> velocity_commands_;
  std::vector<double> position_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  rclcpp::Time last_run_;
  
  // Encoder tracking variables
  int32_t prev_right_encoder_;
  int32_t prev_left_encoder_;
  bool first_read_;

  // Battery/docking status publishing
  rclcpp::Node::SharedPtr charging_node_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr battery_timer_;          // heartbeat: publishes at 1 Hz
  sensor_msgs::msg::BatteryState last_battery_state_;   // last known state (default: disconnected)
  // Executor + thread to spin charging_node_ so it is visible in the ROS2 graph
  rclcpp::executors::SingleThreadedExecutor::SharedPtr charging_executor_;
  std::thread charging_exec_thread_;

  // Charging command state:
  // charge_command_ is enabled only after DOCK_STABLE_COUNT consecutive
  // is_connected=1 signals, ensuring the physical contact is stable.
  // Cleared immediately on any is_connected=0 signal.
  static constexpr int DOCK_STABLE_COUNT = 5;  // ~500 ms at 100 ms firmware interval
  bool charge_command_;
  int  dock_connected_count_;   // consecutive is_connected=1 signals received
};
}  

#endif  