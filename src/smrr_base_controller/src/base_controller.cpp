#include "smrr_base_controller/base_controller.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>


namespace smrr_base_controller
{
BaseController::BaseController()
: prev_right_encoder_(0), prev_left_encoder_(0), first_read_(true)
{
}


BaseController::~BaseController()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BaseController"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn BaseController::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("BaseController"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.reserve(2);
  position_commands_.reserve(4);
  position_states_.reserve(6);
  velocity_states_.reserve(2);
  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> BaseController::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Arm controller position states are commented.
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     "shoulder_r_joint", hardware_interface::HW_IF_POSITION, &position_states_[0]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     "bicep_r_joint", hardware_interface::HW_IF_POSITION, &position_states_[1]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     "elbow_r_joint", hardware_interface::HW_IF_POSITION, &position_states_[2]));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     "wrist_r_joint", hardware_interface::HW_IF_POSITION, &position_states_[3]));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "left_wheel_joint", hardware_interface::HW_IF_POSITION, &position_states_[4]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "right_wheel_joint", hardware_interface::HW_IF_POSITION, &position_states_[5]));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &velocity_states_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &velocity_states_[1]));


  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> BaseController::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interafce

  // Position commands for the arm joints are commented.
  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //       "shoulder_r_joint", hardware_interface::HW_IF_POSITION, &position_commands_[0]));
  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //       "bicep_r_joint", hardware_interface::HW_IF_POSITION, &position_commands_[1]));
  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //       "elbow_r_joint", hardware_interface::HW_IF_POSITION, &position_commands_[2]));
  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //       "wrist_r_joint", hardware_interface::HW_IF_POSITION, &position_commands_[3]));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &velocity_commands_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &velocity_commands_[1]));
  

  return command_interfaces;
}


CallbackReturn BaseController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Starting robot hardware ...");

  // Reset commands and states
  velocity_commands_ = { 0.0, 0.0};
  position_commands_ = { 0.0, 0.0, 0.0, 0.0};
  position_states_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  velocity_states_ = { 0.0, 0.0};
  
  // Reset encoder tracking
  prev_right_encoder_ = 0;
  prev_left_encoder_ = 0;
  first_read_ = true;

  try
  { 
    // arduino_.SetDTR(false); // Disable DTR
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Delay
    // arduino_.SetDTR(true);  // Enable DTR

    // bool isConnected = arduino_.IsOpen();
    // RCLCPP_INFO(rclcpp::get_logger("BaseController"), 
    //         "Port connection status: %s", isConnected ? "true" : "false");
    // RCLCPP_INFO(rclcpp::get_logger("BaseController"),
    //           "Opening port");
    // arduino_.Open(port_);
    // isConnected = arduino_.IsOpen();
    // RCLCPP_INFO(rclcpp::get_logger("BaseController"), 
    //         "Port connection status: %s", isConnected ? "true" : "false");
    // RCLCPP_INFO(rclcpp::get_logger("BaseController"),
    //           "Port opened");
    // arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    // RCLCPP_INFO(rclcpp::get_logger("BaseController"),
    //           "baudrate assigned");

    
    // Open the port before toggling DTR
    arduino_.Open(port_);
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Serial port opened: %s", port_.c_str());

    // Set baud rate
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Baud rate set to 115200");

    // Reset the Arduino via DTR (toggling DTR)
    arduino_.SetDTR(false);
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), "DTR set to false");

    // Verify the DTR state
    bool dtr_state = arduino_.GetDTR();
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), "DTR state after setting to false: %s", dtr_state ? "true" : "false");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Set DTR to true
    arduino_.SetDTR(true);
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), "DTR set to true");

    // Verify the DTR state again
    dtr_state = arduino_.GetDTR();
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), "DTR state after setting to true: %s", dtr_state ? "true" : "false");

    // Wait for the Arduino to reboot
    std::this_thread::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Arduino reset completed");

 
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("BaseController"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("BaseController"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn BaseController::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BaseController"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

// hardware_interface::return_type BaseController::read(const rclcpp::Time &,
//                                                           const rclcpp::Duration &)
// {
//   if (arduino_.IsOpen())
//   {
//     RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Reading");
//     // Interpret the string
//     if(arduino_.IsDataAvailable())
//     {
//       auto dt = (rclcpp::Clock().now() - last_run_).seconds();
//       std::string message;
//       arduino_.ReadLine(message);
//       std::stringstream ss(message);
//       std::string res;
//       int multiplier = 1;
//       while(std::getline(ss, res, ','))
//       {
//         multiplier = res.at(1) == 'p' ? 1 : -1;

//         if(res.at(0) == 'r')
//         {
//           velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
//           position_states_.at(0) += velocity_states_.at(0) * dt;
//         }
//         else if(res.at(0) == 'l')
//         {
//           velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
//           position_states_.at(1) += velocity_states_.at(1) * dt;
//         }
//       }
//       last_run_ = rclcpp::Clock().now();
//     }
//   }
  
//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type BaseController::read(const rclcpp::Time &,
//                                                      const rclcpp::Duration &)
// {
//   if (arduino_.IsOpen())
//   {
//     //RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Reading");
//     // Check if data is available
//     if (arduino_.IsDataAvailable())
//     { 
//       RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Data available");
//       auto dt = (rclcpp::Clock().now() - last_run_).seconds();
//       std::string message;
//       arduino_.ReadLine(message);

//       // Print the raw message received from the serial port
//       RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Received message: '%s'", message.c_str());

//       // std::stringstream ss(message);
//       // std::string res;
//       // int multiplier = 1;

//       // while (std::getline(ss, res, ','))
//       // {
//       //   multiplier = res.at(1) == 'p' ? 1 : -1;

//       //   if (res.at(0) == 'r')
//       //   {
//       //     velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size()));
//       //     position_states_.at(0) += velocity_states_.at(0) * dt;
//       //   }
//       //   else if (res.at(0) == 'l')
//       //   {
//       //     velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));
//       //     position_states_.at(1) += velocity_states_.at(1) * dt;
//       //   }
//       // }

//       last_run_ = rclcpp::Clock().now();
//     }
//     else
//     {
//       RCLCPP_INFO(rclcpp::get_logger("BaseController"), "No data available");
//     }
//   }
//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type BaseController::read(const rclcpp::Time &,
//                                                      const rclcpp::Duration &) {
//   if (arduino_.IsOpen()) {
//     if (arduino_.IsDataAvailable()) {
//       std::string feedback;
//       arduino_.ReadLine(feedback);
//       RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Arduino Feedback: %s", feedback.c_str());

//       // Parse and use the feedback (e.g., updating state variables)
//       std::stringstream ss(feedback);
//       std::string item;
//       while (std::getline(ss, item, ',')) {
//         auto key_val = item.find(':');
//         if (key_val != std::string::npos) {
//           std::string key = item.substr(0, key_val);
//           double value = std::stod(item.substr(key_val + 1));

//           if (key == "pos") {
//             position_states_[0] = value; // Example usage
//           } else if (key == "vel") {
//             velocity_states_[0] = value; // Example usage
//           }
//         }
//       }
//     }
//   }
//   return hardware_interface::return_type::OK;
// }

hardware_interface::return_type BaseController::read(const rclcpp::Time &,
                                                     const rclcpp::Duration &) {
  if (arduino_.IsOpen()) {
    try {
      std::string feedback;
      arduino_.ReadLine(feedback, '\n', 10); // 10ms timeout

      // Skip empty messages
      if (feedback.empty()) {
        return hardware_interface::return_type::OK;
      }

      // Print the received message
      RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Received Message: %s", feedback.c_str());

      // Calculate time difference
      auto current_time = rclcpp::Clock().now();
      double dt = (current_time - last_run_).seconds();

      // Parse the feedback: "right_encoder,left_encoder,value3,value4"
      std::stringstream ss(feedback);
      std::string token;
      std::vector<int32_t> values;
      
      // Parse comma-separated values
      while (std::getline(ss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t\r\n"));
        token.erase(token.find_last_not_of(" \t\r\n") + 1);
        
        if (!token.empty()) {
          values.push_back(std::stoi(token));
        }
      }
      
      // Process encoder counts (expecting at least 2 values: right_encoder, left_encoder)
      if (values.size() >= 2) {
        int32_t right_encoder = values[0];  // First value is right encoder
        int32_t left_encoder = values[1];   // Second value is left encoder
        
        // Encoder specifications
        const double ENCODER_RESOLUTION = 60.0;  // 60 counts per rotation
        const double WHEEL_DIAMETER = 0.1524;    // 6 inches = 0.1524 meters
        const double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;
        const int32_t MAX_ENCODER_COUNT = 65535; // 16-bit counter
        
        // On first read, just store the encoder values
        if (first_read_) {
          prev_right_encoder_ = right_encoder;
          prev_left_encoder_ = left_encoder;
          first_read_ = false;
          last_run_ = current_time;
          
          RCLCPP_INFO(rclcpp::get_logger("BaseController"), 
                     "First encoder read - Right: %d, Left: %d", right_encoder, left_encoder);
          return hardware_interface::return_type::OK;
        }
        
        // Calculate encoder deltas (handle wraparound)
        int32_t right_delta = right_encoder - prev_right_encoder_;
        int32_t left_delta = left_encoder - prev_left_encoder_;
        
        // Handle wraparound for 16-bit counter
        if (right_delta > MAX_ENCODER_COUNT / 2) {
          right_delta -= MAX_ENCODER_COUNT;
        } else if (right_delta < -MAX_ENCODER_COUNT / 2) {
          right_delta += MAX_ENCODER_COUNT;
        }
        
        // Invert right encoder since it's mounted in opposite direction
        right_delta = -right_delta;
        
        if (left_delta > MAX_ENCODER_COUNT / 2) {
          left_delta -= MAX_ENCODER_COUNT;
        } else if (left_delta < -MAX_ENCODER_COUNT / 2) {
          left_delta += MAX_ENCODER_COUNT;
        }
        
        // Convert encoder counts to radians
        // radians = (encoder_counts / ENCODER_RESOLUTION) * 2π
        double right_wheel_delta_rad = (static_cast<double>(right_delta) / ENCODER_RESOLUTION) * 2.0 * M_PI;
        double left_wheel_delta_rad = (static_cast<double>(left_delta) / ENCODER_RESOLUTION) * 2.0 * M_PI;
        
        // Update position states (accumulated angular position in radians)
        position_states_[5] += right_wheel_delta_rad;  // right_wheel_joint position
        position_states_[4] += left_wheel_delta_rad;   // left_wheel_joint position
        
        // Calculate velocities (rad/s)
        if (dt > 0.0 && dt < 1.0) {  // Sanity check on dt
          velocity_states_[1] = right_wheel_delta_rad / dt;  // right_wheel_joint velocity
          velocity_states_[0] = left_wheel_delta_rad / dt;   // left_wheel_joint velocity
        }
        
        // Store current encoder values for next iteration
        prev_right_encoder_ = right_encoder;
        prev_left_encoder_ = left_encoder;
        
        RCLCPP_INFO(rclcpp::get_logger("BaseController"), 
                   "Encoders - R:%d(Δ%d) L:%d(Δ%d) | Pos - R:%.3f L:%.3f | Vel - R:%.3f L:%.3f",
                   right_encoder, right_delta, left_encoder, left_delta,
                   position_states_[5], position_states_[4],
                   velocity_states_[1], velocity_states_[0]);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("BaseController"), 
                   "Unexpected data format. Expected at least 2 values, got %zu", values.size());
      }
      
      last_run_ = current_time;
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("BaseController"), 
                  "Error reading/parsing feedback: %s", e.what());
    }
  }
  return hardware_interface::return_type::OK;
}




hardware_interface::return_type BaseController::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  if (arduino_.IsOpen())
  {
    RCLCPP_INFO(rclcpp::get_logger("BaseController"), "Writing");
    // Implement communication protocol with the Arduino
    
    // Map velocity commands from [-6.17, 6.17] to [-100, 100]
    const double SCALE_FACTOR = 100.0 / 6.17;
    double scaled_left = velocity_commands_.at(0) * SCALE_FACTOR;
    double scaled_right = velocity_commands_.at(1) * SCALE_FACTOR;
    
    // Clamp values to [-100, 100] range
    scaled_left = std::max(-100.0, std::min(100.0, scaled_left));
    scaled_right = std::max(-100.0, std::min(100.0, scaled_right));
    
    std::stringstream message_stream;
    message_stream << std::fixed << std::setprecision(2) 
      << scaled_left << "," << scaled_right << ",0.0,0.0,0.0,0.0\n";

    try
    {
      arduino_.Write(message_stream.str());

      RCLCPP_INFO_STREAM(rclcpp::get_logger("BaseController"), "Message sent: " << message_stream.str());
    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("BaseController"),
                          "Something went wrong while sending the message "
                              << message_stream.str() << " to the port " << port_);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}
}  // namespace base_controller
PLUGINLIB_EXPORT_CLASS(smrr_base_controller::BaseController, hardware_interface::SystemInterface)