#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "smrr_interfaces/srv/start_mission.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

namespace smrr_navigation
{

class BtMissionExecutor : public rclcpp::Node
{
public:
  BtMissionExecutor()
  : Node("smrr_bt_mission_executor")
  {
    // Declare parameters
    this->declare_parameter("bt_xml_path", "");
    this->declare_parameter("plugin_lib_names", std::vector<std::string>{});
    this->declare_parameter("bt_tick_rate_hz", 20.0);
    this->declare_parameter("bt_timeout_sec", 300.0);
    this->declare_parameter("locations_file", "locations.yaml");

    // Get parameters
    bt_tick_rate_hz_ = this->get_parameter("bt_tick_rate_hz").as_double();
    bt_timeout_sec_ = this->get_parameter("bt_timeout_sec").as_double();

    std::string bt_xml_path = this->get_parameter("bt_xml_path").as_string();
    if (bt_xml_path.empty()) {
      // Use default path in package share directory
      try {
        std::string pkg_share = ament_index_cpp::get_package_share_directory("smrr_navigation");
        bt_xml_path = pkg_share + "/behavior_trees/smrr_multifloor.xml";
        RCLCPP_INFO(this->get_logger(), "Using default BT XML path: %s", bt_xml_path.c_str());
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to find package share directory: %s", e.what());
        throw;
      }
    }
    bt_xml_path_ = bt_xml_path;

    plugin_lib_names_ = this->get_parameter("plugin_lib_names").as_string_array();
    
    // Create a reentrant callback group to allow nested service calls
    // This allows BT nodes to make service calls while we're in the StartMission callback
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    // Create service with the reentrant callback group
    service_ = this->create_service<smrr_interfaces::srv::StartMission>(
      "/start_mission",
      std::bind(&BtMissionExecutor::handleStartMission, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    RCLCPP_INFO(this->get_logger(), "BT Mission Executor initialized");
    RCLCPP_INFO(this->get_logger(), "BT XML: %s", bt_xml_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Tick rate: %.1f Hz, Timeout: %.1f sec", bt_tick_rate_hz_, bt_timeout_sec_);
  }

private:
  void handleStartMission(
    const std::shared_ptr<smrr_interfaces::srv::StartMission::Request> request,
    std::shared_ptr<smrr_interfaces::srv::StartMission::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received StartMission request: mission_id=%s", 
                request->mission_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  Location: %s", request->target_location_name.c_str());
    RCLCPP_INFO(this->get_logger(), "  Floors: %s -> %s", 
                request->current_floor_id.c_str(), request->target_floor_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  Target pose: x=%.2f, y=%.2f, yaw=%.2f", 
                request->x, request->y, request->yaw);

    // Validate request
    if (request->current_floor_id.empty() || request->target_floor_id.empty()) {
      response->accepted = false;
      response->success = false;
      response->nav_status = 0;
      response->message = "mission_id=" + request->mission_id + " REJECTED: floor IDs cannot be empty";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    if (!std::isfinite(request->x) || !std::isfinite(request->y) || !std::isfinite(request->yaw)) {
      response->accepted = false;
      response->success = false;
      response->nav_status = 0;
      response->message = "mission_id=" + request->mission_id + " REJECTED: invalid pose values";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    // Request is valid - we will run the BT
    response->accepted = true;

    // Create PoseStamped from x, y, yaw
    geometry_msgs::msg::PoseStamped final_pose;
    final_pose.header.frame_id = "map";
    final_pose.header.stamp = this->now();
    final_pose.pose.position.x = request->x;
    final_pose.pose.position.y = request->y;
    final_pose.pose.position.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, request->yaw);
    final_pose.pose.orientation = tf2::toMsg(quat);

    // Create BT engine and load tree
    try {
      RCLCPP_INFO(this->get_logger(), "Creating BT engine and loading tree from: %s", bt_xml_path_.c_str());
      
      nav2_behavior_tree::BehaviorTreeEngine bt_engine(plugin_lib_names_);
      auto blackboard = BT::Blackboard::create();

      // Set ROS node in blackboard (required by Nav2 BT nodes)
      blackboard->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
      
      // Set server timeout for Nav2 action nodes
      blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(2000));
      blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
      blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", std::chrono::milliseconds(1000));

      // Set blackboard values
      blackboard->set("current_floor_id", request->current_floor_id);
      blackboard->set("target_floor_id", request->target_floor_id);
      blackboard->set("final_pose", final_pose);
      blackboard->set("locations_file", this->get_parameter("locations_file").as_string());

      RCLCPP_INFO(this->get_logger(), "Blackboard set: current_floor_id=%s, target_floor_id=%s", 
                  request->current_floor_id.c_str(), request->target_floor_id.c_str());

      // Create tree from XML
      auto tree = bt_engine.createTreeFromFile(bt_xml_path_, blackboard);

      RCLCPP_INFO(this->get_logger(), "BT created successfully, starting execution...");

      // Execute BT with timeout
      auto start_time = std::chrono::steady_clock::now();
      BT::NodeStatus status = BT::NodeStatus::RUNNING;
      
      while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        // Check timeout
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        auto elapsed_sec = std::chrono::duration<double>(elapsed).count();
        
        if (elapsed_sec > bt_timeout_sec_) {
          RCLCPP_ERROR(this->get_logger(), "BT execution timeout after %.1f seconds", elapsed_sec);
          response->success = false;
          response->nav_status = 0;
          response->message = "mission_id=" + request->mission_id + " FAILED: BT timeout";
          return;
        }

        // Tick the tree
        status = tree.tickRoot();
        
        if (status == BT::NodeStatus::RUNNING) {
          // Sleep according to tick rate
          std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(1000.0 / bt_tick_rate_hz_)));
        }
      }

      // Process result
      if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "BT execution SUCCESS");
        response->success = true;
        response->nav_status = 0;
        response->message = "mission_id=" + request->mission_id + " SUCCESS: BT completed";
      } else {
        RCLCPP_WARN(this->get_logger(), "BT execution FAILURE (status=%d)", static_cast<int>(status));
        response->success = false;
        response->nav_status = 0;
        
        // Provide more specific message if floors differ
        if (request->current_floor_id != request->target_floor_id) {
          response->message = "mission_id=" + request->mission_id + 
                            " FAILED: Cross-floor navigation not yet implemented (current=" +
                            request->current_floor_id + ", target=" + request->target_floor_id + ")";
        } else {
          response->message = "mission_id=" + request->mission_id + " FAILED: BT returned FAILURE";
        }
      }

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during BT execution: %s", e.what());
      response->accepted = true;  // We tried to run it
      response->success = false;
      response->nav_status = 0;
      response->message = "mission_id=" + request->mission_id + 
                         " FAILED: Exception - " + std::string(e.what());
    }
  }

  rclcpp::Service<smrr_interfaces::srv::StartMission>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::string bt_xml_path_;
  std::vector<std::string> plugin_lib_names_;
  double bt_tick_rate_hz_;
  double bt_timeout_sec_;
};

}  // namespace smrr_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smrr_navigation::BtMissionExecutor>();
  
  // Use MultiThreadedExecutor to support reentrant callbacks
  // This allows BT nodes to make service calls while the StartMission service callback is executing
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
