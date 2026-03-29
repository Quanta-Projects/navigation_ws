#include "smrr_navigation/bt_nodes/get_named_pose_action.hpp"
#include <iostream>
#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace smrr_navigation
{

// Static member initialization
std::map<std::string, YAML::Node> GetNamedPoseAction::yaml_cache_;
std::mutex GetNamedPoseAction::cache_mutex_;

std::string GetNamedPoseAction::resolveFilePath(const std::string& locations_file)
{
  // If absolute path, use as-is
  if (std::filesystem::path(locations_file).is_absolute()) {
    return locations_file;
  }
  
  // Otherwise, resolve relative to package share directory
  try {
    std::string pkg_share = ament_index_cpp::get_package_share_directory("smrr_navigation");
    return pkg_share + "/config/" + locations_file;
  } catch (const std::exception& e) {
    std::cerr << "[GetNamedPose] ERROR: Failed to resolve package share directory: " 
              << e.what() << std::endl;
    return "";
  }
}

YAML::Node GetNamedPoseAction::loadYamlFile(const std::string& file_path)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  
  // Check cache first
  auto it = yaml_cache_.find(file_path);
  if (it != yaml_cache_.end()) {
    return it->second;
  }
  
  // Load from file
  try {
    YAML::Node yaml = YAML::LoadFile(file_path);
    yaml_cache_[file_path] = yaml;
    std::cout << "[GetNamedPose] Loaded and cached YAML from: " << file_path << std::endl;
    return yaml;
  } catch (const YAML::Exception& e) {
    std::cerr << "[GetNamedPose] ERROR: Failed to load YAML file '" << file_path 
              << "': " << e.what() << std::endl;
    return YAML::Node();
  }
}

BT::NodeStatus GetNamedPoseAction::tick()
{
  // Read input ports
  auto floor_id_result = getInput<std::string>("floor_id");
  auto location_key_result = getInput<std::string>("location_key");
  auto global_frame_result = getInput<std::string>("global_frame");
  auto locations_file_result = getInput<std::string>("locations_file");

  if (!floor_id_result || !location_key_result) {
    std::cerr << "[GetNamedPose] ERROR: Required inputs (floor_id, location_key) not available" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::string floor_id = floor_id_result.value();
  std::string location_key = location_key_result.value();
  std::string global_frame = global_frame_result.value_or("map");
  std::string locations_file = locations_file_result.value_or("physical_locations.yaml");

  if (floor_id.empty() || location_key.empty()) {
    std::cerr << "[GetNamedPose] ERROR: floor_id or location_key is empty" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  // Resolve file path
  std::string file_path = resolveFilePath(locations_file);
  if (file_path.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  std::cout << "[GetNamedPose] Looking up: floor_id='" << floor_id 
            << "', location_key='" << location_key 
            << "' in file: " << file_path << std::endl;

  // Load YAML (with caching)
  YAML::Node yaml = loadYamlFile(file_path);
  if (!yaml) {
    return BT::NodeStatus::FAILURE;
  }

  // Navigate YAML structure: floors[floor_id]["locations"][location_key]
  try {
    if (!yaml["floors"] || !yaml["floors"][floor_id]) {
      std::cerr << "[GetNamedPose] ERROR: Floor '" << floor_id << "' not found in YAML" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    YAML::Node floor_data = yaml["floors"][floor_id];
    if (!floor_data["locations"] || !floor_data["locations"][location_key]) {
      std::cerr << "[GetNamedPose] ERROR: Location '" << location_key 
                << "' not found for floor '" << floor_id << "'" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    YAML::Node location_data = floor_data["locations"][location_key];
    
    // Extract x, y, yaw
    if (!location_data["x"] || !location_data["y"] || !location_data["yaw"]) {
      std::cerr << "[GetNamedPose] ERROR: Missing x, y, or yaw fields for location '" 
                << location_key << "'" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    double x = location_data["x"].as<double>();
    double y = location_data["y"].as<double>();
    double yaw = location_data["yaw"].as<double>();

    std::cout << "[GetNamedPose] Found pose: x=" << x << ", y=" << y 
              << ", yaw=" << yaw << " rad" << std::endl;

    // Build PoseStamped
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame;
    pose.header.stamp = rclcpp::Clock().now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    // Convert yaw to quaternion
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(quat);

    // Set output port
    setOutput("pose", pose);
    
    std::cout << "[GetNamedPose] SUCCESS: Set output pose for " << location_key << std::endl;
    return BT::NodeStatus::SUCCESS;

  } catch (const YAML::Exception& e) {
    std::cerr << "[GetNamedPose] ERROR: YAML parsing error: " << e.what() << std::endl;
    return BT::NodeStatus::FAILURE;
  } catch (const std::exception& e) {
    std::cerr << "[GetNamedPose] ERROR: " << e.what() << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace smrr_navigation
