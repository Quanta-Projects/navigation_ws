#include "smrr_navigation/bt_nodes/get_named_map_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/rclcpp.hpp>
#include <filesystem>

namespace smrr_navigation
{

// Static member initialization
std::map<std::string, YAML::Node> GetNamedMapAction::yaml_cache_;
std::mutex GetNamedMapAction::yaml_cache_mutex_;

GetNamedMapAction::GetNamedMapAction(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
}

std::string GetNamedMapAction::resolveFilePath(const std::string & file_name)
{
  // If absolute path, use as-is
  if (file_name[0] == '/') {
    return file_name;
  }

  // Otherwise, look in package share directory
  try {
    std::string pkg_share = ament_index_cpp::get_package_share_directory("smrr_navigation");
    return pkg_share + "/config/" + file_name;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetNamedMapAction"),
      "Failed to resolve file path for '%s': %s",
      file_name.c_str(), e.what());
    throw;
  }
}

YAML::Node GetNamedMapAction::loadYamlFile(const std::string & file_path)
{
  std::lock_guard<std::mutex> lock(yaml_cache_mutex_);

  // Check cache first
  auto it = yaml_cache_.find(file_path);
  if (it != yaml_cache_.end()) {
    return it->second;
  }

  // Load and cache
  YAML::Node yaml = YAML::LoadFile(file_path);
  yaml_cache_[file_path] = yaml;
  RCLCPP_INFO(
    rclcpp::get_logger("GetNamedMapAction"),
    "Loaded and cached YAML from: %s", file_path.c_str());

  return yaml;
}

BT::NodeStatus GetNamedMapAction::tick()
{
  auto floor_id = getInput<std::string>("floor_id");
  auto map_key = getInput<std::string>("map_key");
  auto locations_file = getInput<std::string>("locations_file").value();

  if (!floor_id || !map_key) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetNamedMapAction"),
      "Missing required inputs: floor_id or map_key");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("GetNamedMapAction"),
    "Looking up map: floor_id='%s', map_key='%s'",
    floor_id.value().c_str(), map_key.value().c_str());

  try {
    // Load YAML file (cached)
    std::string file_path = resolveFilePath(locations_file);
    YAML::Node yaml = loadYamlFile(file_path);

    // Navigate: floors[floor_id]["maps"][map_key]
    YAML::Node maps_node = yaml["floors"][floor_id.value()]["maps"];
    if (!maps_node || !maps_node[map_key.value()]) {
      RCLCPP_ERROR(
        rclcpp::get_logger("GetNamedMapAction"),
        "Map not found: floors['%s']['maps']['%s']",
        floor_id.value().c_str(), map_key.value().c_str());
      return BT::NodeStatus::FAILURE;
    }

    std::string map_yaml = maps_node[map_key.value()].as<std::string>();

    // Resolve map path (assume relative to maps directory)
    std::string pkg_share = ament_index_cpp::get_package_share_directory("smrr_navigation");
    std::string full_map_path = pkg_share + "/maps/" + map_yaml;

    RCLCPP_INFO(
      rclcpp::get_logger("GetNamedMapAction"),
      "Found map: %s -> %s", map_key.value().c_str(), full_map_path.c_str());

    setOutput("map_yaml", full_map_path);
    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetNamedMapAction"),
      "Exception reading map from YAML: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace smrr_navigation
