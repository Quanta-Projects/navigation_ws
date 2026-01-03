#ifndef SMRR_NAVIGATION__BT_NODES__GET_NAMED_POSE_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__GET_NAMED_POSE_ACTION_HPP_

#include <string>
#include <map>
#include <memory>
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "yaml-cpp/yaml.h"

namespace smrr_navigation
{

/**
 * @brief BT Action node that retrieves a named pose from locations.yaml
 * 
 * Loads the locations.yaml file (cached) and looks up a pose by floor_id and location_key.
 * Outputs a geometry_msgs::msg::PoseStamped for use by NavigateToPose.
 * 
 * Input Ports:
 *   - floor_id: Floor identifier (e.g., "floor0")
 *   - location_key: Location name (e.g., "elevator_staging")
 *   - global_frame: Frame ID for pose (default: "map")
 *   - locations_file: YAML file name or path (default: "locations.yaml")
 * 
 * Output Ports:
 *   - pose: geometry_msgs::msg::PoseStamped with the resolved location
 */
class GetNamedPoseAction : public BT::SyncActionNode
{
public:
  GetNamedPoseAction(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor_id", "Floor identifier"),
      BT::InputPort<std::string>("location_key", "Location name key"),
      BT::InputPort<std::string>("global_frame", "map", "Frame ID for pose"),
      BT::InputPort<std::string>("locations_file", "locations.yaml", "YAML file name or path"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Output pose")
    };
  }

  BT::NodeStatus tick() override;

private:
  // Cached YAML data to avoid reloading on every tick
  static std::map<std::string, YAML::Node> yaml_cache_;
  static std::mutex cache_mutex_;

  /**
   * @brief Load YAML file with caching
   * @param file_path Resolved file path
   * @return YAML::Node or empty node on failure
   */
  YAML::Node loadYamlFile(const std::string& file_path);
  
  /**
   * @brief Resolve file path (absolute or relative to package share)
   * @param locations_file File name or path from input port
   * @return Absolute file path
   */
  std::string resolveFilePath(const std::string& locations_file);
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__GET_NAMED_POSE_ACTION_HPP_
