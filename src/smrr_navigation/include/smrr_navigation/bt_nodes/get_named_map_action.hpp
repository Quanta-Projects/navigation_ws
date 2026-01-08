#ifndef SMRR_NAVIGATION__BT_NODES__GET_NAMED_MAP_ACTION_HPP_
#define SMRR_NAVIGATION__BT_NODES__GET_NAMED_MAP_ACTION_HPP_

#include <string>
#include <map>
#include <mutex>
#include "behaviortree_cpp_v3/action_node.h"
#include "yaml-cpp/yaml.h"

namespace smrr_navigation
{

/**
 * @brief BT SyncActionNode that retrieves a map file path from locations.yaml
 * 
 * Reads floors[floor_id]["maps"][map_key] and outputs the map YAML path.
 * Uses thread-safe YAML caching like GetNamedPose.
 */
class GetNamedMapAction : public BT::SyncActionNode
{
public:
  GetNamedMapAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("floor_id", "Floor identifier (e.g., 'floor0')"),
      BT::InputPort<std::string>("map_key", "Map key ('open' or 'closed')"),
      BT::InputPort<std::string>("locations_file", "locations.yaml", "YAML file name or path"),
      BT::OutputPort<std::string>("map_yaml", "Output map YAML file path")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::string resolveFilePath(const std::string & file_name);
  YAML::Node loadYamlFile(const std::string & file_path);

  static std::map<std::string, YAML::Node> yaml_cache_;
  static std::mutex yaml_cache_mutex_;
};

}  // namespace smrr_navigation

#endif  // SMRR_NAVIGATION__BT_NODES__GET_NAMED_MAP_ACTION_HPP_
