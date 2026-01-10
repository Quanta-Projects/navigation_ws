# BehaviorTree-Based Navigation System Implementation

## Overview

This document describes the complete implementation of a BehaviorTree (BT) based navigation system for the SMRR robot. The system replaces the previous Python state machine approach with a modular, XML-configurable BT architecture that integrates with Nav2 for both same-floor and cross-floor navigation tasks.

**Implementation Date:** January 3, 2026  
**ROS 2 Distribution:** Humble  
**BehaviorTree Library:** BehaviorTree.CPP v3  
**Current Milestone:** ✅ **Cross-floor with elevator call/door-open/entry + map switching both floors**

---

## Architecture

### High-Level Design

```
┌─────────────────┐
│  /location      │  (std_msgs/String topic)
│  Topic          │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────┐
│  location_subscriber.py                                 │
│  - Subscribes to /location topic                       │
│  - Calls /go_to_pose service                           │
└────────┬────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────┐
│  named_goal_server.py                                   │
│  - Resolves location name → (floor_id, x, y, yaw)      │
│  - Calls /start_mission service                        │
└────────┬────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────┐
│  smrr_bt_mission_executor (C++)                        │
│  - MultiThreadedExecutor with Reentrant callbacks      │
│  - Loads BehaviorTree from XML                         │
│  - Sets blackboard variables                           │
│  - Executes BT with Nav2 integration                   │
└────────┬────────────────────────────────────────────────┘
         │
         ▼
┌───────────────────────────────────────────────────────────┐
│  smrr_multifloor.xml (BehaviorTree)                       │
│  Fallback (Root)                                          │
│  ├─ Sequence (SameFloor)                                  │
│  │  ├─ IsSameFloor (Custom Condition)                     │
│  │  └─ NavigateToPose (Nav2 Action)                       │
│  └─ Sequence (CrossFloor_ElevatorEntry)                   │
│     ├─ IsDifferentFloor (Custom Condition)                │
│     ├─ GetNamedPose(elevator_staging) → NavigateToPose    │
│     ├─ GetNamedMap(open@current) → SwitchMap →            │
│     │  PublishInitialPose → ClearCostmaps                 │
│     ├─ CallElevator(current) → WaitForDoorOpen            │
│     ├─ GetNamedPose(elevator_inside) → NavigateToPose →   │
│     │  Spin                                               │
│     ├─ GetNamedMap(open@target) → SwitchMap →             │
│     │  PublishInitialPose → ClearCostmaps                 │
│     ├─ CallElevator(target) → WaitForDoorOpen             │
│     ├─ GetNamedPose(elevator_exit) → NavigateToPose       │
│     ├─ GetNamedMap(closed@target) → SwitchMap →           │
│     │  PublishInitialPose → ClearCostmaps                 │
│     └─ NavigateToPose(final_pose)                         │
└───────────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Separation of Concerns**: Named location resolution happens in Python layer; navigation control logic in BT
2. **Service-Based Architecture**: Clean interfaces between components using ROS 2 services
3. **BT-Driven Logic**: All navigation decision-making happens inside the BehaviorTree
4. **No Rejection Logic**: Service layer accepts all requests; BT decides whether to proceed
5. **Nav2 Integration**: Leverages Nav2's NavigateToPose action for actual path planning and control
6. **YAML-Based Configuration**: Location poses and map paths loaded from config files by BT nodes
7. **Executor Simplicity**: Mission executor remains a pure BT runner with no control logic
8. **Reentrant Callbacks**: MultiThreadedExecutor with reentrant callback group allows nested service calls
9. **Async BT Pattern**: StatefulActionNode for long-running operations (map switching)

---

## Components

### 1. StartMission Service Interface

**Package:** `smrr_interfaces`  
**File:** `srv/StartMission.srv`

```
# Request
string mission_id              # Unique identifier for this mission
string current_floor_id        # Current floor ID (e.g., "floor0")
string target_floor_id         # Target floor ID
string target_location_name    # Named location (e.g., "dock")
float64 x                      # Target X coordinate
float64 y                      # Target Y coordinate
float64 yaw                    # Target orientation (radians)
---
# Response
bool accepted                  # Always true (service layer doesn't reject)
bool success                   # True if BT execution succeeded
int32 nav_status              # Navigation status code
string message                # Human-readable status message
```

**Purpose:** Unified interface for requesting navigation missions, whether same-floor or multi-floor.

### 2. BT Mission Executor (C++)

**Package:** `smrr_navigation`  
**File:** `src/smrr_bt_mission_executor.cpp`  
**Node Name:** `/smrr_bt_mission_executor`  
**Service:** `/start_mission` (smrr_interfaces/srv/StartMission)  
**Executor Type:** `rclcpp::executors::MultiThreadedExecutor`  
**Callback Group:** `Reentrant` (allows nested service calls)

**Responsibilities:**
- Listen for StartMission service requests
- Create and configure BehaviorTree blackboard with mission parameters
- Load BehaviorTree from XML file
- Execute BT with configurable tick rate
- Support async service calls from BT nodes via reentrant callbacks
- Monitor BT execution status (SUCCESS/FAILURE/RUNNING)
- Handle timeouts and exceptions
- Return mission result to caller

**Key Parameters:**
```yaml
bt_xml_path: "/path/to/smrr_multifloor.xml"  # BT XML file path
plugin_lib_names:                             # List of BT plugin libraries
  - nav2_compute_path_to_pose_action_bt_node
  - nav2_navigate_to_pose_action_bt_node
  - nav2_clear_costmap_service_bt_node         # For ClearEntireCostmap
  - smrr_bt_nodes                              # Custom plugin library
bt_tick_rate_hz: 20.0                          # BT tick frequency
bt_timeout_sec: 300.0                          # Maximum execution time
```

**Blackboard Entries Set by Executor:**
```cpp
node                    // rclcpp::Node::SharedPtr - ROS 2 node for Nav2 actions
server_timeout          // std::chrono::milliseconds(10)
bt_loop_duration        // std::chrono::milliseconds(10)
wait_for_service_timeout // std::chrono::milliseconds(1000)
current_floor_id        // string - From service request
target_floor_id         // string - From service request
final_pose              // geometry_msgs::msg::PoseStamped - Target pose
```

**Critical Implementation Pattern - Reentrant Callbacks:**

The executor uses a MultiThreadedExecutor with a reentrant callback group to allow BT nodes to make service calls while the StartMission service callback is executing:

```cpp
// In constructor:
callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

service_ = this->create_service<smrr_interfaces::srv::StartMission>(
  "/start_mission",
  std::bind(&BtMissionExecutor::handleStartMission, this, ...),
  rmw_qos_profile_services_default,
  callback_group_);

// In main():
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

**Why This Pattern?**
- The StartMission callback is blocking (running BT loop)
- BT nodes (like SwitchMap) make async service calls and wait for responses
- Without reentrant callbacks, service responses can't be processed → deadlock
- MultiThreadedExecutor + Reentrant allows other threads to process callbacks
- This enables nested service calls without "already added to executor" errors

### 3. Custom BT Condition Node: IsSameFloor

**Package:** `smrr_navigation`  
**Files:**
- Header: `include/smrr_navigation/bt_nodes/is_same_floor_condition.hpp`
- Source: `src/bt_nodes/is_same_floor_condition.cpp`
- Library: `libsmrr_bt_nodes.so`

**BT Node Name:** `IsSameFloor`  
**Type:** Condition Node

**Purpose:** Checks if the current floor and target floor are the same.

**Input Ports:**
- `current_floor` (string) - Current floor identifier
- `target_floor` (string) - Target floor identifier

**Return Values:**
- `SUCCESS` - Floors match (same-floor navigation allowed)
- `FAILURE` - Floors differ (cross-floor navigation required)

**Implementation:**
```cpp
BT::NodeStatus IsSameFloorCondition::tick()
{
  auto current_floor = getInput<std::string>("current_floor").value();
  auto target_floor = getInput<std::string>("target_floor").value();
  
  if (current_floor == target_floor) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}
```

### 4. Custom BT Condition Node: IsDifferentFloor

**Package:** `smrr_navigation`  
**Files:**
- Header: `include/smrr_navigation/bt_nodes/is_different_floor_condition.hpp`
- Source: `src/bt_nodes/is_different_floor_condition.cpp`
- Library: `libsmrr_bt_nodes.so`

**BT Node Name:** `IsDifferentFloor`  
**Type:** Condition Node

**Purpose:** Checks if the current floor and target floor are different (cross-floor navigation required).

**Input Ports:**
- `current_floor` (string) - Current floor identifier
- `target_floor` (string) - Target floor identifier

**Return Values:**
- `SUCCESS` - Floors differ (cross-floor navigation required)
- `FAILURE` - Floors match or inputs invalid

**Implementation:**
```cpp
BT::NodeStatus IsDifferentFloorCondition::tick()
{
  auto current_floor = getInput<std::string>("current_floor").value();
  auto target_floor = getInput<std::string>("target_floor").value();
  
  if (current_floor != target_floor) {
    return BT::NodeStatus::SUCCESS;  // Different floors
  } else {
    return BT::NodeStatus::FAILURE;  // Same floor
  }
}
```

### 5. Custom BT Action Node: GetNamedPose

**Package:** `smrr_navigation`  
**Files:**
- Header: `include/smrr_navigation/bt_nodes/get_named_pose_action.hpp`
- Source: `src/bt_nodes/get_named_pose_action.cpp`
- Library: `libsmrr_bt_nodes.so`

**BT Node Name:** `GetNamedPose`  
**Type:** SyncActionNode

**Purpose:** Retrieves a named pose from locations.yaml configuration file.

**Input Ports:**
- `floor_id` (string) - Floor identifier (e.g., "floor0")
- `location_key` (string) - Location name (e.g., "elevator_staging")
- `global_frame` (string, optional) - Frame ID for pose (default: "map")
- `locations_file` (string, optional) - YAML file name or path (default: "locations.yaml")

**Output Ports:**
- `pose` (geometry_msgs::msg::PoseStamped) - Resolved pose with header and orientation

**Features:**
- **YAML Caching**: Loads and parses YAML file once, caches for subsequent calls (thread-safe)
- **Path Resolution**: Automatically resolves relative paths using package share directory
- **TF2 Integration**: Converts yaw to quaternion for ROS 2 pose representation
- **AMCL Pose Support**: Can retrieve AMCL initial poses (e.g., location_key="amcl_initial_pose_closed")
- **Unified Interface**: Single action for all pose retrieval (navigation goals, staging, initialization)

**Implementation Details:**
```cpp
BT::NodeStatus GetNamedPoseAction::tick()
{
  // Load YAML from config/locations.yaml (cached)
  YAML::Node yaml = loadYamlFile(resolveFilePath(locations_file));
  
  // Navigate: floors[floor_id]["locations"][location_key]
  YAML::Node location_data = yaml["floors"][floor_id]["locations"][location_key];
  
  double x = location_data["x"].as<double>();
  double y = location_data["y"].as<double>();
  double yaw = location_data["yaw"].as<double>();
  
  // Build PoseStamped with quaternion from yaw
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = global_frame;
  pose.header.stamp = rclcpp::Clock().now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(quat);
  
  setOutput("pose", pose);
  return BT::NodeStatus::SUCCESS;
}
```

**Plugin Registration:**
```cpp
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<smrr_navigation::IsSameFloorCondition>("IsSameFloor");
  factory.registerNodeType<smrr_navigation::IsDifferentFloorCondition>("IsDifferentFloor");
  factory.registerNodeType<smrr_navigation::GetNamedPoseAction>("GetNamedPose");
  factory.registerNodeType<smrr_navigation::GetNamedMapAction>("GetNamedMap");
  factory.registerNodeType<smrr_navigation::SwitchMapAction>("SwitchMap");
  factory.registerNodeType<smrr_navigation::PublishInitialPoseAction>("PublishInitialPose");
}
```

### 6. Custom BT Action Node: GetNamedMap

**Package:** `smrr_navigation`  
**Files:**
- Header: `include/smrr_navigation/bt_nodes/get_named_map_action.hpp`
- Source: `src/bt_nodes/get_named_map_action.cpp`
- Library: `libsmrr_bt_nodes.so`

**BT Node Name:** `GetNamedMap`  
**Type:** SyncActionNode

**Purpose:** Retrieves a map file path from locations.yaml configuration file.

**Input Ports:**
- `floor_id` (string) - Floor identifier (e.g., "floor0")
- `map_key` (string) - Map name (e.g., "open", "closed")
- `locations_file` (string, optional) - YAML file name or path (default: "locations.yaml")

**Output Ports:**
- `map_yaml` (string) - Full path to map YAML file

**Features:**
- **YAML Caching**: Same caching mechanism as GetNamedPose
- **Path Resolution**: Converts relative paths to absolute using package share directory
- **Multiple Map Support**: Supports different map configurations per floor (open/closed doors, etc.)

**YAML Structure:**
```yaml
floors:
  floor0:
    maps:
      closed: first_floor_with_docking_station.yaml
      open: floor0_open.yaml
```

**Usage in BT:**
```xml
<GetNamedMap floor_id="{current_floor_id}" map_key="open" map_yaml="{map_open_current}"/>
```

### 7. Custom BT Action Node: SwitchMap

**Package:** `smrr_navigation`  
**Files:**
- Header: `include/smrr_navigation/bt_nodes/switch_map_action.hpp`
- Source: `src/bt_nodes/switch_map_action.cpp`
- Library: `libsmrr_bt_nodes.so`

**BT Node Name:** `SwitchMap`  
**Type:** StatefulActionNode (async pattern)

**Purpose:** Calls Nav2 LoadMap service to switch active map (triggers AMCL and costmap updates).

**Input Ports:**
- `map_yaml` (string) - Full path to map YAML file
- `service_name` (string, optional) - LoadMap service name (default: "/map_server/load_map")
- `timeout_ms` (int, optional) - Service call timeout in milliseconds (default: 10000)

**Service Used:** `nav2_msgs/srv/LoadMap`

**Features:**
- **Async Pattern**: Uses StatefulActionNode with onStart/onRunning/onHalted lifecycle
- **Non-Blocking**: Returns RUNNING while waiting for service response
- **SharedFuture**: Uses `.future.share()` for proper future handling
- **Error Handling**: Checks service availability, validates responses, handles timeouts
- **Triggers Cascade**: Map load → AMCL receives new map → Costmaps resize automatically

**Implementation Pattern:**
```cpp
class SwitchMapAction : public BT::StatefulActionNode
{
  BT::NodeStatus onStart() override {
    // Send async service request
    load_map_future_ = load_map_client_->async_send_request(request).future.share();
    request_sent_ = true;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    // Check if future is ready (non-blocking)
    if (load_map_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      auto response = load_map_future_.get();
      if (response->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
        return BT::NodeStatus::SUCCESS;
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }
    
    // Check timeout
    if (elapsed_time > timeout_) {
      return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::RUNNING;  // Still waiting
  }

  void onHalted() override {
    request_sent_ = false;
  }
};
```

**Why StatefulActionNode?**
- Service calls take time (1-5 seconds for map loading + AMCL + costmap resize)
- SyncActionNode would block the entire BT tick
- StatefulActionNode returns RUNNING, allowing BT to continue ticking
- Enables MultiThreadedExecutor to process service response callbacks

### 8. Custom BT Action Node: PublishInitialPose

**Package:** `smrr_navigation`  
**Files:**
- Header: `include/smrr_navigation/bt_nodes/publish_initial_pose_action.hpp`
- Source: `src/bt_nodes/publish_initial_pose_action.cpp`
- Library: `libsmrr_bt_nodes.so`

**BT Node Name:** `PublishInitialPose`  
**Type:** SyncActionNode

**Purpose:** Publishes initial pose for AMCL relocalization after map switch.

**Input Ports:**
- `initial_pose` (geometry_msgs::msg::PoseStamped) - Pose to publish
- `topic_name` (string, optional) - Topic name (default: "/initialpose")
- `frame_id` (string, optional) - Override frame_id (default: use pose's frame_id or "map")

**Message Type:** `geometry_msgs/msg/PoseWithCovarianceStamped`

**Features:**
- **Covariance Defaults**: Sets reasonable uncertainty values (0.5m position, 15° orientation)
- **Frame Override**: Can override pose frame_id if needed
- **Safe Input Handling**: Properly checks optional inputs with `.has_value()` instead of `.value()`
- **AMCL Integration**: AMCL subscribes to /initialpose to reset particle filter

**Implementation:**
```cpp
BT::NodeStatus PublishInitialPoseAction::tick()
{
  auto initial_pose = getInput<geometry_msgs::msg::PoseStamped>("initial_pose");
  
  // Safe handling of optional inputs with defaults
  std::string topic_name = "/initialpose";
  std::string frame_id_override = "";
  
  auto topic_input = getInput<std::string>("topic_name");
  if (topic_input.has_value()) {
    topic_name = topic_input.value();
  }
  
  auto frame_input = getInput<std::string>("frame_id");
  if (frame_input.has_value()) {
    frame_id_override = frame_input.value();
  }

  // Build PoseWithCovarianceStamped
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = node_->get_clock()->now();
  pose_msg.header.frame_id = frame_id_override.empty() ? 
    (initial_pose.value().header.frame_id.empty() ? "map" : initial_pose.value().header.frame_id) : 
    frame_id_override;
  pose_msg.pose.pose = initial_pose.value().pose;
  
  // Set covariance (0.25 m² position, 0.0685 rad² orientation)
  pose_msg.pose.covariance[0] = 0.25;   // x
  pose_msg.pose.covariance[7] = 0.25;   // y
  pose_msg.pose.covariance[35] = 0.0685; // yaw
  
  initial_pose_pub_->publish(pose_msg);
  return BT::NodeStatus::SUCCESS;
}
```

### 9. Nav2 BT Node: ClearEntireCostmap

**Package:** `nav2_behavior_tree` (Nav2 plugin)  
**Library:** `nav2_clear_costmap_service_bt_node`  
**BT Node Name:** `ClearEntireCostmap`  
**Type:** Service Node

**Purpose:** Calls Nav2 ClearEntireCostmap service to reset local or global costmap.

**Input Ports:**
- `service_name` (string) - Service name (e.g., "local_costmap/clear_entirely_local_costmap")

**Service Used:** `nav2_msgs/srv/ClearEntireCostmap`

**Usage:** Clear costmaps after map switching to remove stale obstacle data from the previous map.

**Note:** Despite the class being named `ClearEntireCostmapService`, the registered BT node name is `ClearEntireCostmap` (without "Service" suffix). This was discovered using:
```bash
strings /opt/ros/humble/lib/libnav2_clear_costmap_service_bt_node.so | grep -i "clear.*costmap"
```

### 10. Custom BT Action Node: CallElevator

**Package:** `smrr_navigation`  
**Files:** `include/smrr_navigation/bt_nodes/call_elevator_action.hpp`, `src/bt_nodes/call_elevator_action.cpp`

**BT Node Name:** `CallElevator`  
**Type:** SyncActionNode

**Purpose:** Publish a Gazebo elevator command via CLI to bring the elevator to a floor.

**Inputs:**
- `floor_id` (required) - e.g., floor0/floor1/floor2/floor3
- `gz_cli` (optional, default `gz-11.14.0`)
- `gz_elevator_topic` (optional, default `/gazebo/default/elevator`)
- `timeout_sec` (optional, default 5.0)

**Behavior:**
- Maps `floor_id` → numeric string (`floor0`→`0`, …)
- Executes: `gz_cli topic -p <topic> -m 'data: "<floor_num>"'`
- Captures stdout/stderr and exit code; SUCCESS on exit code 0, otherwise FAILURE.

### 11. Custom BT Action Node: WaitForDoorOpen

**Package:** `smrr_navigation`  
**Files:** `include/smrr_navigation/bt_nodes/wait_for_door_open_action.hpp`, `src/bt_nodes/wait_for_door_open_action.cpp`

**BT Node Name:** `WaitForDoorOpen`  
**Type:** StatefulActionNode (async, non-blocking)

**Purpose:** LaserScan-based door-open detection: monitors a window of laser ranges to detect when the elevator door opens by checking if sufficient ranges exceed a distance threshold with temporal stability.

**Input Ports (with defaults):**
- `scan_topic` (string, default: `/scan`) - LaserScan topic to subscribe to
- `timeout_sec` (double, default: 30.0) - Maximum wait time before returning FAILURE
- `window_center_deg` (double, default: 0.0) - Center angle of the scan window in degrees (0° = front)
- `window_width_deg` (double, default: 30.0) - Angular width of the scan window in degrees
- `range_threshold_m` (double, default: 2.0) - Minimum range (in meters) to consider door "open"
- `fraction_threshold` (double, default: 0.6) - Fraction of valid ranges that must exceed threshold (0.0-1.0)
- `stable_time_sec` (double, default: 1.0) - Duration (seconds) the condition must remain true before SUCCESS
- `poll_rate_hz` (double, default: 10.0) - Rate (Hz) at which to check the scan (rate limiting)
- `max_scan_stale_sec` (double, default: 1.0) - Maximum age of scan before considered stale

**Detailed Algorithm:**

The WaitForDoorOpen node implements a robust, non-blocking door detection mechanism using the following logic:

**1. Initialization (onStart):**
- Creates a LaserScan subscription on the configured topic (lazy subscription)
- Initializes timing variables: start time, last check time, stable start time
- Resets state flags and latest scan data
- Returns `RUNNING` immediately (non-blocking)

**2. Scan Processing (callback thread):**
```cpp
void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  latest_scan_ = msg;
  last_scan_time_ = node_->get_clock()->now();
}
```
- Stores each incoming scan with its timestamp
- Runs asynchronously in MultiThreadedExecutor thread

**3. Periodic Checking (onRunning - BT tick thread):**

Rate limiting to avoid excessive checks:
```cpp
auto now = node_->get_clock()->now();
auto elapsed_since_check = (now - last_check_time_).seconds();
if (elapsed_since_check < 1.0 / poll_rate_hz_) {
  return BT::NodeStatus::RUNNING;  // Not time to check yet
}
last_check_time_ = now;
```

**Timeout check:**
```cpp
auto elapsed_total = (now - start_time_).seconds();
if (elapsed_total > timeout_sec_) {
  RCLCPP_ERROR(node_->get_logger(), "WaitForDoorOpen: Timeout exceeded");
  return BT::NodeStatus::FAILURE;
}
```

**Scan validity check:**
```cpp
if (!latest_scan_) {
  return BT::NodeStatus::RUNNING;  // No scan received yet, keep waiting
}

auto scan_age = (now - last_scan_time_).seconds();
if (scan_age > max_scan_stale_sec_) {
  RCLCPP_ERROR(node_->get_logger(), "WaitForDoorOpen: Scan data is stale");
  return BT::NodeStatus::FAILURE;
}
```

**4. Window Index Calculation:**
```cpp
// Convert angular window from degrees to radians
double window_center_rad = window_center_deg_ * M_PI / 180.0;
double window_half_width_rad = (window_width_deg_ / 2.0) * M_PI / 180.0;

// Calculate start and end angles for the window
double window_start_angle = window_center_rad - window_half_width_rad;
double window_end_angle = window_center_rad + window_half_width_rad;

// Convert angles to indices in the LaserScan array
int start_idx = std::max(0, 
  (int)((window_start_angle - latest_scan_->angle_min) / latest_scan_->angle_increment));
int end_idx = std::min((int)latest_scan_->ranges.size() - 1,
  (int)((window_end_angle - latest_scan_->angle_min) / latest_scan_->angle_increment));
```

**5. Range Fraction Calculation:**
```cpp
int valid_count = 0;
int exceeding_count = 0;

for (int i = start_idx; i <= end_idx; i++) {
  float range = latest_scan_->ranges[i];
  
  // Skip invalid ranges (inf, nan, out of bounds)
  if (std::isfinite(range) && 
      range >= latest_scan_->range_min && 
      range <= latest_scan_->range_max) {
    valid_count++;
    
    if (range > range_threshold_m_) {
      exceeding_count++;
    }
  }
}

double fraction = (valid_count > 0) ? 
  (double)exceeding_count / (double)valid_count : 0.0;
```

**6. Stability Check with Temporal Filtering:**
```cpp
bool condition_met = (fraction >= fraction_threshold_);

if (condition_met) {
  if (!stable_start_time_.has_value()) {
    // Condition just became true, start stability timer
    stable_start_time_ = now;
    RCLCPP_INFO(node_->get_logger(), 
      "WaitForDoorOpen: Condition met (%.1f%% > threshold %.1f%%), starting stability timer",
      fraction * 100.0, fraction_threshold_ * 100.0);
  } else {
    // Condition still true, check if stable duration reached
    auto stable_duration = (now - stable_start_time_.value()).seconds();
    if (stable_duration >= stable_time_sec_) {
      RCLCPP_INFO(node_->get_logger(), 
        "WaitForDoorOpen: Door confirmed open (stable for %.1f sec)", 
        stable_duration);
      return BT::NodeStatus::SUCCESS;
    }
  }
} else {
  // Condition no longer met, reset stability timer
  if (stable_start_time_.has_value()) {
    RCLCPP_WARN(node_->get_logger(), 
      "WaitForDoorOpen: Condition dropped (%.1f%% < threshold %.1f%%), resetting timer",
      fraction * 100.0, fraction_threshold_ * 100.0);
    stable_start_time_.reset();
  }
}

return BT::NodeStatus::RUNNING;  // Keep waiting
```

**7. Cleanup (onHalted):**
```cpp
void onHalted() override {
  scan_subscription_.reset();  // Unsubscribe from LaserScan
  latest_scan_.reset();
  stable_start_time_.reset();
}
```

**Key Features:**
- **Non-blocking:** Returns RUNNING while waiting, allowing BT to continue ticking
- **Rate-limited:** Checks scan at configurable frequency (default 10 Hz) to reduce CPU load
- **Robust filtering:** Ignores invalid ranges (inf, nan, out-of-bounds)
- **Temporal stability:** Requires condition to remain true for `stable_time_sec` to avoid false positives from transient obstacles
- **Timeout protection:** Returns FAILURE if door doesn't open within `timeout_sec`
- **Stale data detection:** Fails if scan data stops updating (LaserScan node crashed)
- **Configurable window:** Flexible angular window to focus on elevator door region
- **Thread-safe:** Scan callbacks run in separate thread from BT tick thread via MultiThreadedExecutor

**Usage in BT XML:**
```xml
<WaitForDoorOpen 
    scan_topic="/scan"
    timeout_sec="30.0"
    window_center_deg="0.0"
    window_width_deg="30.0"
    range_threshold_m="2.0"
    fraction_threshold="0.6"
    stable_time_sec="1.0"
    poll_rate_hz="10.0"
    max_scan_stale_sec="1.0"/>
```

**Typical Behavior:**
```
[WaitForDoorOpen]: Subscribed to /scan, waiting for door open...
[WaitForDoorOpen]: Checking scan (fraction: 23.4% < threshold 60.0%)
[WaitForDoorOpen]: Checking scan (fraction: 45.2% < threshold 60.0%)
[WaitForDoorOpen]: Condition met (67.8% > threshold 60.0%), starting stability timer
[WaitForDoorOpen]: Checking scan (fraction: 71.3% > threshold 60.0%), stable for 0.5 sec
[WaitForDoorOpen]: Door confirmed open (stable for 1.0 sec)
[WaitForDoorOpen]: SUCCESS
```

### 12. BehaviorTree XML: smrr_multifloor.xml

**Package:** `smrr_navigation`  
**File:** `behavior_trees/smrr_multifloor.xml`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root main_tree_to_execute="MissionTree">
  <BehaviorTree ID="MissionTree">
    <Fallback name="Root">
      <!-- Same-floor navigation -->
      <Sequence name="SameFloor">
        <IsSameFloor current_floor="{current_floor_id}" target_floor="{target_floor_id}"/>
        <NavigateToPose server_name="/navigate_to_pose" goal="{final_pose}"/>
      </Sequence>

      <!-- Cross-floor navigation with elevator entry/exit and map switching on both floors -->
      <Sequence name="CrossFloor_ElevatorEntry">
        <IsDifferentFloor current_floor="{current_floor_id}" target_floor="{target_floor_id}"/>
        
        <!-- Step 1: Get staging pose from YAML and navigate to staging area -->
        <GetNamedPose floor_id="{current_floor_id}" location_key="elevator_staging" pose="{staging_pose}"/>
        <NavigateToPose server_name="/navigate_to_pose" goal="{staging_pose}"/>
        
        <!-- Step 2: Switch to OPEN map on current floor + relocalize + clear costmaps -->
        <GetNamedMap floor_id="{current_floor_id}" map_key="open" map_yaml="{map_open_current}"/>
        <GetNamedPose floor_id="{current_floor_id}" location_key="amcl_initial_pose_open" pose="{amcl_open_current}"/>
        <SwitchMap map_yaml="{map_open_current}"/>
        <PublishInitialPose initial_pose="{amcl_open_current}"/>
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        
        <!-- Step 3: Call elevator to current floor and wait for door open -->
        <CallElevator floor_id="{current_floor_id}"/>
        <WaitForDoorOpen 
            scan_topic="/scan"
            timeout_sec="30.0"
            window_center_deg="0.0"
            window_width_deg="30.0"
            range_threshold_m="2.0"
            fraction_threshold="0.6"
            stable_time_sec="1.0"
            poll_rate_hz="10.0"
            max_scan_stale_sec="1.0"/>
        
        <!-- Step 4: Enter elevator - navigate to inside position -->
        <GetNamedPose floor_id="{current_floor_id}" location_key="elevator_inside" pose="{inside_pose}"/>
        <NavigateToPose server_name="/navigate_to_pose" goal="{inside_pose}"/>
        <Spin spin_dist="-3.1416" time_allowance="10.0" is_recovery="true"/>
        
        <!-- Step 5: Switch to target floor OPEN map after elevator transit -->
        <GetNamedMap floor_id="{target_floor_id}" map_key="open" map_yaml="{map_open_target}"/>
        <GetNamedPose floor_id="{target_floor_id}" location_key="amcl_initial_pose_open" pose="{amcl_open_target}"/>
        <SwitchMap map_yaml="{map_open_target}"/>
        <PublishInitialPose initial_pose="{amcl_open_target}"/>
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>

        <!-- Step 6: Call elevator to target floor exit side and wait for door open -->
        <CallElevator floor_id="{target_floor_id}"/>
        <WaitForDoorOpen 
            scan_topic="/scan"
            timeout_sec="30.0"
            window_center_deg="0.0"
            window_width_deg="30.0"
            range_threshold_m="2.0"
            fraction_threshold="0.6"
            stable_time_sec="1.0"
            poll_rate_hz="10.0"
            max_scan_stale_sec="1.0"/>

        <!-- Step 7: Exit elevator and relocalize on CLOSED map, then go to final pose -->
        <GetNamedPose floor_id="{target_floor_id}" location_key="elevator_exit" pose="{exit_pose}"/>
        <NavigateToPose server_name="/navigate_to_pose" goal="{exit_pose}"/>

        <GetNamedMap floor_id="{target_floor_id}" map_key="closed" map_yaml="{map_closed_target}"/>
        <GetNamedPose floor_id="{target_floor_id}" location_key="amcl_initial_pose_closed" pose="{amcl_closed_target}"/>
        <SwitchMap map_yaml="{map_closed_target}"/>
        <PublishInitialPose initial_pose="{amcl_closed_target}"/>
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>

        <NavigateToPose server_name="/navigate_to_pose" goal="{final_pose}"/>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>
```

**Logic Flow (CrossFloor_ElevatorEntry):**
1. **Navigate to staging on current floor**: GetNamedPose(elevator_staging) → NavigateToPose
2. **Switch to OPEN map on current floor**: GetNamedMap(open) → SwitchMap → PublishInitialPose → Clear costmaps
3. **Call elevator + wait door open**: CallElevator(current_floor) → WaitForDoorOpen
4. **Enter elevator**: GetNamedPose(elevator_inside) → NavigateToPose → Spin (align inside car)
5. **Switch to target OPEN map**: GetNamedMap(open@target) → SwitchMap → PublishInitialPose → Clear costmaps
6. **Call elevator + wait door open on target side**: CallElevator(target_floor) → WaitForDoorOpen
7. **Exit elevator and relocalize on CLOSED map**: GetNamedPose(elevator_exit) → NavigateToPose → switch to closed map → PublishInitialPose → Clear costmaps → NavigateToPose(final_pose)

**Current Implementation Status:**
- ✅ Same-floor navigation
- ✅ Cross-floor: staging, elevator call/wait, enter elevator, target-floor map switch, exit elevator
- ✅ Map switching on both floors (open then closed), AMCL relocalization, costmap clearing
- ✅ Final NavigateToPose to destination on target floor

**Notes:**
- `{final_pose}` is set once by the executor from the StartMission request and reused at the end. If goal rejection occurs due to an old timestamp in your setup, reintroduce `UpdatePoseTimestamp` before the final NavigateToPose.

### 7. Named Goal Server (Python)

**Package:** `smrr_navigation`  
**File:** `smrr_navigation/named_goal_server.py`  
**Node Name:** `/named_goal_server`  
**Service Provided:** `/go_to_pose` (smrr_interfaces/srv/NavGoal)  
**Service Client:** `/start_mission` (smrr_interfaces/srv/StartMission)

**Responsibilities:**
- Maintain database of named locations (from `locations.yaml`)
- Resolve location names to floor IDs and coordinates
- Track current floor state
- Call StartMission service with resolved parameters
- Return mission result to original caller

**Key Parameters:**
```yaml
locations_file: "locations.yaml"           # Named locations database
use_bt_mission_executor: true              # Use BT executor (vs legacy action)
initial_floor_id: "floor0"                 # Starting floor
start_mission_service_name: "/start_mission"
start_mission_timeout: 5.0                 # Service call timeout
```

**Location Database Structure (`locations.yaml`):**
```yaml
floors:
  floor0:
    maps:
      closed: first_floor_with_docking_station.yaml
      open: floor0_open.yaml
    locations:
      amcl_initial_pose_closed:
        x: -1.5
        y: 0.8
        yaw: 1.57
      amcl_initial_pose_open:
        x: -2.0
        y: 1.3
        yaw: 1.57
      dock:
        x: 2.23
        y: -1.0
        yaw: 0.0
      left_back:
        x: -9.9
        y: 1.33
        yaw: 0.0
      elevator_staging:
        x: -2.0
        y: 1.3
        yaw: 1.57
  floor1:
    maps:
      closed: second_floor.yaml
      open: floor1_open.yaml
    locations:
      amcl_initial_pose_closed:
        x: 2.484
        y: -1.621
        yaw: -1.57
      amcl_initial_pose_open:
        x: -1.759
        y: 1.272
        yaw: 1.57
      office_101:
        x: 7.93
        y: -4.29
        yaw: 0.0
      # ... more locations
```

**Key Structure Changes:**
- AMCL initial poses now stored as regular locations under each floor
- Map file references included for each floor (closed/open variants)
- All poses use consistent x, y, yaw format (radians)
- GetNamedPose action can fetch AMCL poses using location_key="amcl_initial_pose_closed"

### 8. Location Subscriber (Python)

**Package:** `smrr_navigation`  
**File:** `smrr_navigation/location_subscriber.py`  
**Node Name:** `/location_subscriber`  
**Topic Subscribed:** `/location` (std_msgs/String)  
**Service Client:** `/go_to_pose` (smrr_interfaces/srv/NavGoal)

**Purpose:** Simple bridge from topic-based interface to service-based interface.

**Usage:**
```bash
ros2 topic pub --once /location std_msgs/String "{data: 'dock'}"
```

---

## Data Flow

### Complete Navigation Request Flow

```
1. User publishes location name
   └─> /location topic: "dock"

2. location_subscriber receives message
   └─> Calls /go_to_pose service with location="dock"

3. named_goal_server processes request
   ├─> Looks up "dock" in locations database
   ├─> Finds: floor="floor0", x=2.23, y=-1.00, yaw=0.0
   ├─> Generates mission_id (UUID)
   └─> Calls /start_mission service:
       {
         mission_id: "af2fdd01-f01c-...",
         current_floor_id: "floor0",
         target_floor_id: "floor0",
         target_location_name: "dock",
         x: 2.23, y: -1.00, yaw: 0.0
       }

4. smrr_bt_mission_executor receives StartMission request
   ├─> Creates BehaviorTree blackboard
   ├─> Sets blackboard entries:
   │   ├─> node (ROS 2 node handle)
   │   ├─> current_floor_id = "floor0"
   │   ├─> target_floor_id = "floor0"
   │   ├─> final_pose = PoseStamped(2.23, -1.00, 0.0)
   │   └─> Nav2 timing parameters
  ├─> Loads BT from smrr_multifloor.xml
   └─> Starts BT execution loop (20 Hz)

5. BehaviorTree executes
   ├─> Fallback tries SameFloor sequence first
   ├─> IsSameFloor condition checks:
   │   └─> "floor0" == "floor0" → SUCCESS
   ├─> NavigateToPose action executes:
   │   ├─> Sends goal to Nav2 /navigate_to_pose
   │   ├─> Nav2 plans path and executes
   │   └─> Returns SUCCESS when goal reached

     Alternative: Cross-Floor Request
     ├─> IsSameFloor: "floor0" != "floor1" → FAILURE
     ├─> Fallback runs CrossFloor_ElevatorEntry sequence
     ├─> Steps:
     │   1) GetNamedPose(elevator_staging) → NavigateToPose (current floor staging)
     │   2) GetNamedMap(open@current) → SwitchMap → PublishInitialPose → Clear costmaps
     │   3) CallElevator(current_floor) → WaitForDoorOpen
     │   4) GetNamedPose(elevator_inside) → NavigateToPose → Spin (inside elevator)
     │   5) GetNamedMap(open@target) → SwitchMap → PublishInitialPose → Clear costmaps
     │   6) CallElevator(target_floor) → WaitForDoorOpen
     │   7) GetNamedPose(elevator_exit) → NavigateToPose (exit to hallway)
     │   8) GetNamedMap(closed@target) → SwitchMap → PublishInitialPose → Clear costmaps
     │   9) NavigateToPose(final_pose) on target floor
     └─> BT returns SUCCESS once the final NavigateToPose succeeds

6. Mission result propagates back
   ├─> smrr_bt_mission_executor: BT SUCCESS
   │   └─> Returns StartMission response: {success: true, message: "BT completed"}
   ├─> named_goal_server: Logs completion
   │   └─> Returns NavGoal response to location_subscriber
   └─> location_subscriber: Logs success
```

---

## Build Configuration

### Package Structure

```
smrr_navigation/
├── CMakeLists.txt          # Hybrid ament_cmake build
├── package.xml             # Package manifest (ament_cmake)
├── behavior_trees/
│   └── smrr_multifloor.xml
├── config/
│   └── locations.yaml      # Restructured with AMCL poses as locations
├── include/
│   └── smrr_navigation/
│       └── bt_nodes/
│           ├── is_same_floor_condition.hpp
│           ├── is_different_floor_condition.hpp
│           ├── get_named_pose_action.hpp
│           ├── get_named_map_action.hpp
│           ├── switch_map_action.hpp
│           ├── publish_initial_pose_action.hpp
│           ├── call_elevator_action.hpp
│           ├── wait_for_door_open_action.hpp
│           └── update_pose_timestamp_action.hpp
├── src/
│   ├── bt_nodes/
│   │   ├── is_same_floor_condition.cpp
│   │   ├── is_different_floor_condition.cpp
│   │   ├── get_named_pose_action.cpp
│   │   ├── get_named_map_action.cpp
│   │   ├── switch_map_action.cpp
│   │   ├── publish_initial_pose_action.cpp
│   │   ├── call_elevator_action.cpp
│   │   ├── wait_for_door_open_action.cpp
│   │   ├── update_pose_timestamp_action.cpp
│   │   └── bt_node_registration.cpp
│   └── smrr_bt_mission_executor.cpp
├── smrr_navigation/         # Python package
│   ├── __init__.py
│   ├── location_subscriber.py
│   ├── named_goal_client.py
│   ├── named_goal_server.py
│   ├── smrr_multifloor_bt_navigator.py
│   └── startup_localizer.py
└── launch/
    └── smrr_world_navigation.launch.py
```

### CMakeLists.txt Key Sections

```cmake
cmake_minimum_required(VERSION 3.8)
project(smrr_navigation)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(smrr_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Build custom BT plugin library
add_library(smrr_bt_nodes SHARED
  src/bt_nodes/is_same_floor_condition.cpp
  src/bt_nodes/is_different_floor_condition.cpp
  src/bt_nodes/get_named_pose_action.cpp
  src/bt_nodes/get_named_map_action.cpp
  src/bt_nodes/switch_map_action.cpp
  src/bt_nodes/publish_initial_pose_action.cpp
  src/bt_nodes/call_elevator_action.cpp
  src/bt_nodes/wait_for_door_open_action.cpp
  src/bt_nodes/update_pose_timestamp_action.cpp
  src/bt_nodes/bt_node_registration.cpp
)
ament_target_dependencies(smrr_bt_nodes
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
  nav2_msgs
)
target_link_libraries(smrr_bt_nodes
  yaml-cpp
)

# Build mission executor executable
add_executable(smrr_bt_mission_executor
  src/smrr_bt_mission_executor.cpp
)
ament_target_dependencies(smrr_bt_mission_executor
  rclcpp
  nav2_behavior_tree
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  smrr_interfaces
  ament_index_cpp
)

# Install C++ targets
install(TARGETS
  smrr_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  smrr_bt_mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  smrr_navigation/startup_localizer.py
  smrr_navigation/named_goal_server.py
  smrr_navigation/named_goal_client.py
  smrr_navigation/location_subscriber.py
  smrr_navigation/smrr_multifloor_bt_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install configs, behavior trees, maps, and launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

install(DIRECTORY
  resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()
```

### package.xml Key Dependencies

```xml
<package format="3">
  <name>smrr_navigation</name>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>nav2_behavior_tree</depend>
  <depend>nav2_msgs</depend>
  <depend>behaviortree_cpp_v3</depend>
  <depend>smrr_interfaces</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>ament_index_cpp</depend>
  <depend>yaml-cpp</depend>
  
  <exec_depend>nav2_bringup</exec_depend>
</package>
```

---

## Launch Configuration

### Launch File: smrr_world_navigation.launch.py

**Key Nodes Launched:**

1. **Gazebo & Robot Controllers** (via included launch files)
2. **Nav2 Stack** (via nav2_bringup)
3. **Named Goal Server**
4. **Location Subscriber**
5. **BT Mission Executor**
6. **RViz2**

**BT Mission Executor Configuration:**

```python
# Define BT XML path
bt_xml_path = os.path.join(pkg_share, 'behavior_trees', 'smrr_multifloor.xml')

Node(
    package='smrr_navigation',
    executable='smrr_bt_mission_executor',
    name='smrr_bt_mission_executor',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'bt_xml_path': bt_xml_path,
        'plugin_lib_names': [
            # Nav2 BT plugins
            'nav2_compute_path_to_pose_action_bt_node',
            'nav2_navigate_to_pose_action_bt_node',
            # ... (all Nav2 plugins)
            # Custom BT plugins
            'smrr_bt_nodes'  # Note: Library name without lib prefix or .so extension
        ],
        'bt_tick_rate_hz': 20.0,
        'bt_timeout_sec': 300.0
    }]
)
```

**Named Goal Server Configuration:**

```python
Node(
    package='smrr_navigation',
    executable='named_goal_server.py',
    name='named_goal_server',
    output='screen',
    respawn=True,              # Auto-restart on crashes
    respawn_delay=2.0,
    parameters=[{
        'use_sim_time': True,
        'locations_file': 'locations.yaml',
        'use_bt_mission_executor': True,
        'initial_floor_id': initial_floor_id,
        'start_mission_service_name': '/start_mission',
        'start_mission_timeout': 5.0
    }]
)
```

---

## Usage

### Starting the System

```bash
# Terminal 1: Build and source
cd ~/navigation_ws
colcon build --symlink-install
source install/setup.bash

# Terminal 2: Launch complete system
ros2 launch smrr_navigation smrr_world_navigation.launch.py
```

### Testing Same-Floor Navigation

**Via Topic:**
```bash
# Navigate to dock (same floor)
ros2 topic pub --once /location std_msgs/String "{data: 'dock'}"

# Navigate to left_back (same floor)
ros2 topic pub --once /location std_msgs/String "{data: 'left_back'}"

# Navigate to elevator staging area (same floor)
ros2 topic pub --once /location std_msgs/String "{data: 'elevator_staging'}"
```

**Via Service (Direct):**
```bash
ros2 service call /go_to_pose smrr_interfaces/srv/NavGoal \
  "{location: 'dock'}"
```

**Via StartMission Service (Low-Level):**
```bash
ros2 service call /start_mission smrr_interfaces/srv/StartMission \
  "{mission_id: 'test-123', \
    current_floor_id: 'floor0', \
    target_floor_id: 'floor0', \
    target_location_name: 'dock', \
    x: 2.23, y: -1.00, yaw: 0.0}"
```

### Testing Cross-Floor Navigation (Elevator Flow)

**Via Topic:**
```bash
ros2 topic pub --once /location std_msgs/String "{data: 'office_101'}"  # floor1 target
```

**Via StartMission Service (Low-Level, explicit target pose):**
```bash
ros2 service call /start_mission smrr_interfaces/srv/StartMission \
  "{mission_id: 'cross-floor-test', \
    current_floor_id: 'floor0', \
    target_floor_id: 'floor1', \
    target_location_name: 'office_101', \
    x: 7.93, y: -4.29, yaw: 0.0}"
```

### Expected Output (Success)

```
[location_subscriber]: Received location: "dock"
[named_goal_server]: Resolved location "dock" -> floor: "floor0", pose: (2.23, -1.00, 0.0°)
[named_goal_server]: Calling StartMission service: mission_id=af2fdd01-..., current_floor=floor0, target_floor=floor0
[smrr_bt_mission_executor]: Received StartMission request
[smrr_bt_mission_executor]: Creating BT engine and loading tree
[smrr_bt_mission_executor]: BT created successfully, starting execution...
[smrr_bt_mission_executor]: [IsSameFloor] SUCCESS: Both floors are 'floor0'
[bt_navigator]: Begin navigating from current location to (2.23, -1.00)
[controller_server]: Received a goal, begin computing control effort
[controller_server]: Reached the goal!
[bt_navigator]: Goal succeeded
[smrr_bt_mission_executor]: BT execution SUCCESS
[named_goal_server]: Navigation to dock completed: SUCCESS
```

### Expected Output (Cross-floor Elevator Flow)

```
[smrr_bt_mission_executor]: [IsSameFloor] FAILURE: floor0 != floor1
[smrr_bt_mission_executor]: [IsDifferentFloor] SUCCESS
[GetNamedPose] elevator_staging @ floor0
[bt_navigator]: Goal succeeded (staging)
[SwitchMap]: Loaded OPEN map for floor0
[PublishInitialPose]: Published amcl_initial_pose_open (floor0)
[CallElevator]: Sent command for floor0
[WaitForDoorOpen]: Door open detected (current floor)
[bt_navigator]: Navigating to elevator_inside
[Spin]: Completed rotation inside elevator
[SwitchMap]: Loaded OPEN map for floor1
[PublishInitialPose]: Published amcl_initial_pose_open (floor1)
[CallElevator]: Sent command for floor1
[WaitForDoorOpen]: Door open detected (target floor)
[bt_navigator]: Navigating to elevator_exit
[SwitchMap]: Loaded CLOSED map for floor1
[PublishInitialPose]: Published amcl_initial_pose_closed (floor1)
[bt_navigator]: Navigating to final pose (office_101)
[smrr_bt_mission_executor]: BT execution SUCCESS
```

---

## Debugging

### Check Node Status

```bash
# List all nodes
ros2 node list

# Should include:
# - /named_goal_server
# - /location_subscriber
# - /smrr_bt_mission_executor
# - /bt_navigator
# - /controller_server
# - /planner_server
```

### Check Services

```bash
# List services
ros2 service list | grep -E "(go_to_pose|start_mission)"

# Should show:
# - /go_to_pose
# - /start_mission
```

### Check BT Plugin Loading

```bash
# Verify library has registration function
nm -D install/smrr_navigation/lib/libsmrr_bt_nodes.so | grep BT_RegisterNodesFromPlugin

# Should output:
# 00000000000290f4 T BT_RegisterNodesFromPlugin
```

### Monitor BT Execution

```bash
# Watch executor logs
ros2 topic echo /rosout | grep smrr_bt_mission_executor
```

### Common Issues

**Issue 1: "executable 'named_goal_server.py' not found"**
- **Cause:** Python scripts not executable
- **Fix:**
  ```bash
  chmod +x src/smrr_navigation/smrr_navigation/*.py
  colcon build --symlink-install --packages-select smrr_navigation
  ```

**Issue 2: "can't find symbol [BT_RegisterNodesFromPlugin]"**
- **Cause:** Incorrect BT plugin registration
- **Fix:** Ensure `extern "C" void BT_RegisterNodesFromPlugin(...)` is defined

**Issue 3: "Missing key [node] / [bt_loop_duration]"**
- **Cause:** Blackboard not properly initialized
- **Fix:** Ensure executor sets all required Nav2 blackboard entries:
  ```cpp
  blackboard->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
  ```

**Issue 4: Plugin library name issues**
- **Incorrect:** `'libsmrr_bt_nodes.so'` (BT adds lib/so automatically)
- **Correct:** `'smrr_bt_nodes'`

---

## Testing Cross-Floor Elevator Flow

**Test Case 1:** Same-floor navigation (should work as before)

```bash
ros2 service call /start_mission smrr_interfaces/srv/StartMission \
  "{mission_id: 'same-floor-test', \
    current_floor_id: 'floor0', \
    target_floor_id: 'floor0', \
    target_location_name: 'dock', \
    x: 2.23, y: -1.00, yaw: 0.0}"
```

**Expected Result:**
```
[IsSameFloor] SUCCESS: Both floors are 'floor0'
[bt_navigator]: Begin navigating from current location to (2.23, -1.00)
[controller_server]: Reached the goal!
[smrr_bt_mission_executor]: BT execution SUCCESS
```

**Test Case 2:** Cross-floor navigation (full elevator flow to target)

```bash
ros2 service call /start_mission smrr_interfaces/srv/StartMission \
  "{mission_id: 'cross-floor-test', \
    current_floor_id: 'floor0', \
    target_floor_id: 'floor1', \
    target_location_name: 'office_101', \
    x: 7.93, y: -4.29, yaw: 0.0}"
```

**Expected Result (key checkpoints):**
```
[IsSameFloor] FAILURE: floor0 != floor1
[IsDifferentFloor] SUCCESS
[GetNamedPose] elevator_staging @ floor0
[bt_navigator]: Goal succeeded (staging)
[SwitchMap]: Loaded OPEN map for floor0; PublishInitialPose open pose; ClearEntireCostmap (local/global)
[CallElevator]: floor0 command sent; [WaitForDoorOpen]: detected door open
[NavigateToPose]: elevator_inside; [Spin]: completed
[SwitchMap]: Loaded OPEN map for floor1; PublishInitialPose open pose; ClearEntireCostmap (local/global)
[CallElevator]: floor1 command sent; [WaitForDoorOpen]: detected door open
[NavigateToPose]: elevator_exit
[SwitchMap]: Loaded CLOSED map for floor1; PublishInitialPose closed pose; ClearEntireCostmap (local/global)
[NavigateToPose]: final_pose (office_101)
[smrr_bt_mission_executor]: BT execution SUCCESS
```

**Behavior:** 
- Robot stages on the current floor, switches to the OPEN map, calls the elevator, and waits for the door to open.
- It enters the elevator, spins to align, and switches to the target floor OPEN map while in transit.
- On the target side it calls the elevator again, waits for door-open detection, and exits to the hallway.
- It switches to the target CLOSED map, republishes the initial pose, clears costmaps, and navigates to the final target pose.

---

## Critical Implementation Patterns & Debugging Insights

### 1. MultiThreadedExecutor + Reentrant Callback Pattern

**Problem:** BT nodes making async service calls (like SwitchMap calling LoadMap) would timeout even though the service completed successfully.

**Root Cause:** The StartMission service callback blocks while running the BT loop. Without proper executor configuration, service response callbacks from BT nodes cannot be processed, causing deadlock.

**Solution:**
```cpp
// In BtMissionExecutor constructor:
callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

service_ = this->create_service<smrr_interfaces::srv::StartMission>(
  "/start_mission",
  std::bind(&BtMissionExecutor::handleStartMission, this, ...),
  rmw_qos_profile_services_default,
  callback_group_);  // ← Associate with reentrant group

// In main():
rclcpp::executors::MultiThreadedExecutor executor;  // ← Use multi-threaded
executor.add_node(node);
executor.spin();
```

**Why It Works:**
- `Reentrant` callback group allows callbacks to execute concurrently
- `MultiThreadedExecutor` spawns multiple threads to handle callbacks
- While thread A is blocked in StartMission callback (running BT), thread B can process LoadMap response
- This avoids "Node already added to executor" error that occurs with `rclcpp::spin_some()`

**Error If Wrong:** `"Node '/smrr_bt_mission_executor' has already been added to an executor"`

### 2. StatefulActionNode for Async Operations

**Problem:** Long-running service calls (map loading takes 1-5 seconds) would block the entire BT tick loop.

**Solution:** Use `StatefulActionNode` instead of `SyncActionNode`:

```cpp
class SwitchMapAction : public BT::StatefulActionNode
{
  BT::NodeStatus onStart() override {
    // Start async operation, return RUNNING immediately
    load_map_future_ = client_->async_send_request(request).future.share();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    // Check if done (non-blocking)
    if (load_map_future_.wait_for(0ms) == std::future_status::ready) {
      auto response = load_map_future_.get();
      return response->success ? SUCCESS : FAILURE;
    }
    
    // Check timeout
    if (elapsed > timeout_) return FAILURE;
    
    return BT::NodeStatus::RUNNING;  // Still waiting
  }

  void onHalted() override {
    // Cleanup if cancelled
  }
};
```

**Key Points:**
- `onStart()`: Initiate async operation, return RUNNING
- `onRunning()`: Check progress each tick, return RUNNING/SUCCESS/FAILURE
- BT continues ticking while node is RUNNING
- Allows executor to process other callbacks between ticks

### 3. SharedFuture for Service Response Handling

**Problem:** Compilation errors when trying to store `std::future` returned by `async_send_request()`.

**Solution:** Use `.future.share()` to convert to `std::shared_future`:

```cpp
// ❌ Wrong:
std::future<...> future_ = client_->async_send_request(request);

// ✅ Correct:
std::shared_future<...> future_ = client_->async_send_request(request).future.share();
```

**Why:** `std::future` is move-only and gets invalidated. `std::shared_future` can be copied and safely stored.

### 4. Safe Optional Input Handling

**Problem:** `bad_expected_access` exception when calling `.value()` on optional inputs without checking.

**Bad Pattern:**
```cpp
auto topic_name = getInput<std::string>("topic_name").value();  // ❌ Throws if missing
```

**Good Pattern:**
```cpp
std::string topic_name = "/initialpose";  // Default
auto topic_input = getInput<std::string>("topic_name");
if (topic_input.has_value()) {
  topic_name = topic_input.value();  // ✅ Safe
}
```

**Alternative (if default in port definition):**
```cpp
auto topic_name = getInput<std::string>("topic_name").value_or("/initialpose");
```

### 5. Discovering Nav2 BT Node Names

**Problem:** Nav2 BT node names don't always match class names or follow predictable patterns.

**Discovery Method:**
```bash
# Find the plugin library
find /opt/ros/humble -name "libnav2_*_bt_node.so"

# Check registered node names
strings /opt/ros/humble/lib/libnav2_clear_costmap_service_bt_node.so | grep -i "clear.*costmap"
```

**Example Finding:**
- Class: `ClearEntireCostmapService`
- BT Node Name: `ClearEntireCostmap` (no "Service" suffix)

### 6. Blackboard Access from BT Nodes

**Problem:** Custom BT nodes need ROS node for service clients/publishers.

**Wrong Pattern:**
```cpp
auto node = getInput<rclcpp::Node::SharedPtr>("node");  // ❌ node is NOT an input port
```

**Correct Pattern:**
```cpp
config().blackboard->get("node", node_);  // ✅ Access blackboard directly
```

**Setup in Executor:**
```cpp
blackboard->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
```

### 7. Debugging Service Call Timeouts

**Checklist when service calls timeout:**
1. ✅ Service exists: `ros2 service list | grep <service_name>`
2. ✅ Service type matches: `ros2 service type <service_name>`
3. ✅ Service responds manually: `ros2 service call <service_name> <type> ...`
4. ✅ Client creates successfully: Check `load_map_client_` not null
5. ✅ Request sent: Log in onStart()
6. ✅ Service processes request: Check service node logs
7. ✅ **Response callbacks processed**: Ensure executor can handle callbacks (use MultiThreadedExecutor + Reentrant)
8. ✅ Future handling correct: Use `.future.share()` and check `future.valid()`

**Common Issue:** Service completes but response never processed → Missing callback processing in executor

### 8. Map Loading Cascade

**Understanding the LoadMap Service Flow:**
```
1. SwitchMap calls /map_server/load_map
   ↓
2. map_server loads YAML + PGM image (~1 second)
   ↓
3. map_server publishes new map to /map topic
   ↓
4. AMCL subscribes to /map, receives new map
   ↓
5. AMCL reinitializes particle filter (~1-2 seconds)
   ↓
6. Costmaps subscribe to /map, receive new map
   ↓
7. Costmaps resize and rebuild static layer (~2-3 seconds)
   ↓
8. LoadMap service returns response (~5 seconds total)
```

**Why 10-second timeout?** Need time for AMCL + costmap cascade to complete before response.

**Logs to Verify Success:**
```
[map_io]: Read map .../floor0_open.pgm: 346 X 158 map @ 0.05 m/cell
[amcl]: Received a 346 X 158 map @ 0.050 m/pix
[global_costmap]: StaticLayer: Resizing costmap to 346 X 158 at 0.050000 m/pix
```

### 9. AMCL Relocalization After Map Switch

**Critical Steps:**
1. **Switch Map:** LoadMap service
2. **Publish Initial Pose:** To `/initialpose` topic
3. **Clear Costmaps:** Remove stale obstacle data from old map

**AMCL Logs:**
```
[amcl]: initialPoseReceived
[amcl]: Setting pose (32.200000): -2.000 1.300 1.570
```

**If Relocalization Fails:**
- Check pose is in valid map region (not in obstacles)
- Verify frame_id matches AMCL's global_frame_id (usually "map")
- Ensure covariance values are reasonable (too small = overconfident, too large = scattered)

### 10. Build System for BT Plugins

**Plugin Library Requirements:**
```cmake
# CMakeLists.txt for smrr_bt_nodes library
add_library(smrr_bt_nodes SHARED
  src/bt_nodes/bt_node_registration.cpp
  src/bt_nodes/is_different_floor_condition.cpp
  src/bt_nodes/is_same_floor_condition.cpp
  src/bt_nodes/get_named_pose_action.cpp
  src/bt_nodes/get_named_map_action.cpp
  src/bt_nodes/switch_map_action.cpp
  src/bt_nodes/publish_initial_pose_action.cpp
  src/bt_nodes/call_elevator_action.cpp
  src/bt_nodes/wait_for_door_open_action.cpp
  src/bt_nodes/update_pose_timestamp_action.cpp
)

# Must export plugin symbols
target_compile_definitions(smrr_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

# Install plugin library
install(TARGETS smrr_bt_nodes
  LIBRARY DESTINATION lib)
```

**Plugin Registration:**
```cpp
// In bt_node_registration.cpp
extern "C" BT_PLUGIN_EXPORT void BT_RegisterNodesFromPlugin(
  BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<IsSameFloorCondition>("IsSameFloor");
  factory.registerNodeType<IsDifferentFloorCondition>("IsDifferentFloor");
  factory.registerNodeType<SwitchMapAction>("SwitchMap");
  factory.registerNodeType<GetNamedPoseAction>("GetNamedPose");
  factory.registerNodeType<GetNamedMapAction>("GetNamedMap");
  factory.registerNodeType<PublishInitialPoseAction>("PublishInitialPose");
  factory.registerNodeType<CallElevatorAction>("CallElevator");
  factory.registerNodeType<WaitForDoorOpenAction>("WaitForDoorOpen");
  factory.registerNodeType<UpdatePoseTimestampAction>("UpdatePoseTimestamp");
  // ... register all nodes
}
```

**Load Plugin in Executor:**
```yaml
# In params file
plugin_lib_names:
  - smrr_bt_nodes  # Loads libsmrr_bt_nodes.so
  - nav2_navigate_to_pose_action_bt_node
```

---

## Performance Characteristics

- **BT Tick Rate:** 20 Hz (configurable)
- **Service Response Time:** ~10-100ms (depends on BT creation overhead)
- **Navigation Timeout:** 300 seconds (5 minutes, configurable)
- **Maximum Concurrent Missions:** 1 (service-based, sequential execution)

---

## Future Enhancements

### Planned Features
1. **Elevator Robustness:** Handle elevator faults/timeouts (retry/backoff) and door-open edge cases
2. **Recovery Behaviors:** Integrate Nav2 recoveries (e.g., spin, clear costmap) into the BT flow
3. **Path Validation:** Pre-check path feasibility before starting navigation
4. **Mission Queue:** Support multiple pending missions with priority
5. **Mission Cancellation:** Implement cancel service for active missions

### Extensibility Points
- **Custom BT Nodes:** Add new condition/action nodes in `src/bt_nodes/`
- **Alternative BTs:** Create multiple XML files for different mission types
- **Dynamic BT Selection:** Choose BT based on mission parameters
- **Blackboard Extensions:** Add custom data to blackboard for advanced logic

---

## File Summary

### Created Files
```
smrr_interfaces/srv/StartMission.srv
smrr_navigation/behavior_trees/smrr_multifloor.xml
smrr_navigation/include/smrr_navigation/bt_nodes/is_same_floor_condition.hpp
smrr_navigation/src/bt_nodes/is_same_floor_condition.cpp
smrr_navigation/include/smrr_navigation/bt_nodes/is_different_floor_condition.hpp
smrr_navigation/src/bt_nodes/is_different_floor_condition.cpp
smrr_navigation/include/smrr_navigation/bt_nodes/get_named_pose_action.hpp
smrr_navigation/src/bt_nodes/get_named_pose_action.cpp
smrr_navigation/include/smrr_navigation/bt_nodes/get_named_map_action.hpp
smrr_navigation/src/bt_nodes/get_named_map_action.cpp
smrr_navigation/include/smrr_navigation/bt_nodes/switch_map_action.hpp
smrr_navigation/src/bt_nodes/switch_map_action.cpp
smrr_navigation/include/smrr_navigation/bt_nodes/publish_initial_pose_action.hpp
smrr_navigation/src/bt_nodes/publish_initial_pose_action.cpp
smrr_navigation/include/smrr_navigation/bt_nodes/call_elevator_action.hpp
smrr_navigation/src/bt_nodes/call_elevator_action.cpp
smrr_navigation/include/smrr_navigation/bt_nodes/wait_for_door_open_action.hpp
smrr_navigation/src/bt_nodes/wait_for_door_open_action.cpp
smrr_navigation/include/smrr_navigation/bt_nodes/update_pose_timestamp_action.hpp
smrr_navigation/src/bt_nodes/update_pose_timestamp_action.cpp
smrr_navigation/smrr_navigation/smrr_multifloor_bt_navigator.py
smrr_navigation/smrr_navigation/startup_localizer.py
smrr_navigation/src/bt_nodes/bt_node_registration.cpp
smrr_navigation/src/smrr_bt_mission_executor.cpp
MAP_SWITCHING_IMPLEMENTATION.md
COSTMAP_CLEARING_FIX.md
```

### Modified Files
```
smrr_interfaces/CMakeLists.txt                          # Added StartMission.srv
smrr_navigation/package.xml                             # Changed to ament_cmake, added yaml-cpp, nav2 deps
smrr_navigation/CMakeLists.txt                          # Hybrid C++/Python build, BT plugin library, behavior_trees install
smrr_navigation/config/locations.yaml                   # Added maps section, AMCL poses as locations
smrr_navigation/behavior_trees/smrr_multifloor.xml      # Added map switching sequence
smrr_navigation/smrr_navigation/named_goal_server.py    # Added BT executor client
smrr_navigation/launch/smrr_world_navigation.launch.py  # Launch BT executor with MultiThreadedExecutor
```

---

## References

- **BehaviorTree.CPP:** https://www.behaviortree.dev/
- **Nav2 Documentation:** https://navigation.ros.org/
- **Nav2 BT Nodes:** https://navigation.ros.org/plugins/index.html
- **ROS 2 Services:** https://docs.ros.org/en/humble/Tutorials/Services.html

---

## Changelog

| Date | Version | Changes |
|------|---------|---------|
| 2026-01-03 | 1.0.0 | Initial BT-based navigation implementation (same-floor) |
| 2026-01-03 | 1.1.0 | Added cross-floor staging: IsDifferentFloor, GetNamedPose nodes, Fallback BT structure |
| 2026-01-03 | 1.2.0 | Restructured locations.yaml (AMCL poses as locations), renamed BT to smrr_multifloor.xml |
| 2026-01-03 | 1.3.0 | **Map Switching Implementation**: Added GetNamedMap, SwitchMap (async), PublishInitialPose BT nodes |
| 2026-01-03 | 1.3.1 | Fixed executor pattern: MultiThreadedExecutor + Reentrant callback group for nested service calls |
| 2026-01-03 | 1.3.2 | Fixed PublishInitialPose input handling (`bad_expected_access` error) |
| 2026-01-03 | 1.3.3 | Added costmap clearing to BT sequence (ClearEntireCostmap nodes) |
| 2026-01-03 | 1.4.0 | Complete cross-floor map switching (open map, relocalize, clear costmaps) |
| 2026-01-03 | **1.5.0** | **✅ Elevator call + door-open wait + entry/exit + target-floor map switching and final navigation** |
---

## Contact & Support

For questions or issues related to this implementation, refer to:
- Project repository: `Quanta-Projects/navigation_ws`
- Branch: `main`
- Implementation session: January 3, 2026

---

**End of Document**
