# BehaviorTree-Based Navigation System Implementation

## Overview

This document describes the complete implementation of a BehaviorTree (BT) based navigation system for the SMRR robot. The system replaces the previous Python state machine approach with a modular, XML-configurable BT architecture that integrates with Nav2 for both same-floor and cross-floor navigation tasks.

**Implementation Date:** January 3, 2026  
**ROS 2 Distribution:** Humble  
**BehaviorTree Library:** BehaviorTree.CPP v3  
**Current Milestone:** Cross-floor staging (robot navigates to elevator staging area)

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
│  - Loads BehaviorTree from XML                         │
│  - Sets blackboard variables                           │
│  - Executes BT with Nav2 integration                   │
└────────┬────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────┐
│  same_floor_nav.xml (BehaviorTree)                     │
│  Fallback (Root)                                       │
│  ├─ Sequence (SameFloor)                               │
│  │  ├─ IsSameFloor (Custom Condition)                  │
│  │  └─ NavigateToPose (Nav2 Action)                    │
│  └─ Sequence (CrossFloor_StagingOnly)                  │
│     ├─ IsDifferentFloor (Custom Condition)             │
│     ├─ GetNamedPose (Custom Action) → staging_pose     │
│     └─ NavigateToPose (Nav2 Action)                    │
└─────────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Separation of Concerns**: Named location resolution happens in Python layer; navigation control logic in BT
2. **Service-Based Architecture**: Clean interfaces between components using ROS 2 services
3. **BT-Driven Logic**: All navigation decision-making happens inside the BehaviorTree
4. **No Rejection Logic**: Service layer accepts all requests; BT decides whether to proceed
5. **Nav2 Integration**: Leverages Nav2's NavigateToPose action for actual path planning and control
6. **YAML-Based Configuration**: Location poses loaded from config files by BT nodes, not by executor
7. **Executor Simplicity**: Mission executor remains a pure BT runner with no control logic

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

**Responsibilities:**
- Listen for StartMission service requests
- Create and configure BehaviorTree blackboard with mission parameters
- Load BehaviorTree from XML file
- Execute BT with configurable tick rate
- Monitor BT execution status (SUCCESS/FAILURE/RUNNING)
- Handle timeouts and exceptions
- Return mission result to caller

**Key Parameters:**
```yaml
bt_xml_path: "/path/to/same_floor_nav.xml"  # BT XML file path
plugin_lib_names:                            # List of BT plugin libraries
  - nav2_compute_path_to_pose_action_bt_node
  - nav2_navigate_to_pose_action_bt_node
  - smrr_bt_nodes                           # Custom plugin library
bt_tick_rate_hz: 20.0                       # BT tick frequency
bt_timeout_sec: 300.0                       # Maximum execution time
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
}
```

### 6. BehaviorTree XML: same_floor_nav.xml

**Package:** `smrr_navigation`  
**File:** `config/bt/same_floor_nav.xml`

```xml
<root main_tree_to_execute="SameFloorNavigation">
  <BehaviorTree ID="SameFloorNavigation">
    <Sequence>
      <IsSameFloor 
        current_floor="{current_floor_id}" 
        target_floor="{target_floor_id}"/>
      <NavigateToPose 
        goal="{final_pose}"/>
    </Sequence>
  </BehaviorTree>
</root>
```

**Logic Flow:**
1. **IsSameFloor** evaluates floor equality
   - If SUCCESS → proceed to NavigateToPose
   - If FAILURE → entire Sequence fails (cross-floor not supported)
2. **NavigateToPose** sends goal to Nav2
   - Uses Nav2's `/navigate_to_pose` action server
   - Returns SUCCESS when robot reaches goal
   - Returns FAILURE if navigation fails

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
floor0:
  dock:
    position: [2.23, -1.00, 0.0]
  left_back:
    position: [-9.90, 1.33, 0.0]
  elevator_staging:
    position: [-2.00, 1.30, 90.0]
floor1:
  office_101:
    position: [7.93, -4.29, 0.0]
  # ... more locations
```

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
   ├─> Loads BT from same_floor_nav.xml
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
   ├─> Fallback tries CrossFloor_StagingOnly sequence
   ├─> IsDifferentFloor: "floor0" != "floor1" → SUCCESS
   ├─> GetNamedPose:
   │   ├─> Loads locations.yaml
   │   ├─> Retrieves floors["floor0"]["locations"]["elevator_staging"]
   │   └─> Sets staging_pose on blackboard → SUCCESS
   └─> NavigateToPose to staging_pose:
       ├─> Nav2 navigates to elevator staging on current floor
       └─> Returns SUCCESS (robot at staging area)

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
├── config/
│   └── bt/
│       └── same_floor_nav.xml
├── include/
│   └── smrr_navigation/
│       └── bt_nodes/
│           └── is_same_floor_condition.hpp
├── src/
│   ├── bt_nodes/
│   │   └── is_same_floor_condition.cpp
│   └── smrr_bt_mission_executor.cpp
├── smrr_navigation/         # Python package
│   ├── __init__.py
│   ├── named_goal_server.py
│   ├── location_subscriber.py
│   └── named_goal_client.py
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
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(smrr_interfaces REQUIRED)

# Build custom BT plugin library
add_library(smrr_bt_nodes SHARED
  src/bt_nodes/is_same_floor_condition.cpp
  src/bt_nodes/is_different_floor_condition.cpp
  src/bt_nodes/get_named_pose_action.cpp
  src/bt_nodes/bt_node_registration.cpp
)
ament_target_dependencies(smrr_bt_nodes
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)
target_link_libraries(smrr_bt_nodes
  yaml-cpp
)

# Build mission executor executable
add_executable(smrr_bt_mission_executor
  src/smrr_bt_mission_executor.cpp
)
ament_target_dependencies(smrr_bt_mission_executor
  rclcpp nav2_behavior_tree geometry_msgs
  tf2 tf2_geometry_msgs smrr_interfaces ament_index_cpp
)

# Install C++ targets
install(TARGETS smrr_bt_nodes
  LIBRARY DESTINATION lib
)
install(TARGETS smrr_bt_mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  smrr_navigation/named_goal_server.py
  smrr_navigation/location_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install configs and launch files
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

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

# Cross-floor navigation (navigates to staging on current floor)
ros2 topic pub --once /location std_msgs/String "{data: 'office_101'}"  # floor1 location
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

## Testing Cross-Floor Staging

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

**Test Case 2:** Cross-floor navigation (navigates to staging)

```bash
ros2 service call /start_mission smrr_interfaces/srv/StartMission \
  "{mission_id: 'cross-floor-test', \
    current_floor_id: 'floor0', \
    target_floor_id: 'floor1', \
    target_location_name: 'office_101', \
    x: 7.93, y: -4.29, yaw: 0.0}"
```

**Expected Result:**
```
[IsSameFloor] FAILURE: Current floor 'floor0' != target floor 'floor1'
[IsDifferentFloor] SUCCESS: Current floor 'floor0' != target floor 'floor1'
[GetNamedPose] Looking up: floor_id='floor0', location_key='elevator_staging'
[GetNamedPose] Loaded and cached YAML from: .../locations.yaml
[GetNamedPose] Found pose: x=-2, y=1.3, yaw=1.57 rad
[GetNamedPose] SUCCESS: Set output pose for elevator_staging
[bt_navigator]: Begin navigating from current location to (-2.00, 1.30)
[controller_server]: Reached the goal!
[bt_navigator]: Goal succeeded
[smrr_bt_mission_executor]: BT execution SUCCESS
```

**Behavior:** 
- Robot detects different floor requirement
- Retrieves elevator_staging pose from **current floor** (floor0)
- Navigates to staging area successfully
- BT returns SUCCESS (robot ready for elevator entry)
- Note: Robot is at staging but not at final destination (office_101)

---

## Performance Characteristics

- **BT Tick Rate:** 20 Hz (configurable)
- **Service Response Time:** ~10-100ms (depends on BT creation overhead)
- **Navigation Timeout:** 300 seconds (5 minutes, configurable)
- **Maximum Concurrent Missions:** 1 (service-based, sequential execution)

---

## Future Enhancements

### Planned Features
1. **Multi-Floor Navigation BT:** Add elevator control nodes to BT for cross-floor missions
2. **Recovery Behaviors:** Integrate Nav2 recovery behaviors into BT
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
smrr_navigation/config/bt/same_floor_nav.xml
smrr_navigation/include/smrr_navigation/bt_nodes/is_same_floor_condition.hpp
smrr_navigation/src/bt_nodes/is_same_floor_condition.cpp
smrr_navigation/include/smrr_navigation/bt_nodes/is_different_floor_condition.hpp
smrr_navigation/src/bt_nodes/is_different_floor_condition.cpp
smrr_navigation/include/smrr_navigation/bt_nodes/get_named_pose_action.hpp
smrr_navigation/src/bt_nodes/get_named_pose_action.cpp
smrr_navigation/src/bt_nodes/bt_node_registration.cpp
smrr_navigation/src/smrr_bt_mission_executor.cpp
```

### Modified Files
```
smrr_interfaces/CMakeLists.txt          # Added StartMission.srv
smrr_navigation/package.xml             # Changed to ament_cmake, added yaml-cpp
smrr_navigation/CMakeLists.txt          # Hybrid C++/Python build, new BT nodes
smrr_navigation/config/bt/same_floor_nav.xml  # Updated with Fallback + CrossFloor
smrr_navigation/smrr_navigation/named_goal_server.py  # Added BT client
smrr_navigation/launch/smrr_world_navigation.launch.py  # Launch BT executor
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

---

## Contact & Support

For questions or issues related to this implementation, refer to:
- Project repository: `Quanta-Projects/navigation_ws`
- Branch: `main`
- Implementation session: January 3, 2026

---

**End of Document**
