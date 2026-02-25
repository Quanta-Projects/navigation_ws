# `smrr_navigation` — Architecture & Implementation Reference

> **Purpose:** This document provides a comprehensive technical reference for the `smrr_navigation` ROS 2 package. It is intended to supply a reviewing AI (or human engineer) with sufficient detail to evaluate the architecture and propose algorithmic improvements. All critical logic sections include source-level code snippets or pseudocode.

---

## Table of Contents

1. [High-Level Architecture](#1-high-level-architecture)
   - 1.1 [Package Overview](#11-package-overview)
   - 1.2 [Directory Layout](#12-directory-layout)
   - 1.3 [Build System](#13-build-system)
   - 1.4 [ROS Node Graph](#14-ros-node-graph)
   - 1.5 [Custom Interfaces (`smrr_interfaces`)](#15-custom-interfaces-smrr_interfaces)
   - 1.6 [Data Flow Diagram](#16-data-flow-diagram)
2. [Multi-Floor Navigation Logic](#2-multi-floor-navigation-logic)
   - 2.1 [Floor & Map Management](#21-floor--map-management)
   - 2.2 [Elevator Transition Sequence](#22-elevator-transition-sequence)
   - 2.3 [AMCL Re-Initialization](#23-amcl-re-initialization)
   - 2.4 [Legacy Python Action Server vs. C++ BT Executor](#24-legacy-python-action-server-vs-c-bt-executor)
3. [Behavior Tree Architecture](#3-behavior-tree-architecture)
   - 3.1 [BT XML Structure (`smrr_multifloor.xml`)](#31-bt-xml-structure-smrr_multifloortxml)
   - 3.2 [BT Mission Executor (`smrr_bt_mission_executor`)](#32-bt-mission-executor-smrr_bt_mission_executor)
   - 3.3 [Custom BT Node Reference](#33-custom-bt-node-reference)
   - 3.4 [Plugin Registration](#34-plugin-registration)
4. [Algorithms & Data Processing](#4-algorithms--data-processing)
   - 4.1 [Path Planning — NavfnPlanner (Dijkstra)](#41-path-planning--navfnplanner-dijkstra)
   - 4.2 [Local Control — DWB Local Planner](#42-local-control--dwb-local-planner)
   - 4.3 [Door Detection — Three Methods](#43-door-detection--three-methods)
   - 4.4 [Startup Localization Sequence](#44-startup-localization-sequence)
   - 4.5 [Named Goal Resolution](#45-named-goal-resolution)
5. [Configurations & Parameters](#5-configurations--parameters)
   - 5.1 [Navigation Parameters (`smrr_nav_params.yaml`)](#51-navigation-parameters-smrr_nav_paramsyaml)
   - 5.2 [Locations Database (`locations.yaml`)](#52-locations-database-locationsyaml)
6. [Launch & Deployment](#6-launch--deployment)
   - 6.1 [Launch File Hierarchy](#61-launch-file-hierarchy)
   - 6.2 [Runtime Dependencies & Hardware](#62-runtime-dependencies--hardware)

---

## 1. High-Level Architecture

### 1.1 Package Overview

`smrr_navigation` is a hybrid C++/Python ROS 2 package built on **Nav2** (Navigation2) that adds **multi-floor elevator navigation** to a differential-drive mobile robot. Key capabilities:

| Capability | Implementation |
|---|---|
| Same-floor point-to-point navigation | Nav2 `NavigateToPose` action (AMCL + NavfnPlanner + DWB) |
| Cross-floor elevator navigation | Custom BT sequence with 12 BT nodes |
| Elevator door detection | Three methods: LiDAR scan, depth baseline, ONNX TinyCNN classifier |
| Map switching at runtime | `nav2_msgs/srv/LoadMap` service, open/closed map variants per floor |
| Named location resolution | YAML-backed service (`/go_to_pose`) with floor-aware lookup |
| Startup localization | Odometry-driven forward+rotate sequence for AMCL convergence |

**Framework stack:**

```
ROS 2 Humble  →  Nav2 (AMCL, NavfnPlanner, DWB, Costmap2D)
              →  BehaviorTree.CPP v3  (mission orchestration)
              →  ONNX Runtime 1.18  (door classifier inference)
              →  Gazebo Classic  (elevator simulation via gz-11.14.0 CLI)
```

### 1.2 Directory Layout

```
smrr_navigation/
├── behavior_trees/
│   └── smrr_multifloor.xml          # Main BT XML for mission execution
├── config/
│   ├── locations.yaml                # Floor/location database (4 floors)
│   ├── smrr_nav_params.yaml          # Nav2 parameter file (AMCL, DWB, costmaps, planner)
│   └── smrr_nav.rviz                 # RViz configuration
├── include/smrr_navigation/bt_nodes/ # C++ BT node headers (13 files)
├── launch/
│   └── smrr_world_navigation.launch.py
├── maps/                             # Occupancy grid maps (7 YAML+PGM pairs)
│   ├── floor0_open.yaml / first_floor_with_docking_station.yaml
│   ├── floor1_open.yaml / second_floor.yaml
│   ├── third_floor.yaml              # floor2 (open=closed)
│   └── fourth_floor.yaml             # floor3 (open=closed)
├── models/
│   ├── door_classifier.onnx          # Door classifier model (legacy)
│   └── door_classifier_3.onnx        # Door classifier model (active, TinyCNN ~15K params)
├── smrr_navigation/                  # Python package
│   ├── __init__.py
│   ├── depth_preprocess_spec.py      # Canonical depth preprocessing specification
│   ├── door_classifier_node.py       # Standalone ONNX inference ROS node
│   ├── location_subscriber.py        # Topic→service bridge (/location → /go_to_pose)
│   ├── named_goal_client.py          # CLI client for /go_to_pose
│   ├── named_goal_server.py          # Service server: name→pose resolution + dispatch
│   ├── smrr_multifloor_bt_navigator.py  # Legacy Python action server (1563 lines)
│   └── startup_localizer.py          # AMCL convergence helper (drive+rotate)
├── src/
│   ├── bt_nodes/                     # C++ BT node implementations (13 files)
│   │   ├── bt_node_registration.cpp      # Plugin export (BT_RegisterNodesFromPlugin)
│   │   ├── is_same_floor_condition.cpp
│   │   ├── is_different_floor_condition.cpp
│   │   ├── get_named_pose_action.cpp
│   │   ├── get_named_map_action.cpp
│   │   ├── switch_map_action.cpp
│   │   ├── publish_initial_pose_action.cpp
│   │   ├── call_elevator_action.cpp
│   │   ├── wait_for_door_open_action.cpp
│   │   ├── wait_for_door_open_depth_action.cpp
│   │   ├── wait_for_door_open_model_action.cpp
│   │   ├── update_pose_timestamp_action.cpp
│   │   └── stop_robot_action.cpp
│   └── smrr_bt_mission_executor.cpp  # Service server that loads & ticks the BT
├── CMakeLists.txt                    # Hybrid ament_cmake + ament_cmake_python
├── package.xml
└── setup.py                          # Python entry points (6 executables)
```

### 1.3 Build System

The package uses **hybrid `ament_cmake` + `ament_cmake_python`** to build both C++ and Python targets:

**C++ targets:**

| Target | Type | Description |
|---|---|---|
| `smrr_bt_nodes` | Shared library (`SHARED`) | 12 custom BT node implementations, registered as a BT plugin |
| `smrr_bt_mission_executor` | Executable | Service server that loads BT XML and ticks the tree |

**C++ dependencies:** `rclcpp`, `behaviortree_cpp_v3`, `nav2_behavior_tree`, `nav2_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2`, `tf2_geometry_msgs`, `smrr_interfaces`, `ament_index_cpp`, `yaml-cpp`, ONNX Runtime 1.18

**Python entry points (from `setup.py`):**

| Executable | Source |
|---|---|
| `startup_localizer.py` | Startup AMCL convergence helper |
| `named_goal_server.py` | `/go_to_pose` service resolver/dispatcher |
| `named_goal_client.py` | CLI client |
| `location_subscriber.py` | Topic→service bridge |
| `smrr_multifloor_bt_navigator.py` | Legacy action server |
| `door_classifier_node.py` | Standalone ONNX door classifier ROS node |

### 1.4 ROS Node Graph

Nodes launched by `smrr_world_navigation.launch.py`:

```
┌──────────────────────────────────────────────────────────────────────┐
│  Gazebo Classic  (external, provides /scan, /zed2_left_camera/depth)│
└──┬───────────────────────────────────────────────────────────────────┘
   │
   ├── smrr_description/gazebo_classic_controllers.launch.py
   │     └── diff_drive_controller (publishes /diff_drive_controller/odom)
   │
   ├── nav2_bringup/bringup_launch.py
   │     ├── map_server          ← /map_server/load_map (LoadMap service)
   │     ├── amcl                ← /scan, publishes /amcl_pose, /initialpose
   │     ├── planner_server      ← NavfnPlanner (Dijkstra)
   │     ├── controller_server   ← DWB local planner → /cmd_vel
   │     ├── bt_navigator        ← /navigate_to_pose (Nav2 action)
   │     ├── recoveries_server   ← spin, backup, wait recoveries
   │     ├── local_costmap       ← obstacle+voxel+inflation layers
   │     └── global_costmap      ← static+obstacle+voxel+inflation layers
   │
   ├── smrr_bt_mission_executor  ← /start_mission (StartMission service)
   │     Loads smrr_multifloor.xml, ticks BT at 20 Hz, 300s timeout
   │
   ├── named_goal_server         ← /go_to_pose (GoToNamedPose service)
   │     Resolves named locations → dispatches to /start_mission
   │
   ├── location_subscriber       ← /location (String topic) → /go_to_pose
   │
   ├── startup_localizer (conditional)
   │     Drives forward 1m + rotates 360° for AMCL convergence
   │
   └── rviz2
```

### 1.5 Custom Interfaces (`smrr_interfaces`)

#### `StartMission.srv`

```yaml
# Request
string   mission_id
string   current_floor_id
string   target_floor_id
string   target_location_name
float64  x
float64  y
float64  yaw
---
# Response
bool     accepted
bool     success
int32    nav_status
string   message
```

Called by `named_goal_server` → handled by `smrr_bt_mission_executor`. The executor populates the BT blackboard with `current_floor_id`, `target_floor_id`, and a `PoseStamped` built from `(x, y, yaw)` and ticks the behavior tree.

#### `GoToNamedPose.srv`

```yaml
# Request
string name          # Named location, e.g. "office_101"
---
# Response
bool   accepted
string message
```

Called by external systems → handled by `named_goal_server`. The server resolves the name from `locations.yaml`, determines the current floor and target floor, then dispatches a `StartMission` call.

#### `NavigateToNamedLocation.action`

```yaml
# Goal
string   location_name
string   target_floor_id
float64  x
float64  y
float64  yaw
---
# Result
bool     success
int32    nav_status
string   message
---
# Feedback
string   state
string   active_floor_id
string   active_step
```

Used by the legacy Python action server (`smrr_multifloor_bt_navigator.py`). When `use_bt_mission_executor=True` (default), this action is bypassed in favor of `StartMission`.

### 1.6 Data Flow Diagram

```
User/External ──"office_101"──→  /location (String topic)
                                    │
                            location_subscriber
                                    │
                            /go_to_pose (GoToNamedPose srv)
                                    │
                          named_goal_server
                          │ 1. Resolve name → (floor_id, x, y, yaw)
                          │ 2. Determine current_floor_id
                          │ 3. Generate mission_id (UUID)
                                    │
                          /start_mission (StartMission srv)
                                    │
                       smrr_bt_mission_executor
                       │ 1. Set blackboard: {current_floor_id, target_floor_id, final_pose}
                       │ 2. Load smrr_multifloor.xml
                       │ 3. Tick tree at 20 Hz
                                    │
                ┌───────────────────┴───────────────────┐
          IsSameFloor?                          IsDifferentFloor?
                │                                       │
        NavigateToPose                     [ElevatorSequence]
        (Nav2 action)                    GetNamedPose → NavigateToPose
                                         CallElevator → WaitForDoorOpenModel
                                         NavigateToPose(inside) → Spin(-π)
                                         SwitchMap → PublishInitialPose
                                         ClearCostmaps → NavigateToPose(exit)
                                         NavigateToPose(final)
```

---

## 2. Multi-Floor Navigation Logic

### 2.1 Floor & Map Management

The system supports **4 floors** (`floor0`–`floor3`), each with two map variants:

| Map Variant | Purpose |
|---|---|
| `open` | Elevator door region is free space — used when the robot is **inside the elevator** or actively transitioning |
| `closed` | Elevator door region is occupied — used for **normal navigation** on a floor |

The `locations.yaml` database stores per-floor:
- **Maps:** `{closed: <yaml_file>, open: <yaml_file>}` (some floors share the same file for both)
- **Locations:** Named `{x, y, yaw}` poses including mandatory elevator waypoints:
  - `elevator_staging` — position in front of the elevator door
  - `elevator_inside` — position inside the elevator car
  - `elevator_exit` — position just outside the elevator on the target floor
  - `amcl_initial_pose_open` / `amcl_initial_pose_closed` — AMCL seed poses for each map variant

**Map switching** is performed by the `SwitchMap` BT node, which calls the Nav2 `map_server/load_map` service asynchronously:

```cpp
// SwitchMapAction::onStart() — async service call
auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
request->map_url = map_yaml.value();
future_ = load_map_client_->async_send_request(request).future.share();
```

The BT node returns `RUNNING` while waiting, polls with `wait_for(0ms)` each tick, and validates the response code on completion. The default timeout is 10,000 ms.

### 2.2 Elevator Transition Sequence

The cross-floor elevator sequence in `smrr_multifloor.xml` follows this exact step order:

```
 1. IsDifferentFloor ─────────── Guard: only runs if floors differ
 2. GetNamedPose(elevator_staging) → NavigateToPose
                                      Navigate to staging position in front of elevator
 3. RetryUntilSuccessful(2) ──── Retry wrapper for elevator entry:
    ├─ CallElevator(current_floor)   Send Gazebo CLI command
    ├─ WaitForDoorOpenModel          ONNX classifier waits for door (timeout=1500s)
    ├─ GetNamedPose(elevator_inside) → NavigateToPose
    │                                  Drive into elevator
    ├─ Spin(-π)                      Rotate 180° to face the door
    └─ StopRobot                     Zero velocity for 800ms
 4. GetNamedMap(target_floor, "open")
 5. SwitchMap ────────────────── Load target floor's open map
 6. PublishInitialPose ────────── Seed AMCL with known elevator position
 7. ClearEntireCostmap (local)
 8. ClearEntireCostmap (global) ── Remove stale obstacle data
 9. Fallback(ExitElevator) ──── Exit with retry:
    ├─ Attempt 1:
    │   ├─ CallElevator(target_floor)
    │   ├─ WaitForDoorOpenModel (stale tolerance=10s)
    │   └─ NavigateToPose(elevator_exit)
    └─ Attempt 2 (reposition):
        ├─ NavigateToPose(elevator_inside)
        ├─ Spin(-3.54 rad ≈ -203°)
        ├─ StopRobot
        ├─ CallElevator(target_floor)
        ├─ WaitForDoorOpenModel
        └─ NavigateToPose(elevator_exit)
10. NavigateToPose(final_pose) ── Navigate to desired destination
```

**Key design decisions visible in the BT XML:**

| Decision | Value/Rationale |
|---|---|
| `WaitForDoorOpenModel` timeout | 1500 seconds — extremely long to handle real-world elevator wait times |
| `max_depth_stale_sec` at exit | 10s (vs 1s at entry) — depth camera may lose signal inside elevator |
| Exit retry spin angle | -3.54 rad (>π) — ensures the robot achieves a full turnaround even if pose is slightly off |
| `stable_time_sec` | 1.0s for all WaitForDoorOpen calls — temporal hysteresis to avoid false positives |

### 2.3 AMCL Re-Initialization

When the robot changes floors, the map is replaced and AMCL must be re-seeded. The `PublishInitialPose` BT node publishes a `PoseWithCovarianceStamped` to `/initialpose`:

```cpp
// PublishInitialPoseAction::tick()
pose_msg.pose.covariance[0]  = 0.25;    // σ²_x  = 0.25 m² (σ = 0.5m)
pose_msg.pose.covariance[7]  = 0.25;    // σ²_y  = 0.25 m²
pose_msg.pose.covariance[35] = 0.0685;  // σ²_yaw = 0.0685 rad² (σ ≈ 15°)
```

After publishing, two `ClearEntireCostmap` calls (local + global) ensure stale obstacle data from the previous floor does not corrupt planning.

**AMCL configuration for re-localization** (from `smrr_nav_params.yaml`):

```yaml
amcl:
  max_particles: 5000          # Large particle count for robust convergence
  min_particles: 1000
  recovery_alpha_fast: 0.1     # Enabled (was 0.0) for adaptive recovery
  recovery_alpha_slow: 0.001
  initial_cov_xx: 2.0          # Wide initial spread
  initial_cov_yy: 2.0
  initial_cov_aa: 0.03         # ±10° angular spread
```

### 2.4 Legacy Python Action Server vs. C++ BT Executor

Two complete implementations of cross-floor navigation exist:

| Aspect | Python Action Server | C++ BT Executor |
|---|---|---|
| File | `smrr_multifloor_bt_navigator.py` (1563 lines) | `smrr_bt_mission_executor.cpp` (224 lines) + BT XML + 12 BT nodes |
| Interface | `NavigateToNamedLocation` action | `StartMission` service |
| Dispatch mode | `use_bt_mission_executor=False` | `use_bt_mission_executor=True` (default) |
| Door detection | LiDAR scan fraction only | ONNX model (`WaitForDoorOpenModel`) |
| Retry logic | Hardcoded in Python sequences | Declarative in BT XML (`RetryUntilSuccessful`, `Fallback`) |
| Elevator control | Subprocess call to `gz-11.14.0` | Same (`CallElevator` BT node wraps subprocess) |

The BT-based executor is the **active default**. The Python action server is retained as a legacy fallback.

---

## 3. Behavior Tree Architecture

### 3.1 BT XML Structure (`smrr_multifloor.xml`)

Full BT XML (112 lines):

```xml
<root main_tree_to_execute="MissionTree">
  <BehaviorTree ID="MissionTree">
    <Fallback name="Root">

      <!-- Branch 1: Same-floor — simple NavigateToPose -->
      <Sequence name="SameFloor">
        <IsSameFloor current_floor="{current_floor_id}" target_floor="{target_floor_id}"/>
        <NavigateToPose server_name="/navigate_to_pose" goal="{final_pose}"/>
      </Sequence>

      <!-- Branch 2: Cross-floor — full elevator sequence -->
      <Sequence name="CrossFloor_ElevatorEntry">
        <IsDifferentFloor current_floor="{current_floor_id}" target_floor="{target_floor_id}"/>

        <!-- Stage at elevator -->
        <GetNamedPose floor_id="{current_floor_id}" location_key="elevator_staging"
                      pose="{staging_pose}"/>
        <NavigateToPose server_name="/navigate_to_pose" goal="{staging_pose}"/>

        <!-- Enter elevator (with retry) -->
        <RetryUntilSuccessful num_attempts="2" name="CallElevatorAndEnter_Twice">
          <Sequence name="CallWaitAndEnter">
            <CallElevator floor_id="{current_floor_id}"/>
            <WaitForDoorOpenModel
              depth_topic="/zed2_left_camera/depth/image_raw"
              timeout_sec="1500.0" stable_time_sec="1.0"
              clip_min_m="0.2" clip_max_m="5.0"
              open_index="1" threshold="0.7"/>
            <GetNamedPose floor_id="{current_floor_id}" location_key="elevator_inside"
                          pose="{inside_pose}"/>
            <NavigateToPose server_name="/navigate_to_pose" goal="{inside_pose}"/>
            <Spin spin_dist="-3.1416" time_allowance="10.0" is_recovery="true"/>
            <StopRobot topic="/cmd_vel" repeat_ms="200" duration_ms="800"/>
          </Sequence>
        </RetryUntilSuccessful>

        <!-- Switch to target floor's open map -->
        <GetNamedMap floor_id="{target_floor_id}" map_key="open"
                     map_yaml="{map_open_target}"/>
        <GetNamedPose floor_id="{target_floor_id}" location_key="amcl_initial_pose_open"
                      pose="{amcl_open_target}"/>
        <SwitchMap map_yaml="{map_open_target}"/>
        <PublishInitialPose initial_pose="{amcl_open_target}"/>
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>

        <!-- Exit elevator (with fallback retry) -->
        <Fallback name="ExitElevator_WithRepositionRetry">
          <Sequence name="ExitAttempt_1">
            <CallElevator floor_id="{target_floor_id}"/>
            <WaitForDoorOpenModel ... max_depth_stale_sec="10.0" .../>
            <GetNamedPose floor_id="{target_floor_id}" location_key="elevator_exit"
                          pose="{exit_pose}"/>
            <NavigateToPose server_name="/navigate_to_pose" goal="{exit_pose}"/>
          </Sequence>
          <Sequence name="RepositionThenExitAttempt_2">
            <!-- Reposition inside elevator -->
            <GetNamedPose floor_id="{target_floor_id}" location_key="elevator_inside"
                          pose="{inside_pose}"/>
            <NavigateToPose server_name="/navigate_to_pose" goal="{inside_pose}"/>
            <Spin spin_dist="-3.5416" time_allowance="10.0" is_recovery="true"/>
            <StopRobot .../>
            <!-- Retry exit -->
            <CallElevator floor_id="{target_floor_id}"/>
            <WaitForDoorOpenModel .../>
            <GetNamedPose floor_id="{target_floor_id}" location_key="elevator_exit"
                          pose="{exit_pose}"/>
            <NavigateToPose server_name="/navigate_to_pose" goal="{exit_pose}"/>
          </Sequence>
        </Fallback>

        <!-- Final navigation to destination -->
        <NavigateToPose server_name="/navigate_to_pose" goal="{final_pose}"/>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>
```

**Tree control flow summary:**

```
Root [Fallback]
 ├── SameFloor [Sequence]
 │    ├── IsSameFloor [Condition]        → SUCCESS if floors match
 │    └── NavigateToPose                 → Nav2 action
 └── CrossFloor [Sequence]
      ├── IsDifferentFloor [Condition]   → SUCCESS if floors differ
      ├── Navigate to staging
      ├── RetryUntilSuccessful(2)        → elevator entry + spin
      ├── SwitchMap + PublishInitialPose + ClearCostmaps
      ├── Fallback [exit retry]
      │    ├── ExitAttempt_1             → direct exit
      │    └── ExitAttempt_2             → reposition + retry
      └── NavigateToPose(final)
```

### 3.2 BT Mission Executor (`smrr_bt_mission_executor`)

The executor is a standalone ROS 2 node providing the `/start_mission` service. Key implementation:

```cpp
class BtMissionExecutor : public rclcpp::Node {
  // ...
  void handleStartMission(Request, Response) {
    // 1. Validate request (non-empty floor IDs, finite pose values)
    // 2. Build PoseStamped from (x, y, yaw)
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, request->yaw);

    // 3. Create BT engine and blackboard
    nav2_behavior_tree::BehaviorTreeEngine bt_engine(plugin_lib_names_);
    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
    blackboard->set("current_floor_id", request->current_floor_id);
    blackboard->set("target_floor_id",  request->target_floor_id);
    blackboard->set("final_pose",       final_pose);

    // 4. Tick tree at 20 Hz with 300s timeout
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      status = tree.tickRoot();
      std::this_thread::sleep_for(50ms);  // 1000/20Hz
    }
  }
};

// Uses MultiThreadedExecutor + ReentrantCallbackGroup
// to allow BT nodes to make service calls during tree execution
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

**Plugin loading** (from launch file):

```python
'plugin_lib_names': [
    # 25 Nav2 standard BT plugins
    'nav2_compute_path_to_pose_action_bt_node',
    'nav2_follow_path_action_bt_node',
    'nav2_navigate_to_pose_action_bt_node',
    # ... (full list in launch file)
    # Custom BT plugin library
    'smrr_bt_nodes'
]
```

### 3.3 Custom BT Node Reference

All 12 registered BT nodes, with their types, ports, and algorithmic behavior:

---

#### `IsSameFloor` — ConditionNode

**Purpose:** Guard for same-floor navigation branch.

```cpp
BT::NodeStatus tick() {
  auto current = getInput<std::string>("current_floor");
  auto target  = getInput<std::string>("target_floor");
  return (current.value() == target.value())
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
```

| Port | Direction | Type | Default |
|---|---|---|---|
| `current_floor` | Input | `string` | — |
| `target_floor` | Input | `string` | — |

---

#### `IsDifferentFloor` — ConditionNode

Complement of `IsSameFloor`. Returns `SUCCESS` when floors differ.

---

#### `GetNamedPose` — SyncActionNode

**Purpose:** Look up a named location from `locations.yaml` and output a `PoseStamped`.

**Key implementation detail — YAML caching:**

```cpp
// Static cache shared across all GetNamedPose instances
static std::map<std::string, YAML::Node> yaml_cache_;
static std::mutex cache_mutex_;

YAML::Node loadYamlFile(const std::string& file_path) {
  std::lock_guard<std::mutex> lock(cache_mutex_);
  auto it = yaml_cache_.find(file_path);
  if (it != yaml_cache_.end()) return it->second;  // Cache hit
  YAML::Node yaml = YAML::LoadFile(file_path);
  yaml_cache_[file_path] = yaml;
  return yaml;
}
```

**YAML path:** `floors[floor_id]["locations"][location_key]` → `{x, y, yaw}`

| Port | Direction | Type | Default |
|---|---|---|---|
| `floor_id` | Input | `string` | — |
| `location_key` | Input | `string` | — |
| `global_frame` | Input | `string` | `"map"` |
| `locations_file` | Input | `string` | `"locations.yaml"` |
| `pose` | Output | `PoseStamped` | — |

---

#### `GetNamedMap` — SyncActionNode

**Purpose:** Look up a map YAML path from `locations.yaml`. Resolves relative paths to absolute paths via `ament_index_cpp::get_package_share_directory`.

**YAML path:** `floors[floor_id]["maps"][map_key]` → filename → `<pkg_share>/maps/<filename>`

| Port | Direction | Type | Default |
|---|---|---|---|
| `floor_id` | Input | `string` | — |
| `map_key` | Input | `string` | — |
| `locations_file` | Input | `string` | `"locations.yaml"` |
| `map_yaml` | Output | `string` | — |

---

#### `SwitchMap` — StatefulActionNode

**Purpose:** Asynchronously call `nav2_msgs/srv/LoadMap` to replace the active map.

| Port | Direction | Type | Default |
|---|---|---|---|
| `map_yaml` | Input | `string` | — |
| `service_name` | Input | `string` | `"/map_server/load_map"` |
| `timeout_ms` | Input | `int` | `10000` |

**State machine:**
- `onStart()` → creates service client, sends async request, returns `RUNNING`
- `onRunning()` → polls future with `wait_for(0ms)`, checks timeout, returns `SUCCESS`/`FAILURE`/`RUNNING`

---

#### `PublishInitialPose` — SyncActionNode

**Purpose:** Publish `PoseWithCovarianceStamped` to `/initialpose` for AMCL re-initialization.

| Port | Direction | Type | Default |
|---|---|---|---|
| `initial_pose` | Input | `PoseStamped` | — |
| `topic_name` | Input | `string` | `"/initialpose"` |
| `frame_id` | Input | `string` | `""` (uses pose's frame) |

**Covariance matrix** (diagonal): `[0.25, 0.25, 0, 0, 0, 0.0685]`

---

#### `CallElevator` — SyncActionNode

**Purpose:** Command the Gazebo elevator plugin by executing a CLI subprocess.

**Implementation:**

```cpp
// Floor name → Gazebo floor number
floor_map_ = {{"floor0","0"}, {"floor1","1"}, {"floor2","2"}, {"floor3","3"}};

// Build command:
// gz-11.14.0 topic -p /gazebo/default/elevator -m 'data: "0"'
cmd_ss << gz_cli << " topic -p " << gz_elevator_topic
       << " -m 'data: \"" << floor_num << "\"'";

// Execute via popen() with timeout
FILE* pipe = popen(full_cmd.c_str(), "r");
```

| Port | Direction | Type | Default |
|---|---|---|---|
| `floor_id` | Input | `string` | — |
| `gz_cli` | Input | `string` | `"gz-11.14.0"` |
| `gz_elevator_topic` | Input | `string` | `"/gazebo/default/elevator"` |
| `timeout_sec` | Input | `double` | `5.0` |

---

#### `WaitForDoorOpen` — StatefulActionNode (LiDAR-based)

**Purpose:** Detect elevator door opening using LiDAR scan data.

**Algorithm:**
1. Subscribe to `/scan` (`LaserScan`)
2. Define angular window: `[center - width/2, center + width/2]` degrees
3. Count fraction of rays in window where `range > range_threshold_m`
4. Door is "open" when: `fraction_of_open_rays ≥ fraction_threshold`
5. Require stability: condition must hold for `stable_time_sec` continuously

| Port | Direction | Type | Default |
|---|---|---|---|
| `scan_topic` | Input | `string` | `"/scan"` |
| `timeout_sec` | Input | `double` | `30.0` |
| `window_center_deg` | Input | `double` | `0.0` |
| `window_width_deg` | Input | `double` | `30.0` |
| `range_threshold_m` | Input | `double` | `2.0` |
| `fraction_threshold` | Input | `double` | `0.6` |
| `stable_time_sec` | Input | `double` | `1.0` |
| `poll_rate_hz` | Input | `double` | `10.0` |
| `max_scan_stale_sec` | Input | `double` | `1.0` |

---

#### `WaitForDoorOpenDepth` — StatefulActionNode (Depth-based)

**Purpose:** Detect door opening using depth camera data with a statistical baseline approach.

**Algorithm:**
1. **Baseline phase:** Collect `baseline_frames` (default 15) depth images. For each, compute mean and std of depth in a configurable ROI (fractions of image). Store medians as `baseline_mean_depth_` and `baseline_std_depth_`.
2. **Detection phase (dual condition):**
   - **Plane std condition:** Current std > `baseline_std × plane_std_drop_ratio` — door region becomes geometrically diverse
   - **Free-space condition:** Fraction of pixels where `depth > baseline_mean + free_space_delta_m` exceeds `free_space_fraction_threshold` — a corridor appears behind the door
3. By default `require_both_conditions=true`. If baseline std exceeds `plane_std_closed_max`, it falls back to requiring only the free-space condition.
4. **Stability:** Condition must hold for `stable_time_sec`.

| Port | Direction | Type | Default |
|---|---|---|---|
| `depth_topic` | Input | `string` | `"/camera/depth/image_raw"` |
| `roi_x_min` / `roi_x_max` | Input | `double` | `0.30` / `0.70` |
| `roi_y_min` / `roi_y_max` | Input | `double` | `0.20` / `0.85` |
| `baseline_frames` | Input | `int` | `15` |
| `plane_std_closed_max` | Input | `double` | `0.08` |
| `plane_std_drop_ratio` | Input | `double` | `2.0` |
| `free_space_delta_m` | Input | `double` | `0.50` |
| `free_space_fraction_threshold` | Input | `double` | `0.45` |
| `require_both_conditions` | Input | `bool` | `true` |
| `stable_time_sec` | Input | `double` | `1.0` |
| `timeout_sec` | Input | `double` | `30.0` |

---

#### `WaitForDoorOpenModel` — StatefulActionNode (ONNX-based) **[Active in BT XML]**

**Purpose:** Detect door opening using a trained TinyCNN depth classifier via ONNX Runtime.

**Model specification:**
- Architecture: TinyCNN (~15K parameters)
- Input tensor: `depth_input` — `float32 [1, 1, 96, 96]` (NCHW)
- Output tensor: `logits` — `float32 [1, 2]` (raw logits, softmax applied in code)
- Classes: `{0: CLOSED, 1: OPEN}` (configurable via `open_index`)
- Default model file: `models/door_classifier_3.onnx`

**Preprocessing pipeline (must match training exactly):**

```cpp
// Step 1: Convert encoding (32FC1 float meters or 16UC1 uint16 mm→meters)
// Step 2: Replace invalid pixels (NaN, inf, ≤0) with clip_max_m (5.0 = FAR, NOT 0!)
if (!std::isfinite(d) || d <= 0.0f) {
    depth_out[i] = static_cast<float>(clip_max_m_);
}

// Step 3: Resize full image to 96×96 using area-based downsampling
resizeAreaDownsample(depth_m, width, height, resized, 96, 96);

// Step 4: Clip to [clip_min, clip_max] and normalize to [0, 1]
d = std::max(0.2f, std::min(d, 5.0f));
tensor[i] = (d - 0.2f) / (5.0f - 0.2f);
```

**Inference and decision logic:**

```cpp
// Softmax over 2 logits (numerically stable)
float max_logit = std::max(logits[0], logits[1]);
float exp0 = std::exp(logits[0] - max_logit);
float exp1 = std::exp(logits[1] - max_logit);
float p_open = (open_index_ == 0) ? exp0/(exp0+exp1) : exp1/(exp0+exp1);

// Temporal stability: p_open ≥ threshold for stable_time_sec continuously
if (p_open >= threshold_) {
  if (!currently_open_) { stable_start_time_ = now; currently_open_ = true; }
  if ((now - stable_start_time_).seconds() >= stable_time_sec_) return SUCCESS;
} else {
  currently_open_ = false;  // Reset stability timer
}
```

| Port | Direction | Type | Default |
|---|---|---|---|
| `depth_topic` | Input | `string` | `"/zed2_left_camera/depth/image_raw"` |
| `model_path` | Input | `string` | `""` (auto-resolve to package models/) |
| `timeout_sec` | Input | `double` | `30.0` |
| `poll_rate_hz` | Input | `double` | `10.0` |
| `max_depth_stale_sec` | Input | `double` | `1.0` |
| `stable_time_sec` | Input | `double` | `1.0` |
| `clip_min_m` | Input | `double` | `0.0` (overridden to 0.2 in BT XML) |
| `clip_max_m` | Input | `double` | `5.0` |
| `open_index` | Input | `int` | `0` (overridden to 1 in BT XML) |
| `threshold` | Input | `double` | `0.5` (overridden to 0.7 in BT XML) |

---

#### `StopRobot` — StatefulActionNode

**Purpose:** Publish zero-velocity `Twist` messages for a specified duration, used after `Spin` to ensure the robot is fully stopped.

| Port | Direction | Type | Default |
|---|---|---|---|
| `topic` | Input | `string` | `"/cmd_vel"` |
| `repeat_ms` | Input | `int` | `200` |
| `duration_ms` | Input | `int` | `800` |

Returns `RUNNING` until `duration_ms` have elapsed, publishing zero Twist every `repeat_ms`.

---

#### `UpdatePoseTimestamp` — SyncActionNode

**Purpose:** Refresh the `header.stamp` of a `PoseStamped` to the current time. Useful before sending stale poses to Nav2.

| Port | Direction | Type | Default |
|---|---|---|---|
| `input_pose` | Input | `PoseStamped` | — |
| `output_pose` | Output | `PoseStamped` | — |

---

#### `ClearEntireCostmap` — SyncActionNode

**Purpose:** Call a Nav2 costmap clear service. Used to clear both local and global costmaps after a map switch.

| Port | Direction | Type | Default |
|---|---|---|---|
| `service_name` | Input | `string` | — |

Typical usage in BT XML:
```xml
<ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
<ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
```

### 3.4 Plugin Registration

All 12 custom BT nodes are compiled into a single shared library (`libsmrr_bt_nodes.so`) and registered via the BT.CPP plugin mechanism:

```cpp
// bt_node_registration.cpp
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<IsSameFloorCondition>("IsSameFloor");
  factory.registerNodeType<IsDifferentFloorCondition>("IsDifferentFloor");
  factory.registerNodeType<GetNamedPoseAction>("GetNamedPose");
  factory.registerNodeType<GetNamedMapAction>("GetNamedMap");
  factory.registerNodeType<SwitchMapAction>("SwitchMap");
  factory.registerNodeType<PublishInitialPoseAction>("PublishInitialPose");
  factory.registerNodeType<CallElevatorAction>("CallElevator");
  factory.registerNodeType<WaitForDoorOpenAction>("WaitForDoorOpen");
  factory.registerNodeType<WaitForDoorOpenDepthAction>("WaitForDoorOpenDepth");
  factory.registerNodeType<WaitForDoorOpenModelAction>("WaitForDoorOpenModel");
  factory.registerNodeType<UpdatePoseTimestampAction>("UpdatePoseTimestamp");
  factory.registerNodeType<StopRobotAction>("StopRobot");
}
```

---

## 4. Algorithms & Data Processing

### 4.1 Path Planning — NavfnPlanner (Dijkstra)

The global planner uses Nav2's `NavfnPlanner` with **Dijkstra's algorithm** (A* is disabled):

```yaml
planner_server:
  GridBased:
    plugin: "nav2_navfn_planner/NavfnPlanner"
    tolerance: 0.5          # Goal tolerance in cells (0.5m at 0.05m/cell = 10 cells)
    use_astar: false        # Dijkstra guarantees optimal path
    allow_unknown: True     # Can plan through unknown space
```

The planner operates over the **global costmap** at 0.05m resolution with layers:
- `StaticLayer` — occupancy grid from the loaded map
- `ObstacleLayer` — real-time LiDAR obstacles
- `VoxelLayer` — 3D voxel representation (16 voxels × 0.05m = 0.8m height)
- `InflationLayer` — radius 0.4m, cost scaling factor 4.0

### 4.2 Local Control — DWB Local Planner

The local controller uses `dwb_core::DWBLocalPlanner` with 7 trajectory critics:

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  max_vel_x: 0.35           # Max forward velocity (m/s)
  max_vel_theta: 0.75       # Max angular velocity (rad/s)
  acc_lim_x: 2.5            # Linear acceleration limit
  acc_lim_theta: 2.5        # Angular acceleration limit
  vx_samples: 20            # Forward velocity samples
  vtheta_samples: 50        # Angular velocity samples (fine resolution)
  sim_time: 1.7             # Trajectory rollout time (seconds)

  critics: ["RotateToGoal", "Oscillation", "BaseObstacle",
            "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

  PathAlign.scale: 32.0     # Path alignment weight
  PathDist.scale: 32.0      # Path distance weight
  GoalAlign.scale: 24.0     # Goal alignment weight
  GoalDist.scale: 24.0      # Goal distance weight
  RotateToGoal.scale: 48.0  # Strong preference for in-place rotation at goal
  BaseObstacle.scale: 0.02  # Very low obstacle avoidance weight
  Oscillation.scale: 1.0    # Moderate oscillation penalty
```

**Goal tolerances:**

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.15   # 15cm position tolerance
  yaw_goal_tolerance: 0.17  # ~10° orientation tolerance
```

**Local costmap:** 4m × 4m rolling window (0.05m resolution) with obstacle + voxel + inflation (0.55m radius) layers.

### 4.3 Door Detection — Three Methods

The package implements three door detection approaches, with the ONNX model being the active one:

#### Method 1: LiDAR Scan Fraction (`WaitForDoorOpen`)

```
Algorithm: Angular window fraction
─────────────────────────────────
Input:   LaserScan from /scan (RPLiDAR S2L, 360°, 18m range)
Window:  [center - width/2, center + width/2] degrees (default: ±15° from forward)
Metric:  fraction = count(range > threshold) / count(valid rays in window)
Decision: fraction ≥ fraction_threshold (default: 0.6)
Stability: Must hold for stable_time_sec (default: 1.0s)
```

**Strengths:** Simple, robust to lighting conditions, fast.
**Weaknesses:** Cannot distinguish open space behind door from nearby corridor openings. Sensitive to angular alignment of robot relative to door.

#### Method 2: Depth Baseline + Free Space (`WaitForDoorOpenDepth`)

```
Algorithm: Statistical depth change detection
─────────────────────────────────────────────
Phase 1 — Baseline (15 frames):
  For each frame: compute mean/std of depth in ROI
  baseline_mean = median(all_frame_means)
  baseline_std  = median(all_frame_stds)

Phase 2 — Detection (dual condition):
  Condition A (plane_std):
    current_std > baseline_std × plane_std_drop_ratio (default: 2.0)
  Condition B (free_space):
    fraction(pixels where depth > baseline_mean + delta) > threshold
    (delta=0.50m, threshold=0.45)
  
  Door open = A AND B  (or just B if baseline_std > 0.08)

Stability: Must hold for stable_time_sec (1.0s)
```

**Strengths:** Adaptive to different door distances, detects actual depth change.
**Weaknesses:** Requires 15-frame baseline collection, sensitive to depth noise. Can false-trigger if people walk through ROI during baseline.

#### Method 3: ONNX TinyCNN Classifier (`WaitForDoorOpenModel`) **[ACTIVE]**

```
Algorithm: Trained depth classifier
──────────────────────────────────
Preprocessing:
  1. Convert depth encoding (32FC1/16UC1 → float32 meters)
  2. Invalid pixels → 5.0m (FAR, not 0.0!)
  3. Area-based downsample to 96×96 (matches cv2.INTER_AREA)
  4. Clip to [0.2, 5.0] meters
  5. Normalize: (d - 0.2) / (5.0 - 0.2) → [0, 1]
  6. Reshape to NCHW [1, 1, 96, 96]

Inference:
  logits = TinyCNN(tensor)           # [1, 2]
  probs  = softmax(logits)           # Numerically stable (max subtraction)
  p_open = probs[open_index]         # open_index=1 in BT XML

Decision:
  p_open ≥ 0.7 → OPEN → start/continue stability timer
  p_open < 0.7 → CLOSED → reset stability timer
  Stable for 1.0s → SUCCESS

Timeout: 1500s (BT XML override)
```

**Canonical preprocessing is defined in `depth_preprocess_spec.py`** and replicated in both C++ (`WaitForDoorOpenModelAction::preprocessDepth`) and Python (`door_classifier_node.py`). The canonical spec ensures training-inference parity:

```python
# depth_preprocess_spec.py — authoritative preprocessing
def preprocess_depth_training_spec(depth_m, clip_min_m=0.2, clip_max_m=5.0, out_size=96):
    # Step 1: Invalid → FAR (not 0!)
    invalid = ~np.isfinite(depth_m) | (depth_m <= 0)
    depth_m[invalid] = clip_max_m

    # Step 2: Clip
    depth_m = np.clip(depth_m, clip_min_m, clip_max_m)

    # Step 3: Normalize to [0, 1]
    depth_m = (depth_m - clip_min_m) / (clip_max_m - clip_min_m)

    # Step 4: Resize using INTER_AREA
    depth_m = cv2.resize(depth_m, (out_size, out_size), interpolation=cv2.INTER_AREA)

    # Step 5: NCHW tensor
    return depth_m.reshape(1, 1, out_size, out_size).astype(np.float32)
```

> **Note:** The C++ implementation uses a custom `resizeAreaDownsample()` that computes weighted-area averages to match `cv2.INTER_AREA` behavior without requiring OpenCV.

### 4.4 Startup Localization Sequence

The `startup_localizer.py` node helps AMCL converge at boot time by executing a simple odometry-driven motion sequence:

```
State Machine:
  INIT ──(startup_delay=2s)──→ FORWARD ──(1m @ 0.15 m/s)──→ STOP1
    ──(stop_duration=1s)──→ ROTATE ──(360° @ 0.5 rad/s)──→ STOP2
    ──(stop_duration=1s)──→ DONE (shuts down)
```

Motion is controlled purely through odometry feedback (distance via Euclidean calculation, rotation via accumulated yaw deltas with wrap-around handling). The node publishes to `/cmd_vel` and subscribes to `/diff_drive_controller/odom`. It optionally monitors `/amcl_pose` for convergence logging. This node is conditionally launched via the `enable_startup_localizer` launch argument (default: `false`).

### 4.5 Named Goal Resolution

The `named_goal_server.py` provides the `/go_to_pose` service that resolves human-readable location names:

```python
class NamedGoalServer(Node):
    def handle_go_to_pose(self, request, response):
        name = request.name  # e.g. "office_101"
        
        # 1. Search all floors for matching location
        matches = []
        for floor_id, floor_data in self.floors.items():
            if name in floor_data['locations']:
                matches.append((floor_id, floor_data['locations'][name]))
        
        # 2. Handle ambiguity (same name on multiple floors)
        if len(matches) > 1:
            # Use first match (could be improved with floor preference)
            ...
        
        # 3. Dispatch via StartMission service (BT mode)
        mission_request = StartMission.Request()
        mission_request.mission_id = str(uuid.uuid4())
        mission_request.current_floor_id = self.current_floor_id
        mission_request.target_floor_id = target_floor_id
        mission_request.x = location['x']
        mission_request.y = location['y']
        mission_request.yaw = location['yaw']
        
        result = self.start_mission_client.call(mission_request)
```

The server tracks `current_floor_id` (initialized from a launch parameter, updated after successful cross-floor missions).

---

## 5. Configurations & Parameters

### 5.1 Navigation Parameters (`smrr_nav_params.yaml`)

#### AMCL (Adaptive Monte Carlo Localization)

| Parameter | Value | Notes |
|---|---|---|
| `laser_model_type` | `likelihood_field` | Standard for structured environments |
| `laser_max_range` | `18.0` | RPLiDAR S2L specification |
| `laser_min_range` | `0.05` | RPLiDAR S2L specification |
| `max_beams` | `360` | Uses most of scan data |
| `max_particles` | `5000` | Large count for multi-floor re-localization |
| `min_particles` | `1000` | Maintains good coverage |
| `laser_likelihood_max_dist` | `1.5` | Tight field for precision |
| `sigma_hit` | `0.05` | Sharp likelihood peak |
| `recovery_alpha_fast/slow` | `0.1 / 0.001` | Active adaptive particle recovery |
| `update_min_d` | `0.25` | Update every 25cm of translation |
| `update_min_a` | `0.1` | Update every ~6° of rotation |
| `resample_interval` | `1` | Resample every update (fast convergence) |
| `transform_tolerance` | `0.2` | Tight TF sync |
| `robot_model_type` | `DifferentialMotionModel` | Matched to diff-drive kinematics |

#### DWB Local Planner

| Parameter | Value | Notes |
|---|---|---|
| `max_vel_x` | `0.35 m/s` | Conservative forward speed |
| `max_vel_theta` | `0.75 rad/s` | Moderate angular speed |
| `acc_lim_x / acc_lim_theta` | `2.5 / 2.5` | Aggressive acceleration |
| `vx_samples` | `20` | Forward velocity sampling |
| `vtheta_samples` | `50` | Fine angular sampling |
| `sim_time` | `1.7s` | Trajectory rollout horizon |
| `RotateToGoal.scale` | `48.0` | Strong in-place rotation preference |
| `BaseObstacle.scale` | `0.02` | Very low obstacle avoidance weight |
| `PathAlign/PathDist.scale` | `32.0` | Strong path following |
| `GoalAlign/GoalDist.scale` | `24.0` | Goal-seeking |
| `xy_goal_tolerance` | `0.15m` | Position tolerance |
| `yaw_goal_tolerance` | `0.17 rad` | ~10° heading tolerance |

#### Costmaps

| Layer | Local (4×4m rolling) | Global (full map) |
|---|---|---|
| `robot_radius` | `0.25m` | `0.25m` |
| `resolution` | `0.05m` | `0.05m` |
| `inflation_radius` | `0.55m` | `0.40m` |
| `cost_scaling_factor` | `5.0` | `4.0` |
| `update_frequency` | `5.0 Hz` | `1.0 Hz` |
| Layers | obstacle + voxel + inflation | static + obstacle + voxel + inflation |

#### Recovery Behaviors

```yaml
recoveries_server:
  recovery_plugins: ["spin", "backup", "wait"]
  max_rotational_vel: 1.5
  min_rotational_vel: 0.6
  rotational_acc_lim: 2.0
```

### 5.2 Locations Database (`locations.yaml`)

Structure with 4 floors:

```yaml
floors:
  floor0:
    maps:
      closed: first_floor_with_docking_station.yaml
      open: floor0_open.yaml
    locations:
      amcl_initial_pose_closed: {x: -1.5, y: 0.8, yaw: 1.57}
      amcl_initial_pose_open:   {x: -2.0, y: 1.3, yaw: 1.57}
      dock:              {x: 2.23, y: -1.0, yaw: 0.0}
      elevator_staging:  {x: -2.0, y: 1.1, yaw: 1.57}
      elevator_inside:   {x: -1.96, y: 3.00, yaw: 1.57}
      elevator_exit:     {x: -1.5, y: 0.8, yaw: 1.57}
      left_back:         {x: -9.9, y: 1.33, yaw: 0.0}
      right_back:        {x: -9.9, y: -3.0, yaw: 3.14}
      left_front:        {x: 6.15, y: 1.18, yaw: 3.14}
      right_front:       {x: 6.19, y: -2.45, yaw: 3.14}
  floor1:
    maps:
      closed: second_floor.yaml
      open: floor1_open.yaml
    locations:
      elevator_staging:  {x: -0.30, y: -0.83, yaw: 1.57}
      elevator_inside:   {x: -1.96, y: 3.00, yaw: -1.57}
      elevator_exit:     {x: -1.7197, y: 0.38080, yaw: -1.57}
      office_101:        {x: 5.896, y: -2.293, yaw: 0.0}
      office_102:        {x: 5.0, y: 2.0, yaw: 0.0}
      conference_room:   {x: -5.0, y: 3.5, yaw: 1.57}
      reception:         {x: 0.0, y: 0.0, yaw: 3.14}
  floor2:
    maps: {closed: third_floor.yaml, open: third_floor.yaml}  # Same map
    locations:
      lab_201:          {x: 4.5, y: -2.0, yaw: 0.0}
      lab_202:          {x: 6.5, y: -2.0, yaw: 0.0}
      server_room:      {x: -6.0, y: -4.0, yaw: 3.14}
      break_room:       {x: 2.0, y: 4.0, yaw: 1.57}
  floor3:
    maps: {closed: fourth_floor.yaml, open: fourth_floor.yaml}  # Same map
    locations:
      exec_office:      {x: 8.0, y: 3.0, yaw: 0.0}
      board_room:       {x: -7.0, y: 2.5, yaw: 1.57}
      rooftop_access:   {x: 0.0, y: 8.0, yaw: 0.0}
      storage:          {x: -3.0, y: -5.0, yaw: 3.14}
```

**Key observations:**
- `elevator_inside` has **opposite yaw** on floor0 (1.57) vs floors 1-3 (-1.57), reflecting different elevator orientations
- Floors 2 and 3 use the **same YAML for open and closed** maps (no elevator door modeled in the map)
- Each floor has mandatory poses: `amcl_initial_pose_open`, `amcl_initial_pose_closed`, `elevator_staging`, `elevator_inside`, `elevator_exit`

---

## 6. Launch & Deployment

### 6.1 Launch File Hierarchy

```
smrr_world_navigation.launch.py
│
├── IncludeLaunchDescription: smrr_description/gazebo_classic_controllers.launch.py
│     Spawns robot URDF in Gazebo, starts ros2_control diff_drive_controller
│     Args: use_sim_time=true
│
├── IncludeLaunchDescription: nav2_bringup/bringup_launch.py
│     Starts full Nav2 stack (map_server, amcl, planner, controller, bt_navigator, etc.)
│     Args: map=floor0_open.yaml, params_file=smrr_nav_params.yaml,
│           use_sim_time=True, autostart=True, use_composition=True
│
├── TimerAction(2s) → startup_localizer.py
│     Conditional on enable_startup_localizer launch arg (default: false)
│     Params: forward_speed=0.15, forward_distance=1.0m, rotation_speed=0.5,
│             target_rotation_angle=2π, control_period=50ms, stop_duration=1.0s
│
├── named_goal_server.py
│     Params: use_bt_mission_executor=True, initial_floor_id=<launch_arg>,
│             locations_file=locations.yaml
│     respawn=True, respawn_delay=2.0
│
├── location_subscriber.py
│     Bridge: /location (String) → /go_to_pose service
│
├── smrr_bt_mission_executor
│     Params: bt_xml_path=<pkg>/behavior_trees/smrr_multifloor.xml,
│             plugin_lib_names=[25 Nav2 plugins + "smrr_bt_nodes"],
│             bt_tick_rate_hz=20.0, bt_timeout_sec=300.0
│
└── rviz2
      Config: smrr_nav.rviz
```

**Launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `enable_startup_localizer` | `false` | Run startup AMCL convergence sequence |
| `initial_floor_id` | `floor0` | Starting floor for the `named_goal_server` |

### 6.2 Runtime Dependencies & Hardware

| Component | Specification |
|---|---|
| **ROS Distribution** | ROS 2 Humble (Ubuntu 22.04) |
| **Simulator** | Gazebo Classic (gz-11.14.0) with custom elevator plugin |
| **LiDAR** | RPLiDAR S2L — 360° FOV, 18m max range, `/scan` topic |
| **Depth Camera** | ZED2 — `/zed2_left_camera/depth/image_raw` (32FC1 or 16UC1) |
| **Drive System** | Differential drive via `diff_drive_controller` (ros2_control) |
| **Robot Radius** | 0.25m |
| **ONNX Runtime** | v1.18.1 (CPU, single-threaded inference) |
| **BehaviorTree.CPP** | v3 (not v4) |
| **nav2_behavior_tree** | Used for `BehaviorTreeEngine` utility class |

**ONNX Runtime installation path:** `$HOME/libs/onnxruntime-linux-x64-1.18.1/` (fallback to system paths)

---

*Document generated from source-level analysis of the `smrr_navigation` package.*
