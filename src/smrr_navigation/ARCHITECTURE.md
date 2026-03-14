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
   - 4.3 [Door Detection — Four Methods](#43-door-detection--four-methods)
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
| Cross-floor elevator navigation | Custom BT sequence with 15 BT nodes |
| Elevator door detection (entry/exit) | Planner-based: repeated `ComputePathToPose` polling; door open when costmap clears |
| Elevator floor arrival detection | `CheckFloorArrival` action — YOLO + HSV camera analysis of button panel |
| AMCL freeze during in-elevator turn | `SetAMCLParams` pauses particle filter updates while robot spins inside elevator |
| Map switching at runtime | `nav2_msgs/srv/LoadMap` service, open/closed map variants per floor |
| Named location resolution | YAML-backed service (`/go_to_pose`) with floor-aware lookup |
| AMCL re-initialization (floor switch) | Three modes: AprilTag TF correction → relative parking-error → direct publish |
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
├── include/smrr_navigation/bt_nodes/ # C++ BT node headers (16 files)
├── launch/
│   └── smrr_world_navigation.launch.py
├── maps/                             # Occupancy grid maps (7 YAML+PGM pairs)
│   ├── floor0_open.yaml / first_floor_with_docking_station.yaml
│   ├── floor1_open.yaml / second_floor.yaml
│   ├── third_floor.yaml              # floor2 (open=closed)
│   └── fourth_floor.yaml             # floor3 (open=closed)
├── models/
│   ├── door_classifier.onnx          # Door classifier model (legacy, not used in active BT)
│   ├── door_classifier_3.onnx        # Door classifier model (TinyCNN ~15K params, not used in active BT)
│   └── yolo_button_detection.pt      # YOLO model for elevator button panel detection
├── smrr_navigation/                  # Python package
│   ├── __init__.py
│   ├── depth_preprocess_spec.py      # Canonical depth preprocessing specification
│   ├── door_classifier_node.py       # Standalone ONNX inference ROS node (legacy)
│   ├── floor_arrival_server.py       # Action server: YOLO+HSV floor arrival detection
│   ├── location_subscriber.py        # Topic→service bridge (/location → /go_to_pose)
│   ├── models/                       # Python package model assets (onnx, pt files)
│   ├── named_goal_client.py          # CLI client for /go_to_pose
│   ├── named_goal_server.py          # Service server: name→pose resolution + dispatch
│   ├── smrr_multifloor_bt_navigator.py  # Legacy Python action server
│   ├── startup_localizer.py          # AMCL convergence helper (drive+rotate)
│   └── test_floor_vision.py          # Standalone test for floor vision pipeline
├── src/
│   ├── bt_nodes/                     # C++ BT node implementations (17 files)
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
│   │   ├── wait_for_door_open_model_action.cpp  # Compiled but NOT used in active BT XML
│   │   ├── update_pose_timestamp_action.cpp
│   │   ├── stop_robot_action.cpp
│   │   ├── set_controller_params_action.cpp      # Runtime velocity limit adjustment
│   │   ├── set_amcl_params_action.cpp            # NEW: freeze/restore AMCL during in-elevator spin
│   │   ├── check_floor_arrival_action.cpp        # BtActionNode wrapping CheckFloorArrival
│   │   └── clear_costmaps_action.cpp             # Legacy helper (ClearEntireCostmap is Nav2 std)
│   └── smrr_bt_mission_executor.cpp  # Service server that loads & ticks the BT
├── CMakeLists.txt                    # Hybrid ament_cmake + ament_cmake_python
├── package.xml
└── setup.py                          # Python entry points (8 executables)
```

### 1.3 Build System

The package uses **hybrid `ament_cmake` + `ament_cmake_python`** to build both C++ and Python targets:

**C++ targets:**

| Target | Type | Description |
|---|---|---|
| `smrr_bt_nodes` | Shared library (`SHARED`) | 15 custom BT node implementations, registered as a BT plugin |
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
| `door_classifier_node.py` | Standalone ONNX door classifier ROS node (legacy) |
| `floor_arrival_server.py` | Action server: YOLO + HSV floor arrival detection |
| `test_floor_vision.py` | Standalone test for floor vision pipeline |

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
   ├── floor_arrival_server      ← /check_floor_arrival (CheckFloorArrival action)
   │     Subscribes /zed2_left_camera/image_raw, runs YOLO+HSV on button panel
   │     Succeeds when the target-floor button extinguishes (door opens)
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

#### `CheckFloorArrival.action`

```yaml
# Goal
string target_floor    # BT blackboard floor id, e.g. "floor1", "floor3"
---
# Result
bool   arrived         # true when the floor button extinguishes
string message
---
# Feedback
string status          # "ON", "OFF_UNSTABLE", "OFF_STABLE", "NO_DETECTION"
float32 ratio          # HSV illuminated-pixel ratio (0.0 – 1.0)
```

Handled by `floor_arrival_server`. Called by the `CheckFloorArrival` BT node inside the cross-floor exit sequence. The server maps BT floor IDs to YOLO button classes: `floor0→button-g`, `floor1→button-1`, `floor2→button-2`, `floor3→button-3`. Door arrival is confirmed when the illuminated button ratio drops below the OFF threshold for 100 ms.

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
        (Nav2 action)                    GetNamedPose(staging) → NavigateToPose
                                         CallElevator → SetControllerParams(0.55)
                                         ComputePathToPose polling (door open signal)
                                         NavigateToPose(inside) → SetControllerParams(0.35)
                                         SetAMCLParams(10,10) → Spin(-π) → StopRobot
                                         SetAMCLParams(0.25,0.1) → Wait(5s)
                                         SwitchMap → PublishInitialPose(AprilTag) → ClearCostmaps
                                         CallElevator(target) → CheckFloorArrival
                                         SetControllerParams(0.55) → ComputePathToPose poll
                                         NavigateToPose(exit) → SetControllerParams(0.35)
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
 3. GetNamedPose(elevator_inside) ─ Pre-cache inside pose on blackboard (used inside retry)
 4. RetryUntilSuccessful(2) ──── Retry wrapper for elevator entry:
    ├─ CallElevator(current_floor)   Send Gazebo CLI command
    ├─ SetControllerParams(0.55)     Boost max_vel_x before crossing the gap
    ├─ RetryUntilSuccessful(3000)    Planning-based door detection (up to 1500 s):
    │    └─ Delay(500ms) + ComputePathToPose(elevator_inside)
    │         When door opens, LiDAR clears costmap cells → planner succeeds → exit loop
    ├─ GetNamedPose(elevator_inside) → NavigateToPose
    │                                  Drive into elevator
    ├─ SetControllerParams(0.35)     Reset velocity to normal after crossing gap
    ├─ StopRobot(200ms repeat, 800ms duration)
    └─ Wait(5.0s)                    Wait for elevator door to close before issuing
                                     target-floor command (~5 s for Gazebo plugin)
 5. GetNamedMap(target_floor, "open")
 6. SwitchMap ────────────────── Load target floor's open map
 7. PublishInitialPose ────────── Seed AMCL with known elevator position
 8. ClearEntireCostmap (local)
 9. ClearEntireCostmap (global) ── Remove stale obstacle data
10. Fallback(ExitElevator) ──── Exit with retry:
    ├─ Attempt 1:
    │   ├─ GetNamedPose(elevator_exit)   Pre-cache exit pose
    │   ├─ CallElevator(target_floor)
    │   ├─ CheckFloorArrival(target_floor)  Wait for YOLO+HSV button OFF
    │   ├─ SetControllerParams(0.55)     Boost velocity before crossing exit gap
    │   ├─ RetryUntilSuccessful(3000): Delay(500ms) + ComputePathToPose(exit_pose)
    │   ├─ NavigateToPose(elevator_exit)
    │   └─ SetControllerParams(0.35)     Reset velocity
    └─ Attempt 2 (reposition):
        ├─ GetNamedPose(elevator_exit)   Pre-cache exit pose
        ├─ GetNamedPose(elevator_inside, target_floor) → NavigateToPose
        ├─ Spin(-3.5416 rad ≈ -203°)  Full reposition rotation
        ├─ StopRobot
        ├─ CallElevator(target_floor)
        ├─ CheckFloorArrival(target_floor)
        ├─ SetControllerParams(0.55)
        ├─ RetryUntilSuccessful(3000): Delay(500ms) + ComputePathToPose(exit_pose)
        ├─ NavigateToPose(elevator_exit)
        └─ SetControllerParams(0.35)
11. NavigateToPose(final_pose) ── Navigate to desired destination
```

**Key design decisions visible in the BT XML:**

| Decision | Value/Rationale |
|---|---|
| Door detection mechanism | Planning-based: `ComputePathToPose` polls every 500 ms (up to 3000 = 1500 s). When the physical door opens, LiDAR clears costmap obstacle cells and the planner finds a path |
| `SetControllerParams(0.55)` | Temporarily boosts `FollowPath.vx_max` to 0.55 m/s immediately before gap crossing — the robot is already at speed the instant the path is found |
| `SetControllerParams(0.35)` | Resets `FollowPath.vx_max` to normal 0.35 m/s after crossing |
| `SetAMCLParams(10.0, 10.0)` | Freezes AMCL particle-filter updates (`update_min_d`=10 m, `update_min_a`=10 rad) before the in-elevator 180° spin. Symmetric metal walls cause spurious AMCL updates if not suppressed |
| `Spin(-3.1416)` | 180° spin inside elevator to orient robot toward the exit door |
| `SetAMCLParams(0.25, 0.1)` | Restores AMCL to normal thresholds (`update_min_d`=0.25 m, `update_min_a`=0.1 rad) after spin completes |
| `CheckFloorArrival` at exit | YOLO + HSV analysis of elevator button panel; confirms elevator physically arrived at target floor before polling for the exit door |
| Post-spin `Wait(5.0s)` | Waits for the door to close before issuing the floor command, avoiding race conditions with the Gazebo plugin |
| `PublishInitialPose` with AprilTag | Primary mode: looks up live TF `base_link→tag36h11:0`, computes `T_map_to_actual = T_map_to_ideal * T_ideal_to_tag * T_actual_to_tag.inverse()`, flattens to 2D. Falls back to direct publish if TF misses |
| Exit retry spin angle | -3.5416 rad (>π) — ensures a full turnaround even if pose is slightly off |
| `GetNamedPose` before retry loops | Resolved before `RetryUntilSuccessful` so the pose is on the blackboard for all retry iterations |

### 2.3 AMCL Re-Initialization

When the robot changes floors, the map is replaced and AMCL must be re-seeded. The `PublishInitialPose` BT node publishes a `PoseWithCovarianceStamped` to `/initialpose` using one of three modes (evaluated in priority order):

#### Mode 1 — AprilTag TF correction (active in BT XML)

Uses the live TF of the AprilTag (`tag36h11:0`) relative to the robot's base frame to compute the actual map pose:

```
T_map_to_actual_base = T_map_to_ideal_base * T_ideal_base_to_tag * T_actual_base_to_tag.inverse()
```

The tag's *expected* transform is provided via BT ports (6-DOF: x, y, z, roll, pitch, yaw) to handle optical-frame rotations correctly. The result is flattened to strict 2D (`z=0`, `roll=0`, `pitch=0`). If the TF lookup fails, the node falls back to Mode 3 (direct publish) with a warning log.

```cpp
// AprilTag mode — core math in PublishInitialPoseAction::tick()
tf2::Transform map_to_ideal_base = poseToTf(target_pose_stamped.pose);

tf2::Quaternion ideal_tag_quat;
ideal_tag_quat.setRPY(exp_roll, exp_pitch, exp_yaw);  // 6-DOF for optical frame
tf2::Transform ideal_base_to_tag(ideal_tag_quat, tf2::Vector3(exp_x, exp_y, exp_z));

tf2::Transform actual_base_to_tag;  // from live lookupTransform(base_frame, tag_frame)
tf2::fromMsg(tag_tf_stamped.transform, actual_base_to_tag);

tf2::Transform map_to_actual_base =
  map_to_ideal_base * ideal_base_to_tag * actual_base_to_tag.inverse();

// Flatten to 2D
double corrected_yaw = tf2::getYaw(map_to_actual_base.getRotation());
tf2::Quaternion flat_quat;
flat_quat.setRPY(0.0, 0.0, corrected_yaw);
pose_msg.pose.pose.position.x = map_to_actual_base.getOrigin().x();
pose_msg.pose.pose.position.y = map_to_actual_base.getOrigin().y();
pose_msg.pose.pose.position.z = 0.0;
pose_msg.pose.pose.orientation = tf2::toMsg(flat_quat);
```

**BT XML usage (current configuration):**
```xml
<PublishInitialPose
  initial_pose="{amcl_open_target}"
  use_apriltag="true"
  tag_frame="tag36h11:0"
  expected_tag_x="1.313"  expected_tag_y="0.071"  expected_tag_z="1.412"
  expected_tag_roll="1.581" expected_tag_pitch="0.000" expected_tag_yaw="-1.633"/>
```

#### Mode 2 — Relative parking-error correction (optional)

When `current_expected_pose` is provided on the blackboard, the node looks up the actual robot TF (`map→base_link`), computes the parking error relative to the departure floor's expected inside pose, and applies that same error to the target floor's nominal pose:

```cpp
error_tf   = expected_tf.inverse() * actual_tf;
adjusted   = target_tf * error_tf;
```

#### Mode 3 — Direct publish (fallback)

Publishes `initial_pose` unchanged with a standard 2D covariance:

```cpp
pose_msg.pose.covariance[0]  = 0.25;    // σ²_x  = 0.25 m² (σ = 0.5m)
pose_msg.pose.covariance[7]  = 0.25;    // σ²_y  = 0.25 m²
pose_msg.pose.covariance[35] = 0.0685;  // σ²_yaw = 0.0685 rad² (σ ≈ 15°)
```

#### AMCL freeze during in-elevator spin

Before the 180° spin inside the elevator, `SetAMCLParams` is called to effectively freeze the particle filter:

```xml
<SetAMCLParams update_min_d="10.0" update_min_a="10.0"/>  <!-- freeze: 10m/10rad threshold -->
<Spin spin_dist="-3.1416" time_allowance="10.0" is_recovery="true"/>
<StopRobot topic="/cmd_vel" repeat_ms="200" duration_ms="800"/>
<SetAMCLParams update_min_d="0.25" update_min_a="0.1"/>   <!-- restore: normal thresholds -->
```

Setting thresholds to 10.0 m / 10.0 rad means the particle filter will not update regardless of how much the robot moves, preventing the symmetric elevator walls from corrupting the pose estimate during the turn.

After the map switch and `PublishInitialPose`, two `ClearEntireCostmap` calls (local + global) ensure stale obstacle data from the previous floor does not corrupt planning.

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
  update_min_d: 0.25           # Normal: update every 25 cm (overridden during spin)
  update_min_a: 0.1            # Normal: update every ~6° (overridden during spin)
```

### 2.4 Legacy Python Action Server vs. C++ BT Executor

Two complete implementations of cross-floor navigation exist:

| Aspect | Python Action Server | C++ BT Executor |
|---|---|---|
| File | `smrr_multifloor_bt_navigator.py` | `smrr_bt_mission_executor.cpp` (224 lines) + BT XML + 15 BT nodes |
| Interface | `NavigateToNamedLocation` action | `StartMission` service |
| Dispatch mode | `use_bt_mission_executor=False` | `use_bt_mission_executor=True` (default) |
| Door detection | LiDAR scan fraction only | Planning-based (`ComputePathToPose` polling) |
| Floor arrival | Not implemented | `CheckFloorArrival` action (YOLO + HSV) |
| AMCL freeze during spin | Not implemented | `SetAMCLParams` freezes/restores `update_min_d/a` |
| AMCL re-init method | Fixed pose seed | AprilTag TF correction → relative parking-error → direct publish |
| Velocity control | Fixed | `SetControllerParams` dynamically adjusts `vx_max` |
| Retry logic | Hardcoded in Python sequences | Declarative in BT XML (`RetryUntilSuccessful`, `Fallback`) |
| Elevator control | Subprocess call to `gz-11.14.0` | Same (`CallElevator` BT node wraps subprocess) |

The BT-based executor is the **active default**. The Python action server is retained as a legacy fallback.

---

## 3. Behavior Tree Architecture

### 3.1 BT XML Structure (`smrr_multifloor.xml`)

Full BT XML (144 lines — actual file content):

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

      <!-- Cross-floor navigation -->
      <Sequence name="CrossFloor_ElevatorEntry">
        <IsDifferentFloor current_floor="{current_floor_id}" target_floor="{target_floor_id}"/>

        <!-- Navigate to staging pose on current floor -->
        <GetNamedPose floor_id="{current_floor_id}" location_key="elevator_staging" pose="{staging_pose}"/>
        <NavigateToPose server_name="/navigate_to_pose" goal="{staging_pose}"/>

        <!-- Resolve inside pose before the retry loop so it is available on the blackboard -->
        <GetNamedPose floor_id="{current_floor_id}" location_key="elevator_inside" pose="{inside_pose}"/>

        <!-- Elevator entry: call elevator, poll planner until door is open, then enter -->
        <RetryUntilSuccessful num_attempts="2" name="CallElevatorAndEnter_Twice">
          <Sequence name="CallWaitAndEnter">
            <CallElevator floor_id="{current_floor_id}"/>

            <!-- Boost velocity before waiting: robot is already at gap-crossing speed
                 the instant the door opens (zero-delay boost). -->
            <SetControllerParams max_vel_x="0.55"/>

            <!-- Planning-based door detection: poll global planner every 500 ms.
                 When the physical door opens the LiDAR clears the costmap obstacle
                 cells and ComputePathToPose succeeds, breaking out of the loop. -->
            <RetryUntilSuccessful num_attempts="3000" name="WaitForPath_Entry">
              <Delay delay_msec="500">
                <ComputePathToPose goal="{inside_pose}" path="{dummy_path}" planner_id="GridBased"/>
              </Delay>
            </RetryUntilSuccessful>

            <!-- Navigate inside elevator and rotate 180 degrees -->
            <GetNamedPose floor_id="{current_floor_id}" location_key="elevator_inside" pose="{inside_pose}"/>
            <NavigateToPose server_name="/navigate_to_pose" goal="{inside_pose}"/>
            <!-- Reset velocity to normal after crossing the entry gap -->
            <SetControllerParams max_vel_x="0.35"/>

            <SetAMCLParams update_min_d="10.0" update_min_a="10.0"/>

            <Spin spin_dist="-3.1416" time_allowance="10.0" is_recovery="true"/>

            <StopRobot topic="/cmd_vel" repeat_ms="200" duration_ms="800"/>

            <SetAMCLParams update_min_d="0.25" update_min_a="0.1"/>
            <!-- Wait for the elevator door to fully close before issuing the
                 target-floor command (~5 s is sufficient for the Gazebo plugin). -->
            <Wait wait_duration="5.0"/>
          </Sequence>
        </RetryUntilSuccessful>

        <!-- Switch to target floor's open map while inside elevator -->
        <GetNamedMap floor_id="{target_floor_id}" map_key="open" map_yaml="{map_open_target}"/>

        <!-- Fetch both poses: target floor nominal pose AND departure floor expected pose -->
        <GetNamedPose floor_id="{target_floor_id}" location_key="amcl_initial_pose_open" pose="{amcl_open_target}"/>
        <GetNamedPose floor_id="{current_floor_id}" location_key="amcl_initial_pose_open" pose="{amcl_open_current}"/>

        <SwitchMap map_yaml="{map_open_target}"/>

        <!-- Publish initial pose with AprilTag correction:
             Looks up live TF base_link→tag36h11:0, applies
             T_map_to_actual = T_map_to_ideal * T_ideal_base_to_tag * T_actual_base_to_tag.inverse()
             Falls back to direct publish if TF lookup fails. -->
        <PublishInitialPose initial_pose="{amcl_open_target}"
                            use_apriltag="true"
                            tag_frame="tag36h11:0"
                            expected_tag_x="1.313"
                            expected_tag_y="0.071"
                            expected_tag_z="1.412"
                            expected_tag_roll="1.581"
                            expected_tag_pitch="0.000"
                            expected_tag_yaw="-1.633"/>
        <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>

        <!-- Exit elevator with retry mechanism -->
        <Fallback name="ExitElevator_WithRepositionRetry">

          <!-- Attempt 1: call elevator, poll planner until door open, exit -->
          <Sequence name="ExitAttempt_1">
            <!-- Resolve exit pose before the wait loop begins -->
            <GetNamedPose floor_id="{target_floor_id}" location_key="elevator_exit" pose="{exit_pose}"/>
            <CallElevator floor_id="{target_floor_id}"/>

            <CheckFloorArrival target_floor="{target_floor_id}"/>

            <!-- Boost velocity before waiting for exit door to open -->
            <SetControllerParams max_vel_x="0.55"/>

            <RetryUntilSuccessful num_attempts="3000" name="WaitForPath_Exit1">
              <Delay delay_msec="500">
                <ComputePathToPose goal="{exit_pose}" path="{dummy_path}" planner_id="GridBased"/>
              </Delay>
            </RetryUntilSuccessful>

            <NavigateToPose server_name="/navigate_to_pose" goal="{exit_pose}"/>
            <!-- Reset velocity to normal after crossing the exit gap -->
            <SetControllerParams max_vel_x="0.35"/>
          </Sequence>

          <!-- Attempt 2: reposition inside elevator, then retry -->
          <Sequence name="RepositionThenExitAttempt_2">
            <!-- Resolve exit pose before the wait loop begins -->
            <GetNamedPose floor_id="{target_floor_id}" location_key="elevator_exit" pose="{exit_pose}"/>
            <!-- Navigate to elevator inside pose of target floor to reposition -->
            <GetNamedPose floor_id="{target_floor_id}" location_key="elevator_inside" pose="{inside_pose}"/>
            <NavigateToPose server_name="/navigate_to_pose" goal="{inside_pose}"/>
            <Spin spin_dist="-3.5416" time_allowance="10.0" is_recovery="true"/>
            <StopRobot topic="/cmd_vel" repeat_ms="200" duration_ms="800"/>

            <CallElevator floor_id="{target_floor_id}"/>

            <CheckFloorArrival target_floor="{target_floor_id}"/>

            <!-- Boost velocity before waiting for exit door (retry attempt 2) -->
            <SetControllerParams max_vel_x="0.55"/>

            <RetryUntilSuccessful num_attempts="3000" name="WaitForPath_Exit2">
              <Delay delay_msec="500">
                <ComputePathToPose goal="{exit_pose}" path="{dummy_path}" planner_id="GridBased"/>
              </Delay>
            </RetryUntilSuccessful>

            <NavigateToPose server_name="/navigate_to_pose" goal="{exit_pose}"/>
            <!-- Reset velocity to normal after crossing the exit gap (retry) -->
            <SetControllerParams max_vel_x="0.35"/>
          </Sequence>

        </Fallback>

        <!-- Navigate to final destination -->
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
 │    ├── IsSameFloor [Condition]             → SUCCESS if floors match
 │    └── NavigateToPose                      → Nav2 action
 └── CrossFloor [Sequence]
      ├── IsDifferentFloor [Condition]        → SUCCESS if floors differ
      ├── Navigate to staging
      ├── GetNamedPose(elevator_inside)       → pre-cache inside pose
      ├── RetryUntilSuccessful(2)             → elevator entry:
      │    CallElevator → SetControllerParams(0.55)
      │    RetryUntilSuccessful(3000) [ComputePathToPose polls]
      │    NavigateToPose(inside) → SetControllerParams(0.35)
      │    SetAMCLParams(10.0,10.0) → Spin(-π) → StopRobot
      │    SetAMCLParams(0.25,0.1) → Wait(5s)
      ├── GetNamedPose(amcl_open_target) + GetNamedPose(amcl_open_current)
      ├── SwitchMap + PublishInitialPose(AprilTag) + ClearCostmaps
      ├── Fallback [exit retry]
      │    ├── ExitAttempt_1:
      │    │    GetNamedPose(exit) → CallElevator
      │    │    CheckFloorArrival → SetControllerParams(0.55)
      │    │    RetryUntilSuccessful(3000) [ComputePathToPose polls]
      │    │    NavigateToPose(exit) → SetControllerParams(0.35)
      │    └── ExitAttempt_2 (reposition):
      │         GetNamedPose(inside, target) → NavigateToPose
      │         Spin(-3.5416) → StopRobot
      │         CallElevator → CheckFloorArrival
      │         SetControllerParams(0.55) → ComputePathToPose polls
      │         NavigateToPose(exit) → SetControllerParams(0.35)
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
    // Required by Nav2 BT action nodes
    blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(2000));
    blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
    blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", std::chrono::milliseconds(1000));
    blackboard->set("current_floor_id", request->current_floor_id);
    blackboard->set("target_floor_id",  request->target_floor_id);
    blackboard->set("final_pose",       final_pose);

    // 4. Tick tree at 20 Hz with 300s timeout
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      // Check timeout
      if (elapsed > bt_timeout_sec_) { /* report timeout FAILURE */ return; }
      status = tree.tickRoot();
      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1000.0 / bt_tick_rate_hz_)));  // 50ms @ 20Hz
    }
  }
};

// Uses MultiThreadedExecutor + ReentrantCallbackGroup
// to allow BT nodes to make service calls during tree execution
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

**Plugin loading** (from launch file, 26 Nav2 standard plugins + custom library):

```python
'plugin_lib_names': [
    # Nav2 standard BT plugins
    'nav2_compute_path_to_pose_action_bt_node',   # used for door detection polling
    'nav2_follow_path_action_bt_node',
    'nav2_back_up_action_bt_node',
    'nav2_spin_action_bt_node',
    'nav2_wait_action_bt_node',
    'nav2_clear_costmap_service_bt_node',         # provides ClearEntireCostmap
    'nav2_is_stuck_condition_bt_node',
    'nav2_goal_reached_condition_bt_node',
    'nav2_initial_pose_received_condition_bt_node',
    'nav2_goal_updated_condition_bt_node',
    'nav2_reinitialize_global_localization_service_bt_node',
    'nav2_rate_controller_bt_node',
    'nav2_distance_controller_bt_node',
    'nav2_speed_controller_bt_node',
    'nav2_truncate_path_action_bt_node',
    'nav2_goal_updater_node_bt_node',
    'nav2_recovery_node_bt_node',
    'nav2_pipeline_sequence_bt_node',
    'nav2_round_robin_node_bt_node',
    'nav2_transform_available_condition_bt_node',
    'nav2_time_expired_condition_bt_node',
    'nav2_distance_traveled_condition_bt_node',
    'nav2_single_trigger_bt_node',
    'nav2_is_battery_low_condition_bt_node',
    'nav2_navigate_to_pose_action_bt_node',
    # Custom BT plugin library (14 nodes)
    'smrr_bt_nodes'
]
```

### 3.3 Custom BT Node Reference

All 15 registered BT nodes, with their types, ports, and algorithmic behavior:

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

**Purpose:** Publish `PoseWithCovarianceStamped` to `/initialpose` for AMCL re-initialization. Supports three modes evaluated in priority order: (1) AprilTag TF correction, (2) relative parking-error correction, (3) direct publish. See §2.3 for full algorithm details.

| Port | Direction | Type | Default |
|---|---|---|---|
| `initial_pose` | Input | `PoseStamped` | — |
| `current_expected_pose` | Input | `PoseStamped` | `""` (if set, enables Mode 2) |
| `topic_name` | Input | `string` | `"/initialpose"` |
| `frame_id` | Input | `string` | `""` (uses pose's frame) |
| `use_apriltag` | Input | `bool` | `false` |
| `tag_frame` | Input | `string` | `"tag36h11:0"` |
| `base_frame` | Input | `string` | `"base_link"` |
| `expected_tag_x` | Input | `double` | `0.0` |
| `expected_tag_y` | Input | `double` | `0.0` |
| `expected_tag_z` | Input | `double` | `0.0` |
| `expected_tag_roll` | Input | `double` | `0.0` |
| `expected_tag_pitch` | Input | `double` | `0.0` |
| `expected_tag_yaw` | Input | `double` | `0.0` |

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

#### `WaitForDoorOpenModel` — StatefulActionNode (ONNX-based) **[Registered but NOT active in BT XML]**

**Purpose:** Detect door opening using a trained TinyCNN depth classifier via ONNX Runtime. Compiled and registered in the plugin library but **replaced in the active BT XML** by the planner-based `ComputePathToPose` polling approach.

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

#### `ClearEntireCostmap` — Standard Nav2 BT Node

**Purpose:** Call a Nav2 costmap clear service. `ClearEntireCostmap` is a **standard Nav2 BT node** loaded from the `nav2_clear_costmap_service_bt_node` plugin — it is NOT a custom node. Used to clear both local and global costmaps after a map switch.

Typical usage in BT XML:
```xml
<ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
<ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
```

---

#### `SetControllerParams` — SyncActionNode

**Purpose:** Dynamically update the DWB/MPPI local planner velocity limit at runtime by calling `/controller_server/set_parameters`. Used to temporarily boost `max_vel_x` to 0.55 m/s before the robot crosses the physical elevator floor-gap, then reset to 0.35 m/s after.

**Implementation:**

```cpp
// Uses a dedicated helper_node_ (not owned by any executor) to avoid
// "already added to executor" error when calling SyncParametersClient.
params_client_ = std::make_shared<rclcpp::SyncParametersClient>(helper_node_, "/controller_server");

const std::vector<rclcpp::Parameter> params = {
  rclcpp::Parameter("FollowPath.vx_max", max_vel_x)
};
params_client_->set_parameters(params);  // blocks until server responds
```

> **Note:** Only `vx_max` (and similar velocity limits) are exposed as ROS 2 parameters in Nav2 Humble's MPPI/DWB. Acceleration fields (`ax_max`, etc.) are internal and cannot be set via `set_parameters`.

| Port | Direction | Type | Default |
|---|---|---|---|
| `max_vel_x` | Input | `double` | — |

---

#### `SetAMCLParams` — SyncActionNode

**Purpose:** Dynamically update AMCL's motion-model update thresholds at runtime by calling `/amcl/set_parameters`. Used to freeze the particle filter before the in-elevator 180° spin (preventing symmetric metal walls from corrupting the pose estimate), then restore normal thresholds afterward.

**Implementation:**

```cpp
// Uses same helper_node_ pattern as SetControllerParamsAction.
helper_node_ = rclcpp::Node::make_shared("set_amcl_params_helper");
params_client_ = std::make_shared<rclcpp::SyncParametersClient>(helper_node_, "/amcl");

const std::vector<rclcpp::Parameter> params = {
  rclcpp::Parameter("update_min_d", update_min_d),  // e.g. 10.0 to freeze, 0.25 to restore
  rclcpp::Parameter("update_min_a", update_min_a)   // e.g. 10.0 to freeze, 0.1 to restore
};
params_client_->set_parameters(params);  // blocks until server responds
```

| Port | Direction | Type | Default |
|---|---|---|---|
| `update_min_d` | Input | `double` | — |
| `update_min_a` | Input | `double` | — |

---

#### `CheckFloorArrival` — BtActionNode (wraps `check_floor_arrival` action)

**Purpose:** Wait for the elevator to physically arrive at the target floor before polling for the exit door. Sends a goal to the `floor_arrival_server` (`/check_floor_arrival` action), which monitors the elevator button panel via YOLO + OpenCV HSV analysis.

**Action server (`floor_arrival_server.py`) algorithm:**
1. Subscribe to `/zed2_left_camera/image_raw`
2. Run YOLO inference (model: `yolo_button_detection.pt`, conf=0.1) to detect the target floor button bounding box
3. Map BT floor IDs to YOLO class labels: `floor0→button-g`, `floor1→button-1`, `floor2→button-2`, `floor3→button-3`
4. Cache the last known bounding box for up to 100 missed frames (YOLO blind tolerance)
5. Compute HSV illumination ratio in the button crop: `count(pixels in [H:5–35, S:50–255, V:150–255]) / total_pixels`
6. Button is **ON** (elevator moving) when `ratio > 0.04`; **OFF** (arrived) when `ratio ≤ 0.04`
7. Require 100 ms stable OFF before returning `arrived=true`

| Port | Direction | Type | Default |
|---|---|---|---|
| `target_floor` | Input | `string` | — |

### 3.4 Plugin Registration

All 15 custom BT nodes are compiled into a single shared library (`libsmrr_bt_nodes.so`) and registered via the BT.CPP plugin mechanism:

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
  factory.registerNodeType<WaitForDoorOpenAction>("WaitForDoorOpen");           // available, not in active XML
  factory.registerNodeType<WaitForDoorOpenDepthAction>("WaitForDoorOpenDepth"); // available, not in active XML
  factory.registerNodeType<WaitForDoorOpenModelAction>("WaitForDoorOpenModel"); // available, not in active XML
  factory.registerNodeType<UpdatePoseTimestampAction>("UpdatePoseTimestamp");
  factory.registerNodeType<StopRobotAction>("StopRobot");
  factory.registerNodeType<SetControllerParamsAction>("SetControllerParams");
  factory.registerNodeType<SetAMCLParamsAction>("SetAMCLParams");

  // CheckFloorArrival uses a 3-arg BtActionNode constructor
  BT::NodeBuilder check_floor_builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<CheckFloorArrivalAction>(
        name, "check_floor_arrival", config);
    };
  factory.registerBuilder<CheckFloorArrivalAction>("CheckFloorArrival", check_floor_builder);
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

### 4.3 Door Detection — Four Methods

The package implements or has implemented four door detection approaches. The **active method in the BT XML** is the planning-based approach (Method 4). Methods 1–3 are compiled and registered but are not invoked by the current BT XML.

#### Method 1: LiDAR Scan Fraction (`WaitForDoorOpen`) — available, not active

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

#### Method 2: Depth Baseline + Free Space (`WaitForDoorOpenDepth`) — available, not active

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

#### Method 3: ONNX TinyCNN Classifier (`WaitForDoorOpenModel`) — available, not active

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
  p_open = probs[open_index]         # open_index=1 in BT XML when active

Decision:
  p_open ≥ threshold → OPEN → start/continue stability timer
  p_open < threshold → CLOSED → reset stability timer
  Stable for stable_time_sec → SUCCESS
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

#### Method 4: Planner-Based (`ComputePathToPose` polling) **[ACTIVE in BT XML]**

```
Algorithm: Costmap-coupled path planning
─────────────────────────────────────────
Observation: When the elevator door is closed, the obstacle cells in the global
  costmap block all paths from the robot's position to the target pose.
  When the door opens, LiDAR rays clear those cells and the NavfnPlanner
  (Dijkstra) can compute a valid path.

Implementation (BT XML):
  RetryUntilSuccessful(num_attempts=3000):
    Delay(delay_msec=500)
    ComputePathToPose(goal=inside_pose, planner_id="GridBased")
       → FAILURE while door closed (no path through costmap obstacle)
       → SUCCESS when door opens  (path found through cleared cells)

Timing:
  Max wait = 3000 × 500 ms = 1500 s (same budget as previous ONNX approach)
  Typical wait = 1–5 s (Gazebo elevator response)
```

**Strengths:** Zero additional sensors or models required. Leverages the existing costmap/LiDAR pipeline directly. The robot's velocity is already boosted (`SetControllerParams(0.55)`) before the loop, so it starts moving the instant a path is found with no latency.
**Weaknesses:** Depends on LiDAR being able to see through the open door gap and update the costmap promptly. May false-positive if an unrelated costmap gap appears in the door direction.

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
      amcl_initial_pose_closed: {x: -1.424, y: 3.132,   yaw: -1.57}
      amcl_initial_pose_open:   {x: -1.424, y: 3.132,   yaw: -1.57}
      dock:              {x: 2.23,  y: -1.0,   yaw: 0.0}
      elevator_staging:  {x: -2.0,  y: 1.1,    yaw: 1.57}
      elevator_inside:   {x: -1.424,y: 3.132,  yaw: -1.57}
      elevator_exit:     {x: -1.5,  y: 0.8,    yaw: -1.57}
      left_back:         {x: -9.9,  y: 1.33,   yaw: 0.0}
      right_back:        {x: -9.9,  y: -3.0,   yaw: 3.14}
      left_front:        {x: 6.15,  y: 1.18,   yaw: 3.14}
      right_front:       {x: 6.19,  y: -2.45,  yaw: 3.14}
  floor1:
    maps:
      closed: second_floor.yaml
      open: floor1_open.yaml
    locations:
      amcl_initial_pose_closed: {x: -1.425, y: 2.9743, yaw: -1.57}
      amcl_initial_pose_open:   {x: -1.425, y: 2.9743, yaw: -1.57}
      elevator_staging:  {x: -2.022, y: 1.0195, yaw: 1.57}
      elevator_inside:   {x: -1.425, y: 2.9743, yaw: -1.57}
      elevator_exit:     {x: -1.7197,y: 0.38080,yaw: -1.57}
      office_101:        {x: 5.896,  y: -2.293, yaw: 0.0}
      office_102:        {x: 5.0,    y: 2.0,    yaw: 0.0}
      conference_room:   {x: -5.0,   y: 3.5,    yaw: 1.57}
      reception:         {x: 0.0,    y: 0.0,    yaw: 3.14}
  floor2:
    maps: {closed: third_floor.yaml, open: third_floor.yaml}  # Same map
    locations:
      amcl_initial_pose_closed: {x: -1.5,  y: 0.8,  yaw: 1.57}
      amcl_initial_pose_open:   {x: -1.5,  y: 0.8,  yaw: 1.57}
      elevator_staging:  {x: -2.5,  y: 0.8,  yaw: 1.57}
      elevator_inside:   {x: -1.96, y: 3.00, yaw: -1.57}
      elevator_exit:     {x: -1.5,  y: 0.8,  yaw: 1.57}
      lab_201:           {x: 4.5,   y: -2.0, yaw: 0.0}
      lab_202:           {x: 6.5,   y: -2.0, yaw: 0.0}
      server_room:       {x: -6.0,  y: -4.0, yaw: 3.14}
      break_room:        {x: 2.0,   y: 4.0,  yaw: 1.57}
  floor3:
    maps: {closed: fourth_floor.yaml, open: fourth_floor.yaml}  # Same map
    locations:
      amcl_initial_pose_closed: {x: -1.5,  y: 0.8,  yaw: 1.57}
      amcl_initial_pose_open:   {x: -1.5,  y: 0.8,  yaw: 1.57}
      elevator_staging:  {x: -2.5,  y: 0.8,  yaw: 1.57}
      elevator_inside:   {x: -1.96, y: 3.00, yaw: -1.57}
      elevator_exit:     {x: -1.5,  y: 0.8,  yaw: 1.57}
      exec_office:       {x: 8.0,   y: 3.0,  yaw: 0.0}
      board_room:        {x: -7.0,  y: 2.5,  yaw: 1.57}
      rooftop_access:    {x: 0.0,   y: 8.0,  yaw: 0.0}
      storage:           {x: -3.0,  y: -5.0, yaw: 3.14}
```

**Key observations:**
- `elevator_inside` and `amcl_initial_pose_open/closed` for `floor0` now share the **same pose** (`x: -1.424, y: 3.132, yaw: -1.57`), placing the AMCL seed directly at the inside-elevator position
- All floors use `yaw: -1.57` for `elevator_inside`, meaning the robot faces the door in the same direction on all floors (previously floor0 was `yaw: 1.57`)
- Floors 2 and 3 use the **same YAML for open and closed** maps (no elevator door modeled in the map)
- All 4 floors now have complete mandatory pose entries: `amcl_initial_pose_open`, `amcl_initial_pose_closed`, `elevator_staging`, `elevator_inside`, `elevator_exit`
- `floor1` `elevator_staging` moved significantly: was `{x: -0.30, y: -0.83}`, now `{x: -2.022, y: 1.0195}`

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
│             locations_file=locations.yaml, start_mission_service_name=/start_mission,
│             start_mission_timeout=5.0
│     respawn=True, respawn_delay=2.0
│
├── location_subscriber.py
│     Bridge: /location (String) → /go_to_pose service
│
├── floor_arrival_server.py
│     Action server: /check_floor_arrival (CheckFloorArrival)
│     Params: yolo_model_path=<pkg>/models/yolo_button_detection.pt
│     Subscribes: /zed2_left_camera/image_raw
│     Publishes:  /floor_vision/debug_image
│
├── smrr_bt_mission_executor
│     Params: bt_xml_path=<pkg>/behavior_trees/smrr_multifloor.xml,
│             plugin_lib_names=[26 Nav2 plugins + "smrr_bt_nodes"],
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

ros2 topic pub --once /location std_msgs/msg/String '{data: "reception"}'
