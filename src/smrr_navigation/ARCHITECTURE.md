# `smrr_navigation` — Architecture & Implementation Reference

> **Purpose:** This document is the authoritative technical reference for the `smrr_navigation` ROS 2 package.
> It reflects the **currently implemented** system — all algorithms, node graphs, BT structure, interfaces, and parameter values are drawn directly from source code.

---

## Table of Contents

1. [System Overview](#1-system-overview)
   - 1.1 [Package Overview](#11-package-overview)
   - 1.2 [Directory Layout](#12-directory-layout)
   - 1.3 [Build System](#13-build-system)
2. [ROS Node Graph](#2-ros-node-graph)
   - 2.1 [Node Descriptions](#21-node-descriptions)
   - 2.2 [Topic & Service Summary](#22-topic--service-summary)
   - 2.3 [Data Flow Diagram](#23-data-flow-diagram)
3. [Custom Interfaces (`smrr_interfaces`)](#3-custom-interfaces-smrr_interfaces)
4. [Python Nodes](#4-python-nodes)
   - 4.1 [named_goal_server.py](#41-named_goal_serverpy)
   - 4.2 [floor_arrival_server.py](#42-floor_arrival_serverpy)
   - 4.3 [elevator_direction_server.py](#43-elevator_direction_serverpy)
   - 4.4 [apriltag_manager_server.py](#44-apriltag_manager_serverpy)
   - 4.5 [startup_localizer.py](#45-startup_localizerpy)
   - 4.6 [location_subscriber.py](#46-location_subscriberpy)
   - 4.7 [test_floor_vision.py](#47-test_floor_visionpy)
   - 4.8 [elevator_call_button_server.py](#48-elevator_call_button_serverpy)
   - 4.9 [door_classifier_node.py (Legacy)](#49-door_classifier_nodepy-legacy)
5. [Behavior Tree Architecture](#5-behavior-tree-architecture)
   - 5.1 [BT Mission Executor (C++)](#51-bt-mission-executor-c)
   - 5.2 [smrr_multifloor.xml — Full Structure](#52-smrr_multifloortxml--full-structure)
   - 5.3 [Same-Floor Branch](#53-same-floor-branch)
   - 5.4 [Cross-Floor Branch — Elevator Entry](#54-cross-floor-branch--elevator-entry)
   - 5.5 [Cross-Floor Branch — In-Elevator Repositioning](#55-cross-floor-branch--in-elevator-repositioning)
   - 5.6 [Cross-Floor Branch — Door Close & Map Switch](#56-cross-floor-branch--door-close--map-switch)
   - 5.7 [Cross-Floor Branch — Elevator Exit](#57-cross-floor-branch--elevator-exit)
   - 5.8 [Cross-Floor Branch — Final Destination](#58-cross-floor-branch--final-destination)
   - 5.9 [Custom BT Node Reference](#59-custom-bt-node-reference)
6. [Key Algorithms](#6-key-algorithms)
   - 6.1 [Planning-Based Door Detection](#61-planning-based-door-detection)
   - 6.2 [Floor Arrival Detection (YOLO)](#62-floor-arrival-detection-yolo)
   - 6.3 [Elevator Direction Detection (YOLO + HSV)](#63-elevator-direction-detection-yolo--hsv)
   - 6.4 [AMCL Freeze During In-Elevator Spin](#64-amcl-freeze-during-in-elevator-spin)
   - 6.5 [AMCL Re-localization with AprilTag Correction](#65-amcl-re-localization-with-apriltag-correction)
   - 6.6 [Startup Localization Sequence](#66-startup-localization-sequence)
7. [Configuration Files](#7-configuration-files)
   - 7.1 [locations.yaml](#71-locationsyaml)
   - 7.2 [smrr_nav_params.yaml — Key Parameters](#72-smrr_nav_paramsyaml--key-parameters)
8. [Maps](#8-maps)
9. [Launch Files](#9-launch-files)
10. [End-to-End Execution Walkthroughs](#10-end-to-end-execution-walkthroughs)
    - 10.1 [Same-Floor Navigation](#101-same-floor-navigation)
    - 10.2 [Cross-Floor Navigation (floor0 → floor1)](#102-cross-floor-navigation-floor0--floor1)

---

## 1. System Overview

### 1.1 Package Overview

`smrr_navigation` is a hybrid C++/Python ROS 2 package built on **Nav2** that enables a differential-drive mobile robot to navigate across multiple floors of a building using an elevator.

| Capability | Implementation |
|---|---|
| Same-floor point-to-point navigation | Nav2 `NavigateToPose` (AMCL + NavfnPlanner + MPPI) |
| Cross-floor elevator navigation | Custom Behavior Tree (`smrr_multifloor.xml`) |
| Elevator door detection (entry & exit) | Planning-based: repeated `ComputePathToPose` polling |
| Elevator floor arrival detection | `CheckFloorArrival` action — YOLO button class recognition |
| Elevator direction confirmation | `CheckElevatorDirection` action — YOLO + HSV arrow detection |
| AMCL freeze during in-elevator spin | `SetAMCLParams` BT node raises thresholds to ~∞ |
| Map switching at runtime | `nav2_msgs/srv/LoadMap` via `SwitchMap` BT node |
| AMCL re-initialization after floor switch | `PublishInitialPose` BT node with optional AprilTag TF correction |
| Elevator call button detection & 3D localisation | `DetectCallButton` action — YOLO class detection + ZED2 organised point-cloud RANSAC plane fit → approach/press waypoints in `link_0` frame |
| Named location resolution | YAML-backed `/go_to_pose` service, floor-aware lookup |
| Startup localization | Odometry-driven forward + rotate sequence |
| Dynamic AprilTag node management | `apriltag_manager_server` starts/stops `apriltag_ros` on demand |

**Framework stack:**

```
ROS 2 Humble
├── Nav2         — AMCL, NavfnPlanner (Dijkstra), MPPI controller, Costmap2D
├── BehaviorTree.CPP v3  — mission orchestration
├── Ultralytics YOLO v8  — floor button & elevator arrow detection
├── ONNX Runtime 1.18    — legacy door classifier (compiled but inactive in BT)
└── Gazebo Classic       — elevator simulation
```

---

### 1.2 Directory Layout

```
smrr_navigation/
├── behavior_trees/
│   └── smrr_multifloor.xml               # Active BT for all missions
├── config/
│   ├── locations.yaml                     # Floor-aware named location database (simulation)
│   ├── physical_locations.yaml            # Hardware robot location database
│   ├── smrr_nav_params.yaml               # Nav2 params: AMCL, MPPI, costmaps, planner
│   └── smrr_nav.rviz                      # RViz configuration
├── include/smrr_navigation/bt_nodes/      # C++ BT node headers (~20 files)
├── launch/
│   ├── smrr_world_navigation.launch.py    # Simulation launch (Gazebo + Nav2 + all nodes)
│   └── smrr_hardware_navigation.launch.py # Hardware launch (no Gazebo)
├── maps/                                  # Occupancy grid maps (YAML + PGM pairs)
│   ├── first_floor_with_docking_station.yaml  # floor0 closed-door map
│   ├── floor0_open.yaml                       # floor0 open-door map
│   ├── second_floor.yaml                      # floor1 closed-door map
│   ├── floor1_open.yaml                       # floor1 open-door map
│   ├── third_floor.yaml                       # floor2 (single variant)
│   └── fourth_floor.yaml                      # floor3 (single variant)
├── models/
│   ├── button_detection.pt                # YOLO v8 model: floor buttons + direction arrows
│   ├── door_classifier.onnx               # Legacy door classifier (unused in active BT)
│   └── door_classifier_3.onnx             # Legacy TinyCNN door classifier (unused in active BT)
├── smrr_navigation/                       # Python package
│   ├── __init__.py
│   ├── apriltag_manager_server.py         # Start/stop apriltag_ros process on demand
│   ├── depth_preprocess_spec.py           # Canonical depth preprocessing (training spec)
│   ├── door_classifier_node.py            # ONNX inference node (legacy, not launched)
│   ├── elevator_direction_server.py       # Action server: YOLO+HSV arrow direction detection
│   ├── floor_arrival_server.py            # Action server: YOLO button arrival detection
│   ├── location_subscriber.py             # /location topic → /go_to_pose service bridge
│   ├── named_goal_client.py               # CLI test client
│   ├── named_goal_server.py               # Named location resolver + mission dispatcher
│   ├── smrr_multifloor_bt_navigator.py    # Legacy Python action server (not used)
│   ├── elevator_call_button_server.py     # Action server: YOLO + ZED2 point-cloud RANSAC call button localisation
│   ├── startup_localizer.py               # AMCL convergence motion sequence
│   └── test_floor_vision.py               # Debug visualizer: all YOLO detections on camera
├── src/
│   ├── smrr_bt_mission_executor.cpp       # Service server: loads & ticks BT
│   └── bt_nodes/                          # C++ BT node implementations (~18 files)
│       ├── bt_node_registration.cpp
│       ├── is_same_floor_condition.cpp
│       ├── is_different_floor_condition.cpp
│       ├── get_named_pose_action.cpp
│       ├── get_named_map_action.cpp
│       ├── switch_map_action.cpp
│       ├── publish_initial_pose_action.cpp
│       ├── call_elevator_action.cpp
│       ├── stop_robot_action.cpp
│       ├── set_controller_params_action.cpp
│       ├── set_amcl_params_action.cpp
│       ├── set_costmap_inflation_action.cpp
│       ├── check_floor_arrival_action.cpp
│       ├── check_elevator_direction_action.cpp
│       ├── toggle_apriltag_action.cpp
│       ├── build_pose_vector_action.cpp
│       ├── clear_costmaps_action.cpp
│       ├── detect_call_button_action.cpp
│       ├── update_pose_timestamp_action.cpp
│       ├── wait_for_door_open_action.cpp          # Legacy: depth-based door detection (unused)
│       ├── wait_for_door_open_depth_action.cpp    # Legacy: depth-based door detection (unused)
│       └── wait_for_door_open_model_action.cpp    # Legacy: model-based door detection (unused)
├── CMakeLists.txt   # Hybrid ament_cmake + ament_cmake_python
├── package.xml
└── setup.py         # Python entry points (9 executables)
```

---

### 1.3 Build System

The package uses **hybrid `ament_cmake` + `ament_cmake_python`**:

| Target | Type | Description |
|---|---|---|
| `smrr_bt_nodes` | Shared library | All custom BT node implementations, exported as a BT plugin |
| `smrr_bt_mission_executor` | Executable | C++ service server that loads the BT XML and ticks the tree |

**C++ dependencies:** `rclcpp`, `behaviortree_cpp_v3`, `nav2_behavior_tree`, `nav2_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2`, `tf2_geometry_msgs`, `smrr_interfaces`, `ament_index_cpp`, `yaml-cpp`, ONNX Runtime 1.18

**Python executables (from `setup.py`):**

| Executable | Node Name | Role |
|---|---|---|
| `startup_localizer` | startup_localizer | AMCL convergence via drive+rotate |
| `named_goal_server` | named_goal_server | `/go_to_pose` service: resolve + dispatch |
| `named_goal_client` | — | CLI test client |
| `location_subscriber` | location_subscriber | `/location` topic bridge |
| `floor_arrival_server` | floor_arrival_server | `/check_floor_arrival` action server |
| `elevator_direction_server` | elevator_direction_server | `check_elevator_direction` action server |
| `elevator_call_button_server` | elevator_call_button_server | `detect_call_button` action server — YOLO + ZED2 RANSAC |
| `apriltag_manager_server` | apriltag_manager_server | `/toggle_apriltag` service |
| `door_classifier_node` | door_classifier_node | Legacy ONNX depth inference (not launched) |
| `test_floor_vision` | test_floor_vision | YOLO debug visualizer |

---

## 2. ROS Node Graph

### 2.1 Node Descriptions

Nodes launched by `smrr_world_navigation.launch.py`:

```
┌──────────────────────────────────────────────────────────────────────────┐
│  Gazebo Classic                                                           │
│  Publishes: /scan, /zed2_left_camera/image_raw, /zed2_left_camera/depth  │
│             /diff_drive_controller/odom, /joint_states                   │
└──┬───────────────────────────────────────────────────────────────────────┘
   │
   ├── [nav2_bringup]
   │     ├── map_server           — /map_server/load_map (LoadMap service)
   │     ├── amcl                 — consumes /scan; publishes /amcl_pose
   │     ├── planner_server       — NavfnPlanner (Dijkstra) on global_costmap
   │     ├── controller_server    — MPPI controller; publishes /cmd_vel
   │     ├── bt_navigator         — /navigate_to_pose, /navigate_through_poses
   │     ├── smoother_server      — path smoothing
   │     ├── behavior_server      — spin, backup, wait recoveries
   │     ├── waypoint_follower
   │     ├── velocity_smoother
   │     ├── local_costmap        — obstacle_layer + inflation_layer
   │     └── global_costmap       — static_layer + obstacle_layer + inflation_layer
   │
   ├── smrr_bt_mission_executor   — /start_mission (StartMission service)
   │     Loads smrr_multifloor.xml, ticks BT at 20 Hz, 300 s timeout
   │
   ├── named_goal_server          — /go_to_pose (GoToNamedPose service)
   │     Resolves named location → dispatches to /start_mission
   │
   ├── location_subscriber        — bridges /location (String) → /go_to_pose
   │
   ├── floor_arrival_server       — /check_floor_arrival (CheckFloorArrival action)
   │     Subscribes /zed2_left_camera/image_raw
   │     YOLO-only: detects lit/unlit target button class
   │     Publishes /floor_vision/debug_image (target button boxes only)
   │
   ├── elevator_direction_server  — check_elevator_direction (CheckElevatorDirection action)
   │     Subscribes /zed2_left_camera/image_raw (dynamic, per-goal)
   │     YOLO + HSV: detects up/down arrow illumination
   │     Publishes /floor_vision/debug_image
   │
   ├── elevator_call_button_server  — detect_call_button (DetectCallButton action)
   │     Subscribes /zed2_left_camera/image_raw, /zed2_left_camera/camera_info
   │               /zed2/zed_node/point_cloud/cloud_registered
   │     YOLO class detection + ZED2 organised point-cloud RANSAC plane fit
   │     Publishes /button_press_goal (PoseArray: approach + press in link_0 frame)
   │     Publishes /button_detection/full_cloud, /button_detection/plane_cloud (RViz debug)
   │     Publishes /button_detection/markers (MarkerArray), /floor_vision/debug_image
   │
   ├── apriltag_manager_server    — /toggle_apriltag (std_srvs/SetBool service)
   │     Starts/stops apriltag_ros as a subprocess on demand
   │
   ├── startup_localizer (conditional, enabled via launch arg)
   │     Drives forward 1 m, rotates 360° for AMCL convergence
   │
   └── rviz2
```

---

### 2.2 Topic & Service Summary

#### Published Topics

| Topic | Type | Publisher | Purpose |
|---|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | controller_server, startup_localizer | Robot velocity commands |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | PublishInitialPose BT node | AMCL re-localization trigger |
| `/floor_vision/debug_image` | sensor_msgs/Image | floor_arrival_server, elevator_direction_server, elevator_call_button_server | Annotated YOLO frames |
| `/button_press_goal` | geometry_msgs/PoseArray | elevator_call_button_server | Approach + press poses in `link_0` frame for arm controller |
| `/button_detection/full_cloud` | sensor_msgs/PointCloud2 | elevator_call_button_server | Raw ZED2 scene cloud re-published for RViz context |
| `/button_detection/plane_cloud` | sensor_msgs/PointCloud2 | elevator_call_button_server | RANSAC inlier points (green) for RViz overlay |
| `/button_detection/markers` | visualization_msgs/MarkerArray | elevator_call_button_server | Arrow, sphere, text markers in `link_0_fake` frame |

#### Subscribed Topics

| Topic | Type | Subscriber | Purpose |
|---|---|---|---|
| `/location` | std_msgs/String | location_subscriber | High-level location command |
| `/zed2_left_camera/image_raw` | sensor_msgs/Image | floor_arrival_server, elevator_direction_server, elevator_call_button_server | Camera feed for YOLO |
| `/zed2_left_camera/camera_info` | sensor_msgs/CameraInfo | elevator_call_button_server | Camera intrinsics for pinhole ray-plane intersection |
| `/zed2/zed_node/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 | elevator_call_button_server | Organised ZED2 point cloud for RANSAC plane fit |
| `/zed2_left_camera/depth/image_raw` | sensor_msgs/Image | door_classifier_node (legacy) | Depth for ONNX classifier |
| `/diff_drive_controller/odom` | nav_msgs/Odometry | startup_localizer | Odometry for motion tracking |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | named_goal_server | Current pose estimate |
| `/scan` | sensor_msgs/LaserScan | Nav2 AMCL + costmaps | LiDAR for localization & obstacles |

#### Services & Actions

| Name | Type | Provider | Caller |
|---|---|---|---|
| `/go_to_pose` | GoToNamedPose srv | named_goal_server | location_subscriber, CLI |
| `/start_mission` | StartMission srv | smrr_bt_mission_executor | named_goal_server |
| `/toggle_apriltag` | std_srvs/SetBool | apriltag_manager_server | ToggleAprilTag BT node |
| `/map_server/load_map` | nav2_msgs/LoadMap | map_server | SwitchMap BT node |
| `/navigate_to_pose` | nav2_msgs/NavigateToPose action | bt_navigator | NavigateToPose BT node |
| `/navigate_through_poses` | nav2_msgs/NavigateThroughPoses action | bt_navigator | NavigateThroughPoses BT node |
| `/compute_path_to_pose` | nav2_msgs/ComputePathToPose action | planner_server | ComputePathToPose BT node (door detection) |
| `*/spin` | nav2_msgs/Spin action | behavior_server | Spin BT node |
| `/check_floor_arrival` | CheckFloorArrival action | floor_arrival_server | CheckFloorArrival BT node |
| `check_elevator_direction` | CheckElevatorDirection action | elevator_direction_server | CheckElevatorDirection BT node |
| `detect_call_button` | DetectCallButton action | elevator_call_button_server | DetectCallButton BT node |
| `/global_costmap/clear_entirely_global_costmap` | nav2_msgs/ClearEntireCostmap | Nav2 | ClearEntireCostmap BT node |
| `/local_costmap/clear_entirely_local_costmap` | nav2_msgs/ClearEntireCostmap | Nav2 | ClearEntireCostmap BT node |
| `/controller_server/set_parameters` | rcl_interfaces/SetParameters | controller_server | SetControllerParams BT node |
| `/amcl/set_parameters` | rcl_interfaces/SetParameters | amcl | SetAMCLParams BT node |
| `/local_costmap/local_costmap/set_parameters` | rcl_interfaces/SetParameters | local_costmap | SetCostmapInflation BT node |
| `/global_costmap/global_costmap/set_parameters` | rcl_interfaces/SetParameters | global_costmap | SetCostmapInflation BT node |

---

### 2.3 Data Flow Diagram

```
External / UI ──→ /location (std_msgs/String) "office_101"
                          │
                  location_subscriber
                          │ calls
                  /go_to_pose (GoToNamedPose srv)
                          │
                  named_goal_server
                  ├─ Load locations.yaml
                  ├─ Resolve "office_101" → floor1, (5.90, -2.29, 0.0)
                  ├─ current_floor = floor0  (tracked internally)
                  └─ calls
                  /start_mission (StartMission srv)
                          │
                  smrr_bt_mission_executor (C++)
                  ├─ Set blackboard: current_floor_id, target_floor_id, final_pose
                  ├─ Load smrr_multifloor.xml
                  └─ Tick BT at 20 Hz
                          │
            ┌─────────────┴──────────────────┐
       IsSameFloor?                    IsDifferentFloor?
            │                                │
    NavigateToPose                  [Cross-floor sequence]
    (Nav2 → MPPI)                   → Navigate via staging_waypoint → elevator_staging
                                    → DetectCallButton (YOLO + ZED2 RANSAC → /button_press_goal)
                                    → WaitForPath_Entry (global clear + 2s poll loop)
                                    → NavigateThroughPoses (entry → inside)
                                    → SpinInElevator ≈−150° (AMCL frozen)
                                    → PublishBoolTopic(/going_in)
                                    → PressFloorButton (MoveIt IBVS)
                                    → SwitchMap + PublishInitialPose (AprilTag corrected)
                                    → PublishBoolTopic(/going_out)
                                    → CheckFloorArrival (YOLO button detection)
                                    → WaitForPath_Exit (100ms poll loop)
                                    → NavigateToPose (exit)
                                    → NavigateToPose (final destination)
```

---

## 3. Custom Interfaces (`smrr_interfaces`)

### `GoToNamedPose.srv`

```yaml
string name          # e.g. "office_101", "dock"
---
bool   accepted
string message
```

### `StartMission.srv`

```yaml
string   mission_id
string   current_floor_id      # e.g. "floor0"
string   target_floor_id       # e.g. "floor1"
string   target_location_name  # e.g. "office_101"
float64  x
float64  y
float64  yaw
---
bool     accepted
bool     success
int32    nav_status
string   message
```

### `CheckFloorArrival.action`

```yaml
# Goal
string target_floor            # e.g. "floor1"
---
# Result
bool   arrived
string message
---
# Feedback
string  status   # "ON" | "OFF_UNSTABLE" | "NO_DETECTION"
float32 ratio    # illuminated-pixel ratio (kept for feedback compatibility)
```

### `CheckElevatorDirection.action`

```yaml
# Goal
string current_floor   # e.g. "floor0"
string target_floor    # e.g. "floor2"
---
# Result
bool   correct_direction_confirmed
string message
---
# Feedback
string  expected_direction  # "UP" or "DOWN"
string  status              # "ON" | "OFF_STABLE" | "NO_DETECTION"
float32 ratio
```

### `NavigateToNamedLocation.action` (Legacy — not used in active BT)

```yaml
# Goal
string   location_name
string   target_floor_id
float64  x / y / yaw
---
# Result
bool success; int32 nav_status; string message
---
# Feedback
string state; string active_floor_id; string active_step
```

### `DetectCallButton.action`

```yaml
# Goal: floors to determine direction (UP/DOWN) and target button class
string current_floor    # e.g. "floor0"
string target_floor     # e.g. "floor2"
---
# Result: two waypoints in link_0 frame (computed via link_0_fake TF)
bool   success
float64 approach_x      # 6 cm outward along button plane normal
float64 approach_y
float64 approach_z
float64 press_x         # 6 cm into button surface from plane
float64 press_y
float64 press_z
string message
---
# Feedback: per-cycle status
string status           # WAITING_FOR_DATA | NO_DETECTION | BUTTON_LIT | DEPTH_INSUFFICIENT | LOCALISED
string direction        # UP | DOWN
```

---

## 4. Python Nodes

### 4.1 named_goal_server.py

**Service:** `/go_to_pose` (GoToNamedPose)

Resolves a location name to a `(floor_id, x, y, yaw)` tuple using the floor-aware `locations.yaml` database. Maintains the robot's `current_floor_id` across missions (updated on each success).

**Execution mode:** BT executor mode only — always calls `/start_mission`.

**Key flow:**
```
handle_go_to_pose(name)
  ├─ resolve name → (target_floor, pose)
  ├─ build StartMission request with UUID mission_id
  ├─ call /start_mission (blocking, 300 s timeout)
  ├─ if success: update self.current_floor_id = target_floor
  └─ return accepted + message
```

**Parameters (from launch):**
- `locations_file`: path to `locations.yaml`
- `initial_floor_id`: starting floor (default `floor0`)

---

### 4.2 floor_arrival_server.py

**Action:** `/check_floor_arrival` (CheckFloorArrival)

Determines when the elevator arrives at the target floor by monitoring the floor button panel with YOLO. No HSV thresholding — purely class-based detection.

**YOLO class mapping:**

| Floor | Unlit class | Lit class |
|---|---|---|
| floor0 | 2 | 3 |
| floor1 | 4 | 5 |
| floor2 | 8 | 9 |
| floor3 | 6 | 7 |

**State machine (runs at 20 Hz):**

```
lit class detected
  → status = "ON"  (elevator moving, button illuminated)
  → reset off_stable_start

unlit class detected, no lit class
  → start off_stable_start timer if not started
  → if elapsed >= 0.1 s → succeed (arrived = true)
  → else status = "OFF_UNSTABLE"

neither class detected
  → status = "NO_DETECTION"
  → reset off_stable_start
```

**Debug image:** `/floor_vision/debug_image` shows **only** the target floor's bounding boxes.
Green box = lit (ON). Orange box = unlit (OFF).

**Key constants:**
- `INFERENCE_CONF`: 0.15
- `OFF_STABLE_DURATION_SEC`: 0.1 s
- `LOOP_HZ`: 20

---

### 4.3 elevator_direction_server.py

**Action:** `check_elevator_direction` (CheckElevatorDirection)

Confirms the elevator is travelling in the correct direction (UP/DOWN) by detecting illuminated arrow lights on the elevator panel.

**Direction inference:**
```python
target_idx   = int(target_floor.replace('floor', ''))
current_idx  = int(current_floor.replace('floor', ''))
expected_dir = "UP" if target_idx > current_idx else "DOWN"
target_class = "up" if expected_dir == "UP" else "down"
```

**Detection pipeline:**
1. YOLO finds the `up` or `down` arrow in the frame
2. Crop ROI from detected bounding box
3. Convert ROI to HSV, apply orange/yellow threshold mask (`[5,50,150]–[35,255,255]`)
4. If mask coverage ≥ 4% for ≥ 0.1 s → direction confirmed

**Resource optimization:** Image subscription is created on goal start and destroyed on completion to avoid continuous CPU overhead when the server is idle.

**Key constants:**
- `YOLO_CONF`: 0.1
- `HSV_LOWER`: `[5, 50, 150]`, `HSV_UPPER`: `[35, 255, 255]`
- `ON_RATIO_THRESHOLD`: 0.04
- `OFF_STABLE_DURATION_SEC`: 0.1 s

---

### 4.4 apriltag_manager_server.py

**Service:** `/toggle_apriltag` (std_srvs/SetBool)

Dynamically starts and stops `apriltag_ros` as a subprocess. Saves CPU by running AprilTag detection only during elevator transitions when AMCL re-localization is needed.

**Command executed on start:**
```bash
ros2 run apriltag_ros apriltag_node \
  -r image_rect:=/zed2_left_camera/image_raw \
  -r camera_info:=/zed2_left_camera/camera_info \
  -p family:=36h11 \
  -p size:=0.15
```

Stop sends SIGTERM, waits 5 s, then SIGKILL. The process is launched with `os.setsid()` for process group isolation.

---

### 4.5 startup_localizer.py

Executes a predefined motion sequence after startup to help AMCL's particle filter converge before the first navigation goal is issued.

**State machine:**

| State | Action | Completion Condition |
|---|---|---|
| INIT | Wait | `startup_delay` seconds elapsed (default 2.0 s) |
| FORWARD | Publish +x velocity | Euclidean distance ≥ `forward_distance` (1.0 m) |
| STOP1 | Publish zero velocity | `stop_duration` seconds elapsed (1.0 s) |
| ROTATE | Publish +z angular velocity | Accumulated yaw ≥ 2π (360°) |
| STOP2 | Publish zero velocity | `stop_duration` seconds elapsed |
| DONE | Shutdown node | — |

Yaw accumulation uses `normalize_angle()` to handle the ±π wraparound.

**Parameters:** `startup_delay=2.0s`, `forward_speed=0.15 m/s`, `forward_distance=1.0 m`, `rotation_speed=0.5 rad/s`, `stop_duration=1.0 s`

---

### 4.6 location_subscriber.py

Bridges the `/location` (String) topic to the `/go_to_pose` service. Allows external systems (e.g., a task planner or UI) to command navigation without knowing the service interface.

```
/location topic → stripped string → call /go_to_pose → log response
```

---

### 4.7 test_floor_vision.py

Standalone debug visualizer. Runs YOLO on the live camera feed and publishes an annotated image to `/floor_vision/debug_image` showing **all** detected classes — no target floor required, starts immediately on launch.

- Confidence label placed **to the right** of each bounding box (falls back to left if near frame edge)
- Uses the same `MODEL_PATH` and `INFERENCE_CONF` as `floor_arrival_server.py`

---

### 4.8 elevator_call_button_server.py

**Action:** `detect_call_button` (DetectCallButton)

Detects the elevator call button (UP or DOWN) and localises it in 3D using the ZED2 left camera and its organised point cloud. On success it publishes two arm waypoints — an approach point 6 cm in front of the button plane and a press point 6 cm into the surface — to `/button_press_goal` for the arm controller.

**Direction inference:**
```python
direction = "UP" if int(target_floor[-1]) > int(current_floor[-1]) else "DOWN"
```

**YOLO class mapping (button_detection.pt):**

| Direction | Unlit class | Lit class |
|---|---|---|
| UP (call) | 10 | 11 |
| DOWN (call) | 0 | 1 |

**Detection pipeline per goal cycle:**
1. Wait until RGB image, camera info, and ZED2 registered point cloud are all available.
2. Run YOLO at 20 Hz (`conf=0.15`) on the RGB frame to find the `unlit_class` bounding box.
3. If the `lit_class` is detected (button already illuminated) → succeed immediately (`BUTTON_LIT`).
4. If no unlit button detected → feedback `NO_DETECTION`, retry next frame.
5. Call `_extract_button_pose_pointcloud()`:
   - Scale YOLO bounding box from image pixel space to cloud pixel space.
   - Crop a center-focused sub-ROI (`BBOX_CENTER_SHRINK=0.40`) to suppress wall points.
   - Decode raw ZED2 `PointCloud2` bytes directly via `np.frombuffer` + `np.view(float32)`.
   - Filter: remove NaN/inf, keep depths 0.05–5.0 m, apply near-depth percentile gate (15th percentile + 5 mm band), apply median-depth outlier filter (±20 mm).
   - RANSAC plane fit: 200 iterations, 3 mm inlier threshold, minimum 8 inliers.
   - SVD refinement on inlier subset → best-fit plane normal + scalar D.
   - Frame-agnostic normal enforcement: `dot(normal, -centroid) > 0` (toward camera).
   - **Button center via ray-plane intersection:** bbox center pixel → pinhole ray in optical frame → TF2 rotation into cloud frame → intersect with RANSAC plane. This decouples lateral position (bbox center pixel) from depth (plane). Falls back to inlier centroid on TF failure.
   - Compute `approach_cam = button_centroid + 0.06 * normal` and `press_cam = button_centroid - 0.06 * normal`.
   - TF2 transform `approach_cam` and `press_cam` from cloud frame → `link_0_fake` (= `link_0`).
6. Publish `PoseArray` to `/button_press_goal` (header `frame_id = "link_0"`, poses[0]=approach, poses[1]=press) with `EEF_ORIENTATION = (-0.039, 0.691, 0.656, -0.301)`.
7. Publish RViz visualizations: `/button_detection/full_cloud`, `/button_detection/plane_cloud` (green inliers), `/button_detection/markers` (arrow + spheres + text in `link_0_fake`).
8. Return action result with approach/press coordinates in `link_0_fake`.

**Per-direction goal offsets** applied in `link_0_fake` before publishing to `/button_press_goal`:

| Direction | X offset | Y offset | Z offset |
|---|---|---|---|
| UP | 0.0 m | −0.07 m | 0.0 m |
| DOWN | 0.0 m | −0.07 m | −0.01 m |

**Key constants:**

| Constant | Value | Meaning |
|---|---|---|
| `INFERENCE_CONF` | 0.15 | YOLO confidence threshold |
| `APPROACH_DIST_M` | 0.06 m | Distance outward from button plane |
| `PRESS_INSET_M` | −0.06 m | Distance into button surface |
| `BBOX_CENTER_SHRINK` | 0.40 | Center-crop ratio to reduce wall dominance |
| `NEAR_DEPTH_PERCENTILE` | 15.0 | Percentile for near-depth gate |
| `NEAR_DEPTH_BAND_M` | 0.005 m | Band above near-depth percentile |
| `CAMERA_FRAME` | `zed2_left_camera_frame_optical` | Optical frame for ray direction |
| `TARGET_FRAME` | `link_0_fake` | Output coordinate frame |
| `PUBLISH_FRAME_ID` | `link_0` | `frame_id` written into `/button_press_goal` header |
| `POINTCLOUD_TOPIC` | `/zed2/zed_node/point_cloud/cloud_registered` | ZED2 organised point cloud |

**Subscribers (persistent, node lifetime):**
- `/zed2_left_camera/image_raw` (Image)
- `/zed2_left_camera/camera_info` (CameraInfo)
- `/zed2/zed_node/point_cloud/cloud_registered` (PointCloud2)

---

### 4.9 door_classifier_node.py (Legacy)

ONNX-based depth image classifier. Loads `door_classifier_3.onnx` (TinyCNN, ~15K params) and classifies elevator door state as OPEN/CLOSED from a 96×96 normalised depth patch.

**Status:** Compiled and available but **not launched** in any active launch file. Superseded by the planning-based door detection approach in the BT.

**Preprocessing spec (must match training):**
1. Replace NaN/inf/≤0 with `clip_max_m` (5.0 m) — invalid = far, not near
2. Clip to [0.2, 5.0] m
3. Normalize: `(depth − 0.2) / 4.8` → [0, 1]
4. Resize to 96×96 using `cv2.INTER_AREA`
5. Reshape to `[1, 1, 96, 96]`

---

## 5. Behavior Tree Architecture

### 5.1 BT Mission Executor (C++)

**Node:** `smrr_bt_mission_executor`
**Service:** `/start_mission` (StartMission)

On each `StartMission` request:
1. Build a `PoseStamped` from `(x, y, yaw)` in the `map` frame
2. Create a `BehaviorTreeEngine` and load the `smrr_bt_nodes` plugin library
3. Create a blackboard and set:
   - `node` → `this->shared_from_this()`
   - `current_floor_id` / `target_floor_id` / `final_pose` from request
   - `locations_file` → absolute path to `locations.yaml`
   - `server_timeout`, `bt_loop_duration`, `wait_for_service_timeout`
4. Load `smrr_multifloor.xml` from the installed share directory
5. Tick loop at **20 Hz** until the tree returns non-`RUNNING` or 300 s elapses
6. Map `SUCCESS`/`FAILURE`/exception to `StartMission` response fields

**Callback group:** `ReentrantCallbackGroup` — allows BT nodes to make service/action calls while `StartMission` is blocking.

---

### 5.2 smrr_multifloor.xml — Full Structure

```
Root (BehaviorTree id="MissionTree")
└─ Fallback
   ├─ Sequence [SameFloor]
   │  ├─ IsSameFloor
   │  └─ NavigateToPose
   │
   └─ Sequence [CrossFloor_ElevatorEntry]
      ├─ IsDifferentFloor
      ├─ SetGoalCheckerParams (xy=0.20, yaw=0.20)
      ├─ SetCostmapInflation (0.325)
      ├─ GetNamedPose (elevator_staging, current_floor)
      ├─ GetNamedPose (staging_waypoint, current_floor)
      ├─ NavigateToPose (→ staging_waypoint_pose)
      ├─ SetGoalCheckerParams (xy=0.15, yaw=0.20)
      ├─ NavigateToPose (→ elevator_staging)
      ├─ GetNamedPose (elevator_inside, current_floor)         ← pre-fetched for DetectCallButton blackboard
      ├─ SetCostmapInflation (0.265)                          ← tight: inside_pose near walls
      ├─ SetControllerParams (max_vel_x=0.75)                 ← speed set before waiting, ready for gap
      ├─ DetectCallButton (current_floor → target_floor)      ← YOLO + ZED2 RANSAC → /button_press_goal
      │    Also verifies press via BUTTON_LIT or DOOR_OPEN
      ├─ GetNamedPose (elevator_center, current_floor)        ← separate key from inside_pose, tunable
      │
      ├─ RetryUntilSuccessful(2) [CallElevatorAndEnter_Twice]
      │  └─ Sequence [CallWaitAndEnter]
      │     ├─ (CallElevator — commented out)
      │     ├─ RetryUntilSuccessful(150) [WaitForPath_Entry]
      │     │  └─ Sequence
      │     │     ├─ ClearEntireCostmap (global only)         ← NavFn uses global costmap
      │     │     └─ Delay(2000ms) → ComputePathToPose(center_pose)
      │     │        global_costmap update_frequency=0.5 Hz → 2000ms guarantees ≥1 full cycle
      │     ├─ ClearEntireCostmap (local + global)            ← pre-navigation flush
      │     ├─ GetNamedPose (elevator_entry, current_floor)
      │     ├─ GetNamedPose (elevator_inside, current_floor)
      │     ├─ BuildPoseVector (entry_pose + inside_pose)
      │     └─ NavigateThroughPoses
      │
      ├─ StopRobot (800 ms)
      ├─ Wait (1.0 s)
      ├─ ToggleAprilTag (true)
      ├─ SetAMCLParams (update_min_d=10.0, update_min_a=10.0)  ← freeze during spin
      ├─ Fallback [SpinInElevator]
      │  ├─ Spin (-2.61799 rad, 40 s)                          ← 150° ≈ face outward
      │  └─ Sequence [RepositionAndSpin]
      │     ├─ GetNamedPose (elevator_inside, current_floor)
      │     ├─ NavigateToPose (→ inside_pose)
      │     └─ Spin (-2.61799 rad, 40 s)
      ├─ PublishBoolTopic (/going_in = true)
      ├─ StopRobot (800 ms)
      ├─ SetAMCLParams (update_min_d=0.15, update_min_a=0.1)   ← restore AMCL
      ├─ PressFloorButton (target_floor_id)
      ├─ Wait (1.0 s)                                           ← min wait before map switch
      │  (WaitForDoorClose — commented out)
      │  (Wait 3.0 s — commented out)
      │
      ├─ GetNamedMap (target_floor, "open")
      ├─ GetNamedPose (amcl_initial_pose_open, target_floor)
      ├─ GetNamedPose (amcl_initial_pose_open, current_floor)
      ├─ SwitchMap (map_open_target)
      ├─ PublishCurrentFloor
      ├─ PublishInitialPose (use_apriltag=true, tag_frame=tag36h11:0)
      ├─ ClearEntireCostmap (local)
      ├─ ClearEntireCostmap (global)
      ├─ ToggleAprilTag (false)
      │
      ├─ PublishBoolTopic (/going_out = true)
      ├─ Fallback [ExitElevator_WithRepositionRetry]
      │  ├─ Sequence [ExitAttempt_1]
      │  │  ├─ GetNamedPose (elevator_exit, target_floor)
      │  │  ├─ (CallElevator — commented out)
      │  │  ├─ CheckFloorArrival (target_floor)
      │  │  ├─ SetControllerParams (max_vel_x=0.75)
      │  │  ├─ ClearEntireCostmap (local + global)
      │  │  ├─ RetryUntilSuccessful(3000) [WaitForPath_Exit1]
      │  │  │  └─ Delay(100ms) → ComputePathToPose(exit_pose)
      │  │  ├─ SetGoalCheckerParams (xy=0.25, yaw=0.25)
      │  │  ├─ NavigateToPose (→ exit_pose)
      │  │  ├─ SetControllerParams (max_vel_x=0.25)
      │  │  ├─ SetCostmapInflation (0.5)
      │  │  └─ PublishCurrentFloor
      │  │
      │  └─ Sequence [RepositionThenExitAttempt_2]
      │     ├─ GetNamedPose (elevator_exit, target_floor)
      │     ├─ GetNamedPose (elevator_inside, target_floor)
      │     ├─ NavigateToPose (→ inside_pose)
      │     ├─ Spin (-2.61799 rad, 10 s)
      │     ├─ StopRobot (800 ms)
      │     ├─ (CallElevator — commented out)
      │     ├─ CheckFloorArrival (target_floor)
      │     ├─ SetControllerParams (max_vel_x=0.75)
      │     ├─ ClearEntireCostmap (local + global)
      │     ├─ RetryUntilSuccessful(3000) [WaitForPath_Exit2]
      │     │  └─ Delay(100ms) → ComputePathToPose(exit_pose)
      │     ├─ SetGoalCheckerParams (xy=0.25, yaw=0.25)
      │     ├─ NavigateToPose (→ exit_pose)
      │     ├─ SetControllerParams (max_vel_x=0.25)
      │     ├─ SetCostmapInflation (0.5)
      │     └─ PublishCurrentFloor
      │
      └─ NavigateToPose (→ final_pose)  ← destination on target floor
```

---

### 5.3 Same-Floor Branch

```xml
<Sequence name="SameFloor">
  <IsSameFloor current_floor="{current_floor_id}" target_floor="{target_floor_id}"/>
  <NavigateToPose server_name="/navigate_to_pose" goal="{final_pose}"/>
</Sequence>
```

If `current_floor_id == target_floor_id`, dispatch directly to Nav2. No elevator logic runs.

---

### 5.4 Cross-Floor Branch — Elevator Entry

The pre-entry setup runs **once** regardless of retries:

```xml
<SetGoalCheckerParams xy_goal_tolerance="0.20" yaw_goal_tolerance="0.20"/>
<SetCostmapInflation inflation_radius="0.325"/>
<!-- Navigate to staging via intermediate waypoint -->
<GetNamedPose ... location_key="staging_waypoint" pose="{staging_waypoint_pose}"/>
<NavigateToPose ... goal="{staging_waypoint_pose}"/>
<SetGoalCheckerParams xy_goal_tolerance="0.15" yaw_goal_tolerance="0.20"/>
<NavigateToPose ... goal="{staging_pose}"/>
<!-- Pre-fetch inside_pose BEFORE DetectCallButton (blackboard needed by action server) -->
<GetNamedPose ... location_key="elevator_inside" pose="{inside_pose}"/>
<!-- Reduce inflation: inside_pose is near elevator walls -->
<SetCostmapInflation inflation_radius="0.265"/>
<!-- Boost velocity before waiting: robot crosses gap at full speed the instant door opens -->
<SetControllerParams max_vel_x="0.75"/>
<!-- Detect call button and arm waypoints -->
<DetectCallButton current_floor="{current_floor_id}" target_floor="{target_floor_id}" inside_pose="{inside_pose}"/>
<!-- Separate center pose key — tunable independently from inside_pose -->
<GetNamedPose ... location_key="elevator_center" pose="{center_pose}"/>
```

`DetectCallButton` triggers `elevator_call_button_server`: YOLO locates the UP (class 10/11) or DOWN (class 0/1) button, RANSAC fits the ZED2 point cloud, ray-plane intersection computes the button center, and approach/press waypoints are published to `/button_press_goal`. The action also verifies button press via `BUTTON_LIT` or `DOOR_OPEN` detection before returning SUCCESS.

**Why `inside_pose` is pre-fetched before `DetectCallButton`:** The call button server receives `inside_pose` as an action goal field and uses it internally for door-open verification after pressing the button. It must be on the blackboard before the action is called.

**Why `elevator_center` is a separate YAML key from `elevator_inside`:** `ComputePathToPose` for door polling uses `center_pose` (tunable for optimal planner sensitivity) while `NavigateThroughPoses` uses `inside_pose` (robot parking position). Decoupling the two keys allows each to be calibrated independently.

The entry retry loop (wraps only the wait-and-enter sequence, not `DetectCallButton`):

```xml
<RetryUntilSuccessful num_attempts="2" name="CallElevatorAndEnter_Twice">
  <Sequence name="CallWaitAndEnter">
    <!-- CallElevator commented out — physical button press handled by DetectCallButton / arm -->

    <!-- Door-open detection: clear global costmap, wait 2000 ms, then poll planner -->
    <RetryUntilSuccessful num_attempts="150" name="WaitForPath_Entry">
      <Sequence>
        <!-- Global costmap only: NavFn (GridBased) plans on global costmap.
             Local costmap (MPPI) is cleared once after the loop, not per-iteration. -->
        <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
        <!-- 2000 ms = 1 full global costmap update cycle (update_frequency=0.5 Hz).
             Ensures door re-marks before NavFn checks; prevents false positives. -->
        <Delay delay_msec="2000">
          <ComputePathToPose goal="{center_pose}" path="{dummy_path}" planner_id="GridBased"/>
        </Delay>
      </Sequence>
    </RetryUntilSuccessful>
    <!-- Max wait: 150 × 2 s = 5 min -->

    <!-- Flush both costmaps before entry: LiDAR repopulates during polling loop -->
    <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
    <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>

    <GetNamedPose location_key="elevator_entry" .../>
    <GetNamedPose location_key="elevator_inside" .../>
    <BuildPoseVector pose1="{entry_pose}" pose2="{inside_pose}" poses="{entry_poses}"/>
    <NavigateThroughPoses server_name="/navigate_through_poses" goals="{entry_poses}"/>
  </Sequence>
</RetryUntilSuccessful>
```

**Why global-only clear inside the loop:** `ComputePathToPose` (NavFn / `GridBased` planner ID) reads only the **global costmap**. The local costmap is consumed by the MPPI controller during actual navigation, not path planning. Clearing the local costmap per iteration wastes ~50 ms with no benefit.

**Why 2000 ms delay:** `global_costmap.update_frequency = 0.5 Hz` means the obstacle layer processes new LiDAR scans only every 2000 ms. After `ClearEntireCostmap` empties the global grid, a closed door will only be re-marked on the **next** global costmap update cycle. With a shorter delay (e.g., 500 ms), the global grid may still be empty when NavFn runs — a closed door would appear navigable, sending the robot into the door.

**Why inflation 0.265 m:** `robot_radius = 0.275 m`. At `inflation_radius = 0.265 m`, the planner can still route through the elevator doorway. Higher values (0.325+) cause the goal cell at `elevator_center` / `elevator_inside` to fall inside the inflation zone of nearby walls, making NavFn fail even with the door open.

**Why velocity 0.75 m/s:** Carries the robot through the narrow door gap with momentum; prevents MPPI from stopping mid-gap due to micro-obstacle avoidance.

---

### 5.5 Cross-Floor Branch — In-Elevator Repositioning

```xml
<StopRobot topic="/cmd_vel" repeat_ms="200" duration_ms="800"/>
<Wait wait_duration="1.0"/>
<ToggleAprilTag turn_on="true"/>
<SetAMCLParams update_min_d="10.0" update_min_a="10.0"/>  <!-- freeze -->
<Fallback name="SpinInElevator">
  <Spin spin_dist="-2.61799" time_allowance="40.0" is_recovery="true"/>
  <Sequence name="RepositionAndSpin">
    <GetNamedPose ... location_key="elevator_inside" pose="{inside_pose}"/>
    <NavigateToPose ... goal="{inside_pose}"/>
    <Spin spin_dist="-2.61799" time_allowance="40.0" is_recovery="true"/>
  </Sequence>
</Fallback>
<PublishBoolTopic topic="/going_in" value="true"/>
<StopRobot topic="/cmd_vel" repeat_ms="200" duration_ms="800"/>
<SetAMCLParams update_min_d="0.15" update_min_a="0.1"/>    <!-- restore -->
<PressFloorButton target_floor="{target_floor_id}"/>
<Wait wait_duration="1.0"/>
```

The robot spins **−2.61799 rad** (≈ −150°) so it faces the elevator exit. The `SpinInElevator` fallback retries by re-navigating to `elevator_inside` and spinning again if the first spin fails. AMCL is frozen during the spin to prevent particle filter corruption from poor in-elevator LiDAR geometry.

`PublishBoolTopic /going_in = true` signals external nodes (e.g., human tracker, base controller) that the robot is entering the elevator.

`PressFloorButton` triggers the MoveIt arm to press the target floor button on the elevator panel using endoscopic camera IBVS. The BT blocks here until `/press_complete` is received and the button is confirmed lit.

After `PressFloorButton`, `Wait 1.0 s` provides a mandatory minimum settle time.

---

### 5.6 Cross-Floor Branch — Door Close & Map Switch

`WaitForDoorClose` and the 3 s post-close `Wait` are currently **commented out** in the BT. The map switch happens immediately after the 1 s settle wait following `PressFloorButton`:

```xml
<!-- WaitForDoorClose commented out — map switch fires immediately after press -->

<!-- Fetch both floors' open-map poses (target + current) onto blackboard -->
<GetNamedMap locations_file="{locations_file}" floor_id="{target_floor_id}" map_key="open" map_yaml="{map_open_target}"/>
<GetNamedPose ... floor_id="{target_floor_id}" location_key="amcl_initial_pose_open" pose="{amcl_open_target}"/>
<GetNamedPose ... floor_id="{current_floor_id}" location_key="amcl_initial_pose_open" pose="{amcl_open_current}"/>
<SwitchMap map_yaml="{map_open_target}"/>
<!-- Publish floor update immediately so named_goal_server tracks correct floor
     even if exit navigation subsequently fails -->
<PublishCurrentFloor/>
<PublishInitialPose initial_pose="{amcl_open_target}"
                    use_apriltag="true"
                    tag_frame="tag36h11:0"
                    expected_tag_x="1.313"  expected_tag_y="0.071"  expected_tag_z="1.412"
                    expected_tag_roll="1.581" expected_tag_pitch="0.000" expected_tag_yaw="-1.633"/>
<ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
<ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
<ToggleAprilTag turn_on="false"/>
```

**Why `WaitForDoorClose` is commented out:** After the map is switched and `PublishInitialPose` runs, the exit door detection (`WaitForPath_Exit`) handles waiting for the correct floor's door to open. An explicit door-close wait is not required for correctness because `CheckFloorArrival` (YOLO) already confirms the elevator has arrived before the exit door poll begins.

**`PublishCurrentFloor`:** Published immediately after `SwitchMap` — before exit navigation — so the `named_goal_server` floor tracker is consistent even if the robot subsequently fails to exit.

**AprilTag re-localization:** The `PublishInitialPose` node queries `/tf` for the detected AprilTag transform and computes a parking-error correction:
```
corrected_pose = nominal_pose + (expected_tag_pose − actual_tag_pose)
```
This corrects for the elevator not stopping at the exact same position each time. Both costmaps are cleared afterwards to remove LiDAR contamination accumulated during the map switch transition.

---

### 5.7 Cross-Floor Branch — Elevator Exit

`PublishBoolTopic /going_out = true` is published before the exit fallback, signalling external nodes.

The exit uses a `Fallback` (`ExitElevator_WithRepositionRetry`) with two attempts. If the first attempt fails, attempt 2 re-navigates to `elevator_inside`, re-spins to reorient, and repeats the full exit sequence.

**Attempt 1 (`ExitAttempt_1`):**
```
GetNamedPose(elevator_exit, target_floor)
(CallElevator — commented out)
CheckFloorArrival(target_floor)        ← YOLO: wait until floor button extinguishes
SetControllerParams(max_vel_x=0.75)    ← speed through door
ClearEntireCostmap (local + global)    ← clear LiDAR noise accumulated inside elevator
WaitForPath_Exit1 (3000 × 100ms)       ← poll at 10 Hz until door open
  └─ Delay(100ms) → ComputePathToPose(exit_pose)
SetGoalCheckerParams(xy=0.25, yaw=0.25)
NavigateToPose(exit_pose)
SetControllerParams(max_vel_x=0.25)    ← restore normal speed
SetCostmapInflation(0.5)               ← restore normal inflation
PublishCurrentFloor                    ← confirm floor update after successful exit
```

**Attempt 2 (`RepositionThenExitAttempt_2`):**
```
GetNamedPose(elevator_exit, target_floor)
GetNamedPose(elevator_inside, target_floor)
NavigateToPose(elevator_inside)        ← reposition
Spin(-2.61799 rad, 10s)                ← reorient
StopRobot (800 ms)
(CallElevator — commented out)
CheckFloorArrival(target_floor)
SetControllerParams(max_vel_x=0.75)
ClearEntireCostmap (local + global)
WaitForPath_Exit2 (3000 × 100ms)
SetGoalCheckerParams(xy=0.25, yaw=0.25)
NavigateToPose(exit_pose)
SetControllerParams(max_vel_x=0.25)
SetCostmapInflation(0.5)
PublishCurrentFloor
```

**Why exit polls at 100ms (10 Hz) vs. entry at 2000ms:** At exit the robot is inside the elevator, AMCL is already re-localised on the target floor map, and there is no risk of false-positive from a stale empty global costmap (the global costmap was cleared and freshly populated during `CheckFloorArrival`). The 100ms poll gives faster response when the door opens.

---

### 5.8 Cross-Floor Branch — Final Destination

```xml
<NavigateToPose server_name="/navigate_to_pose" goal="{final_pose}"/>
```

After successful exit, the robot navigates to the original target destination using standard Nav2 on the newly loaded floor map.

---

### 5.9 Custom BT Node Reference

| Node | Type | Description |
|---|---|---|
| `IsSameFloor` | Condition | Returns SUCCESS if `current_floor_id == target_floor_id` |
| `IsDifferentFloor` | Condition | Returns SUCCESS if floors differ |
| `GetNamedPose` | SyncAction | Reads `(x, y, yaw)` from `locations.yaml` for a given floor + location key; writes `PoseStamped` to blackboard |
| `GetNamedMap` | SyncAction | Reads map YAML path from `locations.yaml` for a given floor + map key |
| `BuildPoseVector` | SyncAction | Combines two `PoseStamped` values into a `vector<PoseStamped>` for `NavigateThroughPoses` |
| `CallElevator` | AsyncAction | Sends elevator call command (Gazebo CLI service or physical) |
| `SwitchMap` | AsyncAction | Calls `/map_server/load_map` with the given YAML path |
| `PublishInitialPose` | AsyncAction | Publishes `/initialpose`; optionally corrects pose using AprilTag TF lookup |
| `PublishCurrentFloor` | AsyncAction | Publishes the current floor ID to the named_goal_server tracker topic |
| `PressFloorButton` | AsyncAction | Action client wrapping the floor button press action server; triggers MoveIt IBVS arm to press target floor button; blocks until `/press_complete` received and button confirmed lit |
| `CheckFloorArrival` | AsyncAction | Action client wrapping `/check_floor_arrival`; blocks until arrived |
| `CheckElevatorDirection` | AsyncAction | Action client wrapping `check_elevator_direction`; blocks until direction confirmed |
| `DetectCallButton` | AsyncAction | Action client wrapping `detect_call_button`; YOLO + ZED2 RANSAC localises call button; blocks until arm presses button and BUTTON_LIT or DOOR_OPEN confirmed; publishes waypoints to `/button_press_goal` |
| `SetAMCLParams` | AsyncAction | Updates `update_min_d` + `update_min_a` on the AMCL node via parameter service |
| `SetGoalCheckerParams` | AsyncAction | Updates `xy_goal_tolerance` + `yaw_goal_tolerance` on the controller server via parameter service |
| `SetControllerParams` | AsyncAction | Updates `max_vel_x` on the controller server via parameter service |
| `SetCostmapInflation` | AsyncAction | Updates `inflation_radius` on both local + global costmaps via parameter service |
| `ClearEntireCostmap` | AsyncAction | Calls Nav2 costmap clear service (`local_costmap` or `global_costmap`) |
| `StopRobot` | AsyncAction | Publishes zero `Twist` repeatedly for `duration_ms` at `repeat_ms` interval |
| `ToggleAprilTag` | AsyncAction | Calls `/toggle_apriltag` to start (true) or stop (false) `apriltag_ros` subprocess |
| `PublishBoolTopic` | SyncAction | Publishes `std_msgs/Bool` to a named topic with configurable `value` (default `true`). Used for `/going_in` and `/going_out` signals. Lazily creates publisher on first tick per unique topic. |
| `NavigateToPose` | Nav2 BtActionNode | Sends single-goal nav action to bt_navigator |
| `NavigateThroughPoses` | Nav2 BtActionNode | Sends multi-waypoint nav action to bt_navigator |
| `ComputePathToPose` | Nav2 BtActionNode | Requests path plan without executing (used for door detection) |
| `Spin` | Nav2 BtActionNode | In-place rotation via behavior_server |
| `Wait` | BtActionNode | Sleeps for `wait_duration` seconds |
| `Delay` | BtDecorator | Waits `delay_msec` before ticking child |
| `Inverter` | BtDecorator | Flips SUCCESS ↔ FAILURE |
| `RetryUntilSuccessful` | BtDecorator | Re-ticks child up to `num_attempts` times until SUCCESS |
| `Fallback` | Control | Returns SUCCESS on first succeeding child |
| `Sequence` | Control | Returns FAILURE on first failing child |

---

## 6. Key Algorithms

### 6.1 Planning-Based Door Detection

**Concept:** The elevator door physically blocks the path. When the door opens, the LiDAR sees through it and the global costmap obstacle layer clears those cells. NavfnPlanner then finds a feasible path where it previously could not.

**Door open — entry (`WaitForPath_Entry`):**
```xml
<RetryUntilSuccessful num_attempts="150" name="WaitForPath_Entry">
  <Sequence>
    <!-- Global costmap only: NavFn reads global costmap; local is for MPPI -->
    <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
    <!-- 2000 ms = full global costmap update cycle (update_frequency=0.5 Hz) -->
    <Delay delay_msec="2000">
      <ComputePathToPose goal="{center_pose}" path="{dummy_path}" planner_id="GridBased"/>
    </Delay>
  </Sequence>
</RetryUntilSuccessful>
```
- Clears global costmap, waits 2000 ms for one full obstacle-layer update cycle, then checks if NavFn can find a path to `elevator_center`
- SUCCESS → door open → break loop
- FAILURE → door still closed → retry with fresh clear
- Max timeout: 150 × ~2 s = **~5 minutes**
- Uses `elevator_center` (separate YAML key from `elevator_inside`) so the door-poll goal can be tuned independently from the robot's parking position

**Why per-iteration global clear is required:**
After `ClearEntireCostmap`, the global costmap obstacle layer is empty. If the door is still closed, fresh LiDAR scans re-mark the door cells on the next `update_frequency` cycle (up to 2000 ms). Without the clear, old obstacle cells from the closed door persist indefinitely — the planner would never find a path even after the door opens. The clear-then-wait-then-check pattern ensures: (a) stale cells never block detection, and (b) the planner only sees a genuinely re-populated map.

**Why `global_costmap` only (not local):**
`ComputePathToPose` with `planner_id="GridBased"` (NavfnPlanner) plans on the **global costmap**. The local costmap feeds the MPPI controller during active navigation. Clearing the local costmap per iteration wastes ~50 ms and provides no benefit for path planning.

**Why 2000 ms delay:**
`global_costmap.update_frequency = 0.5 Hz` → period = 2000 ms. After the clear, the obstacle layer processes new LiDAR observations only on the next update tick. With a shorter delay (e.g., 500 ms), the global grid may still be empty when NavFn runs — a closed door appears navigable, causing a false positive that sends the robot into the door.

**Door open — exit (`WaitForPath_Exit1` / `WaitForPath_Exit2`):**
```xml
<RetryUntilSuccessful num_attempts="3000" name="WaitForPath_Exit1">
  <Delay delay_msec="100">
    <ComputePathToPose goal="{exit_pose}" path="{dummy_path}" planner_id="GridBased"/>
  </Delay>
</RetryUntilSuccessful>
```
- Poll every 100 ms (10 Hz); no per-iteration clear
- Pre-loop `ClearEntireCostmap` (both local + global) happens once before the retry loop
- Max timeout: 3000 × 100 ms = 5 minutes
- 100 ms is safe at exit because the costmaps were freshly cleared and populated during `CheckFloorArrival` — no stale occupancy risk

**Door close (commented out — not active):**
```xml
<!-- <RetryUntilSuccessful num_attempts="3000" name="WaitForDoorClose">
  <Delay delay_msec="500">
    <Inverter>
      <ComputePathToPose goal="{staging_pose}" path="{dummy_path}" planner_id="GridBased"/>
    </Inverter>
  </Delay>
</RetryUntilSuccessful> -->
```
Currently bypassed — map switch fires immediately after `PressFloorButton` + 1 s wait.

**Advantages over depth/vision-based door detection:**
- Uses existing LiDAR + costmap pipeline — no additional sensor processing
- Robust to lighting changes, occlusion, and model accuracy
- Directly validates navigability rather than a proxy signal

---

### 6.2 Floor Arrival Detection (YOLO)

**Input:** Live ZED2 camera feed, target floor string

**Algorithm:**
1. Map `target_floor` → `(unlit_class_id, lit_class_id)` using `FLOOR_CLASS_MAP`
2. Run YOLO at 20 Hz with `conf=0.15`
3. For each detected box, check if `cls_id ∈ {lit_class, unlit_class}`
4. State machine:
   - **Lit class detected** → elevator in motion (button illuminated) → reset timer, feedback `ON`
   - **Unlit class detected, no lit** → button extinguished → start stability timer; if elapsed ≥ 0.1 s → `arrived = true`
   - **Neither detected** → `NO_DETECTION` → reset timer

**Why 0.1 s stability window:** Prevents false positives from single-frame YOLO drop-outs during LED state transitions.

**Debug image** publishes only the target class boxes to reduce visual noise during debugging.

---

### 6.3 Elevator Direction Detection (YOLO + HSV)

**Input:** Live camera feed, `current_floor`, `target_floor`

**Direction inference:**
```python
direction = "UP" if int(target_floor[-1]) > int(current_floor[-1]) else "DOWN"
```

**Detection pipeline per frame:**
1. YOLO detects `"up"` or `"down"` arrow class (conf > 0.1)
2. Crop the bounding box ROI from the BGR frame
3. Convert ROI to HSV colour space
4. Apply mask: `HSV_LOWER=[5,50,150]` to `HSV_UPPER=[35,255,255]` (orange/amber range)
5. `ratio = non_zero_mask_pixels / total_roi_pixels`
6. If `ratio >= 0.04` for ≥ 0.1 s → direction confirmed

**Why HSV on top of YOLO:** YOLO detects the arrow panel region reliably. HSV then specifically measures whether the LED behind the arrow is illuminated, providing a more robust lit/unlit signal than YOLO confidence alone.

---

### 6.4 AMCL Freeze During In-Elevator Spin

**Problem:** While the robot spins 180° inside the elevator, the LiDAR sees the elevator walls from rapidly changing angles. These scans produce incorrect particle weights in AMCL and corrupt the pose estimate.

**Solution:** Raise AMCL's update thresholds to values that will never be triggered during the spin:

```xml
<SetAMCLParams update_min_d="10.0" update_min_a="10.0"/>
<Fallback name="SpinInElevator">
  <Spin spin_dist="-2.61799" time_allowance="40.0" is_recovery="true"/>
  <Sequence name="RepositionAndSpin">
    <GetNamedPose ... location_key="elevator_inside" pose="{inside_pose}"/>
    <NavigateToPose ... goal="{inside_pose}"/>
    <Spin spin_dist="-2.61799" time_allowance="40.0" is_recovery="true"/>
  </Sequence>
</Fallback>
<SetAMCLParams update_min_d="0.15" update_min_a="0.1"/>
```

`update_min_d=10.0 m` and `update_min_a=10.0 rad` mean AMCL will not update its particle filter until the robot moves 10 m or rotates 10 rad — effectively never during a −2.61799 rad (≈ −150°) spin. Normal values are restored immediately after. The `SpinInElevator` fallback retries by navigating back to `elevator_inside` before spinning again if the first spin action fails.

---

### 6.5 AMCL Re-localization with AprilTag Correction

**Problem:** The elevator does not stop at exactly the same position every time. The robot's pose estimate after a floor transition has a parking error that must be corrected before navigation on the new floor begins.

**Solution:** The `PublishInitialPose` BT node implements three-layer correction:

```
1. Nominal pose    — pre-surveyed initial pose for this floor from locations.yaml
2. AprilTag correction (if use_apriltag=true):
     a. Query /tf for tag36h11:0 → base_link transform
     b. parking_error = expected_tag_pose − actual_tag_pose
     c. corrected_pose = nominal_pose + parking_error
3. Publish to /initialpose → AMCL resets particle cloud around corrected_pose
```

**AprilTag parameters in BT XML:**
```
expected_tag_x=1.313, expected_tag_y=0.071, expected_tag_z=1.412
expected_tag_roll=1.581, expected_tag_pitch=0.000, expected_tag_yaw=-1.633
```
These are the known AprilTag pose in the elevator coordinate frame from the robot's perspective when parked correctly.

---

### 6.6 Startup Localization Sequence

Runs once at launch to sweep the robot through a drive-and-rotate pattern that exposes the LiDAR to enough of the environment for AMCL to converge.

```
Wait 2 s
→ Drive forward 1.0 m at 0.15 m/s
→ Pause 1 s
→ Rotate 360° (2π rad) at 0.5 rad/s  [yaw accumulation with normalize_angle()]
→ Pause 1 s
→ Shutdown
```

Odometry is tracked directly from `/diff_drive_controller/odom` — distance via Euclidean position delta, rotation via signed yaw delta with wraparound correction.

---

## 7. Configuration Files

### 7.1 physical_locations.yaml (Hardware) / locations.yaml (Simulation)

Floor-aware location database. Maps each floor to its map variants and named poses.

**Hardware (`physical_locations.yaml`) — floor1 (primary tested floor):**

```yaml
floor1:
  maps:
    closed: physical_maps/first_floor_with_lift.yaml
    open:   physical_maps/first_floor_with_lift.yaml
  locations:
    amcl_initial_pose_open:   {x: -6.262,    y:  0.381,   yaw:  0.606}
    staging_waypoint:         {x: -6.070722, y:  2.321492, yaw: -2.116}
    elevator_staging:         {x: -6.3201,   y:  1.9011,  yaw: -2.116}
    elevator_entry:           {x: -6.2124,   y:  0.9552,  yaw: -2.0786}
    elevator_inside:          {x: -6.4419,   y:  0.32887, yaw: -2.536}
    elevator_center:          {x: -6.5433,   y:  0.3450,  yaw: -2.052}  ← door-poll goal
    elevator_exit:            {x: -5.7048,   y:  1.9070,  yaw:  1.0947}
    hod_office:               {x:  6.6063,   y:  6.6618,  yaw:  2.7912}
    computer_lab:             {x:  6.40,     y: -1.59,    yaw:  1.57}
    conference_room:          {x: -10.2,     y: 22.60,    yaw:  0.0}
    prof_jayasinghe_office:   {x: 10.92,     y:  9.46,    yaw:  3.14}
    prof_kyew_office:         {x: 11.28,     y:  7.39,    yaw:  3.14}
    prof_dileeka_office:      {x: 11.76,     y:  5.02,    yaw:  3.14}
    prof_rohan_office:        {x: 12.29,     y:  2.47,    yaw:  3.14}

floor3:
  maps:
    closed: physical_maps/third_floor_with_lift.yaml
    open:   physical_maps/third_floor_with_lift.yaml
  locations:
    amcl_initial_pose_open:   {x: -11.422, y: -0.024, yaw: -0.611}
    elevator_staging:         {x:  -9.950, y:  0.035, yaw:  2.618}
    elevator_entry:           {x: -10.387, y: -0.128, yaw:  2.759}
    elevator_inside:          {x: -11.422, y: -0.024, yaw: -3.752}
    elevator_center:          {x: -11.422, y: -0.024, yaw: -3.752}  ← placeholder (=inside)
    elevator_exit:            {x:  -9.4523,y: -0.34338,yaw: -0.3623}
    vision_lab:               {x: -12.6221,y:  7.07402,yaw: -1.9480}
    telecom_lab:              {x: -10.4300,y: -1.9140, yaw:  1.2858}

# floor0, floor2: elevator_center = {0.0, 0.0, 0.0} (placeholder — not yet calibrated)
```

**Location key conventions:**
- `amcl_initial_pose_closed/open` — published to `/initialpose` during floor transition
- `staging_waypoint` — intermediate approach waypoint before staging (avoids narrow approach from far)
- `elevator_staging` — final approach pose facing elevator, button-press position
- `elevator_center` — goal pose for `WaitForPath_Entry` door-poll; decoupled from `elevator_inside` so each can be tuned independently
- `elevator_inside` — robot parking position inside elevator after entry; goal for `NavigateThroughPoses` final waypoint
- `elevator_entry` — threshold waypoint at elevator doorway; first waypoint in `NavigateThroughPoses`
- `elevator_exit` — first navigation goal after exiting elevator on target floor

---

### 7.2 smrr_nav_params.yaml — Key Parameters

**AMCL:**

| Parameter | Value | Effect |
|---|---|---|
| `min_particles` | 1000 | Particle filter minimum size |
| `max_particles` | 3000 | Particle filter maximum size |
| `update_min_d` | 0.15 m | Min distance moved before AMCL update |
| `update_min_a` | 0.1 rad | Min angle turned before AMCL update |
| `laser_model_type` | `likelihood_field` | Beam model type |
| `max_beams` | 360 | LiDAR beams used (Sick S2L at 0.1125°/beam) |
| `laser_max_range` | 10.0 m | Maximum range used |

**Controller (MPPI):**

| Parameter | Value | Effect |
|---|---|---|
| `time_steps` | 56 | Prediction horizon |
| `batch_size` | 2000 | Sampled trajectories per iteration |
| `vx_max` | 0.3 m/s | Default max forward velocity (overridden by BT) |
| `wz_max` | 0.3 rad/s | Max angular velocity |
| `vx_std` | 0.2 | Velocity sampling noise |
| `wz_std` | 0.4 | Angular velocity sampling noise |

**Active critics:** `ConstraintCritic`, `ObstaclesCritic`, `GoalCritic`, `GoalAngleCritic`, `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic`

**Planner (NavfnPlanner / Dijkstra):**
- Uses global_costmap with static_layer + obstacle_layer + inflation_layer
- `update_frequency: 0.5 Hz` — determines minimum delay between costmap clear and valid door-open poll (must be ≥ 2000 ms)
- NavFn `tolerance: 0.15 m` — maximum distance from goal cell that constitutes a valid plan
- Inflation radius: 0.5 m (normal) → 0.325 m (staging approach) → 0.265 m (elevator entry) → 0.5 m (after exit)

---

## 8. Maps

| Map file | Floor | Variant | Notes |
|---|---|---|---|
| `first_floor_with_docking_station.yaml` | floor0 | closed | Includes docking station; default at startup |
| `floor0_open.yaml` | floor0 | open | Loaded after elevator door opens on floor0 |
| `second_floor.yaml` | floor1 | closed | — |
| `floor1_open.yaml` | floor1 | open | Loaded when arriving at floor1 |
| `third_floor.yaml` | floor2 | open/closed | Single variant used for both keys |
| `fourth_floor.yaml` | floor3 | open/closed | Single variant used for both keys |

**Map selection in BT:**
```xml
<GetNamedMap locations_file="{locations_file}"
             floor_id="{target_floor_id}"
             map_key="open"
             map_yaml="{map_open_target}"/>
<SwitchMap map_yaml="{map_open_target}"/>
```

The BT always loads the `"open"` map variant of the target floor after the elevator door closes. The `"open"` map has the elevator doorway clear (no wall), which is the state the robot will observe on the new floor.

---

## 9. Launch Files

### smrr_world_navigation.launch.py (Simulation)

Starts the full simulation stack:

1. `smrr_description/gazebo_classic_controllers.launch.py` — Gazebo, robot URDF, diff_drive + arm controllers
2. `nav2_bringup/bringup_launch.py` — AMCL, planners, controllers, costmaps
   - Initial map: `floor0_open.yaml`
   - Params: `smrr_nav_params.yaml`
3. `smrr_bt_mission_executor` — BT service server
4. `named_goal_server` — location resolver
5. `location_subscriber` — topic bridge
6. `floor_arrival_server` — YOLO floor arrival action server
7. `elevator_direction_server` — YOLO+HSV direction action server
8. `elevator_call_button_server` — YOLO + ZED2 RANSAC call button detection and localisation action server
9. `apriltag_manager_server` — dynamic AprilTag process management
10. `startup_localizer` (conditional on `enable_startup_localizer` arg)
11. `rviz2`

**Key launch arguments:**

| Argument | Default | Description |
|---|---|---|
| `enable_startup_localizer` | `false` | Run AMCL convergence motion on startup |
| `initial_floor_id` | `floor0` | Floor robot starts on |

---

### smrr_hardware_navigation.launch.py (Hardware)

Mirrors the simulation launch but omits Gazebo:

- Default map: `physical_maps/first_floor_with_lift.yaml`
- Default params: `smrr_nav_params_hardware.yaml` (hardware-tuned)
- Default locations file: `physical_locations.yaml`
- Default `initial_floor_id`: `floor1` (physical robot setup)
- `use_rviz`: `false` by default (headless deployment)
- `enable_startup_localizer`: `true` by default

Nodes launched (hardware):

1. `nav2_bringup/bringup_launch.py`
2. `smrr_bt_mission_executor`
3. `named_goal_server`
4. `location_subscriber`
5. `floor_arrival_server`
6. `elevator_call_button_server` — YOLO + ZED2 RANSAC call button action server
7. `apriltag_manager_server`
8. `startup_localizer` (conditional)
9. `rviz2` (conditional on `use_rviz`)

> **Note:** `elevator_direction_server` is **not** launched in the hardware configuration.

---

## 10. End-to-End Execution Walkthroughs

### 10.1 Same-Floor Navigation

**Goal:** Navigate to `"exec_office"` while already on floor0.

```
1. /location topic receives "exec_office"
2. location_subscriber → /go_to_pose service call
3. named_goal_server:
     - resolve "exec_office" → floor0, (8.0, 3.0, 0.0)
     - current_floor = floor0
     - generate mission UUID
     - call /start_mission
4. smrr_bt_mission_executor:
     - blackboard: current=floor0, target=floor0, pose=(8.0, 3.0, 0.0)
     - load smrr_multifloor.xml, tick at 20 Hz
5. BT Fallback → try SameFloor branch:
     - IsSameFloor: floor0 == floor0 → SUCCESS
     - NavigateToPose (8.0, 3.0, 0.0) → Nav2 MPPI drives robot
6. BT returns SUCCESS
7. named_goal_server updates current_floor = floor0
8. /go_to_pose response: accepted=true, message="Navigation to exec_office completed"
```

---

### 10.2 Cross-Floor Navigation (floor0 → floor1)

**Goal:** Navigate to `"office_101"` on floor1 while robot is on floor0.

```
 1. /location receives "office_101"
 2. location_subscriber → /go_to_pose
 3. named_goal_server:
      - resolve → floor1, (5.90, -2.29, 0.0)
      - current_floor = floor0  ≠ floor1
      - call /start_mission: current=floor0, target=floor1
 4. smrr_bt_mission_executor ticks BT
 5. IsSameFloor → FAILURE; IsDifferentFloor → SUCCESS
 6. Pre-entry setup (runs once, outside the retry loop):
      SetGoalCheckerParams(xy=0.20, yaw=0.20)
      SetCostmapInflation(0.325)
      Navigate via staging_waypoint → elevator_staging
      SetGoalCheckerParams(xy=0.15, yaw=0.20)
      GetNamedPose(elevator_inside, floor0) → blackboard for DetectCallButton
      SetCostmapInflation(0.265)            — tight: inside_pose near walls
      SetControllerParams(vx=0.75)          — speed preset before door poll

 7. DetectCallButton(floor0 → floor1):
       - Determines direction = UP (floor1 > floor0)
       - YOLO identifies unlit UP button (class 10) bounding box in camera frame
       - ZED2 registered point cloud: RANSAC fits button surface plane
       - Ray-plane intersection computes button center from bbox center pixel
       - Computes approach_lf (6 cm from plane, in link_0_fake) + press_lf (6 cm into surface)
       - Publishes PoseArray to /button_press_goal for arm controller
       - Arm controller presses button; action verifies via BUTTON_LIT or DOOR_OPEN
       - BT DetectCallButton action returns SUCCESS

 8. GetNamedPose(elevator_center, floor0) → center_pose onto blackboard

 9. [RetryUntilSuccessful x2] CallElevatorAndEnter_Twice:
      a. (CallElevator commented out)
      b. WaitForPath_Entry loop (150 × ~2 s = ~5 min max):
           Clear global costmap
           Wait 2000 ms (one full global costmap update cycle)
           ComputePathToPose(center_pose) → FAIL (door closed) × N
           door opens → LiDAR rays through opening → global costmap clears interior
           ComputePathToPose → SUCCESS → break loop
      c. ClearEntireCostmap (local + global) — flush before entry navigation
      d. BuildPoseVector(entry + inside)
      e. NavigateThroughPoses → robot enters elevator

10. In-elevator repositioning:
      StopRobot (800 ms)
      Wait (1.0 s)
      ToggleAprilTag(true)                   — start AprilTag detection
      SetAMCLParams(d=10.0, a=10.0)          — freeze AMCL during spin
      SpinInElevator Fallback:
        Spin(-2.61799 rad, 40 s)             — ≈150° to face exit
        OR: NavigateToPose(inside) + Spin    — reposition fallback
      PublishBoolTopic(/going_in = true)
      StopRobot (800 ms)
      SetAMCLParams(d=0.15, a=0.1)           — restore AMCL
      PressFloorButton(floor1)               — MoveIt IBVS arm presses floor1 button
      Wait(1.0 s)

11. Door close detection: **skipped** (WaitForDoorClose commented out)

12. Map & localization switch:
      GetNamedMap(floor1, "open")        → floor1_open.yaml
      GetNamedPose(amcl_initial_pose_open, floor1) → amcl_open_target
      GetNamedPose(amcl_initial_pose_open, floor0) → amcl_open_current
      SwitchMap(floor1_open.yaml)        — load new occupancy grid
      PublishCurrentFloor                — floor tracker updated immediately
      PublishInitialPose with AprilTag:
        - query /tf for tag36h11:0
        - parking_error = expected_tag - actual_tag
        - publish corrected pose to /initialpose → AMCL resets
      ClearEntireCostmap (local + global)
      ToggleAprilTag(false)

13. PublishBoolTopic(/going_out = true)

14. Elevator exit (Attempt 1):
      GetNamedPose(elevator_exit, floor1)
      (CallElevator — commented out)
      CheckFloorArrival(floor1):         — YOLO watches button panel
        lit class detected → ON (elevator moving)
        ... elevator arrives at floor1 ...
        unlit class detected for 0.1 s → arrived=true → SUCCESS
      SetControllerParams(vx=0.75)
      ClearEntireCostmap (local + global)
      WaitForPath_Exit1 loop (3000 × 100ms = 5 min max):
        ComputePathToPose(exit_pose) → SUCCESS → break
      SetGoalCheckerParams(xy=0.25, yaw=0.25)
      NavigateToPose(exit_pose)          — robot exits elevator
      SetControllerParams(vx=0.25)
      SetCostmapInflation(0.5)
      PublishCurrentFloor

15. Final destination:
      NavigateToPose(5.90, -2.29, 0.0)  — drive to office_101

16. BT returns SUCCESS
17. named_goal_server updates current_floor = floor1
17. /go_to_pose response: accepted=true, "Navigation to office_101 completed"
```

---

## 11. Arm Button Pressing System

The elevator button pressing pipeline is shared by both the outside call button
(`DetectCallButton` / `elevator_call_button_server`) and the inside floor button
(`PressFloorButton` / `elevator_floor_button_server`).  All physical motion is
handled by three nodes in the `arm_link` stack (located in `moveit_integration/`):

| Node | Package | Role |
|---|---|---|
| `commander` | `arm_link_commander` | MoveIt2 trajectory executor; receives pose goals and IBVS Cartesian steps; owns `/press_complete` |
| `button_tracker` | `arm_link_visual_servo` | YOLO OBB inference on endoscopic camera; publishes button pixel coordinates |
| `visual_alignment_controller` | `arm_link_visual_servo` | IBVS state machine; converts pixel errors to world-frame EEF deltas |

### 11.1 Topic Graph

```
elevator_call_button_server ──┐
elevator_floor_button_server ──┤──► /button_press_goal (PoseArray)
                               │    /target_button     (String, transient_local)
                               │
                               ▼
                         [ commander ]
                               │
              ┌────────────────┼────────────────┐
              │                │                │
              ▼                ▼                ▼
    /visual_servo/start  /axis_adjust      /press_complete
    (String)             (Vector3)         (Bool)
              │                ▲
              ▼                │
  [ visual_alignment_controller ]
              │                │
              ▼                │
    /button_tracker/target  /visual_servo/axis_done
    (String, TL QoS)        (Bool)
              │
              ▼
      [ button_tracker ]
              │
              ▼
    /marker_tracker/state
    (Float64MultiArray [u,v,area,valid])
              │
              └──────────────► visual_alignment_controller
```

### 11.2 Commander Node (`commander_template.cpp`)

**MoveIt2 group:** `"arm"`, end-effector link `"end_effector"`, planning frame `"world"`.
Velocity/acceleration scaling: **50%** (0.45 rad/s effective per joint at `joint_limits.yaml` max 1.5 rad/s).

**Joint state workaround:** MoveGroupInterface's internal `CurrentStateMonitor` subscribes to the relative topic `joint_states`, which resolves to `/joint_states` outside the `arm` namespace — NOT `/arm/joint_states`.  The commander therefore maintains its own direct subscription to `/arm/joint_states` in a `Reentrant` callback group (`cached_joint_state_`) and uses it everywhere FK or settle verification is needed.

**`/button_press_goal` callback** (`buttonPressGoalCallback`, dispatched to a background `std::thread` to allow preemption):

```
Phase 0 — Safe pre-press pose
    joint target: [24°, 0°, 0°, 95°, -29°]   (joints 1–5)
    OMPL plan + execute → settleToJointTarget()

Phase 1 — Approach to poses[0] (approach pose from smrr_navigation server)
    Tier 0: computeCartesianPath from Phase 0 end state (requires ≥95% coverage)
    Tier 1: joint-space OMPL, orientation tolerance = ~3°
    Tier 2: joint-space OMPL, orientation tolerance = ±20°
    Tier 3: joint-space OMPL, orientation tolerance = ±45°
    Tier 4: joint-space OMPL, orientation tolerance = ±90°
    → execute whichever tier succeeds first → settleToJointTarget()

Orientation correction (after Phase 1)
    If approach used a relaxed orientation tier, snap orientation to goal via
    computeCartesianPath (in-place rotation at current FK position, 0.3× speed).
    Angular error < 1° → skip.

Start IBVS
    Read target button name from /target_button (poll up to 2 s if not yet received).
    Normalise to lowercase.
    visual_servo_active_ = true
    Publish to /visual_servo/start → visual_alignment_controller takes over
    (commander thread exits; IBVS runs independently)
```

**`/visual_servo/complete` callback** (`visualServoCompleteCallback`):
```
Retract to pre-press pose: [24°, 0°, 0°, 95°, -29°]  (same as Phase 0)
    OMPL plan (5 attempts, 5 s) + execute → settleToJointTarget()
Publish /press_complete = true     (regardless of IBVS success/failure)
```
The smrr_navigation action servers (`elevator_call_button_server`,
`elevator_floor_button_server`) block waiting for `/press_complete` before
returning a result to the BT.

**`/axis_adjust` callback** (`axisAdjustCallback`):
```
Input: Vector3(dx, dy, dz) in world frame
    Build RobotState from cached_joint_state_ (not CSM)
    FK → current EEF position in world frame
    Target = current + delta
    computeCartesianPath(target waypoint, step=5mm, jump_threshold=0.0)
    Time-parameterise at 50% speed
    execute()
    settleToJointTarget(trajectory.points.back())
    Publish /visual_servo/axis_done = true   → gates next IBVS command
```

**`settleToJointTarget()`** — post-execution closed-loop verification:
- Waits 500 ms for mechanical settle
- Reads actual joint positions from `cached_joint_state_`
- If max joint error > 0.06 rad (~3.4°), issues one corrective re-plan+execute
- Accommodates mechanical backlash on joints 1 and 4

**Preemption:** If a `/joint_command` message arrives while a button press is in progress, `preempt_requested_` is set, the background thread detects it at the next check-point (between motion phases and between settle loops), cancels the press, executes the preempting joint command, and clears the flag.  `visual_servo_abort_pub_` sends an abort signal to the IBVS controller in the same path.

---

### 11.3 Button Tracker Node (`button_tracker.py`)

**Model:** YOLO OBB (`finger_camera_detection.pt`), loaded at startup with a
dummy-inference warmup to pre-compile JIT/CUDA kernels.

**OBB class map:**

| Class ID | Name | Used for |
|---|---|---|
| 0 | `Button-Detection` | Generic fallback (not used — no fallback selection) |
| 1 | `button_1` | Floor 1 |
| 2 | `button_2` | Floor 2 |
| 3 | `button_3` | Floor 3 |
| 4 | `button_down` | Down call button |
| 5 | `button_up` | Up call button |

**Selection logic:** Only the highest-confidence box whose class name **contains** the
target string (`"up"`, `"down"`, `"one"`, `"two"`, `"three"`) is selected.  No
fallback to the generic `Button-Detection` class.  If no matching box is found,
`valid_flag = 0.0` is published.

**Active-only inference:** YOLO only runs when `_target_button` is non-empty
(set by `/button_tracker/target` from `visual_alignment_controller`).
On `/visual_servo/complete`, the target is cleared and `_latest_img` is discarded.
This prevents stale detections leaking into the next press.

**Camera QoS:** RELIABLE + VOLATILE (depth 5) — matches the ZED2 driver publisher.
Using `qos_profile_sensor_data` (Best Effort) caused intermittent silent drops.

**Output:** `/marker_tracker/state` = `[u, v, area, valid_flag]`
- `u`, `v` — OBB centre pixel coordinates (from `xywhr[0]`)
- `area` — `width × height` of the OBB (px²)
- `valid_flag` — 1.0 if detection present, 0.0 otherwise

**Publish rate:** 15 Hz timer. Each frame processed at most once (`_latest_img`
consumed and cleared per tick). Re-entry guard (`_busy`) prevents concurrent YOLO calls.

**Camera stall detection:** If `_latest_img` is `None` for > 0.5 s while a target
is active, a `[CAMERA STALL]` error is logged once; recovery is logged when frames resume.

---

### 11.4 Visual Alignment Controller (`visual_alignment_controller.py`)

**Image geometry:** 640 × 480 px, centre = (320, 240).

**State machine:**

```
IDLE ──/visual_servo/start──► ALIGN ──settled──► FORWARD ──pressed──► FINAL_PRESS
                                 ▲                    │                      │
                                 └─────(re-align)─────┘                      │
                                                                        RETRACT ──► IDLE
```

| State | Description |
|---|---|
| `IDLE` | No active servoing. Waits for `/visual_servo/start` |
| `ALIGN` | Corrects u + v errors simultaneously in world frame via live TF |
| `FORWARD` | Moves 30 mm toward button (camera +Z) after alignment convergence |
| `FINAL_PRESS` | Two-phase final press (40 mm + 40 mm = 80 mm) after stop area reached |
| `RETRACT` | Publishes −8 cm in base-link X before signalling complete |

**Control law (ALIGN state):**

$$\text{depth\_scale} = \text{clamp}\!\left(\sqrt{\frac{13500}{\text{area}}},\ 0.1,\ 1.5\right)$$

$$\text{cam}_{dx} = K_U \cdot u_\text{err} \cdot \text{depth\_scale}, \quad K_U = +3\times10^{-4}\ \text{m/px}$$
$$\text{cam}_{dy} = K_V \cdot v_\text{err} \cdot \text{depth\_scale}, \quad K_V = +3\times10^{-4}\ \text{m/px}$$

$$\text{world\_step} = R_{\text{world}\leftarrow\text{cam}} \cdot [\text{cam}_{dx},\ \text{cam}_{dy},\ 0]^T$$

Where $R_{\text{world}\leftarrow\text{cam}}$ is looked up live from `/tf` (`link_0` → `finger_camera_optical`) every control cycle, so the gain mapping is always correct regardless of current joint configuration.

**Step-size limits:**
- `MIN_STEP_M = 4 mm` (deadband floor — arm doesn't physically move below ~3 mm)
- `MAX_STEP_M = 20 mm` (vector magnitude cap, direction preserved)
- Clamping is **vector magnitude** (not per-axis) to preserve direction

**ALIGN convergence to FORWARD:**
- Settle box: `half_side = min(√(0.75 × area) / 2, 80 px)` — depth-adaptive, maps to a constant physical offset regardless of distance
- Must be settled for **2 consecutive ticks** before entering FORWARD

**FORWARD state:** Issues one `/axis_adjust` step of `FORWARD_STEP_M = 30 mm` in camera +Z (rotated to world), then waits for `/visual_servo/axis_done`. If the marker re-enters the settle box after the step, transitions back to ALIGN for refinement.

**Stop condition (enter FINAL_PRESS):**
- `area ≥ MARKER_AREA_STOP_FRACTION × (640×480) = 0.25 × 307200 = 76800 px²` AND
- Image centre within 80 px of marker centre  
OR  
- `area > 0.35 × 307200` (large-area shortcut) AND computed step < 10 mm AND centred

**FINAL_PRESS — two-phase:**
```
Phase 0 (after 3.0 s settle): forward nudge = FINAL_PRESS_M / 2 = 40 mm
Phase 1 (after 3.0 s settle): forward nudge = FINAL_PRESS_M / 2 = 40 mm
→ _publish_complete(success=True)
```

**RETRACT:** Issues one `/axis_adjust` of `−80 mm` in base-link X (retract from button), then transitions to IDLE and publishes `/visual_servo/complete = True`.

**Stall detection (ALIGN):** If `u_error` does not decrease by ≥ 5 px over 5 consecutive cycles:
- Case A (centred): 10 mm forward nudge to break stall
- Case B (near stop area ≥ 80%): 10 mm forward nudge to shift kinematic config
- Case C (lateral, small step): boost step to 15 mm
- Case C escalation (3 consecutive C fires): override with 10 mm forward nudge

**Hard time cap:** `MAX_ALIGN_TIME_S = 30 s` per ALIGN session. Prevents infinite loops from TF failure, permanent joint limits, or stall-escape loops.

**Axis-done gating:** After publishing `/axis_adjust`, the controller blocks any new correction command until:
1. `/visual_servo/axis_done` is received (commander has settled), AND
2. A fresh YOLO detection newer than the `axis_done` timestamp arrives (prevents stale frames before the arm moved from contaminating the next control step)

---

### 11.5 Full Button Press Sequence — Outside Call Button

Triggered from `elevator_call_button_server.py` via `/button_press_goal`:

```
1. ZED2 RANSAC → approach pose (poses[0], 6 cm from button surface)
                  press  pose  (poses[1], 6 cm into surface)
   Publish EEF_ORIENTATION = (-0.039, 0.691, 0.656, -0.301) on poses
   Publish target class to /target_button: "up" (class 10/11) or "down" (class 0/1)

2. Commander: Phase 0 → pre-press safe pose [24°, 0°, 0°, 95°, -29°]
3. Commander: Phase 1 → approach pose (Cartesian Tier 0 or joint-space Tiers 1-4)
4. Commander: orientation correction (snap to EEF_ORIENTATION)
5. Commander: publish target button to /visual_servo/start → IBVS starts

6. visual_alignment_controller: ALIGN (KU/KV proportional + TF rotation)
7. visual_alignment_controller: FORWARD (30 mm steps toward button)
8. visual_alignment_controller: FINAL_PRESS (2 × 40 mm)
9. visual_alignment_controller: RETRACT (−80 mm base-link X)
   → publishes /visual_servo/complete = True

10. Commander: retracts to [24°, 0°, 0°, 95°, -29°]
    → publishes /press_complete = True

11. elevator_call_button_server: checks for BUTTON_LIT or DOOR_OPEN via YOLO
    → if confirmed: returns SUCCESS to DetectCallButton BT action
    → if not confirmed: re-triggers press loop (up to ARM_TIMEOUT_SEC=120 s)
```

### 11.6 Full Button Press Sequence — Inside Floor Button

Triggered from `elevator_floor_button_server.py` via `/button_press_goal`:

```
1. YOLO on ZED2 image → locates floor button class:
   floor0=2/3, floor1=4/5, floor3=6/7, floor2=8/9  (unlit/lit)
   RANSAC → approach + press poses
   Publish target class to /target_button: "one", "two", "three" etc.

2–10. Same Commander + IBVS pipeline as §11.5

11. elevator_floor_button_server: checks for BUTTON_LIT via YOLO on finger camera
    → if confirmed: publish arm-down signal, return SUCCESS to PressFloorButton BT action
    → on timeout: plain continue (no arm-down) → retry
```

**Key difference from call button:** Floor button server uses the endoscopic
camera class map for floor numbers, not direction arrows. Lit class IDs confirm
the correct floor was pressed before the BT action returns SUCCESS.

---

### 11.7 Key Constants Summary

| Constant | Value | Description |
|---|---|---|
| `KU`, `KV` | `+3e-4 m/px` | ALIGN proportional gains (camera frame) |
| `REFERENCE_AREA_PX` | 13500 px² | Area at which KU/KV were calibrated |
| `MIN_DEPTH_SCALE` | 0.1 | Minimum depth gain scale factor |
| `MAX_DEPTH_SCALE` | 1.5 | Maximum depth gain scale factor (150% of nominal) |
| `MIN_STEP_M` | 4 mm | Deadband floor |
| `MAX_STEP_M` | 20 mm | Vector magnitude cap per ALIGN command |
| `FORWARD_STEP_M` | 30 mm | Step size per FORWARD state press |
| `FINAL_PRESS_M` | 80 mm | Total final press distance (2 × 40 mm) |
| `PRESS_SETTLE_S` | 3.0 s | Settle time before each forward or final press |
| `MARKER_AREA_STOP_FRACTION` | 0.25 | Stop pressing when area ≥ 25% of image area |
| `MAX_SETTLE_BOX_PX` | 80 px | Pixel cap for stop/centred condition |
| `FORWARD_SETTLE_FRACTION` | 0.75 | Marker area fraction for ALIGN→FORWARD settle box |
| `STALL_CYCLES` | 5 | Consecutive cycles without ≥5 px improvement = stall |
| `STALL_BOOST_STEP_M` | 15 mm | Stall escape boost step magnitude |
| `MAX_ALIGN_TIME_S` | 30 s | Hard timeout for one ALIGN session |
| `EEF_ORIENTATION` | `(-0.039, 0.691, 0.656, -0.301)` | End-effector quaternion (x,y,z,w) for button press |
| Pre-press joints | `[24°, 0°, 0°, 95°, -29°]` | Safe configuration before/after press |
| Velocity scaling | 50% | MoveIt trajectory velocity and acceleration scaling |
| Settle threshold | 0.06 rad (~3.4°) | Joint error threshold for post-execution settle check |
