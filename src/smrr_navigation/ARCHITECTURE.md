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
    (Nav2 → MPPI)                   → Navigate to elevator_staging
                                    → DetectCallButton (YOLO + ZED2 RANSAC → /button_press_goal)
                                    → CallElevator
                                    → Wait for door open (ComputePathToPose polling)
                                    → NavigateThroughPoses (entry → inside)
                                    → 180° Spin (AMCL frozen)
                                    → Wait for door close (inverse polling)
                                    → SwitchMap + PublishInitialPose (AprilTag corrected)
                                    → CheckFloorArrival (YOLO button detection)
                                    → Wait for door open on target floor
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
   └─ Sequence [CrossFloor]
      ├─ IsDifferentFloor
      ├─ GetNamedPose (elevator_staging, current_floor)
      ├─ NavigateToPose (→ elevator_staging)
      ├─ DetectCallButton (current_floor → target_floor)          ← YOLO + ZED2 RANSAC → /button_press_goal
      ├─ GetNamedPose (elevator_inside, current_floor)            ← pre-fetched for ComputePathToPose
      │
      ├─ RetryUntilSuccessful(2) [CallElevatorAndEnter]
      │  └─ Sequence
      │     ├─ CallElevator (current_floor)
      │     ├─ SetCostmapInflation (0.325)
      │     ├─ SetControllerParams (max_vel_x=0.75)
      │     ├─ RetryUntilSuccessful(3000) [WaitForPath_Entry]
      │     │  └─ Delay(500ms) → ComputePathToPose(inside_pose)
      │     ├─ GetNamedPose (elevator_entry, current_floor)
      │     ├─ GetNamedPose (elevator_inside, current_floor)     ← re-fetched inside retry
      │     ├─ BuildPoseVector (entry_pose + inside_pose)
      │     └─ NavigateThroughPoses
      │
      ├─ StopRobot (800 ms)
      ├─ Wait (1.0 s)
      ├─ ToggleAprilTag (true)
      ├─ SetAMCLParams (update_min_d=10.0, update_min_a=10.0)  ← freeze
      ├─ Spin (-π rad, 20 s)
      ├─ StopRobot (800 ms)
      ├─ SetAMCLParams (update_min_d=0.15, update_min_a=0.1)   ← restore
      ├─ Wait (1.0 s)
      │
      ├─ RetryUntilSuccessful(3000) [WaitForDoorClose]
      │  └─ Delay(500ms) → Inverter → ComputePathToPose(staging_pose)
      ├─ Wait (3.0 s)
      │
      ├─ GetNamedMap (target_floor, "open")
      ├─ GetNamedPose (amcl_initial_pose_open, target_floor)
      ├─ SwitchMap (map_open_target)
      ├─ PublishInitialPose (use_apriltag=true, tag_frame=tag36h11:0)
      ├─ ClearEntireCostmap (local)
      ├─ ClearEntireCostmap (global)
      ├─ ToggleAprilTag (false)
      │
      ├─ Fallback [ExitElevator]
      │  ├─ Sequence [ExitAttempt_1]
      │  │  ├─ GetNamedPose (elevator_exit, target_floor)
      │  │  ├─ CallElevator (target_floor)
      │  │  ├─ CheckFloorArrival (target_floor)
      │  │  ├─ SetControllerParams (max_vel_x=0.75)
      │  │  ├─ ClearEntireCostmap (local + global)
      │  │  ├─ RetryUntilSuccessful(3000) [WaitForPath_Exit]
      │  │  │  └─ Delay(500ms) → ComputePathToPose(exit_pose)
      │  │  ├─ NavigateToPose (→ exit_pose)
      │  │  ├─ SetControllerParams (max_vel_x=0.5)
      │  │  └─ SetCostmapInflation (0.5)
      │  │
      │  └─ Sequence [RepositionAndExit_Attempt_2]
      │     ├─ GetNamedPose (elevator_inside, target_floor)
      │     ├─ NavigateToPose (→ inside_pose)
      │     ├─ Spin (-3.5416 rad, 10 s)
      │     ├─ StopRobot
      │     └─ [Repeat exit sequence: CallElevator → CheckFloorArrival →
      │          WaitForPath → NavigateToPose → RestoreParams]
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

Before the entry retry loop, the BT runs two steps that are outside the retry so they execute only once regardless of how many retries occur:

```xml
<!-- Detect UP/DOWN call button and publish arm waypoints -->
<DetectCallButton current_floor="{current_floor_id}" target_floor="{target_floor_id}"/>

<!-- Pre-fetch inside_pose so it is on the blackboard for ComputePathToPose polling -->
<GetNamedPose locations_file="{locations_file}" floor_id="{current_floor_id}"
              location_key="elevator_inside" pose="{inside_pose}"/>
```

`DetectCallButton` triggers `elevator_call_button_server` which uses YOLO to locate the correct call button (class 10/11 for UP, 0/1 for DOWN) and then fits a RANSAC plane to the ZED2 point cloud crop to compute 3D approach and press waypoints. These are published to `/button_press_goal` for the arm controller and the BT action returns SUCCESS once localised.

Then the entry retry loop:

```xml
<RetryUntilSuccessful num_attempts="2" name="CallElevatorAndEnter_Twice">
  <Sequence name="CallWaitAndEnter">
    <CallElevator floor_id="{current_floor_id}"/>
    <SetCostmapInflation inflation_radius="0.325"/>
    <SetControllerParams max_vel_x="0.75"/>

    <!-- Planning-based door open detection (see §6.1) -->
    <RetryUntilSuccessful num_attempts="3000" name="WaitForPath_Entry">
      <Delay delay_msec="500">
        <ComputePathToPose goal="{inside_pose}" path="{dummy_path}" planner_id="GridBased"/>
      </Delay>
    </RetryUntilSuccessful>

    <GetNamedPose location_key="elevator_entry" .../>
    <GetNamedPose location_key="elevator_inside" .../>
    <BuildPoseVector pose1="{entry_pose}" pose2="{inside_pose}" poses="{entry_poses}"/>
    <NavigateThroughPoses server_name="/navigate_through_poses" goals="{entry_poses}"/>
  </Sequence>
</RetryUntilSuccessful>
```

**Why inflation is reduced:** The elevator gap is tight; 0.5 m inflation causes path planning to fail through the narrow door. Reduced to 0.325 m to allow a feasible path.

**Why velocity is increased:** 0.75 m/s carries the robot through the door with momentum, reducing the chance of stopping partway through.

**Why `elevator_inside` is fetched twice:** The pre-fetch before the retry loop puts `inside_pose` on the blackboard so the `ComputePathToPose` door-poll can use it immediately. The re-fetch inside the retry loop refreshes it before `NavigateThroughPoses` to ensure correctness on retries.

---

### 5.5 Cross-Floor Branch — In-Elevator Repositioning

```xml
<StopRobot topic="/cmd_vel" repeat_ms="200" duration_ms="800"/>
<Wait wait_duration="1.0"/>
<ToggleAprilTag turn_on="true"/>
<SetAMCLParams update_min_d="10.0" update_min_a="10.0"/>
<Spin spin_dist="-3.1416" time_allowance="20.0" is_recovery="true"/>
<StopRobot topic="/cmd_vel" repeat_ms="200" duration_ms="800"/>
<SetAMCLParams update_min_d="0.15" update_min_a="0.1"/>
<Wait wait_duration="1.0"/>
```

The robot spins 180° (−π rad) so it faces the elevator exit ready to drive out. AMCL is frozen during the spin to prevent particle filter corruption from poor in-elevator LiDAR geometry. The subsequent 1 s wait is mandatory: the robot is now facing the door, and the staging pose is unreachable through the closed door — without the wait, door-close detection could trigger immediately on stale costmap data.

---

### 5.6 Cross-Floor Branch — Door Close & Map Switch

```xml
<!-- Wait until door is physically closed -->
<RetryUntilSuccessful num_attempts="3000" name="WaitForDoorClose">
  <Delay delay_msec="500">
    <Inverter>
      <ComputePathToPose goal="{staging_pose}" path="{dummy_path}" planner_id="GridBased"/>
    </Inverter>
  </Delay>
</RetryUntilSuccessful>
<Wait wait_duration="3.0"/>

<!-- Switch to target floor's open map -->
<GetNamedMap locations_file="{locations_file}" floor_id="{target_floor_id}" map_key="open" map_yaml="{map_open_target}"/>
<SwitchMap map_yaml="{map_open_target}"/>

<!-- Re-localize with AprilTag correction -->
<GetNamedPose locations_file="{locations_file}" floor_id="{target_floor_id}"
              location_key="amcl_initial_pose_open" pose="{amcl_open_target}"/>
<PublishInitialPose initial_pose="{amcl_open_target}"
                    use_apriltag="true"
                    tag_frame="tag36h11:0"
                    expected_tag_x="1.313"  expected_tag_y="0.071"  expected_tag_z="1.412"
                    expected_tag_roll="1.581" expected_tag_pitch="0.000" expected_tag_yaw="-1.633"/>

<ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
<ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
<ToggleAprilTag turn_on="false"/>
```

**Door close detection logic:** After the 180° spin, the staging pose (outside the elevator) is physically behind the closed door. `ComputePathToPose` to that pose returns FAILURE (no path through wall). `Inverter` flips FAILURE → SUCCESS, breaking the retry loop. This is the inverse of the door-open detection used for entry.

**AprilTag re-localization:** The `PublishInitialPose` node queries `/tf` for the detected AprilTag transform and computes a parking-error correction:
```
corrected_pose = nominal_pose + (expected_tag_pose - actual_tag_pose)
```
This corrects for the elevator not stopping at the exact same position each time. Costmaps are then cleared to remove LiDAR contamination accumulated during the map switch.

---

### 5.7 Cross-Floor Branch — Elevator Exit

The exit uses a `Fallback` with two attempts. If the first attempt fails (e.g., robot ends up at wrong angle inside the elevator), attempt 2 navigates back to the inside pose, spins −3.54 rad to reorient, and retries the full exit sequence.

**Attempt 1:**
```
GetNamedPose(elevator_exit, target_floor)
CallElevator(target_floor)           ← press button on target floor
CheckFloorArrival(target_floor)      ← YOLO: wait until button extinguishes
SetControllerParams(max_vel_x=0.75)  ← speed through door
ClearEntireCostmap (local + global)  ← clear LiDAR noise from elevator
WaitForPath_Exit (3000 × 500ms)      ← wait until door open (planning-based)
NavigateToPose(exit_pose)
SetControllerParams(max_vel_x=0.5)   ← restore normal speed
SetCostmapInflation(0.5)             ← restore normal inflation
```

**Attempt 2 (fallback):**
```
NavigateToPose(elevator_inside)      ← reposition inside elevator
Spin(-3.5416 rad, 10s)               ← reorient
[Full exit sequence repeated]
```

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
| `CheckFloorArrival` | AsyncAction | Action client wrapping `/check_floor_arrival`; blocks until arrived |
| `CheckElevatorDirection` | AsyncAction | Action client wrapping `check_elevator_direction`; blocks until direction confirmed |
| `DetectCallButton` | AsyncAction | Action client wrapping `detect_call_button`; YOLO + ZED2 RANSAC localises call button; blocks until waypoints published to `/button_press_goal` |
| `SetAMCLParams` | AsyncAction | Updates `update_min_d` + `update_min_a` on the AMCL node via parameter service |
| `SetControllerParams` | AsyncAction | Updates `max_vel_x` on the controller server via parameter service |
| `SetCostmapInflation` | AsyncAction | Updates `inflation_radius` on both local + global costmaps |
| `ClearEntireCostmap` | AsyncAction | Calls Nav2 costmap clear service (local or global) |
| `StopRobot` | AsyncAction | Publishes zero `Twist` repeatedly for `duration_ms` at `repeat_ms` interval |
| `ToggleAprilTag` | AsyncAction | Calls `/toggle_apriltag` to start (true) or stop (false) `apriltag_ros` subprocess |
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

**Concept:** The elevator door physically blocks the path. When the door opens, the LiDAR sees through and the costmap clears. NavfnPlanner then finds a feasible path where it previously could not.

**Door open (entry / exit):**
```xml
<RetryUntilSuccessful num_attempts="3000">
  <Delay delay_msec="500">
    <ComputePathToPose goal="{inside_pose}" path="{dummy_path}" planner_id="GridBased"/>
  </Delay>
</RetryUntilSuccessful>
```
- Poll every 500 ms (2 Hz)
- Planner SUCCESS → door open → break loop
- Planner FAILURE → door still closed → retry
- Max timeout: 3000 × 500 ms = 25 minutes

**Door close (after entering elevator):**
```xml
<RetryUntilSuccessful num_attempts="3000">
  <Delay delay_msec="500">
    <Inverter>
      <ComputePathToPose goal="{staging_pose}" path="{dummy_path}" planner_id="GridBased"/>
    </Inverter>
  </Delay>
</RetryUntilSuccessful>
```
- `Inverter` flips the logic: SUCCESS when plan FAILS (door closed = no path to outside)

**Advantages over depth/vision-based door detection:**
- Uses the existing LiDAR + costmap pipeline — no additional sensor processing
- Robust to lighting changes, occlusion, and model accuracy
- Directly validates navigability rather than proxy signals

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
<Spin spin_dist="-3.1416" time_allowance="20.0"/>
<SetAMCLParams update_min_d="0.15" update_min_a="0.1"/>
```

`update_min_d=10.0 m` and `update_min_a=10.0 rad` mean AMCL will not update its particle filter until the robot moves 10 m or rotates 10 rad — effectively never during a 180° spin. Normal values are restored immediately after.

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

### 7.1 locations.yaml

Floor-aware location database. Maps each floor to its map variants and named poses.

```yaml
floors:
  floor0:
    maps:
      closed: first_floor_with_docking_station.yaml
      open:   floor0_open.yaml
    locations:
      amcl_initial_pose_closed:   {x: -1.50, y:  0.80, yaw:  1.571}
      amcl_initial_pose_open:     {x: -1.50, y:  0.80, yaw:  1.571}
      elevator_staging:           {x: -2.50, y:  0.80, yaw:  1.571}
      elevator_inside:            {x: -1.96, y:  3.00, yaw: -1.571}
      elevator_entry:             {x: -6.01, y:  1.35, yaw: -1.956}
      elevator_exit:              {x: -1.50, y:  0.80, yaw:  1.571}
      exec_office:                {x:  8.00, y:  3.00, yaw:  0.0}
      board_room:                 {x: -7.00, y:  2.50, yaw:  1.571}
      rooftop_access:             {x:  0.00, y:  8.00, yaw:  0.0}
      storage:                    {x: -3.00, y: -5.00, yaw:  3.14}
  floor1:
    maps:
      closed: second_floor.yaml
      open:   floor1_open.yaml
    locations:
      amcl_initial_pose_open:     {x: -1.425, y: 2.974, yaw: -1.571}
      elevator_inside:            {x: ...,    y: ...,   yaw: ...}
      elevator_exit:              {x: ...,    y: ...,   yaw: ...}
      office_101:                 {x:  5.90,  y: -2.29, yaw:  0.0}
      ...
  floor2: { ... }
  floor3: { ... }
```

**Location key conventions:**
- `amcl_initial_pose_closed/open` — published to `/initialpose` during floor transition
- `elevator_staging` — approach pose before calling elevator
- `elevator_inside` — pose inside elevator after entry
- `elevator_entry` — intermediate waypoint used in `NavigateThroughPoses` to thread through the door
- `elevator_exit` — first goal after exiting elevator on target floor

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
- Inflation radius: 0.5 m (normal) — reduced to 0.325 m during elevator transit via BT

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
 6. GetNamedPose(elevator_staging, floor0) → (-2.50, 0.80, 90°)
 7. NavigateToPose(-2.50, 0.80) — robot drives to elevator staging position

 7.5 DetectCallButton(floor0 → floor1):
       - Determines direction = UP (floor1 > floor0)
       - YOLO identifies unlit UP button (class 10) bounding box in camera frame
       - ZED2 registered point cloud: RANSAC fits button surface plane
       - Ray-plane intersection computes button center from bbox center pixel
       - Computes approach_lf (6 cm from plane, in link_0_fake) + press_lf (6 cm into surface)
       - Publishes PoseArray to /button_press_goal for arm controller
       - Pre-fetches elevator_inside pose onto blackboard
       - BT DetectCallButton action returns SUCCESS

 8. [RetryUntilSuccessful x2] CallElevatorAndEnter:
      a. CallElevator(floor0)             — press call button
      b. SetCostmapInflation(0.325)       — tighten for door gap
      c. SetControllerParams(vx=0.75)     — increase speed for gap crossing
      d. WaitForPath_Entry loop:          — poll every 500 ms
           ComputePathToPose(inside_pose) → FAIL (door closed) × N
           door opens → LiDAR clears → ComputePathToPose → SUCCESS → break
      e. BuildPoseVector(entry + inside)
      f. NavigateThroughPoses → robot enters elevator

 9. In-elevator repositioning:
      StopRobot (800 ms)
      Wait (1.0 s)
      ToggleAprilTag(true)               — start AprilTag detection
      SetAMCLParams(d=10.0, a=10.0)      — freeze AMCL
      Spin(-π rad)                       — 180° turn, faces exit
      StopRobot (800 ms)
      SetAMCLParams(d=0.15, a=0.1)       — restore AMCL
      Wait(1.0 s)

10. Door close detection:
      WaitForDoorClose loop:
        Inverter(ComputePathToPose(staging)) → SUCCESS when plan fails
        door closes → no path → plan FAIL → Inverter SUCCESS → break
      Wait(3.0 s)

11. Map & localization switch:
      GetNamedMap(floor1, "open")        → floor1_open.yaml
      SwitchMap(floor1_open.yaml)        — load new occupancy grid
      PublishInitialPose with AprilTag:
        - query /tf for tag36h11:0
        - parking_error = expected_tag - actual_tag
        - publish corrected pose to /initialpose → AMCL resets
      ClearEntireCostmap (local + global)
      ToggleAprilTag(false)

12. Elevator exit (Attempt 1):
      GetNamedPose(elevator_exit, floor1)
      CallElevator(floor1)               — press floor1 button
      CheckFloorArrival(floor1):         — YOLO watches button panel
        lit class detected → ON (elevator moving)
        ... elevator arrives at floor1 ...
        unlit class detected for 0.1 s → arrived=true → SUCCESS
      SetControllerParams(vx=0.75)
      ClearEntireCostmap (local + global)
      WaitForPath_Exit loop:             — door opens on floor1
        ComputePathToPose(exit_pose) → SUCCESS → break
      NavigateToPose(exit_pose)          — robot exits elevator
      SetControllerParams(vx=0.5)
      SetCostmapInflation(0.5)

13. Final destination:
      NavigateToPose(5.90, -2.29, 0.0)  — drive to office_101

14. BT returns SUCCESS
15. named_goal_server updates current_floor = floor1
16. /go_to_pose response: accepted=true, "Navigation to office_101 completed"
```
