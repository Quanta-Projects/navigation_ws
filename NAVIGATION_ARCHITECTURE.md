# Complete Navigation Architecture Documentation

## Overview

This document describes the complete navigation architecture from high-level location commands down to the hardware level, showing how a simple location name like "dock" results in robot movement.

---

## Architecture Layers

```
┌─────────────────────────────────────────────────────────────────────┐
│                        APPLICATION LAYER                             │
│  User publishes location name to /location topic                    │
│  Example: rostopic pub /location std_msgs/String "data: 'dock'"    │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    LOCATION SUBSCRIBER NODE                          │
│  Node: location_subscriber                                           │
│  Subscribes to: /location (std_msgs/String)                         │
│  Calls service: /go_to_pose (smrr_interfaces/GoToNamedPose)        │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    NAMED GOAL SERVER NODE                            │
│  Node: named_goal_server                                             │
│  Service: /go_to_pose (smrr_interfaces/GoToNamedPose)              │
│  Loads: config/locations.yaml                                        │
│  Sends action: NavigateToPose (nav2_msgs/action)                   │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    NAV2 BT NAVIGATOR                                 │
│  Node: bt_navigator                                                  │
│  Action server: /navigate_to_pose                                    │
│  Uses: Behavior Tree for navigation orchestration                   │
│  XML: navigate_w_replanning_and_recovery.xml                        │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                ┌──────────┴──────────┐
                ▼                     ▼
┌───────────────────────────┐  ┌──────────────────────────────┐
│   PLANNER SERVER          │  │   CONTROLLER SERVER          │
│   Node: planner_server    │  │   Node: controller_server    │
│   Plugin: NavfnPlanner    │  │   Plugin: DWBLocalPlanner    │
│   Generates global path   │  │   Executes local planning    │
└───────────┬───────────────┘  └──────────────┬───────────────┘
            │                                  │
            └──────────────┬───────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    COSTMAP LAYERS                                    │
│  Global Costmap: Planning space (map frame)                         │
│  Local Costmap: Dynamic obstacle avoidance (odom frame)            │
│  Subscribes: /scan (sensor_msgs/LaserScan)                         │
│  Uses: AMCL localization (/map → /odom transform)                  │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    VELOCITY COMMAND OUTPUT                           │
│  Topic: /diff_drive_controller/cmd_vel_unstamped                    │
│  Message: geometry_msgs/Twist                                        │
│  Contains: linear.x (m/s), angular.z (rad/s)                       │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                 DIFF DRIVE CONTROLLER (ros2_control)                │
│  Node: controller_manager                                            │
│  Controller: diff_drive_controller                                   │
│  Converts Twist → individual wheel velocities                       │
│  Publishes: /diff_drive_controller/odom                            │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                ┌──────────┴──────────┐
                ▼                     ▼
    ┌──────────────────┐   ┌──────────────────┐
    │  SIMULATION      │   │  HARDWARE        │
    │  (Gazebo)        │   │  (BaseController)│
    │  GazeboSystem    │   │  STM32 + Motors  │
    └──────────────────┘   └──────────────────┘
```

---

## 1. Application Layer: Location Topic

### Publisher
Any ROS 2 node or command-line tool can publish location names.

**Example Command:**
```bash
ros2 topic pub /location std_msgs/String "data: 'dock'"
```

**Message Type:** `std_msgs/String`
- Simple string containing the location name
- Location names must match entries in `locations.yaml`

**Available Locations** (from `config/locations.yaml`):
```yaml
locations:
  dock:
    x: 2.23
    y: -1.0
    yaw: 0.0
  elevator:
    x: -2.0
    y: 1.3
    yaw: 1.57
  left_back:
    x: -9.9
    y: 1.33
    yaw: 0.0
  right_back:
    x: -9.9
    y: -3.0
    yaw: 3.14
```

---

## 2. Location Subscriber Node

**Source:** `src/smrr_navigation/smrr_navigation/location_subscriber.py`

### Purpose
Acts as a bridge between the simple topic interface and the service-based navigation system.

### Functionality
1. **Subscribes** to `/location` topic
2. **Receives** location name as string
3. **Validates** location name is not empty
4. **Checks** if `/go_to_pose` service is available
5. **Calls** `/go_to_pose` service with location name
6. **Returns** immediately (non-blocking)

### Key Code Flow
```python
def location_callback(self, msg):
    location_name = msg.data.strip()
    
    # Create service request
    request = GoToNamedPose.Request()
    request.name = location_name
    
    # Call service asynchronously
    future = self.client.call_async(request)
    future.add_done_callback(lambda f: self.service_response_callback(f, location_name))
```

### Service Definition
**Service:** `smrr_interfaces/srv/GoToNamedPose`
```
# Request
string name  # Location name to navigate to
---
# Response
bool accepted     # Whether the request was accepted
string message    # Status or error message
```

### Parameters
- `location_topic` (default: "location"): Topic to subscribe to
- `service_name` (default: "/go_to_pose"): Service to call
- `service_timeout` (default: 5.0): Service connection timeout

### Lifecycle
- Waits up to 10 seconds for service at startup
- Attempts reconnection if service not available when message received
- Logs all requests and responses

---

## 3. Named Goal Server Node

**Source:** `src/smrr_navigation/smrr_navigation/named_goal_server.py`

### Purpose
Translates string-based location names into Nav2 NavigateToPose action goals.

### Initialization Process

#### 1. Load Locations from YAML
```python
def load_locations(self, filename):
    # Loads from config/locations.yaml
    # Returns: {name: {x, y, yaw}}
```

**File Location:**
- Primary: `install/smrr_navigation/share/smrr_navigation/config/locations.yaml`
- Fallback: Relative path from source

**Example Location Data:**
```python
locations = {
    'dock': {'x': 2.23, 'y': -1.0, 'yaw': 0.0},
    'elevator': {'x': -2.0, 'y': 1.3, 'yaw': 1.57}
}
```

#### 2. Create Service Server
```python
self.srv = self.create_service(
    GoToNamedPose,
    '/go_to_pose',
    self.handle_go_to_pose,
    callback_group=self.callback_group
)
```

#### 3. Create Nav2 Action Client
```python
self.nav_client = ActionClient(
    self,
    NavigateToPose,
    'navigate_to_pose',
    callback_group=self.callback_group
)
```

### Service Handler Flow

```python
def handle_go_to_pose(self, request, response):
    location_name = request.name
    
    # 1. Validate location exists
    if location_name not in self.locations:
        response.accepted = False
        response.message = f'Unknown location: {location_name}'
        return response
    
    # 2. Get location coordinates
    loc = self.locations[location_name]
    
    # 3. Check action server availability
    if not self.nav_client.server_is_ready():
        response.accepted = False
        response.message = 'Navigation action server not available'
        return response
    
    # 4. Create NavigateToPose goal
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = self.create_pose_stamped(loc['x'], loc['y'], loc['yaw'])
    
    # 5. Send action goal asynchronously
    send_goal_future = self.nav_client.send_goal_async(
        goal_msg,
        feedback_callback=self.navigation_feedback_callback
    )
    
    # 6. Register callbacks
    send_goal_future.add_done_callback(
        lambda future: self.goal_response_callback(future, location_name)
    )
    
    # 7. Return immediately (non-blocking service)
    response.accepted = True
    response.message = f'Navigation goal to {location_name} sent successfully'
    return response
```

### Coordinate Frame Conversion
```python
def create_pose_stamped(self, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'  # Global frame
    pose.header.stamp = self.get_clock().now().to_msg()
    
    # Position
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    
    # Orientation (yaw to quaternion)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    return pose
```

### Action Callbacks

**Goal Response Callback:**
```python
def goal_response_callback(self, future, location_name):
    goal_handle = future.result()
    if goal_handle.accepted:
        # Goal accepted by Nav2
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self.navigation_result_callback(f, location_name)
        )
```

**Result Callback:**
```python
def navigation_result_callback(self, future, location_name):
    result = future.result()
    status = result.status
    
    if status == 4:  # SUCCEEDED
        self.get_logger().info(f'Successfully navigated to {location_name}')
    elif status == 5:  # CANCELED
        self.get_logger().warn(f'Navigation to {location_name} was canceled')
    elif status == 6:  # ABORTED
        self.get_logger().error(f'Navigation to {location_name} was aborted')
```

### Parameters
- `locations_file` (default: "locations.yaml"): YAML file with location data
- `global_frame` (default: "map"): Global coordinate frame
- `action_timeout` (default: 300.0): Maximum navigation time (5 minutes)

### Executor
Uses `MultiThreadedExecutor` for concurrent service and action handling.

---

## 4. Nav2 BT Navigator

**Configuration:** `config/smrr_nav_params.yaml`

### Purpose
Orchestrates the navigation process using a Behavior Tree (BT) state machine.

### Core Components

#### Behavior Tree Plugins
```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /diff_drive_controller/odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    bt_loop_duration: 10
    default_server_timeout: 20
    
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node      # Planning
    - nav2_follow_path_action_bt_node               # Path following
    - nav2_clear_costmap_service_bt_node            # Recovery
    - nav2_spin_action_bt_node                      # Recovery
    - nav2_back_up_action_bt_node                   # Recovery
    - nav2_wait_action_bt_node                      # Recovery
    # ... more plugins
```

### Navigation Behavior Tree Flow

```
NavigateToPose Action Received
        │
        ▼
┌──────────────────┐
│ Compute Path     │  → Calls planner_server
│ to Pose          │     Returns global path
└─────┬────────────┘
      │
      ▼
┌──────────────────┐
│ Follow Path      │  → Calls controller_server
│                  │     Executes path with local planning
└─────┬────────────┘
      │
      ├─ Success → Goal Reached ✓
      │
      ├─ Failure → Recovery Behaviors
      │            ├─ Clear Costmap
      │            ├─ Spin in Place
      │            ├─ Back Up
      │            └─ Wait
      │
      └─ Replanning → Compute new path and retry
```

### Frame Hierarchy
The BT Navigator coordinates between different coordinate frames:
- **map**: Global planning frame (static, from AMCL)
- **odom**: Local odometry frame (drifts over time)
- **base_footprint**: Robot center on ground plane
- **base_link**: Robot center (may have z-offset)

---

## 5. Planner Server

**Configuration:** `config/smrr_nav_params.yaml` (lines 317-327)

### Purpose
Generates collision-free global paths from start to goal using the static map.

### Configuration
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"  # Dijkstra/A* planner
      tolerance: 0.5         # Goal tolerance (meters)
      use_astar: false       # Use Dijkstra (A* disabled)
      allow_unknown: true    # Can plan through unknown space
```

### Algorithm: NavFn (Navigation Function)
- **Type:** Grid-based planner
- **Method:** Dijkstra's algorithm (wavefront propagation)
- **Input:** 
  - Start pose (current robot position from AMCL)
  - Goal pose (from NavigateToPose action)
  - Global costmap (inflated obstacles)
- **Output:** 
  - `nav_msgs/Path`: Sequence of waypoints from start to goal

### Planning Process
1. Receives planning request from BT Navigator
2. Reads global costmap (map frame)
3. Runs Dijkstra algorithm to find minimum cost path
4. Inflates obstacles using robot footprint
5. Returns path as series of poses
6. Publishes visualization on `/plan` topic

### Global Costmap Configuration
```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_footprint
  update_frequency: 1.0
  publish_frequency: 1.0
  
  plugins: ["static_layer", "inflation_layer"]
  
  static_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_subscribe_transient_local: true
  
  inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    cost_scaling_factor: 3.0
    inflation_radius: 0.55
```

---

## 6. Controller Server

**Configuration:** `config/smrr_nav_params.yaml` (lines 109-178)

### Purpose
Executes the global path using local reactive planning and obstacle avoidance.

### Configuration
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # 20 Hz control loop
    odom_topic: /diff_drive_controller/odom
    
    controller_plugins: ["FollowPath"]
    
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      
      # Velocity limits
      max_vel_x: 0.35          # m/s
      min_vel_x: 0.00
      max_vel_theta: 1.5       # rad/s
      
      # Acceleration limits
      acc_lim_x: 2.5           # m/s²
      acc_lim_theta: 4.0       # rad/s²
      decel_lim_x: -2.5
      decel_lim_theta: -4.0
      
      # Trajectory simulation
      vx_samples: 20           # Linear velocity samples
      vtheta_samples: 40       # Angular velocity samples
      sim_time: 1.7            # Lookahead time (seconds)
      
      # Trajectory scoring
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", 
                "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

### DWB Local Planner Algorithm

#### Dynamic Window Approach (DWA)
The DWB (Dynamic Window B) planner generates and evaluates velocity trajectories:

**1. Velocity Space Sampling**
```
For each velocity (v_x, v_θ) in dynamic window:
  - Generate trajectory by forward simulation
  - Check for collisions with local costmap
  - Score trajectory using critic functions
  - Select best trajectory
```

**2. Dynamic Window Constraints**
```
Reachable velocities = current velocity ± acceleration * dt
```
Example:
- Current: v_x = 0.2 m/s, v_θ = 0.5 rad/s
- Max accel: 2.5 m/s², 4.0 rad/s²
- Control period: 0.05s (20 Hz)
- Window: [0.075-0.325 m/s] × [-0.7 to 1.7 rad/s]

**3. Trajectory Critics (Scoring Functions)**

| Critic | Weight | Purpose |
|--------|--------|---------|
| BaseObstacle | 0.02 | Penalize trajectories near obstacles |
| PathAlign | 32.0 | Reward alignment with global path |
| GoalAlign | 24.0 | Reward pointing toward goal |
| PathDist | 32.0 | Reward staying close to path |
| GoalDist | 24.0 | Reward progress toward goal |
| RotateToGoal | 32.0 | In-place rotation when near goal |
| Oscillation | - | Penalize back-and-forth motion |

**4. Trajectory Evaluation**
```
Total Score = Σ(critic_weight × critic_score)
Best trajectory = argmax(Total Score)
```

### Goal Checking
```yaml
general_goal_checker:
  plugin: "nav2_controller::SimpleGoalChecker"
  xy_goal_tolerance: 0.15      # 15cm position tolerance
  yaw_goal_tolerance: 0.15     # ~8.6° orientation tolerance
  stateful: true
```

### Progress Checking
```yaml
progress_checker:
  plugin: "nav2_controller::SimpleProgressChecker"
  required_movement_radius: 0.5   # Must move 0.5m
  movement_time_allowance: 10.0   # Within 10 seconds
```

### Control Loop (20 Hz)
```
Every 0.05 seconds:
  1. Read robot pose from odometry
  2. Transform global path to odom frame
  3. Sample velocity trajectories (20 × 40 = 800)
  4. Forward simulate each trajectory (1.7s ahead)
  5. Check collisions in local costmap
  6. Score trajectories with critics
  7. Select best trajectory
  8. Send velocity command to /diff_drive_controller/cmd_vel_unstamped
```

### Local Costmap Configuration
```yaml
local_costmap:
  global_frame: odom          # Local frame (drifts)
  robot_base_frame: base_footprint
  update_frequency: 5.0       # 5 Hz updates
  publish_frequency: 2.0
  
  width: 3                    # 3m × 3m window
  height: 3
  resolution: 0.05            # 5cm cells
  
  plugins: ["voxel_layer", "inflation_layer"]
  
  voxel_layer:
    plugin: "nav2_costmap_2d::VoxelLayer"
    observation_sources: scan
    
    scan:
      topic: /scan
      max_obstacle_height: 2.0
      clearing: true
      marking: true
```

---

## 7. Localization (AMCL)

**Configuration:** `config/smrr_nav_params.yaml` (lines 1-50)

### Purpose
Provides the robot's position in the map frame using Adaptive Monte Carlo Localization.

### AMCL Parameters
```yaml
amcl:
  ros__parameters:
    global_frame_id: "map"
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    odom_topic: /diff_drive_controller/odom
    scan_topic: scan
    
    # Particle filter
    max_particles: 2000
    min_particles: 500
    
    # Motion model
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    alpha1: 0.2  # Rotation from rotation
    alpha2: 0.2  # Rotation from translation
    alpha3: 0.2  # Translation from translation
    alpha4: 0.2  # Translation from rotation
    
    # Sensor model
    laser_model_type: "likelihood_field"
    max_beams: 60
    laser_likelihood_max_dist: 2.0
    
    # Update thresholds
    update_min_d: 0.25    # Move 0.25m to update
    update_min_a: 0.2     # Rotate 0.2 rad to update
    
    # Initial pose
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      yaw: 0.0
```

### TF Tree Published by AMCL
```
map
 └─ odom (published by AMCL)
     └─ base_footprint (published by diff_drive_controller)
         └─ base_link
             └─ laser_frame
```

### Startup Localization
**Node:** `startup_localizer.py`
- Waits for odometry to stabilize (5 messages)
- Publishes initial pose to AMCL
- Allows AMCL to converge before navigation starts

---

## 8. Velocity Command Generation

### Topic: `/diff_drive_controller/cmd_vel_unstamped`
**Message Type:** `geometry_msgs/Twist`

```python
Twist:
  linear:
    x: 0.25      # Forward velocity (m/s)
    y: 0.0       # Lateral velocity (always 0 for diff drive)
    z: 0.0       # Vertical velocity (always 0)
  angular:
    x: 0.0       # Roll rate (always 0)
    y: 0.0       # Pitch rate (always 0)
    z: 0.3       # Yaw rate (rad/s) - turning
```

### Publisher
- **Node:** `controller_server`
- **Rate:** 20 Hz (every 0.05 seconds)
- **Range:** 
  - Linear: [-0.35, 0.35] m/s
  - Angular: [-1.5, 1.5] rad/s

---

## 9. Differential Drive Controller (ros2_control)

**Configuration:** `config/hardware_controller.yaml`

### Purpose
Converts Twist commands into individual wheel velocities and manages odometry.

### ros2_control Framework
```
controller_manager (ROS 2 Control Manager)
    │
    ├─ joint_state_broadcaster → Publishes /joint_states
    │
    └─ diff_drive_controller
         ├─ Subscribes: /diff_drive_controller/cmd_vel_unstamped
         ├─ Publishes: /diff_drive_controller/odom
         └─ Commands: left_wheel_joint, right_wheel_joint
```

### Configuration
```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]  
    right_wheel_names: ["right_wheel_joint"]
    
    # Robot geometry (calibrated)
    wheel_separation: 0.402  # Distance between wheels (m)
    wheel_radius: 0.081      # Wheel radius (m)
    
    # Frames
    odom_frame_id: odom
    base_frame_id: base_footprint
    
    # Feedback mode
    position_feedback: false  # Use velocity feedback
    open_loop: false          # Closed-loop control
    
    publish_rate: 100.0  # Odometry publish rate (Hz)
```

### Differential Drive Kinematics

**Forward Kinematics** (wheel velocities → robot velocity):
```python
# Inputs: wheel velocities (rad/s)
v_left = left_wheel_velocity * wheel_radius
v_right = right_wheel_velocity * wheel_radius

# Robot velocity in body frame
v_linear = (v_left + v_right) / 2              # m/s
v_angular = (v_right - v_left) / wheel_separation  # rad/s
```

**Inverse Kinematics** (robot velocity → wheel velocities):
```python
# Inputs: desired robot velocity (Twist message)
v_x = cmd_vel.linear.x      # m/s
ω = cmd_vel.angular.z       # rad/s

# Required wheel velocities
v_left = (v_x - ω * wheel_separation/2) / wheel_radius   # rad/s
v_right = (v_x + ω * wheel_separation/2) / wheel_radius  # rad/s
```

**Example Calculation:**
```
Given: v_x = 0.3 m/s, ω = 0.5 rad/s
       wheel_separation = 0.402 m, wheel_radius = 0.081 m

v_left = (0.3 - 0.5 × 0.402/2) / 0.081 = 2.457 rad/s
v_right = (0.3 + 0.5 × 0.402/2) / 0.081 = 4.948 rad/s

Verification:
  v_linear = (2.457 + 4.948)/2 × 0.081 = 0.3 m/s ✓
  v_angular = (4.948 - 2.457) / 0.402 = 0.5 rad/s ✓
```

### Odometry Integration
```python
# At each timestep (dt = 0.01s at 100Hz):
# 1. Read encoder feedback (wheel positions/velocities)
# 2. Compute forward kinematics
v_linear, v_angular = forward_kinematics(v_left, v_right)

# 3. Update pose in odom frame
x += v_linear * cos(θ) * dt
y += v_linear * sin(θ) * dt
θ += v_angular * dt

# 4. Publish odometry message
odom_msg.pose.pose.position.x = x
odom_msg.pose.pose.position.y = y
odom_msg.pose.pose.orientation = quaternion_from_yaw(θ)
odom_msg.twist.twist.linear.x = v_linear
odom_msg.twist.twist.angular.z = v_angular
```

### Control Loop (50-100 Hz)
```
Every 0.01-0.02 seconds:
  1. Receive Twist command from controller_server
  2. Convert to wheel velocities (inverse kinematics)
  3. Write wheel velocity commands to hardware interface
  4. Read wheel encoder feedback from hardware interface
  5. Compute odometry (forward kinematics + integration)
  6. Publish odometry on /diff_drive_controller/odom
  7. Publish TF transform: odom → base_footprint
```

### Odometry Message
**Topic:** `/diff_drive_controller/odom`
**Type:** `nav_msgs/Odometry`

```python
Odometry:
  header:
    frame_id: "odom"
  child_frame_id: "base_footprint"
  
  pose:
    pose:
      position: {x, y, z}        # Robot position in odom frame
      orientation: {x, y, z, w}  # Quaternion (from yaw)
    covariance: [36 elements]    # Position uncertainty
  
  twist:
    twist:
      linear: {x, y, z}          # Robot velocity
      angular: {x, y, z}         # Angular velocity
    covariance: [36 elements]    # Velocity uncertainty
```

---

## 10. Hardware Layer

### Two Implementations

#### A. Simulation (Gazebo)

**URDF Configuration:** `smrr_ros2_control.urdf.xacro`
```xml
<ros2_control name="RobotSystem" type="system">
  <xacro:if value="$(arg is_sim)">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </xacro:if>
</ros2_control>
```

**How it works:**
- Gazebo Physics Engine simulates wheel dynamics
- GazeboSystem plugin bridges Gazebo ↔ ros2_control
- Reads commanded velocities from diff_drive_controller
- Updates wheel positions in simulation
- Returns encoder feedback to diff_drive_controller

#### B. Hardware (BaseController + STM32)

**URDF Configuration:** `smrr_ros2_control.urdf.xacro`
```xml
<ros2_control name="RobotSystem" type="system">
  <xacro:unless value="$(arg is_sim)">
    <hardware>
      <plugin>smrr_base_controller/BaseController</plugin>
      <param name="port">/dev/ttyUSB0</param>
    </hardware>
    
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </xacro:unless>
</ros2_control>
```

### Hardware Interface: BaseController

**Source:** `src/smrr_base_controller/src/base_controller.cpp`

**Architecture:**
```
Jetson Nano (Running ROS 2)
    │
    │ USB-to-Serial (UART)
    │ /dev/ttyUSB0 @ 115200 baud
    │
    ▼
STM32 Microcontroller (Firmware)
    │
    ├─ PID Control Loop (100 Hz)
    ├─ Encoder Reading (Interrupts)
    │
    ▼
L298N Motor Driver (PWM)
    │
    ▼
DC Motors + Encoders
```

### BaseController Implementation

**Lifecycle States:**
```
on_init()
   │
   ├─ Open serial port (/dev/ttyUSB0)
   ├─ Configure: 115200 baud, 8N1
   ├─ Register command interfaces (velocity)
   └─ Register state interfaces (position, velocity)
   │
   ▼
on_activate()
   │
   ├─ Flush serial buffers
   └─ Reset encoder positions
   │
   ▼
write() (50-100 Hz)
   │
   ├─ Read commanded velocities from diff_drive_controller
   ├─ Format: "v_left,v_right\n"
   ├─ Send via serial: "-2.5,3.2\n"
   └─ (Non-blocking)
   │
   ▼
read() (50-100 Hz)
   │
   ├─ Read serial data from STM32
   ├─ Parse: "pos_left,pos_right,vel_left,vel_right\n"
   ├─ Update state interfaces
   └─ Available to diff_drive_controller
```

### Serial Communication Protocol

**Command Format** (Jetson → STM32):
```
"v_left,v_right\n"

Example: "-2.456,3.789\n"
  - v_left: Target left wheel velocity (rad/s)
  - v_right: Target right wheel velocity (rad/s)
  - Newline terminated
```

**Feedback Format** (STM32 → Jetson):
```
"pos_left,pos_right,vel_left,vel_right\n"

Example: "12.345,15.678,2.1,3.4\n"
  - pos_left: Left wheel position (radians)
  - pos_right: Right wheel position (radians)
  - vel_left: Left wheel velocity (rad/s)
  - vel_right: Right wheel velocity (rad/s)
  - Newline terminated
```

### STM32 Firmware Architecture

**Source:** `src/smrr_base_controller/firmware/firmware.ino`

**Main Components:**
1. **Serial Communication** (9600 baud)
2. **PID Controllers** (left and right wheels)
3. **Encoder Reading** (interrupt-based)
4. **PWM Motor Control** (L298N driver)
5. **Control Loop** (100 Hz)

**Control Loop:**
```cpp
void loop() {
  // 1. Read commanded velocities from serial
  if (Serial.available()) {
    parseCommand();  // Updates target_vel_left, target_vel_right
  }
  
  // 2. Read encoder counts (from interrupts)
  long left_ticks = encoder_left.read();
  long right_ticks = encoder_right.read();
  
  // 3. Compute velocities from encoder delta
  float vel_left = (left_ticks - prev_left) / dt * ticks_to_rad_per_sec;
  float vel_right = (right_ticks - prev_right) / dt * ticks_to_rad_per_sec;
  
  // 4. PID control
  float error_left = target_vel_left - vel_left;
  float error_right = target_vel_right - vel_right;
  
  pwm_left = pid_left.compute(error_left);
  pwm_right = pid_right.compute(error_right);
  
  // 5. Set motor PWM
  setMotorPWM(LEFT_MOTOR, pwm_left);
  setMotorPWM(RIGHT_MOTOR, pwm_right);
  
  // 6. Send feedback to Jetson
  Serial.print(left_ticks * ticks_to_rad);
  Serial.print(",");
  Serial.print(right_ticks * ticks_to_rad);
  Serial.print(",");
  Serial.print(vel_left);
  Serial.print(",");
  Serial.println(vel_right);
  
  delay(10);  // 100 Hz loop
}
```

**PID Controller:**
```cpp
class PIDController {
  float kp = 1.0, ki = 0.1, kd = 0.01;
  float integral = 0.0, prev_error = 0.0;
  
  float compute(float error) {
    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    
    float output = kp * error + ki * integral + kd * derivative;
    
    prev_error = error;
    return constrain(output, -255, 255);  // PWM limits
  }
};
```

**Encoder Interrupts:**
```cpp
// Called on every encoder tick (rising/falling edge)
void encoderISR_left() {
  encoder_left_count++;
}

void encoderISR_right() {
  encoder_right_count++;
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderISR_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderISR_right, RISING);
}
```

---

## Complete Data Flow Example

### Scenario: Navigate to "dock" location

```
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 1: User Command                                                 │
│ $ ros2 topic pub /location std_msgs/String "data: 'dock'"          │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 2: Location Subscriber                                          │
│ - Receives: "dock"                                                   │
│ - Calls: /go_to_pose service                                        │
│ - Request: name="dock"                                               │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 3: Named Goal Server                                            │
│ - Looks up "dock" in locations.yaml                                 │
│ - Found: x=2.23, y=-1.0, yaw=0.0                                    │
│ - Creates PoseStamped in map frame                                  │
│ - Sends NavigateToPose action goal                                  │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 4: BT Navigator                                                 │
│ - Receives action goal: pose=(2.23, -1.0, 0.0) in map frame        │
│ - Executes behavior tree                                             │
│ - Calls planner_server to compute path                              │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 5: Planner Server                                               │
│ - Current pose: (0.0, 0.0, 0.0) from AMCL                          │
│ - Goal pose: (2.23, -1.0, 0.0)                                     │
│ - Runs NavfnPlanner (Dijkstra)                                      │
│ - Generates path: [(0,0), (0.5,-0.2), (1.0,-0.5), ..., (2.23,-1.0)]│
│ - Returns nav_msgs/Path with 15 waypoints                           │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 6: Controller Server                                            │
│ - Receives path from planner                                         │
│ - Starts 20 Hz control loop                                          │
│                                                                      │
│ Loop iteration (every 0.05s):                                        │
│   1. Read current pose: (0.05, -0.01, 0.1) from odometry           │
│   2. Transform path to odom frame                                   │
│   3. Sample 800 velocity trajectories:                              │
│      - v_x ∈ [0.0, 0.35] m/s (20 samples)                          │
│      - v_θ ∈ [-1.5, 1.5] rad/s (40 samples)                        │
│   4. Simulate each trajectory 1.7s into future                      │
│   5. Score trajectories:                                             │
│      - Best: v_x=0.25 m/s, v_θ=0.3 rad/s                           │
│      - Score: PathAlign=0.95, GoalDist=0.87, BaseObstacle=1.0      │
│   6. Send Twist command to /diff_drive_controller/cmd_vel_unstamped│
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 7: Diff Drive Controller                                       │
│ - Receives: Twist(v_x=0.25, v_θ=0.3)                               │
│ - Inverse kinematics:                                                │
│     v_left = (0.25 - 0.3×0.402/2) / 0.081 = 2.341 rad/s            │
│     v_right = (0.25 + 0.3×0.402/2) / 0.081 = 3.829 rad/s           │
│ - Writes to hardware interface:                                     │
│     hw_commands_[LEFT] = 2.341                                      │
│     hw_commands_[RIGHT] = 3.829                                     │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 8a: SIMULATION PATH                                             │
│ GazeboSystem                                                         │
│ - Sets wheel joint velocities in Gazebo                             │
│ - Physics engine updates wheel positions                            │
│ - Returns encoder feedback (position, velocity)                     │
└─────────────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 8b: HARDWARE PATH                                               │
│ BaseController (Jetson)                                              │
│ - Formats command: "2.341,3.829\n"                                  │
│ - Sends via UART to STM32 @ 115200 baud                            │
│                                                                      │
│ STM32 Firmware:                                                      │
│   1. Parses: target_vel_left=2.341, target_vel_right=3.829         │
│   2. Reads encoders: left_ticks=1234, right_ticks=5678              │
│   3. Computes current velocity: vel_left=2.1, vel_right=3.5        │
│   4. PID control:                                                    │
│        error_left = 2.341 - 2.1 = 0.241                            │
│        error_right = 3.829 - 3.5 = 0.329                           │
│        pwm_left = PID_compute(0.241) = 185                          │
│        pwm_right = PID_compute(0.329) = 195                         │
│   5. Writes PWM to L298N motor driver                               │
│   6. Sends feedback: "12.45,56.78,2.1,3.5\n"                       │
│                                                                      │
│ BaseController (Jetson):                                             │
│ - Receives feedback via UART                                         │
│ - Parses: pos_left=12.45, pos_right=56.78, vel_left=2.1, vel_right=3.5 │
│ - Updates state interfaces for diff_drive_controller               │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 9: Odometry Integration                                         │
│ Diff Drive Controller (100 Hz):                                     │
│ - Reads wheel feedback: vel_left=2.1, vel_right=3.5 rad/s          │
│ - Forward kinematics:                                                │
│     v_linear = (2.1 + 3.5)/2 × 0.081 = 0.227 m/s                   │
│     v_angular = (3.5 - 2.1) / 0.402 = 0.348 rad/s                  │
│ - Integrates pose (dt=0.01s):                                       │
│     x += 0.227 × cos(θ) × 0.01 = x + 0.00227                       │
│     y += 0.227 × sin(θ) × 0.01 = y + 0.00002                       │
│     θ += 0.348 × 0.01 = θ + 0.00348                                │
│ - Publishes Odometry message                                         │
│ - Publishes TF: odom → base_footprint                              │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 10: Localization (AMCL)                                         │
│ - Receives odometry: pose in odom frame                             │
│ - Receives laser scan: /scan                                         │
│ - Matches scan to map using particle filter                         │
│ - Estimates robot pose in map frame                                 │
│ - Publishes TF: map → odom (corrects odometry drift)               │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│ STEP 11: Loop Continues                                              │
│ - Controller server reads updated pose from AMCL                    │
│ - Generates new velocity command based on updated pose              │
│ - Cycle repeats at 20 Hz until goal reached                         │
│                                                                      │
│ When within goal tolerance:                                          │
│   - xy_tolerance: ||(x,y) - (2.23,-1.0)|| < 0.15m                  │
│   - yaw_tolerance: |θ - 0.0| < 0.15 rad (~8.6°)                    │
│ → Navigation succeeds, action completes                             │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Timing and Frequencies

| Component | Frequency | Period | Notes |
|-----------|-----------|--------|-------|
| location_subscriber | Event-driven | - | Responds to /location messages |
| named_goal_server | Event-driven | - | Responds to service calls |
| bt_navigator | 10 Hz | 100ms | Behavior tree loop |
| planner_server | On-demand | - | Replans when needed |
| controller_server | 20 Hz | 50ms | DWB trajectory planning |
| diff_drive_controller | 100 Hz | 10ms | Odometry integration |
| BaseController (hardware) | 50-100 Hz | 10-20ms | Serial communication |
| STM32 firmware | 100 Hz | 10ms | PID control loop |
| AMCL | 1-5 Hz | 200-1000ms | Localization updates |
| Local costmap | 5 Hz | 200ms | Obstacle updates |
| Global costmap | 1 Hz | 1000ms | Static map updates |

---

## Key Topics

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/location` | std_msgs/String | User/App | location_subscriber | Location name commands |
| `/navigate_to_pose` | nav2_msgs/action/NavigateToPose | - | bt_navigator | Nav2 action |
| `/plan` | nav_msgs/Path | planner_server | controller_server | Global path |
| `/diff_drive_controller/cmd_vel_unstamped` | geometry_msgs/Twist | controller_server | diff_drive_controller | Velocity commands |
| `/diff_drive_controller/odom` | nav_msgs/Odometry | diff_drive_controller | AMCL, controller_server | Odometry |
| `/scan` | sensor_msgs/LaserScan | Lidar driver | AMCL, costmaps | Laser scans |
| `/map` | nav_msgs/OccupancyGrid | map_server | planner_server, costmaps | Static map |
| `/tf` | tf2_msgs/TFMessage | Multiple | Multiple | Transform tree |

---

## Key Services

| Service | Type | Server | Description |
|---------|------|--------|-------------|
| `/go_to_pose` | smrr_interfaces/srv/GoToNamedPose | named_goal_server | Navigate to named location |

---

## TF Frame Tree

```
map (AMCL localization frame)
 │
 └── odom (Odometry frame, drifts over time)
      │ [Published by: diff_drive_controller]
      │
      └── base_footprint (Robot center on ground)
           │ [Published by: diff_drive_controller]
           │
           ├── base_link (Robot center)
           │    │
           │    ├── laser_frame (Lidar sensor)
           │    │
           │    ├── front_camera_link (Front ZED camera)
           │    │
           │    └── rear_camera_link (Rear ZED camera)
           │
           ├── left_wheel_link
           │
           └── right_wheel_link

[Published by AMCL: map → odom]
```

---

## Summary

The navigation architecture follows a hierarchical control structure:

1. **Application Layer**: Simple string-based location commands
2. **Abstraction Layer**: Named goal service translates names to coordinates
3. **Task Planning**: BT Navigator orchestrates high-level behaviors
4. **Motion Planning**: Planner generates collision-free paths
5. **Motion Control**: Controller executes paths with obstacle avoidance
6. **Robot Control**: Diff drive controller converts to wheel velocities
7. **Hardware Interface**: BaseController communicates with low-level controller
8. **Embedded Control**: STM32 firmware performs PID control
9. **Actuators**: Motor drivers command DC motors
10. **Feedback Loop**: Encoders → odometry → localization → planning

Each layer operates at its appropriate frequency, from 1 Hz global planning to 100 Hz motor control, ensuring smooth and responsive navigation behavior.
