# Running SMRR Robot on Real Hardware

## Overview

Your robot has three launch modes:
1. **Gazebo Simulation** (current setup)
2. **Real Hardware** (what you're asking about)
3. **Hybrid** (simulation + hardware testing)

## Architecture

```
┌─────────────────────────────────────────────┐
│  Launch File (hardware_robot.launch.py)     │
└───────────────┬─────────────────────────────┘
                │
                ├─→ robot_state_publisher (is_sim:=false)
                │
                ├─→ ros2_control_node
                │   └─→ Loads: smrr_base_controller/BaseController
                │       └─→ Communicates with: /dev/ttyUSB0
                │
                ├─→ Controller Spawners
                │   ├─→ joint_state_broadcaster
                │   ├─→ arm_controller
                │   └─→ diff_drive_controller
                │
                └─→ Navigation Stack (optional)
```

## Prerequisites

### 1. Hardware Interface Package

Your `smrr_base_controller` package must be built and installed:

```bash
cd ~/navigation_ws
colcon build --packages-select smrr_base_controller
source install/setup.bash
```

### 2. Serial Port Permissions

Grant access to the serial port:

```bash
# Find your device
ls -l /dev/ttyUSB* /dev/ttyACM*

# Add user to dialout group (one-time setup)
sudo usermod -a -G dialout $USER

# OR set permissions directly
sudo chmod 666 /dev/ttyUSB0

# Reboot or log out/in for group changes to take effect
```

### 3. Verify Hardware Interface Plugin

Check that your hardware interface is registered:

```bash
ros2 pkg prefix smrr_base_controller
```

## Launch Options

### Option 1: Full Hardware Launch (Recommended)

Uses the comprehensive `hardware_robot.launch.py`:

```bash
# Basic hardware launch
ros2 launch smrr_description hardware_robot.launch.py

# With navigation
ros2 launch smrr_description hardware_robot.launch.py use_navigation:=true

# Custom serial port
ros2 launch smrr_description hardware_robot.launch.py port:=/dev/ttyACM0
```

**Features:**
- ✅ Proper use_sim_time:=false for all nodes
- ✅ Hardware interface auto-loaded
- ✅ All controllers spawned
- ✅ Navigation optional
- ✅ EKF localization included

### Option 2: Simple Hardware Launch

Uses existing controller.launch.py with hardware flag:

```bash
ros2 launch smrr_description simple_hardware.launch.py
```

**Features:**
- ✅ Reuses your existing controller setup
- ✅ Minimal configuration
- ⚠️ You may need to manually start navigation

### Option 3: Manual Launch (For Debugging)

Step-by-step manual launch:

```bash
# Terminal 1: Start controller manager with hardware interface
ros2 run controller_manager ros2_control_node \
    --ros-args \
    --params-file ~/navigation_ws/src/smrr_controller/config/arm_controller.yaml \
    -p use_sim_time:=false

# Terminal 2: Spawn controllers
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner arm_controller  
ros2 run controller_manager spawner diff_drive_controller

# Terminal 3: Start robot state publisher
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(xacro ~/navigation_ws/src/smrr_description/urdf/test.urdf.xacro is_sim:=false)"
```

## Configuration Files

### Key Files for Hardware Mode

1. **ros2_control URDF** (`src/smrr_description/urdf/smrr_ros2_control.urdf.xacro`):
   ```xml
   <xacro:unless value="$(arg is_sim)">
       <hardware>
           <plugin>smrr_base_controller/BaseController</plugin>
           <param name="port">/dev/ttyUSB0</param>
       </hardware>
   </xacro:unless>
   ```

2. **Controller Config** (`src/smrr_controller/config/arm_controller.yaml`):
   - Same file used for both sim and hardware
   - Controllers auto-detect hardware vs simulation

3. **Hardware Interface** (`src/smrr_base_controller/src/base_controller.cpp`):
   - Implements ros2_control hardware interface
   - Communicates with microcontroller via serial

## Differences: Simulation vs Hardware

| Aspect | Simulation | Hardware |
|--------|-----------|----------|
| **is_sim** | true | false |
| **use_sim_time** | true | false |
| **Hardware Plugin** | gazebo_ros2_control/GazeboSystem | smrr_base_controller/BaseController |
| **Physics** | Gazebo | Real world |
| **Serial Port** | N/A | /dev/ttyUSB0 |
| **Clock** | /clock topic | System time |

## Troubleshooting

### Issue: Controllers fail to load

**Check hardware interface is visible:**
```bash
ros2 control list_hardware_interfaces
```

**Expected output:**
```
command interfaces:
  shoulder_r_joint/position [available] [claimed]
  left_wheel_joint/velocity [available] [claimed]
  ...
state interfaces:
  shoulder_r_joint/position
  left_wheel_joint/position
  ...
```

### Issue: Serial port permission denied

```bash
# Fix permissions
sudo chmod 666 /dev/ttyUSB0

# OR add to group permanently
sudo usermod -a -G dialout $USER
# Then logout/login or reboot
```

### Issue: Hardware interface not found

```bash
# Rebuild and source
cd ~/navigation_ws
colcon build --packages-select smrr_base_controller --cmake-clean-cache
source install/setup.bash

# Verify plugin
ros2 pkg prefix smrr_base_controller
```

### Issue: Wrong clock (use_sim_time mismatch)

All nodes must have `use_sim_time:=false` for hardware. Check each node's parameters:

```bash
ros2 param get /robot_state_publisher use_sim_time
# Should return: Boolean value is: False
```

## Testing Sequence

### 1. Test Hardware Interface Only

```bash
# Start just the controller manager
ros2 run controller_manager ros2_control_node \
    --ros-args -p use_sim_time:=false

# In another terminal, check hardware
ros2 control list_hardware_interfaces
```

### 2. Test Individual Controllers

```bash
# After controller_manager is running:
ros2 run controller_manager spawner joint_state_broadcaster

# Check joint states
ros2 topic echo /joint_states
```

### 3. Test Movement

```bash
# After diff_drive_controller is spawned:
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"
```

## Next Steps

1. **Build the workspace:**
   ```bash
   cd ~/navigation_ws
   colcon build
   source install/setup.bash
   ```

2. **Test hardware launch:**
   ```bash
   ros2 launch smrr_description hardware_robot.launch.py
   ```

3. **Monitor topics:**
   ```bash
   # In separate terminals:
   ros2 topic echo /joint_states
   ros2 topic echo /diff_drive_controller/odom
   ```

4. **Add teleoperation:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
   ```

## Summary

**To run on real hardware:**
```bash
ros2 launch smrr_description hardware_robot.launch.py
```

This automatically:
- ✅ Sets is_sim:=false
- ✅ Loads hardware interface from smrr_base_controller  
- ✅ Uses real-time clock
- ✅ Connects to /dev/ttyUSB0
- ✅ Spawns all controllers
- ✅ Ready for navigation

The key difference from Gazebo is just the launch command - everything else is handled by the `is_sim` flag!
