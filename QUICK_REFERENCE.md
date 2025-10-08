# SMRR Robot Quick Reference

## 🚀 Launch Commands

### Simulation (Gazebo)
```bash
ros2 launch smrr_navigation smrr_world_navigation.launch.py
```

### Real Hardware (Simple)
```bash
# 1. Upload firmware.ino to Arduino via Arduino IDE
# 2. Connect Arduino to PC (e.g., /dev/ttyUSB0)
# 3. Launch:
ros2 launch smrr_description simple_hardware.launch.py
```

### Real Hardware (Full)
```bash
ros2 launch smrr_description hardware_robot.launch.py
```

### Hardware with Navigation
```bash
ros2 launch smrr_description hardware_robot.launch.py use_navigation:=true
```

## 📁 Important Files

| File | Purpose | Sim/HW |
|------|---------|--------|
| `src/smrr_description/urdf/smrr_ros2_control.urdf.xacro` | Hardware interface config | Both |
| `src/smrr_controller/config/arm_controller.yaml` | Controller parameters | Both |
| `src/smrr_base_controller/src/base_controller.cpp` | Hardware interface code | Hardware |
| `src/smrr_description/launch/gazebo_classic_controllers.launch.py` | Simulation launch | Sim |
| `src/smrr_description/launch/hardware_robot.launch.py` | Hardware launch | Hardware |

## 🔧 Key Parameters

### In URDF (test.urdf.xacro)
- `is_sim:=true` → Use Gazebo
- `is_sim:=false` → Use real hardware

### In Nodes
- `use_sim_time:=true` → Simulation
- `use_sim_time:=false` → Hardware

## 🔌 Hardware Interface Selection

Controlled in `smrr_ros2_control.urdf.xacro`:

```xml
<!-- Simulation -->
<xacro:if value="$(arg is_sim)">
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
</xacro:if>

<!-- Hardware -->
<xacro:unless value="$(arg is_sim)">
    <plugin>smrr_base_controller/BaseController</plugin>
    <param name="port">/dev/ttyUSB0</param>
</xacro:unless>
```

## 🐛 Quick Diagnostics

```bash
# Check active controllers
ros2 control list_controllers

# Check hardware interfaces
ros2 control list_hardware_interfaces

# Monitor odometry
ros2 topic echo /diff_drive_controller/odom

# Check joint states
ros2 topic echo /joint_states

# Verify serial port
ls -l /dev/ttyUSB*

# Check if controller manager is running
ros2 node list | grep controller_manager
```

## ⚡ Common Issues

| Problem | Solution |
|---------|----------|
| Permission denied on /dev/ttyUSB0 | `sudo chmod 666 /dev/ttyUSB0` |
| Hardware interface not found | Rebuild: `colcon build --packages-select smrr_base_controller` |
| Controllers won't load | Check `ros2 control list_hardware_interfaces` |
| Clock mismatch warnings | Ensure all nodes have same `use_sim_time` value |

## 📊 System Architecture

```
Simulation Mode:
  Launch → Gazebo → gazebo_ros2_control → Controllers → Topics

Hardware Mode:
  Launch → ros2_control_node → smrr_base_controller → Serial → Microcontroller
                              → Controllers → Topics
```

## 🎮 Teleoperation

```bash
# Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Joystick control (if configured)
ros2 launch smrr_controller joystick_teleop.launch.py
```
