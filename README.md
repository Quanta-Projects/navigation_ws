# SMRR Robot Navigation Workspace

## Overview
This ROS 2 workspace contains the complete navigation system for the SMRR (Service Mobile Robot for Research) robot in an elevator simulation environment. The system integrates Gazebo simulation, Nav2 navigation stack, AMCL localization, and autonomous navigation capabilities.

## System Requirements
- **ROS 2 Humble** (tested and verified)
- **Ubuntu 22.04** LTS
- **Gazebo Classic** 11
- **Nav2** Navigation Stack
- **GPU**: NVIDIA GPU recommended for Gazebo simulation

## Package Structure
```
navigation_ws/
├── src/
│   ├── elevator/                    # Elevator control functionality
│   ├── gazebo_elevator_plugin/      # Elevator simulation plugin and world files
│   ├── smrr_controller/             # Robot controller with joystick teleop
│   ├── smrr_description/            # Robot URDF/XACRO descriptions and launch files
│   └── smrr_navigation/             # Navigation stack with Nav2 integration
└── README.md                        # This file
```

## Installation & Build

### 1. Clone and Setup Workspace
```bash
# Navigate to your workspace
cd ~/navigation_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 2. Install Additional Dependencies
```bash
# Install Nav2 and Gazebo packages
sudo apt update
sudo apt install ros-humble-nav2-bringup ros-humble-nav2-lifecycle-manager \
                 ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
                 ros-humble-xacro ros-humble-joy-teleop
```

## Quick Start Guide

### Complete Navigation System 
Launch the complete system with Gazebo simulation, robot, navigation stack, and RViz:

```bash
# Source the workspace
source install/setup.bash

# Launch complete navigation system
ros2 launch smrr_navigation smrr_world_navigation.launch.py
```

## Navigation Usage

### 1. **Set Initial Pose**
- In RViz, click the **"2D Pose Estimate"** tool
- Click and drag on the map to set the robot's initial position and orientation
- This initializes AMCL localization

### 2. **Navigate to Goal**
- Click the **"2D Nav Goal"** tool in RViz
- Click and drag on the map to set the destination
- The robot will automatically plan and execute the path

### 3. **Monitor Navigation**
- **RViz**: Real-time visualization of robot, map, paths, and sensor data
- **Terminal**: Navigation status and diagnostic messages
- **Gazebo**: 3D simulation environment

## Advanced Usage

### Available Maps
The workspace includes multiple floor maps:
- `ground_floor.yaml` (default)
- `second_floor.yaml`
- `third_floor.yaml`
- `fourth_floor.yaml`

## Robot Specifications

### SMRR Robot Features
- **Base**: Differential drive with wheel separation: 0.402m, diameter: 0.162m
- **Sensors**: 
  - RPLidar (360° laser scanner)
  - ZED2 Stereo Camera
  - IMU (Inertial Measurement Unit)
- **Frames**: `base_footprint` → `base_link` → sensor frames
- **Topics**: 
  - `/scan` - Laser scan data
  - `/cmd_vel` - Velocity commands
  - `/odom` - Odometry data

### Navigation Configuration
- **Localization**: AMCL (Adaptive Monte Carlo Localization)
- **Global Planner**: NavFn Planner
- **Local Planner**: DWB Local Planner
- **Costmaps**: Global and local costmaps with obstacle detection
- **Recovery**: Automatic recovery behaviors for stuck situations

## Troubleshooting

### Common Issues

#### 1. **Robot Not Localizing**
```bash
# Check AMCL status
ros2 topic echo /amcl_pose

# Solution: Set initial pose in RViz using "2D Pose Estimate"
```

#### 2. **No Map Displayed**
```bash
# Check map server
ros2 topic echo /map --once

# Check map file path in launch files
```

#### 3. **Transform Errors**
```bash
# Check transform tree
ros2 run tf2_tools view_frames

# Ensure consistent frame names (base_footprint) across all config files
```

#### 4. **Navigation Not Working**
```bash
# Check if all nodes are running
ros2 node list

# Check navigation status
ros2 topic echo /behavior_tree_log
```

### Log Analysis
```bash
# View detailed logs
ros2 launch smrr_navigation navigation_launch.py --ros-args --log-level debug

# Check specific node logs
ros2 run rqt_console rqt_console
```

## Development

### Modifying Configuration
- **Navigation parameters**: `src/smrr_navigation/config/nav2_params.yaml`
- **Robot description**: `src/smrr_description/urdf/test.urdf.xacro`
- **Launch configurations**: `src/smrr_navigation/launch/`

### Adding New Maps
1. Place `.yaml` and `.pgm` files in `src/smrr_navigation/maps/`
2. Update launch files to reference new map
3. Rebuild workspace: `colcon build --symlink-install`

### Custom Behavior Trees
Navigation behavior can be customized using Nav2 behavior trees in `config/` directory.

## Performance Tips

### Optimization
- **Hardware**: Use NVIDIA GPU for Gazebo simulation
- **Parameters**: Tune navigation parameters in `smrr_nav_params.yaml`
- **Map Resolution**: Use appropriate map resolution (0.05m recommended)
- **Sensor Rate**: Adjust laser scan frequency for performance

### Resource Usage
- **CPU**: Multi-core recommended for real-time performance
- **Memory**: 8GB+ RAM recommended
- **GPU**: NVIDIA GPU with OpenGL 4.0+ support

## Support & Contributing

### Getting Help
- Check the troubleshooting section above
- Review ROS 2 Nav2 documentation
- Examine log outputs for specific error messages

### Contributing
1. Fork the repository
2. Create a feature branch
3. Test thoroughly with the navigation system
4. Submit a pull request

## License
This workspace is provided for research and educational purposes.

## Authors
- **Maintainers**: Nadil Gunawardane, Achira Hansindu
- **Institution**: University of Moratuwa

---

**Last Updated**: September 2025
**ROS 2 Version**: Humble
**Tested On**: Ubuntu 22.04 LTS
