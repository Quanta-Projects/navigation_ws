#!/bin/bash
# Script to launch Gazebo with clean state

echo "Cleaning Gazebo state..."
# Kill any existing Gazebo processes
killall -9 gzserver gzclient gazebo 2>/dev/null || true

# Clear Gazebo logs and locks
rm -rf ~/.gazebo/log/* ~/.gazebo/.lock 2>/dev/null || true

# Wait a moment for processes to fully terminate
sleep 2

echo "Launching Gazebo with fresh state..."
# Source workspace and launch
cd ~/navigation_ws
source install/setup.bash
ros2 launch smrr_description gazebo_classic.launch.xml
