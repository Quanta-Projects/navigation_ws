#!/usr/bin/env python3
"""
Hardware Docking Bringup Launch File

This launch file brings up all components needed for docking on real hardware:
1. ZED2 rear camera (via zed_camera.launch.py with TF publishing disabled)
2. Static transform bridge from URDF frame to ZED frame
3. AprilTag detection and docking pipeline (via detections.launch.py)

Usage:
    ros2 launch smrr_docking docking_bringup_hardware.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get package directories
    docking_pkg_share = get_package_share_directory('smrr_docking')
    zed_wrapper_share = get_package_share_directory('zed_wrapper')
    
    # Path to launch files
    detections_launch = os.path.join(docking_pkg_share, 'launch', 'detections.launch.py')
    docking_bringup_launch = os.path.join(docking_pkg_share, 'launch', 'docking_bringup.launch.py')
    zed_camera_launch = os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
    
    return LaunchDescription([
        
        # ---------------------------------------------------------
        # ZED2 Rear Camera Launch
        # ---------------------------------------------------------
        # Launches ZED camera with TF publishing disabled
        # Camera outputs will be namespaced under /rear_camera/
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_camera_launch),
            launch_arguments={
                'camera_model': 'zed2',
                'camera_name': 'rear_camera',
                'publish_tf': 'false',           # Disable odom->camera_link TF
                'publish_map_tf': 'false',       # Disable map->odom TF
                'publish_urdf': 'true',          # Use robot's URDF instead
                'use_sim_time': 'false'          # Real hardware
            }.items()
        ),
        
        # ---------------------------------------------------------
        # Static Transform Bridge
        # ---------------------------------------------------------
        # Bridges rear_camera_link (from robot URDF) to 
        # rear_camera_camera_link (expected by ZED)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_camera_link_bridge',
            output='screen',
            arguments=[
                '--frame-id', 'rear_camera_link', 
                '--child-frame-id', 'rear_camera_camera_link',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0'
            ]
        ),
        
        # ---------------------------------------------------------
        # AprilTag Detection & Docking Pipeline
        # ---------------------------------------------------------
        # Includes detections.launch.py which runs:
        # - AprilTag detection node
        # - Static transform (tag -> dock)
        # - TF to Pose bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(detections_launch),
            launch_arguments={
                'camera_topic': '/rear_camera/zed_node/rgb/color/rect/image',
                'info_topic': '/rear_camera/zed_node/rgb/color/rect/camera_info'
            }.items()
        ),
        
        # ---------------------------------------------------------
        # Docking Server & Lifecycle Manager
        # ---------------------------------------------------------
        # Includes docking_bringup.launch.py which runs:
        # - OpenNav docking server
        # - Lifecycle manager for docking
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(docking_bringup_launch),
            launch_arguments={
                'use_sim_time': 'false'
            }.items()
        )
    ])
