#!/usr/bin/env python3
"""
Hardware Docking Bringup Launch File

This launch file brings up all components needed for docking on real hardware:
1. RealSense D435 camera driver
2. Image rectification for RealSense D435 color camera
3. Static transform bridge from URDF frame to RealSense camera_link
4. AprilTag detection and docking pipeline (via detections.launch.py)

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
    docking_pkg_share    = get_package_share_directory('smrr_docking')
    realsense_pkg_share  = get_package_share_directory('realsense2_camera')

    # Path to launch files
    realsense_launch     = os.path.join(realsense_pkg_share, 'launch', 'rs_launch.py')
    detections_launch    = os.path.join(docking_pkg_share, 'launch', 'detections.launch.py')
    docking_bringup_launch = os.path.join(docking_pkg_share, 'launch', 'docking_bringup.launch.py')

    # RealSense D435 source topics
    # camera_name='rear_cam' gives namespace /camera/rear_cam/
    realsense_image_raw  = '/camera/rear_cam/color/image_raw'
    realsense_info_topic = '/camera/rear_cam/color/camera_info'
    
    return LaunchDescription([

        # ---------------------------------------------------------
        # RealSense D435 Camera Driver
        # ---------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'camera_name':                    'rear_cam',
                'depth_module.profile':           '640x480x30',
                'rgb_camera.profile':             '640x480x30',
                'enable_sync':                    'true',
                'rgb_camera.power_line_frequency': '1',
            }.items()
        ),

        # ---------------------------------------------------------
        # Static Transform Bridge
        # ---------------------------------------------------------
        # Bridges rear_camera_link (from robot URDF) to
        # rear_cam_link (RealSense D435 base frame, camera_name='rear_cam').
        # The RealSense driver publishes the rest of its own TF tree
        # (rear_cam_link -> rear_cam_color_frame -> rear_cam_color_optical_frame).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_camera_link_bridge',
            output='screen',
            arguments=[
                '--frame-id', 'rear_camera_link', 
                '--child-frame-id', 'rear_cam_link',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0.2094', '--yaw', '0.0174'
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
                'camera_topic': realsense_image_raw,
                'info_topic':   realsense_info_topic
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
