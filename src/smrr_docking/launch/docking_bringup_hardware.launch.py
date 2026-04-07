#!/usr/bin/env python3
"""
Hardware Docking Bringup Launch File

This launch file brings up all components needed for docking on real hardware:
1. Image rectification for RealSense D435 color camera
2. Static transform bridge from URDF frame to RealSense camera_link
3. AprilTag detection and docking pipeline (via detections.launch.py)

Note: RealSense D435 camera must be launched separately before this file:
    ros2 launch realsense2_camera rs_launch.py \
        depth_module.profile:=640x480x90 \
        rgb_camera.profile:=640x480x60

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
    
    # Path to launch files
    detections_launch = os.path.join(docking_pkg_share, 'launch', 'detections.launch.py')
    docking_bringup_launch = os.path.join(docking_pkg_share, 'launch', 'docking_bringup.launch.py')

    # RealSense D435 source topics
    realsense_image_raw  = '/camera/camera/color/image_raw'
    realsense_info_topic = '/camera/camera/color/camera_info'
    realsense_image_rect = '/camera/camera/color/image_rect'
    
    return LaunchDescription([

        # ---------------------------------------------------------
        # Image Rectification for RealSense D435
        # ---------------------------------------------------------
        # The ZED2 provided pre-rectified images directly; the RealSense
        # D435 publishes raw images that must be rectified before being
        # fed to the AprilTag detector.
        Node(
            package='image_proc',
            executable='rectify_node',
            name='realsense_color_rectify',
            remappings=[
                ('image',       realsense_image_raw),
                ('camera_info', realsense_info_topic),
                ('image_rect',  realsense_image_rect),
            ],
            output='screen'
        ),
        
        # ---------------------------------------------------------
        # Static Transform Bridge
        # ---------------------------------------------------------
        # Bridges rear_camera_link (from robot URDF) to
        # camera_link (RealSense D435 base frame).
        # The RealSense driver publishes the rest of its own TF tree
        # (camera_link -> camera_color_frame -> camera_color_optical_frame).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_camera_link_bridge',
            output='screen',
            arguments=[
                '--frame-id', 'rear_camera_link', 
                '--child-frame-id', 'camera_link',
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
                'camera_topic': realsense_image_rect,
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
