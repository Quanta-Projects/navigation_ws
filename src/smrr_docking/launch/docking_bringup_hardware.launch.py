#!/usr/bin/env python3
"""
Hardware Docking Bringup Launch File
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ---------------------------------------------------------
# Bulletproof Python QoS Relay Script
# Manually subscribes RELIABLE and publishes BEST_EFFORT
# ---------------------------------------------------------
qos_relay_script = """
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy

def main():
    rclpy.init()
    node = Node('custom_qos_relay')
    
    # Define exact QoS profiles
    best_effort_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    
    # Publisher (Best Effort)
    pub = node.create_publisher(Image, '/camera/rear_cam/color/image_raw_best_effort', best_effort_qos)
    
    # Subscriber (Reliable) -> Callbacks directly to publisher
    sub = node.create_subscription(Image, '/camera/rear_cam/color/image_raw', pub.publish, reliable_qos)
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

def generate_launch_description():
    
    docking_pkg_share    = get_package_share_directory('smrr_docking')
    realsense_pkg_share  = get_package_share_directory('realsense2_camera')

    realsense_launch     = os.path.join(realsense_pkg_share, 'launch', 'rs_launch.py')
    detections_launch    = os.path.join(docking_pkg_share, 'launch', 'detections.launch.py')
    docking_bringup_launch = os.path.join(docking_pkg_share, 'launch', 'docking_bringup.launch.py')

    realsense_image_raw  = '/camera/rear_cam/color/image_raw_best_effort'
    realsense_info_topic = '/camera/rear_cam/color/camera_info'
    
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'camera_name':                    'rear_cam',
                'depth_module.profile':           '640x480x30',
                'rgb_camera.profile':             '640x480x30',
                'enable_sync':                    'false',
                'enable_depth':                   'false',
                'rgb_camera.power_line_frequency': '1',
            }.items()
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_camera_link_bridge',
            output='screen',
            arguments=[
                '--frame-id', 'rear_camera_link', 
                '--child-frame-id', 'rear_cam_link',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0.2094', '--yaw', '0.04'
            ]
        ),

        # ---------------------------------------------------------
        # Execute the Python QoS Relay
        # ---------------------------------------------------------
        ExecuteProcess(
            cmd=['python3', '-c', qos_relay_script],
            output='screen'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(detections_launch),
            launch_arguments={
                'camera_topic': realsense_image_raw,
                'info_topic':   realsense_info_topic
            }.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(docking_bringup_launch),
            launch_arguments={
                'use_sim_time': 'false'
            }.items()
        )
    ])