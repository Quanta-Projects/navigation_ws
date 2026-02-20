#!/usr/bin/env python3
"""
Separate ZED Camera and Human Tracker Launch File

Launches:
1. ZED camera using standard zed_wrapper launch
2. Human tracker node with topic remapping

Usage:
    ros2 launch smrr_human_tracker zed_human_tracker_separate.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get model path from package
    human_tracker_pkg_share = get_package_share_directory('smrr_human_tracker')
    default_model = os.path.join(human_tracker_pkg_share, 'models', 'yolo26n-seg.pt')
    
    # Launch ZED camera using standard launch file
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2',
            'camera_name': 'zed2',  # Use standard name
        }.items()
    )
    
    # Launch human tracker with topic remapping
    human_tracker_node = Node(
        package='smrr_human_tracker',
        executable='human_tracker',
        name='human_tracker',
        output='screen',
        parameters=[{
            'model_path': default_model,
            'confidence_threshold': 0.5,
            'rgb_topic': '/zed2/zed_node/rgb/color/rect/image',
            'depth_topic': '/zed2/zed_node/depth/depth_registered',
            'camera_info_topic': '/zed2/zed_node/rgb/color/rect/camera_info',
            'publish_visualization': True,
            'max_detection_distance': 15.0,
            'tracker': 'bytetrack.yaml',
            'iou_threshold': 0.3,
            'target_frame': 'map'
        }]
    )
    
    return LaunchDescription([
        zed_camera_launch,
        human_tracker_node,
    ])
