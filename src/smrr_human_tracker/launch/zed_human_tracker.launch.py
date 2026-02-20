#!/usr/bin/env python3
"""
ZED Camera + Full Human Detection Pipeline Launch File (Hardware)

Brings up the complete human detection and fusion stack for real hardware:
1. ZED2 front camera
2. Static TF bridge (camera_link -> zed2_camera_link)
3. YOLO26 vision tracker (subscribes to ZED topics)
4. DR-SPAAM LiDAR detector (subscribes to /scan)
5. Kalman Filter fusion node (fuses YOLO + LiDAR detections)

Usage:
    ros2 launch smrr_human_tracker zed_human_tracker.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Get package directories
    pkg_share = get_package_share_directory('smrr_human_tracker')
    zed_wrapper_share = get_package_share_directory('zed_wrapper')

    # Paths
    zed_camera_launch = os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
    zed_override_config = os.path.join(pkg_share, 'config', 'zed_override.yaml')
    default_yolo_model = os.path.join(pkg_share, 'models', 'yolo26n-seg.pt')
    default_drspaam_model = os.path.join(pkg_share, 'models', 'ckpt_jrdb_ann_ft_dr_spaam_e20.pth')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        # ==================== Common ====================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (true for Gazebo, false for hardware)'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Global/odometry reference frame'
        ),

        # ==================== ZED Camera ====================
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2',
            description='ZED camera model'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed2',
            description='Camera name / topic namespace'
        ),

        # ==================== YOLO Vision Tracker ====================
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value=default_yolo_model,
            description='Path to YOLO26 segmentation model'
        ),
        DeclareLaunchArgument(
            'yolo_confidence_threshold',
            default_value='0.5',
            description='YOLO detection confidence threshold'
        ),
        DeclareLaunchArgument(
            'max_detection_distance',
            default_value='15.0',
            description='Maximum detection distance in meters'
        ),
        DeclareLaunchArgument(
            'tracker_type',
            default_value='bytetrack.yaml',
            description='YOLO tracker config (bytetrack.yaml or botsort.yaml)'
        ),
        DeclareLaunchArgument(
            'iou_threshold',
            default_value='0.5',
            description='IOU threshold for tracking'
        ),

        # ==================== DR-SPAAM LiDAR Detector ====================
        DeclareLaunchArgument(
            'drspaam_model_path',
            default_value=default_drspaam_model,
            description='Path to DR-SPAAM model checkpoint'
        ),
        DeclareLaunchArgument(
            'lidar_conf_thresh',
            default_value='0.95',
            description='DR-SPAAM detection confidence threshold (higher = fewer false positives)'
        ),
        DeclareLaunchArgument(
            'min_detection_range',
            default_value='0.5',
            description='Minimum range for person detections in metres'
        ),
        DeclareLaunchArgument(
            'max_detection_range',
            default_value='5.0',
            description='Maximum range for person detections in metres'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Input laser scan topic'
        ),
        DeclareLaunchArgument(
            'detector_model',
            default_value='DR-SPAAM',
            description='Detector model type (DR-SPAAM or DROW3)'
        ),
        DeclareLaunchArgument(
            'stride',
            default_value='1',
            description='Stride for scan downsampling'
        ),
        DeclareLaunchArgument(
            'panoramic_scan',
            default_value='True',
            description='True for 360-degree scans'
        ),

        # ==================== KF Fusion Node ====================
        DeclareLaunchArgument(
            'camera_fov_degrees',
            default_value='110.0',
            description='Camera horizontal FOV in degrees'
        ),
        DeclareLaunchArgument(
            'fusion_distance_threshold',
            default_value='1.0',
            description='Max distance (m) to match YOLO and LiDAR detections'
        ),
        DeclareLaunchArgument(
            'track_timeout_sec',
            default_value='1.0',
            description='Time (sec) before removing lost tracks'
        ),
        DeclareLaunchArgument(
            'max_track_distance',
            default_value='2.0',
            description='Max distance (m) to associate tracks between frames'
        ),
        DeclareLaunchArgument(
            'process_noise_pos',
            default_value='0.1',
            description='KF process noise for position (m²)'
        ),
        DeclareLaunchArgument(
            'process_noise_vel',
            default_value='0.5',
            description='KF process noise for velocity (m²/s²)'
        ),
        DeclareLaunchArgument(
            'measurement_noise',
            default_value='0.3',
            description='KF measurement noise (m²)'
        ),

        # ---------------------------------------------------------
        # ZED Camera Launch
        # ---------------------------------------------------------
        # pos_tracking enabled internally (required for ZED SDK grab to work)
        # publish_tf disabled to avoid TF conflict with robot's odom frame
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_camera_launch),
            launch_arguments={
                'camera_model': LaunchConfiguration('camera_model'),
                'camera_name': LaunchConfiguration('camera_name'),
                'publish_tf': 'false',
                'publish_map_tf': 'false',
                'publish_urdf': 'true',
                'use_sim_time': use_sim_time,
                'ros_params_override_path': zed_override_config,
            }.items()
        ),

        # ---------------------------------------------------------
        # Static Transform Bridge
        # ---------------------------------------------------------
        # camera_link  (robot URDF body frame)
        #   -> zed2_camera_link  (ZED SDK body frame, camera_name=zed2)
        # Mirrors the rear-camera pattern: rear_camera_link -> rear_camera_camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_bridge',
            output='screen',
            arguments=[
                '--frame-id', 'camera_link',
                '--child-frame-id', 'zed2_camera_link',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0'
            ]
        ),

        # ---------------------------------------------------------
        # Node 1: YOLO Vision Tracker
        # ---------------------------------------------------------
        Node(
            package='smrr_human_tracker',
            executable='human_tracker',
            name='human_tracker',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_path': LaunchConfiguration('yolo_model_path'),
                'confidence_threshold': LaunchConfiguration('yolo_confidence_threshold'),
                'rgb_topic': '/zed2/zed_node/rgb/color/rect/image',
                'depth_topic': '/zed2/zed_node/depth/depth_registered',
                'camera_info_topic': '/zed2/zed_node/rgb/color/rect/camera_info',
                'publish_visualization': True,
                'max_detection_distance': LaunchConfiguration('max_detection_distance'),
                'tracker': LaunchConfiguration('tracker_type'),
                'iou_threshold': LaunchConfiguration('iou_threshold'),
                'target_frame': LaunchConfiguration('map_frame'),
            }]
        ),

        # ---------------------------------------------------------
        # Node 2: DR-SPAAM LiDAR Detector
        # ---------------------------------------------------------
        Node(
            package='smrr_human_tracker',
            executable='human_lidar_matcher',
            name='dr_spaam_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'weight_file': LaunchConfiguration('drspaam_model_path'),
                'detector_model': LaunchConfiguration('detector_model'),
                'conf_thresh': LaunchConfiguration('lidar_conf_thresh'),
                'stride': LaunchConfiguration('stride'),
                'panoramic_scan': LaunchConfiguration('panoramic_scan'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'target_frame': LaunchConfiguration('map_frame'),
                'detections_topic': 'detected_people',
                'marker_topic': 'detected_people_markers',
                'min_detection_range': LaunchConfiguration('min_detection_range'),
                'max_detection_range': LaunchConfiguration('max_detection_range'),
            }]
        ),

        # ---------------------------------------------------------
        # Node 3: Kalman Filter Fusion
        # ---------------------------------------------------------
        Node(
            package='smrr_human_tracker',
            executable='human_fusion_kf',
            name='human_fusion_kf',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_fov_degrees': LaunchConfiguration('camera_fov_degrees'),
                'fusion_distance_threshold': LaunchConfiguration('fusion_distance_threshold'),
                'track_timeout_sec': LaunchConfiguration('track_timeout_sec'),
                'max_track_distance': LaunchConfiguration('max_track_distance'),
                'base_frame': LaunchConfiguration('base_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
                'process_noise_pos': LaunchConfiguration('process_noise_pos'),
                'process_noise_vel': LaunchConfiguration('process_noise_vel'),
                'measurement_noise': LaunchConfiguration('measurement_noise'),
            }]
        ),
    ])
