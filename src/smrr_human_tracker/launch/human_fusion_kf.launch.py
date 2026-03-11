#!/usr/bin/env python3
"""
Launch file for Kalman Filter-based Human Detection System

Launches all three detection nodes with KF-based fusion:
- YOLO26 vision tracker
- DR-SPAAM LiDAR detector  
- KF fusion node with optimal state estimation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories and default model paths
    pkg_share = get_package_share_directory('smrr_human_tracker')
    default_yolo_model = os.path.join(pkg_share, 'models', 'yolo26n-seg.pt')
    default_drspaam_model = os.path.join(pkg_share, 'models', 'ckpt_jrdb_ann_ft_dr_spaam_e20.pth')
    
    # Simulation time — must be 'true' when running in Gazebo so that
    # all nodes (and TF lookups / KF dt calculations) use /clock.
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # ==================== Simulation Time ====================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock for all nodes'
        ),
        
        # ==================== YOLO Vision Tracker Arguments ====================
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
            'rgb_topic',
            default_value='/zed2_left_camera/image_raw',
            description='RGB image topic'
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/zed2_left_camera/depth/image_raw',
            description='Depth image topic'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/zed2_left_camera/camera_info',
            description='Camera info topic'
        ),
        DeclareLaunchArgument(
            'max_detection_distance',
            default_value='15.0',
            description='Maximum detection distance in meters'
        ),
        DeclareLaunchArgument(
            'tracker_type',
            default_value='bytetrack.yaml',
            description='YOLO tracker type (bytetrack.yaml or botsort.yaml)'
        ),
        DeclareLaunchArgument(
            'iou_threshold',
            default_value='0.5',
            description='IOU threshold for tracking'
        ),
        
        # ==================== DR-SPAAM LiDAR Detector Arguments ====================
        DeclareLaunchArgument(
            'drspaam_model_path',
            default_value=default_drspaam_model,
            description='Path to DR-SPAAM model checkpoint'
        ),
        DeclareLaunchArgument(
            'lidar_conf_thresh',
            default_value='0.3',
            description='DR-SPAAM detection confidence threshold'
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
        
        # ==================== KF Fusion Node Arguments ====================
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
        
        # ==================== Common Arguments ====================
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Global reference frame'
        ),
        
        # ==================== Node 1: YOLO Vision Tracker ====================
        Node(
            package='smrr_human_tracker',
            executable='human_tracker',
            name='human_tracker',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_path': LaunchConfiguration('yolo_model_path'),
                'confidence_threshold': LaunchConfiguration('yolo_confidence_threshold'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'publish_visualization': True,
                'max_detection_distance': LaunchConfiguration('max_detection_distance'),
                'tracker': LaunchConfiguration('tracker_type'),
                'iou_threshold': LaunchConfiguration('iou_threshold'),
                'target_frame': LaunchConfiguration('map_frame'),
            }]
        ),
        
        # ==================== Node 2: DR-SPAAM LiDAR Detector ====================
        Node(
            package='smrr_human_tracker',
            executable='lidar_human_detection',
            name='lidar_human_detection_node',
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
                'use_onnx': True,
            }]
        ),
        
        # ==================== Node 3: Kalman Filter Fusion ====================
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
