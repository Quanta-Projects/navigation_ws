from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch DR-SPAAM 2D LiDAR Person Detector
    """
    
    # Get model path from smrr_human_tracker package
    package_share_dir = get_package_share_directory('smrr_human_tracker')
    default_weight_file = os.path.join(package_share_dir, 'models', 'ckpt_jrdb_ann_ft_dr_spaam_e20.pth')
    
    # Declare launch arguments
    weight_file_arg = DeclareLaunchArgument(
        'weight_file',
        default_value=default_weight_file,
        description='Path to pre-trained model checkpoint (.pth file)'
    )
    
    detector_model_arg = DeclareLaunchArgument(
        'detector_model',
        default_value='DR-SPAAM',
        description='Detector model: DR-SPAAM or DROW3'
    )
    
    conf_thresh_arg = DeclareLaunchArgument(
        'conf_thresh',
        default_value='0.95',
        description='Detection confidence threshold (0.0-1.0), higher = fewer false positives'
    )
    
    stride_arg = DeclareLaunchArgument(
        'stride',
        default_value='1',
        description='Stride for downsampling scan (1=no downsampling)'
    )
    
    panoramic_scan_arg = DeclareLaunchArgument(
        'panoramic_scan',
        default_value='True',
        description='Set to True for 360-degree scans'
    )
    
    min_range_arg = DeclareLaunchArgument(
        'min_detection_range',
        default_value='0.5',
        description='Minimum range for person detections in meters'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_detection_range',
        default_value='5.0',
        description='Maximum range for person detections in meters'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Input laser scan topic'
    )
    
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='odom',
        description='Target frame for detections (map/odom/base_link)'
    )
    
    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='detected_people',
        description='Output detections topic (PoseArray)'
    )
    
    marker_topic_arg = DeclareLaunchArgument(
        'marker_topic',
        default_value='detected_people_markers',
        description='Output marker topic for RViz visualization'
    )

    # DR-SPAAM detector node
    detector_node = Node(
        package='smrr_human_tracker',
        executable='human_lidar_matcher',
        name='dr_spaam_detector',
        output='screen',
        parameters=[{
            'weight_file': LaunchConfiguration('weight_file'),
            'detector_model': LaunchConfiguration('detector_model'),
            'conf_thresh': LaunchConfiguration('conf_thresh'),
            'stride': LaunchConfiguration('stride'),
            'panoramic_scan': LaunchConfiguration('panoramic_scan'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'target_frame': LaunchConfiguration('target_frame'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'marker_topic': LaunchConfiguration('marker_topic'),
            'min_detection_range': LaunchConfiguration('min_detection_range'),
            'max_detection_range': LaunchConfiguration('max_detection_range'),
        }]
    )

    return LaunchDescription([
        weight_file_arg,
        detector_model_arg,
        conf_thresh_arg,
        stride_arg,
        panoramic_scan_arg,
        min_range_arg,
        max_range_arg,
        scan_topic_arg,
        target_frame_arg,
        detections_topic_arg,
        marker_topic_arg,
        detector_node,
    ])
