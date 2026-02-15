from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get model path from smrr_crowdnav package
    package_share_dir = get_package_share_directory('smrr_crowdnav')
    default_weight_file = os.path.join(package_share_dir, 'models', 'ckpt_jrdb_ann_ft_dr_spaam_e20.pth')
    
    # Declare launch arguments
    weight_file_arg = DeclareLaunchArgument(
        'weight_file',
        default_value=default_weight_file,
        description='Path to pre-trained DR-SPAAM model checkpoint (.pth file)'
    )
    
    detector_model_arg = DeclareLaunchArgument(
        'detector_model',
        default_value='DR-SPAAM',
        description='Detector model: DR-SPAAM or DROW3'
    )
    
    conf_thresh_arg = DeclareLaunchArgument(
        'conf_thresh',
        default_value='0.5',
        description='Detection confidence threshold (0.0-1.0)'
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
    
    matching_threshold_arg = DeclareLaunchArgument(
        'matching_distance_threshold',
        default_value='0.5',
        description='Maximum distance (m) between camera and LiDAR detection for matching'
    )
    
    cluster_distance_arg = DeclareLaunchArgument(
        'cluster_distance_threshold',
        default_value='0.3',
        description='Maximum distance (m) between points to form a cluster'
    )
    
    min_cluster_points_arg = DeclareLaunchArgument(
        'min_cluster_points',
        default_value='3',
        description='Minimum points required to form a cluster'
    )

    # Human-LiDAR matcher node
    matcher_node = Node(
        package='smrr_crowdnav',
        executable='human_lidar_matcher',
        name='human_lidar_matcher',
        output='screen',
        parameters=[{
            'weight_file': LaunchConfiguration('weight_file'),
            'detector_model': LaunchConfiguration('detector_model'),
            'conf_thresh': LaunchConfiguration('conf_thresh'),
            'stride': LaunchConfiguration('stride'),
            'panoramic_scan': LaunchConfiguration('panoramic_scan'),
            'scan_topic': '/scan',
            'human_poses_topic': '/detected_humans/poses',
            'matching_distance_threshold': LaunchConfiguration('matching_distance_threshold'),
            'cluster_distance_threshold': LaunchConfiguration('cluster_distance_threshold'),
            'min_cluster_points': LaunchConfiguration('min_cluster_points'),
            'human_leg_width_min': 0.1,
            'human_leg_width_max': 0.6,
            'min_range': 0.2,
            'max_range': 10.0,
            'publish_rate': 10.0,
        }]
    )

    return LaunchDescription([
        weight_file_arg,
        detector_model_arg,
        conf_thresh_arg,
        stride_arg,
        panoramic_scan_arg,
        matching_threshold_arg,
        cluster_distance_arg,
        min_cluster_points_arg,
        matcher_node,
    ])
