from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Complete human detection + LiDAR matching pipeline
    
    Launches:
    1. Human detector (YOLO26 + RGBD)
    2. Human-LiDAR matcher (fusion)
    """
    
    pkg_dir = get_package_share_directory('smrr_human_tracker')
    default_model = os.path.join(pkg_dir, 'models', 'yolo26n.pt')
    
    # Arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model,
        description='YOLO26 model file'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='YOLO confidence threshold'
    )
    
    tracking_threshold_arg = DeclareLaunchArgument(
        'tracking_distance_threshold',
        default_value='3.0',
        description='Max distance to match same person instance (m)'
    )
    
    matching_threshold_arg = DeclareLaunchArgument(
        'matching_distance_threshold',
        default_value='1.5',
        description='Camera-LiDAR matching threshold (m)'
    )

    # Human detector node
    human_detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'human_detector.launch.py')
        ),
        launch_arguments={
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'tracking_distance_threshold': LaunchConfiguration('tracking_distance_threshold'),
        }.items()
    )

    # Human-LiDAR matcher node
    matcher_node = Node(
        package='smrr_human_tracker',
        executable='human_lidar_matcher',
        name='human_lidar_matcher',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'human_poses_topic': '/detected_humans/poses',
            'matching_distance_threshold': LaunchConfiguration('matching_distance_threshold'),
            'cluster_distance_threshold': 0.15,
            'min_cluster_points': 3,
            'human_leg_width_min': 0.05,
            'human_leg_width_max': 0.8,
            'min_range': 0.2,
            'max_range': 10.0,
            'publish_rate': 10.0,
        }]
    )

    return LaunchDescription([
        model_path_arg,
        confidence_threshold_arg,
        tracking_threshold_arg,
        matching_threshold_arg,
        human_detector_launch,
        matcher_node,
    ])
