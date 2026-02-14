from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
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
        matching_threshold_arg,
        cluster_distance_arg,
        min_cluster_points_arg,
        matcher_node,
    ])
