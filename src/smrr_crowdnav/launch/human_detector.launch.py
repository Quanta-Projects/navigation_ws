from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('smrr_crowdnav')
    default_model = os.path.join(pkg_dir, 'models', 'yolo26n.pt')
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model,
        description='Path to YOLO26 model file'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for detections'
    )
    
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/zed2_left_camera/image_raw',
        description='RGB image topic'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/zed2_left_camera/depth/image_raw',
        description='Depth image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/zed2_left_camera/camera_info',
        description='Camera info topic'
    )
    
    max_distance_arg = DeclareLaunchArgument(
        'max_detection_distance',
        default_value='10.0',
        description='Maximum detection distance in meters'
    )
    
    tracking_threshold_arg = DeclareLaunchArgument(
        'tracking_distance_threshold',
        default_value='1.0',
        description='Max distance to match same person instance (m)'
    )

    # Human detector node
    human_detector_node = Node(
        package='smrr_crowdnav',
        executable='human_detector',
        name='human_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'publish_visualization': True,
            'max_detection_distance': LaunchConfiguration('max_detection_distance'),
            'tracking_distance_threshold': LaunchConfiguration('tracking_distance_threshold'),
        }]
    )

    return LaunchDescription([
        model_path_arg,
        confidence_threshold_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        max_distance_arg,
        tracking_threshold_arg,
        human_detector_node,
    ])
