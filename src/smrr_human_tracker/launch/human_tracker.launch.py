from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch YOLO26 instance segmentation with tracking
    """
    
    # Get model path from package
    pkg_share = get_package_share_directory('smrr_human_tracker')
    # Use TensorRT FP16 engine by default for full GPU acceleration on Orin.
    # Fall back to yolo26n-seg.pt only if the engine hasn't been exported yet.
    default_model = os.path.join(pkg_share, 'models', 'yolo26n-seg.engine')
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model,
        description='Path to YOLO26 segmentation model file'
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
        default_value='15.0',
        description='Maximum detection distance in meters'
    )
    
    tracker_arg = DeclareLaunchArgument(
        'tracker',
        default_value='bytetrack.yaml',
        description='Tracker type (botsort.yaml or bytetrack.yaml)'
    )
    
    iou_threshold_arg = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.3',
        description='IOU threshold for tracking'
    )

    publish_viz_arg = DeclareLaunchArgument(
        'publish_visualization',
        default_value='true',
        description='Publish debug visualization image (adds CPU/memory overhead)'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='odom',
        description='TF target frame for published human poses (map/odom/base_link)'
    )

    # Human tracker node
    tracker_node = Node(
        package='smrr_human_tracker',
        executable='human_tracker',
        name='human_tracker',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'publish_visualization': LaunchConfiguration('publish_visualization'),
            'max_detection_distance': LaunchConfiguration('max_detection_distance'),
            'tracker': LaunchConfiguration('tracker'),
            'iou_threshold': LaunchConfiguration('iou_threshold'),
            'target_frame': LaunchConfiguration('target_frame'),
        }]
    )

    return LaunchDescription([
        model_path_arg,
        confidence_threshold_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        max_distance_arg,
        tracker_arg,
        iou_threshold_arg,
        publish_viz_arg,
        target_frame_arg,
        tracker_node,
    ])
