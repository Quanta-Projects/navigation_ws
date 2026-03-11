from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def _cudnn_lib_dir():
    """Return the pip-installed nvidia-cudnn lib dir (for onnxruntime CUDA EP)."""
    try:
        import nvidia.cudnn as _cudnn
        init = getattr(_cudnn, "__file__", None) or _cudnn.__path__[0]
        candidate = os.path.join(os.path.dirname(init), "lib")
        if os.path.isdir(candidate):
            return candidate
    except Exception:
        pass
    return None


def generate_launch_description():
    """
    Launch LidarHumanDetectionNode — DR-SPAAM 2D LiDAR Person Detector
    with O(1) direct-grid-indexing static obstacle filtering (OpenCV dilation).
    Optionally uses ONNX Runtime CUDA backend for ~12 % faster inference.
    """

    package_share_dir = get_package_share_directory('smrr_human_tracker')
    default_weight_file = os.path.join(
        package_share_dir, 'models', 'ckpt_jrdb_ann_ft_dr_spaam_e20.pth'
    )

    # ── Launch arguments ────────────────────────────────────────────────────
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
        default_value='0.5',
        description='Detection confidence threshold (0.0–1.0)'
    )

    stride_arg = DeclareLaunchArgument(
        'stride',
        default_value='1',
        description='Stride for downsampling scan (1 = no downsampling)'
    )

    panoramic_scan_arg = DeclareLaunchArgument(
        'panoramic_scan',
        default_value='True',
        description='Set to True for 360-degree scans'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Input laser scan topic'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='map',
        description='Target frame for published detections (map / odom / base_link)'
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

    use_onnx_arg = DeclareLaunchArgument(
        'use_onnx',
        default_value='True',
        description='Use ONNX Runtime backend if .onnx sibling exists (True/False)'
    )

    # ── Environment: make cuDNN visible for ONNX Runtime CUDA EP ────────
    env_actions = []
    cudnn_dir = _cudnn_lib_dir()
    if cudnn_dir:
        existing_ld = os.environ.get("LD_LIBRARY_PATH", "")
        new_ld = f"{cudnn_dir}:{existing_ld}" if existing_ld else cudnn_dir
        env_actions.append(SetEnvironmentVariable("LD_LIBRARY_PATH", new_ld))

    # ── Node ────────────────────────────────────────────────────────────────
    lidar_human_detection_node = Node(
        package='smrr_human_tracker',
        executable='lidar_human_detection',
        name='lidar_human_detection_node',
        output='screen',
        parameters=[{
            'weight_file':       LaunchConfiguration('weight_file'),
            'detector_model':    LaunchConfiguration('detector_model'),
            'conf_thresh':       LaunchConfiguration('conf_thresh'),
            'stride':            LaunchConfiguration('stride'),
            'panoramic_scan':    LaunchConfiguration('panoramic_scan'),
            'scan_topic':        LaunchConfiguration('scan_topic'),
            'target_frame':      LaunchConfiguration('target_frame'),
            'detections_topic':  LaunchConfiguration('detections_topic'),
            'marker_topic':      LaunchConfiguration('marker_topic'),
            'use_onnx':          LaunchConfiguration('use_onnx'),
        }]
    )

    return LaunchDescription([
        *env_actions,
        weight_file_arg,
        detector_model_arg,
        conf_thresh_arg,
        stride_arg,
        panoramic_scan_arg,
        scan_topic_arg,
        target_frame_arg,
        detections_topic_arg,
        marker_topic_arg,
        use_onnx_arg,
        lidar_human_detection_node,
    ])
