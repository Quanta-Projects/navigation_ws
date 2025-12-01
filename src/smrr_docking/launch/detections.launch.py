from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # UPDATE THIS PATH to your actual config file location
    apriltag_config_path = 'src/smrr_docking/config/apriltag.yaml'

    return LaunchDescription([
        
        # ---------------------------------------------------------
        # Node 1: Image Processing (Rectification)
        # ---------------------------------------------------------
        Node(
            package='image_proc',
            executable='image_proc',
            name='rectify_node_zed',
            namespace='zed2_rear_left_camera',
            # -----------------------------------------------------
            # THE FIX IS HERE:
            # We must tell the node that 'image' input is actually 'image_raw'
            # -----------------------------------------------------
            remappings=[
                ('image', 'image_raw') 
            ]
        ),

        # ---------------------------------------------------------
        # Node 2: AprilTag Detection
        # ---------------------------------------------------------
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                # Subscribe to the rectified image from Node 1
                ('image_rect', '/zed2_rear_left_camera/image_rect'),
                ('camera_info', '/zed2_rear_left_camera/camera_info'),
            ],
            parameters=[
            apriltag_config_path, 
            {
                'publish_tf': True,
                'size': 0.20,      # <--- FORCE THE SIZE HERE
                'max_hamming': 0
            }
        ],
            output='screen'
        ),

        # 3. Static Transform: Tag -> Dock
        # "My charger is 30cm (0.3m) below the tag on the wall"
        # Format: x y z yaw pitch roll frame_id child_frame_id
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '-1.57', '-1.57', 'tag36h11:0', 'dock_link'],
            output='screen'
        ),

        # 4. Docking Bridge Node: TF to Pose
        Node(
            package='smrr_docking',
            executable='tf_to_pose',
            name='docking_bridge_node',
            parameters=[{
                'tag_frame': 'dock_link',   # Listen for the offset dock frame
                'reference_frame': 'odom'   # Publish pose relative to odom
            }],
            output='screen'
        )
    ])