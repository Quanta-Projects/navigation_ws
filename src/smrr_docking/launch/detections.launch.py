# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
    
#     # 1. PATH CONFIGURATION
#     pkg_share = get_package_share_directory('smrr_docking')
#     apriltag_config_path = os.path.join(pkg_share, 'config', 'apriltag.yaml')

#     # 2. SOURCE TOPICS (Derived from your provided list)
#     # The Raw RGB Camera topics
#     camera_topic = '/zed2_rear_left_camera/image_raw'
#     info_topic   = '/zed2_rear_left_camera/camera_info'

#     return LaunchDescription([
        
#         # ---------------------------------------------------------
#         # Node 2: AprilTag Detection
#         # ---------------------------------------------------------
#         Node(
#             package='apriltag_ros',
#             executable='apriltag_node',
#             name='apriltag_node',
#             remappings=[
#                 # INPUT: Listen to the rectified output from Node 1
#                 ('image_rect', camera_topic), 
                
#                 # INPUT: Camera Info comes directly from the source
#                 ('camera_info', info_topic),
#             ],
#             parameters=[
#                 apriltag_config_path, 
#                 {
#                     'publish_tf': True,
#                     'size': 0.20,
#                     'max_hamming': 2,       # Increased from 0 to 2 for robust continuous detection
#                     'decimate': 1.0,        # No decimation for best accuracy
#                     'blur': 0.0,            # No blur
#                     'refine_edges': 1,      # Better edge refinement
#                     'threads': 4,           # Parallel processing for speed
#                     'debug': 0,
#                     'tag_family': 'tag36h11'
#                 }
#             ],
#             output='screen'
#         ),

#         # ---------------------------------------------------------
#         # Node 3: Static Transform (Tag -> Dock)
#         # ---------------------------------------------------------
#         # Rotates the Tag Frame so Z-out becomes X-out (for the robot to back into)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             arguments=['0', '-0.35', '0.20', '0', '-1.57', '-1.57', 'tag36h11:0', 'dock_link'],
#             output='screen'
#         ),

#         # ---------------------------------------------------------
#         # Node 4: Docking Bridge (TF to Pose)
#         # ---------------------------------------------------------
#         Node(
#             package='smrr_docking',
#             executable='tf_to_pose',
#             name='docking_bridge_node',
#             parameters=[{
#                 'tag_frame': 'dock_link',   # Listen for the offset dock frame
#                 'reference_frame': 'odom'   # Publish pose relative to odom
#             }],
#             output='screen'
#         )
#     ])



import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('smrr_docking')
    apriltag_config_path = os.path.join(pkg_share, 'config', 'apriltag.yaml')

    # --- REAL HARDWARE TOPICS ---
    camera_topic = '/zed/zed_node/rgb/color/rect/image'
    info_topic   = '/zed/zed_node/rgb/color/rect/camera_info'

    return LaunchDescription([
        
        # ---------------------------------------------------------
        # Node 1: AprilTag Detection
        # ---------------------------------------------------------
        # DIRECT CONNECTION: We skip image_proc because ZED provides 
        # '/rect/image' directly.
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                # Remap standard input to the ZED topic
                ('image_rect', camera_topic), 
                ('camera_info', info_topic),
            ],
            parameters=[
                apriltag_config_path, 
                {
                    'publish_tf': True,
                    'size': 0.20,
                    'max_hamming': 2,       # Increased from 0 to 2 for robust continuous detection
                    'decimate': 1.0,        # No decimation for best accuracy
                    'blur': 0.0,            # No blur
                    'refine_edges': 1,      # Better edge refinement
                    'threads': 4,           # Parallel processing for speed
                    'debug': 0,
                    'tag_family': 'tag36h11'
                }
            ],
            output='screen'
        ),

        # ---------------------------------------------------------
        # Node 2: Static Transform (Tag -> Dock)
        # ---------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '-0.35', '0.20', '0', '-1.57', '-1.57', 'tag36h11:0', 'dock_link'],
            output='screen'
        ),

        # ---------------------------------------------------------
        # Node 3: Docking Bridge (TF to Pose)
        # ---------------------------------------------------------
        Node(
            package='smrr_docking',
            executable='tf_to_pose',
            name='docking_bridge_node',
            parameters=[{
                'tag_frame': 'dock_link',
                'reference_frame': 'odom'  # Note: ZED publishes 'odom' frame too!
            }],
            output='screen'
        )
    ])