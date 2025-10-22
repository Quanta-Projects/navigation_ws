from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('smrr_docking')
    
    # Configuration files
    apriltag_params = os.path.join(pkg_share, 'config', 'apriltag.yaml')
    
    # Camera namespace
    camera_ns = '/zed2_rear_left_raw_camera'
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        
        # AprilTag detection node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            output='screen',
            parameters=[
                apriltag_params,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                # Subscribe to rectified image and camera info
                ('image_rect', f'{camera_ns}/image_rect'),
                ('camera_info', f'{camera_ns}/camera_info'),
                # Publish detections
                ('detections', '/apriltag/detections'),
            ]
        ),
        
        # Optional: RViz for visualization
        # Uncomment to launch RViz automatically
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}]
        # ),
    ])
