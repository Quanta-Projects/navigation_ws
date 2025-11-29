from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('smrr_docking')
    
    # Configuration files
    apriltag_params = os.path.join(pkg_share, 'config', 'apriltag.yaml')
    docking_params = os.path.join(pkg_share, 'config', 'docking.yaml')
    
    # Camera namespace
    camera_ns = '/zed2_rear_left_camera'
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    dock_tag_id_arg = DeclareLaunchArgument(
        'dock_tag_id',
        default_value='0',
        description='AprilTag ID for the docking station'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    dock_tag_id = LaunchConfiguration('dock_tag_id')

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        dock_tag_id_arg,
        
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
                ('image_rect', f'{camera_ns}/image_raw'),
                ('camera_info', f'{camera_ns}/camera_info'),
                ('detections', '/apriltag/detections'),  
            ]
        ),
        
        # AprilTag to Dock Pose converter
        Node(
            package='smrr_docking',
            executable='apriltag_to_dock_pose',
            name='apriltag_to_dock_pose',
            output='screen',
            parameters=[
                {
                    'dock_tag_id': dock_tag_id,
                    'target_frame': 'map',
                    'use_sim_time': use_sim_time
                }
            ]
        ),
        
        # Docking server node
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[
                docking_params,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),
        
        # Lifecycle manager for docking server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_docking',
            output='screen',
            parameters=[
                {'node_names': ['docking_server']},
                {'autostart': True},
                {'use_sim_time': use_sim_time}
            ]
        ),
    ])
