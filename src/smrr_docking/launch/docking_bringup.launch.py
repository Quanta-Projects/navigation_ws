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
    docking_params = os.path.join(pkg_share, 'config', 'docking.yaml')
    
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
                # Remap topics to match your robot's topics
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),
        
        # Lifecycle manager for docking server (recommended)
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

        # Node(
        #   package='apriltag_ros', executable='apriltag_node',
        #   name='apriltag',
        #   parameters=['config/apriltag.yaml'],
        #   remappings=[
        #     ('image', '/camera/image_raw'),
        #     ('camera_info', '/camera/camera_info'),
        #   ],
        #   output='screen'
        # ),

        # # Static dock pose publisher (for testing)
        # Node(
        #     package='smrr_docking',
        #     executable='publish_state_dock_pose',
        #     name='publish_state_dock_pose',
        #     output='screen',
        #     parameters=[
        #         {'use_sim_time': use_sim_time}
        #     ]
        # ),

    ])
