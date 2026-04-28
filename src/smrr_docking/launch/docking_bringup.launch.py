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
    docking_stage1_params = os.path.join(pkg_share, 'config', 'docking_stage1.yaml')
    docking_stage2_params = os.path.join(pkg_share, 'config', 'docking_stage2.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,

        # ---------------------------------------------------------------
        # Stage 1 docking server (/stage1 namespace)
        # High k_phi / k_delta — aggressive correction from staging pose.
        # Stops at 3/5 of staging distance (0.64 m from dock).
        # ---------------------------------------------------------------
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            namespace='stage1',
            output='screen',
            parameters=[
                docking_stage1_params,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('cmd_vel',           '/cmd_vel'),
                ('odom',              '/odom'),
                ('navigate_to_pose',  '/navigate_to_pose'),
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_docking',
            namespace='stage1',
            output='screen',
            parameters=[
                {'node_names': ['docking_server']},
                {'autostart': True},
                {'use_sim_time': use_sim_time}
            ]
        ),

        # ---------------------------------------------------------------
        # Stage 2 docking server (/stage2 namespace)
        # Low k_phi / k_delta — smooth final approach to full contact.
        # Battery charging current confirms successful docking.
        # ---------------------------------------------------------------
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            namespace='stage2',
            output='screen',
            parameters=[
                docking_stage2_params,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('cmd_vel',           '/cmd_vel'),
                ('odom',              '/odom'),
                ('navigate_to_pose',  '/navigate_to_pose'),
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_docking',
            namespace='stage2',
            output='screen',
            parameters=[
                {'node_names': ['docking_server']},
                {'autostart': True},
                {'use_sim_time': use_sim_time}
            ]
        ),

        # ---------------------------------------------------------------
        # Two-stage orchestrator — exposes /dock_robot to external callers
        # Chains Stage 1 → Stage 2 internally.
        # ---------------------------------------------------------------
        Node(
            package='smrr_docking',
            executable='two_stage_dock',
            name='two_stage_dock',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Safe undock wrapper — call /undock_robot_safe instead of /undock_robot
        # Stops charging and waits before forwarding to the real undock action
        Node(
            package='smrr_docking',
            executable='undock_with_charge_stop',
            name='undock_with_charge_stop',
            output='screen',
            parameters=[{
                'settle_time': 5.0,
                'charge_stop_timeout': 10.0,
            }]
        ),

    ])
