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
        default_value='false',
        description='Use simulation time'
    )
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        
        # Docking server node
        # 'battery_state' is hardcoded in SimpleChargingDock — it does NOT read
        # the battery_topic yaml param.  We remap it to /dock_confirmed so that
        # dock_confirm_node (which gates on 3s of real charging) is in the loop.
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[
                docking_params,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[('battery_state', '/dock_confirmed')]
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

        # Dock confirmation node — gates docking success behind real charging.
        # Subscribes to /battery_state; publishes /dock_confirmed only after
        # battery.current > charging_threshold is held for settle_time seconds.
        Node(
            package='smrr_docking',
            executable='dock_confirm_node',
            name='dock_confirm_node',
            output='screen',
            parameters=[{
                'settle_time': 5.0,
                'charging_threshold': 0.5,
                'battery_topic': '/battery_state',
            }]
        ),

    ])
