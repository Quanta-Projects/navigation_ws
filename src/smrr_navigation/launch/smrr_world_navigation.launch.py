from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('smrr_navigation'),'config')
    maps_dir = os.path.join(get_package_share_directory('smrr_navigation'),'maps')
    map_file = os.path.join(maps_dir,'first_floor_with_docking_station.yaml')
    params_file = os.path.join(config_dir,'smrr_nav_params.yaml')
    rviz_config= os.path.join(config_dir,'smrr_nav.rviz')
    
    # Launch arguments
    declare_enable_startup_localizer = DeclareLaunchArgument(
        'enable_startup_localizer',
        default_value='false',
        description='Enable automatic startup localization sequence to help AMCL converge'
    )
    
    declare_enable_multifloor_navigator = DeclareLaunchArgument(
        'enable_multifloor_navigator',
        default_value='true',
        description='Enable multi-floor BT navigator (Step 4a - same floor navigation only)'
    )
    
    enable_startup_localizer = LaunchConfiguration('enable_startup_localizer')
    enable_multifloor_navigator = LaunchConfiguration('enable_multifloor_navigator')
    
    return LaunchDescription([
    
    # Arguments
    declare_enable_startup_localizer,
    declare_enable_multifloor_navigator,

    # Bringing our Robot with ros2_control controllers
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('smrr_description'), 'launch', 'gazebo_classic_controllers.launch.py')),
        launch_arguments={
            'use_sim_time': 'true'
        }.items(),
    ),


    # Integrating Nav2 Stack
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'True',
            'autostart': 'True',
            'use_composition': 'True',
            'use_respawn': 'False'
        }.items(),

    ),
    
    # Startup Localizer - Automatic sequence to help AMCL converge
    # Drives forward ~1m (odometry-based), then rotates 360° (yaw-based) to scan environment
    # Delayed start to allow AMCL to initialize first
    TimerAction(
        period=2.0,  # Wait 2 seconds after launch
        actions=[
            Node(
                package='smrr_navigation',
                executable='startup_localizer',
                name='startup_localizer',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'startup_delay': 2.0},           # Additional wait for AMCL init
                    {'forward_speed': 0.15},          # m/s - drive forward speed
                    {'forward_distance': 1.0},        # meters - distance to drive
                    {'rotation_speed': 0.5},          # rad/s - rotation speed
                    {'target_rotation_angle': 2.0 * 3.141592653589793},  # radians (360°)
                    {'control_period': 0.05},         # 20 Hz control loop
                    {'stop_duration': 1.0}            # Pause between motions
                ],
                condition=IfCondition(enable_startup_localizer)
            )
        ]
    ),

    # Named Goal Server - Resolver and dispatcher for named locations
    Node(
        package='smrr_navigation',
        executable='named_goal_server',
        name='named_goal_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'locations_file': 'locations.yaml'},
            {'multifloor_action_name': '/navigate_to_named_location'},
            {'multifloor_action_timeout': 10.0}
        ]
    ),

    # Location Subscriber - Bridge from /location topic to named_goal_server
    Node(
        package='smrr_navigation',
        executable='location_subscriber',
        name='location_subscriber',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'location_topic': 'location'},
            {'service_name': '/go_to_pose'},
            {'service_timeout': 5.0}
        ]
    ),

    # SMRR Multi-Floor BT Navigator (Step 4a - same floor only, enabled by default)
    Node(
        package='smrr_navigation',
        executable='smrr_multifloor_bt_navigator',
        name='smrr_multifloor_bt_navigator',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'global_frame': 'map'},
            {'nav2_action_name': 'navigate_to_pose'},
            {'nav2_wait_timeout': 10.0},
            {'feedback_rate_hz': 2.0}
        ],
        condition=IfCondition(enable_multifloor_navigator)
    ),

    # Rviz2 bringup
    Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d',rviz_config],
        parameters=[{'use_sim_time': True}]
    ),

    ])