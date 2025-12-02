#!/usr/bin/env python3

"""
Nav2 Navigation Launch File for Real Hardware Robot
This launch file only includes Nav2 stack components for hardware deployment.
Hardware interface, controllers, and sensors should be launched separately.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace, Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    smrr_navigation_dir = get_package_share_directory('smrr_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Paths to configuration files
    config_dir = os.path.join(smrr_navigation_dir, 'config')
    maps_dir = os.path.join(get_package_share_directory('smrr_navigation'),'maps', 'physical_maps')
    default_map_file = os.path.join(maps_dir, 'first_floor.yaml')
    
    # Nav2 parameters file
    params_file = os.path.join(config_dir, 'smrr_nav_params_hardware.yaml')
    
    # RViz configuration file
    rviz_config = os.path.join(config_dir, 'smrr_nav.rviz')
    
    # If hardware params don't exist, use the existing params file
    if not os.path.exists(params_file):
        params_file = os.path.join(config_dir, 'smrr_nav_params.yaml')
    
    # Declare launch arguments
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for Nav2 nodes'
    )
    
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack'
    )
    
    declare_use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Use composed bringup if True'
    )
    
    declare_use_respawn_arg = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes'
    )
    
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )
    
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',  # Set to False for SSH/headless operation
        description='Whether to launch RViz'
    )
    
    declare_enable_startup_localizer = DeclareLaunchArgument(
        'enable_startup_localizer',
        default_value='true',
        description='Enable automatic startup localization sequence to help AMCL converge'
    )
    
    # Launch configurations
    map_yaml_file = LaunchConfiguration('map')
    params_file_config = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    namespace = LaunchConfiguration('namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_startup_localizer = LaunchConfiguration('enable_startup_localizer')
    
    # Include Nav2 bringup launch file
    # This launches: map_server, amcl, planner_server, controller_server, 
    # recoveries_server, bt_navigator, waypoint_follower, lifecycle_manager
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file_config,
            'use_sim_time': 'False',  # Hardware mode - no simulation time
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'namespace': namespace
        }.items(),
    )
    
    # Group actions with namespace if provided
    bringup_cmd_group = GroupAction([
        PushRosNamespace(namespace),
        nav2_bringup,
    ])
    
    # RViz2 bringup
    rviz_node = Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz)
    )
    
    # Startup Localizer - Automatic sequence to help AMCL converge
    # Drives forward ~1m (odometry-based), then rotates 360° (yaw-based) to scan environment
    # Delayed start to allow AMCL to initialize first
    startup_localizer = TimerAction(
        period=2.0,  # Wait 2 seconds after launch
        actions=[
            Node(
                package='smrr_navigation',
                executable='startup_localizer',
                name='startup_localizer',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
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
    )
    
    # Named Goal Server - Navigate to predefined named locations
    named_goal_server = Node(
        package='smrr_navigation',
        executable='named_goal_server',
        name='named_goal_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'locations_file': 'physical_locations.yaml'},
            {'global_frame': 'map'},
            {'action_timeout': 300.0}
        ]
    )
    
    # Location Subscriber - Bridge from /location topic to named_goal_server
    location_subscriber = Node(
        package='smrr_navigation',
        executable='location_subscriber',
        name='location_subscriber',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'location_topic': 'location'},
            {'service_name': '/go_to_pose'},
            {'service_timeout': 5.0}
        ]
    )
    
    return LaunchDescription([
        # Declare launch arguments
        declare_map_arg,
        declare_params_file_arg,
        declare_autostart_arg,
        declare_use_composition_arg,
        declare_use_respawn_arg,
        declare_namespace_arg,
        declare_use_rviz_arg,
        declare_enable_startup_localizer,
        
        # Launch Nav2 stack
        bringup_cmd_group,
        
        # Launch startup localizer
        startup_localizer,
        
        # Launch named goal server
        named_goal_server,
        
        # Launch location subscriber
        location_subscriber,
        
        # Launch RViz2
        rviz_node,
    ])
