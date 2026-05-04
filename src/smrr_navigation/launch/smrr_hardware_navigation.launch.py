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
    maps_dir = os.path.join(smrr_navigation_dir, 'maps', 'physical_maps')
    default_map_file = os.path.join(maps_dir, 'first_floor_with_lift.yaml')
    bt_xml_path = os.path.join(smrr_navigation_dir, 'behavior_trees', 'smrr_multifloor.xml')

    # Nav2 parameters file (hardware-tuned; fallback to sim params if absent)
    params_file = os.path.join(config_dir, 'smrr_nav_params_hardware.yaml')
    if not os.path.exists(params_file):
        params_file = os.path.join(config_dir, 'smrr_nav_params.yaml')

    # YOLO button detection model for floor arrival verification
    yolo_model_path = os.path.join(smrr_navigation_dir, 'models', 'button_detection.pt')

    # RViz configuration file
    rviz_config = os.path.join(config_dir, 'smrr_nav.rviz')

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
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
        default_value='False',
        description='Enable automatic startup localization sequence to help AMCL converge'
    )

    declare_initial_floor_id = DeclareLaunchArgument(
        'initial_floor_id',
        default_value='floor1',
        description='Initial floor ID for the robot (must match physical_locations.yaml)'
    )

    # ---------------------------------------------------------------------------
    # Launch configurations
    # ---------------------------------------------------------------------------
    map_yaml_file = LaunchConfiguration('map')
    params_file_config = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    namespace = LaunchConfiguration('namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_startup_localizer = LaunchConfiguration('enable_startup_localizer')
    initial_floor_id = LaunchConfiguration('initial_floor_id')

    # ---------------------------------------------------------------------------
    # EKF Localization
    # Fuses wheel odometry + ZED2 IMU (yaw rate) → /odometry/filtered
    # ---------------------------------------------------------------------------
    ekf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('smrr_localization'),
                'launch',
                'local_localization.launch.py',
            )
        )
    )

    # ---------------------------------------------------------------------------
    # Nav2 bringup
    # Launches: map_server, amcl, planner_server, controller_server,
    #           recoveries_server, bt_navigator, waypoint_follower, lifecycle_manager
    # ---------------------------------------------------------------------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file_config,
            'use_sim_time': 'False',  # Hardware mode — no simulation time
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'namespace': namespace
        }.items(),
    )

    # Group with namespace support
    bringup_cmd_group = GroupAction([
        PushRosNamespace(namespace),
        nav2_bringup,
    ])

    # ---------------------------------------------------------------------------
    # Startup Localizer
    # Drives forward ~1 m then rotates 360° to help AMCL converge on hardware
    # ---------------------------------------------------------------------------
    startup_localizer = TimerAction(
        period=2.0,  # Wait 2 s after launch for AMCL to initialise
        actions=[
            Node(
                package='smrr_navigation',
                executable='startup_localizer.py',
                name='startup_localizer',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'startup_delay': 2.0},
                    {'forward_speed': 0.15},
                    {'forward_distance': 1.0},
                    {'rotation_speed': 0.5},
                    {'target_rotation_angle': 2.0 * 3.141592653589793},
                    {'control_period': 0.05},
                    {'stop_duration': 1.0}
                ],
                condition=IfCondition(enable_startup_localizer)
            )
        ]
    )

    # ---------------------------------------------------------------------------
    # Named Goal Server
    # Resolves named locations from physical_locations.yaml and dispatches
    # missions to the BT Mission Executor via /start_mission service
    # ---------------------------------------------------------------------------
    named_goal_server = Node(
        package='smrr_navigation',
        executable='named_goal_server.py',
        name='named_goal_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': False},
            {'locations_file': 'physical_locations.yaml'},
            {'use_bt_mission_executor': True},
            {'initial_floor_id': initial_floor_id},
            {'start_mission_service_name': '/start_mission'},
            {'start_mission_timeout': 5.0}
        ]
    )

    # ---------------------------------------------------------------------------
    # Location Subscriber
    # Bridges /location string topic to /go_to_pose service
    # ---------------------------------------------------------------------------
    location_subscriber = Node(
        package='smrr_navigation',
        executable='location_subscriber.py',
        name='location_subscriber',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'location_topic': 'location'},
            {'service_name': '/go_to_pose'},
            {'service_timeout': 5.0}
        ]
    )

    # ---------------------------------------------------------------------------
    # Floor Arrival Server
    # Verifies floor arrival via YOLO button detection + HSV camera analysis
    # ---------------------------------------------------------------------------
    floor_arrival_server = Node(
        package='smrr_navigation',
        executable='floor_arrival_server.py',
        name='floor_arrival_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'yolo_model_path': yolo_model_path}
        ]
    )

    # ---------------------------------------------------------------------------
    # Elevator Call Button Server
    # Detects UP/DOWN call button via YOLO + depth and computes approach point
    # ---------------------------------------------------------------------------
    elevator_call_button_server = Node(
        package='smrr_navigation',
        executable='elevator_call_button_server.py',
        name='elevator_call_button_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'yolo_model_path': yolo_model_path}
        ]
    )

    # ---------------------------------------------------------------------------
    # Elevator Floor Button Server
    # Presses floor panel buttons (G/1/2/3) inside elevator via YOLO + RANSAC;
    # verifies press via lit-button detection
    # ---------------------------------------------------------------------------
    elevator_floor_button_server = Node(
        package='smrr_navigation',
        executable='elevator_floor_button_server.py',
        name='elevator_floor_button_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'yolo_model_path': yolo_model_path}
        ]
    )

    # ---------------------------------------------------------------------------
    # BT Mission Executor
    # Runs the smrr_multifloor BehaviorTree for same/cross-floor navigation
    # ---------------------------------------------------------------------------
    bt_mission_executor = Node(
        package='smrr_navigation',
        executable='smrr_bt_mission_executor',
        name='smrr_bt_mission_executor',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'bt_xml_path': bt_xml_path,
            'locations_file': 'physical_locations.yaml',
            'plugin_lib_names': [
                # Nav2 BT plugins
                'nav2_compute_path_to_pose_action_bt_node',
                'nav2_follow_path_action_bt_node',
                'nav2_back_up_action_bt_node',
                'nav2_spin_action_bt_node',
                'nav2_wait_action_bt_node',
                'nav2_clear_costmap_service_bt_node',
                'nav2_is_stuck_condition_bt_node',
                'nav2_goal_reached_condition_bt_node',
                'nav2_initial_pose_received_condition_bt_node',
                'nav2_goal_updated_condition_bt_node',
                'nav2_reinitialize_global_localization_service_bt_node',
                'nav2_rate_controller_bt_node',
                'nav2_distance_controller_bt_node',
                'nav2_speed_controller_bt_node',
                'nav2_truncate_path_action_bt_node',
                'nav2_goal_updater_node_bt_node',
                'nav2_recovery_node_bt_node',
                'nav2_pipeline_sequence_bt_node',
                'nav2_round_robin_node_bt_node',
                'nav2_transform_available_condition_bt_node',
                'nav2_time_expired_condition_bt_node',
                'nav2_distance_traveled_condition_bt_node',
                'nav2_single_trigger_bt_node',
                'nav2_is_battery_low_condition_bt_node',
                'nav2_navigate_through_poses_action_bt_node',
                'nav2_navigate_to_pose_action_bt_node',
                # Custom BT plugins
                'smrr_bt_nodes'
            ],
            'bt_tick_rate_hz': 20.0,
            'bt_timeout_sec': 1500.0
        }]
    )

    # ---------------------------------------------------------------------------
    # AprilTag Manager Server
    # Dynamically start/stop apriltag_node via /toggle_apriltag service
    # to save CPU when detection is not needed
    # ---------------------------------------------------------------------------
    apriltag_manager_server = Node(
        package='smrr_navigation',
        executable='apriltag_manager_server.py',
        name='apriltag_manager_server',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # ---------------------------------------------------------------------------
    # RViz2 (optional — disabled by default for SSH/headless operation)
    # ---------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # Launch arguments
        declare_map_arg,
        declare_params_file_arg,
        declare_autostart_arg,
        declare_use_composition_arg,
        declare_use_respawn_arg,
        declare_namespace_arg,
        declare_use_rviz_arg,
        declare_enable_startup_localizer,
        declare_initial_floor_id,

        # Nav2 stack
        bringup_cmd_group,

        # EKF localization (wheel odom + ZED2 IMU yaw rate)
        # ekf_localization,

        # Startup localizer (hardware AMCL convergence helper)
        startup_localizer,

        # Named goal server (physical_locations.yaml → BT executor)
        named_goal_server,

        # Location subscriber (/location topic bridge)
        location_subscriber,

        # Floor arrival server (YOLO + HSV floor verification)
        floor_arrival_server,

        # Elevator call button server (YOLO + depth)
        elevator_call_button_server,

        # Elevator floor button server (YOLO + depth, inside elevator)
        elevator_floor_button_server,

        # BT Mission Executor (multi-floor behavior tree)
        bt_mission_executor,

        # AprilTag Manager Server (CPU-saving toggle for AprilTag detection)
        apriltag_manager_server,

        # RViz2 (opt-in)
        rviz_node,
    ])

