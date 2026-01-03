from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('smrr_navigation')
    config_dir = os.path.join(pkg_share, 'config')
    maps_dir = os.path.join(pkg_share, 'maps')
    bt_xml_path = os.path.join(pkg_share, 'config', 'bt', 'same_floor_nav.xml')
    map_file = os.path.join(maps_dir, 'first_floor_with_docking_station.yaml')
    params_file = os.path.join(config_dir, 'smrr_nav_params.yaml')
    rviz_config = os.path.join(config_dir, 'smrr_nav.rviz')
    
    # Launch arguments
    declare_enable_startup_localizer = DeclareLaunchArgument(
        'enable_startup_localizer',
        default_value='false',
        description='Enable automatic startup localization sequence to help AMCL converge'
    )
    
    declare_initial_floor_id = DeclareLaunchArgument(
        'initial_floor_id',
        default_value='floor0',
        description='Initial floor ID for the robot'
    )
    
    enable_startup_localizer = LaunchConfiguration('enable_startup_localizer')
    initial_floor_id = LaunchConfiguration('initial_floor_id')
    
    return LaunchDescription([
    
    # Arguments
    declare_enable_startup_localizer,
    declare_initial_floor_id,

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
                executable='startup_localizer.py',
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

    # Named Goal Server - Resolver and dispatcher for named locations (with BT executor)
    Node(
        package='smrr_navigation',
        executable='named_goal_server.py',
        name='named_goal_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': True},
            {'locations_file': 'locations.yaml'},
            {'use_bt_mission_executor': True},
            {'initial_floor_id': initial_floor_id},
            {'start_mission_service_name': '/start_mission'},
            {'start_mission_timeout': 5.0}
        ]
    ),

    # Location Subscriber - Bridge from /location topic to named_goal_server
    Node(
        package='smrr_navigation',
        executable='location_subscriber.py',
        name='location_subscriber',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'location_topic': 'location'},
            {'service_name': '/go_to_pose'},
            {'service_timeout': 5.0}
        ]
    ),

    # BT Mission Executor - Runs BehaviorTree for same-floor navigation
    Node(
        package='smrr_navigation',
        executable='smrr_bt_mission_executor',
        name='smrr_bt_mission_executor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'bt_xml_path': bt_xml_path,
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
                'nav2_navigate_to_pose_action_bt_node',
                # Custom BT plugins
                'smrr_bt_nodes'
            ],
            'bt_tick_rate_hz': 20.0,
            'bt_timeout_sec': 300.0
        }]
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