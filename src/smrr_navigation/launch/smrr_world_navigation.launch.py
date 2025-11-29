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
        default_value='true',
        description='Enable automatic startup localization sequence to help AMCL converge'
    )
    
    enable_startup_localizer = LaunchConfiguration('enable_startup_localizer')
    
    return LaunchDescription([
    
    # Arguments
    declare_enable_startup_localizer,

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