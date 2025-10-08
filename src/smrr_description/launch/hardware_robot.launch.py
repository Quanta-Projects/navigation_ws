#!/usr/bin/env python3

"""
Launch file for running the SMRR robot on real hardware.

This launch file:
1. Loads robot description with is_sim:=false
2. Starts robot_state_publisher with real-time clock
3. Launches ros2_control hardware interface
4. Spawns all controllers
5. Optionally starts navigation stack

Usage:
    ros2 launch smrr_description hardware_robot.launch.py
    
    # With navigation:
    ros2 launch smrr_description hardware_robot.launch.py use_navigation:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    
    # Declare arguments
    declare_use_navigation = DeclareLaunchArgument(
        'use_navigation',
        default_value='false',
        description='Launch navigation stack'
    )
    
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for hardware communication'
    )

    # Get launch configurations
    use_navigation = LaunchConfiguration('use_navigation')
    
    # Robot description with is_sim:=false for hardware
    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("smrr_description"),
                "urdf",
                "test.urdf.xacro"
            ]),
            " is_sim:=false"
        ]),
        value_type=str
    )

    # Robot state publisher - CRITICAL: use_sim_time:=false for hardware
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": False}  # Use real-time clock
        ],
        output="screen"
    )

    # ros2_control node - loads hardware interface from smrr_base_controller
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            PathJoinSubstitution([
                FindPackageShare("smrr_controller"),
                "config",
                "arm_controller.yaml"
            ]),
            {"use_sim_time": False}
        ],
        output="screen",
        emulate_tty=True
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Spawn arm controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Spawn differential drive controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Twist mux for velocity command arbitration
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("smrr_controller"),
                "config",
                "twist_mux.yaml"
            ]),
            {"use_sim_time": False}
        ],
        remappings=[('/cmd_vel_out', '/cmd_vel_out_unstamped')],
        output="screen"
    )

    # Twist stamper to add timestamps to velocity commands
    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        parameters=[{"use_sim_time": False}],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel_out_unstamped'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ],
        output="screen"
    )

    # Optional: Robot localization (EKF)
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("smrr_localization"),
                "config",
                "modified_ekf.yaml"
            ]),
            {"use_sim_time": False}
        ],
        output="screen"
    )

    # Optional: Include navigation stack
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("smrr_navigation"),
                "launch",
                "navigation.launch.py"  # Make sure this exists
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false'
        }.items(),
        condition=IfCondition(use_navigation)
    )

    return LaunchDescription([
        # Arguments
        declare_use_navigation,
        declare_port,
        
        # Core nodes
        robot_state_publisher,
        controller_manager,
        
        # Controller spawners
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        diff_drive_controller_spawner,
        
        # Velocity command processing
        twist_mux,
        twist_stamper,
        
        # Localization
        robot_localization,
        
        # Optional navigation
        navigation
    ])
