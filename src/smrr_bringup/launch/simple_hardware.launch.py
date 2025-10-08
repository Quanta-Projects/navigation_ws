#!/usr/bin/env python3

"""
Simplified hardware launch - Uses existing controller.launch.py

This launch file starts the hardware interface and calls your existing
controller.launch.py with is_sim:=False

Usage:
    ros2 launch smrr_description simple_hardware.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():

    # Load controller manager with hardware interface
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(
                get_package_share_directory("smrr_controller"),
                "config",
                "arm_controller.yaml"
            ),
            {"use_sim_time": False}
        ],
        output="screen"
    )

    # Include your existing controller launch with is_sim:=False
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("smrr_controller"),
                "launch",
                "controller.launch.py"
            ])
        ]),
        launch_arguments={
            'is_sim': 'False'  # This tells it to use hardware interface
        }.items()
    )

    joystick_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("smrr_controller"),
                "launch",
                "joystick_teleop.launch.py"
            ])
        ])
    )

    return LaunchDescription([
        controller_manager,
        controllers,
        joystick_control
    ])
