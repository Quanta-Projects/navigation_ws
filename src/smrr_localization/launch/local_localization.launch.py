#!/usr/bin/env python3
"""
EKF localization launch for real hardware.
Fuses wheel odometry (/diff_drive_controller/odom) with ZED2 IMU
(/zed2/zed_node/imu/data, RELIABLE publisher — compatible with EKF's
BEST_EFFORT subscriber in DDS).
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('smrr_localization'),
        'config',
        'ekf.yaml',
    )

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([
        robot_localization,
    ])
