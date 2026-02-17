import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smrr_base_controller"),
            "launch",
            "hardware_interface.launch.py"
        ),
        launch_arguments={'port': '/dev/ttyACM2'}.items()
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smrr_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={'is_sim': 'False'}.items()
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smrr_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
    )
    
    # SLLidar ROS2 launch (using sllidar_ros2 package)
    sllidar = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sllidar_ros2"),
            "launch",
            "sllidar_a2m8_launch.py"
        ),
        launch_arguments={
            'frame_id': 'rplidar_link'
        }.items()
    )
    
    # Odometry path visualization
    odom_path = Node(
        package='smrr_navigation',
        executable='odom_to_path',
        name='odom_to_path',
        parameters=[{
            'odom_topic': '/diff_drive_controller/odom',
            'path_topic': '/odom_path',
            'max_path_length': 10000
        }],
        output='screen'
    )
    
    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smrr_navigation"),
            "launch",
            "smrr_hardware_navigation.launch.py"
        ),
        launch_arguments={
            'autostart': 'True'
        }.items()
    )
    
    return LaunchDescription([
        hardware_interface,
        controller,
        joystick,
        sllidar
        # odom_path,
        # navigation
    ])