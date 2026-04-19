import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('smrr_bringup')
    zed_wrapper_share = get_package_share_directory('zed_wrapper')

    zed_camera_launch = os.path.join(zed_wrapper_share, 'launch', 'zed_camera.launch.py')
    zed_override_config = os.path.join(bringup_share, 'config', 'zed_override.yaml')
    laser_filter_config = os.path.join(bringup_share, 'config', 'laser_scan_240deg_filter.yaml')

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smrr_base_controller"),
            "launch",
            "hardware_interface.launch.py"
        ),
        launch_arguments={'port': '/dev/ttyACM0'}.items()
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

    # SLLidar ROS2 launch (RPLIDAR S2L)
    sllidar = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sllidar_ros2"),
            "launch",
            "sllidar_s2_launch.py"
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '1000000',
            'frame_id': 'rplidar_link',
            'scan_topic': '/scan_raw',
            'scan_mode': 'DenseBoost'
        }.items()
    )

    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[laser_filter_config],
        remappings=[
            ('scan', '/scan_raw'),
            ('scan_filtered', '/scan'),
        ],
        output='screen'
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

    # =========================================================
    # ZED Camera Launch
    # =========================================================
    human_tracker_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_human_tracker')),
        actions=[
            # ---------------------------------------------------------
            # ZED Camera Launch
            # ---------------------------------------------------------
            # IMU publishing is disabled — EKF uses wheel odometry only.
            # ZED positional tracking / visual odometry is disabled.
            # publish_tf remains disabled to avoid TF conflict with robot's odom frame.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zed_camera_launch),
                launch_arguments={
                    'camera_model': 'zed2',
                    'camera_name': 'zed2',
                    'publish_tf': 'false',
                    'publish_map_tf': 'false',
                    'publish_imu_tf': 'false',
                    'publish_urdf': 'true',
                    'use_sim_time': 'false',
                    'ros_params_override_path': zed_override_config,
                }.items()
            ),

            # ---------------------------------------------------------
            # Static Transform Bridges
            # ---------------------------------------------------------
            # camera_link  (robot URDF body frame)
            #   -> zed2_camera_link  (ZED SDK body frame, camera_name=zed2)
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='camera_link_bridge',
                output='screen',
                arguments=[
                    '--frame-id', 'camera_link',
                    '--child-frame-id', 'zed2_camera_link',
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0'
                ]
            ),

# ---------------------------------------------------------
            # Topic Relay Nodes
            # ---------------------------------------------------------
            # Republish ZED native topics under sim-compatible names:
            #   /zed2_left_camera/image_raw
            #   /zed2_left_camera/depth/image_raw
            #   /zed2_left_camera/camera_info
            Node(
                package='topic_tools',
                executable='relay',
                name='rgb_relay',
                output='screen',
                arguments=[
                    '/zed2/zed_node/rgb/color/rect/image',
                    '/zed2_left_camera/image_raw',
                ],
            ),
            Node(
                package='topic_tools',
                executable='relay',
                name='depth_relay',
                output='screen',
                arguments=[
                    '/zed2/zed_node/depth/depth_registered',
                    '/zed2_left_camera/depth/image_raw',
                ],
            ),
            Node(
                package='topic_tools',
                executable='relay',
                name='camera_info_relay',
                output='screen',
                arguments=[
                    '/zed2/zed_node/rgb/color/rect/camera_info',
                    '/zed2_left_camera/camera_info',
                ],
            ),

        ]
    )

    return LaunchDescription([
        # ==================== Launch Arguments ====================
        DeclareLaunchArgument(
            'use_human_tracker',
            default_value='true',
            description='Launch ZED2 camera with topic relays'
        ),

        # ==================== Core Robot ====================
        hardware_interface,
        controller,
        joystick,
        sllidar,
        scan_filter,
        # odom_path,
        # navigation,

        # ==================== Human Tracker (optional) ====================
        human_tracker_group,
    ])