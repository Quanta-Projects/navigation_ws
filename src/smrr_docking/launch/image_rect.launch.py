from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_ns = '/zed2_rear_left_raw_camera'
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Rectify node as a regular node (not composable)
    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_node',
        output='screen',
        # Subscribes to:
        #   /zed2_rear_left_raw_camera/image_raw
        #   /zed2_rear_left_raw_camera/camera_info
        # Publishes:
        #   /zed2_rear_left_raw_camera/image_rect
        remappings=[
            ('image',       f'{camera_ns}/image_raw'),
            ('camera_info', f'{camera_ns}/camera_info'),
            ('image_rect',  f'{camera_ns}/image_rect'),
        ],
        parameters=[
            {'queue_size': 10},
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rectify_node
    ])
