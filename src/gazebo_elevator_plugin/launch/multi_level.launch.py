from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='turtlebot3_waffle.urdf',
        description='Model type [burger, waffle, waffle_pi]'
    )

    x_pos_arg = DeclareLaunchArgument(
        'x_pos',
        default_value='10.0',
        description='Initial X position'
    )

    y_pos_arg = DeclareLaunchArgument(
        'y_pos',
        default_value='10.0',
        description='Initial Y position'
    )

    z_pos_arg = DeclareLaunchArgument(
        'z_pos',
        default_value='0.0',
        description='Initial Z position'
    )

    # Load the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_waffle.urdf'
    )

    print("URDF File Path:", urdf_file)


    robot_description = {'robot_description': urdf_file}

    # Path to the world file
    world_path = os.path.join(get_package_share_directory('gazebo_plugins'), 'model', 'moving_joint_model', 'level_simple.world')

    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")),
        launch_arguments={'world': world_path}.items())

    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"
    )))

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-x', LaunchConfiguration('x_pos'),
            '-y', LaunchConfiguration('y_pos'),
            '-z', LaunchConfiguration('z_pos'),
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        model_arg,
        x_pos_arg,
        y_pos_arg,
        z_pos_arg,
        spawn_robot_node,
        robot_state_publisher_node,
        start_gazebo_client,
        start_gazebo_server
    ])




# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, TextSubstitution
# from launch_ros.actions import Node
# import os

# def generate_launch_description():
#     model_arg = DeclareLaunchArgument(
#         'model',
#         default_value='waffle.urdf',
#         description='model type [burger, waffle, waffle_pi]'
#     )

#     x_pos_arg = DeclareLaunchArgument(
#         'x_pos',
#         default_value='10.0',
#         description='Initial X position'
#     )

#     y_pos_arg = DeclareLaunchArgument(
#         'y_pos',
#         default_value='10.0',
#         description='Initial Y position'
#     )

#     z_pos_arg = DeclareLaunchArgument(
#         'z_pos',
#         default_value='0.0',
#         description='Initial Z position'
#     )

#     # robot_description_content = Command([
#     #     'xacro ',
#     #     PathJoinSubstitution([
#     #         get_package_share_directory('turtlebot3_description'),
#     #         'urdf',
#     #         'turtlebot3_waffle.urdf.xacro'
#     #     ])
#     # ])
#     robot_description_content = PathJoinSubstitution([
#             get_package_share_directory('turtlebot3_description'),
#             'urdf',
#             'turtlebot3_waffle.urdf'
#         ])
    
#     robot_description = {'robot_description': robot_description_content}


#     # Path to the world file
#     world_path = os.path.join(get_package_share_directory('gazebo_plugins'), 'model', 'moving_joint_model', 'level_simple.world')

#     start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
#         get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")),
#         launch_arguments={'world': world_path}.items())

#     start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
#         get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"
#     )))

#     spawn_robot_node = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         name='spawn_urdf',
#         arguments=[
#             '-entity', ['turtlebot3_waffle'],
#             '-x', LaunchConfiguration('x_pos'),
#             '-y', LaunchConfiguration('y_pos'),
#             '-z', LaunchConfiguration('z_pos'),
#             '-topic', 'robot_description'
#         ],
#         output='screen'
#     )

#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description]
#     )

#     return LaunchDescription([
#         model_arg,
#         x_pos_arg,
#         y_pos_arg,
#         z_pos_arg,
#         spawn_robot_node,
#         robot_state_publisher_node,
#         start_gazebo_client,
#         start_gazebo_server
#     ])
