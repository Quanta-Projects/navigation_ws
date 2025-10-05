import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Environment variables for GPU usage (matching gazebo_classic.launch.xml)
    env_use_nvidia_gpu = [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('__VK_LAYER_NV_optimus', 'NVIDIA_only'),
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES', '/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
    ]

    smrr_description = get_package_share_directory("smrr_description")
    smrr_description_prefix = get_package_prefix("smrr_description")

    # Help Gazebo find installed models/resources (matching gazebo_classic.launch.xml)
    model_path = os.path.join(smrr_description_prefix, "share")
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Use test.urdf.xacro as in gazebo_classic.launch.xml
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        smrr_description, "urdf", "test.urdf.xacro"),
                                      description="Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time":True}
                    ],
    )

    # Use same world as gazebo_classic.launch.xml
    gazebo_world_arg = DeclareLaunchArgument("world", 
                                             default_value=os.path.join(get_package_share_directory("gazebo_elevator_plugin"), 
                                                                       "model", "moving_joint_model", "level_simple.world"),
                                             description="Gazebo world file name")
    gazebo_world = LaunchConfiguration("world")

    # Use standard gazebo.launch.py as in gazebo_classic.launch.xml
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": gazebo_world
        }.items()
    )

    # Remove separate gzclient launch since gazebo.launch.py includes both server and client

    # Spawn robot with same entity name and position as gazebo_classic.launch.xml
    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "smrr_fresh",
                                   "-topic", "robot_description",
                                   "-x", "2.0", "-y", "-2.0", "-z", "0.1",
                                   "-R", "0.0", "-P", "0.0", "-Y", "0.0"
                                  ],
                        output="screen"
    )

    joystick_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("smrr_controller"), "launch", "joystick_teleop.launch.py")
        )
    )


    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("smrr_controller"), "launch", "controller.launch.py")
        )
    )

    # Add RViz with same config as gazebo_classic.launch.xml
    rviz_config_path = os.path.join(smrr_description, "config", "urdf_config.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        *env_use_nvidia_gpu,
        env_var,
        gazebo_world_arg,
        model_arg,
        start_gazebo_server,
        robot_state_publisher_node,
        spawn_robot,
        controllers,
        joystick_control,
        # rviz_node
    ])