import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Optional: PRIME offload env for laptops with NVIDIA
    # env_use_nvidia_gpu = [
    #     SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
    #     SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
    # ]

    # Paths
    smrr_share = get_package_share_directory("smrr_description")
    smrr_prefix = get_package_prefix("smrr_description")

    # Help Gazebo resolve package:// resources
    model_path = os.path.join(smrr_prefix, "share")
    env_gz_models = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Launch args
    default_urdf = os.path.join(smrr_share, "urdf", "smrr_description.urdf.xacro")
    model = LaunchConfiguration("model")
    model_arg = DeclareLaunchArgument("model", default_value=default_urdf)

    # Optional RViz config (starts even if the file doesn't exist)
    default_rviz = os.path.join(smrr_share, "rviz", "smrr_view.rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_arg = DeclareLaunchArgument("rviz_config", default_value=default_rviz)

    # xacro → robot_description
    robot_description = ParameterValue(
        Command(["xacro ", model, " is_sim:=true"]),
        value_type=str
    )

    # Gazebo Classic
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"verbose": "false"}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        )
    )

    # robot_state_publisher holds and republishes robot_description → /tf
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # Spawn robot into Gazebo from /robot_description
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "smrr", "-topic", "robot_description"],
        output="screen",
    )

    # RViz2 (reads the same robot_description parameter)
    rviz2 = Node(
        package="rviz2", executable="rviz2",
        parameters=[{"robot_description": robot_description}],
        )


    return LaunchDescription([
        # *env_use_nvidia_gpu,
        env_gz_models,
        model_arg,
        rviz_arg,
        gzserver,
        gzclient,
        rsp,
        spawn,
        rviz2,
    ])
