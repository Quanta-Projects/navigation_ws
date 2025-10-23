from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('smrr_navigation'),'config')
    maps_dir = os.path.join(get_package_share_directory('smrr_navigation'),'maps')
    map_file = os.path.join(maps_dir,'first_floor_with_docking_station.yaml')
    params_file = os.path.join(config_dir,'smrr_nav_params.yaml')
    # params_file = os.path.join(config_dir,'nav2_params.yaml')
    rviz_config= os.path.join(config_dir,'smrr_nav.rviz')
    return LaunchDescription([

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