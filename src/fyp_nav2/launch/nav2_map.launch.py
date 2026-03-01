from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    fyp_nav2_dir = FindPackageShare('fyp_nav2')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'slam': 'False',
            'map': PathJoinSubstitution([fyp_nav2_dir, 'maps', 'my_map.yaml']),
            'params_file': PathJoinSubstitution([fyp_nav2_dir, 'params', 'nav2_params.yaml']),
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'rviz_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': 'True',
        }.items()
    )

    return LaunchDescription([
        bringup,
        rviz
    ])
