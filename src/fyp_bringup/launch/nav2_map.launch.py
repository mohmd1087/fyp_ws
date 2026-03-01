import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_pkg = get_package_share_directory('fyp_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml = os.path.join(bringup_pkg, 'maps', 'my_map.yaml')
    nav2_params = os.path.join(bringup_pkg, 'params', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': 'true',
                'params_file': nav2_params,
                'autostart': 'true'
            }.items(),
        ),
    ])
