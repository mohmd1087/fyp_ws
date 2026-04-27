#!/usr/bin/env python3
"""
Launch file for D* Lite planner node.
Included by sim_nav.launch.py when planner:=dstar.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    lethal_threshold = LaunchConfiguration('lethal_threshold', default='253')

    dstar_node = Node(
        package='fyp_dstar_lite',
        executable='dstar_planner_node.py',
        name='dstar_planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'costmap_topic': '/global_costmap/costmap',
            'lethal_threshold': lethal_threshold,
            'use_costmap': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time'),
        DeclareLaunchArgument('lethal_threshold', default_value='253',
                              description='Nav2 costmap cost value treated as obstacle'),
        dstar_node,
    ])
