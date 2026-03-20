#!/usr/bin/env python3
"""
Waiter Orchestrator Launch File

Launches the waiter_orchestrator node with table positions config.
Assumes Nav2 is already running (via bringup_nav.launch.py mode:=nav).

Usage:
  ros2 launch fyp_bringup waiter.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_pkg = get_package_share_directory("fyp_bringup")

    default_config = os.path.join(bringup_pkg, "config", "table_positions.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to table_positions.yaml",
        ),
        Node(
            package="fyp_bringup",
            executable="waiter_orchestrator.py",
            name="waiter_navigator",
            output="screen",
            parameters=[
                {"config_file": default_config},
            ],
        ),
    ])
