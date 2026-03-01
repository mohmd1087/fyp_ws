#!/usr/bin/env python3
"""
Hardware bringup launch file for real robot
Launches robot state publisher and hardware driver placeholders

IMPORTANT: Replace the placeholder nodes with your actual hardware drivers:
- Wheel odometry driver: publishes nav_msgs/Odometry to /wheel/odometry
- IMU driver: publishes sensor_msgs/Imu to /imu/data
- Motor controller: subscribes to geometry_msgs/Twist on /cmd_vel
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    description_pkg = get_package_share_directory('fyp_description')

    # Robot URDF
    urdf_file = os.path.join(description_pkg, 'urdf', 'new_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1) Robot State Publisher
    # Publishes TF transforms and robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
        }]
    )

    # 2) Joint State Publisher (for non-wheel joints if any)
    # Optional: only needed if you have joints that aren't published by your drivers
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ========================================
    # PLACEHOLDER: Wheel Odometry Driver
    # ========================================
    # Replace this with your actual wheel odometry driver
    # Your driver should:
    # - Subscribe to wheel encoder data from your hardware
    # - Compute odometry using wheel kinematics
    # - Publish nav_msgs/Odometry to /wheel/odometry
    # - Frame: odom -> base_link
    # - Include covariance values (higher for skid-steer due to slippage)
    #
    # Example covariance for skid-steer:
    # pose_covariance: [0.01, 0, 0, 0, 0, 0,
    #                   0, 0.01, 0, 0, 0, 0,
    #                   0, 0, 1e6, 0, 0, 0,
    #                   0, 0, 0, 1e6, 0, 0,
    #                   0, 0, 0, 0, 1e6, 0,
    #                   0, 0, 0, 0, 0, 0.05]
    #
    # wheel_odometry_driver = Node(
    #     package='your_driver_package',
    #     executable='wheel_odometry_node',
    #     name='wheel_odometry',
    #     output='screen',
    #     parameters=[{
    #         'wheel_separation': 0.435,  # Front wheel center-to-center (measured)
    #         'wheel_radius': 0.0675,    # Wheel radius in meters (13.5cm dia)
    #         'publish_rate': 50.0,
    #         'base_frame': 'base_link',
    #         'odom_frame': 'odom',
    #     }],
    #     remappings=[
    #         ('odom', '/wheel/odometry'),
    #     ]
    # )

    # ========================================
    # PLACEHOLDER: IMU Driver
    # ========================================
    # Replace this with your actual IMU driver
    # Your driver should:
    # - Read data from your IMU hardware (I2C/SPI/Serial)
    # - Publish sensor_msgs/Imu to /imu/data
    # - Frame: imu_link (or base_link if IMU is at robot center)
    # - Include orientation, angular velocity, linear acceleration
    # - Include proper covariance values
    #
    # imu_driver = Node(
    #     package='your_imu_package',
    #     executable='imu_node',
    #     name='imu',
    #     output='screen',
    #     parameters=[{
    #         'frame_id': 'base_link',  # or 'imu_link' if separate frame
    #         'publish_rate': 100.0,
    #     }],
    #     remappings=[
    #         ('imu', '/imu/data'),
    #     ]
    # )

    # ========================================
    # PLACEHOLDER: Motor Controller
    # ========================================
    # Replace this with your motor controller driver
    # Your driver should:
    # - Subscribe to geometry_msgs/Twist on /cmd_vel
    # - Convert to individual wheel velocities for 4WD skid-steer
    # - Send commands to motor controllers (PWM, serial, CAN, etc.)
    #
    # motor_controller = Node(
    #     package='your_motor_package',
    #     executable='motor_controller_node',
    #     name='motor_controller',
    #     output='screen',
    #     parameters=[{
    #         'wheel_separation': 0.435,
    #         'wheel_radius': 0.0675,
    #         'max_linear_speed': 0.5,  # m/s
    #         'max_angular_speed': 1.0, # rad/s
    #     }],
    #     remappings=[
    #         ('cmd_vel', '/cmd_vel'),
    #     ]
    # )

    # ========================================
    # micro-ROS Agent
    # ========================================
    # Bridges ESP32 (micro-ROS) <-> ROS 2 DDS
    # ESP32 publishes: /wheel/odometry, /imu/data
    # ESP32 subscribes: /cmd_vel
    #
    # Install: sudo apt install ros-humble-micro-ros-agent
    # Or build from: https://github.com/micro-ROS/micro-ROS-Agent
    #
    # For USB serial connection:
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
        # For WiFi UDP use instead:
        # arguments=['udp4', '--port', '8888'],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (false for real robot)'
        ),
        robot_state_publisher,
        joint_state_publisher,
        micro_ros_agent,
    ])
