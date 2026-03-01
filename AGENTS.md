# FYP Robot - ROS2 Autonomous Navigation Workspace

## Project Overview
4WD skid-steer autonomous robot using ROS 2 (Jazzy/Humble). Currently focused on **mapping (SLAM) and navigation (Nav2)**. Supports both Gazebo simulation and real hardware (ESP32 + micro-ROS).

## Workspace Structure
```
fyp_ws/
├── src/
│   ├── fyp_description/   # URDF, meshes, robot model
│   ├── fyp_bringup/       # Launch files, configs, firmware, worlds, maps
│   └── fyp_nav2/          # Nav2 params, nav launch files, maps, rviz configs
├── build/
├── install/
└── log/
```

## Packages

### fyp_description (ament_cmake)
- **URDF:** `urdf/robot.urdf` — 4-wheel skid-steer, depth camera, IMU
- **Meshes:** `meshes/*.STL`
- base_footprint → base_link has +90° yaw (maps cmd_vel +X to physical forward)
- ros2_control configured for Gazebo sim (gz_ros2_control)

### fyp_bringup (ament_cmake)
Main bringup package with launch files, configs, worlds, and ESP32 firmware.

**Key launch files:**
- `sim_with_cmd.launch.py` — full Gazebo sim (robot + controllers + sensors + EKF)
- `full_nav2.launch.py` — sim + Nav2 navigation
- `maze_sim.launch.py` — maze world sim
- `bringup_nav.launch.py` — **master real-robot launch** (`mode:=slam` or `mode:=nav`)
- `hardware_bringup.launch.py` — robot_state_publisher + micro-ROS agent
- `camera.launch.py` — Orbbec Astra Pro + depthimage_to_laserscan
- `slam_real.launch.py` — SLAM Toolbox
- `localization.launch.py` — AMCL with saved map
- `navigation.launch.py` — Nav2 stack

**Config files:**
- `config/controllers.yaml` — diff_drive_controller (wheel_separation: 0.40, wheel_radius: 0.0675)
- `config/ekf.yaml` — EKF for simulation (fuses /odom + /imu)
- `config/ekf_real.yaml` — EKF for real robot (fuses /wheel/odometry + /imu/data)

**Firmware:**
- `firmware/micro_ros_nav2_esp32.ino` — ESP32 firmware (motor control, encoders, MPU6050 IMU, micro-ROS)

### fyp_nav2 (ament_cmake)
- `params/nav2_params.yaml` — DWB controller + NavfnPlanner (main sim config)
- `params/nav2_params_rpp.yaml` — Regulated Pure Pursuit + SMAC Planner (alternative)
- `params/nav2_params_real.yaml` — real robot Nav2 config (use_sim_time: false)
- `maps/` — my_map and maze map files
- `rviz/nav2.rviz` — pre-configured RViz layout

## Build
```bash
cd ~/fyp_ws
colcon build --symlink-install
source install/setup.bash
```

Build a single package:
```bash
colcon build --packages-select fyp_bringup --symlink-install
```

## Common Workflows

### Simulation
```bash
# Full sim with navigation
ros2 launch fyp_bringup full_nav2.launch.py

# Sim only (no nav2)
ros2 launch fyp_bringup sim_with_cmd.launch.py
```

### Real Robot — SLAM (create map)
```bash
ros2 launch fyp_bringup bringup_nav.launch.py mode:=slam
# In another terminal: teleop to drive around
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Save map when done
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Real Robot — Navigation (use existing map)
```bash
ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav map:=/path/to/map.yaml
```

## TF Tree
```
map → odom → base_footprint → base_link → Front_Left_Wheel
                                         → Front_Right_Wheel
                                         → Lower_Left_Wheel
                                         → Lower_Right_Wheel
                                         → camera_link → camera_depth_frame → camera_depth_optical_frame
                                         → imu_link
                             → scan_frame
```

## Key Topics
| Topic | Type | Source |
|-------|------|--------|
| `/cmd_vel` | Twist | teleop / Nav2 |
| `/scan` | LaserScan | depthimage_to_laserscan |
| `/odom` (sim) or `/wheel/odometry` (real) | Odometry | diff_drive_controller / ESP32 |
| `/odometry/filtered` | Odometry | robot_localization EKF |
| `/imu` (sim) or `/imu/data` (real) | Imu | Gazebo / ESP32 MPU6050 |
| `/camera/depth/image_raw` | Image | depth camera |

## Robot Specs
- **Chassis:** ~0.45m x 0.32m, 5.3 kg
- **Wheels:** radius 0.0675m, separation 0.40m
- **Max velocity:** 0.3 m/s linear, 0.5 rad/s angular (controller-limited)
- **Camera:** Orbbec Astra Pro (640x480, 0.6-8.0m range, 60° HFOV)
- **IMU:** MPU6050 (gyro ±500°/s, accel ±4g)
- **Encoders:** 330 ticks/rev (front wheels only)
- **MCU:** ESP32 via micro-ROS (Serial or WiFi)

## Important Conventions
- All packages use **ament_cmake**
- Nav2 robot_base_frame is `base_footprint`
- Odometry for Nav2 comes from EKF (`/odometry/filtered`), not raw wheel odom
- Depth camera is converted to LaserScan for SLAM and Nav2 costmaps
- Real robot uses `use_sim_time: false`; simulation uses `use_sim_time: true`
- The 90° yaw between base_footprint and base_link is intentional — do not remove it
- Controllers.yaml path in URDF is hardcoded to install path — rebuild after config changes

## Hardware Pin Map (ESP32)
- **Motors:** M1(23,22) M2(26,25) M3(14,13) M4(18,19) — RPWM/LPWM pairs
- **Encoders:** FL(A=32,B=33) FR(A=16,B=17)
- **IMU I2C:** SDA=21, SCL=27
