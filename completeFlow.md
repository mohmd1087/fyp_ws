# Autonomous Waiter Robot System - Complete Technical Documentation

## Table of Contents

1. [System Overview](#1-system-overview)
2. [System Architecture](#2-system-architecture)
3. [Hardware Platform](#3-hardware-platform)
4. [Firmware - ESP32 Microcontroller](#4-firmware---esp32-microcontroller)
5. [Robot Description (URDF)](#5-robot-description-urdf)
6. [Sensor Fusion - Extended Kalman Filter](#6-sensor-fusion---extended-kalman-filter)
7. [Mapping - RTAB-Map RGB-D SLAM](#7-mapping---rtab-map-rgb-d-slam)
8. [Localization](#8-localization)
9. [Navigation - Nav2 Stack](#9-navigation---nav2-stack)
10. [D* Lite Global Path Planner](#10-d-lite-global-path-planner)
11. [Waiter Orchestrator - State Machine](#11-waiter-orchestrator---state-machine)
12. [AI Voice Agent](#12-ai-voice-agent)
13. [Local Participant - Mic/Speaker Bridge](#13-local-participant---micspeaker-bridge)
14. [Web Dashboard](#14-web-dashboard)
15. [Real-Time Communication Architecture](#15-real-time-communication-architecture)
16. [Complete End-to-End Flow](#16-complete-end-to-end-flow)
17. [TF Frame Tree](#17-tf-frame-tree)
18. [ROS 2 Topic Summary](#18-ros-2-topic-summary)
19. [Launch Commands Reference](#19-launch-commands-reference)

---

## 1. System Overview

The Autonomous Waiter Robot is a Final Year Project (FYP) that integrates autonomous mobile robot navigation with an AI-powered voice ordering system for restaurant environments. The robot autonomously navigates to customer tables upon staff dispatch, engages customers through a conversational AI voice agent to take food orders, transmits the order to a real-time web dashboard for kitchen staff, and returns to its home position after order completion.

### Key Technologies

| Layer | Technology |
|-------|-----------|
| Robot Operating System | ROS 2 Humble Hawksbill |
| Microcontroller | ESP32 with micro-ROS |
| Depth Camera | Orbbec Astra Pro (RGB-D, USB 2.0) |
| SLAM / Mapping | RTAB-Map (RGB-D visual SLAM) |
| Localization | RTAB-Map (visual) or AMCL (scan-based) |
| Global Planner | Custom D* Lite implementation |
| Local Planner | DWB (Dynamic Window Based) Controller |
| Sensor Fusion | robot_localization (EKF) |
| Voice AI | Google Gemini 2.5 Flash Native Audio via LiveKit Agents SDK |
| Web Dashboard | Next.js 16 + Prisma (PostgreSQL) + Pusher (real-time) |
| Communication Bus | Pusher (WebSocket pub/sub for dashboard-robot sync) |
| Voice Transport | LiveKit Cloud (WebRTC rooms for audio streaming) |

---

## 2. System Architecture

### High-Level Block Diagram

```
+------------------+       Pusher (WebSocket)       +-------------------+
|   Web Dashboard  | <-----------------------------> | Waiter            |
|   (Next.js on    |   robot.dispatch / robot.status | Orchestrator      |
|    Vercel)        |                                | (ROS 2 Node)      |
+------------------+                                +--------+----------+
        |                                                    |
        | POST /api/orders                                   | Nav2 goToPose()
        |                                                    v
+------------------+       LiveKit Cloud (WebRTC)   +-------------------+
|   AI Voice Agent | <-----------------------------> | Local Participant |
|   (Gemini 2.5    |   Audio streams (mic/speaker)  | (Laptop mic/spk)  |
|    Flash)         |                                |                   |
+------------------+                                +-------------------+
                                                             |
                                                    +--------v----------+
                                                    |   Robot Hardware   |
                                                    |  ESP32 + Motors    |
                                                    |  Encoders + IMU    |
                                                    |  Orbbec Astra Pro  |
                                                    +-------------------+
```

### Software Stack Layers

```
Layer 5: Application      Orchestrator State Machine + Voice Agent + Dashboard
Layer 4: Navigation       Nav2 (BT Navigator, DWB Controller, D* Lite Planner)
Layer 3: Localization     RTAB-Map (visual) or AMCL (scan-based)
Layer 2: Sensor Fusion    EKF (wheel odometry + visual odometry)
Layer 1: Drivers          micro-ROS Agent, Orbbec Camera Driver
Layer 0: Hardware         ESP32 (motors, encoders, IMU), Orbbec Astra Pro
```

---

## 3. Hardware Platform

### Robot Chassis

- **Drive type:** 4-wheel skid-steer (differential drive)
- **Mass:** 26.5 kg
- **Wheel separation (track width):** 0.435 m (front axle, center-to-center)
- **Wheel radius:** 0.0675 m (diameter 13.5 cm)
- **Wheelbase:** 0.265 m (front-to-rear axle)

### Motors and Motor Drivers

Four DC motors controlled via RPWM/LPWM dual-channel H-bridge drivers:

| Motor | Position | RPWM Pin | LPWM Pin |
|-------|----------|----------|----------|
| M1 | Front Left | GPIO 23 | GPIO 22 |
| M2 | Front Right | GPIO 26 | GPIO 25 |
| M3 | Rear Left | GPIO 14 | GPIO 13 |
| M4 | Rear Right | GPIO 18 | GPIO 19 |

- **PWM Frequency:** 1000 Hz
- **PWM Resolution:** 8-bit (0-255)
- **Stiction Threshold:** 120 PWM (minimum to overcome static friction)
- **Safety Timeout:** 250 ms (motors stop if no `/cmd_vel` received)

### Wheel Encoders

Quadrature encoders on front wheels only (rear wheels are passive followers on skid-steer):

| Encoder | Channel A Pin | Channel B Pin |
|---------|---------------|---------------|
| Front Left | GPIO 16 | GPIO 17 |
| Front Right | GPIO 32 | GPIO 33 |

- **Ticks per revolution:** 1136 (1x decode, rising edge on Channel A)
- **Meters per tick:** 0.000373 m (calculated from `2 * pi * 0.0675 / 1136`)
- **Decoding:** ISR-based interrupt on Channel A rising edge, direction from Channel B state

### IMU (MPU6050)

- **Interface:** I2C at address 0x68
- **SDA:** GPIO 21, **SCL:** GPIO 27
- **I2C Clock:** 400 kHz
- **Gyroscope Range:** +/-500 degrees per second
- **Accelerometer Range:** +/-4g
- **Digital Low Pass Filter (DLPF):** 44 Hz
- **Calibration:** 500 samples at startup (robot must be stationary)
- **Current Status:** Hardware not functional; disabled in EKF configuration

### Depth Camera (Orbbec Astra Pro)

- **Type:** RGB-D structured light camera
- **Interface:** USB 2.0 (480 Mbps bandwidth limit)
- **Depth Stream:** 640 x 480 at 30 fps
- **Color Stream:** 640 x 480 at 30 fps
- **Depth Range:** 0.6 - 8.0 m
- **Mounting Position:** Front-center of robot at height 0.34 m, offset 0.14 m forward from base_link
- **Driver:** astra_camera ROS 2 package
- **TF Publishing:** Disabled (URDF provides static transforms)

### Tray Actuation

Two GPIO pins signal a secondary ESP32 (tray controller) to open serving trays:

| Tray | GPIO Pin | Signal |
|------|----------|--------|
| Tray 1 | GPIO 4 | HIGH for 5 seconds |
| Tray 2 | GPIO 5 | HIGH for 5 seconds |

Commanded via ROS 2 topic `/waiter/tray_cmd` (std_msgs/Int32).

---

## 4. Firmware - ESP32 Microcontroller

**File:** `src/fyp_bringup/firmware/micro_ros_nav2_esp32.ino`

### Communication

The ESP32 communicates with the laptop running ROS 2 via **micro-ROS** over USB serial at 115,200 baud. The micro-ROS agent on the laptop bridges micro-ROS (XRCE-DDS) to the full ROS 2 DDS network.

### Published Topics

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `/wheel/odometry` | nav_msgs/Odometry | 10 Hz | Integrated wheel encoder odometry |
| `/imu/data` | sensor_msgs/Imu | 20 Hz | IMU angular velocity and linear acceleration |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands from Nav2 |
| `/waiter/tray_cmd` | std_msgs/Int32 | Tray open commands (1 or 2) |

### Odometry Computation

The firmware computes dead-reckoning odometry from front wheel encoder ticks at 10 Hz:

1. **Delta computation:** Reads tick count difference since last cycle for left and right front wheels.
2. **Distance conversion:** Multiplies delta ticks by meters-per-tick constant (0.000373 m/tick).
3. **Differential kinematics:**
   ```
   d_center = (d_left + d_right) / 2.0    (forward distance)
   d_theta  = (d_right - d_left) / WHEEL_SEPARATION  (heading change)
   ```
4. **Skid-steer slip compensation:** During pure rotation (when `|d_theta| > 0.02 rad` and `|d_center| < |d_theta| * WHEEL_SEPARATION * 0.3`), the translational component `d_center` is zeroed out. This prevents phantom forward/backward drift during in-place turns caused by wheel slip on smooth floors.
5. **Pose integration (midpoint method):**
   ```
   theta_mid = odom_theta + d_theta * 0.5
   odom_x += d_center * cos(theta_mid)
   odom_y += d_center * sin(theta_mid)
   odom_theta += d_theta
   ```
6. **Velocity computation:**
   ```
   vx = d_center / dt
   v_theta = d_theta / dt
   ```

The odometry message publishes both the integrated pose (in the `pose` field) and instantaneous velocities (in the `twist` field), with conservative covariance values reflecting skid-steer uncertainty (0.02 m^2 for position, 0.08 rad^2 for yaw).

### Motor Control (Skid-Steer Kinematics)

When a `/cmd_vel` message arrives, the firmware converts the desired linear and angular velocities into individual wheel speeds:

```
left_velocity  = linear_x - (angular_z * WHEEL_SEPARATION * 0.5)
right_velocity = linear_x + (angular_z * WHEEL_SEPARATION * 0.5)
```

Each velocity is then normalized to a PWM duty cycle (0-255), with a minimum stiction threshold of 120 PWM to ensure the motors overcome static friction. Motor direction is controlled by selecting the RPWM (forward) or LPWM (reverse) channel.

### Time Synchronization

The firmware synchronizes its clock with the ROS 2 agent every 30 seconds, ensuring odometry and IMU message timestamps are consistent with the rest of the ROS 2 system. If time synchronization fails, the firmware falls back to its local `millis()` timer.

---

## 5. Robot Description (URDF)

**File:** `src/fyp_description/urdf/new_robot.urdf`

The robot's kinematic structure is described in a URDF (Unified Robot Description Format) file loaded by the `robot_state_publisher` node, which publishes the static transform tree.

### TF Frame Hierarchy

```
base_footprint (ground-projected virtual frame)
  └── base_link (center of gravity, 26.5 kg)
        ├── Front_Left_Wheel   (x=+0.133, y=+0.218, z=+0.024)
        ├── Front_Right_Wheel  (x=+0.133, y=-0.218, z=+0.024)
        ├── Lower_Left_Wheel   (x=-0.133, y=+0.215, z=+0.024)
        ├── Lower_Right_Wheel  (x=-0.133, y=-0.215, z=+0.024)
        ├── camera_link        (x=+0.140, y=0.000, z=+0.340)
        │     ├── camera_color_optical_frame  (rpy: -90, 0, -90 deg)
        │     └── camera_depth_optical_frame  (rpy: -90, 0, -90 deg)
        ├── imu_link           (x=0.000, y=0.000, z=+0.150)
        └── scan_frame         (x=+0.140, y=0.000, z=+0.340)
```

The optical frames follow the REP-103 convention: Z-axis forward, X-axis right, Y-axis down. This is achieved by a -90 degree pitch and -90 degree yaw rotation from the body-aligned camera_link frame.

---

## 6. Sensor Fusion - Extended Kalman Filter

**File:** `src/fyp_bringup/config/ekf_real.yaml`

The robot_localization package provides an Extended Kalman Filter (EKF) that fuses multiple odometry sources into a single, robust odometry estimate published on `/odometry/filtered`. The EKF also publishes the `odom -> base_footprint` transform.

### Configuration

| Parameter | Value |
|-----------|-------|
| Frequency | 50 Hz |
| 2D Mode | true (constrains Z, roll, pitch to zero) |
| Publish TF | true (odom -> base_footprint) |
| World Frame | odom |

### Input Sources

#### Source 1: Wheel Odometry (`/wheel/odometry`)

| State | Fused | Rationale |
|-------|-------|-----------|
| Position (x, y) | No | Skid-steer slip causes position drift during turns |
| Yaw | No | Use velocity-only approach for better integration |
| Linear velocity (vx) | Yes | Reliable from encoder tick rates |
| Angular velocity (vyaw) | Yes | Computed from differential wheel speeds |

- **Mode:** Differential + Relative (EKF integrates velocities itself rather than trusting the accumulated pose from the microcontroller, preventing drift accumulation)

#### Source 2: Visual Odometry (`/vo/odometry` from rgbd_odometry node)

| State | Fused | Rationale |
|-------|-------|-----------|
| Position (x, y) | No | Visual odometry drifts over time |
| Yaw | No | Use velocity-only approach |
| Linear velocity (vx) | Yes | Camera-based motion estimate |
| Angular velocity (vyaw) | Yes | Camera-based rotation estimate |

- **Mode:** Differential + Relative (same as wheel odometry; only velocity deltas are fused)

#### Source 3: IMU (`/imu/data`) - Currently Disabled

The MPU6050 IMU hardware is currently non-functional (publishing static values across all axes). When operational, the EKF would fuse:
- Angular velocity (vyaw) from the gyroscope Z-axis
- Linear acceleration (ax) for straight-line stability

### Output

The EKF publishes:
- `/odometry/filtered` (nav_msgs/Odometry) at 50 Hz
- TF: `odom -> base_footprint`

This fused odometry is consumed by:
- RTAB-Map (for visual SLAM/localization)
- Nav2 (for robot pose tracking in the costmap and controller)

---

## 7. Mapping - RTAB-Map RGB-D SLAM

**File:** `src/fyp_bringup/launch/rtabmap.launch.py`

RTAB-Map (Real-Time Appearance-Based Mapping) provides simultaneous localization and mapping (SLAM) using RGB-D camera data. The system supports two mapping modes: robot-based mapping (with teleop) and handheld mapping (without the robot).

### Robot-Based Mapping

In robot-based mapping, the full hardware stack is running. The mapping pipeline uses three ROS 2 nodes:

#### Node 1: rgbd_odometry (Visual Odometry)

Computes camera-based motion estimates from consecutive RGB-D frame pairs using visual feature matching.

| Parameter | Value |
|-----------|-------|
| Package | rtabmap_odom |
| Frame ID | base_footprint |
| Odom Frame ID | vo_odom |
| Publish TF | false (EKF owns odom -> base_footprint) |
| Strategy | Frame-to-Map (Odom/Strategy: 0) |
| Min Inliers | 10 |
| Force 3DoF | true (2D robot) |
| Approx Sync | true (0.05s max interval) |

The visual odometry output is published to `/vo/odometry` and fed into the EKF as a secondary velocity source, complementing the wheel encoders. This is particularly useful when the wheels slip (smooth floors, turns) or when encoder counts are noisy.

#### Node 2: rtabmap (SLAM Engine)

The core SLAM node that builds and maintains the map.

| Parameter | Value |
|-----------|-------|
| Frame ID | base_footprint |
| Map Frame ID | map |
| Odom Frame ID | odom |
| Odometry Source | /odometry/filtered (from EKF) |
| Database Path | ~/.ros/rtabmap.db |
| Force 3DoF | true |
| Feature Detection | Max 400 features, Min 12 inliers |
| Update Thresholds | 0.01 m (linear), 0.01 rad (angular) |

**2D Occupancy Grid Generation:**

RTAB-Map generates a 2D occupancy grid from the 3D depth data, suitable for Nav2 navigation:

| Parameter | Value |
|-----------|-------|
| Grid/FromDepth | true |
| Grid/CellSize | 0.05 m (5 cm resolution) |
| Grid/RangeMax | 4.0 m |
| Grid/MaxGroundHeight | 0.1 m |
| Grid/MaxObstacleHeight | 1.5 m |
| Grid/NormalsSegmentation | true (floor detection) |
| Grid/3D | false (2D grid only) |

The occupancy grid is published on `/map` (nav_msgs/OccupancyGrid) with TRANSIENT_LOCAL QoS durability.

#### Node 3: rtabmap_viz (Visualization)

Provides a real-time 3D visualization window showing the point cloud map, loop closure detections, and camera trajectory.

### Handheld Mapping

**File:** `src/fyp_bringup/launch/rtabmap_handheld_mapping.launch.py`

For environments where the robot hardware is not available, mapping can be performed by holding the Orbbec Astra Pro camera by hand and walking through the space.

Key differences from robot-based mapping:
- Base frame: `camera_link` (not `base_footprint`)
- rgbd_odometry publishes the `odom -> camera_link` TF directly (no EKF)
- Static TF: `map -> odom` (identity transform)
- Reduced camera resolution: 320 x 240 at 15 fps (USB 2.0 bandwidth constraint)
- No wheel odometry; visual odometry is the sole motion source

### Map Management

Maps are stored as SQLite databases (`.db` files):

```bash
# Build a new map (robot-based)
LOCALIZATION_MODE=rtabmap RTABMAP_MAPPING=true ros2 launch fyp_bringup bringup_nav.launch.py mode:=mapping

# Save the map after Ctrl+C (wait for "Saving memory... done!")
cp ~/.ros/rtabmap.db ~/fyp_ws/src/fyp_bringup/maps/rtabmap_restaurant.db

# Load a specific map for navigation
cp ~/fyp_ws/src/fyp_bringup/maps/rtabmap_restaurant.db ~/.ros/rtabmap.db
LOCALIZATION_MODE=rtabmap ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav
```

---

## 8. Localization

The system supports two localization methods, selectable via the `LOCALIZATION_MODE` environment variable.

### RTAB-Map Visual Localization (LOCALIZATION_MODE=rtabmap)

In localization mode, RTAB-Map loads the pre-built map database in read-only mode (`Mem/IncrementalMemory: false`, `Mem/InitWMWithAllNodes: true`). It matches current camera frames against stored visual features to determine the robot's position within the map, publishing the `map -> odom` transform.

This mode does not require a 2D laser scan, making it suitable for environments where converting depth to a fake laser scan is unreliable.

### AMCL Scan-Based Localization (LOCALIZATION_MODE=amcl)

**File:** `src/fyp_bringup/launch/camera.launch.py`

In AMCL mode, the depth camera image is converted to a 2D laser scan through a processing pipeline:

1. **depth_sync_node:** Fixes a ~400 ms timestamp mismatch between the depth image and its camera_info message.
2. **depthimage_to_laserscan:** Converts the synchronized depth image into a `sensor_msgs/LaserScan` message on `/scan`, using a 60-pixel horizontal slice of the depth image.

The Adaptive Monte Carlo Localization (AMCL) particle filter uses this fake laser scan along with a pre-built 2D occupancy grid map to localize the robot.

### Localization Mode Comparison

| Feature | RTAB-Map | AMCL |
|---------|----------|------|
| Sensor Input | RGB-D images | 2D laser scan (from depth) |
| Map Format | SQLite database (.db) | PGM image + YAML |
| Feature Matching | Visual features (SURF/ORB) | Scan-to-map correlation |
| Loop Closure | Yes | No |
| 3D Awareness | Yes (projects to 2D grid) | No (2D only) |
| Fake Scan Required | No | Yes |

---

## 9. Navigation - Nav2 Stack

**File:** `src/fyp_nav2/params/nav2_params_rtabmap.yaml`

### Behavior Tree Navigator

The BT Navigator orchestrates the navigation process using a behavior tree defined in `navigate_to_pose_w_smoothing.xml`. It manages the interaction between the global planner, local controller, and recovery behaviors.

| Parameter | Value |
|-----------|-------|
| Global Frame | map |
| Robot Base Frame | base_footprint |
| Odometry Topic | /odometry/filtered |
| BT Loop Duration | 10 ms |
| Server Timeout | 20 s |

### DWB Local Controller

The Dynamic Window Based (DWB) local planner generates smooth velocity commands to follow the global path while avoiding obstacles.

#### Velocity Limits (Tuned for Smooth Waiter Motion)

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| max_vel_x | 0.12 m/s | Slow walking pace, gentle for carrying trays |
| max_vel_theta | 0.2 rad/s | 11.5 deg/s, very gentle turns |
| acc_lim_x | 0.08 m/s^2 | ~1.5 s ramp to full speed |
| decel_lim_x | -0.1 m/s^2 | ~1.2 s to stop |
| acc_lim_theta | 0.15 rad/s^2 | Gradual turn initiation |
| decel_lim_theta | -0.2 rad/s^2 | Smooth turn deceleration |

#### Trajectory Evaluation

| Parameter | Value |
|-----------|-------|
| Simulation Time | 2.0 s |
| Linear Samples | 20 |
| Angular Samples | 40 |
| Linear Granularity | 0.05 m |
| Angular Granularity | 0.025 rad |

#### Cost Function Critics

| Critic | Weight | Purpose |
|--------|--------|---------|
| PathAlign | 32.0 | Align robot heading with path direction |
| PathDist | 32.0 | Minimize distance from path |
| GoalDist | 24.0 | Progress toward goal |
| Oscillation | 1.0 | Penalize back-and-forth jitter |
| BaseObstacle | 0.02 | Low-weight collision avoidance |

`PathAlign.forward_point_distance: 0.325 m` ensures the robot looks 32.5 cm ahead to determine its heading alignment, preventing the initial wrong-direction movement that occurred with shorter lookahead distances.

#### Goal Tolerances

| Parameter | Value |
|-----------|-------|
| xy_goal_tolerance | 0.5 m |
| yaw_goal_tolerance | 0.524 rad (30 degrees) |

### Velocity Smoother

Provides smooth acceleration and deceleration profiles to prevent jerky motion:

| Parameter | Value |
|-----------|-------|
| Smoothing Frequency | 20 Hz |
| Max Acceleration | [0.08, 0.0, 0.15] m/s^2 |
| Max Deceleration | [-0.1, 0.0, -0.2] m/s^2 |
| Max Velocity | [0.12, 0.0, 0.2] m/s |
| Deadband Velocity | [0.01, 0.0, 0.01] |
| Feedback Mode | OPEN_LOOP |

The deadband velocity filter ignores commands below 0.01 m/s, eliminating micro-jitter at start and stop.

### Costmap Configuration

#### Local Costmap (Obstacle Avoidance)

| Parameter | Value |
|-----------|-------|
| Rolling Window | 3.0 x 3.0 m |
| Resolution | 0.05 m/cell |
| Update Frequency | 5 Hz |
| Obstacle Source | PointCloud2 from `/camera/depth/points` |
| Min Obstacle Height | 0.25 m (ignores floor noise) |
| Max Obstacle Height | 1.5 m |
| Obstacle Max Range | 2.0 m |
| Inflation Radius | 0.35 m |

#### Global Costmap (Path Planning)

| Parameter | Value |
|-----------|-------|
| Map Source | Static layer from `/map` topic |
| Resolution | 0.05 m/cell |
| Update Frequency | 1 Hz |
| Inflation Radius | 0.30 m |

### Recovery Behaviors

When the robot gets stuck, Nav2 can invoke recovery behaviors:
- **Spin:** Rotate in place to clear obstacles
- **Backup:** Drive backwards
- **Drive on Heading:** Move in a specific direction
- **Wait:** Pause and wait for dynamic obstacles to clear

---

## 10. D* Lite Global Path Planner

**File:** `src/fyp_dstar_lite/fyp_dstar_lite/dstar_planner_node.py`

### Overview

A custom global path planner implementing the D* Lite algorithm as a Nav2 action server (`ComputePathToPose`). D* Lite is an incremental heuristic search algorithm that efficiently replans when the environment changes, making it well-suited for dynamic restaurant environments.

### Algorithm Pipeline

1. **Costmap Reception:** Subscribes to `/global_costmap/costmap` with TRANSIENT_LOCAL QoS. Cells with cost >= 65 (lethal threshold) or < 0 (unknown) are marked as obstacles.

2. **Start/Goal Resolution:** The start pose is obtained from the robot's current position via TF lookup (`base_footprint` in `map` frame). The goal comes from the action request.

3. **Grid Preparation:** Ensures start and goal cells are obstacle-free (clears them if necessary to prevent planning failures).

4. **D* Lite Search:** Executes the D* Lite algorithm with 8-neighbor connectivity on the occupancy grid, producing a raw waypoint sequence.

5. **Path Post-Processing:**
   - **Line-of-sight pruning:** Removes intermediate waypoints that are directly visible from the current waypoint, creating straighter paths.
   - **Ramer-Douglas-Peucker (RDP) simplification:** With epsilon = 3.0 grid cells (~15 cm), collapses near-collinear points to further smooth the path.

6. **Orientation Assignment:** Each waypoint's orientation is set to face the next waypoint. The final waypoint receives the goal orientation from the request.

7. **Publication:** The path is published to both the action result and `/dstar_path` for RViz visualization.

---

## 11. Waiter Orchestrator - State Machine

**File:** `src/fyp_bringup/scripts/waiter_orchestrator.py`

The orchestrator is the central coordinator that manages the robot's mission lifecycle, bridging the navigation stack, voice agent, and web dashboard.

### State Machine

```
   +---------+     dispatch      +---------------------+
   |  IDLE   | ----------------> | NAVIGATING_TO_TABLE |
   +---------+                   +----------+----------+
       ^                                    |
       |                          Nav2 SUCCEEDED
       |                                    v
       |    NAVIGATING_HOME         +-------------+
       +------<---------+---------  |  AT_TABLE   |
              Nav2 done |           +------+------+
                        |                  |
                  +-----+-------+    order_complete
                  | NAVIGATING  |          |
                  |   _HOME     | <--------+
                  +-------------+
```

### States

| State | Description | Exit Condition |
|-------|-------------|----------------|
| IDLE | Waiting for dispatch from dashboard | Dispatch event received |
| NAVIGATING_TO_TABLE | Robot moving to table via Nav2 | Nav2 TaskResult.SUCCEEDED |
| AT_TABLE | At table, LiveKit room active, taking order | Order complete event |
| NAVIGATING_HOME | Returning to home position | Nav2 task complete |

### HTTP API (Port 5050)

| Endpoint | Method | Body | Description |
|----------|--------|------|-------------|
| `/dispatch` | POST | `{table_id, tray?}` | Dispatch robot to a table |
| `/order_complete` | POST | `{room_name, order_id}` | Signal order completion |
| `/status` | GET | - | Get current state, table, room |

### Pusher Integration (Remote Dispatch)

The orchestrator subscribes to the Pusher "robot" channel for `robot.dispatch` events. When the dashboard staff clicks a dispatch button, the event flows through Pusher to the orchestrator, which triggers the state machine.

The orchestrator also publishes status updates back to the dashboard by POSTing to the dashboard's `/api/robot/status` endpoint, which republishes via Pusher for real-time UI updates.

### LiveKit Room Management

When the robot arrives at a table (`AT_TABLE` state), the orchestrator creates a LiveKit room via the Server SDK:

```python
await lk.room.create_room(
    livekit_api.CreateRoomRequest(
        name=room_name,       # e.g., "table-1"
        empty_timeout=3600,   # 1 hour
    )
)
```

This room serves as the rendezvous point for:
- The **AI voice agent** (auto-dispatched by LiveKit when a room is created)
- The **local participant** (laptop mic/speaker bridge)

### ROS 2 Integration

| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `/waiter/state` | Publish | std_msgs/String | JSON state + current_table |
| `/waiter/tray_cmd` | Publish | std_msgs/Int32 | Tray open command (1 or 2) |

### Configuration

Table positions are loaded from `config/table_positions.yaml`:

```yaml
home:
  x: 15.43
  y: 1.91
  yaw: -1.57    # facing -Y direction

tables:
  table-1:
    x: 3.638
    y: -1.625
    yaw: -0.534  # approximately -30 degrees
```

Environment variables are loaded from `config/orchestrator.env`:
- `DASHBOARD_URL` - Dashboard base URL for status publishing
- `AGENT_API_KEY` - Shared authentication key
- `PUSHER_KEY`, `PUSHER_CLUSTER` - Pusher connection credentials
- `LIVEKIT_URL`, `LIVEKIT_API_KEY`, `LIVEKIT_API_SECRET` - LiveKit credentials for room creation

### Threading Model

| Thread | Responsibility |
|--------|---------------|
| Main (ROS 2 spin) | Timer callbacks, state machine ticks (1 Hz) |
| HTTP server (daemon) | Serves `/dispatch`, `/order_complete`, `/status` |
| Background threads | LiveKit room creation, dashboard status publishing |
| Pusher thread | Receives remote dispatch events |

All shared state is protected by `threading.Lock()`.

---

## 12. AI Voice Agent

**File:** `src/AI Voice(FYP)/fyp/agent.py`

### Overview

The voice agent is an AI-powered restaurant waiter named **AMORA** that takes customer food orders through natural voice conversation. It uses Google's Gemini 2.5 Flash Native Audio model for real-time audio understanding and generation, deployed through the LiveKit Agents SDK.

### Technology Stack

| Component | Technology |
|-----------|-----------|
| LLM | Google Gemini 2.5 Flash Native Audio (Preview 12-2025) |
| Voice | "Puck" (Gemini voice preset) |
| Temperature | 0.4 (semi-deterministic) |
| Framework | LiveKit Python Agents SDK (AgentServer + AgentSession) |
| Audio Processing | LiveKit room_io with BVC noise cancellation |

### Agent Behavior

The agent is initialized with system instructions that define its persona and workflow:

1. **Greet the guest** as a professional waiter at Saffron Garden restaurant
2. **Ask about order type** (dine-in, takeaway, or delivery)
3. **Inquire about allergies** before menu discussion
4. **Take the order conversationally** using the available menu tools
5. **Repeat the complete order** and ask for confirmation before finalizing
6. **Finalize the order** which automatically sends it to the kitchen dashboard

### Restaurant Information

```
Restaurant: Saffron Garden
Location: Main Boulevard, Lahore
Cuisine: Pakistani, BBQ, Continental
Hours: Mon-Thu 12-11pm, Fri-Sat 12-12am, Sun 12-10:30pm
Spice Levels: Mild / Medium / Spicy
Customization: Can adjust spice, remove onions/garlic on request
```

### Menu

| Category | Item | Price (PKR) | Tags | Allergens |
|----------|------|-------------|------|-----------|
| Starters | Chicken Corn Soup | 450 | popular | egg (optional) |
| Starters | Dynamite Shrimp | 1,190 | spicy | shellfish, dairy |
| Mains | Chicken Karahi (Half) | 1,850 | signature | - |
| Mains | Beef Burger | 990 | kids_friendly | gluten, dairy |
| Mains | Alfredo Pasta | 1,090 | mild | dairy, gluten |
| BBQ | Chicken Tikka (1 pc) | 720 | - | - |
| BBQ | Seekh Kebab (2 pcs) | 690 | - | - |
| Drinks | Mint Margarita | 390 | - | - |
| Drinks | Cold Coffee | 520 | - | dairy |
| Desserts | Kunafa | 890 | - | dairy, gluten |

### Function Tools

The agent has four function tools available for structured interactions:

#### 1. get_restaurant_info()
Returns restaurant name, location, hours, cuisine types, and policies.

#### 2. get_menu(category?)
Returns the full menu or a filtered category. Each item includes name, price, description, tags, and allergens.

#### 3. get_item_details(item_id)
Returns detailed information about a specific menu item including full description and allergen information.

#### 4. finalize_order(...)
The primary order submission tool. Accepts a complex structured input:

**Input Parameters:**
- `customer_name` (optional string)
- `order_type` (required: "dine_in", "takeaway", or "delivery")
- `party_size` (optional integer)
- `table_number` (optional string)
- `phone` (optional string)
- `notes` (optional string)
- `allergies` (optional string)
- `spice_level` (required: "mild", "medium", "spicy", or "not_specified")
- `items` (required array of `{item_id, qty, modifications}`)

**Processing Pipeline:**
1. Normalizes all inputs (handles string "null", fuzzy matches item names to IDs at 72% cutoff)
2. Enriches each item with pricing, allergen data, category, and line totals
3. Calculates subtotal and 10% service charge
4. Generates a unique order ID (`LK-{10 hex chars}`)
5. Builds a comprehensive JSON order structure
6. Saves to `last_order.json` locally
7. **POSTs the order to the dashboard** at `/api/orders` with the shared agent API key
8. Returns a confirmation summary to the voice agent

### LiveKit Integration

```python
server = AgentServer()

@server.rtc_session()
async def waiter_agent(ctx: agents.JobContext):
    session = AgentSession(
        llm=google.beta.realtime.RealtimeModel(
            model="gemini-2.5-flash-native-audio-preview-12-2025",
            voice="Puck",
            temperature=0.4,
        )
    )
    await session.start(
        room=ctx.room,
        agent=RestaurantWaiter(),
        room_options=room_io.RoomOptions(
            audio_input=room_io.AudioInputOptions(
                noise_cancellation=lambda _: noise_cancellation.BVC()
            )
        ),
    )
    await ctx.connect()
    await session.generate_reply(instructions="Greet the guest...")
```

When a LiveKit room is created by the orchestrator and a participant joins, LiveKit's infrastructure automatically dispatches a job to the running agent server, which connects to the room and begins the conversation.

---

## 13. Local Participant - Mic/Speaker Bridge

**File:** `src/AI Voice(FYP)/fyp/local_participant.py`

### Purpose

The local participant runs on the laptop connected to (or mounted on) the robot. It bridges the laptop's physical microphone and speakers into the LiveKit room, enabling face-to-face voice interaction between the customer and the AI agent without requiring any app or phone.

### Audio Configuration

| Parameter | Value |
|-----------|-------|
| Sample Rate | 48,000 Hz |
| Channels | 1 (mono) |
| Frame Duration | 20 ms |
| Samples per Frame | 960 |
| Audio Library | sounddevice (PortAudio) |

### Operational Flow

1. **Polling Phase:** Continuously polls the orchestrator's `/status` endpoint every 2 seconds, waiting for the robot state to become `AT_TABLE`.

2. **Connection Phase:** When `AT_TABLE` is detected:
   - Generates a LiveKit access token with room join permissions
   - Connects to the LiveKit room as participant identity `robot-customer`
   - Publishes a local microphone audio track

3. **Audio Streaming Phase:**
   - **Microphone -> Room:** A sounddevice `RawInputStream` captures audio from the laptop microphone. A callback pushes raw audio bytes into a queue, and an async loop converts them to LiveKit `AudioFrame` objects and publishes them to the room.
   - **Room -> Speaker:** When the voice agent's audio track is received (`track_subscribed` event), an async iterator reads incoming `AudioFrame` objects and writes them to a sounddevice `RawOutputStream` connected to the laptop speakers.

4. **Monitoring Phase:** During the session, the local participant continues polling the orchestrator. When the state changes from `AT_TABLE` (robot is leaving), it disconnects from the LiveKit room, closes audio streams, and returns to the polling phase.

### Token Generation

```python
token = (
    livekit_api.AccessToken(LIVEKIT_API_KEY, LIVEKIT_API_SECRET)
    .with_identity("robot-customer")
    .with_grants(livekit_api.VideoGrants(room_join=True, room=room_name))
)
```

---

## 14. Web Dashboard

**File:** `src/AI Voice(FYP)/fyp/dashboard/`

### Technology Stack

| Component | Technology |
|-----------|-----------|
| Framework | Next.js 16.1.6 (React 19.2.3) |
| UI Components | Shadcn/UI (Radix UI + TailwindCSS 4) |
| Database | PostgreSQL (Neon hosted) via Prisma 6.19.2 ORM |
| Real-time | Pusher (WebSocket pub/sub) |
| Authentication | JWT (jose) + bcryptjs password hashing |
| Notifications | sonner (toast notifications) |
| Hosting | Vercel (serverless deployment) |

### Pages and Components

#### Login Page (`/login`)
- Email + password authentication
- Rate limited: 10 attempts per minute per IP
- JWT stored in httpOnly secure cookie (8-hour expiry)
- Redirects to `/dashboard` on success

#### Dashboard Page (`/dashboard`)
The main operational interface for kitchen staff:

1. **Robot Dispatch Panel** (`RobotDispatch` component):
   - Displays current robot state with color-coded badges:
     - IDLE (gray), NAVIGATING_TO_TABLE (blue), AT_TABLE (green), NAVIGATING_HOME (yellow)
   - Dispatch buttons for each table (e.g., "table-1", "table-2")
   - Real-time state updates via Pusher subscription to "robot" channel
   - On dispatch click: POSTs to `/api/robot/dispatch` with `table_id`

2. **Order Board** (`OrderBoard` component):
   - Tabbed view: PENDING orders and COMPLETED orders
   - Search filter by order ID, customer name, or table number
   - Real-time order arrival via Pusher subscription to "orders" channel
   - Toast notification on new order arrival

3. **Order Ticket** (`OrderTicket` component):
   Each order card displays:
   - Order ID and status badge (PENDING/COMPLETED)
   - Customer name and phone number
   - Order type (dine-in/takeaway/delivery), table number, party size
   - Itemized list with quantities, names, modifications, and line totals
   - Total amount with currency (PKR)
   - Allergy warnings (highlighted in red)
   - Spice level and special notes
   - Action buttons: "Mark Complete" / "Reopen"

### API Routes

| Route | Method | Auth | Purpose |
|-------|--------|------|---------|
| `/api/orders` | POST | X-Agent-Key | Voice agent submits new order |
| `/api/orders` | GET | JWT cookie | Staff fetches order list |
| `/api/orders/[id]` | GET | JWT cookie | Fetch single order |
| `/api/orders/[id]` | PATCH | JWT cookie | Update order status |
| `/api/robot/dispatch` | POST | JWT cookie | Staff dispatches robot |
| `/api/robot/status` | POST | X-Agent-Key | Orchestrator pushes status |
| `/api/auth/login` | POST | None | Staff login |

### Database Schema (Prisma)

```
Order {
  id            String    (agent-generated: LK-XXXXXXXXXX)
  tableNumber   String?
  orderType     String    (dine_in / takeaway / delivery)
  status        String    (pending / completed)
  customer      Json      {name, phone}
  dineIn        Json?     {party_size, table_number}
  preferences   Json      {spice_level, allergies, notes}
  items         Json[]    [{name, qty, unit_price, modifications, allergens, category, line_total}]
  totals        Json      {subtotal, service_charge, service_rate, total, currency}
  rawPayload    Json
  createdAt     DateTime
  updatedAt     DateTime
}
```

### Middleware

- **Public routes:** `/login`, `/api/auth/*`
- **Agent routes:** POST to `/api/orders` (authenticated via X-Agent-Key header)
- **Protected routes:** All others require valid JWT cookie; redirects to `/login` if missing

---

## 15. Real-Time Communication Architecture

### Pusher (Dashboard <-> Orchestrator)

Pusher provides the WebSocket backbone for real-time communication between the cloud-hosted dashboard and the on-premise robot.

#### Channels and Events

| Channel | Event | Publisher | Subscriber | Payload |
|---------|-------|-----------|-----------|---------|
| robot | robot.dispatch | Dashboard API | Orchestrator | {table_id, tray?} |
| robot | robot.status | Dashboard API | Dashboard UI | {state, current_table, timestamp} |
| orders | order.created | Dashboard API | Dashboard UI | {order data} |
| orders | order.updated | Dashboard API | Dashboard UI | {id, status} |

#### Flow: Staff Dispatch

```
Staff clicks "table-1" button
  → Dashboard client POSTs to /api/robot/dispatch
    → Dashboard server publishes to Pusher "robot" channel: robot.dispatch
      → Orchestrator's Pusher client receives event
        → handle_dispatch("table-1") triggers state machine
```

#### Flow: Status Update

```
Orchestrator state changes (e.g., NAVIGATING_TO_TABLE → AT_TABLE)
  → _publish_status_to_dashboard() POSTs to Dashboard /api/robot/status
    → Dashboard server publishes to Pusher "robot" channel: robot.status
      → Dashboard UI updates robot state badge in real-time
```

### LiveKit (Voice Agent <-> Local Participant)

LiveKit provides WebRTC-based real-time audio streaming between the AI voice agent and the local participant (laptop mic/speaker).

#### Room Lifecycle

```
1. Robot arrives at table → Orchestrator creates LiveKit room "table-1"
2. Voice agent server auto-dispatches agent to room "table-1"
3. Local participant detects AT_TABLE, joins room as "robot-customer"
4. Audio flows: Customer speaks → Mic → LiveKit → Agent processes → LiveKit → Speaker
5. Robot leaves → Local participant detects state change → Disconnects
6. Room auto-closes after empty_timeout (3600s)
```

---

## 16. Complete End-to-End Flow

### Phase 1: System Startup

```
Terminal 1: Navigation Stack
  $ LOCALIZATION_MODE=rtabmap ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

  Startup sequence:
    0s  → robot_state_publisher (URDF → TF tree)
    0s  → micro_ros_agent (ESP32 serial bridge)
    2s  → Orbbec Astra Pro camera
    3s  → EKF node (wheel + visual odometry fusion)
    5s  → RTAB-Map (loads map database, visual localization)
    6s  → RViz (visualization)
    15s → D* Lite planner (global path planning)
    18s → Nav2 (controller, costmap, behavior tree)

Terminal 2: Waiter Orchestrator
  $ LOCALIZATION_MODE=rtabmap ros2 launch fyp_bringup waiter.launch.py

  → Loads table positions from YAML
  → Starts HTTP server on port 5050
  → Connects to Pusher for remote dispatch
  → Waits for Nav2 to become active
  → State: IDLE

Terminal 3: AI Voice Agent
  $ cd ~/fyp_ws/src/AI\ Voice\(FYP\)/fyp && python agent.py dev

  → Connects to LiveKit Cloud
  → Registers as agent server
  → Waits for room dispatch

Terminal 4: Local Participant (Laptop Mic/Speaker)
  $ cd ~/fyp_ws/src/AI\ Voice\(FYP\)/fyp && python local_participant.py

  → Polls orchestrator status every 2s
  → Waits for AT_TABLE state
```

### Phase 2: Dispatch

```
Kitchen staff opens dashboard (https://fyp-ws.vercel.app/dashboard)
  → Sees robot status: IDLE
  → Clicks "Dispatch to table-1"
  → Dashboard POSTs to /api/robot/dispatch
  → Pusher event → Orchestrator receives dispatch

Orchestrator:
  → State: IDLE → NAVIGATING_TO_TABLE
  → Loads table-1 pose: (x=3.638, y=-1.625, yaw=-0.534)
  → Calls navigator.goToPose(goal)
  → Publishes status to dashboard → Badge changes to "NAVIGATING_TO_TABLE"
```

### Phase 3: Navigation

```
Nav2 receives goal:
  → D* Lite plans global path on /global_costmap
  → Path post-processing: LOS pruning + RDP simplification
  → DWB local controller follows path:
    - Velocity smoother ensures gentle acceleration (0.08 m/s²)
    - Max speed 0.12 m/s (slow walking pace)
    - Local costmap detects dynamic obstacles via depth pointcloud
    - PathAlign critic keeps robot oriented along path
  → /cmd_vel → velocity_smoother → ESP32 motors

EKF continuously fuses:
  → Wheel encoder velocities (vx, vyaw) at 10 Hz
  → Visual odometry velocities (vx, vyaw) from rgbd_odometry
  → Publishes fused /odometry/filtered at 50 Hz

RTAB-Map continuously:
  → Matches camera frames against stored map features
  → Publishes map → odom transform
  → Maintains localization within the environment
```

### Phase 4: Arrival at Table

```
Nav2 goal checker:
  → Robot position within 0.5m of table AND yaw within 30°
  → TaskResult.SUCCEEDED

Orchestrator:
  → State: NAVIGATING_TO_TABLE → AT_TABLE
  → Creates LiveKit room "table-1"
  → Publishes status → Dashboard badge changes to "AT_TABLE"
  → (Optional) Publishes tray command to open serving tray

Local Participant:
  → Detects AT_TABLE state via polling
  → Generates LiveKit token for room "table-1"
  → Connects to room, publishes mic track
  → Subscribes to agent audio tracks
  → Laptop mic is now live, speakers ready

Voice Agent:
  → LiveKit dispatches agent to room "table-1"
  → Agent greets customer through laptop speaker:
    "Welcome to Saffron Garden! I'm AMORA, your waiter today.
     Will this be dine-in, takeaway, or delivery?
     And do you have any food allergies I should know about?"
```

### Phase 5: Order Taking

```
Customer speaks into laptop mic:
  → Audio → LiveKit → Gemini 2.5 Flash processes speech
  → Agent responds conversationally, uses function tools:
    - get_menu() to describe available items
    - get_item_details() for specific item information
    - Collects: order_type, items, allergies, spice_level, party_size

Agent confirms order:
  "So that's one Chicken Karahi half, two Seekh Kebabs, and a Mint Margarita,
   medium spice, no allergies. Party of 3, table 1. Shall I confirm?"

Customer confirms → Agent calls finalize_order():
  → Builds order JSON with ID (LK-a1b2c3d4e5)
  → Calculates totals (subtotal + 10% service charge)
  → POSTs to dashboard /api/orders
  → Dashboard stores in PostgreSQL
  → Pusher event → Kitchen staff sees new order ticket with toast notification
```

### Phase 6: Order Processing

```
Kitchen staff dashboard:
  → New order ticket appears in PENDING tab
  → Shows: items, quantities, modifications, allergies, spice level, total
  → Staff prepares order
  → Clicks "Mark Complete" → PATCH /api/orders/[id]
  → Pusher event updates ticket to COMPLETED
```

### Phase 7: Return Home

```
Staff signals order complete:
  → POST /order_complete to orchestrator (or via dashboard trigger)

Orchestrator:
  → State: AT_TABLE → NAVIGATING_HOME
  → Loads home pose: (x=15.43, y=1.91, yaw=-1.57)
  → Calls navigator.goToPose(home)

Local Participant:
  → Detects state change from AT_TABLE
  → Disconnects from LiveKit room
  → Returns to polling mode

Robot navigates home:
  → Same Nav2 pipeline (D* Lite → DWB → velocity smoother → motors)
  → On arrival: State → IDLE
  → Ready for next dispatch
```

---

## 17. TF Frame Tree

### Complete Transform Chain

```
map
 └── (RTAB-Map publishes)
     odom
      └── (EKF publishes)
          base_footprint
           └── (URDF static)
               base_link
                ├── camera_link
                │    ├── camera_color_optical_frame (Z fwd, X right, Y down)
                │    └── camera_depth_optical_frame (Z fwd, X right, Y down)
                ├── imu_link
                ├── scan_frame
                ├── Front_Left_Wheel
                ├── Front_Right_Wheel
                ├── Lower_Left_Wheel
                └── Lower_Right_Wheel
```

### Transform Publishers

| Transform | Publisher | Type | Rate |
|-----------|----------|------|------|
| map -> odom | RTAB-Map | Dynamic | 1 Hz |
| odom -> base_footprint | EKF | Dynamic | 50 Hz |
| base_footprint -> base_link | robot_state_publisher | Static | - |
| base_link -> camera_link | robot_state_publisher | Static | - |
| camera_link -> camera_*_optical_frame | robot_state_publisher | Static | - |
| base_link -> imu_link | robot_state_publisher | Static | - |
| base_link -> scan_frame | robot_state_publisher | Static | - |
| base_link -> *_Wheel | robot_state_publisher | Static | - |

---

## 18. ROS 2 Topic Summary

### Sensor Topics

| Topic | Type | Publisher | Rate | Description |
|-------|------|-----------|------|-------------|
| `/wheel/odometry` | nav_msgs/Odometry | ESP32 | 10 Hz | Wheel encoder odometry |
| `/imu/data` | sensor_msgs/Imu | ESP32 | 20 Hz | IMU gyro + accel (disabled) |
| `/camera/color/image_raw` | sensor_msgs/Image | Astra Pro | 30 Hz | RGB image |
| `/camera/depth/image_raw` | sensor_msgs/Image | Astra Pro | 30 Hz | Depth image |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | Astra Pro | 30 Hz | 3D point cloud |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | Astra Pro | 30 Hz | Camera intrinsics |

### Processed Topics

| Topic | Type | Publisher | Rate | Description |
|-------|------|-----------|------|-------------|
| `/odometry/filtered` | nav_msgs/Odometry | EKF | 50 Hz | Fused odometry |
| `/vo/odometry` | nav_msgs/Odometry | rgbd_odometry | Variable | Visual odometry |
| `/scan` | sensor_msgs/LaserScan | depthimage_to_laserscan | 30 Hz | Fake laser scan (AMCL only) |

### Navigation Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/map` | nav_msgs/OccupancyGrid | RTAB-Map | 2D occupancy grid |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 | Velocity commands to motors |
| `/goal_pose` | geometry_msgs/PoseStamped | RViz/Orchestrator | Navigation goal |
| `/dstar_path` | nav_msgs/Path | D* Lite | Global path visualization |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 | Planning costmap |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | Nav2 | Obstacle avoidance costmap |

### Application Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/waiter/state` | std_msgs/String | Orchestrator | JSON: {state, current_table} |
| `/waiter/tray_cmd` | std_msgs/Int32 | Orchestrator | Tray open command |

---

## 19. Launch Commands Reference

### Mapping

```bash
# Robot-based mapping with RTAB-Map (teleop in second terminal)
LOCALIZATION_MODE=rtabmap RTABMAP_MAPPING=true ros2 launch fyp_bringup bringup_nav.launch.py mode:=mapping

# Teleop control (second terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Handheld mapping (no robot needed, just camera)
ros2 launch fyp_bringup rtabmap_handheld_mapping.launch.py

# Save map after Ctrl+C (wait for "Saving memory... done!")
cp ~/.ros/rtabmap.db ~/fyp_ws/src/fyp_bringup/maps/rtabmap_restaurant.db
```

### Navigation

```bash
# Load specific map
cp ~/fyp_ws/src/fyp_bringup/maps/rtabmap_restaurant.db ~/.ros/rtabmap.db

# Start navigation with RTAB-Map localization
LOCALIZATION_MODE=rtabmap ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

# Start navigation with AMCL localization (requires /scan)
ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav
```

### Full System (4 Terminals)

```bash
# Terminal 1: Navigation
LOCALIZATION_MODE=rtabmap ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

# Terminal 2: Orchestrator
LOCALIZATION_MODE=rtabmap ros2 launch fyp_bringup waiter.launch.py

# Terminal 3: Voice Agent
cd ~/fyp_ws/src/AI\ Voice\(FYP\)/fyp && python agent.py dev

# Terminal 4: Local Participant (mic/speaker)
cd ~/fyp_ws/src/AI\ Voice\(FYP\)/fyp && python local_participant.py
```

### Utility Commands

```bash
# Check database integrity
ros2 run rtabmap_slam rtabmap --ros-args -p database_path:=/path/to/map.db -p frame_id:=base_footprint -p subscribe_depth:=true -p subscribe_rgb:=true

# Record table positions (echo in one terminal, click in RViz)
ros2 topic echo /goal_pose

# Check TF tree
ros2 run tf2_ros tf2_echo map base_footprint

# Check odometry
ros2 topic echo /odometry/filtered --once

# Check robot velocity
ros2 topic echo /cmd_vel
```
