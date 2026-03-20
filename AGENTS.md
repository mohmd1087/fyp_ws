# FYP Robot - ROS2 Autonomous Navigation Workspace

## Project Overview
4WD skid-steer autonomous robot using ROS 2 (Jazzy/Humble). Focused on **mapping (SLAM), navigation (Nav2), and robot waiter service** with LiveKit voice agent integration. Supports both Gazebo simulation and real hardware (ESP32 + micro-ROS).

## Workspace Structure
```
fyp_ws/
├── src/
│   ├── fyp_description/   # URDF, meshes, robot model
│   ├── fyp_bringup/       # Launch files, configs, firmware, worlds, maps
│   ├── fyp_nav2/          # Nav2 params, nav launch files, maps, rviz configs
│   ├── fyp_dstar_lite/    # D* Lite global planner for Nav2
│   └── Dstarlite-dynamic/ # D* Lite with dynamic obstacle support (Gazebo reference)
├── AI Voice(FYP)/fyp/          # LiveKit voice agent + Next.js dashboard (separate project)
├── build/
├── install/
└── log/
```

## Packages

### fyp_dstar_lite (ament_cmake + ament_cmake_python) — NEW
D* Lite incremental pathfinding algorithm integrated as a Nav2 global planner.

**How it works:**
- Standalone Python ROS2 action server implementing `ComputePathToPose`
- Replaces `nav2_planner/planner_server` (NavfnPlanner/A*) without C++ code
- Subscribes to `/global_costmap/costmap` (TRANSIENT_LOCAL QoS)
- Gets robot pose from TF: `map` → `base_footprint`
- Publishes planned path to `/dstar_path` for RViz visualization

**Key files:**
- `fyp_dstar_lite/dstar_planner_node.py` — main ROS2 action server
- `fyp_dstar_lite/d_star_lite.py` — D* Lite algorithm (adapted from Downloads)
- `fyp_dstar_lite/grid.py` — OccupancyGridMap (SLAM class removed)
- `fyp_dstar_lite/priority_queue.py` — priority queue (verbatim)
- `fyp_dstar_lite/utils.py` — heuristic + movement helpers (verbatim)
- `launch/dstar_planner.launch.py` — launches the planner node

**Source:** `~/Downloads/D-star-lite-in-gazebo_ROS2-main/Dstarlite-static/`

**Adaptations from source:**
- Removed Gazebo-specific SLAM class and `/model_states` subscriber from grid.py
- Fixed imports to `fyp_dstar_lite.*` package-relative paths in d_star_lite.py
- Reduced `danger_radius` 8 → 4 cells (tuned for 0.05 m/cell Nav2 costmap)
- Added `max_steps=100000` guard in `move_and_replan()` (prevents known infinite loop bug)
- Added `arg_min is None` check (graceful robot-trapped detection)

**Planner toggle (no file edits needed):**
```bash
ros2 launch fyp_bringup sim_nav.launch.py              # D* Lite (default)
ros2 launch fyp_bringup sim_nav.launch.py planner:=navfn  # A* (NavfnPlanner)
```

**Algorithm standalone test:**
```python
from fyp_dstar_lite.grid import OccupancyGridMap
from fyp_dstar_lite.d_star_lite import DStarLite
g = OccupancyGridMap(30, 30)
d = DStarLite(g, (5,5), (25,25)); d.sensed_map = g
path, _, _ = d.move_and_replan((5,5))
assert path[-1] == (25,25)
```

---

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
- `navigation.launch.py` — Nav2 stack + obstacle gate node
- `waiter.launch.py` — waiter orchestrator node (robot waiter service)

**Config files:**
- `config/controllers.yaml` — diff_drive_controller (wheel_separation: 0.40, wheel_radius: 0.0675)
- `config/ekf.yaml` — EKF for simulation (fuses /odom + /imu)
- `config/ekf_real.yaml` — EKF for real robot (fuses /wheel/odometry + /imu/data)
- `config/table_positions.yaml` — home pose + table coordinates for waiter orchestrator

**Scripts:**
- `scripts/obstacle_gate_node.py` — obstacle stop-and-wait node (gates `/cmd_vel`)
- `scripts/pose_sync.py` — pose synchronization utility
- `scripts/waiter_orchestrator.py` — robot waiter service orchestrator

**Firmware:**
- `firmware/micro_ros_nav2_esp32.ino` — ESP32 firmware (motor control, encoders, MPU6050 IMU, micro-ROS)

### fyp_nav2 (ament_cmake)
- `params/nav2_params.yaml` — DWB controller + NavfnPlanner (main sim config)
- `params/nav2_params_rpp.yaml` — Regulated Pure Pursuit + SMAC Planner (alternative)
- `params/nav2_params_real.yaml` — real robot Nav2 config (use_sim_time: false)
- `maps/` — my_map and maze map files
- `rviz/nav2.rviz` — pre-configured RViz layout

## Obstacle Gate (Stop-and-Wait)

Safety node that stops the robot when a dynamic or static obstacle is detected in its path. Inserted into the Nav2 velocity pipeline between the velocity smoother and the robot.

**Velocity pipeline:**
```
controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → obstacle_gate → /cmd_vel → robot
```

**How it works:**
- Subscribes to `/scan` (LaserScan from depth camera)
- Subscribes to `/cmd_vel_smoothed` (output of velocity smoother)
- Checks if any laser reading within a ±30° front cone is closer than 0.5m
- If obstacle detected: publishes zero velocity (robot stops and waits)
- If obstacle clears: forwards velocity commands unchanged (robot resumes)

**Key file:** `fyp_bringup/scripts/obstacle_gate_node.py`

**Parameters (configurable at launch):**
- `stop_distance` — distance threshold in meters (default: 0.5)
- `cone_angle` — half-angle of front detection cone in radians (default: 0.524 = ±30°)

**Launched automatically** by `navigation.launch.py` — no extra steps needed.

---

## Waiter Orchestrator (Robot Waiter Service)

Integrates the robot with a LiveKit voice agent for autonomous table service. Staff dispatch the robot from the dashboard; it navigates to a table, the voice agent takes the order, then the robot returns home.

**Architecture:**
```
Dashboard (Next.js)  ──POST /dispatch──>  Orchestrator (ROS2)
                                            │
                                            ├─ Nav2 BasicNavigator (goToPose)
                                            ├─ HTTP server on :5050
                                            └─ LiveKit room creation
                                                    │
Voice Agent (agent.py)  ──auto-joins room──>  LiveKit Cloud
        │
        └── POST /order_complete ──> Orchestrator ──> robot goes home
```

**State machine:** `IDLE` → `NAVIGATING_TO_TABLE` → `AT_TABLE` → `NAVIGATING_HOME` → `IDLE`

**Key files:**
- `scripts/waiter_orchestrator.py` — ROS2 orchestrator (wraps BasicNavigator, not a Node subclass)
- `config/table_positions.yaml` — home + table coordinates (placeholder; measure from RViz)
- `launch/waiter.launch.py` — launches the orchestrator

**HTTP API (port 5050):**
- `POST /dispatch` — `{"table_id": "table-1"}` — send robot to a table
- `POST /order_complete` — `{"room_name": "table-1", "order_id": "..."}` — trigger return home
- `GET /status` — `{"state": "IDLE", "current_table": null, "tables": [...]}`

**Room persistence:** Room names are deterministic per table (e.g. `"table-1"`). Rooms are created with `empty_timeout=3600s` so return trips to the same table reuse the room and preserve conversation context.

**Dependencies:** `pip install livekit-api` (optional — room creation skipped if not installed)

---

## AI Voice Agent (LiveKit)

**Location:** `~/AI Voice(FYP)/fyp/`

LiveKit-based voice agent ("AMORA") using Google Gemini 2.5 Flash with native audio. Takes restaurant orders conversationally.

**Key files:**
- `agent.py` — main voice agent (RestaurantWaiter class with function tools)
- `dashboard/` — Next.js order dashboard with robot dispatch UI
- `.env.local` — LiveKit + Google API + orchestrator callback credentials

**Dashboard additions for robot waiter:**
- `dashboard/src/app/api/robot/dispatch/route.ts` — API route proxying to orchestrator
- `dashboard/src/components/robot-dispatch.tsx` — "Send to Table 1/2" buttons + status badge
- Integrated into `dashboard/src/components/dashboard-shell.tsx` header

**Agent ↔ orchestrator integration:**
- `finalize_order()` in agent.py POSTs to `ORCHESTRATOR_CALLBACK_URL/order_complete` after order confirmation
- Room name retrieved via `context.session.room.name` (RunContext → AgentSession → Room)

---

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

### Robot Waiter Service
```bash
# Terminal 1: Nav2 stack
ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

# Terminal 2: Waiter orchestrator (set LiveKit env vars first)
export LIVEKIT_URL="wss://demo-assistant-5dhlk5bz.livekit.cloud"
export LIVEKIT_API_KEY="<key>"
export LIVEKIT_API_SECRET="<secret>"
ros2 launch fyp_bringup waiter.launch.py

# Terminal 3: Voice agent
cd ~/AI\ Voice\(FYP\)/fyp && python agent.py dev

# Terminal 4 (optional): Dashboard locally
cd ~/AI\ Voice\(FYP\)/fyp/dashboard && npm run dev

# Dispatch via dashboard UI or curl:
curl -X POST localhost:5050/dispatch -H 'Content-Type: application/json' -d '{"table_id":"table-1"}'
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
| `/cmd_vel` | Twist | obstacle_gate (gated output) |
| `/cmd_vel_nav` | Twist | Nav2 controller_server |
| `/cmd_vel_smoothed` | Twist | Nav2 velocity_smoother |
| `/scan` | LaserScan | depthimage_to_laserscan |
| `/odom` (sim) or `/wheel/odometry` (real) | Odometry | diff_drive_controller / ESP32 |
| `/odometry/filtered` | Odometry | robot_localization EKF |
| `/imu` (sim) or `/imu/data` (real) | Imu | Gazebo / ESP32 MPU6050 |
| `/camera/depth/image_raw` | Image | depth camera |
| `/waiter/state` | String (JSON) | waiter_orchestrator |

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
