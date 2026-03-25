# Autonomous Waiter Robot with AI Voice Ordering

A ROS2-based autonomous waiter robot that navigates to restaurant tables, takes orders through an AI voice agent (Google Gemini), and returns home — all orchestrated through a real-time web dashboard.

Built as a Final Year Project (FYP).

## System Overview

```
                          +-------------------+
                          |   Web Dashboard   |  (Next.js + Pusher)
                          |  Dispatch / Status |
                          +--------+----------+
                                   |
                            Pusher WebSocket
                                   |
                          +--------v----------+
    +----------+          |   Orchestrator    |          +--------------+
    |  Nav2    | <------> |  State Machine    | -------> |  LiveKit     |
    | (AMCL + |  ROS2    |  HTTP :5050       |  Room    |  Voice Agent |
    | DWB +   |          +-------------------+  Create  |  (Gemini)    |
    | D*Lite) |                                         +--------------+
    +----+-----+                                               |
         |                                                LiveKit Room
    +----v-----+                                               |
    |  Robot   |                                     +---------v--------+
    | ESP32 +  |                                     | Local Participant|
    | Sensors  |                                     | (Mic / Speaker)  |
    +----------+                                     +------------------+
```

**Flow:**
1. Staff dispatches the robot to a table via the dashboard
2. Robot autonomously navigates to the table using Nav2 + D* Lite
3. On arrival, a LiveKit room is created and the AI voice agent joins
4. The local participant bridges the laptop's mic/speaker into the room
5. Customer places their order by talking to the AI agent
6. Agent finalizes the order and signals the orchestrator
7. Robot navigates back home, ready for the next dispatch

## Repository Structure

```
fyp_ws/
├── src/
│   ├── fyp_description/        # Robot URDF model + meshes (SolidWorks export)
│   ├── fyp_bringup/            # Launch files, configs, firmware, maps
│   │   ├── launch/             # All launch files (real + sim)
│   │   ├── config/             # EKF, controllers, table positions, env vars
│   │   ├── scripts/            # Orchestrator, obstacle gate, utilities
│   │   ├── firmware/           # ESP32 micro-ROS firmware
│   │   ├── maps/               # Saved occupancy grid maps
│   │   └── worlds/             # Gazebo simulation worlds
│   ├── fyp_nav2/               # Nav2 parameters + maps
│   │   └── params/             # Nav2 config variants (sim, real, RPP)
│   ├── fyp_dstar_lite/         # Custom D* Lite global planner (Nav2 plugin)
│   └── AI Voice(FYP)/fyp/     # AI voice agent + web dashboard
│       ├── agent.py            # LiveKit voice agent (Gemini 2.5 Flash)
│       ├── local_participant.py# Auto-bridges laptop mic/speaker to room
│       ├── dashboard/          # Next.js web dashboard
│       └── .env.local          # LiveKit + Google API credentials
└── README.md
```

## Hardware

| Component | Details |
|---|---|
| Chassis | 4-wheel differential drive (~0.45m x 0.32m) |
| Microcontroller | ESP32 running micro-ROS |
| Camera | Orbbec Astra Pro (RGB-D, depth converted to LaserScan) |
| IMU | MPU6050 via I2C |
| Encoders | 330 ticks/rev on front wheels |
| Motor Control | Differential drive (wheel_separation: 0.40m, wheel_radius: 0.0675m) |

## Software Stack

| Layer | Technology |
|---|---|
| Robot Framework | ROS2 Humble |
| Navigation | Nav2 (AMCL + DWB Controller + Velocity Smoother) |
| Global Planner | Custom D* Lite (with NavfnPlanner/A* fallback) |
| Localization | AMCL (navigation) / SLAM Toolbox (mapping) |
| Sensor Fusion | robot_localization EKF (wheel odometry + IMU) |
| Simulation | Gazebo (Fortress) |
| Orchestrator | Python state machine + HTTP server (port 5050) |
| Voice Agent | LiveKit Agents SDK + Google Gemini 2.5 Flash (native audio) |
| Dashboard | Next.js 16, React 19, Prisma (SQLite), Pusher, TailwindCSS |
| Firmware | Arduino / micro-ROS on ESP32 |

## Prerequisites

- **ROS2 Humble** (Ubuntu 22.04)
- **Nav2**, **SLAM Toolbox**, **robot_localization** ROS2 packages
- **Gazebo Fortress** (for simulation)
- **Python 3.10+** with conda
- **Node.js 18+** (for the dashboard)
- **ESP32** flashed with micro-ROS firmware (for real robot)

## Setup

### 1. Build the ROS2 workspace

```bash
cd ~/fyp_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Install voice agent dependencies

```bash
conda activate fyp
cd "src/AI Voice(FYP)/fyp"
pip install -r requirements.txt
```

### 3. Install dashboard dependencies

```bash
cd "src/AI Voice(FYP)/fyp/dashboard"
npm install
npx prisma generate
npx prisma migrate dev
npx prisma db seed
```

### 4. Configure environment variables

Voice agent credentials in `src/AI Voice(FYP)/fyp/.env.local`:
```
LIVEKIT_URL=wss://your-livekit-instance.livekit.cloud
LIVEKIT_API_KEY=your_api_key
LIVEKIT_API_SECRET=your_api_secret
GOOGLE_API_KEY=your_google_api_key
NEXT_PUBLIC_LIVEKIT_URL=wss://your-livekit-instance.livekit.cloud
DASHBOARD_API_URL=https://your-dashboard.vercel.app
DASHBOARD_AGENT_KEY=your_agent_key
```

Orchestrator credentials in `src/fyp_bringup/config/orchestrator.env`:
```
DASHBOARD_URL=https://your-dashboard.vercel.app
AGENT_API_KEY=your_agent_key
PUSHER_KEY=your_pusher_key
PUSHER_CLUSTER=ap2
LIVEKIT_URL=wss://your-livekit-instance.livekit.cloud
LIVEKIT_API_KEY=your_api_key
LIVEKIT_API_SECRET=your_api_secret
```

## Running

### Real Robot (Full Waiter Service)

Open 5 terminals:

```bash
# Terminal 1 — Robot hardware + Nav2
source ~/fyp_ws/install/setup.bash
ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav

# Terminal 2 — Waiter orchestrator
source ~/fyp_ws/install/setup.bash
ros2 launch fyp_bringup waiter.launch.py

# Terminal 3 — AI voice agent
conda activate fyp
cd ~/fyp_ws/src/AI\ Voice\(FYP\)/fyp
python agent.py dev

# Terminal 4 — Web dashboard
cd ~/fyp_ws/src/AI\ Voice\(FYP\)/fyp/dashboard
npm run dev

# Terminal 5 — Local mic/speaker bridge
conda activate fyp
cd ~/fyp_ws/src/AI\ Voice\(FYP\)/fyp
python local_participant.py
```

### Mapping (SLAM)

```bash
# Terminal 1 — Launch with SLAM mode
ros2 launch fyp_bringup bringup_nav.launch.py mode:=slam

# Terminal 2 — Drive the robot around
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save the map when done
ros2 run nav2_map_server map_saver_cli -f ~/fyp_ws/src/fyp_bringup/maps/my_map
```

### Simulation

```bash
# Full simulation with Nav2 and D* Lite
ros2 launch fyp_bringup full_nav2.launch.py

# Or choose the A* planner instead
ros2 launch fyp_bringup sim_nav.launch.py planner:=navfn
```

## Key Configuration

### Table Positions (`config/table_positions.yaml`)

Define table coordinates in the map frame. Measure positions using RViz's "Publish Point" tool:

```yaml
home:
  x: 0.0
  y: 0.0
  yaw: 0.0
tables:
  table-1:
    x: 2.5
    y: 1.0
    yaw: 1.57
  table-2:
    x: 2.5
    y: -1.0
    yaw: -1.57
```

### Nav2 Parameters

Multiple parameter files are available in `src/fyp_nav2/params/`:

| File | Use Case |
|---|---|
| `nav2_params.yaml` | Default (DWB + NavfnPlanner) |
| `nav2_params_real.yaml` | Real robot (use_sim_time: false) |
| `nav2_params_sim.yaml` | Simulation |
| `nav2_params_rpp.yaml` | Regulated Pure Pursuit + SMAC Planner |

## API Reference

### Orchestrator HTTP API (port 5050)

| Endpoint | Method | Body | Description |
|---|---|---|---|
| `/dispatch` | POST | `{"table_id": "table-1"}` | Send robot to a table |
| `/order_complete` | POST | `{"room_name": "table-1", "order_id": "..."}` | Signal order done, robot returns home |
| `/status` | GET | — | Get current state, table, and room info |

### Quick test with curl

```bash
# Dispatch to table-1
curl -X POST localhost:5050/dispatch \
  -H 'Content-Type: application/json' \
  -d '{"table_id": "table-1"}'

# Check status
curl localhost:5050/status

# Signal order complete
curl -X POST localhost:5050/order_complete \
  -H 'Content-Type: application/json' \
  -d '{"room_name": "table-1", "order_id": "order-123"}'
```

## Architecture Details

### State Machine

```
IDLE ──dispatch──> NAVIGATING_TO_TABLE ──arrived──> AT_TABLE ──order_complete──> NAVIGATING_HOME ──arrived──> IDLE
                         │                                                            │
                         └──nav_failed──> IDLE                                        └──> IDLE
```

### D* Lite Global Planner

A custom Nav2 global planner implementing the D* Lite algorithm as a ROS2 action server. It subscribes to `/global_costmap/costmap`, computes paths using incremental replanning, and publishes visualization on `/dstar_path`.

Toggle at launch:
```bash
ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav planner:=dstar   # D* Lite (default)
ros2 launch fyp_bringup bringup_nav.launch.py mode:=nav planner:=navfn   # A* NavfnPlanner
```

### Voice Agent

The AI voice agent uses Google Gemini 2.5 Flash with native audio I/O through LiveKit. It acts as a restaurant waiter for "Saffron Garden" — a Pakistani/BBQ/Continental restaurant. It can:

- Present the menu and make recommendations
- Handle dietary restrictions and allergies
- Adjust spice levels and customizations
- Finalize orders and notify the orchestrator

### Real-Time Dashboard

The web dashboard provides:
- Table dispatch buttons with live robot status badges
- Order management (CRUD via Prisma + SQLite)
- Real-time updates via Pusher WebSocket
- JWT-based authentication
