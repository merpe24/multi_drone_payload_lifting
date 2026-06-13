# Multi-Drone Cooperative Payload Lifting

Two Holybro X500 quadcopters that cooperatively lift a shared payload using UWB relative localization, a custom Gazebo cable plugin, and ROS 2 offboard control.

---

## Hardware

| Item | Choice |
|---|---|
| Frame | Holybro X500 v2 ×2 |
| Flight Controller | Holybro Pixhawk 6C ×2 |
| Companion Computer | Raspberry Pi 4 (4GB) ×2 |
| Relative Localization | Nooploop LinkTrack P UWB ×4 |
| Battery | 4S 5000mAh 30C ×8 |
| Payload Release | SG90/MG90S servo ×2 + Dyneema 50kg cord + steel hooks |

---

## Software Stack

```
Gazebo (standalone)
    ↕ PX4 SITL / Pixhawk 6C
    ↕ uXRCE-DDS bridge
    ↕ ROS 2 Jazzy (Ubuntu 24.04)
    ↕ Python offboard control nodes
```

**Key dependencies:**
- ROS 2 Jazzy with `rmw_cyclonedds_cpp` (FastDDS breaks multi-drone namespacing)
- PX4-Autopilot built from source at `~/src/PX4-Autopilot`
- empy pinned to 3.3.4
- Custom Gazebo cable physics plugin (`simulation/src/cable.cpp`)

---

## Repository Structure

```
multi_drone_payload_lifting/
├── ros2_ws/
│   └── src/
│       ├── offboard_control.py       # Main state machine for each drone
│       └── launch_formation.sh       # Launches both drone controllers
├── simulation/
│   ├── src/
│   │   └── cable.cpp                 # Gazebo cable spring-damper plugin
│   ├── build/
│   │   └── libcable_plugin.so        # Compiled plugin (copy to Gazebo path)
│   └── worlds/
│       └── cable_world.sdf           # World file — copy to ~/.simulation-gazebo/worlds/
└── drone_handoff.md                  # Full session notes and technical reference
```

---

## Simulation Setup

### 1. Build the cable plugin
```bash
cd simulation/build
cmake .. && make
```

### 2. Copy world file
```bash
cp simulation/worlds/cable_world.sdf ~/.simulation-gazebo/worlds/
```

### 3. Launch (7 terminals in order)

```bash
# T1 — DDS bridge
MicroXRCEAgent udp4 -p 8888

# T2 — Ground station (optional)
QGroundControl

# T3 — Gazebo
python3 ~/src/PX4-gazebo-models/simulation-gazebo --world=cable_world

# T4 — Drone 0
cd ~/src/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_POSE="0,0,0,0,0,1.57" PX4_GZ_WORLD=cable_world make px4_sitl gz_x500

# T5 — Drone 1
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=cable_world PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="2,0,0,0,0,1.57" PX4_INSTANCE=1 \
./build/px4_sitl_default/bin/px4 -i 1

# T6+7 — Wait for "Ready for takeoff!" in both PX4 terminals first
cd ~/Projects/multi_drone_payload_lifting/ros2_ws/src
./launch_formation.sh
```

> **Important:** Always wait for `Ready for takeoff!` in both T4 and T5 before running T6+7.

---

## Coordinate Frames

| Frame | X | Y | Z |
|---|---|---|---|
| Gazebo world | sideways | forward | up |
| PX4 / Python NED | forward | sideways | down (negative = up) |

- Drone 0 spawns at Gazebo `(0, 0, 0)` = NED origin
- Drone 1 spawns at Gazebo `(2, 0, 0)` = NED `(0, 2, 0)`
- Payload spawns at Gazebo `(1, 0, 0.1)` — midpoint between drones

---

## State Machine

```
PREFLIGHT → ARMING → OFFBOARD → HOVER → WAYPOINT → HOLD → LAND → DISARMED
```

| Phase | Ticks | Action |
|---|---|---|
| Pre-stream | 0–39 | Publish setpoints at current position to satisfy PX4 offboard requirement |
| Arm | 40 | Switch to offboard mode + arm |
| Climb | — | Both drones climb to `hover_z = -3.5` (NED), cables engage at ~3m |
| Waypoint | — | Lerp to target, wait for both drones to arrive |
| Hold | — | Hold 5s then land |

---

## Payload Lift Design

Cables attach to tip links at each end of a log-shaped payload rather than both attaching to the center. This eliminates the asymmetric force feedback loop that caused payload drift in the V-shape configuration.

```
Drone 0          Drone 1
   |                |
   | cable          | cable
   |                |
[tip_0]----base----[tip_1]
  (-1,0)   (0,0)   (1,0)   ← relative to payload centre
```

Each cable runs vertically to its own drone, so any altitude difference between drones causes a much smaller and self-correcting horizontal force rather than a runaway imbalance.

**Cable plugin SDF params:** `rest_length=3.1m`, `stiffness=50 N/m`, `damping=20 N·s/m`

---

## Current Status

| Milestone | Status |
|---|---|
| Offboard control state machine | ✅ Complete |
| Multi-drone simulation | ✅ Complete |
| Drone-to-drone facing | ✅ Complete |
| Formation flying | ✅ Complete |
| Cable plugin (physics + visual + gz-transport) | ✅ Complete |
| Cooperative payload lift | 🟡 Working — minor tuning in progress |
| UWB driver integration | ⬜ Not started |
| Hardware assembly | ⬜ Not started |
| Port to real drones | ⬜ Not started |

### Known Tuning Items
- **Initial climb wobble (~0.8m):** Tip link collision boxes interact with drone landing gear at spawn. Fix: reduce box thickness to `0.05 × 0.05 × 0.001`.
- **Waypoint swing:** Payload pendulums at waypoint arrival. Fix: increase cable `damping` to 40, or reduce waypoint lerp speed in `offboard_control.py`.

---

## Team

| Person | Role |
|---|---|
| A | Flight software — offboard control, PID tuning |
| B | Tracking — UWB, facing algorithm |
| C | Hardware — frame, thrust calc, payload bracket |
| D | Systems — RPi, ROS 2 on hardware, MAVLink |