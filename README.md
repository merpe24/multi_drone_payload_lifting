# Multi-Drone Aerial Crane for Teleoperated Payload Manipulation

A cooperative two-quadcopter system that behaves as a **teleoperated flying crane**. A human operator commands a target position for a cable-suspended payload; two Holybro X500 quadcopters fly in formation and continuously regulate their positions so the payload tracks the commanded point in real time. The architecture cleanly decouples the *operator input source* from the *flight controllers*, so the same control stack accepts an automated trajectory, a hand-held radio, or (planned) a lab 3-DOF robotic arm without modification.

The project is developed and validated in a PX4 SITL + Gazebo simulation, including a custom cable physics plugin, ahead of a port to physical hardware.

---

## System Overview

```
Operator input (interchangeable)
        │   geometry_msgs/PointStamped  →  /payload_position_ref  (NED)
        ▼
crane_control.py  ×2   (one offboard controller per drone)
        │   per-drone TrajectorySetpoint
        ▼
PX4 (SITL or Pixhawk 6C)  ×2
        │   rotor thrust → cable tension
        ▼
Suspended payload  →  tracks the commanded position
```

**Design principle.** `crane_control.py` subscribes only to `/payload_position_ref`. It has no knowledge of how that reference is produced or that two drones exist on the operator side. From a single payload reference `(px, py, pz)` it derives a per-drone formation target:

- Drone 0 → `(px, py − FORMATION_Y_OFFSET, pz − CABLE_LENGTH)`
- Drone 1 → `(px, py + FORMATION_Y_OFFSET, pz − CABLE_LENGTH)`

with `FORMATION_Y_OFFSET = 1.0 m` and `CABLE_LENGTH = 3.1 m`. The drones hold a fixed 2 m lateral separation and fly a constant offset above the payload reference, tracking it with a velocity-limited setpoint (0.5 m/s).

---

## Hardware

| Item | Choice |
|---|---|
| Frame | Holybro X500 v2 ×2 |
| Flight Controller | Holybro Pixhawk 6C ×2 |
| Companion Computer | Raspberry Pi 4 (4GB) ×2 |
| Relative Localization | Nooploop LinkTrack P UWB ×4 |
| Battery | 4S 5000 mAh 30C ×8 |
| Payload Release | SG90/MG90S servo ×2 + Dyneema 50 kg cord + steel hooks |
| Operator Radio | RadioMaster Pocket (ELRS) + RP2 receiver |
| Indoor Localization | Optical flow (velocity) + AprilTags (absolute) — under evaluation |

---

## Software Stack

```
Gazebo (standalone)
    ↕ PX4 SITL / Pixhawk 6C
    ↕ uXRCE-DDS bridge (MicroXRCEAgent)
    ↕ ROS 2 Jazzy (Ubuntu 24.04)
    ↕ Python control & teleoperation nodes
```

**Key dependencies**
- ROS 2 Jazzy with `rmw_cyclonedds_cpp` (the FastDDS RMW breaks multi-drone namespacing)
- PX4-Autopilot built from source at `~/src/PX4-Autopilot`
- `empy` pinned to 3.3.4
- `ros-jazzy-joy` (radio teleoperation)
- Custom Gazebo cable physics plugin (`simulation/src/cable.cpp`)

---

## Repository Structure

```
multi_drone_payload_lifting/
├── ros2_ws/
│   └── src/
│       ├── crane_control.py            # Per-drone offboard controller (teleoperated crane)
│       ├── test_ref_publisher.py       # Automated sine-sweep payload reference (testing)
│       ├── joystick_ref_publisher.py   # RadioMaster radio → /payload_position_ref
│       ├── offboard_control.py         # Single-drone baseline (waypoint + land)
│       ├── drone_listener.py           # Position-monitoring/debug node
│       └── launch_formation.sh         # Convenience launcher for the baseline controllers
├── simulation/
│   ├── src/cable.cpp                   # Gazebo cable spring-damper plugin
│   ├── build/libcable_plugin.so        # Compiled plugin
│   └── worlds/cable_world.sdf          # World file — copy to ~/.simulation-gazebo/worlds/
└── CLAUDE.md                           # Condensed technical handoff / reference
```

---

## Simulation Setup

### 1. Build the cable plugin
```bash
cd simulation/build
cmake .. && make
```

### 2. Install the world file
```bash
cp simulation/worlds/cable_world.sdf ~/.simulation-gazebo/worlds/
```

### 3. Launch

Default DDS discovery is used (no `CYCLONEDDS_URI` configuration required).

```bash
# T1 — uXRCE-DDS bridge
MicroXRCEAgent udp4 -p 8888

# T2 — Gazebo
python3 ~/src/PX4-gazebo-models/simulation-gazebo --world=cable_world

# T3 — Drone 0
cd ~/src/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_POSE="0,0,0,0,0,1.57" PX4_GZ_WORLD=cable_world make px4_sitl gz_x500

# T4 — Drone 1
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=cable_world PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="2,0,0,0,0,1.57" PX4_INSTANCE=1 \
./build/px4_sitl_default/bin/px4 -i 1
# ↑ wait for "Ready for takeoff!" in BOTH T3 and T4

# T5 / T6 — Crane controllers (one per drone)
cd ~/Projects/multi_drone_payload_lifting/ros2_ws/src
python3 crane_control.py
python3 crane_control.py --ros-args -r __ns:=/px4_1
# ↑ wait for "Hover reached" in BOTH

# T7 / T8 — Payload reference source (choose one)
python3 test_ref_publisher.py                 # automated sine sweep
# — or — radio teleoperation:
ros2 run joy joy_node
python3 joystick_ref_publisher.py
```

> Both drones must be running: the climb phase synchronizes on the companion drone's altitude, so a single drone will not take off on its own.

To fully stop the simulation (the Gazebo server process outlives its launcher):
```bash
pkill -9 -f "gz sim server"; pkill -9 -f MicroXRCEAgent; pkill -9 -f "bin/px4"; pkill -9 -f simulation-gazebo
```

---

## Teleoperation

`joystick_ref_publisher.py` reads a RadioMaster Pocket in EdgeTX USB joystick mode (a standard HID gamepad on `/joy`) and integrates stick deflection into a payload position command at 20 Hz:

```
pos += axis × MAX_SPEED × dt          (MAX_SPEED = 0.5 m/s, deadzone 0.1)
```

The reference starts at the formation center `(0, 1, 0)` so engaging teleoperation from a stable hover produces no discontinuity. Stick-to-axis mapping (verified on hardware): right stick → payload X/Y, left stick (vertical) → payload Z.

---

## Coordinate Frames

| Frame | X | Y | Z |
|---|---|---|---|
| Gazebo world | sideways | forward | up |
| PX4 / Python NED | forward | sideways | down (negative = up) |

- Drone 0 spawns at Gazebo `(0, 0, 0)` = NED origin
- Drone 1 spawns at Gazebo `(2, 0, 0)` = NED `(0, 2, 0)`
- Payload spawns at Gazebo `(1, 0, 0.1)` — midpoint between the drones

---

## Control State Machine

```
PREFLIGHT → ARMING → OFFBOARD → HOVER → TRACK (/payload_position_ref)
```

| Phase | Action |
|---|---|
| Pre-stream | Publish setpoints at the current position to satisfy PX4's offboard entry requirement |
| Arm | Switch to offboard mode and arm |
| Climb | Both drones climb to `hover_z = −3.5 m` (NED), synchronized on companion altitude; cables engage |
| Track | Velocity-limited tracking of the live `/payload_position_ref` (no landing phase — stop with Ctrl+C) |

The legacy `offboard_control.py` retains the original single-drone `WAYPOINT → HOLD → LAND` flow as a baseline reference.

---

## Payload & Cable Design

Cables attach to **tip links at each end** of a horizontal bar payload rather than both attaching to the center. Each cable runs vertically to its own drone, so an altitude difference between drones produces a small, self-correcting horizontal force instead of the runaway imbalance seen with a centered V-shape attachment.

```
Drone 0            Drone 1
   |                  |
   | cable            | cable
   |                  |
[tip0]----base----[tip1]
 (−1,0)   (0,0)    (1,0)      ← relative to payload centre (metres)
```

| Parameter | Value |
|---|---|
| Payload `base_link` | 1.5 kg, box 1.5 × 0.2 × 0.2 m |
| Tip links | 0.02 kg, fixed joints, sphere collision r = 0.01 m |
| Cable plugin (per instance) | `rest_length = 2.5 m`, `stiffness = 50 N/m`, `damping = 40 N·s/m` |

> The payload X dimension must stay ≤ 2.0 m; at exactly 2.0 m the tip links intersect the drone landing gear at spawn and destabilize the physics. The current bar is 1.5 m (tips at ±1.0 m).

---

## Status & Roadmap

| Milestone | Status |
|---|---|
| Offboard control state machine | ✅ Complete |
| Multi-drone simulation | ✅ Complete |
| Drone-to-drone facing & formation flying | ✅ Complete |
| Cable plugin (physics + visual + gz-transport) | ✅ Complete |
| Cooperative payload lift | ✅ Tuned |
| Decoupled crane controller (`crane_control.py`) | ✅ Complete |
| Teleoperated crane (full sim, RadioMaster) | ✅ Verified end-to-end |
| Indoor localization (optical flow + AprilTags in sim) | 🟡 In progress |
| 3-DOF arm operator interface | ⬜ Pending arm interface |
| UWB driver integration | ⬜ Not started |
| Port to physical drones | ⬜ Not started |
