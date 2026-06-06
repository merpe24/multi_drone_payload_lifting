# Multi-Drone Payload Lifting
Two autonomous quadcopters that coordinate to fly in formation and lift a payload
using UWB relative localisation and ROS 2 offboard control.

**Stack:** ROS 2 Jazzy · PX4 v1.18 · Gazebo · Ubuntu 24.04  
**Status:** 🟡 In development — simulation working, hardware pending

---

## Repo Structure
```
multi_drone_payload_lifting/
├── ros2_ws/        # ROS 2 nodes (offboard control, UWB driver, facing algorithm)
├── simulation/     # Gazebo worlds, cable plugin, CMakeLists
├── hardware/       # Wiring diagrams, CAD files, BOM
└── docs/           # Calculations, meeting notes, report drafts
```

---

## Prerequisites
Install the following before cloning this repo:
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- [PX4 Autopilot](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- [Micro XRCE-DDS Agent](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html)
- [px4_msgs](https://github.com/PX4/px4_msgs) — build with colcon in your workspace

> ⚠️ Use `rmw_cyclonedds_cpp`, not FastDDS. FastDDS does not work with this setup.  
> ⚠️ Pin `empy` to 3.3.4 — newer versions break ROS 2 builds.

---

## Environment Setup

| Tool | Version |
|------|---------|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy |
| PX4 | v1.18 (built from source) |
| Gazebo | Standalone (gz-garden) |
| RMW | CycloneDDS (`rmw_cyclonedds_cpp`) |

**Required env vars in `~/.bashrc`:**
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/Projects/multi_drone_payload_lifting/simulation/worlds
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$HOME/src/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$HOME/Projects/multi_drone_payload_lifting/simulation/build
```

---

## Building the Cable Plugin

```bash
cd ~/Projects/multi_drone_payload_lifting/simulation
mkdir -p build && cd build
cmake .. && make
```
Output: `simulation/build/libcable_plugin.so`

After editing `cable_world.sdf`, always deploy it:
```bash
cp ~/Projects/multi_drone_payload_lifting/simulation/worlds/cable_world.sdf ~/.simulation-gazebo/worlds/
```

---

## Running the Simulation

Open 7 terminals in this exact order:

**Terminal 1 — uXRCE-DDS bridge**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — QGroundControl** (optional, for monitoring)
```bash
./QGroundControl.AppImage
```

**Terminal 3 — Gazebo world**
```bash
python3 ~/src/PX4-gazebo-models/simulation-gazebo --world cable_world
```

**Terminal 4 — PX4 SITL (drone 0)**
```bash
cd ~/src/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=cable_world make px4_sitl gz_x500
```

**Terminal 5 — PX4 SITL (drone 1)**
```bash
cd ~/src/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=cable_world \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="2,0,0,0,0,0" \
PX4_INSTANCE=1 \
./build/px4_sitl_default/bin/px4 -i 1
```

**Terminals 6 & 7 — ROS 2 nodes**
```bash
cd ~/Projects/multi_drone_payload_lifting/ros2_ws/src
./launch_formation.sh
```

> ⚠️ Wait for `INFO [commander] Ready for takeoff!` in **both** PX4 terminals before launching nodes.  
> ⚠️ Kill all Python nodes with Ctrl+C before relaunching — stale nodes corrupt the setpoint stream.  
> ⚠️ Always use `--spin-time 5` with `ros2 topic list` or topics won't appear.

---

## Running the Nodes

```bash
cd ~/Projects/multi_drone_payload_lifting/ros2_ws/src

# Listen to drone position (optional debug)
python3 drone_listener.py

# Formation offboard control — launches both drones simultaneously
./launch_formation.sh

# Or launch individually:
python3 offboard_control.py 2>&1 | tee drone1_log.txt                            # drone 0
python3 offboard_control.py --ros-args -r __ns:=/px4_1 2>&1 | tee drone2_log.txt # drone 1
```

**To filter logs after flight:**
```bash
cat drone1_log.txt | grep "keyword"
```

---

## What the Simulation Does

1. Both drones spawn 2m apart along world X axis, wait for EKF to settle
2. Both arm and climb to 2m altitude
3. Both fly forward to local `(3, 0, -2)` NED via lerp-based trajectory
   - Drone 0: world `(0,0,0)` → world `(3, 0, -2)`
   - Drone 1: world `(2,0,0)` → world `(5, 0, -2)`
   - World separation at destination: **2.0m along X** — cable stays slack
4. Hold formation for 5 seconds, then land
5. A yellow cylinder renders in Gazebo showing the cable between drones

**Formation geometry note:** Waypoints are in each drone's local NED frame. Both drones use the same local waypoint `(3, 0, -2)` but end up 2m apart in world frame due to their spawn offset. Cable rest length is 2.05m — just enough slack at the destination.

---

## Cable Plugin

Spring-damper cable connecting the two drones. Force only applied when cable is taut (stretched beyond rest length).

**Parameters (in `cable_world.sdf`):**
```xml
<rest_length>2.05</rest_length>   <!-- cable length at which tension starts (m) -->
<stiffness>150.0</stiffness>      <!-- spring constant k (N/m) -->
<damping>20.0</damping>           <!-- damping d (N·s/m) -->
```

**Visual:** Yellow 3D cylinder, 2cm diameter, updates every physics tick via Gazebo marker API.

---

## Team

| Person | Role |
|--------|------|
| A | Flight software — offboard control, PID tuning |
| B | Tracking — UWB localisation, drone-facing algorithm |
| C | Hardware — frame assembly, thrust calculations, payload bracket |
| D | Systems — RPi setup, ROS 2 on hardware, MAVLink |

---

## Current Progress

- [x] Simulation environment working
- [x] Single drone offboard control (arm → hover → waypoint → hold → land → disarm)
- [x] Multi-drone simulation
- [x] Drone-to-drone facing algorithm (P controller, angle unwrapping, deadband)
- [x] Formation flying (lerp-based trajectory, stable hold)
- [x] Cable plugin — physics (spring-damper, slack detection, no tilt)
- [x] Cable plugin — visual (yellow cylinder via Gazebo marker API)
- [ ] Payload attachment in Gazebo (hook + Dyneema model)
- [ ] UWB driver
- [ ] Hardware assembly
- [ ] Port to real drones