# Multi-Drone Payload Lifting

Two autonomous quadcopters that coordinate to lift and transport a payload 
using UWB relative localisation and ROS 2 offboard control.

**Stack:** ROS 2 Jazzy · PX4 v1.18  · Gazebo · Ubuntu 24.04  
**Status:** 🟡 In development — simulation working, hardware pending

---

## Repo Structure

multi_drone_payload_lifting/
├── ros2_ws/        # ROS 2 nodes (offboard control, UWB driver, facing algorithm)
├── simulation/     # Gazebo worlds, PX4 configs
├── hardware/       # Wiring diagrams, CAD files, BOM
└── docs/           # Calculations, meeting notes, report drafts

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
```

---

## Running the Simulation

Open 4 terminals in this exact order:

**Terminal 1 — Gazebo world**
```bash
cd ~/src/PX4-gazebo-models
python3 simulation-gazebo --world=default
```

**Terminal 2 — PX4 SITL (first drone)**
```bash
cd ~/src/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=default make px4_sitl gz_x500
```

**Terminal 3 — PX4 SITL (second drone)**
```bash
cd ~/src/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=default \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="2,0,0,0,0,0" \
PX4_INSTANCE=1 \
./build/px4_sitl_default/bin/px4 -i 1
```

**Terminal 4 — uXRCE-DDS bridge**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 5 — ROS 2 workspace**
```bash
source ~/.bashrc
source /opt/ros/jazzy/setup.bash
source ~/Projects/multi_drone_payload_lifting/ros2_ws/install/setup.bash
```

---

## Running the Nodes

```bash
cd ~/Projects/multi_drone_payload_lifting/ros2_ws/src

# Listen to drone position
python3 drone_listener.py

# Full offboard control (arm → hover → waypoint → hold → land → disarm)
# Terminal A - drone 1 (no namespace)
python3 offboard_control.py

# Terminal B - drone 2 
python3 offboard_control.py --ros-args -r __ns:=/px4_1
```

> ⚠️ Make sure all 4 simulation terminals are running before launching any node.  
> ⚠️ Always use `--spin-time 5` with `ros2 topic list` or topics won't appear.

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
- [x] Drone-to-drone facing algorithm
- [ ] UWB driver
- [ ] Hardware assembly
- [ ] Port to real drones