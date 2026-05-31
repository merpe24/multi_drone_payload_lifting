# Drone Research Project — Handoff Summary
> University robotics lab project. Team of 4, ~3 months. Read this fully before responding.

## How Claude Should Teach

The student learns best through guided discovery, not direct answers.
Always follow this approach:

**When the student hits an error:**
- Ask them to read the error literally first
- Ask what data they can gather before guessing
- Let them form a hypothesis, then test it

**When writing code:**
- Never write the full solution immediately
- Ask them to reason through the logic first
- Have them do a tick-by-tick trace before running anything
- Ask "what happens if..." questions to find bugs before running

**When they get something wrong:**
- Ask a question that leads them to find the mistake themselves
- Only reveal the answer if they're stuck after 2-3 attempts

**General rules:**
- Group 2-3 related questions together per response (student requested this — avoids wasting tokens on one-liners)
- Make them trace through their own code before running it
- Celebrate correct reasoning, not just correct output
- If they get it right, ask why it works — not just that it works
- Bugs are teaching moments, not problems to fix for them

**What has worked so far:**
- Tick-by-tick traces catch logic bugs before running
- "What does this error say literally?" before any debugging
- "You already have something in your code that does this pattern..."
- Asking them to compare two versions of code and spot the difference
- Redirecting output to file with `tee` when terminal scrolls too fast

---

## Project Goal
Build **2 drones from scratch** (assemble + flash firmware) that can:
1. **Track each other** — hover while facing one another using UWB relative localization
2. **Lift payloads** — using a servo-released hook + Dyneema cord mechanism
3. Demonstrate real-world autonomous behavior via ROS 2 offboard control

Learning objective is deep understanding — calculations, dynamics, control theory — not just flying a product.

---

## Hardware Decisions (final, do not re-discuss)

| Component | Choice | Reason |
|---|---|---|
| Frame | **Holybro X500 v2 kit** ×2 | Exact match to `gz_x500` Gazebo model — sim params already correct |
| Flight Controller | **Holybro Pixhawk 6C** ×2 | PX4 first-class support, made by Holybro (fits X500 perfectly), $150 cheaper than 6X |
| GPS | Holybro M10 GPS + compass ×2 | Position hold for payload pickup |
| Telemetry | Holybro SiK Radio v3 ×2 pairs | MAVLink monitoring from laptop |
| RC | RadioMaster Boxer TX + ELRS EP1 RX ×2 | Manual override / emergency stop |
| Battery | **4S 5000mAh 30C LiPo ×8** | ~18min unloaded, 4 per drone for continuous lab sessions |
| Charger | ISDT 608AC dual output | Charges 2 batteries simultaneously |
| Companion computer | **Raspberry Pi 4 (4GB) ×2** | Runs ROS 2 nodes onboard, connects to Pixhawk via UART |
| Power for RPi | UBEC 5V 3A ×2 | Steps down 14.8V LiPo to 5V |
| Tracking | **Nooploop LinkTrack P UWB ×4** | 2 per drone, centimeter-level relative ranging at ~100Hz |
| Release mechanism | **SG90/MG90S micro servo ×2** | Releases hook, controlled via Pixhawk AUX or RPi GPIO |
| Cord | Dyneema 50kg rated + steel hooks | Lightweight, non-stretch, payload hangs below drone |
| Mount | 3D printed PETG bracket ×2 | Custom servo/hook mount to X500 bottom plate |
| Safety | LiPo bags ×4, voltage alarms ×8, tools | Non-negotiable lab safety |

**Total BOM: ~$2,400**
**Order now (critical path):** X500 kits, Pixhawk 6C, GPS, SiK radios, RC, batteries, charger, RPi, UBEC, LiPo bags, tools — ships from China, 2–3 week lead time
**Order week 2–3:** UWB modules, servos, Dyneema, 3D print brackets

---

## Drone Type
**Quadcopter** (X500 v2) for now. Discussed hexacopter for more payload/redundancy but decided X500 sim-to-real advantage outweighs it for the 3-month timeline.

**Payload mechanism:** hook + Dyneema cord (not electromagnet, not gripper). Servo releases the hook. Each drone carries independently — cooperative dual-lift is stretch goal only.

---

## Software Stack (decided)

```
Gazebo (physics sim)
  ↕
PX4 SITL / PX4 on Pixhawk 6C
  ↕
uXRCE-DDS Agent (MicroXRCEAgent)
  ↕
ROS 2 Jazzy (on Ubuntu 24.04)
  ↕
Your ROS 2 nodes (Python)
```

- **OS:** Ubuntu 24.04 (native, not Docker — student solved all compatibility issues)
- **ROS 2:** Jazzy (not Humble)
- **PX4:** Built from source in `~/src/PX4-Autopilot`
- **Simulation:** Gazebo standalone mode (`python3 simulation-gazebo --world=default`)
- **RMW:** `rmw_cyclonedds_cpp` — **required**, FastDDS does not work with their setup
- **ROS_DOMAIN_ID:** 0 for drone work, 42 reserved for separate RPi project

---

## Simulation Stack — Working ✅

### Launch sequence (5 terminals, always in this order):
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
# Drone 1
python3 offboard_control.py 2>&1 | tee drone1_log.txt
# Drone 2 (separate terminal)
python3 offboard_control.py --ros-args -r __ns:=/px4_1 2>&1 | tee drone2_log.txt
```

> ⚠️ Wait for `INFO [commander] Ready for takeoff!` in BOTH PX4 terminals before launching Python nodes.
> ⚠️ Always kill Python nodes with Ctrl+C before relaunching — stale nodes corrupt the setpoint stream.
> ⚠️ Use `cat drone1_log.txt | grep "keyword"` to filter logs after flight.

---

### Workspace layout:
```
~/Projects/multi_drone_payload_lifting/
└── ros2_ws/
    └── src/
        ├── px4_msgs/          (dependency, not committed to git)
        ├── offboard_control.py
        └── drone_listener.py
```

### Key .bashrc settings:
```bash
alias ros2dev="source /opt/ros/jazzy/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # critical — do NOT remove
# CYCLONEDDS_URI is commented out — no config file = auto-discover loopback
export ROS_DOMAIN_ID=0
export GZ_SIM_SYSTEM_PLUGIN_PATH=...           # PX4 Gazebo plugins
export QT_QPA_PLATFORM=xcb
export PATH=$PATH:/opt/xtensa-esp-elf/bin/
# venv auto-activate is COMMENTED OUT — activate manually when needed
```

### Python venv situation:
- `empy==3.3.4` is installed (NOT 4.x — 4.x breaks ROS 2 builds)

### Known quirks already solved:
| Problem | Fix |
|---|---|
| `ros2 topic list` empty | Always use `--spin-time 5`; restart daemon with `ros2 daemon stop && ros2 daemon start` |
| CycloneDDS vs FastDDS mismatch | Keep `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, remove CYCLONEDDS_URI |
| empy version | Pin to 3.3.4, NOT latest |
| colcon uses wrong Python | Always deactivate venv first, delete build/install/log and rebuild |
| `libtinfo5` 404 on Noble | Symlink `.so.6 → .so.5` (already done) |
| Gazebo camera frozen | Press Escape to exit select mode |
| Absolute vs relative topic names | Remove leading `/` from all topic strings — absolute paths bypass ROS 2 namespacing |
| target_system must match instance | Drone 1 = system 1, Drone 2 = system 2 — parse from namespace at runtime |
| `Ignore command X from 255/190 to 2/1` on drone 1 terminal | Normal — drone 1 sees drone 2's commands and correctly ignores them |
| `Accel #0 fail: TIMEOUT` on drone 2 | SITL CPU load issue — wait for `Ready for takeoff!` before launching node |
| EKF drift during climb | Normal SITL behavior — drone wobbles slightly during EKF settling, not a code bug |
| Drone returns to spawn on relaunch | Phase 1 now publishes `self.pos` not `(0,0,0)` — fixed |
| `//fmu/out/...` topic (double slash) | Use `''` not `'/'` for drone 1 namespace in drones_dict |

---

## First ROS 2 Node — Working ✅

File: `~/Projects/multi_drone_payload_lifting/ros2_ws/src/drone_listener.py`

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition

class DroneListener(Node):
    def __init__(self):
        super().__init__('drone_listener')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position_v1',
            self.callback,
            qos
        )

    def callback(self, msg):
        self.get_logger().info(f'x={msg.x:.2f}  y={msg.y:.2f}  z={msg.z:.2f}')

def main():
    rclpy.init()
    rclpy.spin(DroneListener())

main()
```

---

## Offboard Controller — Working ✅

File: `~/Projects/multi_drone_payload_lifting/ros2_ws/src/offboard_control.py`

State machine: `PREFLIGHT → ARMING → OFFBOARD → HOVER → WAYPOINT → HOLD → LAND → DISARMED`

### Current full implementation:

```python
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

class OffboardController(Node):
    def __init__(self):
        super().__init__('offboard_controller')

        # Drone identity
        namespace = self.get_namespace()
        if namespace == '/':
            self.instance = 0
        else:
            self.instance = int(namespace.split('_')[-1])

        total_drones = 2
        drones_dict = {i: '' if i == 0 else f'px4_{i}' for i in range(0, total_drones)}

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Publishers
        self.ocm_pub = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', px4_qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', px4_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', px4_qos)

        # Subscribers
        self.create_subscription(VehicleStatus, 'fmu/out/vehicle_status_v4', self.status_cb, px4_qos)
        self.create_subscription(VehicleLocalPosition, 'fmu/out/vehicle_local_position_v1', self.position_cb, px4_qos)
        self.create_subscription(
            VehicleLocalPosition,
            f'{drones_dict[(self.instance + 1) % 2]}/fmu/out/vehicle_local_position_v1',
            self.companion_pos_cb, px4_qos)

        # State
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.pos = (0.0, 0.0, 0.0)
        self.companion_pos = (0.0, 0.0, 0.0)
        self.current_yaw = 0.0
        self.facing_yaw = 0.0
        self.tick = 0
        self.hold_ticks = 0

        # Waypoints (NED) — formation positions 2m apart along Y
        self.hover_z = -2.0
        waypoints = {
            0: (5.0, -1.0, -2.0),
            1: (5.0,  1.0, -2.0)
        }
        self.waypoint = waypoints[self.instance]

        # Flags
        self.offboard = False
        self.reached_hover = False
        self.reached_waypoint = False
        self.REACH_THRESHOLD = 0.3
        self.sent_landing_command = False
        self.landing_complete = False
        self.yaw_initialized = False

        self.create_timer(0.05, self.timer_cb)

    def status_cb(self, msg): self.nav_state = msg.nav_state; self.arming_state = msg.arming_state
    def position_cb(self, msg): self.pos = (msg.x, msg.y, msg.z)
    def companion_pos_cb(self, msg): self.companion_pos = (msg.x, msg.y, msg.z)

    def timer_cb(self):
        self.tick += 1

        # Update yaw only when close to waypoint (reduces oscillation during flight)
        if self._distance_to(*self.waypoint) < 2.0:
            self.facing_yaw = self._compute_relative_yaw(self.pos, self.companion_pos)

        self._publish_ocm()

        # Phase 1: pre-stream setpoints (hold current position)
        if self.tick < 40:
            self._publish_setpoint(self.pos[0], self.pos[1], self.pos[2])
            return

        # Phase 2: switch to offboard + arm
        if self.tick == 40:
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if not self.offboard:
            if self.nav_state == 14:
                self._arm()
                self.offboard = True
            return

        # Phase 3: climb to hover altitude (no yaw — avoids drift)
        if not self.reached_hover:
            self._publish_setpoint(0.0, 0.0, self.hover_z)
            if self._distance_to(0.0, 0.0, self.hover_z) < self.REACH_THRESHOLD:
                self.reached_hover = True
            return

        # Phase 4: fly to formation waypoint + face companion
        if not self.reached_waypoint:
            self._publish_setpoint(*self.waypoint, self.facing_yaw)
            if self._distance_to(*self.waypoint) < self.REACH_THRESHOLD:
                self.reached_waypoint = True
            return

        # Phase 5: hold formation + face companion, then land
        if not self.sent_landing_command:
            self._publish_setpoint(*self.waypoint, self.facing_yaw)
            self.hold_ticks += 1
            if self.hold_ticks >= 100:
                self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.sent_landing_command = True

        # Phase 6: detect landing
        if self.sent_landing_command and not self.landing_complete:
            if self.arming_state == 1:
                self.get_logger().info('Landing detected - drone disarmed.')
                self.landing_complete = True

    def _compute_relative_yaw(self, pos, companion_pos):
        """
        Compute yaw angle to face companion drone.
        Uses proportional controller with angle unwrapping and deadband.
        """
        alpha = 0.05
        dx = companion_pos[0] - pos[0]
        dy = companion_pos[1] - pos[1]

        if dx == 0 and dy == 0:
            return self.current_yaw

        target_yaw = math.atan2(dy, dx)

        # Lazy init: snap to real angle on first valid reading (avoids startup swing)
        if not self.yaw_initialized and self.companion_pos != (0.0, 0.0, 0.0):
            self.current_yaw = target_yaw
            self.yaw_initialized = True

        # Normalize error to [-π, π] to handle atan2 wrap-around
        error = target_yaw - self.current_yaw
        error = (error + math.pi) % (2 * math.pi) - math.pi

        # Deadband: ignore tiny errors to prevent oscillation
        if abs(error) < math.pi / 72:  # ~2.5 degrees
            return self.current_yaw

        new_yaw = self.current_yaw + alpha * error
        self.current_yaw = new_yaw
        return new_yaw

    def _publish_ocm(self):
        msg = OffboardControlMode()
        msg.timestamp = self._now()
        msg.position = True
        self.ocm_pub.publish(msg)

    def _publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = self._now()
        msg.position = [x, y, z]
        msg.yaw = yaw
        self.sp_pub.publish(msg)

    def _arm(self):
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._now()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self.instance + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def _distance_to(self, tx, ty, tz):
        return ((self.pos[0]-tx)**2 + (self.pos[1]-ty)**2 + (self.pos[2]-tz)**2) ** 0.5

    def _now(self):
        return self.get_clock().now().nanoseconds // 1000

def main():
    rclpy.init()
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Namespace and Instance Situation

- Drone 1: namespace = `/` → instance = 0 → target_system = 1 → DDS prefix: `fmu/...`
- Drone 2: namespace = `/px4_1` → instance = 1 → target_system = 2 → DDS prefix: `px4_1/fmu/...`

**Companion topic lookup:**
```python
drones_dict = {i: '' if i == 0 else f'px4_{i}' for i in range(0, total_drones)}
companion_ns = drones_dict[(self.instance + 1) % 2]
# Drone 0 → companion is drones_dict[1] = 'px4_1'
# Drone 1 → companion is drones_dict[0] = ''
```

**Important:** PX4 uses NED coordinates — Z is negative when flying up. Topic name has `_v1` suffix. Always use `BEST_EFFORT` QoS.

---

## Key PX4 Topics

**Read (`/fmu/out/...`):**
- `vehicle_local_position_v1` — x, y, z, vx, vy, vz (NED, relative to home/spawn)
- `vehicle_attitude` — quaternion orientation
- `vehicle_status_v4` — armed state, flight mode (nav_state=14 = offboard)
- `vehicle_global_position` — GPS lat/lon/alt
- `battery_status_v1` — voltage, remaining %

**Write (`/fmu/in/...`):**
- `offboard_control_mode` — must publish at >2Hz or PX4 exits offboard
- `trajectory_setpoint` — target position [x,y,z] + yaw (NED, radians)
- `vehicle_command` — arm, disarm, mode change

---

## Facing Algorithm — Working ✅

**Goal:** each drone yaws to face the other during formation hold.

**Key design decisions and why:**
- `atan2(dy, dx)` not `atan(dy/dx)` — handles dx=0 and correct quadrant
- Lazy yaw initialization — snap `current_yaw` to real angle on first valid reading, not 0.0, to avoid startup swing
- Angle unwrapping `(error + π) % (2π) - π` — prevents 6-radian jumps when atan2 crosses ±π boundary
- P controller with `alpha=0.05` — smooth approach instead of hard jump to target
- Deadband of `π/72` (~2.5°) — prevents oscillation from chasing noisy position updates
- Yaw only activated within 2.0m of waypoint — avoids yaw fighting position controller during flight
- No yaw during Phase 3 climb — yaw during ascent causes horizontal drift

**Coordinate frame note:** both drones share approximately the same NED origin (GPS locks near the same point), so companion positions are comparable. On hardware, UWB gives true relative ranging in a shared frame.

---

## Formation Flying — Working ✅

**Goal:** drones fly to positions 2m apart, face each other, hold for payload operations.

**Waypoints (NED, relative to spawn):**
```python
waypoints = {
    0: (5.0, -1.0, -2.0),   # Drone 1: 5m forward, 1m left, 2m up
    1: (5.0,  1.0, -2.0)    # Drone 2: 5m forward, 1m right, 2m up
}
```

Drones spawn 2m apart (world X), fly to formation 2m apart (local Y), face each other along Y axis. Yaw at formation: Drone 1 = +π/2, Drone 2 = -π/2 (computed automatically by `_compute_relative_yaw`).

---

## Immediate Next Steps

Completed:
1. ✅ Offboard control — full state machine working
2. ✅ Multi-drone simulation — two gz_x500 instances, namespace-based control
3. ✅ Drone-to-drone facing — yaw controller, angle unwrapping, P controller, deadband
4. ✅ Formation flying — instance-based waypoints, 2m separation, stable at destination

Next in order:
5. **Payload simulation** — attach cable between drones in Gazebo, test stability
6. **UWB driver** — ROS 2 node to read Nooploop LinkTrack serial data
7. **Hardware assembly** — parts arriving week 2-3
8. **Port to real drones** — RPi runs same nodes, same namespace approach

---

## Teaching Approach
One teammate has Linux/ROS2/RPi experience. Others are beginners.
- Experienced member mentors, does not write code for others
- Each feature owned end-to-end by one person
- Pair sessions: owner writes, mentor asks questions (Socratic method)
- Same debugging methodology throughout: read the error → get data → form hypothesis → test

---

## Team Structure (4 people)

| Person | Role | Owns |
|---|---|---|
| A | Flight software | Offboard control nodes, PID tuning, sim environment |
| B | Tracking + perception | UWB localization, drone-to-drone facing algorithm |
| C | Hardware | Frame assembly, thrust calcs, weight budget, payload bracket |
| D | Systems + comms | RPi setup, ROS 2 on hardware, MAVLink integration, Git |

**Tools:** GitHub (one repo), Discord (team comms), GitHub Projects (kanban), Overleaf (final report)

---

## Key Theory the Student Needs

- NED coordinate frame (Z negative = up)
- PID control — PX4 handles low-level, student writes high-level
- Thrust budget: `T = (m_drone + m_payload) × g × 2.0`
- LiPo: never below 20% charge (0.8 capacity factor in flight time calc)
- QoS in ROS 2: always BEST_EFFORT for PX4 topics
- Quaternions for attitude (not Euler — avoid gimbal lock)
- UWB gives relative distance, not position — needs geometry to get bearing
- Angle wrapping: atan2 returns [-π, π]; normalize errors with `(e + π) % (2π) - π`
- P controller: `new = current + alpha * error`; small alpha = smooth but slow

---

## What NOT to Re-discuss
- Frame choice (X500 v2 is decided)
- FC choice (Pixhawk 6C is decided)
- Battery choice (4S 5000mAh ×8 is decided)
- Grasping mechanism (servo hook + Dyneema is decided)
- Docker vs native (native is working, don't switch)
- ArduPilot vs PX4 (PX4 is decided and working)
- Facing algorithm design (atan2, P controller, unwrapping — all decided and working)