# Drone Project — Handoff (v10, condensed)
> **Direction: Option B — Teleoperated drone crane with 3-DOF arm.**

## Status / Pickup (2026-06-23)
**Working now:**
- Full 2-drone crane teleop **verified end-to-end in sim** (RadioMaster sticks → `/payload_position_ref` → both drones carry payload).
- **Optical-flow sim (phases 1+2 DONE).** Single `x500_flow` over textured granite, GPS off, flies on flow+rangefinder only. Drift now **measured** vs Gazebo ground truth via RViz harness: **~0.46 m total after a 3 m forward move** (mostly lateral). See **Optical Flow Sim** + **Eval Harness**.
- **Phase 3 detection world DONE (2026-06-23).** Downward `mono_cam` added to `x500_flow` (publishes image + camera_info, looks down). New `apriltag_tags` world: tag36h11 ids 0/1/2 as thin boxes along the NED-forward (Gazebo +Y) path. **Verified the downward camera sees crisp tags in flight.** See **AprilTag Sim**.

**Do next:** Phase 3 **detection wiring** — bridge camera (`ros_gz_image` image + `parameter_bridge` camera_info) → `apriltag_ros` (`ros-jazzy-apriltag-ros`, family tag36h11, size 0.5) → confirm `/detections` populates. **Then fusion:** `apriltag_odom.py` (tag pose → `/fmu/in/vehicle_visual_odometry`, NED) + bake EKF2 EV params into airframe 4021; re-run eval_harness to show drift collapse. Then 3-DOF arm driver (pending arm interface from professor).
- Open item: flow match count only **6/13** at 3.5 m (granite texture soft) → make `granite.png` more corner-rich if motion shows lock loss.
- Durable TODO: `flow_hover_test.py` arms at a fixed tick with no EKF2-readiness check → gate the arm on `VehicleLocalPosition.xy_valid && z_valid` (see Key Quirks).

## Teaching Rules
- Socratic method only — never give direct answers immediately
- Ask student to read errors literally before debugging
- Tick-by-tick traces before running code
- Group 2–3 related questions per response
- Bugs are teaching moments — reveal answer only after 1–2 failed attempts
- Celebrate correct reasoning, not just correct output

---

## Project Goal
Two X500 quadcopters act as a **teleoperated drone crane**. A human moves a lab 3-DOF arm; the drone pair follows in real time, keeping the suspended payload at the commanded position. A flying crane with a human in the loop — not autonomous A→B transport.

## Hardware (Final — do not re-discuss)
| Item | Choice |
|---|---|
| Frame | Holybro X500 v2 ×2 |
| FC | Pixhawk 6C ×2 |
| Companion | Raspberry Pi 4 (4GB) ×2 — **lab-provided** |
| Tracking | **AprilTag downward cameras ×2** (global-shutter mono, e.g. Arducam OV9281 USB). UWB dropped — see Localization |
| Battery | 4S 5000mAh 30C ×8 |
| Release | SG90/MG90S servo ×2 + Dyneema 50kg + steel hooks |
| RC | RadioMaster Pocket **ELRS variant** + RP2 receiver (2.4GHz ELRS) |
| Localization | Optical flow (velocity) + AprilTags (absolute) — **DECIDED 2026-06-21** |
| Operator input | Lab 3-DOF arm — interface TBC with professor |

Radio roles: SiK V3 = MAVLink/telemetry to QGC over 433MHz; ELRS RC = manual stick input to Pixhawk via CRSF.

### Procurement / Order List (as of 2026-06-21)
**Have:** 1× X500 v2 dev kit (incl. Pixhawk 6C, PM02 V3 power module, M8N GPS, **SiK V3 radio pair**, frame w/ motors+ESCs+1045 props), 1× 4S battery, 1× RadioMaster Pocket ELRS + RP2 rx. Pi 4 (4GB) ×2 **lab-provided**.

**To order:**
- **1× X500 v2 dev kit** — covers 2nd frame + Pixhawk 6C + M8N GPS + power module + 2nd SiK radio in one box (cheaper/cleaner than buying bare parts).
- **1× 4S 5000mAh 30C battery** (→ 2 total = one flyable set for both drones; buy more later for charge-rotation during test sessions).
- **2× AprilTag camera** — global-shutter **mono** USB, e.g. Arducam OV9281 (~$40–60 ea). One per drone. Global shutter required (rolling shutter warps tags under vibration/swing). Prices from general knowledge, not a live lookup.
- **2× SG90/MG90S servo** + Dyneema 50kg line + steel hooks (payload release).
- AprilTags themselves: printed on paper (free).

**Not ordering:** Nooploop UWB (dropped — see Localization); standalone SiK radio (comes in dev kit); Raspberry Pi (lab-provided).

---

## Control Architecture
```
Input source (swappable) → /payload_position_ref (geometry_msgs/PointStamped, NED)
      → crane_control.py (one per drone) → per-drone TrajectorySetpoint → PX4 offboard → cable tension → payload
```
**Key decision:** input source is fully decoupled from flight control. `crane_control.py` only subscribes to `/payload_position_ref` — no knowledge of how the point is generated or that 2 drones exist on the operator side. Makes all input sources interchangeable.

| File | Input source | Status |
|---|---|---|
| `test_ref_publisher.py` | Automated sine sweep on X | ✅ |
| `joystick_ref_publisher.py` | RadioMaster → `/joy` | ✅ verified in full sim |
| 3-DOF arm driver | Lab arm joint angles → FK | ⬜ pending arm interface |

**Per-drone target** (`crane_control.py::_compute_drone_target`) from payload ref `(px,py,pz)` NED:
- Drone 0: `(px, py − 1.0, pz − CABLE_LENGTH)`
- Drone 1: `(px, py + 1.0, pz − CABLE_LENGTH)`
- `FORMATION_Y_OFFSET = 1.0`, `CABLE_LENGTH = 3.1`. Formation stays 2 m on Y; only payload ref moves.

**Phase 3 climb is synchronized to the companion drone's altitude** → a single drone alone will NOT take off (`climb_z=0` while `companion_z==0`). Both drones required.

**Phase 4 (crane mode):** velocity-limits (`_step_toward`, 0.5 m/s) toward the live payload-derived target; tracks continuously. No landing phase — stop with Ctrl+C. `offboard_control.py` kept as the single-drone baseline (hardcoded waypoint + land).

**joystick_ref_publisher.py:** `joy_cb` stores axis values; `timer_cb` integrates `pos += axis*MAX_SPEED*dt` at 20 Hz, publishes. Deadzone 0.1. Starts at `INIT=(0,1,0)` (= formation center, matches a stable hover, no lurch). Verified axis/sign map: `AXIS_X=1/SIGN_X=−1`, `AXIS_Y=0/SIGN_Y=−1`, `AXIS_Z=2/SIGN_Z=+1`. Needs `ros-jazzy-joy`.

---

## Launch Sequence (default DDS — no CYCLONEDDS_URI exports)
```bash
# T1 — agent
MicroXRCEAgent udp4 -p 8888
# T2 — world
python3 ~/src/PX4-gazebo-models/simulation-gazebo --world=cable_world
# T3 — drone 0
cd ~/src/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_POSE="0,0,0,0,0,1.57" PX4_GZ_WORLD=cable_world make px4_sitl gz_x500
# T4 — drone 1 (from ~/src/PX4-Autopilot)
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=cable_world PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="2,0,0,0,0,1.57" PX4_INSTANCE=1 ./build/px4_sitl_default/bin/px4 -i 1
#   ↑ wait for "Ready for takeoff!" in BOTH, and wait for the bridge too!
# T5/T6 — controllers (from ros2_ws/src)
python3 crane_control.py
python3 crane_control.py --ros-args -r __ns:=/px4_1
#   ↑ wait for "Hover reached" in BOTH
# T7/T8 — payload reference (pick ONE):
python3 test_ref_publisher.py                 # automated sine
# --- OR radio teleop ---
ros2 run joy joy_node
python3 joystick_ref_publisher.py
```
- World files must be copied to `~/.simulation-gazebo/worlds/`
- Always Ctrl+C Python nodes before relaunching
- **Full sim cleanup** (gz sim server survives the launcher): `pkill -9 -f "gz sim server"; pkill -9 -f MicroXRCEAgent; pkill -9 -f "bin/px4"; pkill -9 -f simulation-gazebo`

### Flow sim (single drone, optical-flow only)
```bash
# T1 — agent
MicroXRCEAgent udp4 -p 8888
# T2 — world  (the export is REQUIRED — see Optical Flow Sim landmines)
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/src/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins
python3 ~/src/PX4-gazebo-models/simulation-gazebo --world=flow_world
# T3 — single x500_flow (airframe 4021, GPS off)
cd ~/src/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=flow_world \
  PX4_SYS_AUTOSTART=4021 PX4_SIM_MODEL=x500_flow PX4_GZ_MODEL_POSE="0,0,0,0,0,0" make px4_sitl gz_x500_flow
# in pxh> once booted (headless flight, no QGC):
#   param set NAV_DLL_ACT 0
# T4 — WAIT for "Ready for takeoff!" in T3 first (EKF2 must converge — see Key Quirks),
#      then solo takeoff + hover (from ros2_ws/src)
python3 flow_hover_test.py
```
The downward `mono_cam` + `apriltag_tags` are loaded by this same flow sim (camera is passive
for flow-only runs). For AprilTag detection add: image bridge + camera_info bridge + `apriltag_ros`
— see **AprilTag Sim**.

---

## Coordinate Frames (critical)
- Gazebo world: X=sideways, Y=forward, Z=up
- Python local NED: X=forward, Y=sideways, Z=down (negative = up)
- ⚠️ **NED-forward (NED +x) = Gazebo +Y.** Lay ground assets along the flight path by varying **Y**, not X.
- Drone 0 spawns Gazebo (0,0,0) = NED origin; Drone 1 Gazebo (2,0,0) = NED (0,2,0)
- Payload spawns Gazebo (1,0,0.1) — midpoint
- PX4 NED: Z negative = up

## Namespace & Instance
- Drone 0: ns `/` → instance 0 → target_system 1 → topics `fmu/...`
- Drone 1: ns `/px4_1` → instance 1 → target_system 2 → topics `px4_1/fmu/...`

## State Machine
`PREFLIGHT → ARMING → OFFBOARD → HOVER → (Phase 4: track ref) → Ctrl+C`
- Phase 1 (tick<40): pre-stream setpoints at current pos
- Phase 2 (tick=40): offboard + arm
- Phase 3: climb to `hover_z=−3.5`, synced to companion
- Phase 4: track `/payload_position_ref`

---

## Payload & Cable (tuned, efficient enough)
- `base_link` world (1,0,0.1) — **1.5 kg**, box **1.5×0.2×0.2** (horizontal bar)
- `tip0_link`/`tip1_link` at relative (∓1,0,0) → below each drone. 0.02 kg, fixed joints, sphere collision r=0.01
- ⚠️ Payload X dimension must stay **≤2.0 m** — at exactly 2.0 m tips collide with drone landing gear at spawn → physics explosion. Current bar 1.5 m, tips at ±1.0 m.
- **SDF cable params (per instance):** `rest_length=2.5`, `stiffness=50`, `damping=40`
- **Controller offset (separate):** `CABLE_LENGTH=3.1` in crane_control.py

### Cable Plugin
- File `simulation/src/cable.cpp`; build `cd simulation/build && cmake .. && make` → `libcable_plugin.so`
- Two instances: x500_0↔tip0_link, x500_1↔tip1_link. `active_=true` default (slack near ground)
- gz-transport Node must be a class member; wrench persists — explicitly zero when slack
- SDF: `<plugin filename="libcable_plugin.so" name="CablePlugin">` with `<model0>/<link0>/<model1>/<link1>/<rest_length>/<stiffness>/<damping>`
- C++ uses `model.LinkByName(ecm, link_name)`; `link0_name_ = sdf->Get<std::string>("link0","base_link").first`

---

## Software Stack
`Gazebo (standalone) ↕ PX4 SITL / Pixhawk 6C ↕ uXRCE-DDS ↕ ROS 2 Jazzy (Ubuntu 24.04) ↕ Python`
- RMW: `rmw_cyclonedds_cpp` (required — FastDDS RMW breaks). **No `CYCLONEDDS_URI`** — default works.
- `ROS_DOMAIN_ID=0`; empy pinned 3.3.4; PX4 from source `~/src/PX4-Autopilot`; venv activated manually, deactivate before colcon builds.

---

## Localization (DECIDED 2026-06-21 — optical flow + AprilTags, fused in EKF2)
**Chosen plan:** AprilTags (absolute, kills drift) + optical flow (smooth velocity), fused in EKF2. Lowest cost AND the only path validatable in sim before buying any hardware (Phase 3 pipeline already being built). Under ~$150.
**Considered and dropped:** UWB (Nooploop LinkTrack P — ~10 cm, multipath-twitchy indoors, weak reviews; DIY Qorvo same physics + worse support), Lighthouse (sub-mm but no off-the-shelf PX4 receiver → too much integration for the timeline), Marvelmind (pricier, starter set lacks a 2nd mobile).
- Optical flow (PMW3901-class) + downward range (VL53L1X/TFmini): `velocity = flow_rate × height`. PX4 fuses natively. Velocity-only → drifts; fails on featureless/dark floors.
- AprilTag: camera + known tag size + 4 corners → 6-DOF pose via PnP. Needs line-of-sight + light. Idea: tag ON payload → downward cam measures cable swing.
- **Camera:** global-shutter **mono** (e.g. Arducam OV9281 USB), one per drone. Global shutter is required — rolling shutter warps tags under prop vibration/swing → bad corner detection/PnP. AprilTags need only grayscale.

**Sim plan (phased):** (1) ✅ optical flow in sim → (2) ✅ RViz harness (ground-truth vs EKF2) → (3) AprilTags: downward cam in SDF, tag textures in world, `ros_gz_bridge` image → `apriltag_ros` → `VehicleVisualOdometry` → EKF2.

---

## Optical Flow Sim (phases 1+2 DONE — flow-only flight + drift measured)
**Don't hand-author a flow camera — PX4 ships `x500_flow`** (= x500 + `optical_flow` model + `LW20` rangefinder + downward lidar). Airframe **`4021_gz_x500_flow`** wires the params AND disables GPS (`SYS_HAS_GPS 0`, `SIM_GPS_USED 0`, `EKF2_GPS_CTRL 0`) — so selecting it IS the flow-only test.

**Pipeline:** Gazebo flow_camera (100×100) → `OpticalFlowSystem` plugin (computes flow) → gz topic → PX4 gz bridge → `sensor_optical_flow` uORB → EKF2 (`EKF2_OF_CTRL=1` flow, `EKF2_RNG_CTRL=1` height). Flow only means anything ×height (`velocity = flow_rate × height`) → needs the rangefinder; both fuse together.

**Files (repo `simulation/`, then copy to `~/.simulation-gazebo/`):**
- `worlds/flow_world.sdf` — minimal: physics + gz systems + light + granite ground. **Single drone, no payload/cable** (strips the debug surface to just flow).
- `models/granite_ground/` — textured box (25×25, top at z=0), PBR `albedo_map` = procedural granite (`materials/textures/granite.png`, 2048²). **Untextured floor = blind flow camera** (every pixel identical → 0 matches). Swap in a real lab-floor photo later.
- `ros2_ws/src/flow_hover_test.py` — solo takeoff + hover, NO companion gate (unlike `offboard_control.py`). Climb to `hover_z=-3.5`, hold, Ctrl+C to stop.

**Two landmines (both cost time if missed):**
1. **`OpticalFlowSystem` must be both FOUND and LOADED.** The `simulation-gazebo` launcher has the plugin-path line commented out, so `export GZ_SIM_SYSTEM_PLUGIN_PATH=~/src/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins` **before launching the world** (inherited; launcher won't clobber it). AND the world must list `<plugin filename="OpticalFlowSystem" name="custom::OpticalFlowSystem"/>` — path alone only lets gz *find* it; the System must be *requested*. If only the path is set, the gz `optical_flow` topic appears in `gz topic -l` (from PX4's *subscriber*) but `gz topic -i` shows **"No publishers"** → sensor not running.
2. **Headless flight needs `param set NAV_DLL_ACT 0`** (data-link-loss action). >0 ⇒ PX4 requires a GCS/RC link to arm+fly; with no QGC/RC it arms then `Disarmed by auto preflight disarming` (armed but never took off). Set to 0 to fly purely from ROS 2 offboard. (Runtime set resets on PX4 restart — bake into the 4021 airframe to persist.)

**⚠️ Verification trap:** a steady `vehicle_local_position` does NOT prove no drift — that reading IS the flow output (EKF2's estimate); flow can drift while EKF2 reports a stable hover. Real drift only shows vs Gazebo ground truth → **Eval Harness** below.

**Quality note:** match count 1/20 on ground (camera too close → blurry), 6/13 at 3.5 m. Soft granite limits the feature matcher; make texture more corner-rich if motion shows lock loss.

---

## AprilTag Sim (Phase 3 — absolute fix for flow drift)
**Goal:** downward camera sees ground AprilTags → absolute position fix fused in EKF2 → kills the ~0.46 m flow drift. **Detection world DONE (2026-06-23); detection wiring + EKF2 fusion are next.**

**Pipeline (built through step 3; 4–5 deferred):**
```
1 gz downward camera (on x500_flow)  →  image + camera_info gz topics            [DONE]
2 ros_gz bridge                      →  sensor_msgs/Image + CameraInfo           [next]
3 apriltag_ros (tag36h11, size 0.5)  →  /detections + tf(camera→tag)             [next, VERIFY here]
4 apriltag_odom.py                   →  /fmu/in/vehicle_visual_odometry (NED)    [deferred]
5 EKF2 (EKF2_EV_CTRL, baked in 4021) →  drift collapses in eval_harness          [deferred]
```

**Done so far (files):**
- **Downward camera:** edited store `~/.simulation-gazebo/models/x500_flow/model.sdf` — merge-include
  `model://mono_cam` (`<pose>0 0 0.10 0 1.5707 0</pose>`) + fixed `CameraJoint` (pitch `1.5707`),
  mirroring PX4 `x500_mono_cam_down`. A gz `camera` sensor looks down its link **+X**; +1.5707 pitch
  swings +X from forward to **down**. `mono_cam` resolves from the PX4 models path like
  `optical_flow`/`LW20` (no copy needed). Publishes `.../x500_flow_0/link/camera_link/sensor/camera/{image,camera_info}`.
  Passive for flow-only runs.
- **Tag world:** `simulation/models/apriltag_tags/` — one model, tag36h11 **ids 0/1/2** as thin
  `<box>` (0.5×0.5×0.002) at Gazebo **Y = 0 / 1.5 / 3.0** (= along the NED-forward flight path),
  model z=0.02. Textures = `apriltag-imgs` PNGs upscaled 10px→1000px with **Pillow `Image.NEAREST`**
  (crisp edges; bilinear blurs → detector fails). Full `model://` albedo URI + `<diffuse>` fallback.
  `flow_world.sdf` includes it once.

**Three landmines hit (and the fixes):**
1. **Tag row on the wrong axis.** NED-forward maps to **Gazebo +Y**, NOT +X (see Coordinate Frames).
   Vary **Y** to lay assets along the flight path.
2. **Zero-thickness `<plane>` z-fights / is one-sided** → tag flickers / looks half-submerged even
   straight down. Use a **thin `<box>`** for ground tags.
3. **EKF2-not-ready race** (see Key Quirks) — wait for `Ready for takeoff!` before flying.

**Next-session wiring (detection):** `sudo apt install ros-jazzy-apriltag-ros ros-jazzy-apriltag-msgs`;
bridge image via `ros_gz_image` + camera_info via `ros_gz_bridge parameter_bridge`
(`@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo`); run `apriltag_ros` (family `36h11`, `size: 0.5`,
remap to bridged image_rect + camera_info); verify `/detections` populates and tag TF appears while
flying the +Y move. Bridge is lazy — apriltag_ros subscribing wakes it. Then fusion (deferred): map
each tag_id to its known world-NED pose, compose with the detected camera↔tag transform + camera
mount offset → publish `px4_msgs/VehicleOdometry` (`POSE_FRAME_NED`) on `/fmu/in/vehicle_visual_odometry`;
bake `EKF2_EV_CTRL`/`EKF2_EV_DELAY`/`EKF2_EV_POS_{X,Y,Z}` into `4021_gz_x500_flow` beside the GPS lines.

---

## Eval Harness (phase 2 — measure flow drift in RViz)
**Goal:** overlay EKF2 estimate vs Gazebo ground truth; the gap = flow drift. Baseline measured: **~0.46 m after a 3 m move.**

**Data flow:**
```
Gazebo OdometryPublisher → gz /model/x500_flow_0/odometry ─(ros_gz_bridge)→ ROS nav_msgs/Odometry (ENU, TRUTH)
PX4 EKF2 → uXRCE-DDS → /fmu/out/vehicle_local_position_v1 (NED, ESTIMATE)
        both → eval_harness.py → /eval/{ekf2,truth}_path (Path) + TF → RViz (fixed frame "map")
```
- **Ground truth needs `OdometryPublisher`** — stock `x500`/`x500_flow` DON'T have it (only `x500_vision`). Added `<plugin filename="gz-sim-odometry-publisher-system" .../>` to store `x500_flow/model.sdf`. Without it, `/model/.../odometry_with_covariance` only *looks* alive (bridge's own subscriber) — `gz topic -e` hangs.
- **`eval_harness.py`:** converts estimate NED→ENU `(e,n,u)=(y,x,−z)`; zeroes each source to its 1st sample (kills the constant `base_link` vs `base_footprint` offset, which is NOT drift); publishes 2 Path trails + TF; logs `drift=‖est−truth‖` @1 Hz.
- **`ros_gz_bridge` is lazy** — relays only when a typed ROS subscriber connects (so `ros2 topic echo` alone won't wake it; the harness does).
- **Launch (add to flow sim):** `ros2 run ros_gz_bridge parameter_bridge /model/x500_flow_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry` → `python3 eval_harness.py` → `rviz2 -d eval_harness.rviz`.

**Topic-pipeline debug rule:** advertised ≠ publishing (`gz topic -l`/`ros2 topic list` show topics that may only have a *subscriber*). Test each hop source→sink with `gz topic -e -t … -n 1` / `ros2 topic echo … --once`; **hang = no real publisher → its plugin isn't loaded.** gz-side OK but ROS-side empty = bridge (check name+type, remember lazy). RViz blank/offset = frames/QoS (fixed frame, sign, origin).

---

## Key Quirks (solved)
| Problem | Fix |
|---|---|
| DDS discovery / `ros2 topic list` empty | Default Cyclone works post-reboot. If broken: reboot first. Localhost/loopback `CYCLONEDDS_URI` configs were a transient-WiFi band-aid — don't lead with them |
| `ros2` CLI hangs ~2min, `Errno 110` | Wedged daemon: `ss -tlnp \| grep 127.0.0.1:11511` → `kill -9 <pid>` (shows as python3; `pkill -f _ros2_daemon` misses it) |
| Phantom drone at world launch + IMU timestamp-error flood | Leftover `gz sim server` from a prior session (survives killing the launcher) — kill it explicitly |
| empy version | Pin 3.3.4 |
| Absolute topic names | No leading `/` — breaks namespacing |
| target_system | Drone 0=1, Drone 1=2 |
| `type="ignored"` in physics | Gazebo ignores ALL physics — never use |
| max_step_size | Use 0.004 (0.001 = 4× CPU load) |
| world files | Must copy to `~/.simulation-gazebo/worlds/` |
| Plugin name in SDF | Must match registered name exactly (`CablePlugin`) |
| Python→gz-transport | `subprocess.Popen(['gz','topic',...])` |
| SDF link/joint syntax | `<link name="...">` never `<link0>`; SDF joints: child/parent as text content, not attributes |
| Tip collision boxes at spawn | Keep Z≈0.001 to avoid landing-gear intersection |
| Joystick axis indices/signs | Controller-specific — verify with `ros2 topic echo /joy`; joy_node flips signs vs jstest |
| RadioMaster as PC joystick | EdgeTX → System → USB Mode → Joystick; appears as HID gamepad, read via `ros2 run joy joy_node` |
| QGC blank map / speechd error / scroll zoom | All harmless — use +/− buttons to zoom |
| Marvelmind quantity | Need 4 stationary + 2 mobile (starter set has only 1 mobile) |
| Flow sensor silent / `gz topic -i` says "No publishers" | `OpticalFlowSystem` not loaded. Export `GZ_SIM_SYSTEM_PLUGIN_PATH` (find) AND add `<plugin filename="OpticalFlowSystem" name="custom::OpticalFlowSystem"/>` to world (load). Topic in `gz topic -l` ≠ publishing — it can be the subscriber. |
| Armed then `Disarmed by auto preflight disarming` (headless) | No GCS/RC link → `param set NAV_DLL_ACT 0`. (Logger open/close + "Max log file size" lines are normal, not the cause.) |
| Textured ground renders pure black | PBR `albedo_map` failed to resolve — use full `model://name/...` URI + a `<diffuse>` fallback. View what the flow cam sees: GUI ⋮ → Image Display → camera image topic. |
| `offboard_control.py` won't fly solo | Phase-3 climb is companion-gated (`climb_z=0` while `companion_z==0`) → arms then auto-disarms. Use `flow_hover_test.py` for single drone. |
| Ground-truth odometry topic hangs on `gz topic -e` | `x500`/`x500_flow` lack `OdometryPublisher` (only `x500_vision` has it) — added it to store `x500_flow/model.sdf`. `_with_covariance` topic appearing = bridge's own subscriber, not a real publisher. |
| RViz harness: trails offset / flipped | Constant offset = `base_link` vs `base_footprint` reference (zero each to 1st sample). Flip = NED→ENU sign (`u=−z`). Blank = wrong Fixed Frame or `ros_gz_bridge` lazy (needs a real subscriber). |
| Ground asset on wrong axis (tags/markers off the flight path) | NED-forward = **Gazebo +Y**, not +X (Coordinate Frames). Lay assets along the path by varying **Y**. |
| `<plane>` tag/marker flickers / looks half-submerged in floor | Zero-thickness plane z-fights + is one-sided. Use a thin `<box>` (e.g. 0.5×0.5×0.002) raised a couple cm above the ground top. |
| AprilTag texture blurry → not detected | Source PNGs are ~10px; upscale with **nearest-neighbour** (Pillow `Image.NEAREST` / IM `-filter point`) before mapping onto the plane — bilinear blurs the cell edges. |
| Drone climbs infinitely; `ekf2 missing data` + `High Accelerometer Bias` | `flow_hover_test.py` arms at a fixed tick with no EKF2 check; launched before EKF2 converged (GPS off → rangefinder+flow take seconds). **Wait for `Ready for takeoff!`** before flying. Durable fix: gate arm on `VehicleLocalPosition.xy_valid && z_valid`. (Also do the full `pkill` clean — leftover `gz sim server` feeds bad IMU timestamps → accel bias.) |
