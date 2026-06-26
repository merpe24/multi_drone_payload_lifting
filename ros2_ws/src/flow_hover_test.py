import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

class FlowHoverTest(Node):
    """
    Single-drone takeoff + hover, with NO companion dependency.
    Purpose: validate optical-flow-only flight (GPS disabled, airframe 4021).

    State: stream setpoints -> OFFBOARD -> arm -> climb to hover_z -> hold.
    No landing phase: stop with Ctrl+C and observe whether the EKF2
    position estimate holds or drifts under flow-only.
    """

    def __init__(self):
        super().__init__('flow_hover_test')

        # Drone instance: 0 (default ns) or 1 (px4_1). Sets VehicleCommand
        # target_system = instance + 1, so drone 1 must run with -p instance:=1
        # or its arm/mode commands hit drone 0 (target_system 1).
        self.declare_parameter('instance', 0)
        self.instance = self.get_parameter('instance').value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.ocm_pub = self.create_publisher(
            OffboardControlMode, 'fmu/in/offboard_control_mode', px4_qos)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, 'fmu/in/trajectory_setpoint', px4_qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, 'fmu/in/vehicle_command', px4_qos)

        self.create_subscription(
            VehicleStatus, 'fmu/out/vehicle_status_v4', self.status_cb, px4_qos)
        self.create_subscription(
            VehicleLocalPosition, 'fmu/out/vehicle_local_position_v1',
            self.position_cb, px4_qos)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.pos = (0.0, 0.0, 0.0)
        self.tick = 0
        self.waypoint = (3.0, 0.0, -3.5)

        # Path tracking variables
        self.climb_progress = 0.0
        self.start_pos = (0.0, 0.0, 0.0)
        self.path_distance = 0.0
        self.path_progress = 0.0
        self.desired_speed = 1.0
        self.dt = 0.05

        # Flag Variables
        self.hover_z = -3.5          # NED, negative = up
        self.REACH_THRESHOLD = 0.3
        self.offboard = False
        self.mode_requested = False
        self.reached_hover = False
        self.reached_waypoint = False
        self.REACH_THRESHOLD = 0.3

        # EKF2 readiness flags (gate arming on these, not a fixed tick)
        self.xy_valid = False
        self.z_valid = False
        self.heading_ok = False

        self.create_timer(0.05, self.timer_cb)   # 20 Hz
        self.get_logger().info('Flow hover test ready.')

    def status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def position_cb(self, msg: VehicleLocalPosition):
        self.pos = (msg.x, msg.y, msg.z)
        self.xy_valid = msg.xy_valid
        self.z_valid = msg.z_valid
        self.heading_ok = msg.heading_good_for_control

    def _est_ready(self):
        """EKF2 flight-ready: valid horizontal + vertical position estimate.

        NOTE: heading_good_for_control is intentionally NOT required. In this
        GPS-off / flow-only config yaw is mag-derived and PX4 won't mark heading
        'good for control' until the vehicle has moved -> can't satisfy it before
        takeoff (deadlock). Position validity is the flag that actually prevents
        arming into an unconverged estimate (the runaway-climb cause)."""
        return self.xy_valid and self.z_valid

    def timer_cb(self):
        self.tick += 1

        # Always stream OffboardControlMode so PX4 stays in offboard
        self._publish_ocm()

        # Phase 1: pre-stream setpoints at current pos before requesting mode
        if self.tick < 40:
            self._publish_setpoint(self.pos[0], self.pos[1], self.pos[2])
            return

        # Phase 2: wait for EKF2 to be flight-ready, THEN request OFFBOARD + arm.
        # Arming on a fixed tick (before the estimator settles) lets the
        # heading/position estimate diverge during climb -> runaway climb and
        # sideways wander. Gate on the EKF2 validity flags instead.
        if not self.offboard:
            self._publish_setpoint(0.0, 0.0, self.pos[2])
            if not self._est_ready():
                if self.tick % 20 == 0:
                    self.get_logger().info(
                        f'waiting for EKF2: xy_valid={self.xy_valid} '
                        f'z_valid={self.z_valid} heading_ok={self.heading_ok}')
                return
            if not self.mode_requested:
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.mode_requested = True
            if self.nav_state == 14:  # OFFBOARD
                self._arm()
                self.offboard = True
            return

        # Phase 3: climb straight to hover_z (no companion gating)
        if not self.reached_hover:
            self._publish_setpoint(0.0, 0.0, self.hover_z)
            if self._distance_to(0.0, 0.0, self.hover_z) < self.REACH_THRESHOLD:
                self.reached_hover = True
                # Initiate path tracking variables
                self.start_pos = self.pos
                self.path_distance = self._distance_to(*self.waypoint)
                self.path_progress = 0.0
                
                self.get_logger().info(f'Hover reached: {self.pos} - holding')
            return

        # Phase 4: go to waypoint
        if not self.reached_waypoint:
            self.path_progress += self.desired_speed * self.dt
            if self.path_progress >= self.path_distance:
                self.path_progress = self.path_distance

            self.target = self._compute_lerp(self.path_progress, self.path_distance, self.start_pos, self.waypoint)
            self._publish_setpoint(*self.target)

            if self.path_progress >= self.path_distance and self._distance_to(*self.waypoint) < self.REACH_THRESHOLD:
                self.reached_waypoint = True
                self.get_logger().info('Waypoint reached - holding position')
            return

        # Phase 5: hold at waypoint
        self._publish_setpoint(*self.waypoint)

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
        self.get_logger().info('Arm command sent.')

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
        dx = self.pos[0] - tx
        dy = self.pos[1] - ty
        dz = self.pos[2] - tz
        return (dx**2 + dy**2 + dz**2) ** 0.5
    
    def _distance_between(self, point_1, point_2):
        dx = point_2[0] - point_1[0]
        dy = point_2[1] - point_1[1]
        dz = point_2[2] - point_1[2]
        return (dx**2 + dy**2 + dz**2) ** 0.5
    
    def _compute_lerp(self, progress, distance, start, end):
        ratio = progress / distance if distance > 0 else 1.0
        tx =  start[0] + (end[0] - start[0]) * ratio
        ty =  start[1] + (end[1] - start[1]) * ratio
        tz =  start[2] + (end[2] - start[2]) * ratio
        return (tx, ty, tz)

    def _now(self):
        return self.get_clock().now().nanoseconds // 1000

def main():
    rclpy.init()
    node = FlowHoverTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
