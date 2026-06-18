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
from geometry_msgs.msg import PointStamped

FORMATION_Y_OFFSET = 1.0  # metres — drones are 2 m apart on Y axis
CABLE_LENGTH = 3.1        # metres — vertical offset drones fly above payload ref


class CraneController(Node):
    """
    Teleoperated crane controller.
    Phases 1-3 identical to offboard_control.py.
    Phase 4: continuously tracks /payload_position_ref instead of a hardcoded waypoint.
    Each drone derives its own NED target from the shared payload ref:
        drone 0 → (px, py - 1.0, pz - CABLE_LENGTH)
        drone 1 → (px, py + 1.0, pz - CABLE_LENGTH)
    Stop with Ctrl+C.
    """

    def __init__(self):
        super().__init__('crane_controller')

        namespace = self.get_namespace()
        self.instance = 0 if namespace == '/' else int(namespace.split('_')[-1])
        self.get_logger().info(f'Instance: {self.instance}, target_system: {self.instance + 1}')

        total_drones = 2
        drones_dict = {i: '' if i == 0 else f'px4_{i}' for i in range(total_drones)}

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Publishers
        self.ocm_pub = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', px4_qos)
        self.sp_pub  = self.create_publisher(TrajectorySetpoint,   'fmu/in/trajectory_setpoint',    px4_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand,        'fmu/in/vehicle_command',         px4_qos)

        # Subscribers
        self.create_subscription(VehicleStatus,       'fmu/out/vehicle_status_v4',          self.status_cb,       px4_qos)
        self.create_subscription(VehicleLocalPosition, 'fmu/out/vehicle_local_position_v1', self.position_cb,     px4_qos)
        self.create_subscription(
            VehicleLocalPosition,
            f'{drones_dict[(self.instance + 1) % 2]}/fmu/out/vehicle_local_position_v1',
            self.companion_pos_cb, px4_qos)
        self.create_subscription(PointStamped, '/payload_position_ref', self.payload_ref_cb, 10)

        # State
        self.nav_state    = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.pos           = (0.0, 0.0, 0.0)
        self.companion_pos = (0.0, 0.0, 0.0)
        self.current_yaw   = 0.0
        self.facing_yaw    = 0.0
        self.yaw_initialized = False
        self.tick = 0

        self.hover_z       = -3.5   # NED metres (negative = up)
        self.payload_ref   = None   # (x, y, z) NED, set by subscriber
        self.commanded_pos = None   # velocity-limited position sent to PX4

        # Flags
        self.offboard      = False
        self.reached_hover = False
        self.REACH_THRESHOLD = 0.3
        self.desired_speed = 0.5    # m/s — slower than offboard_control for payload stability
        self.dt = 0.05

        self.create_timer(0.05, self.timer_cb)
        self.get_logger().info('Crane controller ready.')

    # ------------------------------------------------ #
    # Callbacks                                         #
    # ------------------------------------------------ #

    def status_cb(self, msg: VehicleStatus):
        self.nav_state    = msg.nav_state
        self.arming_state = msg.arming_state

    def position_cb(self, msg: VehicleLocalPosition):
        self.pos = (msg.x, msg.y, msg.z)

    def companion_pos_cb(self, msg: VehicleLocalPosition):
        self.companion_pos = (msg.x, msg.y, msg.z)

    def payload_ref_cb(self, msg: PointStamped):
        self.payload_ref = (msg.point.x, msg.point.y, msg.point.z)

    # ------------------------------------------------ #
    # Main 20 Hz loop                                   #
    # ------------------------------------------------ #

    def timer_cb(self):
        self.tick += 1
        self._publish_ocm()

        # Phase 1: pre-stream setpoints so PX4 sees a stream before mode switch
        if self.tick < 40:
            self._publish_setpoint(*self.pos)
            return

        # Phase 2: switch to offboard + arm
        if self.tick == 40:
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        if not self.offboard:
            if self.nav_state == 14:
                self._arm()
                self.offboard = True
            return

        # Phase 3: climb to hover altitude (synchronized with companion)
        if not self.reached_hover:
            companion_z = self.companion_pos[2]
            climb_z = 0.0 if companion_z == 0.0 else max(self.hover_z, companion_z - 0.3)
            self._publish_setpoint(0.0, 0.0, climb_z)

            if self._distance_to(0.0, 0.0, self.hover_z) < self.REACH_THRESHOLD:
                if self.companion_pos[2] <= self.hover_z + self.REACH_THRESHOLD:
                    self.reached_hover = True
                    self.commanded_pos = self.pos  # seed from actual hover position
                    self.get_logger().info('Hover reached — waiting for /payload_position_ref')
            return

        # Phase 4: crane mode — follow /payload_position_ref continuously
        if self.payload_ref is None:
            if self.tick % 40 == 0:
                self.get_logger().info('Waiting for /payload_position_ref ...')
            self._publish_setpoint(*self.commanded_pos, self.facing_yaw)
            return

        target = self._compute_drone_target()
        self.commanded_pos = self._step_toward(self.commanded_pos, target, self.desired_speed * self.dt)
        self.facing_yaw = self._compute_relative_yaw(self.pos, self.companion_pos)
        self._publish_setpoint(*self.commanded_pos, self.facing_yaw)

    # ------------------------------------------------ #
    # Crane helpers                                     #
    # ------------------------------------------------ #

    def _compute_drone_target(self):
        """Derive per-drone NED target from shared payload reference."""
        px, py, pz = self.payload_ref
        y_sign = -1.0 if self.instance == 0 else 1.0
        return (px, py + y_sign * FORMATION_Y_OFFSET, pz - CABLE_LENGTH)

    def _step_toward(self, current, target, max_step):
        """Move current position toward target by at most max_step metres."""
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        dz = target[2] - current[2]
        dist = (dx**2 + dy**2 + dz**2) ** 0.5
        if dist <= max_step or dist == 0:
            return target
        scale = max_step / dist
        return (current[0] + dx * scale, current[1] + dy * scale, current[2] + dz * scale)

    # ------------------------------------------------ #
    # Shared helpers (identical to offboard_control.py) #
    # ------------------------------------------------ #

    def _publish_ocm(self):
        msg = OffboardControlMode()
        msg.timestamp    = self._now()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        self.ocm_pub.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = self._now()
        msg.position  = [x, y, z]
        msg.yaw       = yaw
        self.sp_pub.publish(msg)

    def _arm(self):
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent.')

    def _send_vehicle_command(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp        = self._now()
        msg.command          = command
        msg.param1           = param1
        msg.param2           = param2
        msg.target_system    = self.instance + 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self.cmd_pub.publish(msg)

    def _distance_to(self, tx: float, ty: float, tz: float) -> float:
        dx = self.pos[0] - tx
        dy = self.pos[1] - ty
        dz = self.pos[2] - tz
        return (dx**2 + dy**2 + dz**2) ** 0.5

    def _compute_relative_yaw(self, pos, companion_pos):
        alpha = 0.05
        dx = companion_pos[0] - pos[0]
        dy = companion_pos[1] - pos[1]
        if dx == 0 and dy == 0:
            return self.current_yaw
        target_yaw = math.atan2(dy, dx)
        if not self.yaw_initialized and self.companion_pos != (0.0, 0.0, 0.0):
            self.current_yaw = target_yaw
            self.yaw_initialized = True
        error = target_yaw - self.current_yaw
        error = (error + math.pi) % (2 * math.pi) - math.pi
        if abs(error) < math.pi / 72:
            return self.current_yaw
        self.current_yaw = self.current_yaw + alpha * error
        return self.current_yaw

    def _now(self) -> int:
        return self.get_clock().now().nanoseconds // 1000


def main():
    rclpy.init()
    node = CraneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
