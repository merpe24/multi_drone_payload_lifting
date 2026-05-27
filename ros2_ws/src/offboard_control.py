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

class OffboardController(Node):
    """
    Minimal offboard controller.
    State machine: PREFLIGHT --> ARMING --> OFFBOARD --> HOVER --> WAYPOINT --> HOLD --> LAND
    """

    def __init__(self):
        super().__init__('offboard_controller')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Publishers
        self.ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        
        # Subscribers
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_cb, px4_qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.position_cb, px4_qos)
        

        # State
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.pos = (0.0, 0.0, 0.0)
        self.tick = 0
        self.hold_ticks = 0

        #Waypoint (NED)
        self.hover_z = -2.0
        self.waypoint = (3.0, 0.0, -2.0)

        # State variables
        self.offboard = False
        self.reached_hover = False
        self.reached_waypoint = False
        self.REACH_THRESHOLD = 0.3
        self.sent_landing_command = False
        self.landing_complete = False

        # 20 Hz control loop
        self.create_timer(0.05, self.timer_cb)
        self.get_logger().info('Offboard controller ready.')


        #------------------------------------------------#
        # Callbacks                                       #
        #------------------------------------------------#

    def status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        
    def position_cb(self, msg: VehicleLocalPosition):
        self.pos = (msg.x, msg.y, msg.z)


    #------------------------------------------------#
    # Main 20 Hz loop                                 #
    #------------------------------------------------#

    def timer_cb(self):
        self.tick += 1

        # Always publish ocm(offboard control mode) to keep px4 from bailing out of ocm
        self._publish_ocm()

        # Phase 1:
        # First 10 ticks (0.5 s) pre-publishing setpoints so PX4 sees a stream before we request the mode switch
        if self.tick < 10:
            self._publish_setpoint(0.0, 0.0, 0.0)
            return

        # Phase 2: arm + switch to offboard once on tick 10 
        # It should switch to offboard first, then arm
        if self.tick == 10:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # offboard

        if not self.offboard:
            if self.nav_state == 14:
                self._arm()
                self.offboard = True
            return

        # Phase 3: climb to hover altitude
        if not self.reached_hover:
            self._publish_setpoint(0.0, 0.0, self.hover_z)
            if self._distance_to(0.0, 0.0, self.hover_z) < self.REACH_THRESHOLD:
                self.reached_hover = True
                self.get_logger().info(
                    f'Hover reached: {self.pos} - flying to waypoint')
            return

        # Phase 4: fly to way point
        if not self.reached_waypoint:
            self._publish_setpoint(*self.waypoint)
            if self._distance_to(*self.waypoint) < self.REACH_THRESHOLD:
                self.reached_waypoint = True
                self.get_logger().info('Waypoint reached - holding position')
            return
        
        # Phase 5: hold then land
        if not self.sent_landing_command:
            self._publish_setpoint(*self.waypoint)
            self.hold_ticks += 1
            if self.hold_ticks >=60:
                self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.sent_landing_command = True
        
    
        # Phase 6: detect landing and log
        if self.sent_landing_command and not self.landing_complete:
            if self.arming_state == 1: # disarmed: 1, armed: 2
                self.get_logger().info('Landing detected - drone disarmed.')
                self.landing_complete = True

    
    #------------------------------------------------#
    # Helpers                                         #
    #------------------------------------------------#

    def _publish_ocm(self):
        msg = OffboardControlMode()
        msg.timestamp = self._now()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.ocm_pub.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = self._now()
        msg.position = [x, y, z]
        msg.yaw = yaw # radians, 0 = North
        # Leave velocity/acceleration as NaN (PX4 ignores them is position mode)
        self.sp_pub.publish(msg)

    def _arm(self):
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent.')

    def _disarm(self):
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent.')

    def _send_vehicle_command(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._now()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def _distance_to(self, tx: float, ty: float, tz: float) -> float:
        dx = self.pos[0] - tx
        dy = self.pos[1] - ty
        dz = self.pos[2] - tz
        return (dx**2 + dy**2 + dz**2) ** 0.5
    
    def _now(self) -> int:
        return self.get_clock().now().nanoseconds // 1000 # PX4 uses nanosecond
    

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
