import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# Known tag positions in world NED (x=North/forward, y=East/right, z=Down).
TAG_NED = {
    0: (0.0, 0.0, -0.02),
    1: (1.5, 0.0, -0.02),
    2: (3.0, 0.0, -0.02),
}

# Optical->NED yaw (radians). Tune THIS ONE NUMBER until tag0/1/2 all print the
# same drone_NED (and it matches the gz ground-truth pose). Fixed-yaw hover
# assumption for now; we add live attitude later.
THETA = 1.34


class AprilTagOdom(Node):
    """For each visible tag: rotate the camera->tag vector into NED and subtract
    from the tag's known NED pose -> drone NED position. All tags should agree."""

    def __init__(self):
        super().__init__('apriltag_odom')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz

        # Create a publisher to px4
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.odom_pub = self.create_publisher(
            VehicleOdometry, 'fmu/in/vehicle_visual_odometry',
            px4_qos
        )

    def tick(self):
        c, s = math.cos(THETA), math.sin(THETA)
        acc = {'n': [], 'e': [], 'd': []}
        for tag_id, (tN, tE, tD) in TAG_NED.items():
            frame = f'tag{tag_id}'
            try:
                tf = self.tf_buffer.lookup_transform(
                    'camera_link', frame, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().warn(f'{frame}: {ex}')
                continue

            t = tf.transform.translation
            # rotate camera->tag optical (x,y) into NED (N,E); z is straight down
            vN = c * t.x - s * t.y
            vE = s * t.x + c * t.y
            drone_n = tN - vN
            drone_e = tE - vE
            drone_d = tD - t.z
            acc['n'].append(drone_n)
            acc['e'].append(drone_e)
            acc['d'].append(drone_d)

        if not acc['n']:
            return
        else:
            n = sum(acc['n']) / len(acc['n'])
            e = sum(acc['e']) / len(acc['e'])
            d = sum(acc['d']) / len(acc['d'])

        # publish only pos, nan on quartenion & velocity (pos_var = 0.04, confidence about +-0.2m)
        msg = VehicleOdometry()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp_sample = msg.timestamp
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = (n, e, d)
        msg.q = [float('nan')] * 4
        msg.velocity = [float('nan')] * 3
        msg.angular_velocity = [float('nan')] * 3
        msg.position_variance = [0.04, 0.04, 0.04]
        msg.velocity_variance = [float('nan')] * 3
        self.odom_pub.publish(msg)
        self.get_logger().info(f'pub NED = ({n:+.2f}, {e:+.2f}, {d:+.2f})')

def main():
    rclpy.init()
    node = AprilTagOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
