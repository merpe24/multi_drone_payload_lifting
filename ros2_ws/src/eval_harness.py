import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

# Frame: "map" is ENU with origin at the drone spawn (= gz world origin,
# = PX4 NED local origin). Both sources are expressed there so they overlay.
MAP_FRAME = 'map'
GT_TOPIC = '/model/x500_flow_0/odometry'   # gz ground-truth odom (ENU)


class EvalHarness(Node):
    """
    Overlay EKF2 estimate vs Gazebo ground truth to MEASURE optical-flow drift.

    EKF2 estimate : px4 vehicle_local_position (NED) -> ENU (y, x, -z)
    Ground truth  : gz /world/flow_world/pose/info (Pose_V -> TFMessage, ENU)
    Output        : two nav_msgs/Path trails + TF + 1 Hz error log.
    """

    def __init__(self):
        super().__init__('eval_harness')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # --- estimate (PX4 NED) ---
        self.create_subscription(
            VehicleLocalPosition, 'fmu/out/vehicle_local_position_v1',
            self.ekf2_cb, px4_qos)
        # --- ground truth (bridged gz odometry, ENU) ---
        self.create_subscription(
            Odometry, GT_TOPIC, self.truth_cb, 10)

        self.ekf2_path_pub = self.create_publisher(Path, 'eval/ekf2_path', 10)
        self.truth_path_pub = self.create_publisher(Path, 'eval/truth_path', 10)
        self.tf_bc = TransformBroadcaster(self)

        self.ekf2_path = Path()
        self.ekf2_path.header.frame_id = MAP_FRAME
        self.truth_path = Path()
        self.truth_path.header.frame_id = MAP_FRAME

        self.ekf2_enu = None
        self.truth_enu = None
        # Each source uses a different reference point (EKF2: base_link at boot;
        # gz truth: base_footprint at ground), giving a constant frame offset.
        # Zero each to its own first sample so what remains is true drift.
        self.ekf2_origin = None
        self.truth_origin = None
        self.MAX_POSES = 4000

        self.create_timer(1.0, self.report_cb)   # 1 Hz error log

    # ------------------------------------------------------------------ #
    def ekf2_cb(self, msg: VehicleLocalPosition):
        if not (msg.xy_valid and msg.z_valid):
            return
        # NED -> ENU
        e, n, u = msg.y, msg.x, -msg.z
        if self.ekf2_origin is None:
            self.ekf2_origin = (e, n, u)
        e -= self.ekf2_origin[0]
        n -= self.ekf2_origin[1]
        u -= self.ekf2_origin[2]
        self.ekf2_enu = (e, n, u)
        self._append(self.ekf2_path, self.ekf2_path_pub, e, n, u)
        self._send_tf('ekf2_est', e, n, u)

    def truth_cb(self, msg: Odometry):
        p = msg.pose.pose.position          # gz odometry already ENU
        x, y, z = p.x, p.y, p.z
        if self.truth_origin is None:
            self.truth_origin = (x, y, z)
        x -= self.truth_origin[0]
        y -= self.truth_origin[1]
        z -= self.truth_origin[2]
        self.truth_enu = (x, y, z)
        self._append(self.truth_path, self.truth_path_pub, x, y, z)
        self._send_tf('truth', x, y, z)

    # ------------------------------------------------------------------ #
    def _append(self, path: Path, pub, x, y, z):
        ps = PoseStamped()
        ps.header.frame_id = MAP_FRAME
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
        if len(path.poses) > self.MAX_POSES:
            path.poses.pop(0)
        path.header.stamp = ps.header.stamp
        pub.publish(path)

    def _send_tf(self, child, x, y, z):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = MAP_FRAME
        tf.child_frame_id = child
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = float(z)
        tf.transform.rotation.w = 1.0
        self.tf_bc.sendTransform(tf)

    def report_cb(self):
        if self.ekf2_enu is None or self.truth_enu is None:
            self.get_logger().info('waiting for both estimate and ground truth...')
            return
        ex = self.ekf2_enu[0] - self.truth_enu[0]
        ey = self.ekf2_enu[1] - self.truth_enu[1]
        ez = self.ekf2_enu[2] - self.truth_enu[2]
        horiz = (ex * ex + ey * ey) ** 0.5
        total = (ex * ex + ey * ey + ez * ez) ** 0.5
        self.get_logger().info(
            f'drift  horiz={horiz:.3f} m  total={total:.3f} m  '
            f'(dE={ex:+.3f} dN={ey:+.3f} dU={ez:+.3f})')


def main():
    rclpy.init()
    node = EvalHarness()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
