"""
Simulates the 3-DOF arm operator input for testing crane_control.py.
Publishes /payload_position_ref (geometry_msgs/PointStamped) in NED.

Sequence:
  0 – 10 s  : hold at initial position (payload center between drones)
  10 s+     : slow sine sweep on X axis (simulates operator moving arm forward/back)

Run after both crane_control.py instances have reached hover:
  python3 test_ref_publisher.py
"""

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

# Drones spawn at NED (0,0) and (0,2) — formation center is Y=1
INIT_X = 0.0
INIT_Y = 1.0
INIT_Z = 0.0   # NED Z=0 → payload at ground level, drones fly CABLE_LENGTH above

SWEEP_AMPLITUDE = 1.5   # metres on X axis
SWEEP_PERIOD    = 20.0  # seconds for one full back-and-forth


class TestRefPublisher(Node):
    def __init__(self):
        super().__init__('test_ref_publisher')
        self.pub = self.create_publisher(PointStamped, '/payload_position_ref', 10)
        self.t = 0.0
        self.create_timer(0.05, self.timer_cb)  # 20 Hz
        self.get_logger().info('Test ref publisher started — holding initial position for 10 s.')

    def timer_cb(self):
        self.t += 0.05

        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        if self.t < 10.0:
            # Hold at formation center so drones settle before any motion
            msg.point.x = INIT_X
            msg.point.y = INIT_Y
            msg.point.z = INIT_Z
        else:
            # Sweep X: oscillates between INIT_X and INIT_X + 2*SWEEP_AMPLITUDE
            elapsed = self.t - 10.0
            msg.point.x = INIT_X + SWEEP_AMPLITUDE * (1 - math.cos(2 * math.pi * elapsed / SWEEP_PERIOD))
            msg.point.y = INIT_Y
            msg.point.z = INIT_Z

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TestRefPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
