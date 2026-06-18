"""
Reads a RadioMaster (EdgeTX USB joystick mode) gamepad and publishes
/payload_position_ref (geometry_msgs/PointStamped) in NED.

Operator moves the sticks -> payload position integrates -> drones follow.
This node knows NOTHING about drones; crane_control.py turns the payload
reference into per-drone targets.

Setup:
  1. Plug RadioMaster in via USB, set USB mode = Joystick (EdgeTX).
  2. Run the joy driver:   ros2 run joy joy_node
  3. Run this node:        python3 joystick_ref_publisher.py
  4. Verify axes first:    ros2 topic echo /joy

Stop with Ctrl+C.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped

# Starting payload position (NED).
INIT_X = 0.0
INIT_Y = 1.0
INIT_Z = 0.0

# Stick 
MAX_SPEED = 0.5    
DEADZONE  = 0.1     

# Axis mapping (verify against `ros2 topic echo /joy` for your RadioMaster)
AXIS_X = 1          # right stick up/down  -> payload X (forward/back)
AXIS_Y = 0          # right stick left/right -> payload Y (sideways)
AXIS_Z = 3          # left stick up/down   -> payload Z (up/down)

# Sign per axis
# NED Z is positive DOWN
# Flip any of these to +1/-1 once stick directions confirmed
SIGN_X = 1.0
SIGN_Y = 1.0
SIGN_Z = -1.0


class JoystickRefPublisher(Node):
    def __init__(self):
        super().__init__('joystick_ref_publisher')

        self.pub = self.create_publisher(PointStamped, '/payload_position_ref', 10)
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # Accumulated payload position (remembered between ticks)
        self.x = INIT_X
        self.y = INIT_Y
        self.z = INIT_Z

        # Latest stick values, stored by joy_cb, consumed by timer_cb
        self.axis_x = 0.0
        self.axis_y = 0.0
        self.axis_z = 0.0

        self.dt = 0.05
        self.create_timer(self.dt, self.timer_cb)  # 20 Hz
        self.get_logger().info('Joystick ref publisher started — move sticks to drive payload.')

    def joy_cb(self, msg: Joy):
        # Just store the latest axis values; no maths here.
        self.axis_x = SIGN_X * self._deadzone(msg.axes[AXIS_X])
        self.axis_y = SIGN_Y * self._deadzone(msg.axes[AXIS_Y])
        self.axis_z = SIGN_Z * self._deadzone(msg.axes[AXIS_Z])

    def timer_cb(self):
        # Integrate stick velocity into position at a fixed rate.
        self.x += self.axis_x * MAX_SPEED * self.dt
        self.y += self.axis_y * MAX_SPEED * self.dt
        self.z += self.axis_z * MAX_SPEED * self.dt

        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = self.z
        self.pub.publish(msg)

    def _deadzone(self, value: float) -> float:
        return 0.0 if abs(value) < DEADZONE else value


def main():
    rclpy.init()
    node = JoystickRefPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
