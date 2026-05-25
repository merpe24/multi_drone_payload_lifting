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
            '/fmu/out/vehicle_local_position_v1',
            self.callback,
            qos
        )

    def callback(self, msg):
        self.get_logger().info(f'x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f}')

def main():
    rclpy.init()
    rclpy.spin(DroneListener())

main()