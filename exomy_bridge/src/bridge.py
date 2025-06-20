import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class BridgeNode(Node):
    def __init__(self):
        super().__init__('exomy_bridge_node')
        self.get_logger().info('ExoMy Bridge Node has been started.')

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan_raw',
            self.scan_callback,
            10
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            'scan',
            10
        )



    def scan_callback(self, msg):
        message = msg
        message.header.
        self.get_logger().info('Received LaserScan message')