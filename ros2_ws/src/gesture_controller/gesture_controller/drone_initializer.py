import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time


class DroneInitializer(Node):
    def __init__(self):
        super().__init__('drone_initializer')
        self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 10)
        self.initialize_drone()
        
    def initialize_drone(self):
        self.get_logger().info("Initializing drone...")
        time.sleep(1)
        self.takeoff_pub.publish(Empty())
        self.get_logger().info("Sent: takeoff")
		 
		 
def main(args=None):
    rclpy.init(args=args)
    node = DroneInitializer() 
    rclpy.shutdown()
