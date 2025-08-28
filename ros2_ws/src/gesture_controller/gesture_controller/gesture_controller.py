import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math
import time


class GestureController(Node):
    def __init__(self):
        super().__init__('gesture_controller')
        self.subscription = self.create_subscription(
            Float32MultiArray, 
            'pose_keypoints', 
            self.listener_callback, 
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.get_logger().info("Gesture controller ready.")
    
    def listener_callback(self, msg):
        keypoints = msg.data
        twist = Twist()

        if len(keypoints) < 33 * 2:
            twist.linear.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Command: hover (no pose detected)")
            return
        
        left_hand = (keypoints[15 * 2], keypoints[15 * 2 + 1])
        right_hand = (keypoints[16 * 2], keypoints[16 * 2 + 1])
        left_shoulder = (keypoints[11 * 2], keypoints[11 * 2 + 1])
        right_shoulder = (keypoints[12 * 2], keypoints[12 * 2 + 1])
        
        def euclidean_dist(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
        
        distance_th = 0.1
        angle_th = 0.2
        horizontal_th = 0.1
        vertical_th = 0.1
        
        right_touch = euclidean_dist(right_hand, right_shoulder) < distance_th
        left_touch = euclidean_dist(left_hand, left_shoulder) < distance_th
        
        right_horizontal = (abs(right_shoulder[1] - right_hand[1]) < angle_th and 
                           (right_hand[0] > right_shoulder[0] - horizontal_th))
        left_horizontal = (abs(left_shoulder[1] - left_hand[1]) < angle_th and 
                          (left_hand[0] > left_shoulder[0] - horizontal_th))
        
        right_up = right_hand[1] < right_shoulder[1] - vertical_th
        left_up = left_hand[1] < left_shoulder[1] - vertical_th
        
        if right_touch and not left_touch:
            twist.linear.x = 1.0
            cmd = 'forward'
        elif not right_touch and left_touch:
            twist.linear.x = -1.0
            cmd = 'backward'
        elif right_up and not left_up:
            twist.linear.z = 1.0
            cmd = 'ascend'
        elif not right_up and left_up:
            twist.linear.z = -1.0
            cmd = 'descend'
        elif right_horizontal:
            twist.angular.z = -1.0
            cmd = 'rotate_right'
        elif left_horizontal:
            twist.angular.z = 1.0
            cmd = 'rotate_left'
        else:
            twist.linear.z = 0.0
            cmd = 'hover'
        
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Command: {cmd}")
		

def main(args=None):
    rclpy.init(args=args)
    node = GestureController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

