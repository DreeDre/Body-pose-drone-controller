import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import mediapipe as mp
import atexit

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, 'pose_keypoints', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Webcam not available!")
            exit()

        self.mp_pose = mp.solutions.pose.Pose()
        self.mp_drawing = mp.solutions.drawing_utils
        
        atexit.register(self.cleanup)


    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().error("Webcam not available!")
            return
		
        ret, frame = self.cap.read()
        if not ret:
            return

        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.mp_pose.process(image_rgb)
        black_frame = np.zeros_like(frame)

        keypoints = []
        if results.pose_landmarks:
            for landmark in results.pose_landmarks.landmark:
                keypoints.extend([landmark.x, landmark.y])

            self.mp_drawing.draw_landmarks(
                black_frame, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS
            )

        cv2.imshow("Pose Estimation", black_frame)
        cv2.waitKey(1)

        msg = Float32MultiArray()
        msg.data = keypoints
        self.publisher.publish(msg)


    def cleanup(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Webcam released!")


    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
