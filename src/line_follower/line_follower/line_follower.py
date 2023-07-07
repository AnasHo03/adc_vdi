#!/usr/bin/env python3

from platform import node
import rclpy
import signal
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import UInt8, UInt16MultiArray
from sensor_msgs.msg import Image as ROS_Image

# Python dependancies
import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
from cv_bridge import CvBridge # For converting ROS image message to jpg

# Parameters
MAX_STEERING_ANGLE = 0.442  # [rad]
CONSTANT_THRUST = float(0.2)  # [0 to 2.5]
KP = 0.005  # Proportional gain constant


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.counter = 1
        # Logic variables
        self.emergency_stop = False
        self.destroyed = False
        self.center_offset = 0.0

        # Define messages
        self.ack_msg = AckermannDrive()

        # Initialize subscribers
        self.joy_sub = self.create_subscription(UInt16MultiArray, '/joy', self.emergency_shutdown_callback, 10)
        self.camera_sub = self.create_subscription(ROS_Image, '/zed/zed_node/left_raw/image_raw_color', self.cam_callback, 10)

        # Initialize publisher
        self.ackermann_pub = self.create_publisher(AckermannDrive, '/ackermann_cmd', 10)

        # Register shutdown callback (function triggered at ctr+c) 
        signal.signal(signal.SIGINT, self.shutdown_callback)

        # Initialize CvBridge
        self.bridge = CvBridge()

    def shutdown_callback(self, signum, frame):
          if self.destroyed:
            return

          self.get_logger().info('Node terminated!')

          # Send ackermann_halt for 1 second
          t0 = self.get_clock().now().to_msg().sec
          t_close = 1
          while (self.get_clock().now().to_msg().sec - t0) < t_close:
            self.send_ackermann_halt()

          self.node.destroy_node()    ## TODO: fix this
          self.destroyed = True
          rclpy.shutdown()

    def emergency_shutdown_callback(self, joy_msg):
        if joy_msg.buttons == 2:
            self.emergency_stop = True
            self.get_logger().info('Killswitch activated!')

    def cam_callback(self, col_img_raw):
        # Return if emergncy stop activated
        if self.emergency_stop == True:
            return

        # Convert the ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(col_img_raw, desired_encoding='bgr8')

        # Process the image
        self.process_image(cv_image)

    def process_image(self, cv_image):        
        # Calculate center offset
        center_offset = lane_obj.calculate_car_position(print_to_terminal=True)

        # Calculate steering angle (P-controller)
        steering_angle = self.calculate_steering_angle(center_offset, MAX_STEERING_ANGLE)

        # Publish Ackermann message
        self.send_ackermann(steering_angle)

    def calculate_steering_angle(self, center_offset, MAX_STEERING_ANGLE):
        error = 0 - center_offset  # Desired offset is zero
        control_signal = KP * error
        limited_control_signal = np.clip(control_signal, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)
        return limited_control_signal

    def send_ackermann(self, steering_angle):
        ack_msg = AckermannDrive()
        ack_msg.steering_angle = steering_angle
        ack_msg.steering_angle_velocity = 0.0
        ack_msg.speed = CONSTANT_THRUST
        ack_msg.acceleration = 0.0
        ack_msg.jerk = 0.0
        self.ackermann_pub.publish(ack_msg)

    def send_ackermann_halt(self):
        ack_msg = AckermannDrive()
        ack_msg.steering_angle = 0.0
        ack_msg.speed = 0.0
        self.ackermann_pub.publish(ack_msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()

    rclpy.spin(node)

if __name__ == '__main__':
    main()