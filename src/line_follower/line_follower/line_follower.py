#!/usr/bin/env python3

from platform import node
import rclpy
import signal
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import UInt8, UInt16MultiArray
from team_interfaces.msg import Lane

# Python dependancies
import numpy as np # Import the NumPy scientific computing library

# Parameters driving
MAX_STEERING_ANGLE = 0.442  # [rad]
CONSTANT_THRUST = float(0.2)  # [0 to 2.5]
KP = 0.005  # Proportional gain constant

# Parameters filtering
NUM_ELEMENTS_TO_AVERAGE = 5

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # Variables
        self.offset_array = []

        # Logic variables
        self.emergency_stop = False
        self.destroyed = False
        self.center_offset = 0.0

        # Define messages
        self.ack_msg = AckermannDrive()
        # Initialize subscribers
        self.joy_sub = self.create_subscription(UInt16MultiArray, '/joy', self.emergency_shutdown_callback, 10)
        self.lane_sub = self.create_subscription(Lane, 'lane_topic', self.lane_callback, 10)
        # Initialize publiher
        self.ackermann_pub = self.create_publisher(AckermannDrive, '/ackermann_cmd', 10)
        # Register shutdown callback (function triggered at ctr+c) 
        signal.signal(signal.SIGINT, self.shutdown_callback)

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

    def filter_signal(self, steering_angle):
        self.offset_array.append(steering_angle)
        if len(self.offset_array) == NUM_ELEMENTS_TO_AVERAGE:
            average = sum(self.offset_array) / len(self.offset_array)
            self.steering_angle = [] 
            return average


    def lane_callback(self, msg):
        steering_angle = self.calculate_steering_angle(msg.center_offset)
        self.send_ackermann(steering_angle)

    def calculate_steering_angle(self, center_offset):
        error = 0 - center_offset  # Desired offset is zero
        control_signal = KP * error
        limited_control_signal = np.clip(control_signal, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)
        return limited_control_signal

    def send_ackermann(self, steering_angle):
        ack_msg = AckermannDrive()
        ack_msg.steering_angle = steering_angle
        ack_msg.steering_angle_velocity = 0.0
        ack_msg.speed = 0.0       #CONSTANT_THRUST
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