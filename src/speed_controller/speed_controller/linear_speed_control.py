#!/usr/bin/env python3
# ROS dependancies
import rclpy
from rclpy.node import Node
# import functionalities from rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from ackermann_msgs.msg import AckermannDriveStamped
import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
from geometry_msgs.msg import  Twist
from team_interfaces.msg import Car, Lane, Signs

class LinearSpeedControl(Node):
    def __init__(self):
        super().__init__('linear_speed_control')
        # define quality of service
        self.pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        # Publishers
        self.lane_publisher = self.create_publisher(Lane, "lane_topic", self.pose_qos)
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel_topic", self.pose_qos)
        # Subscribers
        self.lane_sub = self.create_subscription(Lane, "lane_topic", self.lane_callback, self.pose_qos) 
        # Timer
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        lane_obj = Lane()
        lane_obj.right_lane_detected = True
        self.lane_publisher.publish(lane_obj)
        
    def lane_callback(self, msg):
        self.get_logger().info(f'Received status {msg.right_lane_detected}')


def main(args=None):
    rclpy.init(args=args)
    linear_speed_control = LinearSpeedControl()
    rclpy.spin(linear_speed_control)
    linear_speed_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()