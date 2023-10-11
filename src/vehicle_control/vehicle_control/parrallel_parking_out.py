#!/usr/bin/env python3

from platform import node
import rclpy
import signal
import time  # Import the time module
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import UInt8, Int16MultiArray
from team_interfaces.msg import Emergency
from team_interfaces.msg import Trafficlight
import sys

MAX_STEERING_ANGLE = 0.442  # [rad]
MIN_THRUST = 0.6
t_0 = 0.5
t_1 = 1.5
t_2 = 0.4

sign = False
# 0: Forward
# 1: Forward right 


class ParrallelParkingNode(Node):
    def __init__(self):
        super().__init__('parallel_parking_node')

        #Publishers and subscribers
        self.ackermann_pub = self.create_publisher(AckermannDrive, '/ackermann_cmd', 2)
        self.traffic_light_sub = self.create_subscription(Trafficlight, 'traffic_light', self.traffic_light_callback, 2)


        self.timer_30hz = self.create_timer(1.0 / 30.0, self.timer_callback_30hz)
        #Variables
        self.state_wan = 0
        self.state_too = 0
        self.red = True
        self.park_slot = sys.argv[1] # 0 closest 2 furthest

    def traffic_light_callback(self, msg):
        self.red = msg.traffic_light 

    def timer_callback_30hz(self):
        
        
        self.msg = AckermannDrive()

        # Thrust
        # if self.state == (0):
        # msg.steering_angle = 0.0
        # msg.speed = MIN_THRUST
        # self.ackermann_pub.publish(msg)
        # time.sleep(0.15)
        if self.state_wan == 0:
            self.flexible_ackermann(MIN_THRUST, 0.0, 0.15)
            self.flexible_ackermann(MIN_THRUST, MAX_STEERING_ANGLE, 2)
            if self.park_slot == "1":
                self.flexible_ackermann(MIN_THRUST, MAX_STEERING_ANGLE, 0.8)
                # self.flexible_ackermann(MIN_THRUST, 0.0, 0.1)
                self.flexible_ackermann(MIN_THRUST, -MAX_STEERING_ANGLE, 0.8)
            elif self.park_slot == "2":
                self.flexible_ackermann(MIN_THRUST, MAX_STEERING_ANGLE, 0.8)
                self.flexible_ackermann(MIN_THRUST, 0.0, 0.3)
                self.flexible_ackermann(MIN_THRUST, -MAX_STEERING_ANGLE, 0.8)
            self.flexible_ackermann(0.0, 0.0, 0.1)
        if self.red == False and self.state_wan != 0 and self.state_too == 0:
            self.flexible_ackermann(MIN_THRUST, -MAX_STEERING_ANGLE, 1)
            self.flexible_ackermann(MIN_THRUST, 0.0, 0.8)
            self.flexible_ackermann(MIN_THRUST, MAX_STEERING_ANGLE, 1.8)
            self.state_too = 1

        # self.flexible_ackermann(MIN_THRUST, 0.0, 0.15)
        # self.flexible_ackermann(MIN_THRUST, 0.0, 0.15)
        self.state_wan = 1
            

        # Thrust and right
        # if self.state == 1:
        #     msg.steering_angle = MAX_STEERING_ANGLE
        #     msg.speed = MIN_THRUST
        #     self.ackermann_pub.publish(msg)
        #     time.sleep(1)
        #     msg.steering_angle = MAX_STEERING_ANGLE
        #     msg.speed = MIN_THRUST
        #     self.ackermann_pub.publish(msg)
        #     time.sleep(1)            
        #     self.state += 1

        # # Thrust and half left
        # if self.state == 2:
        #     msg.steering_angle = -MAX_STEERING_ANGLE / 2
        #     msg.speed = MIN_THRUST
        #     self.ackermann_pub.publish(msg)
        #     time.sleep(t_2)
        #     self.state += 1


        self.send_ackermann_halt()
        time.sleep(1)

    def flexible_ackermann(self, speed, angle, t):
        secs = math.floor(t)
        msecs = float(t - secs)
        self.msg.speed = speed
        self.msg.steering_angle = angle

        for i in range(0,secs):
            self.ackermann_pub.publish(self.msg)
            time.sleep(1.0)
        self.ackermann_pub.publish(self.msg)
        time.sleep(msecs)
    def send_ackermann_halt(self):
        ack_msg = AckermannDrive()
        ack_msg.steering_angle = 0.0
        ack_msg.speed = 0.0
        self.ackermann_pub.publish(ack_msg)

        


def main(args=None):
    rclpy.init(args=args)
    node = ParrallelParkingNode()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
