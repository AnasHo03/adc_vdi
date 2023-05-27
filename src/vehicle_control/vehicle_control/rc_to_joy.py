#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Joy

class RCJoystick(Node):

    def __init__(self):
        super().__init__('rc_joystick')

        # load parameters
        self.load_params()

        # define thresholds to filter out noise
        self.mode_pwm_threshold = 100
        self.throt_pwm_threshold = 12
        self.steer_pwm_threshold = 5

        # define messages
        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = 'rc_control'
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()

        # subscribe to pwm signals from rc receiver
        self.rc_sub = self.create_subscription(UInt16MultiArray, '/veh_remote_ctrl', self.callback, 10) 

        # publish joy message
        self.rc_pub = self.create_publisher(Joy, '/joy', 2)

    def parse_pwm(self, pwm_signal):
        steer_pwm, throt_pwm, mode_pwm = pwm_signal
        steer_val, throt_val, mode_val = 0, 0, 0 # init
    
        # connection lost if pwm = 0
        if 0 in pwm_signal:
           return steer_val, throt_val, mode_val
      
        # steering
        if abs(steer_pwm - self.steer_mid_pwm) < self.steer_pwm_threshold:
           steer_val = 0.0
        elif steer_pwm >= self.steer_mid_pwm:
           steer_val = min(float(steer_pwm - self.steer_mid_pwm) / (self.steer_max_pwm - self.steer_mid_pwm), 1.0) # [0.0, 1.0]
        else:
           steer_val = max(float(steer_pwm - self.steer_mid_pwm) / (self.steer_mid_pwm - self.steer_min_pwm), -1.0) # [-1.0, 0.0]
     
        # throttle
        if abs(throt_pwm - self.throt_mid_pwm) < self.throt_pwm_threshold:
           throt_val = 0.0
        elif throt_pwm >= self.throt_mid_pwm:
           throt_val = min(float(throt_pwm - self.throt_mid_pwm) / (self.throt_max_pwm - self.throt_mid_pwm), 1.0) # [0.0, 1.0]
        else:
           throt_val = max(float(throt_pwm - self.throt_mid_pwm) / (self.throt_mid_pwm - self.throt_min_pwm), -1.0) # [-1.0, 0.0]
      
        # mode
        if abs(self.mode_min_pwm - mode_pwm) < self.mode_pwm_threshold:
           mode_val = 0
        elif abs(self.mode_mid_pwm - mode_pwm) < self.mode_pwm_threshold:
           mode_val =  1
        elif abs(self.mode_max_pwm - mode_pwm) < self.mode_pwm_threshold:
           mode_val =  2
      
        return steer_val, throt_val, mode_val

    def callback(self, data):
        steer_val, throt_val, mode_val = self.parse_pwm(data.data)

        self.joy_msg.axes = [steer_val, throt_val]
        self.joy_msg.buttons = [mode_val]
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.joy_msg.header.seq += 1
    
    def load_params(self):
        # declare parameters
        self.declare_parameter('steering_min_pwm', 1000)
        self.declare_parameter('steering_mid_pwm', 1500)
        self.declare_parameter('steering_max_pwm', 2000)
        self.declare_parameter('throttle_min_pwm', 1000)
        self.declare_parameter('throttle_mid_pwm', 1500)
        self.declare_parameter('throttle_max_pwm', 2000)
        self.declare_parameter('mode_min_pwm', 1000)
        self.declare_parameter('mode_mid_pwm', 1500)
        self.declare_parameter('mode_max_pwm', 2000)

        # get parameter values
        self.steer_min_pwm = self.get_parameter('steering_min_pwm').get_parameter_value().integer_value
        self.steer_mid_pwm = self.get_parameter('steering_mid_pwm').get_parameter_value().integer_value
        self.steer_max_pwm = self.get_parameter('steering_max_pwm').get_parameter_value().integer_value
        self.throt_min_pwm = self.get_parameter('throttle_min_pwm').get_parameter_value().integer_value
        self.throt_mid_pwm = self.get_parameter('throttle_mid_pwm').get_parameter_value().integer_value
        self.throt_max_pwm = self.get_parameter('throttle_max_pwm').get_parameter_value().integer_value
        self.mode_min_pwm = self.get_parameter('mode_min_pwm').get_parameter_value().integer_value
        self.mode_mid_pwm = self.get_parameter('mode_mid_pwm').get_parameter_value().integer_value
        self.mode_max_pwm = self.get_parameter('mode_max_pwm').get_parameter_value().integer_value

def main(args=None):
    rclpy.init(args=args)
    rc_joy = RCJoystick()
    rclpy.spin(rc_joy)
    rc_joy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
