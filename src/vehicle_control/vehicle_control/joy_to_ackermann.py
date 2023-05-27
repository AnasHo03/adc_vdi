#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

class JoyControl(Node):

    def __init__(self):
        super().__init__('joy_control')

        # select control type
        self.control_types = ['joy', 'rc']  # [joystick, remote control]
        self.control_type = self.declare_parameter("control_type", 'rc')
        self.control_type =  self.get_parameter('control_type').get_parameter_value().string_value

        # check parameter
        if not self.control_type in self.control_types:
            self.get_logger().error('Invalid control type parameter. Choose between %s, and %s.' 
            % tuple(self.control_types))
        else:
            self.get_logger().info("control_type: %s", self.control_type)

        # load parameters
        self.load_params()

        # define messages
        self.ackMsg = AckermannDriveStamped()

        # publish ackermann messages to VESC
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 1)

        # subscribe to joy
        self.joy_sub = self.create_subscription(Joy, '/joy', self.callback, 10)


    def callback(self, msg):

        # deadman button
        if msg.buttons[self.dead_btn] == 0:
            steer_rad = self.servo_mid
            speed_mps = self.speed_mid
        else:
            # steering
            steer_val = msg.axes[self.steer_ax] * self.steer_scale
            if steer_val > self.steer_deadzone:
                steer_rad = steer_val * self.servo_max
            elif steer_val < -self.steer_deadzone:
                steer_rad = steer_val * self.servo_min * (-1)
            else:
                steer_rad = self.servo_mid

            # throttle
            speed_val = msg.axes[self.speed_ax]
            if speed_val > self.speed_deadzone:
                speed_mps = speed_val * self.speed_max
            elif speed_val < -self.speed_deadzone:
                speed_mps = speed_val * self.speed_min * (-1)
            else:
                speed_mps = self.speed_mid

        self.ackMsg.header.stamp = self.get_clock().now().to_msg()
        self.ackMsg.drive.steering_angle = steer_rad
        self.ackMsg.drive.speed = speed_mps

        self.ackermann_pub.publish(self.ackMsg)

    def load_params(self):

        # Declare parameters with default values
        self.declare_parameter('rc_steering_axis', 0)
        self.declare_parameter('rc_speed_axis', 1)
        self.declare_parameter('rc_deadman_button', 0)
        self.declare_parameter('joy_steering_axis', 3)
        self.declare_parameter('joy_speed_axis', 1)
        self.declare_parameter('joy_deadman_button', 4)
        self.declare_parameter('servo_rad_min', -0.442)
        self.declare_parameter('servo_rad_mid', 0)
        self.declare_parameter('servo_rad_max', 0.442)
        self.declare_parameter('speed_min', -2.0)
        self.declare_parameter('speed_mid', 0)
        self.declare_parameter('speed_max', 2.5)
        self.declare_parameter('speed_deadzone', 0.001)
        self.declare_parameter('steer_deadzone', 0.001)

        # Load parameters
        if self.control_type == 'rc':
            self.steer_ax = self.get_parameter('rc_steering_axis').get_parameter_value().integer_value
            self.speed_ax = self.get_parameter('rc_speed_axis').get_parameter_value().integer_value
            self.dead_btn = self.get_parameter('rc_deadman_button').get_parameter_value().integer_value
            self.steer_scale = 1
        elif self.control_type == 'joy':
            self.steer_ax = self.get_parameter('joy_steering_axis').get_parameter_value().integer_value
            self.speed_ax = self.get_parameter('joy_speed_axis').get_parameter_value().integer_value
            self.dead_btn = self.get_parameter('joy_deadman_button').get_parameter_value().integer_value
            self.steer_scale = -1  # switch directions

        self.servo_min = self.get_parameter('servo_rad_min').get_parameter_value().double_value
        self.servo_mid = self.get_parameter('servo_rad_mid').get_parameter_value().double_value
        self.servo_max = self.get_parameter('servo_rad_max').get_parameter_value().double_value
        self.speed_min = self.get_parameter('speed_min').get_parameter_value().double_value
        self.speed_mid = self.get_parameter('speed_mid').get_parameter_value().double_value
        self.speed_max = self.get_parameter('speed_max').get_parameter_value().double_value
        self.speed_deadzone = self.get_parameter('speed_deadzone').get_parameter_value().double_value
        self.steer_deadzone = self.get_parameter('steer_deadzone').get_parameter_value().double_value



def main(args=None):
    rclpy.init(args=args)

    joy_control = JoyControl()

    rclpy.spin(joy_control)

    # Destroy the node explicitly
    joy_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


