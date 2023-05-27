#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from math import atan

class AckermannToVesc(Node):

   def __init__(self):

      super().__init__('ackermann_to_vesc')

      # load parameters
      self.load_params()

      # init
      self.init_mapping_function()

      # define messages
      self.erpm_msg = Float64()
      self.servo_msg = Float64()

      # subscribe to pwm signals from rc receiver
      self.rc_subscription = self.create_subscription(AckermannDriveStamped, '/ackermann_cmd', self.callback, 10)

      # publish commands to vesc driver
      self.erpm_publisher = self.create_publisher(Float64, '/commands/motor/speed', 10) 
      self.servo_publisher = self.create_publisher(Float64, '/commands/servo/position', 10) 

   def callback(self, ackermann_msg):
      steer_rad = ackermann_msg.drive.steering_angle
      speed = ackermann_msg.drive.speed

      erpm = self.speed_to_erpm_gain * speed + self.speed_to_erpm_offset

      if abs(steer_rad) > self.rad_max:
         self.get_logger().warn("Clipping steering command to +/- %.3f" %self.rad_max)
         steer_rad = max(min(steer_rad, self.rad_max), -self.rad_max) # clip [+/- steer_rad]

      if steer_rad > 0:
         val = self.rr_m * steer_rad + self.servo_mid
      else:
         val = self.lr_m * steer_rad + self.servo_mid
         
      self.servo_msg.data = max(min(val, self.servo_max), self.servo_min) # clip [0.0, 1.0]
      self.erpm_msg.data = erpm

      self.servo_publisher.publish(self.servo_msg)
      self.erpm_publisher.publish(self.erpm_msg)

   def load_params(self):
      self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
      self.servo_mid = self.get_parameter('servo_mid').get_parameter_value().double_value
      self.servo_max = self.get_parameter('servo_max').get_parameter_value().double_value
      self.servo_min = self.get_parameter('servo_min').get_parameter_value().double_value
      self.lr_rmin = self.get_parameter('lr_rmin').get_parameter_value().double_value
      self.rr_rmin = self.get_parameter('rr_rmin').get_parameter_value().double_value
      self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').get_parameter_value().double_value
      self.speed_to_erpm_offset = self.get_parameter('speed_to_erpm_offset').get_parameter_value().double_value

   def init_mapping_function(self):
      self.lr_rad_max = atan(self.wheelbase / self.lr_rmin)
      self.rr_rad_max = atan(self.wheelbase / self.rr_rmin)
      self.rr_dy = self.servo_max - self.servo_mid
      self.lr_dy = self.servo_mid - self.servo_min
      self.rr_m = self.rr_dy / self.rr_rad_max
      self.lr_m = self.lr_dy / self.lr_rad_max
      self.rad_max = max(abs(self.rr_rad_max), abs(self.lr_rad_max))

      self.get_logger().info("The maximum steering angle is set to +/- %.3f rad" % self.rad_max)


def main(args=None):
    rclpy.init(args=args)

    node = AckermannToVesc()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down ackermann_to_vesc")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
