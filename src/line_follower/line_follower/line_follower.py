#!/usr/bin/env python3

from platform import node
import rclpy
import signal
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import UInt8, UInt16MultiArray
from team_interfaces.msg import Lane
from team_interfaces.msg import Emergency


# Python dependancies
import numpy as np # Import the NumPy scientific computing library
import math

# Parameters general
DRIVE_MODE = 0 # 0 = normal lap, 1 = drag racing
DELAY_IN_FRAMES = 70

# Parameters driving
MAX_STEERING_ANGLE = 0.442  # [rad]
MAX_STEERING_ANGLE_DRAG = 0.1
MIN_THRUST = 1
MAX_THRUST = 1.6
CONSTANT_THRUST = float(0.6)  # [m/second] (min. is 0.4 m/s)
CONSTANT_THRUST_DRAG = 4.0
KP_LO = 0.018 # Proportional gain constant
KP_DRAG_RACING = 0.0195   # drag racing KP
KP_THRUST = 1.0
KI = 0.0001    # Integral gain
KI_DRAG_RACING = 0.0004 # drag racing KI
INTEGRAL_CONTROLLER_FRAMES = 8 # frames
KD = 0.0 #0.00050    # Derivative gain
KD_DRAG_RACING = 0.0040 # drag racing KD
SIGMOID_SLOPE = 7.5
SIGMOID_X_OFFSET = 0.9
SIGMOID_YMAX_OFFSET = 0.25

# Parameters filtering
NUM_ELEMENTS_TO_AVERAGE_OFFSET = 1
NUM_ELEMENTS_TO_AVERAGE_HEADING = 10
NUM_ELEMENTS_TO_CONSIDER_HEADING = 5 # must be smaller than NUM_ELEMENTS_TO_AVERAGE_HEADING

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # Variables
        self.offset_array = []
        self.heading_array = []

        # Logic variables
        self.emergency_stop = False
        self.destroyed = False

        # Variables
        self.center_offset = 0.0
        self.heading_angle = 0.0
        self.previous_center_offset = 0.0
        self.previous_heading = 0.0
        self.integral_term = []
        self.integral_term_speed = 0.0
        self.start_ctr = 0

        # Define messages
        self.ack_msg = AckermannDrive()
        
        # Initialize subscribers
        #self.joy_sub = self.create_subscription(UInt16MultiArray, '/joy', self.emergency_shutdown_callback, 10)
        self.lane_sub = self.create_subscription(Lane, 'lane_topic', self.lane_callback, 10)
        self.lane_sub = self.create_subscription(Emergency, 'emergency', self.emergency_shutdown_callback, 10)

        # Initialize publiher
        self.ackermann_pub = self.create_publisher(AckermannDrive, '/ackermann_cmd', 10)
        
        # Register shutdown callback (function triggered at ctr+c) 
        signal.signal(signal.SIGINT, self.shutdown_callback)

    def lane_callback(self, msg):
        # Fetch current offset from message
        self.center_offset = msg.center_offset
        self.heading_angle = msg.heading_angle
        self.right_lane_detected = msg.right_lane_detected
        self.left_lane_detected = msg.left_lane_detected

        # self.get_logger().info('Offset:' + str(self.center_offset))
        # self.get_logger().info('Heading:' + str(self.heading_angle))
        # if self.right_lane_detected and self.left_lane_detected:
        #     self.get_logger().info('I detect both  lanes!')
        # elif self.right_lane_detected:
        #     self.get_logger().info('I detect right lane !')
        # elif self.left_lane_detected:
        #     self.get_logger().info('I detect left  lane !')
        # else:
        #     self.get_logger().info('I detect no    lane !')

        ## Only proceed if emergency stop is not triggered
        if self.emergency_stop == False:
            # Filter signal
            self.center_offset = self.filter_signal_offset(self.center_offset)
            self.heading_angle = self.filter_signal_heading(self.heading_angle)

            # Calculate steering angle with PID 
            steering_angle = self.pid_controller(self.center_offset, self.previous_center_offset)

            # Calculate thrust
            thrust = self.speed_controller(self.heading_angle)

            # Update previous offset and heading if offset is NaN
            if not math.isnan(self.center_offset):
                self.previous_center_offset = self.center_offset
            if not math.isnan(self.heading_angle):
                self.previous_heading = self.heading_angle

            # Publish Ackermann message after delay
            if self.start_ctr > DELAY_IN_FRAMES:
                self.send_ackermann(steering_angle, thrust)
            # Countdownpid
            elif (DELAY_IN_FRAMES - self.start_ctr) % 13 == 0:
                if DRIVE_MODE == 1:
                    self.get_logger().info("DRAG RACE MODE!")

                self.get_logger().info(str((DELAY_IN_FRAMES - self.start_ctr) / 13))
            self.start_ctr += 1

    def pid_controller(self, center_offset, previous_center_offset):
        # Desired offset is zero
        current_error = 0 - center_offset
        previous_error = 0 - previous_center_offset
        

        # Signal (terms combined) and proportional
        if DRIVE_MODE == 0:
            proportional_term = KP_LO * current_error
            derivative_term = KD * (current_error - previous_error)
            signal = proportional_term + derivative_term
            clipped_signal = np.clip(signal, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)


        else:
            proportional_term = KP_DRAG_RACING * current_error
            self.integral_term.append(KI_DRAG_RACING * current_error)
            if len(self.integral_term) > INTEGRAL_CONTROLLER_FRAMES:
                self.integral_term.pop(0)
            if len(self.integral_term) == INTEGRAL_CONTROLLER_FRAMES:
                integral_term = sum(self.integral_term) / INTEGRAL_CONTROLLER_FRAMES
            else:
                integral_term = KI_DRAG_RACING * current_error
            derivative_term = KD_DRAG_RACING * (current_error - previous_error)
            signal = proportional_term + integral_term + derivative_term
            clipped_signal = np.clip(signal, -MAX_STEERING_ANGLE_DRAG, MAX_STEERING_ANGLE_DRAG)

        
        #self.get_logger().info('steering angle:' + str(clipped_signal))
        return clipped_signal
    
    def speed_controller(self, heading_angle):
        # Proportional term
        norm_heading_angle = abs(abs(heading_angle) - 1.3)
        proportional_term = self.sigmoid_controller(norm_heading_angle)
        # proportional_term = KP_THRUST * norm_heading_angle
        # Clip signal
        # clipped_signal = np.clip(proportional_term, MIN_THRUST, MAX_THRUST)
        # return clipped_signal
        return proportional_term

    def filter_signal_offset(self, offset):
        if not math.isnan(offset):  # Check if offset is NaN
            self.offset_array.append(offset)
        else:
            self.offset_array.append(self.previous_center_offset)

        if len(self.offset_array) > NUM_ELEMENTS_TO_AVERAGE_OFFSET:
            self.offset_array.pop(0)  # Remove the oldest element from the array
        if len(self.offset_array) == NUM_ELEMENTS_TO_AVERAGE_OFFSET:
            average = sum(self.offset_array) / NUM_ELEMENTS_TO_AVERAGE_OFFSET
            #self.get_logger().info('Average offset:' + str(average))
            return (average)
        else:
            return (offset)

    def filter_signal_heading(self, heading):
        if not math.isnan(heading):  # Check if offset is NaN
            self.heading_array.append(heading)
        else:
            self.heading_array.append(self.previous_heading)

        if len(self.heading_array) > NUM_ELEMENTS_TO_AVERAGE_HEADING:
            self.heading_array.pop(0)  # Remove the oldest element from the array
        if len(self.heading_array) == NUM_ELEMENTS_TO_AVERAGE_HEADING:
            average = sum(self.heading_array[0:NUM_ELEMENTS_TO_CONSIDER_HEADING]) / NUM_ELEMENTS_TO_CONSIDER_HEADING
            #self.get_logger().info('Average heading:' + str(average))
            return (average)
        else:
            return (heading)

    def sigmoid_controller(self, angle):
	    return MIN_THRUST + (MAX_THRUST + SIGMOID_YMAX_OFFSET - MIN_THRUST)/(1 + math.exp(-SIGMOID_SLOPE*(angle - SIGMOID_X_OFFSET)))

    def send_ackermann(self, steering_angle, thrust):
        if DRIVE_MODE == 1: # constant thrust for drag racing
            thrust = CONSTANT_THRUST_DRAG
        ack_msg = AckermannDrive()
        ack_msg.steering_angle = steering_angle
        ack_msg.steering_angle_velocity = 0.0
        ack_msg.speed = thrust #CONSTANT_THRUST 
        ack_msg.acceleration = 0.0
        ack_msg.jerk = 0.0
        self.ackermann_pub.publish(ack_msg)

    def send_ackermann_halt(self):
        ack_msg = AckermannDrive()
        ack_msg.steering_angle = 0.0
        ack_msg.speed = 0.0
        self.ackermann_pub.publish(ack_msg)

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

    def emergency_shutdown_callback(self, msg):
        if msg.emergency_stop == True:
            self.emergency_stop = True

            self.get_logger().info('EMERGENCY STOP!')
            # Send ackermann_halt for 2 second
            t0 = self.get_clock().now().to_msg().sec
            t_close = 2
            while (self.get_clock().now().to_msg().sec - t0) < t_close:
                self.send_ackermann_halt()
        else: 
            self.get_logger().info('Resuming!')
            self.emergency_stop = False
        
        ## Alternative for RC
        # if joy_msg.buttons == 2:
        #     self.emergency_stop = True
        #     self.get_logger().info('Killswitch activated!')


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()

    rclpy.spin(node)

if __name__ == '__main__':
    main()