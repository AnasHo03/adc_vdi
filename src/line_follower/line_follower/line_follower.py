#!/usr/bin/env python3

from platform import node
import rclpy
import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import UInt8, Int16MultiArray
from team_interfaces.msg import Lane
from team_interfaces.msg import Emergency
from team_interfaces.msg import Signs
from team_interfaces.msg import Trafficlight

# Python dependancies
import numpy as np # Import the NumPy scientific computing library
import math
import time
import sys


# Parameters general
DRIVE_MODE = 0 # 0 = normal lap, 1 = drag racing, 2 = overtaking, 3 = park out cross
BLIND_OVERTAKE = True
SNAIL_MODE = False
DELAY_IN_FRAMES = 70

# Parameters steering
MAX_STEERING_ANGLE = 0.442  # [rad]
MAX_STEERING_ANGLE_DRAG = 0.1
KP_LO = 0.016/1.4 # Proportional gain constant
KP_DRAG_RACING = 0.019 / 1.4   # drag racing KP
KI = 0.0001    # Integral gain
KI_DRAG_RACING = 0.0004 / 1.4 # drag racing KI
INTEGRAL_CONTROLLER_FRAMES = 8 # frames
KD = 0.00050    # Derivative gain
KD_DRAG_RACING  = 0.0050 / 1.4 # drag racing KD
STEERING_BIAS = -0.021
HEADING_ANGLE_MULTIPLIER = -3
HEADING_ANGLE_MULTIPLIER_SQUARE = -5

# Parameters speed
MIN_THRUST = 0.5  #0.7  # Pursuit: 0.75 #Timed trial: 0.7 / 0.5 / 0.9 / Competition: 1.0
MAX_THRUST = 1.0 #1.4  # Pursuit: 1.2  #Timed trial: 1.4 / 1.0 / 1.4 / Competition : 1.6
MIN_THRUST = float(sys.argv[1])
MAX_THRUST = float(sys.argv[2])
MAX_OVERTAKING_THRUST = 4.5
CONSTANT_THRUST = float(0.6)  # [m/second] (min. is 0.4 m/s)
CONSTANT_THRUST_DRAG = 1.7
KP_THRUST = 1.0
SIGMOID_SLOPE = 4
SIGMOID_X_OFFSET = 1.04
SIGMOID_YMAX_OFFSET = 0.45
USS_PUNISHMENT_MULTIPLIER = 0.0 #0.05

# Parameters ultra-sonic sensors
USS_MAX_DRAW_FRONT_LEFT = 20
USS_MAX_DRAW_FRONT_MIDDLE = 20 
USS_MAX_DRAW_FRONT_RIGHT = 20

# Parameters signs
THRESHOLD_SIGN_HEIGHT_MIN = 300
THRESHOLD_SIGN_HEIGHT_MAX = 450

# Parameters filtering
NUM_ELEMENTS_TO_AVERAGE_OFFSET = 1
NUM_ELEMENTS_TO_AVERAGE_HEADING = 10
NUM_ELEMENTS_TO_CONSIDER_HEADING = 5 # must be smaller than NUM_ELEMENTS_TO_AVERAGE_HEADING
NUM_ELEMENTS_TO_AVERAGE_THRUST = 5

# Parameters overtaking
OVERTAKING_LANE_LENGTH = 5.0
if SNAIL_MODE:
    OVERTAKING_LANE_LENGTH = 10.0
LEFT_OVERTAKE = False #True means left, false means right
OVERTAKING_ALLOWED_FRAMES = 1
OVERTAKING_ALLOWED_TIMEOUT = 4.0
THRESHOLD_ALLOWED_HEADING = 0.08
OPPONENT_SEEN_TIMEOUT = 3.0

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # Variables
        self.offset_array = []
        self.heading_array = []
        self.thrust_array = []

        # Logic variables
        self.emergency_stop = False
        self.destroyed = False
        self.off_track_mode = False

        # Variables
        self.center_offset = 0.0
        self.heading_angle = 0.0
        self.current_heading = 0.0
        self.previous_center_offset = 0.0
        self.previous_heading = 0.0
        self.integral_term = []
        self.integral_term_speed = 0.0
        self.start_ctr = 0
        self.uss_data_front = np.full(3, 15)  # Front-left, front-middle, front-right
                            # right-front, right back
                            # back-right, back-middle, back_left
                            # left-back, left-front 
        self.punishment_term = 0
        self.min_front_distance = 0
        self.thrust = 0.0
        self.average_thrust = 0.0
        self.overtaking_allowed_frames = []
        self.within_overtaking_region = False
        self.car_in_front = False
        self.optimal_heading_to_overtake = False

        self.time_since_overtaking_allowed = 0.0
        self.time_since_opponent_seen = 0.0

        # Define messages
        self.ack_msg = AckermannDrive()
        
        # Define a compatible QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Change to BEST_EFFORT
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize subscribers
        self.lane_sub = self.create_subscription(Lane, 'lane_topic', self.lane_callback, 2)
        # self.lane_sub_off_track = self.create_subscription(Lane, 'lane_topic', self.line_follow_off_track, 10)
        self.emergency_sub = self.create_subscription(Emergency, 'emergency', self.emergency_shutdown_callback, 2)
        self.detected_signs_sub = self.create_subscription(Signs, 'detected_signs', self.detected_signs_callback, 2)
        self.uss_sensors_sub = self.create_subscription(Int16MultiArray, 'uss_sensors', self.uss_callback, qos_profile=qos_profile)    
        
        # Initialize publiher
        self.ackermann_pub = self.create_publisher(AckermannDrive, '/ackermann_cmd', 2)
        
        # Register shutdown callback (function triggered at ctr+c) 
        signal.signal(signal.SIGINT, self.shutdown_callback)

    def uss_callback(self, msg):
        considered_distances = [100, 100, 100]

        self.uss_data_front = msg.data[7:10]
        if self.uss_data_front[0] <= USS_MAX_DRAW_FRONT_LEFT and self.uss_data_front[0] != -1 and self.uss_data_front[0] != -2:
            considered_distances[0] = self.uss_data_front[0]
        if self.uss_data_front [1] <= USS_MAX_DRAW_FRONT_MIDDLE and self.uss_data_front[1] != -1 and self.uss_data_front[1] != -2:
            considered_distances[1] = self.uss_data_front[1]
        if self.uss_data_front[2] <= USS_MAX_DRAW_FRONT_RIGHT and self.uss_data_front[2] != -1 and self.uss_data_front[2] != -2:
            considered_distances[2] = self.uss_data_front[2]
       
        self.min_front_distance = min(considered_distances)
       
        if self.min_front_distance < 100:
            #self.get_logger().info('Min front distance:' + str(self.min_front_distance))

            self.time_since_opponent_seen = time.time()
        #self.get_logger().info('Min front distance:' + str(self.min_front_distance))
        # self.get_logger().info('Front left:' + str(self.uss_data_front[0]))
        #self.get_logger().info('Front middle:' + str(self.uss_data_front[1]))
        # self.get_logger().info('Front right:' + str(self.uss_data_front[2]))



    def detected_signs_callback(self, msg):
        
        # make sure overtaking is really overtaking
        if msg.overtaking_allowed and msg.sign_detected == True:
            self.overtaking_allowed_frames.append(1)
            #self.get_logger().info('Height is:' + str(msg.sign_height))


        else:
            self.overtaking_allowed_frames.append(0)
        if len(self.overtaking_allowed_frames) > OVERTAKING_ALLOWED_FRAMES:
            self.overtaking_allowed_frames.pop(0)

        # very confident that there's an overtaking sign now
        if sum(self.overtaking_allowed_frames) == OVERTAKING_ALLOWED_FRAMES:
            # sign is close to car -> now car is in overtaking region
            if (msg.sign_height > THRESHOLD_SIGN_HEIGHT_MIN) and (msg.sign_height < THRESHOLD_SIGN_HEIGHT_MAX):
                self.within_overtaking_region = True
                self.time_since_overtaking_allowed = time.time()

        if (time.time() - self.time_since_opponent_seen) > OPPONENT_SEEN_TIMEOUT: 
            self.within_overtaking_region = False
        if (time.time() - self.time_since_overtaking_allowed) > OVERTAKING_ALLOWED_TIMEOUT:
            self.within_overtaking_region = False

        if self.within_overtaking_region:
            #self.get_logger().info('WITHIN REGION!!!')
            # average and current heading angle must be straight and two lanes must be detected
            if (self.heading_angle < abs(THRESHOLD_ALLOWED_HEADING)) and (self.current_heading < abs(THRESHOLD_ALLOWED_HEADING)) and self.left_lane_detected and self.right_lane_detected and self.emergency_stop == False:
                # self.send_ackermann_halt()
                self.off_track_mode = True 
                #self.get_logger().info('START MANAEIOUVER! Height:' + str(msg.sign_height))


                self.go_off_track(self.average_thrust, LEFT_OVERTAKE) # True is left, false is right
                self.line_follow_off_track(self.average_thrust)
                self.go_back_on_track(self.average_thrust, LEFT_OVERTAKE) # True is left, false is right
                self.off_track_mode = False

                self.send_ackermann_halt()

                self.within_overtaking_region = False
        #self.get_logger().info('Height is:' + str(msg.sign_height))
        return

    def go_off_track(self, current_thrust, overtake_left):
        ack_msg = AckermannDrive()
        hardcode_steer = 0.35
        hardcode_thrust = 1.7
        t = 0.55 / (current_thrust + 0.1)
        if t > 1.0:
            t = 1.0
        # HARDCODED SEQUENCE START
        # going out
        if overtake_left:
            ack_msg.steering_angle = -hardcode_steer
        else:
            ack_msg.steering_angle = hardcode_steer
        ack_msg.speed = hardcode_thrust * current_thrust 

        # t0 = time.time()
        # while (time.time() - t0) < t:
        #     self.ackermann_pub.publish(ack_msg)
        self.ackermann_pub.publish(ack_msg)
        time.sleep(t)
        if self.emergency_stop:
            return

        # restraighten
        if overtake_left:
            ack_msg.steering_angle = hardcode_steer + STEERING_BIAS
        else:
            ack_msg.steering_angle = -hardcode_steer + STEERING_BIAS
        ack_msg.speed = hardcode_thrust * current_thrust 

        # t0 = time.time()
        # while (time.time() - t0) < t:
        #     self.ackermann_pub.publish(ack_msg)
        self.ackermann_pub.publish(ack_msg)
        time.sleep(t)
        if self.emergency_stop:
            return

        # HARDCODED SEQUENCE END
        return

    def line_follow_off_track(self, current_thrust):
        ack_msg = AckermannDrive()
        hardcode_thrust = 2.5
        t = 0.0
        while True:
            delta_speed = (current_thrust*(hardcode_thrust - 1))
            t = 1 / (delta_speed + 0.1)
            if delta_speed * t < OVERTAKING_LANE_LENGTH:
                break
            else:
                hardcode_thrust = hardcode_thrust * 1.2
        ack_msg.steering_angle = 0.0 + STEERING_BIAS
        ack_msg.speed = hardcode_thrust * current_thrust 

        if SNAIL_MODE:
            ack_msg.speed = 3.0
            t = 2.0
        if ack_msg.speed > MAX_OVERTAKING_THRUST:
            ack_msg.speed = MAX_OVERTAKING_THRUST

        t0 = time.time()
        while (time.time() - t0) < t:
            self.ackermann_pub.publish(ack_msg)
        if self.emergency_stop:
            return

        return

    def go_back_on_track(self, current_thrust, overtake_left):
        ack_msg = AckermannDrive()
        hardcode_steer = 0.35
        hardcode_thrust = 1.7
        t = 0.55 / (current_thrust + 0.1)
        if t > 1.0:
            t = 1.0
        # HARDCODED SEQUENCE START
        # going out
        if overtake_left:
            ack_msg.steering_angle = hardcode_steer + STEERING_BIAS
        else:
            ack_msg.steering_angle = -hardcode_steer + STEERING_BIAS
        ack_msg.speed = hardcode_thrust * current_thrust 

        # t0 = time.time()
        # while (time.time() - t0) < t:
        #     self.ackermann_pub.publish(ack_msg)
        self.ackermann_pub.publish(ack_msg)
        time.sleep(t)
        if self.emergency_stop:
            return
            
        # restraighten
        # if overtake_left:
        #     ack_msg.steering_angle = -hardcode_steer
        # else:
        #     ack_msg.steering_angle = hardcode_steer
        # ack_msg.speed = hardcode_thrust * current_thrust 

        # t0 = time.time()
        # while (time.time() - t0) < t:
        #     self.ackermann_pub.publish(ack_msg)
        # if self.emergency_stop:
        #     return
        # HARDCODED SEQUENCE END
        return



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

        if self.emergency_stop == False and self.off_track_mode == False:

            if DRIVE_MODE in [0,1,2]:
                # Filter signal
                self.center_offset = self.filter_signal_offset(self.center_offset)
                self.average_thrust = self.filter_signal_thrust(self.thrust)
                self.current_heading = self.heading_angle
                self.heading_angle = self.filter_signal_heading(self.heading_angle)
                

                # Flexible offset point depending on heading angle to bank sharper on curves
                self.center_offset = self.center_offset + HEADING_ANGLE_MULTIPLIER * self.heading_angle + HEADING_ANGLE_MULTIPLIER_SQUARE * self.heading_angle * abs(self.heading_angle)

                # Calculate steering angle with PID 
                steering_angle = self.pid_controller(self.center_offset, self.previous_center_offset)

                # Calculate thrust
                self.thrust = self.speed_controller(self.heading_angle)

                # Update previous offset and heading if offset is NaN
                if not math.isnan(self.center_offset):
                    self.previous_center_offset = self.center_offset
                if not math.isnan(self.heading_angle):
                    self.previous_heading = self.heading_angle

                # Publish Ackermann message after delay
                if self.start_ctr > DELAY_IN_FRAMES and not SNAIL_MODE:
                    self.send_ackermann(steering_angle, self.thrust)
                elif self.start_ctr > DELAY_IN_FRAMES and SNAIL_MODE:
                    self.send_ackermann(steering_angle, 0.6)
                    if np.random.rand(1,1) < 0.03:
                        self.send_ackermann(steering_angle, 0.0)
                        time.sleep(1.0)
                # Countdown
                elif (DELAY_IN_FRAMES - self.start_ctr) % 13 == 0:
                    if DRIVE_MODE == 1:
                        self.get_logger().info("DRAG RACE MODE!")

                    self.get_logger().info(str((DELAY_IN_FRAMES - self.start_ctr) / 13))
                self.start_ctr += 1
                if self.start_ctr == DELAY_IN_FRAMES:
                    self.emergency_stop = True 
            elif DRIVE_MODE == 3:

                # Publish Ackermann message after delay
                if self.start_ctr > DELAY_IN_FRAMES:
                    # Hardcoded stuff
                    self.send_ackermann(0, 0.7)
                # Countdown
                elif (DELAY_IN_FRAMES - self.start_ctr) % 13 == 0:
                    if DRIVE_MODE == 1:
                        self.get_logger().info("DRAG RACE MODE!")

                    self.get_logger().info(str((DELAY_IN_FRAMES - self.start_ctr) / 13))
                self.start_ctr += 1
                if self.start_ctr == DELAY_IN_FRAMES:
                    self.emergency_stop = True 

    def pid_controller(self, center_offset, previous_center_offset):
        # Desired offset is zero
        current_error = 0 - center_offset
        previous_error = 0 - previous_center_offset
        

        # Signal (terms combined) and proportional
        if DRIVE_MODE == 0 or DRIVE_MODE == 2:
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

        # Apply punishment term if too close to oponent car
        ### TODO: only use punishment term for racing (DRIVE_MODE = 2)
        if DRIVE_MODE == 2 and self.min_front_distance != 100:
            self.punishment_term =  1/(USS_PUNISHMENT_MULTIPLIER*self.min_front_distance+1) * self.sigmoid_controller(norm_heading_angle)
            proportional_term = proportional_term - self.punishment_term
            

        #self.get_logger().info('punishment term:' + str(self.punishment_term))
        #self.get_logger().info('proportional_term:' + str(proportional_term))

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

    def filter_signal_thrust(self, thrust):
        self.thrust_array.append(self.thrust)
        if len(self.thrust_array) > NUM_ELEMENTS_TO_AVERAGE_THRUST:
            self.thrust_array.pop(0)  # Remove the oldest element from the array
        if len(self.thrust_array) == NUM_ELEMENTS_TO_AVERAGE_THRUST:
            average = sum(self.thrust_array) / NUM_ELEMENTS_TO_AVERAGE_THRUST
            #self.get_logger().info('Average offset:' + str(average))
            return (average)
        else:
            return (thrust)

    def filter_signal_heading(self, heading):
        if not math.isnan(heading):  # Check if offset is NaN
            self.heading_array.append(heading)
        else:
            self.heading_array.append(self.previous_heading)

        if len(self.heading_array) > NUM_ELEMENTS_TO_AVERAGE_HEADING:
            self.heading_array.pop(0)  # Remove the oldest element from the array
        if len(self.heading_array) == NUM_ELEMENTS_TO_AVERAGE_HEADING:
            average = sum(self.heading_array[0:NUM_ELEMENTS_TO_CONSIDER_HEADING]) / NUM_ELEMENTS_TO_CONSIDER_HEADING
            return (average)
        else:
            return (heading)

    def sigmoid_controller(self, angle):
	    return MIN_THRUST + (MAX_THRUST + SIGMOID_YMAX_OFFSET - MIN_THRUST)/(1 + math.exp(-SIGMOID_SLOPE*(angle - SIGMOID_X_OFFSET)))

    def send_ackermann(self, steering_angle, thrust):
        if DRIVE_MODE == 1: # constant thrust for drag racing
            thrust = CONSTANT_THRUST_DRAG
        ack_msg = AckermannDrive()
        ack_msg.steering_angle = steering_angle + STEERING_BIAS
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
            self.within_overtaking_region = False

            self.get_logger().info('EMERGENCY STOP!')
            # Send ackermann_halt for 2 second
            t0 = self.get_clock().now().to_msg().sec
            t_close = 1
            while (self.get_clock().now().to_msg().sec - t0) < t_close:
                self.send_ackermann_halt()
        else: 
            self.get_logger().info('Resuming!')
            self.emergency_stop = False
        
        if msg.phase_change == True:
            self.get_logger().info('START MANEOUIVER!')
            self.off_track_mode = True
            self.go_off_track(self.average_thrust, LEFT_OVERTAKE) # True is left, false is right
            self.line_follow_off_track(self.average_thrust)
            self.go_back_on_track(self.average_thrust, LEFT_OVERTAKE) # True is left, false is right
            self.off_track_mode = False

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