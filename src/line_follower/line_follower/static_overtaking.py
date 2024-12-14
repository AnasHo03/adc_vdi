#!/usr/bin/env python3

from platform import node
import time
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
from mxcarkit_vehctrl_message.msg import VehCtrlCustomMessage
from mxcarkit_uss_message.msg import USSCustomMessage
from std_msgs.msg import UInt8, Int16MultiArray , UInt16MultiArray
from mxcarkit_vehctrl_message.msg import VehCtrlCustomMessage
from vesc_msgs.msg import VescState, VescStateStamped




# Python dependancies
import numpy as np # Import the NumPy scientific computing library
import math
import time
import sys


# Parameters general
# Normal lap: lane, rc
# Drag: lane message
# Overtaking: lane message, signs, uss
# Parking: traffic light
DISTANCE = 2000
DRIVE_MODE = 0 # 0 = normal lap, 1 = drag racing, 2 = overtaking, 3 = park out cross
SNAIL_MODE = False # Not used. Princple is to slow our car in pursuit mode to 
                   # provide a dissadvantage for the opnent. Develop if needed
DELAY_IN_FRAMES = 70 # Count down till first Ackermann is sent

# Parameters steering
MAX_STEERING_ANGLE = 0.442  # [rad]
MAX_STEERING_ANGLE_DRAG = 0.1
KP_LO = 0.016/2.0 #1.4 # Proportional gain constant (divided by 1.4 for no good reason, use float)
KP_DRAG_RACING = 0.019 / 1.4   # drag racing KP
KI = 0.0001    # Integral gain
KI_DRAG_RACING = 0.0004 / 1.4 # drag racing KI
INTEGRAL_CONTROLLER_FRAMES = 8 # frames to consider
KD = 0.00050    # Derivative gain
KD_DRAG_RACING  = 0.0050 / 1.4 # drag racing KD
STEERING_BIAS = -0.021 # Correct hardware bias. (See if you can fix hardware)
HEADING_ANGLE_MULTIPLIER = -3 # Adapt to high curvature (modeled by a poloynomial function)
HEADING_ANGLE_MULTIPLIER_SQUARE = -5 # Adapt to high curvature (modeled by a poloynomial function)

# Parameters speed
# [m/second] (min. is 0.4 m/s)
MIN_THRUST = 0.4   # 0.7  # Pursuit: 0.75 #Timed trial: 0.7 / 0.5 / 0.9 / Competition: 1.0
MAX_THRUST = 0.4  #0.6 # 1.4  # Pursuit: 1.2  #Timed trial: 1.4 / 1.0 / 1.4 / Competition : 1.6
#MIN_THRUST = float(sys.argv[1]) # Argument when launching
#MAX_THRUST = float(sys.argv[2]) # Argument when launching
MAX_OVERTAKING_THRUST = 4.5
CONSTANT_THRUST_DRAG = 1.7
KP_THRUST = 1.0 # Proportional term of speed controller
SIGMOID_SLOPE = 4 # Params for sigmoid shape. Those and min/max thrust define the curve (plot for visualization)
SIGMOID_X_OFFSET = 1.04 
SIGMOID_YMAX_OFFSET = 0.45
USS_PUNISHMENT_MULTIPLIER = 0.05 # Proportional slow-down depending on USS signal

# Parameters ultra-sonic sensors
USS_MAX_DRAW_FRONT_LEFT = 20
USS_MAX_DRAW_FRONT_MIDDLE = 20 
USS_MAX_DRAW_FRONT_RIGHT = 20


# Parameters filtering
NUM_ELEMENTS_TO_AVERAGE_OFFSET = 1
NUM_ELEMENTS_TO_AVERAGE_HEADING = 5
NUM_ELEMENTS_TO_CONSIDER_HEADING = 3 # must be smaller than NUM_ELEMENTS_TO_AVERAGE_HEADING
NUM_ELEMENTS_TO_AVERAGE_THRUST = 5

# Parameters overtaking
THRESHOLD_SIGN_HEIGHT_MIN = 300
THRESHOLD_SIGN_HEIGHT_MAX = 450
OVERTAKING_LANE_LENGTH = 5.0
if SNAIL_MODE:
    OVERTAKING_LANE_LENGTH = 10.0
LEFT_OVERTAKE = False #True means left, false means right
OVERTAKING_ALLOWED_FRAMES = 1   # Shift register size to confirm overtaking sign was seen
OVERTAKING_ALLOWED_TIMEOUT = 4.0 # Last confirmation of overtaking sign
THRESHOLD_ALLOWED_HEADING = 0.08 # Used to initiate the manevouver only if straight heading
OPPONENT_SEEN_TIMEOUT = 3.0     # Last seen oponent
start_time =time.time()
class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        # Variables
        self.offset_array = []
        self.heading_array = []
        self.thrust_array = []
        self.traffic= False #Green is false
        # Logic variables
        self.parking_step_=0
        self.parking_step_counter_ =0
        self.emergency_stop = False
        self.neutral = False
        self.destroyed = False
        self.off_track_mode = False
        self.distance_counter = 0
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
        self.steering_angle =0.0
        self.average_thrust = 0.0
        self.overtaking_allowed_frames = []
        self.within_overtaking_region = False
        self.car_in_front = False
        self.optimal_heading_to_overtake = False
        self.overtaking_allowed = False
        self.overtaking_forbidden = False

        self.time_since_overtaking_allowed = 0.0
        self.time_since_opponent_seen = 0.0
        
        self.auto_detected = False
        self.parking_step = 0
        self.parking_step_counter_0 =0
        self.parking_step_counter_1 =0
        self.parking_step_counter_2 =0
        self.parking_step_counter_3 =0
        self.parking_step_counter_4 =0
        self.difference_displcament = 0
        # USS
        self.uss_array = []
        self.average_uss = 0.0
        self.auto_detected_check =False
        
        # Manual driving variables
        self.remote_pwm = 0     # holds remote state
        self.steering_pwm = 0
        self.throttle_pwm = 0

        # Define messages
        self.ack_msg = AckermannDrive()
        
        # Define a compatible QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Change to BEST_EFFORT
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize subscribers
        self.lane_sub = self.create_subscription(Lane, 'lane_topic', self.lane_callback, 10)
        self.traffic = self.create_subscription(Trafficlight, 'traffic_light', self.traffic_callback, 10)
        self.signs = self.create_subscription(Signs,'detected_signs',self.signs_callback,10)
        self.uss_sensors_sub = self.create_subscription(USSCustomMessage, 'uss_sensors', self.uss_callback, qos_profile=qos_profile)    
        self.veh_remote_sub = self.create_subscription(VehCtrlCustomMessage, 'veh_remote_ctrl', self.veh_remote_callback, qos_profile=qos_profile)    
        self.sensor_core_sub = self.create_subscription(VescStateStamped, 'sensors/core', self.displacement_callback, qos_profile =qos_profile)
        self.brake_light_pub = self.create_publisher(UInt16MultiArray, '/brake_light', 10)

        self.initial_displacement = None
        self.displacement_difference = 0
        self.gear_ratio = 2.769

        # Initialize publiher
        self.ackermann_pub = self.create_publisher(AckermannDrive, '/ackermann_cmd', 10)
        self.lights_pub = self.create_publisher(UInt16MultiArray, '/lights', 10)

        
        # Register shutdown callback (function triggered at ctr+c) 
        signal.signal(signal.SIGINT, self.shutdown_callback)
    def lights(self,front_light,blinker,brake):
        light_msg= UInt16MultiArray()
        light_msg.data = [0,0,0,0,0,0,0,0,0]
        brake_light_msg = UInt16MultiArray()
        brake_light_msg.data =[0,0,0,0]
        if front_light == "ON" :
            light_msg.data[0] =2
        elif front_light == "on" :
            light_msg.data[0]=1
        elif front_light == "off":
            light_msg.data[0]=0

        if blinker == "right" :
            light_msg.data[4]=1
            light_msg.data[5]=20
            light_msg.data[6]=5
            light_msg.data[7]=20

        elif blinker == "left" :
            light_msg.data[4]=2
            light_msg.data[4]=20
            light_msg.data[4]=5
            light_msg.data[4]=20
            
        elif blinker == "all" :
            light_msg.data[4]=3
            light_msg.data[4]=20
            light_msg.data[4]=5
            light_msg.data[4]=20

        elif blinker == "off":
            light_msg.data[4]=0
        if brake == "run" :
            brake_light_msg.data[2]=1
        elif brake == "stop" :
            brake_light_msg.data[0]=1
        elif brake == "parked" :
            brake_light_msg.data[1]=1
        elif brake == "off":
            brake_light_msg.data[1]=0
            brake_light_msg.data[2]=0
        # self.get_logger().info("light must be on ")
        self.lights_pub.publish(light_msg)
        self.brake_light_pub.publish(brake_light_msg)
  
    def veh_remote_callback(self, msg):
        self.remote_pwm = msg.steering_pwm
        self.steering_pwm = msg.remote_pwm
        self.throttle_pwm = 0.0 #msg.throttle_pwm

        if (self.remote_pwm > 1990 and self.remote_pwm < 2010):
            self.emergency_stop = True
            self.neutral = False
        elif (self.remote_pwm > 990 and self.remote_pwm < 1010):
            self.emergency_stop = False
            self.neutral = False
        else:
            self.emergency_stop = True
            self.neutral = True
        # print("emergency stop :"+str(self.emergency_stop))
        # print(" neutral :"+str(self.neutral))
        # print (str(self.remote_pwm))
        # print (str(self.steering_pwm))
        # print (str(self.throttle_pwm))
    def displacement_callback(self,msg):
        self.displacement = msg.state.displacement
        # print(self.displacement)
        self.dispacement()  
    def uss_callback(self, msg):
        considered_distances = [100, 100, 100]

        self.uss_data_front = msg.uss_range[7:10]
        if self.uss_data_front[0] <= USS_MAX_DRAW_FRONT_LEFT and self.uss_data_front[0] != -1 and self.uss_data_front[0] != -2:
            considered_distances[0] = self.uss_data_front[0]
        if self.uss_data_front [1] <= USS_MAX_DRAW_FRONT_MIDDLE and self.uss_data_front[1] != -1 and self.uss_data_front[1] != -2:
            considered_distances[1] = self.uss_data_front[1]
        if self.uss_data_front[2] <= USS_MAX_DRAW_FRONT_RIGHT and self.uss_data_front[2] != -1 and self.uss_data_front[2] != -2:
            considered_distances[2] = self.uss_data_front[2]
       
        self.min_front_distance = min(considered_distances)
       
        if self.uss_data_front [1] < 80 and self.min_front_distance > 0 :
            self.auto_detected_counter = True


#self.get_logger().info('Min front distance:' + str(self.min_front_distance))
            
            #self.time_since_opponent_seen = time.time()
        # self.get_logger().info('Min front distance:' + str(self.min_front_distance))
        # self.get_logger().info('Front left:' + str(self.uss_data_front[0]))
        # self.get_logger().info('Front middle:' + str(self.uss_data_front[1]))
        # self.get_logger().info('Front right:' + str(self.uss_data_front[2]))
    def traffic_callback(self,msg):
        self.traffic=msg.traffic_light

    def signs_callback(self,msg):
        self.overtaking_allowed=msg.overtaking_allowed
        self.overtaking_forbidden=msg.overtaking_forbidden
    def overtaking_manouver(self):
        self.get_logger().info("Start overtaking ")    
        if self.parking_step == 0:
            self.get_logger().info("Step 00000000000000000 ")    

            if self.parking_step_counter_0 == 0 :
                self.parking_step_counter_0 = 1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 600 : 
                self.lights("ON","left","run")
                self.get_logger().info("diff :" + str(self.difference_displcament))
                self.send_ackermann(-0.5,0.4)
            else :
                self.parking_step = 1                    ############################################33
        elif self.parking_step == 1 :
            self.get_logger().info("Step 1 ")    

            if self.parking_step_counter_1 == 0:
                self.parking_step_counter_1 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 200 : 
                self.send_ackermann(0.5,0.4)
                self.lights("ON","right","run")

            else :
                self.parking_step = 2
        elif self.parking_step == 2 :
            self.get_logger().info("Step 2 ")    

            if self.parking_step_counter_2 == 0:
                self.parking_step_counter_2 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 200 : 
                self.send_ackermann(0.0,0.4)
            else :
                self.parking_step = 3
        elif self.parking_step == 3 :
            self.get_logger().info("Step 3 ")    

            if self.parking_step_counter_3 == 0:
                self.parking_step_counter_3 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 400 : 
                self.send_ackermann(0.5,0.4)
            else :
                self.parking_step = 4
        elif self.parking_step == 4 :
            self.get_logger().info("Step 3 ")    

            if self.parking_step_counter_4 == 0:
                self.parking_step_counter_4 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 100 : 
                self.send_ackermann(0.0,0.4)
            else :
                self.parking_step = 5
        elif self.parking_step ==5:
            self.get_logger().info("Self aligining ")    
            #self.self_aligning()
            self.parking_step = 6
        else :
            self.send_ackermann(self.steering_angle, self.thrust)
            self.lights("off","off","off")
            print("overtaking is done")

    def reset_displacement(self):
        self.initial_displacement = None
        self.difference_displcament = 0
        print("Reset")
 
    def dispacement(self):
        if self.initial_displacement is None:
            self.initial_displacement = self.displacement
        else :
            self.current_displacement = self.displacement
            self.difference_displcament = (self.current_displacement - self.initial_displacement)*self.gear_ratio
        # print("difference_displcament : " + str(self.difference_displcament))
  
    def lane_callback(self, msg):
        # Fetch current offset from message
        self.center_offset = msg.center_offset
        self.heading_angle = msg.heading_angle
        self.right_lane_detected = msg.right_lane_detected
        self.left_lane_detected = msg.left_lane_detected
        self.second_time = time.time()
        # self.get_logger().info("time is :"+str(self.second_time-start_time ))

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

        # manual steering
            

        #if self.emergency_stop == False and self.off_track_mode == False and self.remote_pwm == SOMETHING: 
        

        if self.emergency_stop == False and self.neutral == False:
            
            # Filter signal
            self.center_offset = self.filter_signal_offset(self.center_offset)
            self.average_thrust = self.filter_signal_thrust(self.thrust)
            self.current_heading = self.heading_angle
            self.heading_angle = self.filter_signal_heading(self.heading_angle)
            

            # Flexible offset point depending on heading angle to bank sharper on curves
            self.center_offset = self.center_offset #+ HEADING_ANGLE_MULTIPLIER * self.heading_angle + HEADING_ANGLE_MULTIPLIER_SQUARE * self.heading_angle * abs(self.heading_angle)

            # Calculate steering angle with PID 
            steering_angle = self.pid_controller(self.center_offset, self.previous_center_offset)
            self.steering_angle = steering_angle
            # Calculate thrust
            self.thrust = self.speed_controller(self.heading_angle)

            # Update previous offset and heading if offset is NaN
            if not math.isnan(self.center_offset):
                self.previous_center_offset = self.center_offset
            if not math.isnan(self.heading_angle):
                self.previous_heading = self.heading_angle
            if self.auto_detected :
                self.overtaking_manouver()
            else:
                if self.parking_step_ == 0:
                    if self.parking_step_counter_ ==0:
                        self.parking_step_counter_ =1
                        self.reset_displacement()
                    if abs(self.difference_displcament) <= DISTANCE:
                        print("here and distance is : " + str(self.difference_displcament))
                        self.send_ackermann(self.steering_angle, self.thrust)
                    else:
                        if self.auto_detected_counter :
                            self.distance_counter += 1
                            self.get_logger().info(str(self.distance_counter))
                        if self.distance_counter == 15 :
                            self.auto_detected = True
     
        elif self.emergency_stop == True and self.neutral == False :
            manual_steer = float(0)
            manual_thrust = float(0)
            # self.get_logger().info(str(self.remote_pwm))
            # self.get_logger().info(str(self.throttle_pwm))
            if self.steering_pwm > 1510 or self.steering_pwm < 1490:
                manual_steer = float(self.steering_pwm - 1500)*0.442/600.0
            else :
                manual_steer = 0.0
            if self.throttle_pwm > 1260 or self.throttle_pwm < 1240:
                manual_thrust = float((self.throttle_pwm - 1250)*1.0/600.0)
            else : 
                manual_thrust = 0.0
            # print(str(manual_steer))
            # print(str(manual_thrust))c
            # manual_thrust = float(2.5)
            self.send_ackermann_halt()
        else :
            self.send_ackermann_halt()

            

           

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
        return MIN_THRUST #+ (MAX_THRUST + SIGMOID_YMAX_OFFSET - MIN_THRUST)/(1 + math.exp(-SIGMOID_SLOPE*(angle - SIGMOID_X_OFFSET)))

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

   # def emergency_shutdown_callback(self, msg):
    #
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
