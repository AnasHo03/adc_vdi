#!/usr/bin/env python3
############################################################
# This code is new from Mechatronik-Labor WS23 
############################################################
from platform import node
import rclpy
#import signal
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import UInt8, Int16MultiArray , UInt16MultiArray
from team_interfaces.msg import Lane
# from team_interfaces.msg import Emergency
# from team_interfaces.msg import Signs
# from team_interfaces.msg import Trafficlight
from mxcarkit_vehctrl_message.msg import VehCtrlCustomMessage
from mxcarkit_uss_message.msg import USSCustomMessage
from std_msgs.msg import Bool
from vesc_msgs.msg import VescState, VescStateStamped
from team_interfaces.msg import Trafficlight




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
DRIVE_MODE = 0 # 0 = normal lap, 1 = drag racing, 2 = overtaking, 3 = park out cross
SNAIL_MODE = False # Not used. Princple is to slow our car in pursuit mode to 
                   # provide a dissadvantage for the opnent. Develop if needed
DELAY_IN_FRAMES = 70 # Count down till first Ackermann is sent

# Parameters steering
MAX_STEERING_ANGLE = 0.442  # [rad]
MAX_STEERING_ANGLE_DRAG = 0.1
KP_LO = 0.016/1.4 # Proportional gain constant (divided by 1.4 for no good reason, use float)
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
MIN_THRUST = 0.35  # 0.7  # Pursuit: 0.75 #Timed trial: 0.7 / 0.5 / 0.9 / Competition: 1.0
MAX_THRUST = 0.35 # 1.4  # Pursuit: 1.2  #Timed trial: 1.4 / 1.0 / 1.4 / Competition : 1.6
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
NUM_ELEMENTS_TO_AVERAGE_HEADING = 10
NUM_ELEMENTS_TO_CONSIDER_HEADING = 5 # must be smaller than NUM_ELEMENTS_TO_AVERAGE_HEADING
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

#us sensor array
x=-1
array_usrf=np.array([x,x,x,x,x])
array_usrb=np.array([x])
#print(array_usrb)

#us sensor array for collison prevention
array_us0 = np.array([x,x,x])
array_us1 = np.array([x,x,x,x,x])
array_us2 = np.array([x,x,x,x,x])
array_us3 = np.array([x,x,x,x,x])
array_us4 = np.array([x,x,x,x,x])
array_us5 = np.array([x,x,x,x,x])
array_us6 = np.array([x,x,x,x,x])
array_us7 = np.array([x])
array_us8 = np.array([x])
array_us9 = np.array([x])

# range for parking
range_taken = range(0,60)
range_free_parallel = range(60,100)
range_free_cross = range(100, 120)
range_no_parking = range(120, 3000)



# parking detection status
usrf_detect_parking = "no_parking"
usrb_detect_parking = "no_parking"

#Parking Constants
parallel_parking = True
cross_parking = False

print("Hii")
class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.get_logger().error("code started")


        # Variables
        self.offset_array = []
        self.heading_array = []
        self.thrust_array = []

        # Logic variables
        self.emergency_stop = False
        self.neutral = False
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

        
        # USS
        self.uss_array = []
        self.average_uss = 0.0
        self.filter=0
        self.check=0
        self.check_=0

        #minimumm distance to avoid collision
        y=250
        self.distb=[y,y,y]
        self.distf=[y,y,y]
        self.min_dist = 250
        self.min_dist_front = 250
        self.min_dist_back = 250
        self.dist_usrf = 250
        self.dist_usrb = 250
        self.front_obstacle = False
        self.back_obstacle = False


        
        #parking status
        self.last_state_parking = "no_parking"
        self.p_slot_det =False
        self.c_slot_det = False
        self.t0 = 0
        self.tpf = 0
        self.tpb =0
        self.started =False
        self.parked = False
        self.align =False
        self.go_right =False
        self.go_left = False
        self.check=0
        self.check_=0
        self.check_free=0   
        self.check_true=0
        self.parking=True #False
        # Manual driving variables
        self.remote_pwm = 0     # holds remote state
        self.steering_pwm = 0
        self.throttle_pwm = 0

        # Define messages
        self.ack_msg = AckermannDrive()
        self.steering_angle = 0.0
        self.thrust = 0.0

        self.ausparking_step = 0
        self.ausparking_step_counter_0 =0
        self.ausparking_step_counter_1 =0
        self.ausparking_step_counter_2 =0
        self.ausparking_step_counter_3 =0
        self.ausparking_step_counter_4 =0


        #Parking step counter
        self.parking_step = 0
        self.parking_step_counter_0 =0
        self.parking_step_counter_1 =0
        self.parking_step_counter_2 =0
        self.parking_step_counter_3 =0
        self.parking_step_counter_4 =0
        self.ausparken_ok = True
        self.difference_displcament = 0
        
        # Define a compatible QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Change to BEST_EFFORT
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize subscribers
        self.lane_sub = self.create_subscription(Lane, 'lane_topic', self.lane_callback, 2)
        #self.lane_sub_off_track = self.create_subscription(Lane, 'lane_topic', self.line_follow_off_track, 10)
        #self.emergency_sub = self.create_subscription(Emergency, 'emergency', self.emergency_shutdown_callback, 2)
        #self.detected_signs_sub = self.create_subscription(Signs, 'detected_signs', self.detected_signs_callback, 2)
        #self.uss_sensors_sub = self.create_subscription(USSCustomMessage, 'uss_sensors', self.uss_callback, qos_profile=qos_profile)    
        self.uss_parking_sub = self.create_subscription(USSCustomMessage, 'uss_sensors', self.parking_callback, qos_profile=qos_profile)    
        self.veh_remote_sub = self.create_subscription(VehCtrlCustomMessage, 'veh_remote_ctrl', self.veh_remote_callback, qos_profile=qos_profile)    
        #self.light = self.create_subscription(UInt16MultiArray, 'lights', self.lights_callback, qos_profile=qos_profile)    
        self.brake_light = self.create_subscription(UInt16MultiArray, 'brake_light', self.brake_light_callback, qos_profile=qos_profile)    
        self.sensor_core_sub = self.create_subscription(VescStateStamped, 'sensors/core', self.displacement_callback, qos_profile =qos_profile)
        self.traffic_light_sub = self.create_subscription(Trafficlight, '/traffic_light', self.traffic_light_callback, qos_profile =qos_profile)

        # Initialize publiher
        self.ackermann_pub = self.create_publisher(AckermannDrive, '/ackermann_cmd', 10)
        #self.ackermann_timer = self.create_timer(0.05, self.send_ackermann_cmd)
        self.lights_pub = self.create_publisher(UInt16MultiArray, '/lights', qos_profile=qos_profile)
        self.brake_light_pub = self.create_publisher(UInt16MultiArray, '/brake_light', 10)
        #self.parked_pub= self.create_publisher(Bool, 'Einparken', 10)
        self.initial_displacement = None
        self.displacement_difference = 0
        self.gear_ratio = 2.769

        # Register shutdown callback (function triggered at ctr+c) 
        # signal.signal(signal.SIGINT, self.shutdown_callback)
    def veh_remote_callback(self, msg):
        self.remote_pwm = msg.steering_pwm
        self.steering_pwm = msg.remote_pwm
        self.throttle_pwm = msg.throttle_pwm

        if (self.remote_pwm > 1990 and self.remote_pwm < 2010):
            self.emergency_stop = True
            self.neutral = False
        elif (self.remote_pwm > 990 and self.remote_pwm < 1010):
            self.emergency_stop = False
            self.neutral = False
        else:
            self.emergency_stop = True
            self.neutral = True
        print(str(self.emergency_stop))
        # print (str(self.remote_pwm))
        # print (str(self.steering_pwm))
        # print (str(self.throttle_pwm))

    def displacement_callback(self,msg):
        self.displacement = msg.state.displacement
        # print(self.displacement)
        self.dispacement()
    def traffic_light_callback(self,msg):
        self.traffic_light = msg.traffic_light

   
    def lights_callback(self, msg):
        self.get_logger().info('light is on')
    def brake_light_callback(self, msg):
         pass #self.get_logger().info('brake light is on')

    ###########################################################
    #this part added by WS2023 Team
    ###########################################################
    def parking_callback (self, msg):

        #self.dist = msg.uss_range
        #self.min_dist = np.min(self.dist)
        if self.parking:
        #cash the data from us sensor
            self.cache_us0 = msg.uss_range[0]
            self.cache_us1 = msg.uss_range[1]
            self.cache_us2 = msg.uss_range[2]
            self.cache_us3 = msg.uss_range[3]
            self.cache_us4 = msg.uss_range[4]
            self.cache_us5 = msg.uss_range[5]
            self.cache_us6 = msg.uss_range[6]
            self.cache_us7 = msg.uss_range[7]
            self.cache_us8 = msg.uss_range[8]
            self.cache_us9 = msg.uss_range[9]


            array_us0[0:2] = array_us0[1:3]
            array_us0[2]=self.cache_us0
            array_us1[0:4] = array_us1[1:5]
            array_us1[4]=self.cache_us1
            array_us2[0:4] = array_us2[1:5]
            array_us2[4]=self.cache_us2
            array_us3[0:4] = array_us3[1:5]
            array_us3[4]=self.cache_us3
            array_us4[0:4] = array_us4[1:5]
            array_us4[4]=self.cache_us4
            array_us5[0:4] = array_us5[1:5]
            array_us5[4]=self.cache_us5
            array_us6[0:4] = array_us6[1:5]
            array_us6[4]=self.cache_us6
            #array_us7[0:4] = array_us7[1:5]
            array_us7=self.cache_us7
            #array_us8[0:4] = array_us8[1:5]
            array_us8=self.cache_us8
            #array_us9[0:4] = array_us9[1:5]
            array_us9=self.cache_us9

            
            dist_us0=np.round(np.mean(array_us0),0)
            dist_us1=np.round(np.mean(array_us1),0)
            dist_us2=np.round(np.mean(array_us2),0)
            dist_us3=np.round(np.mean(array_us3),0)
            dist_us4=np.round(np.mean(array_us4),0)
            dist_us5=np.round(np.mean(array_us5),0)
            dist_us6=np.round(np.mean(array_us6),0)
            dist_us7=np.round(np.mean(array_us7),0)
            dist_us8=np.round(np.mean(array_us8),0)
            dist_us9=np.round(np.mean(array_us9),0)
            if dist_us0 != -1 and dist_us1 != -1:
                if abs(dist_us0 - dist_us1) < 5 and (dist_us0 < 20 or dist_us1 < 20):
                    self.align = True
                else:
                    self.align = False
                    if dist_us0 > dist_us1 :
                        self.go_right = True
                        self.go_left =False
                    else:
                        self.go_right = False
                        self.go_left =True
            else : 
                self.align =False
            if dist_us8 < 35  and dist_us8 > 0 :
                self.front_free = False
            else :
                self.front_free = True
            if dist_us8 < 30   and dist_us8 > 0 :
                self.front_free_ausparken = False
            else :
                self.front_free_ausparken = True
            if dist_us3 < 25 and dist_us3 > 0 :
                self.back_free = False
            else :
                self.back_free = True
            # print("self.align :" + str(self.align))
            # print("self.go_right : " + str(self.go_right))
            # print("self.front : " + str(self.front_free))
            # print("self.back : " + str(self.back_free))
            

            if (abs(array_us0[2]-array_us0[1]))>80 and (self.filter==0):
                dist_us0=np.round(np.mean(array_us0[0:1]))
                self.get_logger().info('Did new')
                self.filter=array_us0[2]

            elif(abs(array_us0[2]-array_us0[1]))>80 and (self.filter!=0):
                array_us0[1]=array_us0[0]
                self.filter=0

            elif(abs(array_us0[2]-array_us0[1]))<80 and (self.filter!=0): 
                array_us0[0]=array_us0[1]
                dist_us0=np.round(np.mean(array_us0))
                self.filter=0
            
            else:
                self.filter=0
            #print(str(dist))

        
            #### cach the data from ultrasonic sensors 
            # self.cache_usrf = msg.uss_range[0]

            # array_usrf[0:4] = array_usrf[1:5]
            # array_usrf[4]=self.cache_usrf
            # print(str(array_usrf))
            #array_usrb[0:4] = array_usrb[1:5]
            #self.get_logger().info(str(array_usrf))
            #self.get_logger().info(str(array_usrb))

            # median of Array (last 5 values of right front and backus sensor ) 
            dist_usrf=dist_us0
            #print(str(array_us0))
            #print(str(dist_usrf))
            # self.get_logger().info("usrf distance = " + str(dist_usrf))
            # self.get_logger().info("usrb distance = " + str(dist_usrb))
            self.dist_usrf = dist_usrf
            # find the parking status for right front us sensor
            if dist_usrf in range_no_parking:
                usrf_detect_parking = "no_parking"
                self.check_=self.check_+1
                self.check=0
                self.check_free=0

            elif dist_usrf in range_free_parallel:
                self.check_free=self.check_free+1
                self.check_=0
                self.check=0
                if self.check_free> 3: #####3
                    usrf_detect_parking = "free_parallel"
                else:
                    usrf_detect_parking = self.last_state_parking
                
            elif dist_usrf in range_free_cross:
                usrf_detect_parking = "free_cross"
                self.check_=0
                self.check=0
                self.check_free=0

            elif dist_usrf in range_taken:
                usrf_detect_parking ="taken"
                self.check=self.check+1
                self.check_=0
                self.check_free=0
            else:
                self.get_logger().info("error usrf")
                usrf_detect_parking = self.last_state_parking
                self.check_free=0
                self.check_=0
                self.check=0
            #self.get_logger().info("usrf detect parking : " + usrf_detect_parking)
            #print("self.check= "+str(self.check)+" "+str(self.check_)+" " +str(self.check_free))
            
            #comparing the state of both us sensors
            light_msg= UInt16MultiArray()

            if parallel_parking == True :    
                if usrf_detect_parking == "free_parallel" :    
                    #self.get_logger().warn(str(usrf_detect_parking)) 
                    self.last_state_parking = usrf_detect_parking
                    #self.send_ackermann_halt() ## added 08.11
                    self.p_slot_det = True 

                # elif (self.last_state_parking == "free_parallel")  ####################### and ((self.check_>3) or (self.check>3)):
                #     self.p_slot_det = True 
                    #self.send_ackermann_halt()
                    #self.get_logger().warn("p slot detected")

            if cross_parking == True :    
                if usrf_detect_parking == "free_cross" :    
                    self.get_logger().warn(str(usrf_detect_parking)) 
                    self.last_state_parking = usrf_detect_parking
                elif (self.last_state_parking == "free_cross") and ((self.check_>3) or (self.check>3)):
                    self.c_slot_det = True 
                    self.send_ackermann_halt()
                    self.get_logger().warn("c slot detected") 
                self.get_logger().info("park place is "+ str(self.c_slot_det))
                      

                #self.get_logger().info("park place is "+ str(self.p_slot_det))
    
    def lane_callback(self,msg):
        self.dashed= msg.dashed_line_detected
        self.center_offset = msg.center_offset
        self.heading_angle = msg.heading_angle
        self.right_lane_detected = msg.right_lane_detected
        self.left_lane_detected = msg.left_lane_detected
        #print(str(self.dashed))
        light_msg= UInt16MultiArray()
        brake_light_msg = UInt16MultiArray()
        self.lights_pub.publish(light_msg)
        self.brake_light_pub.publish(brake_light_msg)

        if self.emergency_stop == False:
            if self.ausparken_ok:
                print("ausparken started")
                self.lights("off","left","run")
                self.parallel_ausparken()
            else:
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
                    if self.parking_step == 0 :
                        if self.parking_step_counter_0 ==0:
                            self.parking_step_counter_0 = 1
                            self.reset_displacement()
                        if abs(self.difference_displcament) <= 3000 :
                            self.send_ackermann(steering_angle, self.thrust) # here add the traffic light condition if it is red car stops and by green car drives
                            self.lights("off","left","run")
                        else :
                            self.parking_step = 1
                    elif self.parking_step == 1:
                        if self.traffic_light :
                            self.send_ackermann_halt()
                            self.lights("off","off","stop")
                        else :
                            self.send_ackermann(steering_angle, self.thrust)
                            self.parking_step = 2
                    else:
                        self.send_ackermann(steering_angle, self.thrust)
                        self.lights("off","left","run")        
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
            self.send_ackermann(manual_steer,manual_thrust)
        else :
            self.send_ackermann_halt()

    def parallel_ausparken(self):
        self.get_logger().info("Start Parallel Parking ")    
        self.lights("off","left","run")
        if self.ausparking_step == 0:
            self.get_logger().info("Step 00000000000000000 ")    

            if self.ausparking_step_counter_0 == 0 :
                self.ausparking_step_counter_0 = 1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 50 : 
                self.send_ackermann(0.5,-0.3)
            else :
                self.ausparking_step = 1                    ############################################33
        elif self.ausparking_step == 1 :
            self.get_logger().info("Step 1 ")    

            if self.ausparking_step_counter_1 == 0:
                self.ausparking_step_counter_1 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 50 : 
                self.send_ackermann(-0.5,0.3)
            else :
                self.ausparking_step = 2
        elif self.ausparking_step == 2 :
            self.get_logger().info("Step 2 ")    

            if self.ausparking_step_counter_2 == 0:
                self.ausparking_step_counter_2 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 50 : 
                self.send_ackermann(0.5,-0.3)
            else :
                self.ausparking_step = 3
        elif self.ausparking_step == 3 :
            self.get_logger().info("Step 3 ")    

            if self.ausparking_step_counter_3 == 0:
                self.ausparking_step_counter_3 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 50 : 
                self.send_ackermann(-0.5,0.3)
            else :
                self.ausparking_step = 4
        elif self.parking_step == 4 :
            self.get_logger().info("Step 3 ")    

            if self.ausparking_step_counter_4 == 0:
                self.ausparking_step_counter_4 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 200 : 
                self.send_ackermann(0.0,0.3)
            else :
                self.ausparking_step = 5
        else :
            self.ausparken_ok =False
            
            
    
        
        
            
            
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
           
    def send_ackermann(self,steering_angle, thrust):
        ack_msg = AckermannDrive()
        # self.get_logger().info("thrust is :" + str(thrust))
        # self.get_logger().info("front obstacle is :" + str(self.front_obstacle))
        # self.get_logger().info("back obstacle is :" + str(self.back_obstacle))

        if self.front_obstacle == True and thrust > 0:
            ack_msg.speed = 0.0
            self.get_logger().error("FRONT OBSTACLE")
        elif self.back_obstacle == True and thrust < 0 :
            ack_msg.speed = 0.0
            self.get_logger().error("BACK OBSTACLE")
        else:
            ack_msg.speed = thrust #thrust #CONSTANT_THRUST
            ack_msg.steering_angle = steering_angle + STEERING_BIAS
            ack_msg.steering_angle_velocity = 0.0
            ack_msg.acceleration = 0.0
            ack_msg.jerk = 0.0
        self.ackermann_pub.publish(ack_msg)

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
  
    def self_aligning(self):
        print("self.align : "+ str(self.align))
        if not self.align:
            if self.go_right:
                if self.front_free:
                    self.send_ackermann(0.5,0.3)
                    print("this step")
                elif not self.front_free and self.back_free:
                    self.send_ackermann(-0.5,-0.3)
                    print("that sttep")
            else :
                if self.front_free and not self.back_free:
                    self.send_ackermann(-0.5,0.3)
                elif self.back_free and not self.front_free:
                    self.send_ackermann(0.5,-0.3)
                else :      
                    pass
        else :

            self.parking_step = 5


    def parallel_parking_manouver(self):
        self.get_logger().info("Start Parallel Parking ")    
        if self.parking_step == 0:
            self.get_logger().info("Step 00000000000000000 ")    

            if self.parking_step_counter_0 == 0 :
                self.parking_step_counter_0 = 1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 600 : 
                self.send_ackermann(0.0,0.3)
            else :
                self.parking_step = 1                    ############################################33
        elif self.parking_step == 1 :
            self.get_logger().info("Step 1 ")    

            if self.parking_step_counter_1 == 0:
                self.parking_step_counter_1 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 250 : 
                self.send_ackermann(-0.5,0.3)
            else :
                self.parking_step = 2
        elif self.parking_step == 2 :
            self.get_logger().info("Step 2 ")    

            if self.parking_step_counter_2 == 0:
                self.parking_step_counter_2 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 450 : 
                self.send_ackermann(0.5,-0.3)
            else :
                self.parking_step = 3
        elif self.parking_step == 3 :
            self.get_logger().info("Step 3 ")    

            if self.parking_step_counter_3 == 0:
                self.parking_step_counter_3 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 650 : 
                self.send_ackermann(-0.5,-0.3)
            else :
                self.parking_step = 4
        elif self.parking_step == 4 :
            self.get_logger().info("Step 3 ")    

            if self.parking_step_counter_4 == 0:
                self.parking_step_counter_4 =1
                self.reset_displacement()
            if abs(self.difference_displcament) <= 50 : 
                self.send_ackermann(0.5,0.3)
            else :
                self.parking_step = 5
        elif self.parking_step ==5:
            self.get_logger().info("Self aligining ")    
            #self.self_aligning()
            self.parking_step = 6
        else :
            self.send_ackermann_halt()
            self.p_slot_det=False
            parallel_parking = False
            self.parked =True
            

        #steering < 0 ---> turns left
        #steering > 0 ---> turns right
        #speed < 0 ---> goes forward
        #speed > 0 ---> goes backward

        # self.p_slot_det=False
        # parallel_parking = False
        # self.parked =True

    def send_ackermann_halt(self):
        ack_msg = AckermannDrive()
        ack_msg.steering_angle = 0.0
        ack_msg.speed = 0.0
        #self.lights("off","off","off")
        self.ackermann_pub.publish(ack_msg)

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


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()

    rclpy.spin(node)

if __name__ == '__main__':
    main()