#!/usr/bin/env python3

from platform import node
import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import UInt8, UInt16MultiArray
from sensor_msgs.msg import Image as ROS_Image
from team_interfaces.msg import Lane
from team_interfaces.msg import Signs
from team_interfaces.msg import Trafficlight
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge # For converting ROS image message to jpg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Python dependancies
import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
from numpy.linalg import norm
import math
import pyzed.sl as sl
import sys
qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Change to BEST_EFFORT
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

## Modes
debug_mode = True # True to enable printing out images
debug_modulo = 20

drag_mode = False


############ Classifier ##############
CLASSES = ['cross_parking','overtaking_allowed','overtaking_forbidden','parallel_parking','pit_in','pit_out']

## camera output params
width = 1280
height = 720
scale = 1

## BEV params
top_roi = 0.4395 # 0.4395 # 0-1 (0 is top)
bottom_roi = 0.6347 #0.6347 # 0-1 (0 is top)
width_use = 1 # 0-1 (For cropping vertically. Not used)
height_multiplier = 3.5 # How much to stretch vertically
skew_level = 0.887 # 0-1 (1 is triangle, 0 is rectangle)

## filter params
thresh = 130 #94 # 0-255 (lower means higher sensitivity) 
#thresh = int(sys.argv[1])
gaussian = 13 # must be odd number
adaptive_block_size_factor = 11 # must be odd number
adaptive_const = 2
adaptive_block_size = int(adaptive_block_size_factor*scale)
if adaptive_block_size % 2 == 0:
	adaptive_block_size += 1

## line detect params
search_step = int(3) #2 # px
search_offset = int(8)
left_search_dist = int(94 + search_offset)
right_search_dist = int(94 + search_offset)
start_search_height = int(488) #488
height_step = int(8) #8
radius = 20 # px
angle_sweep_0 = 200 # deg
angle_sweep_1 = 140 # deg
sweep_step = math.radians(6)
max_points = 6

start_angle_0 = math.radians(90+angle_sweep_0/2)
stop_angle_0 = math.radians(90-angle_sweep_0/2)
start_angle_1 = math.radians(90+angle_sweep_1/2)
stop_angle_1 = math.radians(90-angle_sweep_1/2)

## process lane params
lane_distance = int(74) #74
center_offset_constant = 645


roi_in = np.float32(np.floor([
	((1-width_use)*width*scale,0), # Top-left corner
	((1-width_use)*width*scale,(bottom_roi-top_roi)*height*scale), # Bottom-left corner            
	(width_use*width*scale,(bottom_roi-top_roi)*height*scale), # Bottom-right corner
	(width_use*width*scale,0) # Top-right corner
]))
roi_out = np.float32(np.floor([
	((1-width_use)*width*scale,0), # Top-left corner
	(width*skew_level*0.5*width_use*scale,(bottom_roi-top_roi)*height*height_multiplier*scale), # Bottom-left corner            
	(width*(1-skew_level*0.5*width_use)*scale,(bottom_roi-top_roi)*height*height_multiplier*scale), # Bottom-right corner
	(width_use*width*scale,0) # Top-right corner
]))
bird_transform_matrix = cv2.getPerspectiveTransform(roi_in,roi_out)








class LaneRecognition(Node):
    def __init__(self):
        super().__init__('lane_recognition_node')
        self.cv_bridge = CvBridge()
        self.image_sub=self.create_subscription(ROS_Image,'picture',self.timer_callback,qos_profile =qos_profile)

        
        
       
        self.publisher_traffic_light = self.create_publisher(Trafficlight, 'traffic_light', 10)
        self.decision = False


        
        
        # Initialize subscribers
        #self.camera_sub = self.create_subscription(ROS_Image, '/zed/zed_node/left_raw/image_raw_color', self.cam_callback, 10)
        self.timer_ = self.create_timer(1.0 / 29.0, self.timer_callback)  # 30 fps

        
       
    
    #def cam_callback(self, msg):
    def timer_callback(self,msg):
        image=msg

        self.image_ocv=self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='bgra8')

        self.decision = self.detectTrafficLight(self.image_ocv, self.decision)
        traffic_lights_msg = Trafficlight()
        traffic_lights_msg.traffic_light = self.decision
        self.publisher_traffic_light.publish(traffic_lights_msg)




       


        
    

    def detectTrafficLight(self, cv_img, decision):

        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)

        # variables
        size = cv_img.shape
        font = cv2.FONT_HERSHEY_SIMPLEX
        cimg = cv_img
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)    # convert to hsv space
        # select color to detect
        if decision == False:           # detect red traffic light
            # rgb color range
            lower = np.array([50, 50, 200])         # bgr
            upper = np.array([255, 255, 255])       # bgr
            # hsv color range
            lower_1 = np.array([0, 150, 200])
            upper_1 = np.array([15, 255, 255])    # 30 155 255
            lower_2 = np.array([165, 150, 200])   # 160 50 200
            upper_2 = np.array([180, 255, 255])   # 180 155 255
            # constants
            color_label = 'RED'
            thresh = 90
            # create masks
            mask_rgb = cv2.inRange(cv_img, lower, upper)        # rgb mask
            mask1 = cv2.inRange(hsv, lower_1, upper_1)
            mask2 = cv2.inRange(hsv, lower_2, upper_2)
            mask_hsv = cv2.add(mask1, mask2)                # hsv mask
        elif decision == True:        # detect green traffic light
            # rgb color range
            lower = np.array([100, 200, 100])       # bgr
            upper = np.array([255, 255, 255])       # bgr
            # hsv color range
            lower_1 = np.array([40, 150, 200])      # 40,50,150
            upper_1 = np.array([90, 255, 255])    # 0,255,255
            # constants
            color_label = 'GREEN'
            thresh = 30
            # create masks
            mask_rgb = cv2.inRange(cv_img, lower, upper)        # rgb mask
            mask_hsv = cv2.inRange(hsv, lower_1, upper_1)    # hsv mask
        # hough circle detection in hsv threshold image
        circles = cv2.HoughCircles(mask_hsv, cv2.HOUGH_GRADIENT, 1, 80, param1=50, param2=10, minRadius=0, maxRadius=50)
        # constants
        num = 0        # counter for relevant circles
        radius = 0    # radius of relevant circle
        # iterate over detected circles
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                if i[0] > size[1] or i[1] > size[0]:    # Check circle center point
                    continue
                h, s = 0.0, 0.0        # value & amount of pixels included
                r = int(i[2])        # current radius
                # check average over region in rgb threshold image
                for m in range(-r, r):
                    for n in range(-r, r):
                        if (i[1] + m) >= size[0] or (i[0] + n) >= size[1]:
                            continue
                        h += mask_rgb[i[1] + m, i[0] + n]
                        s += 1
                # print('h/s',h/s)
                # Check if average in center point region high enough
                if h / s > thresh:
                    cv2.circle(cimg, (i[0], i[1]), i[2] + 1, (0, 0, 0), 2)
                    cv2.circle(mask_hsv, (i[0], i[1]), i[2] + 3, (255, 255, 255), 2)
                    cv2.circle(mask_rgb, (i[0], i[1]), i[2] + 3, (255, 255, 255), 2)
                    cv2.putText(cimg, color_label, (i[0], i[1]), font, 1, (0, 0, 0), 2, cv2.LINE_AA)
                    num += 1
                    # terminate after threshold (1) amount of accepted cirles
                    if num == 1:
                        radius = i[2]
                        xpos = i[0]
                        break
        # Check radius and x position in image
        if radius > 1 and xpos > 320:
            decision = not decision #Change of Light detected
        #     print('decision:', decision)
        #     print('Xpos:', xpos)
        # print("lul")
        return decision



    ## transform image to euclidian distance
    

        
def main(args=None):
    rclpy.init(args=args)
    node = LaneRecognition()

    rclpy.spin(node)

if __name__ == '__main__':
    main()