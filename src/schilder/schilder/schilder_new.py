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



# Python dependancies
import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
from numpy.linalg import norm
import math
import pyzed.sl as sl
import sys



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


# Create a ZED camera object


# Set configuration parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720 # Use HD1080 video mode
init_params.camera_fps = 20 # Set fps at 30
#init_params.camera_auto_exposure_gain = True

# Open the camera




class LaneRecognition(Node):
    def __init__(self):
        super().__init__('lane_recognition_node')
        self.image_sub= self.create_subscription(ROS_Image,'picture',self.timer_callback,10)
        model_path = get_package_share_directory('traffic_control_system_detection')+'/model_files/new_models/best_refitted.pt'
        self.model = YOLO(model_path, task = 'detect')  # pretrained YOLOv8n model 
        self.publisher_signs = self.create_publisher(Signs,'detected_signs',10)
        self.cross_parking = False
        self.parallel_parking = False
        self.pit_in = False
        self.pit_out = False
        self.overtaking_allowed = False
        self.overtaking_forbidden = False
        self.sign_height = 0.0
        
    

        # Variables lane detection
        self.center_offset = 0.0
        self.heading_angle = 0.0
        self.img_saving_counter_1 = int(0)
        self.img_saving_counter_2 = int(0)
        self.img_saving_counter_yolo = int(0)
        self.image_ocv = np.empty((0,), dtype=np.float32)

        
        # Initialize subscribers
        #self.camera_sub = self.create_subscription(ROS_Image, '/zed/zed_node/left_raw/image_raw_color', self.cam_callback, 10)
        #self.timer_ = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 fps

        
        
    
    #def cam_callback(self, msg):
    def timer_callback(self,msg):
        image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
        self.image_ocv = image
        
        
    
        cv_image = cv2.cvtColor(self.image_ocv, cv2.COLOR_RGB2BGR)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)


        # Run inference
        results = self.model(cv_image, device = 'cpu', imgsz = 640, conf = 0.5, stream=True)

        # Preprocess output
        for result in results:
            boxes = result.boxes
        boxes=boxes.data.cpu()
        boxes=boxes.numpy() #see if this still works

        scores=[]
        height=[]
        class_ids=[]

        for i in range(len(boxes)):
            scores.append(boxes[i][4])
            height.append(boxes[i][3])
            class_ids.append(boxes[i][5])

        # Publish the detections as ROS String message
        self.result_msg = Signs()

        max_class_id = 6
        if scores:
            max_score_index = np.argmax(scores)
            max_class_id = class_ids[max_score_index]
            self.result_msg.sign_detected = True
            self.sign_height = height[max_score_index]

        else:
            self.result_msg.sign_detected = False

        if max_class_id == 1:
            self.cross_parking = True
            self.parallel_parking = False
        elif max_class_id == 3:
            self.overtaking_allowed = True
            self.overtaking_forbidden = False
        elif max_class_id == 2:
            self.overtaking_allowed = False
            self.overtaking_forbidden = True
        elif max_class_id == 4:
            self.cross_parking = False
            self.parallel_parking = True
        elif max_class_id == 5:
            self.pit_in = True
            self.pit_out = False
        elif max_class_id == 6:
            self.pit_out = True
            self.pit_in = False

        self.result_msg.cross_parking = self.cross_parking
        self.result_msg.parallel_parking = self.parallel_parking
        self.result_msg.overtaking_allowed = self.overtaking_allowed
        self.result_msg.overtaking_forbidden = self.overtaking_forbidden
        self.result_msg.pit_in = self.pit_in
        self.result_msg.pit_out = self.pit_out
        self.result_msg.sign_height = float(self.sign_height)
        
        self.publisher_signs.publish(self.result_msg)

            # Yolo image streamer
            # if self.img_saving_counter_yolo % 4 == 0:
            #     name = './frame_samples_zed_troubleshoot/yolo/img_' + str(self.img_saving_counter_yolo) + '.jpeg'
            #     cv2.imwrite(name, cv_image)
            # self.img_saving_counter_yolo += 1


    


        
def main(args=None):
    rclpy.init(args=args)
    node = LaneRecognition()

    rclpy.spin(node)

if __name__ == '__main__':
    main()