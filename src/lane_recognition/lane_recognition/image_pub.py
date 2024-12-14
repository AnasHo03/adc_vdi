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



## camera output params
width =1280 #1280
height = 720  #720
scale = 1





# Create a ZED camera object
zed = sl.Camera()

# Set configuration parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720 # Use HD1080 video mode
init_params.camera_fps = 20 # Set fps at 30
#init_params.camera_auto_exposure_gain = True

# Open the camera
err = zed.open(init_params)
if (err != sl.ERROR_CODE.SUCCESS) :
    print('Zed exited script!')
    exit(-1)



class LaneRecognition(Node):
    def __init__(self):
        super().__init__('lane_recognition_node')
        self.cv_bridge = CvBridge()
        self.publisher_image = self.create_publisher(ROS_Image,'picture',10)
        
        self.image_ocv = np.empty((0,), dtype=np.float32)

        
        
        # Initialize subscribers
        #self.camera_sub = self.create_subscription(ROS_Image, '/zed/zed_node/left_raw/image_raw_color', self.cam_callback, 10)
        self.timer_ = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 fps

        
    
    
    #def cam_callback(self, msg):
    def timer_callback(self):
        image = sl.Mat()
        if (zed.grab() == sl.ERROR_CODE.SUCCESS) :
            # A new image is available if grab() returns SUCCESS
            zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
            # Use get_data() to get the numpy array
            self.image_ocv = image.get_data()
            image=self.image_ocv
            #cv2.imwrite('main_image.jpeg',image)
            image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding='bgra8')

            self.publisher_image.publish(image_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = LaneRecognition()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
