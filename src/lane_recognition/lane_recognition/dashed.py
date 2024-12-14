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
from std_msgs.msg import Bool



## camera output params
width = 1280
height = 720
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
        super().__init__('dashed_node')
        self.cv_bridge = CvBridge()
        self.publisher_dashed = self.create_publisher(Bool,'dashed',10)
        
        self.image_ocv = np.empty((0,), dtype=np.float32)    
        
        self.Strich = False
        # Initialize subscribers
        #self.camera_sub = self.create_subscription(ROS_Image, '/zed/zed_node/left_raw/image_raw_color', self.cam_callback, 10)
        self.timer_ = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 fps

        
    
    

    def timer_callback(self):
        image = sl.Mat()
        if (zed.grab() == sl.ERROR_CODE.SUCCESS) :
            # A new image is available if grab() returns SUCCESS
            zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
            # Use get_data() to get the numpy array
            self.image_ocv = image.get_data()

            
           # image=self.image_ocv
            #cv2.imwrite('main_image.jpeg',image)
           # image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding='bgra8')
	


   
            
            dashed=Bool()
        

            col_images=[]
        #img = cv2.imread("/home/nano/mechatroniklabor_ws/src/control/ATS/001.png")
            col_images.append(self.image_ocv)
            
        # specify frame index
            idx = 0

            # create a zero array
            stencil = np.zeros_like(col_images[idx][:,:,0])

            # specify coordinates of the polygon
            #polygon = np.array([[10,280], [10,360], [630,360], [630,280]])         #Hough Bereich
            polygon = np.array([[650,400], [650,440], [1020,440], [1020,400]])
            # fill polygon with ones
            cv2.fillConvexPoly(stencil, polygon, 1)

            # apply polygon as a mask on the frame
            img = cv2.bitwise_and(col_images[idx][:,:,0], col_images[idx][:,:,0], mask=stencil)
            cv2.imwrite("008.png",img) 
            # apply image thresholding
            ret, thresh = cv2.threshold(img, 170, 145, cv2.THRESH_BINARY)
            
            # Gestrichelt -------------------------------------------------------------------------------------------
            
            # Hough Lines
            lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 30, maxLineGap=1)      #Line Gap

            # create a copy of the original frame
            dmy = col_images[idx].copy()

            # draw Hough lines
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(dmy, (x1, y1), (x2, y2), (0, 0, 255), 3)             #Evtl Strichdicke und Strichmenge anpassen

            img1 = dmy
            #cv2.imwrite("001.png",img1)
            # plot frame
            #cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/002.png",img1)

            
            
            
            # Durchgezogen Werte=ATS -------------------------------------------------------------------------------------------
            
            # Hough Lines
            lines2 = cv2.HoughLinesP(thresh, 1, np.pi/180, 30, maxLineGap=40)      #Line Gap: orignial 15

            # create a copy of the original frame
            dmy2 = col_images[idx].copy()

            # draw Hough lines
            if lines2 is not None:
                for line in lines2:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(dmy2, (x1, y1), (x2, y2), (0, 0, 255), 3)             #Evtl Strichdicke und Strichmenge anpassen

            img2 = dmy2 
            #cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/003.png",img2)      
            #cv2.imwrite("002.png",img2) 
            
            #Vergleich---------------------------------------------------------------------------------------------   
            
            
            diff = img2.copy()
            cv2.absdiff(img2, img1, diff) 
            #diff2=diff[320:360,320:630,0]       #polygon = np.array([[10,280], [10,360], [630,360], [630,280]])
            diff2=diff[400:440,700:1010,0]       # polygon = np.array([[650,280], [650,360], [1200,360], [1200,280]])
            #cv2.imwrite("004.png",diff)  
            #cv2.imwrite("005.png",diff2)
            print(diff2)      
            print('diff size_:',np.sum(diff2[:,:]))
            
            if np.sum(diff2[:,:])>30000:     #30000
                    self.Strich = True
                    print('diff size_True:',np.sum(diff2[:,:]))
                    #cv2.imwrite("002.png",img1)
                    #cv2.imwrite("003.png",img2)  
                    #cv2.imwrite("004.png",diff) 
                    #cv2.imwrite("005.png",diff2) 
            
            else:
                self.Strich=False  
        dashed.data= self.Strich  
    
        self.publisher_dashed.publish(dashed)
        
def main(args=None):
    rclpy.init(args=args)   
    node = LaneRecognition()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
