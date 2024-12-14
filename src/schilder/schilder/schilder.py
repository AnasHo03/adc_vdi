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
            depth=30)



# ############ Classifier ##############
# CLASSES = ['cross_parking','overtaking_allowed','overtaking_forbidden','parallel_parking','pit_in','pit_out']

## camera output params
width = 1280
height = 720
scale = 1




class LaneRecognition(Node):
    def __init__(self):
        super().__init__('lane_recognition_node')
        self.cv_bridge = CvBridge()
        self.boxes = []
        self.scores = []
        self.heights = []
        self.class_ids = []
        
        model_path = get_package_share_directory('traffic_control_system_detection')+'/model_files/new_models/best_refitted.pt'
        self.model = YOLO(model_path, task = 'detect')  # pretrained YOLOv8n model 
        self.image_sub= self.create_subscription(ROS_Image,'picture',self.timer_callback,qos_profile =qos_profile)
        self.publisher_signs = self.create_publisher(Signs,'detected_signs',10)
        self.cross_parking = False
        self.parallel_parking = False
        self.pit_in = False
        self.pit_out = False
        self.overtaking_allowed = False
        self.overtaking_forbidden = False
        self.sign_height = 0.0
        
        

      
       
        

    def timer_callback(self,msg):
       try:
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
            #cv2.imwrite('image.jpeg',image)
            # Use get_data() to get the numpy array
            self.image_ocv = image

        

        
            cv_image = cv2.cvtColor(self.image_ocv, cv2.COLOR_RGB2BGR)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)


            # Run inference
            results = self.model(cv_image, device = 'cpu', imgsz = 640, conf = 0.5, stream=True)

            # Preprocess output
            for result in results:
                self.boxes = result.boxes
            self.boxes=self.boxes.data.cpu()
            self.boxes=self.boxes.numpy() #see if this still works

            # scores=[]
            # height=[]
            # class_ids=[]

            for i in range(len(self.boxes)):
                self.scores.append(self.boxes[i][4])
                self.heights.append(self.boxes[i][3])
                self.class_ids.append(self.boxes[i][5])

            # Publish the detections as ROS String message
            self.result_msg = Signs()

            max_class_id = 6
            if self.scores:
                max_score_index = np.argmax(self.scores)
                max_class_id = self.class_ids[max_score_index]
                self.result_msg.sign_detected = True
                self.sign_height = self.heights[max_score_index]

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
       except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
        # Yolo image streamer
        # if self.img_saving_counter_yolo % 4 == 0:
        #     name = './frame_samples_zed_troubleshoot/yolo/img_' + str(self.img_saving_counter_yolo) + '.jpeg'
        #     cv2.imwrite(name, cv_image)
        # self.img_saving_counter_yolo += 1


        


      
def main(args=None):
    rclpy.init(args=args)
    node = LaneRecognition()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()