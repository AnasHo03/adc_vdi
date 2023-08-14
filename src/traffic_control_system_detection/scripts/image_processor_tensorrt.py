#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from team_interfaces.msg import Signs
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


CLASSES = ['cross_parking','overtaking_allowed','overtaking_forbidden','parallel_parking','pit_in','pit_out']

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        #set up model
        model_path = get_package_share_directory('traffic_control_system_detection')+'/model_files/best_s.engine'
        self.model = YOLO(model_path, task = 'detect')  # pretrained YOLOv8n model
        self.subscription = self.create_subscription(CompressedImage,'/zed/zed_node/left/image_rect_color/compressed',self.process_image,10)
        self.publisher = self.create_publisher(Signs,'detected_signs',10)
        self.cv_bridge = CvBridge()
        self.cross_parking = False
        self.parallel_parking = False
        self.pit_in = False
        self.pit_out = False
        self.overtaking_allowed = False
        self.overtaking_forbidden = False
        self.sign_height = 0.0
        self.ctr = 0

    def process_image(self, msg):
        if self.ctr % 2:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run inference
            results = self.model([cv_image], device = 0, imgsz = 640, conf = 0.5)

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

            if max_class_id == 1 or max_class_id==0:
                self.cross_parking = True
                self.parallel_parking = False
            elif max_class_id == 2:
                self.overtaking_allowed = True
                self.overtaking_forbidden = False
            elif max_class_id == 3:
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

            self.publisher.publish(self.result_msg)
        self.ctr += 1

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()