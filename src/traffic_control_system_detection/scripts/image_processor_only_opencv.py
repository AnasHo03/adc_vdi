#!/usr/bin/env python3

import cv2
#import cv2.dnn.cuda as cuda_dnn #*1 for gpu use (open cv with cuda support necessary)
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from team_interfaces.msg import Signs
import os
from ament_index_python.packages import get_package_share_directory

#cuda_dnn.dnn_cuda.init_device() #*1 for gpu use (open cv with cuda support necessary)
CLASSES = ['cross_parking','overtaking_allowed','overtaking_forbidden','parallel_parking','pit_in','pit_out']

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        #set up model
        model_path = os.path.join(get_package_share_directory('sign_detection'), 'model_files', 'best.onnx')
        self.model = cv2.dnn.readNetFromONNX(model_path)    #model = cuda_dnn.readNetFromONNX(model_path) #*1
        self.subscription = self.create_subscription(CompressedImage,'/zed/zed_node/left/image_rect_color/compressed',self.process_image,10)
        self.publisher = self.create_publisher(Signs,'detected_signs',10)
        self.cv_bridge = CvBridge()
        self.cross_parking = False
        self.pit_in = False
        self.pit_out = False
        self.overtaking = False


    def process_image(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess input
        [height, width, _] = cv_image.shape
        length = max((height, width))
        image = np.zeros((length, length, 3), np.uint8)
        image[0:height, 0:width] = cv_image
        blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True)    #blob = cuda_dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True) #*1
        self.model.setInput(blob)

        # Run inference
        outputs = self.model.forward()

        # Preprocess output
        outputs = np.array([cv2.transpose(outputs[0])])    #outputs = np.array([cuda_dnn.imagesFromBlob(outputs)[0]]) #*1
        rows = outputs.shape[1]

        boxes = []
        scores = []
        class_ids = []

        for i in range(rows):
            classes_scores = outputs[0][i][4:]
            (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
            if maxScore >= 0.25:
                box = [
                    outputs[0][i][0] - (0.5 * outputs[0][i][2]), outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                    outputs[0][i][2], outputs[0][i][3]]
                boxes.append(box)
                scores.append(maxScore)
                class_ids.append(maxClassIndex)
 


        # Publish the detections as ROS String message
        self.result_msg = Signs()
        max_class_id = 6
        if scores:
            max_score_index = np.argmax(scores)
            #max_score = scores[max_score_index]
            #max_box = boxes[max_score_index]
            max_class_id = class_ids[max_score_index]
            self.result_msg.sign_detected = True
        else:
            self.result_msg.sign_detected = False

        if max_class_id == 0:
            self.cross_parking = True
        elif max_class_id == 1:
            self.overtaking = True
        elif max_class_id == 2:
            self.overtaking = False
        elif max_class_id == 3:
            self.cross_parking = False
        elif max_class_id == 4:
            self.pit_in = True
            self.pit_out = False
        elif max_class_id == 5:
            self.pit_out = True
            self.pit_in = False

        self.result_msg.cross_parking = self.cross_parking
        self.result_msg.overtaking = self.overtaking
        self.result_msg.pit_in = self.pit_in
        self.result_msg.pit_out = self.pit_out

        # Fix the line below to use self.result_msg
        self.publisher.publish(self.result_msg)

            



def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()