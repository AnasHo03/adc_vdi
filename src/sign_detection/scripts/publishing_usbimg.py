#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'image_topic', 10)
        self.timer = self.create_timer(0.1, self.publish_image)
        self.cv_bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Set the camera index according to your system configuration

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            ros_image = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ros_image)

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
