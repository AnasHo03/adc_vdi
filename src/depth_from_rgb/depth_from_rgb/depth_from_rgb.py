import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs
import cv2

class DepthFromRGBNode(Node):
    def __init__(self):
        super().__init__('depth_from_rgb')
        self.color_image = None
        self.depth_image = None
        self.registered_depth_image = None

        self.color_image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_image_callback,
            10
        )

        self.depth_image_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10
        )

        self.align = rs.align(rs.stream.color)  # Create an align object for image registration

    def color_image_callback(self, msg):
        bridge = CvBridge()
        self.color_image = msg

    def depth_image_callback(self, msg):
        bridge = CvBridge()
        self.depth_image = msg

    def run_depth_from_rgb(self):
        bridge = CvBridge()

        while rclpy.ok():
            if self.color_image is not None and self.depth_image is not None:
                # Convert the ROS color image message to OpenCV image
                color_image_cv = bridge.imgmsg_to_cv2(self.color_image, desired_encoding="passthrough")

                # Convert color and depth images to RealSense frames
                color_frame = rs.pyrealsense2.frame(color_image_cv)
                depth_frame = rs.pyrealsense2.depth_frame(self.depth_image)

                # Perform image registration
                aligned_frames = self.align.process([color_frame, depth_frame])

                # Extract registered depth image from the aligned frames
                registered_depth_frame = aligned_frames.get_depth_frame()
                registered_depth_image = np.asanyarray(registered_depth_frame.get_data())

                # Now you have the registered depth image, and you can find the corresponding pixel

                # Example: Find the depth value for a specific pixel in the color image
                color_pixel_x = 320
                color_pixel_y = 240

                # Find the corresponding pixel in the registered depth image
                registered_depth_value = registered_depth_image[color_pixel_y, color_pixel_x]

                self.get_logger().info("Registered Depth at pixel ({}, {}): {}".format(
                    color_pixel_x, color_pixel_y, registered_depth_value))

                # Perform further processing or analysis

            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFromRGBNode()
    node.run_depth_from_rgb()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
