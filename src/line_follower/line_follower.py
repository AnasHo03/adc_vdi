# ROS dependancies
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image as ROS_Image
import time
from std_msgs.msg import UInt16MultiArray

# Python dependancies
import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
import line_follower.edge_detection as edge # Handles the detection of lane lines
from cv_bridge import CvBridge # For converting ROS image message to jpg

# Local dependancies
from lane import Lane

# Parameters
max_steering_angle = 0.4692 # [rad]
constant_thrust = 100 # [rpm]
kp = 0.5  # Proportional gain constant



class Line_Follower():
    def __init__(self):
        # Logic variables
        self.emergency_stop = False
        self.shutdowned = False

        # define messages
        self.ackMsg = AckermannDriveStamped()

        # init Subcriber
        rospy.Subscriber('/joy', UInt16MultiArray, self.emergency_shutdown_callback) 
        rospy.Subscriber('/camera/color/image_raw', ROS_Image, self.cam_callback)

        # init publisher
        self.ctrl_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1) 

        # init hook (Shut down procedure)
        rospy.on_shutdown(self.hook)

    def emergency_shutdown_callback(self, msg):
        if self.joy_msg.buttons == 2:
                emergency_stop = True
                rospy.signal_shutdown('Killswitch activated!')


    def cam_callback(self, col_img_raw):
        # Return if emergncy stop activated
        if self.emergency_stop == True: return
        
        # Convert ROS message to jpeg (TODO: find more efficient way to load image to Lane)
        jpeg_img = self.convert_image_msg_to_jpg(col_img_raw.data)
        # TODO: possible more efficient way
        #self.bridge.imgmsg_to_cv2(col_img_raw.data, "bgr8")


        # Lane recognition (Fetch center_offset) 
        lane_instance = Lane(col_img_raw.data)
        center_offset = lane_instance.calculate_car_position(print_to_terminal=True)
        print("Center Offset:", center_offset)

        # Calculate steering angle (P-controller)
        steering_angle = self.calculate_steering_angle(center_offset, max_steering_angle)

        # Publish Ackermann message
        self.send_ackermann(steering_angle)
    
    def convert_image_msg_to_jpg(self, col_img_raw_data):
        bridge = CvBridge()

        # Convert the ROS image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(col_img_raw_data, "bgr8")

        # Convert the OpenCV image to JPEG format
        ret, jpeg_image = cv2.imencode('.jpg', cv_image)

        return jpeg_image

    def calculate_steering_angle(self, center_offset, max_steering_angle):
        error = 0 - center_offset  # Desired offset is zero
        control_signal = kp * error
        limited_control_signal = max(min(control_signal, max_steering_angle), -max_steering_angle)
        return limited_control_signal

    def send_ackermann(self,steering_angle, thrust = constant_thrust):
        self.ackMsg.header.stamp = rospy.Time.now()
        self.ackMsg.drive.steering_angle = steering_angle
        self.ackMsg.drive.speed = thrust
        self.ackermann_pub.publish(self.ackMsg)

    def hook(self):                 # When node shuts down send 0 thrust and steering for 1 second
        print("Initiate shutdown!")
        self.shutdowned = True
        t0 = time.time()
        t_close = 1

        while (time.time() - t0) < t_close: 
            self.send_ackermann(0,0)

def main():
    node = Line_Follower()
    print("Line follower node initialized")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
