#!/usr/bin/env python3

from platform import node
import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import UInt8, UInt16MultiArray
from sensor_msgs.msg import Image as ROS_Image
from team_interfaces.msg import Lane

# Python dependancies
import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
from cv_bridge import CvBridge # For converting ROS image message to jpg
import math

## camera output params
width = 896
height = 512
scale = 1

## BEV params
top_roi = 0.4238 # 0-1 (0 is top)
bottom_roi = 0.5859 # 0-1 (0 is top)
width_use = 1 # 0-1
height_multiplier = 4.5
skew_level = 0.885 # 0-1

## filter params
thresh = 125 # 0-255 (lower means higher sensitivity) 
gaussian = 9 # must be odd number
adaptive_block_size_factor = 11 # must be odd number
adaptive_const = 2
adaptive_block_size = int(adaptive_block_size_factor*scale)
if adaptive_block_size % 2 == 0:
	adaptive_block_size += 1

## line detect params
search_step = int(2) # px
search_offset = int(7)
left_search_dist = int(62 + search_offset)
right_search_dist = int(62 + search_offset)
start_search_height = int(370)
height_step = int(4)
radius = 20 # px
angle_sweep = 200 # deg
sweep_step = math.radians(5)
max_points = 20

start_angle = math.radians(90+angle_sweep/2)
stop_angle = math.radians(90-angle_sweep/2)

## process lane params
lane_distance = int(44)

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
        # Variables ariables
        self.center_offset = 0.0
        self.heading_angle = 0.0
        self.img_saving_counter_1 = int(0)
        self.img_saving_counter_2 = int(0)

        
        # Initialize subscribers
        self.camera_sub = self.create_subscription(ROS_Image, '/zed/zed_node/left_raw/image_raw_color', self.cam_callback, 10)
        # Initialize CvBridge
        self.bridge = CvBridge()
        # Create publisher
        self.publisher = self.create_publisher(Lane, 'lane_topic', 10)
        
    def cam_callback(self, col_img_raw):
        lane = Lane()
        # Convert the ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(col_img_raw, desired_encoding='bgr8')

        # Image stream writer
        # name = './src/frame_samples_zed_troubleshoot/pid_troubleshooting/img_' + str(self.img_saving_counter_1/20) + '.jpeg'
        # if self.img_saving_counter_1 % 20 == 0:
        #     cv2.imwrite(name, cv_image)
        # self.img_saving_counter_1 += 1

        # Load frame for testing
        #cv_image = cv2.imread('./src/frame_samples_zed/6.jpeg')
        img_bird = self.birdy_view(cv_image)
        img_filtered = self.filter_line(img_bird)
        img_out, left_lane, right_lane = self.detect_lane(img_filtered)


        # Image stream writer (post processing)
        # name = './src/frame_samples_zed_troubleshoot/pid_troubleshooting/postprocess_' + str(self.img_saving_counter_2/20) + '.jpeg'
        # if self.img_saving_counter_2 % 20 == 0:
        #     cv2.imwrite(name, img_out)
        # self.img_saving_counter_2 += 1

        center_offset, heading_angle, left_detected, right_detected = self.process_lane(left_lane, right_lane)
        
        # Print relevant info
        # print("left", left_detected)
        # print("right", right_detected)
        #self.get_logger().info('Center offset:' + str(center_offset))
        #self.get_logger().info('Heading anle:' + str(heading_angle))

        # Fill message
        lane.right_lane_detected = right_detected
        lane.left_lane_detected = left_detected
        lane.center_offset = center_offset
        lane.heading_angle = heading_angle

        # Extract x and y points from left_lane and right_lane
        lane.right_lane_x_points = [float(point[0]) for point in right_lane]
        lane.right_lane_y_points = [float(point[1]) for point in right_lane]
        lane.left_lane_x_points = [float(point[0]) for point in left_lane]
        lane.left_lane_y_points = [float(point[1]) for point in left_lane]

        lane.dashed_line_detected = False # placeholder value

        self.publisher.publish(lane)

    ## transform image to euclidian distance
    def birdy_view(self, img_in):
        # ROI 
        img_in = img_in[int(top_roi*height):int(bottom_roi*height),0:width]
        cropped_height, cropped_width, _ = img_in.shape
        img_out = cv2.warpPerspective(img_in,bird_transform_matrix,(cropped_width,int(cropped_height*height_multiplier)),flags=(cv2.INTER_LINEAR))
        return img_out

    ## thresholding to get binary image
    def filter_line(self, img_in):
        img_out = cv2.cvtColor(img_in, cv2.COLOR_BGR2HLS)
        img_out = cv2.GaussianBlur(img_out,(gaussian,gaussian),0)
        _, img_out = cv2.threshold(img_out[:,:,1], thresh, 255, cv2.THRESH_BINARY)
        # img_out = cv2.adaptiveThreshold(img_out[:,:,1], 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, adaptive_block_size, adaptive_const)
        # img_out = img_out[:,:,1]
        # img_out = cv2.Canny(img_in,100,200)


        return img_out

    ## lane to array of points (x,y)
    def detect_lane(self, img_in):
        max_x = img_in.shape[1]
        max_y = img_in.shape[0]
        blank = np.zeros((max_y,max_x,3), np.uint8)
        blank[:,:,1] = img_in
        img_out = blank
        left_start = int(max_x/2 + search_offset)
        right_start = int(max_x/2 - search_offset)
        left_end = left_start - left_search_dist
        right_end = right_start + right_search_dist
        left_search_height = start_search_height
        right_search_height = start_search_height
        left_lane_found = False
        right_lane_found = False
        left_is_right = False
        lane_points_left = []
        lane_points_right = []

        ## find left lane start to the left of center point
        for x in range(0,5):
            if not left_lane_found:
                left_search_height -= height_step
                left_start = int(max_x/2 + search_offset - x*7)
                while left_start > left_end - (x-1)*4:
                    left_start -= search_step
                    if img_in[left_search_height,left_start] == 0:
                        img_out = cv2.circle(img_out, (left_start,left_search_height), radius = 1,color=(150, 0, 150), thickness=1)
                        continue
                    else:
                        left_lane_found = True
                        lane_points_left.append((left_start,left_search_height))
                        break
            else:
                break

        ## find right lane start to the right of center point
        for x in range(0,5):
            if not right_lane_found:
                right_search_height -= height_step
                right_start = int(max_x/2 + search_offset + x*7)
                while right_start < right_end + (x-1)*4:
                    right_start += search_step
                    if img_in[right_search_height,right_start] == 0:
                        img_out = cv2.circle(img_out, (right_start,right_search_height), radius = 1,color=(150, 150, 0), thickness=1)
                        continue
                    else:
                        right_lane_found = True
                        lane_points_right.append((right_start,right_search_height))
                        break
            else:
                break

        ## check if left lane is right lane
        if left_lane_found and right_lane_found:
            if left_start - 2*search_step <= right_start <= left_start + 2*search_step and left_search_height - 2*height_step <= right_search_height <= left_search_height + 2*height_step:
                left_is_right = True

        ## detect and discretize left lane
        if left_lane_found:
            x = left_start
            y = left_search_height
            heading = math.radians(0)

            ## sweep through angle range
            for i in range(0,max_points):
                valid_points = []
                angle = start_angle
                while angle >= stop_angle:
                    cx = int(radius * math.cos(-angle + heading) + x)
                    cy = int(radius * math.sin(-angle + heading) + y)

                    # adjust if point is beyond image boundaries
                    if cx < 0 or cx >= max_x - 1 or cy < 0 or cy >= max_y:
                        angle -= sweep_step
                        continue
                    img_out = cv2.circle(img_out, (cx,cy), radius = 1,color=(0, 0, 255), thickness=1) # visualization purposes
                    
                    # calculate pixel ratio if found potential line connection
                    if img_in[cy,cx] == 255:
                        # mask = np.zeros_like(img_in)
                        mask = np.zeros((abs(cy-y)+1,abs(cx-x)+1))
                        img_roi = []
                        if cx > x:
                            if cy > y:
                                cv2.line(mask, (0,0), (cx-x,cy-y), 255, 1)
                                img_roi = img_in[y:cy+1,x:cx+1]
                            else:
                                cv2.line(mask, (0,y-cy), (cx-x,0), 255, 1)
                                img_roi = img_in[cy:y+1,x:cx+1]
                        else:
                            if cy > y:
                                cv2.line(mask, (x-cx,0), (0,cy-y), 255, 1)
                                img_roi = img_in[y:cy+1,cx:x+1]
                            else:
                                cv2.line(mask, (x-cx,y-cy), (0,0), 255, 1)
                                img_roi = img_in[cy:y+1,cx:x+1]
                        # cv2.line(mask, (x,y), (cx,cy), 255, 1)
                        region_pixels = np.sum(mask == 255)
                        white_pixels = np.sum(np.logical_and(mask == 255, img_roi == 255))
                        pixel_ratio = (white_pixels / region_pixels)
                        valid_points.append((cx,cy,pixel_ratio))

                    angle -= sweep_step

                # sort all potential connections and pick best choice
                if valid_points:
                    sorted_points = sorted(valid_points, key=lambda tup: tup[2], reverse=True)
                    optimum_point = (sorted_points[0][0], sorted_points[0][1])
                    lane_points_left.append(optimum_point)
                    cv2.line(img_out, (x,y), (optimum_point[0], optimum_point[1]), (255, 0, 0), 3)
                    heading = math.atan2((optimum_point[1]-y),(optimum_point[0]-x)) + math.radians(90)
                    x = optimum_point[0]
                    y = optimum_point[1]
                else:
                    break

        ## detect and discretize right lane
        if right_lane_found and not left_is_right:
            x = right_start
            y = right_search_height
            heading = math.radians(0)

            ## sweep through angle range
            for i in range(0,max_points):
                valid_points = []
                angle = start_angle
                while angle >= stop_angle:
                    cx = int(radius * math.cos(-angle + heading) + x)
                    cy = int(radius * math.sin(-angle + heading) + y)

                    # adjust if point is beyond image boundaries
                    if cx < 0 or cx >= max_x - 1 or cy < 0 or cy >= max_y:
                        angle -= sweep_step
                        continue
                    img_out = cv2.circle(img_out, (cx,cy), radius = 1,color=(255, 0, 0), thickness=1) # visualization purposes
                    
                    # calculate pixel ratio if found potential line connection
                    if img_in[cy,cx] == 255:
                        # mask = np.zeros_like(img_in)
                        mask = np.zeros((abs(cy-y)+1,abs(cx-x)+1))
                        img_roi = []
                        if cx > x:
                            if cy > y:
                                cv2.line(mask, (0,0), (cx-x,cy-y), 255, 1)
                                img_roi = img_in[y:cy+1,x:cx+1]
                            else:
                                cv2.line(mask, (0,y-cy), (cx-x,0), 255, 1)
                                img_roi = img_in[cy:y+1,x:cx+1]
                        else:
                            if cy > y:
                                cv2.line(mask, (x-cx,0), (0,cy-y), 255, 1)
                                img_roi = img_in[y:cy+1,cx:x+1]
                            else:
                                cv2.line(mask, (x-cx,y-cy), (0,0), 255, 1)
                                img_roi = img_in[cy:y+1,cx:x+1]
                        # cv2.line(mask, (x,y), (cx,cy), 255, 1)
                        region_pixels = np.sum(mask == 255)
                        white_pixels = np.sum(np.logical_and(mask == 255, img_roi == 255))
                        pixel_ratio = (white_pixels / region_pixels)
                        valid_points.append((cx,cy,pixel_ratio))
                    angle -= sweep_step

                # sort all potential connections and pick best choice
                if valid_points:
                    sorted_points = sorted(valid_points, key=lambda tup: tup[2], reverse=True)
                    optimum_point = (sorted_points[0][0], sorted_points[0][1])
                    lane_points_right.append(optimum_point)
                    cv2.line(img_out, (x,y), (optimum_point[0], optimum_point[1]), (0, 0, 255), 3)
                    heading = math.atan2((optimum_point[1]-y),(optimum_point[0]-x)) + math.radians(90)
                    x = optimum_point[0]
                    y = optimum_point[1]
                else:
                    break
        
        # img_out = cv2.circle(img_out, (x,y), radius = 30,color=(0, 0, 255), thickness=1)
        return img_out, lane_points_left, lane_points_right

## array of points to center offset
    def process_lane(self, left, right):
        left_defined = len(left) > 1
        right_defined = len(right) > 1

        actual_left = False
        actual_right = False
        if left_defined and right_defined:
            center_offset = 450 - 0.5*(left[0][0]+right[0][0])
            left_angle = math.atan2((left[-1][1] - left[0][1]),(left[-1][0] - left[0][0]))
            right_angle = math.atan2((right[-1][1] - right[0][1]),(right[-1][0] - right[0][0]))
            heading_angle = (left_angle + right_angle)/2 + math.radians(90)
            actual_left = True
            actual_right = True
        elif left_defined:
            lane_heading = left[-1][0] - left[0][0]
            heading_angle = math.atan2((left[-1][1] - left[0][1]),(left[-1][0] - left[0][0])) + math.radians(90)
            if lane_heading > 0:
                center_offset = 450 - (left[0][0] + lane_distance/2)
                actual_left = True
            else:
                center_offset = 450 - (left[0][0] - lane_distance/2)
                actual_right = True
        elif right_defined:
            lane_heading = right[-1][0] - right[0][0]
            heading_angle = math.atan2((right[-1][1] - right[0][1]),(right[-1][0] - right[0][0])) + math.radians(90)
            if lane_heading > 0:
                center_offset = 450 - (right[0][0] + lane_distance/2)
                actual_left = True
            else:
                center_offset = 450 - (right[0][0] - lane_distance/2)
                actual_right = True
        else:
            center_offset = math.nan
            heading_angle = math.nan

        return center_offset, heading_angle, actual_left, actual_right

def main(args=None):
    rclpy.init(args=args)
    node = LaneRecognition()

    rclpy.spin(node)

if __name__ == '__main__':
    main()