import cv2
import numpy as np
import os
import math

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(script_dir, os.pardir))
# filepath = os.path.join(script_dir, "examples/frames/left", "1.jpg")
# filepath = os.path.join(script_dir, "examples/frames/left", "WIN_20230601_13_31_04_Pro.jpg")

## camera output params
width = 896
height = 512
scale = 1

## BEV params
top_roi = .5138 # 0-1 (0 is top)
bottom_roi = .75 # 0-1 (0 is top)
width_use = 1 # 0-1
height_multiplier = 5
skew_level = 0.885 # 0-1

## filter params
thresh = 146 # 0-255
gaussian = 13 # must be odd number
adaptive_block_size_factor = 11 # must be odd number
adaptive_const = 2
adaptive_block_size = int(adaptive_block_size_factor*scale)
if adaptive_block_size % 2 == 0:
	adaptive_block_size += 1

## line detect params
search_step = int(2) # px
search_offset = int(10)
left_search_dist = int(75 + search_offset)
right_search_dist = int(75 + search_offset)
start_search_height = int(845)
height_step = int(5)
radius = 30 # px
angle_sweep = 200 # deg
sweep_step = math.radians(5)
max_points = 20

start_angle = math.radians(90+angle_sweep/2)
stop_angle = math.radians(90-angle_sweep/2)

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

## loads image and crops to roi
def load_roi_img(path): 
	frame = cv2.imread(path)
	frame = frame[int(top_roi*height):int(bottom_roi*height),0:width]
	return frame

def resize_img(img_in, scale_factor):
	return cv2.resize(img_in, (int(img_in.shape[1]*scale_factor),int(img_in.shape[0]*scale_factor)), interpolation = cv2.INTER_AREA)

def filter_line(img_in):


	img_out = cv2.cvtColor(img_in, cv2.COLOR_BGR2HLS)
	img_out = cv2.GaussianBlur(img_out,(gaussian,gaussian),0)
	_, img_out = cv2.threshold(img_out[:,:,1], thresh, 255, cv2.THRESH_BINARY)
	# img_out = cv2.adaptiveThreshold(img_out[:,:,1], 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, adaptive_block_size, adaptive_const)
	# img_out = img_out[:,:,1]


	# img_out = cv2.Canny(img_in,100,200)
	return img_out

def detect_lane(img_in):
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

	## find left lane start to the left of center point
	for x in range(0,10):
		if not left_lane_found:
			left_search_height -= height_step
			left_start = int(max_x/2 + search_offset - x*10)
			while left_start > left_end - (x-1)*4:
				left_start -= search_step
				if img_in[left_search_height,left_start] == 0:
					img_out = cv2.circle(img_out, (left_start,left_search_height), radius = 1,color=(150, 0, 150), thickness=1)
					continue
				else:
					left_lane_found = True
					break
		else:
			break

	## find right lane start to the right of center point
	for x in range(0,10):
		if not right_lane_found:
			right_search_height -= height_step
			right_start = int(max_x/2 + search_offset + x*10)
			while right_start < right_end + (x-1)*4:
				right_start += search_step
				if img_in[right_search_height,right_start] == 0:
					img_out = cv2.circle(img_out, (right_start,right_search_height), radius = 1,color=(150, 150, 0), thickness=1)
					continue
				else:
					right_lane_found = True
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
		lane_points = []

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
					mask = np.zeros_like(img_in)
					cv2.line(mask, (x,y), (cx,cy), 255, 1)
					region_pixels = np.sum(mask == 255)
					white_pixels = np.sum(np.logical_and(mask == 255, img_in == 255))
					pixel_ratio = (white_pixels / region_pixels)
					valid_points.append((cx,cy,pixel_ratio))
				angle -= sweep_step

			# sort all potential connections and pick best choice
			if valid_points:
				sorted_points = sorted(valid_points, key=lambda tup: tup[2], reverse=True)
				optimum_point = (sorted_points[0][0], sorted_points[0][1])
				lane_points.append(optimum_point)
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
		lane_points = []

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
					mask = np.zeros_like(img_in)
					cv2.line(mask, (x,y), (cx,cy), 255, 1)
					region_pixels = np.sum(mask == 255)
					white_pixels = np.sum(np.logical_and(mask == 255, img_in == 255))
					pixel_ratio = (white_pixels / region_pixels)
					valid_points.append((cx,cy,pixel_ratio))
				angle -= sweep_step

			# sort all potential connections and pick best choice
			if valid_points:
				sorted_points = sorted(valid_points, key=lambda tup: tup[2], reverse=True)
				optimum_point = (sorted_points[0][0], sorted_points[0][1])
				lane_points.append(optimum_point)
				cv2.line(img_out, (x,y), (optimum_point[0], optimum_point[1]), (0, 0, 255), 3)
				heading = math.atan2((optimum_point[1]-y),(optimum_point[0]-x)) + math.radians(90)
				x = optimum_point[0]
				y = optimum_point[1]
			else:
				break
	
	# img_out = cv2.circle(img_out, (x,y), radius = 30,color=(0, 0, 255), thickness=1)
	return img_out

def birdy_view(img_in):
	cropped_height, cropped_width, _ = img_in.shape
	img_out = cv2.warpPerspective(img_in,bird_transform_matrix,(cropped_width,int(cropped_height*height_multiplier)),flags=(cv2.INTER_LINEAR))


	## for visualization purposes
	# img_out = cv2.polylines(img_in,np.int32([roi_in]),True,(255,0,0),2)
	return img_out

def round_up(img_in):
	_, img_out = cv2.threshold(img_in, 1, 255, cv2.THRESH_BINARY)
	return img_out

def main():
	print(filepath)

	## load image
	img = load_roi_img(filepath)
	img = resize_img(img,scale)

	## transform to BEV
	img = birdy_view(img)
	# img = filter_line(img)
	# img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	# img = detect_lane(img)

	## to occupancy grid
	# img = resize_img(img,1/6.8)
	# img = round_up(img)
	# img = resize_img(img,6.8)
	cv2.imshow('a',img)
	cv2.waitKey(0)

for x in range(1,131):
	filename = str(x) + ".jpeg"
	filepath = os.path.join(script_dir, "examples/z2", filename)
	main()