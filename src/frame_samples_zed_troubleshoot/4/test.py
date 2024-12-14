#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2 as cv
import pyzed.sl as sl




# Create a ZED camera object
zed = sl.Camera()

# Set configuration parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720 # Use HD1080 video mode
init_params.camera_fps = 30 # Set fps at 30
#init_params.camera_auto_exposure_gain = True

# Open the camera
err = zed.open(init_params)
if (err != sl.ERROR_CODE.SUCCESS) :
    print('Zed exited script!')
    exit(-1)

cap = cv.VideoCapture(0)



	
left_image = sl.Mat()
right_image = sl.Mat()

if (zed.grab() == sl.ERROR_CODE.SUCCESS) :            
	zed.retrieve_image(left_image, sl.VIEW.LEFT) # Get the left image    
	zed.retrieve_image(right_image, sl.VIEW.RIGHT) # Get the left image    
	left_image_np = left_image.get_data()
        right_image_np = right_image.get_data()
        
        frame = np.hstack((left_image_np, )) 

plt.ion()
fig, ax = plt.subplots()

while True : 
	frame = cv.cvtColor(frame,cv.COLOR_RGB2BGR)
	
	ax.imshow(frame)
	plt.draw()
	plt.pause(0.01)
	ax.clear()
	
	if plt.waitforbuttonpress(1):
		break

cap.release()
plt.close()





