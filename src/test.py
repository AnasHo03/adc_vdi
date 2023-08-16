import sys
import numpy as np
import pyzed.sl as sl
import cv2


# Create a ZED camera object
zed = sl.Camera()

# Set configuration parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720 # Use HD1080 video mode
init_params.camera_fps = 30 # Set fps at 30
# Open the camera
err = zed.open(init_params)
if (err != sl.ERROR_CODE.SUCCESS) :
    exit(-1)

image = sl.Mat()
if (zed.grab() == sl.ERROR_CODE.SUCCESS) :
    # A new image is available if grab() returns SUCCESS
    zed.retrieve_image(image, sl.VIEW.LEFT) # Get the left image
    # Use get_data() to get the numpy array
    image_ocv = image.get_data()

    # opencv_image = cv2.cvtColor(image_ocv, cv2.COLOR_RGB2BGR)

    # Display the left image from the numpy array
    cv2.imwrite("fuckingshit.jpg", image_ocv)


