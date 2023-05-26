# -*- coding: utf-8 -*-
"""
Created on Tue Mar  8 09:11:28 2022

@author: alexh
"""
#-----------------------------------------------------------
# Imports
#-----------------------------------------------------------
import numpy as np
import cv2
import matplotlib.pyplot as plt

#-----------------------------------------------------------
# Function
#-----------------------------------------------------------

def detectTrafficLight(img,decision):
	# variables
	size = img.shape
	font = cv2.FONT_HERSHEY_SIMPLEX
	cimg = img
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)		# convert to hsv space
	# select color to detect
	if decision==False:			# detect red traffic light
		# rgb color range
		lower = np.array([50,50,200])		#bgr
		upper = np.array([255,255,255])		#bgr
		# hsv color range
		lower_1 = np.array([0,150,200])
		upper_1 = np.array([15,255,255])	# 30 155 255
		lower_2 = np.array([165,150,200])	# 160 50 200
		upper_2 = np.array([180,255,255])	# 180 155 255
		# constants
		color_label = 'RED'
		thresh = 90
		# create masks
		mask_rgb = cv2.inRange(img, lower, upper)		# rgb mask
		mask1 = cv2.inRange(hsv, lower_1, upper_1)
		mask2 = cv2.inRange(hsv, lower_2, upper_2)
		mask_hsv = cv2.add(mask1, mask2)				# hsv mask
	elif decision==True:		# detect green traffic light
		# rgb color range
		lower = np.array([100,200,100])		#bgr
		upper = np.array([255,255,255])		#bgr
		# hsv color range
		lower_1 = np.array([40,150,200])		# 40,50,150
		upper_1 = np.array([90,255,255])	# 0,255,255
		# constants
		color_label = 'GREEN'
		thresh = 30
		# create masks
		mask_rgb = cv2.inRange(img, lower, upper)		# rgb mask
		mask_hsv = cv2.inRange(hsv, lower_1, upper_1)	# hsv mask
    # hough circle detection in hsv threshold image
	circles = cv2.HoughCircles(mask_hsv, cv2.HOUGH_GRADIENT, 1, 80, param1=50, param2=10, minRadius=0, maxRadius=50)
    # constants
	num = 0		# counter for relevant circles
	radius = 0	# radius of relevant circle
	# iterate over detected circles
	if circles is not None:
		circles = np.uint16(np.around(circles))        
		for i in circles[0, :]:
			if i[0] > size[1] or i[1] > size[0]:	# Check circle center point
				continue
			h, s = 0.0, 0.0		# value & amount of pixels included
			r = int(i[2])		# current radius
			# check average over region in rgb threshold image 
			for m in range(-r, r):
				for n in range(-r, r):
					if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
						continue
					h += mask_rgb[i[1]+m, i[0]+n]
					s += 1
			#print('h/s',h/s)
			# Check if average in center point region high enough
			if h / s > thresh:
				cv2.circle(cimg, (i[0], i[1]), i[2]+1, (0, 0, 0), 2)
				cv2.circle(mask_hsv, (i[0], i[1]), i[2]+3, (255, 255, 255), 2)
				cv2.circle(mask_rgb, (i[0], i[1]), i[2]+3, (255, 255, 255), 2)
				cv2.putText(cimg,color_label,(i[0], i[1]), font, 1,(0,0,0),2,cv2.LINE_AA)
				num += 1
				# terminate after threshold (1) amount of accepted cirles
				if num == 1:
					radius = i[2]
					xpos = i[0]
					break
	# Check radius and x position in image
	if radius > 1 and xpos >320:
		decision = not decision
		print('decision:',decision)
		print('Xpos:',xpos)
	#return decision , radius, mask_hsv, mask_rgb, cimg
	#cv2.imwrite("RoterPunkt.png",cimg)
	return decision