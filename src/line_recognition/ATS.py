import numpy as np
import cv2
import os
import re
import matplotlib.pyplot as plt

# Load/show image
image_path = 'frame0038.jpg'  # Replace with the actual path to your image
image = cv2.imread(image_path)
#image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # Convert the image from BGR to RGB (Matplotlib expects RGB format)


# Filler variables
MAX_RAD = 30
servoW = 0
state = "both"


def LaneDetection(img,MAX_RAD,servoW,state):
   
    def get_hist(img):
        hist = np.sum(img[img.shape[0]//2:,:], axis=0)
        return hist

    servoNull=0 #ATS
    minSpeed=0.3 # Mindest Geschwindigkeit in Prozent



    col_images=[]
    #img = cv2.imread("/home/nano/mechatroniklabor_ws/src/control/ATS/test.jpg")
    col_images.append(img)

    # specify frame index
    idx = 0

    # create a zero array
    stencil = np.zeros_like(col_images[idx][:,:,0])

    # specify coordinates of the polygon
    polygon = np.array([[10,280], [10,360], [630,360], [630,280]])         #Hough Bereich

    # fill polygon with ones
    cv2.fillConvexPoly(stencil, polygon, 1)

    # apply polygon as a mask on the frame
    img = cv2.bitwise_and(col_images[idx][:,:,0], col_images[idx][:,:,0], mask=stencil)
    
    # apply image thresholding
    ret, thresh = cv2.threshold(img, 170, 145, cv2.THRESH_BINARY) #170,145
    
    ## ADDED CODE: visualization
    # plt.imshow(img)           # Display the image using Matplotlib
    # plt.axis('off')  # Turn off axes (optional)
    # plt.show()

    # Hough Lines
    lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 30, maxLineGap=15)      #Line Gap

    # create a copy of the original frame
    dmy = col_images[idx].copy()

    # draw Hough lines
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(dmy, (x1, y1), (x2, y2), (255, 0, 0), 3)             #Evtl Strichdicke und Strichmenge anpassen

    # plot frame
    # plt.figure(figsize=(10,10))
    # plt.show()
    # cv2.rectangle(dmy,(10,321),(630,309),(0,255,0),1)
    # print('bild drucken')
    #cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/17.png",dmy)

    # creating a object

    im = dmy  
    # Find starting position
    hist_tresh = 1000                                                       # Histogram Grenzwert
    histogram = get_hist(im[310:320,:,0])                                   # Grünen Bereich
    histogram[histogram<hist_tresh] = 0
    midpoint = int(histogram.shape[0]/2)
    if np.amax(histogram[:midpoint])<hist_tresh:
        failed_left = True
        leftx_base = 0
    else:
        leftx_base = histogram[:midpoint].nonzero()[0][-2]
    if np.amax(histogram[midpoint:])<hist_tresh:
        failed_right = True
        rightx_base = 2*midpoint
    else:
        rightx_base = histogram[midpoint:].nonzero()[0][0] + midpoint


    a = midpoint-leftx_base # Abstand Linke Fahrbahn
    b = rightx_base-midpoint # Abstand Rechte Fahrbahn





    # Visualization
    visualization_img = im.copy()

    # Visualize the green area
    visualization_img[310:320, :, 1] = 255  # Set green channel to maximum in the region of interest

    # Visualize 'a' and 'b'
    midpoint_x = visualization_img.shape[1] // 2
    midpoint_y = 315  # Adjust the y-coordinate as desired for visualization

    left_lane_x = midpoint_x - a
    right_lane_x = midpoint_x + b

    # Draw lines for 'a' and 'b'
    visualization_img = cv2.line(visualization_img, (midpoint_x, midpoint_y), (left_lane_x, midpoint_y), (0, 0, 255), 2)
    visualization_img = cv2.line(visualization_img, (midpoint_x, midpoint_y), (right_lane_x, midpoint_y), (0, 0, 255), 10)

    # Show the visualization
    plt.imshow(cv2.cvtColor(visualization_img, cv2.COLOR_BGR2RGB))
    plt.axis('off')
    plt.show()













    
    # Statusentscheidung
    if not(a==midpoint) and not(b==midpoint) and (a+b)>200:                               # a+b Grenzwert   
        state="both"
    elif a==midpoint and not(b==midpoint) and (state=="both" or state=="right") and (a+b)>200:
        state="right"
    elif not(a==midpoint) and b==midpoint and (state=="both" or state=="left") and (a+b)>200:
        state="left"
    else:
        state="none"


    #Lenkwinkel berechnen
    
    if not(state=="none"):
        if a>b:
            servoW=servoNull-2*(100*(a/(a+b))-50)
        elif a<b:
            servoW=servoNull+2*(100*(b/(a+b))-50)
        else:
            servoW=0
          
        servoW=servoW*0.5
            
        #if servoW > 40 and not (state==2):
        #    servoW=100
        #    state = 1 #Rechts  
        #if servoW < -40 and not (state==1):
        #    servoW=-100 
        #    state = 2 #Links  
    else:
        #servoW=servoW*100/MAX_RAD
        #if servoW<0:
        #    servoW=-75
        #if servoW>0:
        #    servoW=75




        servoW=servoW*100/MAX_RAD
        if servoW<0:
            servoW=servoW-5
        if servoW>0:
            servoW=servoW+5

        if servoW<-90:
            servoW=-90
        if servoW>90:
            servoW=90






    minSpeed=0.3 # Mindest Geschwindigkeit in Prozent         
    speed=100-(np.absolute(servoW)*(1-minSpeed))




    servoW = MAX_RAD * servoW/100
       
    return servoW, state
       
    
servoW, state = LaneDetection(image, MAX_RAD, servoW, state)
print("servoW:", servoW)
print("state:", state)
