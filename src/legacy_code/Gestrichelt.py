import numpy as np
import cv2
import os
import re
from tqdm import tqdm_notebook
import matplotlib.pyplot as plt

def GestrichelteLinie(img):
   
    Strich = False

    

    col_images=[]
    #img = cv2.imread("/home/nano/mechatroniklabor_ws/src/control/ATS/001.png")
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
    ret, thresh = cv2.threshold(img, 170, 145, cv2.THRESH_BINARY)
    
    # Gestrichelt -------------------------------------------------------------------------------------------
    
    # Hough Lines
    lines = cv2.HoughLinesP(thresh, 1, np.pi/180, 30, maxLineGap=1)      #Line Gap

    # create a copy of the original frame
    dmy = col_images[idx].copy()

    # draw Hough lines
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(dmy, (x1, y1), (x2, y2), (255, 0, 0), 3)             #Evtl Strichdicke und Strichmenge anpassen

    img1 = dmy
    
    # plot frame
    #cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/002.png",img1)

    
    
    
    # Durchgezogen Werte=ATS -------------------------------------------------------------------------------------------
    
    # Hough Lines
    lines2 = cv2.HoughLinesP(thresh, 1, np.pi/180, 30, maxLineGap=40)      #Line Gap: orignial 15

    # create a copy of the original frame
    dmy2 = col_images[idx].copy()

    # draw Hough lines
    if lines2 is not None:
        for line in lines2:
            x1, y1, x2, y2 = line[0]
            cv2.line(dmy2, (x1, y1), (x2, y2), (255, 0, 0), 3)             #Evtl Strichdicke und Strichmenge anpassen
       
    img2 = dmy2 
    #cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/003.png",img2)      
   
       
    #Vergleich---------------------------------------------------------------------------------------------   
       
       
    diff = img2.copy()
    cv2.absdiff(img2, img1, diff) 
    diff2=diff[320:360,320:630,0] 
  
    cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/004.png",diff) 
    cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/005.png",diff2)      
    print('diff size_:',np.sum(diff2[:,:]))
    
    if np.sum(diff2[:,:])>30000:
            Strich = True
            print('diff size_True:',np.sum(diff2[:,:]))
            cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/002.png",img1)
            cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/003.png",img2)  
            cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/004.png",diff) 
            cv2.imwrite("/home/nano/mechatroniklabor_ws/src/control/ATS/005.png",diff2) 
       
    return Strich
       
    
