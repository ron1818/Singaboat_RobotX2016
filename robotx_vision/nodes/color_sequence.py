#! /usr/bin/python

""" detect color sequence
ren ye 2016-10-21
reference:
http://stackoverflow.com/questions/14476683/identifying-color-sequence-in-opencv

algorithm:
    # image preparation
    1. subwindowing to light buoy by laser and camera
    2. convert to hsv
    3. check hue for the blob

    # detection
    1. wait until first detection is made
    2. wait until no detection is found for 2 seconds
    3. record color
    4. if color is different from previous frame, add to sequence
    5. if no detection, to step 2
    6. if sequence is length 3, report and end

"""

import numpy as np
import cv2

# read file
img_file = 'xxx.png'
img  = cv2.imread(img_file)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
gray = cv2.equalizeHist(gray)
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

# detect edge
edges = cv2.Canny(gray,50,150)
contours,hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

# color sequence
res = []
for cnt in contours:
    if cv2.contourArea(cnt) > 100:
        x,y,w,h = cv2.boundingRect(cnt)
        cx,cy = x+w/2, y+h/2
        color = hsv[cy,cx,0]

        if (color < 10 or color > 170):
            res.append([cx,cy,'R'])
        elif(50 < color < 70):
            res.append([cx,cy,'G'])
        elif(20 < color <40):
            res.append([cx,cy,'Y'])
        elif(110 < color < 130):
            res.append([cx,cy,'B'])

res = sorted(res,key = lambda res : res[0])
colors = [x[2] for x in res]
print colors
