#! /usr/bin/python
"""modified from:
    https://pythonprogramming.net/haar-cascade-face-i\
            eye-detection-python-opencv-tutorial/"""

# import numpy as np
import cv2

# trained cascades for triangles from imagenet
triangle_cascade = \
    cv2.CascadeClassifier('cascade/haarcascade_triangle_imagenet.xml')
# future circle and cross cascades

# capture from video
cap = cv2.VideoCapture(0)

while True:
    # read frame
    ret, img = cap.read()
    # convert to gray, future need to have color treshold
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # apply cascade, future need parameter tuning
    # future check single detection
    triangles = triangle_cascade.detectMultiScale(gray,

    cv2.imshow('img', img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
