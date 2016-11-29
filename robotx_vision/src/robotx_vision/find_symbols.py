#! /usr/bin/python
"""https://pythonprogramming.net/haar-cascade-face-eye-detection-python-opencv-tutorial/"""
import numpy as np
import cv2

# multiple cascades: https://github.com/Itseez/opencv/tree/master/data/haarcascades

#https://github.com/Itseez/opencv/blob/master/data/haarcascades/haarcascade_frontalface_default.xml
circle_cascade = cv2.CascadeClassifier('cascade/circle.xml')
#https://github.com/Itseez/opencv/blob/master/data/haarcascades/haarcascade_eye.xml
cruciform_cascade = cv2.CascadeClassifier('cascade/cruciform.xml')

cap = cv2.VideoCapture(0)

while 1:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    circle = circle_cascade.detectMultiScale(gray, 1.1, 2)
    cruciform = cruciform_cascade.detectMultiScale(gray, 1.1, 2)

    for (x,y,w,h) in cruciform:
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]

    cv2.imshow('img',img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()

