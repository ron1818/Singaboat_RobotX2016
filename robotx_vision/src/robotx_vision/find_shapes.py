# import the necessary packages
import numpy as np
# import argparse
import cv2

lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])
lower_blue = np.array([50, 50, 50])
upper_blue = np.array([130, 255, 255])
lower_green = np.array([30, 50, 45])
upper_green = np.array([100, 147, 255])
# load the image
# image = cv2.imread("image/finding_shapes_example.png")
# image = cv2.imread("image/docking.jpg")
# image = cv2.imread("image/blue_cross.png")
image = cv2.imread("image/t1.png")
imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
red_mask = cv2.inRange(hsv, lower_red, upper_red)
green_mask = cv2.inRange(hsv, lower_green, upper_green)
blue_res = cv2.bitwise_and(image, image, mask=blue_mask)
red_res = cv2.bitwise_and(image, image, mask=red_mask)
green_res = cv2.bitwise_and(image, image, mask=green_mask)

im_blue, contours_blue, hierarchy = cv2.findContours(blue_mask.copy(),
                                                     cv2.RETR_TREE,
                                                     cv2.CHAIN_APPROX_NONE)
im_green, contours_green, hierarchy = cv2.findContours(green_mask.copy(),
                                                       cv2.RETR_TREE,
                                                       cv2.CHAIN_APPROX_NONE)
im_red, contours_red, hierarchy = cv2.findContours(red_mask.copy(),
                                                   cv2.RETR_TREE,
                                                   cv2.CHAIN_APPROX_NONE)

edges = cv2.Canny(image, 100, 200)
im_edge, contours_edge, hierarchy = cv2.findContours(edges.copy(),
                                                   cv2.RETR_TREE,
                                                   cv2.CHAIN_APPROX_NONE)
cv2.drawContours(imgray, contours_edge, 0, (0,255,0), 3)
counter = 0
for cnt in contours_edge:
    approx = cv2.approxPolyDP(cnt,0.05*cv2.arcLength(cnt,True),True)
    print len(approx)
    # if len(approx) == 3:
    #     cv2.drawContours(imgray, cnt, counter, (0,255,0), 3)
    # counter += 1

cv2.imshow('image', image)
cv2.imshow('edges', edges)
cv2.imshow('contours', imgray)
# cv2.imshow('blue_res', blue_res)
# cv2.imshow('green_res', green_res)
# cv2.imshow('red_res', red_res)
# cv2.imshow("im_blue", im_blue)
# cv2.imshow("im_red", im_red)
# cv2.imshow("im_green", im_green)
cv2.waitKey(0)

cv2.destroyAllWindows()
