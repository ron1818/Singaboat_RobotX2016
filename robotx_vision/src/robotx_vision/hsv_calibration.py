#! /usr/bin/python

import numpy as np
import cv2

def nothing():
    pass

def hsv_calibartion():
    cv2.namedWindow('calibrate_res')
    cv2.createTrackbar('Hl', 'calibrate_res', 0, 255, nothing)
    cv2.createTrackbar('Hu', 'calibrate_res', 0, 255, nothing)
    cv2.createTrackbar('Sl', 'calibrate_res', 0, 255, nothing)
    cv2.createTrackbar('Su', 'calibrate_res', 0, 255, nothing)
    cv2.createTrackbar('Vl', 'calibrate_res', 0, 255, nothing)
    cv2.createTrackbar('Vu', 'calibrate_res', 0, 255, nothing)

    # connect device
    cap = cv2.VideoCapture(0)

    # deal for each frame
    while True:
        ret, image = cap.read()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        cv2.imshow('image', image)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

        calibrate_mask = cv2.inRange(hsv,
                                     np.array([cv2.getTrackbarPos('Hl', 'calibrate_res'),
                                              cv2.getTrackbarPos('Sl', 'calibrate_res'),
                                              cv2.getTrackbarPos('Vl', 'calibrate_res')]),
                                     np.array([cv2.getTrackbarPos('Hu', 'calibrate_res'),
                                              cv2.getTrackbarPos('Su', 'calibrate_res'),
                                              cv2.getTrackbarPos('Vu', 'calibrate_res')]))

        calibrate_res = cv2.bitwise_and(image, image, mask=calibrate_mask)
        cv2.imshow('calibrate_res', calibrate_res)

    # disconnect and destroy
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    hsv_calibration()
