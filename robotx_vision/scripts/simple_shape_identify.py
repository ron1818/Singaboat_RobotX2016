import numpy as np
import cv2

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # gray = cv2.equalizeHist(gray)
    gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY,11,2)
    # mask = cv2.Canny(gray, 100, 200, L2gradient=True)
    ret, mask = cv2.threshold(gray,127,255,cv2.THRESH_BINARY_INV)
    # cv2.imshow("mask", mask)

    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # # morphological closing (fill small objects from the foreground)
    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    cv2.imshow("mask morp", mask)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # hsv range
    color_mask = cv2.inRange(hsv, np.array([0,90,90]), np.array([25,255,255]))\
        + cv2.inRange(hsv, np.array([175,90,90]), np.array([255,255,255]))

    kernel = np.ones((2, 2), np.uint8)
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
    # # morphological closing (fill small objects from the foreground)
    kernel = np.ones((2, 2), np.uint8)
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)
    cv2.imshow("color mask morp", color_mask)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
cap.release()
