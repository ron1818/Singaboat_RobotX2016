# written in opencv3, need modification for 14.04 which is opencv2
# import the necessary packages
import numpy as np
# import argparse
import cv2


class ColorDetection(object):
    """ detect object based on shape and color,
    for Robotx challenge"""

    # calibrated by palette
    lower_red1 = np.array([0, 90, 90])
    upper_red1 = np.array([25, 255, 255])
    lower_red2 = np.array([175, 90, 90])
    upper_red2 = np.array([255, 255, 255])
    lower_blue = np.array([85, 90, 90])
    upper_blue = np.array([130, 255, 255])
    lower_green = np.array([25, 50, 75])
    upper_green = np.array([75, 255, 255])

    def __init__(self, dev=0, col_under_det="red"):
        self.dev = dev
        self.col_under_det = col_under_det

    def bgr_hsv_cvt(self):
        self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

    def connect_device(self):
        self.cap = cv2.VideoCapture(self.dev)

    def disconnect_device(self):
        cv2.destroyAllWindows()
        self.cap.release()

    def nothing(self):
        pass

    def hsv_calibartion(self):
        cv2.namedWindow('calibrate_res')
        cv2.createTrackbar('Hl', 'calibrate_res', 0, 255, self.nothing)
        cv2.createTrackbar('Hu', 'calibrate_res', 0, 255, self.nothing)
        cv2.createTrackbar('Sl', 'calibrate_res', 0, 255, self.nothing)
        cv2.createTrackbar('Su', 'calibrate_res', 0, 255, self.nothing)
        cv2.createTrackbar('Vl', 'calibrate_res', 0, 255, self.nothing)
        cv2.createTrackbar('Vu', 'calibrate_res', 0, 255, self.nothing)

        # connect device
        self.connect_device()

        # deal for each frame
        while True:
            ret, self.image = self.cap.read()
            self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
            cv2.imshow('image', self.image)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break

            calibrate_mask = cv2.inRange(self.hsv,
                                         np.array([cv2.getTrackbarPos('Hl', 'calibrate_res'),
                                                  cv2.getTrackbarPos('Sl', 'calibrate_res'),
                                                  cv2.getTrackbarPos('Vl', 'calibrate_res')]),
                                         np.array([cv2.getTrackbarPos('Hu', 'calibrate_res'),
                                                  cv2.getTrackbarPos('Su', 'calibrate_res'),
                                                  cv2.getTrackbarPos('Vu', 'calibrate_res')]))

            calibrate_res = cv2.bitwise_and(self.image, self.image, mask=calibrate_mask)
            cv2.imshow('calibrate_res', calibrate_res)

        # disconnect and destroy
        self.disconnect_device()

    def color_inrange(self):
        if self.col_under_det == "red":
            lower, upper = self.lower_red1, self.upper_red1
            lower2, upper2 = self.lower_red1, self.upper_red1
        elif self.col_under_det == "green":
            lower, upper = self.lower_green, self.upper_green
        elif self.col_under_det == "blue":
            lower, upper = self.lower_blue, self.upper_blue
        else:
            raise ValueError

        # connect device
        self.connect_device()

        # deal for each frame
        while True:
            ret, self.image = self.cap.read()
            self.hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

            if self.col_under_det == "red":
                # hsv range
                color_mask = cv2.inRange(self.hsv, lower, upper) + \
                    cv2.inRange(self.hsv, lower2, upper2)
            else:
                color_mask = cv2.inRange(self.hsv, lower, upper)

            # morphological openning (remove small objects from the foreground)
            kernel = np.ones((5, 5), np.uint8)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)
            # # morphological closing (fill small objects from the foreground)
            kernel = np.ones((10, 10), np.uint8)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)
            cv2.imshow('mask', color_mask)
            res = cv2.bitwise_and(self.image, self.image, mask=color_mask)

            _, contours, hierarchy = cv2.findContours(color_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # cv2.drawContours(self.image, contours, -1, (0, 255, 0), 3)

            # area = list()
            # for i, cnt in enumerate(contours):
            #     area.append(cv2.contourArea(cnt))
            # print area

            for i, cnt in enumerate(contours):
                cv2.drawContours(self.image, cnt, -1, (0, 255, 0), 3)
                approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
                if len(approx) > 15:
                    print "circle"
                elif len(approx) == 3:
                    print "triangle"
                elif len(approx) > 4 and cv2.isContourConvex(cnt):
                    print "cross"

                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(self.image, (x, y), (x+w, y+h), (0, 255, 0), 3)

            # cv2.imshow('res', res_bw)
            cv2.imshow('image', self.image)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break

        # disconnect and destroy
        self.disconnect_device()


if __name__ == "__main__":
    # cd = ColorDetection(dev=0, col_under_det="red")
    # cd = ColorDetection(dev=0, col_under_det="green")
    cd = ColorDetection(dev=1, col_under_det="blue")
    # cd.hsv_calibartion()
    cd.color_inrange()
