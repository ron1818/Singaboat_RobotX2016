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

    def __init__(self, dev=0):
        self.dev = dev
        # self.col_under_det = col_under_det
        # self.detector = cv2.ORB(edgeThreshold=5)
        self.detector = cv2.SURF()

        # # calibrated and assigned by color name
        # if self.col_under_det == "red":
        #     self.lower, self.upper = self.lower_red1, self.upper_red1
        #     self.lower2, self.upper2 = self.lower_red1, self.upper_red1
        # elif self.col_under_det == "green":
        #     self.lower, self.upper = self.lower_green, self.upper_green
        # elif self.col_under_det == "blue":
        #     self.lower, self.upper = self.lower_blue, self.upper_blue
        # else:
        #     raise ValueError

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

    def streaming(self, masking_method, *args):
        if masking_method.lower() == "color_mask":
            masking = self.color_mask
            print masking
        elif masking_method.lower() == "canny_edge_mask":
            masking = self.canny_edge_mask
        else:
            masking = self.binary_mask
        # connect device
        self.connect_device()

        # deal for each frame
        while True:
            # read image
            ret, frame = self.cap.read()
            frame = cv2.GaussianBlur(frame, (5, 5), 0)
            mask = masking(frame, *args)
            x, y, w, h = self.shape_detect(frame, mask)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)

            cv2.imshow('mask', frame)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break

        # disconnect and destroy
        self.disconnect_device()

    def morphological(self, mask):
        # morphological openning (remove small objects from the foreground)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # # morphological closing (fill small objects from the foreground)
        kernel = np.ones((10, 10), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def color_mask(self, frame, color="red"):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if color.lower() == "red":
            # hsv range
            mask = cv2.inRange(hsv, self.lower_red1, self.upper_red1) + \
                cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        elif color.lower() == "blue":
            mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        elif color.lower() == "green":
            mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        else:
            mask = None
            return mask
            raise ValueError

        return self.morphological(mask)

    def binary_mask(self, frame, is_adaptive=False):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        if not is_adaptive:
            ret, mask = cv2.threshold(gray, 100, 200, cv2.THRESH_BINARY_INV)

        else:
            mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        # return self.morphological(mask)
        return mask

    def canny_edge_mask(self, frame, threshold1=100, threshold2=200):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        mask = cv2.Canny(gray, threshold1, threshold2, L2gradient=True)
        return mask

    def shape_detect(self, frame, mask):
        # find contours
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        area = list()
        for i, cnt in enumerate(contours):
            area.append(cv2.contourArea(cnt))

        max_idx = np.argmax(area)
        max_contour = contours[max_idx]
        approx = cv2.approxPolyDP(max_contour, 0.01 * cv2.arcLength(max_contour, True), True)
        # print len(approx)

        # find convex hull
        hull = cv2.convexHull(max_contour, returnPoints=False)
        defects = cv2.convexityDefects(max_contour, hull)

        #customized isconvex based on empirical
        defect_distance=list()
        for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0]
            defect_distance.append(d)

        if np.amax(defect_distance) < 1000:  # 1000 is empirical
            isconvex = True
        else:
            isconvex = False

        # shape determine
        if len(approx) > 10 and isconvex is True:
            print "circle"
        elif 7 >= len(approx) >=3:
            print "triangle"
        elif len(approx) > 8 and isconvex is False:
            print "cross"
        else:
            print "unkown"

        return cv2.boundingRect(approx)


if __name__ == "__main__":
    # cd = ColorDetection(dev=0, col_under_det="red")
    # cd = ColorDetection(dev=0, col_under_det="green")
    cd = ColorDetection(dev=0)
    # cd.hsv_calibartion()
    # cd.streaming("binary_mask", True)
    cd.streaming("color_mask", "red")
    # cd.streaming("canny_edge_mask")
