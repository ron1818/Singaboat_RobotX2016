#!/usr/bin/env python

""" symbol_tracker.py - Version 1.1 2013-12-20

    Modification of the ROS OpenCV Camshift example
    Addition with template tracker to track shapes
    using cv_bridge and publishing the ROI
    coordinates to the /roi topic.
"""

import rospy
import cv2
from cv2 import cv as cv
from robotx_vision.ros2opencv2 import ROS2OpenCV2
from robotx_vision.find_mask import Masking  # feature matching
from camshift_color import CamShiftColor
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np

class SymbolTracker(CamShiftColor):
    # taken from robotx_vision.find_shapes.Color_Detection

    def __init__(self, node_name):
        CamShiftColor.__init__(self, node_name)

        self.shape = rospy.get_param("~shape", "circle")
        self.masker = rospy.get_param("~masker", "canny")
        self.detector = rospy.get_param("~detector", "orb")
        self.matcher = rospy.get_param("~matcher", "flann")
        self.matching_method = rospy.get_param("~matching_method", "one")
        # call masking alglorthm to get the color mask
        self.mymask = Masking(color=self.color_under_detect,
                              shape=self.shape,
                              # how to solve the absolute path problem?
                              shape_path="/home/ry1404/catkin_ws/src/Singaboat_Robotx2016/robotx_vision/src/robotx_vision/",
                              masker=self.masker,
                              detector=self.detector,
                              matcher=self.matcher,
                              matching_method=self.matching_method)

        print self.mymask.target
        # features from template
        if self.mymask.detector is not None:
            self.kp_r, self.des_r = self.mymask.detector.detectAndCompute(self.mymask.target, None)

    # The main processing function computes the histogram and backprojection
    def process_image(self, cv_image):

        try:
            # Blur the image
            frame = cv2.blur(cv_image, (5, 5))

            # Convert from RGB to HSV space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create a mask using the current saturation and value parameters
            mask = cv2.inRange(hsv, np.array((0., self.smin, self.vmin)), np.array((180., 255., self.vmax)))

            # not select any region, do automatic color rectangle
            if self.selection is None:
                # obtain the color mask
                color_mask = self.mymask.color_mask(frame, self.color_under_detect)
                # create bounding boxes from the masks
                self.selection = self.mymask.find_max_contour(color_mask)
                self.detect_box = self.selection
                self.track_box = None

            # If region is selected,
            # calculate a new histogram to track
            if self.selection is not None:
                x0, y0, w, h = self.selection
                x1 = x0 + w
                y1 = y0 + h
                self.track_window = (x0, y0, x1, y1)
                hsv_roi = hsv[y0:y1, x0:x1]
                mask_roi = mask[y0:y1, x0:x1]
                self.hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
                cv2.normalize(self.hist, self.hist, 0, 255, cv2.NORM_MINMAX);
                self.hist = self.hist.reshape(-1)
                self.show_hist()

            # If we have a detection, clear out the selection
            if self.detect_box is not None:
                self.selection = None

            # If we have a histogram, track it with CamShift
            if self.hist is not None:
                # print "track window"
                # Compute the backprojection from the histogram
                backproject = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)

                # Mask the backprojection with the mask created earlier
                backproject &= mask

                # Threshold the backprojection
                ret, backproject = cv2.threshold(backproject, self.threshold, 255, cv.CV_THRESH_TOZERO)

                x, y, w, h = self.track_window
                if self.track_window is None or w <= 0 or h <=0:
                    self.track_window = 0, 0, self.frame_width - 1, self.frame_height - 1

                # Set the criteria for the CamShift algorithm
                term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

                # Run the CamShift algorithm
                self.track_box, self.track_window = cv2.CamShift(backproject, self.track_window, term_crit)

                # Display the resulting backprojection
                cv2.imshow("Backproject", backproject)

            # If we have a track_box,
            # check if it has the shape we want
            if self.track_window is not None:

                D = 10
                x, y, w, h = self.track_window
                x0 = max(0, x - D - 1)
                y0 = max(0, y - D - 1)
                x1 = min(x + w + D, self.frame_width - 1)
                t1 = min(y + h + D, self.frame_width - 1)
                cropped_image = cv_image[y0:y1, x0:x1]
                cropped_gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
                cropped_gray = cv2.equalizeHist(cropped_gray)
                cv2.imshow("Tracked_obj", cropped_gray)
                # features from template
                if self.mymask.detector is not None:
                    kp_r, des_r = self.mymask.detector.detectAndCompute(self.target, None)
                # features from target
                kp_o, des_o = self.mymask.detector.detectAndCompute(cropped_gray,None)

                matches = self.mymask.matching_method(self.des_r, des_o, k=2)
                print matches




        except:
            pass

        return cv_image

    def show_hist(self):
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i*bin_w+2, 255), ((i+1)*bin_w-2, 255-h), (int(180.0*i/bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('Histogram', img)


    def hue_histogram_as_image(self, hist):
            """ Returns a nice representation of a hue histogram """
            histimg_hsv = cv.CreateImage((320, 200), 8, 3)

            mybins = cv.CloneMatND(hist.bins)
            cv.Log(mybins, mybins)
            (_, hi, _, _) = cv.MinMaxLoc(mybins)
            cv.ConvertScale(mybins, mybins, 255. / hi)

            w,h = cv.GetSize(histimg_hsv)
            hdims = cv.GetDims(mybins)[0]
            for x in range(w):
                xh = (180 * x) / (w - 1)  # hue sweeps from 0-180 across the image
                val = int(mybins[int(hdims * x / w)] * h / 255)
                cv2.rectangle(histimg_hsv, (x, 0), (x, h-val), (xh,255,64), -1)
                cv2.rectangle(histimg_hsv, (x, h-val), (x, h), (xh,255,255), -1)

            histimg = cv2.cvtColor(histimg_hsv, cv.CV_HSV2BGR)

            return histimg


if __name__ == '__main__':
    try:
        node_name = "camshift"
        SymbolTracker(node_name)
        try:
            rospy.init_node(node_name)
        except:
            pass
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

