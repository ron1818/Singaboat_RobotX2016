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

#!/usr/bin/env python

""" camshift_color.py - Version 1.1 2013-12-20

    Modification of the ROS OpenCV Camshift example using cv_bridge and publishing the ROI
    coordinates to the /roi topic.
"""

import time
import rospy
import cv2
from cv2 import cv as cv
from robotx_vision.ros2opencv2 import ROS2OpenCV2
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
import numpy as np

class ColorSequence(ROS2OpenCV2):
    # taken from robotx_vision.find_shapes.Color_Detection

    x0, y0 = 0, 0

    def __init__(self, node_name):
        ROS2OpenCV2.__init__(self, node_name)
        self.sequence_pub = rospy.Publisher("color_sequence", Vector3, queue_size=10)
        self.odom_received = False
        rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=50)
        while not self.odom_received:
            pass

        rospy.Subscriber("odometry/filtered/global", Odometry, self.odom_callback, queue_size=50)

        self.node_name = node_name
        # The minimum saturation of the tracked color in HSV space,
        # as well as the min and max value (the V in HSV) and a
        # threshold on the backprojection probability image.
        self.smin = rospy.get_param("~smin", 85)
        self.vmin = rospy.get_param("~vmin", 50)
        self.vmax = rospy.get_param("~vmax", 254)
        self.threshold = rospy.get_param("~threshold", 50)


        # all done in ros2opencv2.py:
        # self.depth_sub, self.depth_callback, self.depth_image
        # self.depth_image can be used globally
        # self.depth_sub = rospy.Subscriber("input_depth_image", Image, self.depth_callback, queue_size=1)

        # Create a number of windows for displaying the histogram,
        # parameters controls, and backprojection image
        cv.NamedWindow("Histogram", cv.CV_WINDOW_NORMAL)
        cv.MoveWindow("Histogram", 300, 50)
        cv.NamedWindow("Parameters", 0)
        cv.MoveWindow("Parameters", 700, 50)
        cv.NamedWindow("Backproject", 0)
        cv.MoveWindow("Backproject", 700, 325)
        # cv.NamedWindow("Tracked_obj", 0)
        # cv.MoveWindow("Tracked_obj", 700, 900)

        # Create the slider controls for saturation, value and threshold
        cv.CreateTrackbar("Saturation", "Parameters", self.smin, 255, self.set_smin)
        cv.CreateTrackbar("Min Value", "Parameters", self.vmin, 255, self.set_vmin)
        cv.CreateTrackbar("Max Value", "Parameters", self.vmax, 255, self.set_vmax)
        cv.CreateTrackbar("Threshold", "Parameters", self.threshold, 255, self.set_threshold)

        # Initialize a number of variables
        self.hist = None
        self.track_window = None
        self.show_backproj = False

    # These are the callbacks for the slider controls
    def set_smin(self, pos):
        self.smin = pos

    def set_vmin(self, pos):
        self.vmin = pos

    def set_vmax(self, pos):
       self.vmax = pos

    def set_threshold(self, pos):
        self.threshold = pos


    # def color_masking(self, frame):
    #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #     mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange) + \
    #             cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
    #     return mask

    def depth_masking(self):
        self.depth_array = np.array(self.depth_image, dtype=np.float32)
        # self.depth_image
        depth_mask = np.zeros((self.frame_height, self.frame_width))

        for x in range(self.frame_height):
            for y in range(self.frame_width):
                try:
                    # Get a depth value in meters
                    z = self.depth_array[y, x]

                    # Check for NaN values returned by the camera driver
                    if isnan(z):
                        continue

                except:
                    # It seems to work best if we convert exceptions to big value
                    z = 255

                if z < self.depth_threshold:
                    depth_mask[y, x] = 255  # white
                else:
                    depth_mask[y, x] = 0
        return depth_mask

    def find_max_contour(self, mask):
        # find contours
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # for multiple contours, find the maximum
        area=list()
        approx=list()
        for i, cnt in enumerate(contours):
            approx.append(cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True))
            area.append(cv2.contourArea(cnt))
        # overwrite selection box by automatic color matching
        return cv2.boundingRect(approx[np.argmax(area)])

    def find_contours(self, mask):
        # find contours
        mask = self.morphological(mask)
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # for multiple contours, find the maximum
        area=list()
        approx=list()
        for i, cnt in enumerate(contours):
            approx.append(cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True))
            area.append(cv2.contourArea(cnt))
        # overwrite selection box by automatic color matching

        self.area_ratio = np.sum(area) / (self.frame_width * self.frame_height)
        if np.max(area) / np.sum(area) > 0.95:
            # print "one blob"
            self.number_blob = 1
        else:
            # print "more than one blobs"
            self.number_blob = 2

        if len(area) > 1:  # more than one blob, find the ratio of the 1st and 2nd largest
            area_rev_sorted = np.sort(area)[::-1]
            self.area_ratio = area_rev_sorted[0] / area_rev_sorted[1]
        else:  # only one blob found
            self.area_ratio = 0

        print self.area_ratio


    def morphological(self, mask):
        """ tune the mask """
        # morphological openning (remove small objects from the foreground)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # # morphological closing (fill small objects from the foreground)
        kernel = np.ones((10, 10), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask


    # The main processing function computes the histogram and backprojection
    def process_image(self, cv_image):

        try:
            # First blur the image
            frame = cv2.blur(cv_image, (5, 5))

            # Convert from RGB to HSV space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create a mask using the current saturation and value parameters
            mask = cv2.inRange(hsv, np.array((0., self.smin, self.vmin)), np.array((180., 255., self.vmax)))

            # not select any region, do automatic color rectangle
            if self.selection is None:
                # obtain the color mask
                depth_mask = self.depth_masking()
                # print color_mask
                # create bounding box from the maximum mask
                self.selection = self.find_max_contour(depth_mask)
                self.detect_box = self.selection
                self.track_box = None

            # If the user is making a selection with the mouse,
            # calculate a new histogram to track
            if self.selection is not None:
                x0, y0, w, h = self.selection
                x1 = x0 + w
                y1 = y0 + h
                self.track_window = (x0, y0, x1, y1)
                hsv_roi = hsv[y0:y1, x0:x1]
                mask_roi = mask[y0:y1, x0:x1]
                self.hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
                cv2.normalize(self.hist, self.hist, 0, 255, cv2.NORM_MINMAX)
                self.hist = self.hist.reshape(-1)
                print self.hist
                self.hist_prob = np.argmax(self.hist)
                self.show_hist()

            # if self.detect_box is not None:
            #     self.selection = None

            # # If we have a histogram, track it with CamShift
            # if self.hist is not None:
            #     # Compute the backprojection from the histogram
            #     backproject = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)

            #     # Mask the backprojection with the mask created earlier
            #     backproject &= mask

            #     # Threshold the backprojection
            #     ret, backproject = cv2.threshold(backproject, self.threshold, 255, cv.CV_THRESH_TOZERO)

            #     # self.find_contours(backproject)
            #     # Detect blobs.
            #     # keypoints = self.blob_detector.detect(backproject)
            #     # print keypoints

            #     x, y, w, h = self.track_window
            #     if self.track_window is None or w <= 0 or h <=0:
            #         self.track_window = 0, 0, self.frame_width - 1, self.frame_height - 1

            #     # Set the criteria for the CamShift algorithm
            #     term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

            #     # Run the CamShift algorithm
            #     self.track_box, self.track_window = cv2.CamShift(backproject, self.track_window, term_crit)
            #     x0, y0, x1, y1 = self.track_window
            #     # print self.track_window

            #     # Display the resulting backprojection
            #     cv2.imshow("Backproject", backproject)

        except:
            pass

        return cv_image

    def show_hist(self):
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count*bin_w, 3), np.uint8)
        print np.argmax(self.hist)
        self.hist_prob = np.argmax(self.hist)
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

    def odom_callback(self, msg):
        """ call back to subscribe, get odometry data:
        pose and orientation of the current boat,
        suffix 0 is for origin """
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        self.odom_received = True

    def image_callback(self, data):
        # Store the image header in a global variable
        self.image_header = data.header

        # Time this loop to get cycles per second
        start = time.time()

        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        frame = self.convert_image(data)

        # Some webcams invert the image
        if self.flip_image:
            frame = cv2.flip(frame, 0)

        # Store the frame width and height in a pair of global variables
        if self.frame_width is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size

        # Create the marker image we will use for display purposes
        if self.marker_image is None:
            self.marker_image = np.zeros_like(frame)

        # Copy the current frame to the global image in case we need it elsewhere
        self.frame = frame.copy()

        # Reset the marker image if we're not displaying the history
        if not self.keep_marker_history:
            self.marker_image = np.zeros_like(self.marker_image)

        # Process the image to detect and track objects or features
        processed_image = self.process_image(frame)

        # If the result is a greyscale image, convert to 3-channel for display purposes """
        #if processed_image.channels == 1:
            #cv.CvtColor(processed_image, self.processed_image, cv.CV_GRAY2BGR)
        #else:

        # Make a global copy
        self.processed_image = processed_image.copy()

        # Display the user-selection rectangle or point
        self.display_selection()

        # Night mode: only display the markers
        if self.night_mode:
            self.processed_image = np.zeros_like(self.processed_image)

        # Merge the processed image and the marker image
        self.display_image = cv2.bitwise_or(self.processed_image, self.marker_image)

        # If we have a track box, then display it.  The track box can be either a regular
        # cvRect (x,y,w,h) or a rotated Rect (center, size, angle).
        if self.show_boxes:
            if self.track_box is not None and self.is_rect_nonzero(self.track_box):
                if len(self.track_box) == 4:
                    x,y,w,h = self.track_box
                    size = (w, h)
                    center = (x + w / 2, y + h / 2)
                    angle = 0
                    self.track_box = (center, size, angle)
                else:
                    (center, size, angle) = self.track_box

                # For face tracking, an upright rectangle looks best
                if self.face_tracking:
                    pt1 = (int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
                    pt2 = (int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
                    cv2.rectangle(self.display_image, pt1, pt2, cv.RGB(50, 255, 50), self.feature_size, 8, 0)
                else:
                    # Otherwise, display a rotated rectangle
                    vertices = np.int0(cv2.cv.BoxPoints(self.track_box))
                    cv2.drawContours(self.display_image, [vertices], 0, cv.RGB(50, 255, 50), self.feature_size)

            # If we don't yet have a track box, display the detect box if present
            elif self.detect_box is not None and self.is_rect_nonzero(self.detect_box):
                (pt1_x, pt1_y, w, h) = self.detect_box
                if self.show_boxes:
                    cv2.rectangle(self.display_image, (pt1_x, pt1_y), (pt1_x + w, pt1_y + h), cv.RGB(50, 255, 50), self.feature_size, 8, 0)

        # Publish the ROI
        self.publish_roi()
        self.publish_sequence()

        # Compute the time for this loop and estimate CPS as a running average
        end = time.time()
        duration = end - start
        fps = int(1.0 / duration)
        self.cps_values.append(fps)
        if len(self.cps_values) > self.cps_n_values:
            self.cps_values.pop(0)
        self.cps = int(sum(self.cps_values) / len(self.cps_values))

        # Display CPS and image resolution if asked to
        if self.show_text:
            font_face = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5

            """ Print cycles per second (CPS) and resolution (RES) at top of the image """
            if self.frame_size[0] >= 640:
                vstart = 25
                voffset = int(50 + self.frame_size[1] / 120.)
            elif self.frame_size[0] == 320:
                vstart = 15
                voffset = int(35 + self.frame_size[1] / 120.)
            else:
                vstart = 10
                voffset = int(20 + self.frame_size[1] / 120.)
            cv2.putText(self.display_image, "CPS: " + str(self.cps), (10, vstart), font_face, font_scale, cv.RGB(255, 255, 0))
            cv2.putText(self.display_image, "RES: " + str(self.frame_size[0]) + "X" + str(self.frame_size[1]), (10, voffset), font_face, font_scale, cv.RGB(255, 255, 0))

        # Update the image display
        cv2.imshow(self.node_name, self.display_image)

        # Process any keyboard commands
        self.keystroke = cv2.waitKey(5)
        if self.keystroke is not None and self.keystroke != -1:
            try:
                cc = chr(self.keystroke & 255).lower()
                if cc == 'n':
                    self.night_mode = not self.night_mode
                elif cc == 'f':
                    self.show_features = not self.show_features
                elif cc == 'b':
                    self.show_boxes = not self.show_boxes
                elif cc == 't':
                    self.show_text = not self.show_text
                elif cc == 'q':
                    # The has press the q key, so exit
                    rospy.signal_shutdown("User hit q key to quit.")
            except:
                pass

    def publish_sequence(self):
        # Watch out for negative offsets

        try:
            sequence = Vector3()
            sequence.data.x = self.x0
            sequence.data.y = self.y0
            sequence.data.z = self.hist_prob
            print sequence.data
            self.sequence_pub.publish(sequence)
        except:
            rospy.loginfo("Publishing sequence failed")



if __name__ == '__main__':
    try:
        node_name = "camshift"
        ColorSequence(node_name)
        try:
            rospy.init_node(node_name)
        except:
            pass
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()


