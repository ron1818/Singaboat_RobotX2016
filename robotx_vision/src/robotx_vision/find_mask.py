# import the necessary packages
import numpy as np
import cv2
import inspect, os

class Masking(object):
    """ detect object based on shape and color,
    for Robotx challenge"""

    MIN_MATCH_COUNT = 3
    K = 2
    # calibrated by palette
    lower_red1 = np.array([0, 90, 90])
    upper_red1 = np.array([10, 255, 255])
    # upper_red1 = np.array([25, 255, 255])
    lower_red2 = np.array([190, 90, 90])
    # lower_red2 = np.array([175, 90, 90])
    upper_red2 = np.array([255, 255, 255])
    lower_blue = np.array([105, 90, 90])
    upper_blue = np.array([135, 255, 255])
    lower_green = np.array([45, 50, 75])
    upper_green = np.array([75, 255, 255])


    circle_template = os.path.dirname(os.path.realpath(__file__))+('/image/circle.jpg')
    triangle_template = os.path.dirname(os.path.realpath(__file__))+('/image/triangle.jpg')
    cross_template = os.path.dirname(os.path.realpath(__file__))+('/image/cross.jpg')

    def __init__(self, color, shape, masker, detector, matcher, matching_method):

        self._color = color
        self._shape = shape
        self._masker = masker
        self._detector = detector
        self._matcher = matcher
        self._matching_method = matching_method

        # read color
        if self._color is not None:
            self.color = color.lower()
        else:
            print "color not supported or no color for masking"
            self.color = None

        # read template
        if self._shape is not None:
            if self._shape.lower() == "circle":
                self.target = cv2.imread(self.circle_template, 0)
            elif self._shape.lower() == "cross":
                self.target = cv2.imread(self.cross_template,0)
            elif self._shape.lower() == "triangle":
                self.target = cv2.imread(self.triangle_template,0)
            else:
                self.target = None
                print "target not supported"
        else:
            self.target = None
            print "target not present"

        # read masker
        if self._masker is not None:
            if self._masker.lower() == "bw":
                self.masker = self.binary_mask
            elif self._masker.lower() == "canny":
                self.masker = self.canny_edge_mask
            elif self._masker.lower() == "color" and self.color is not None:
                self.masker = self.color_mask
            else:
                print "mask method not supported"
                self.masker = None
        else:
            print "mask method not present"
            self.masker = None

        # read detector
        # todo detector_params
        if self.target is not None and self._detector is not None: # have target to detect
            if self._detector.lower() == "sift":
                self.detector = cv2.SIFT()
            elif self._detector.lower() == "surf":
                self.detector = cv2.SURF()
            elif self._detector.lower() == "orb":
                self.detector = cv2.ORB(nlevels=2, edgeThreshold=5)
            elif self._detector.lower() == "fast":
                self.detector = cv2.FastFeatureDetector()
            else:
                self.detector = None
                print "detector not supported or no detector for matching"
        else:
            self.detector = None
            print "detector not required"

        # read matcher
        if self.detector is not None and self._matcher is not None:
            if self._matcher.lower() == "flann":
                FLANN_INDEX_KDTREE = 0
                index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
                search_params = dict(checks = 50)
                self.matcher = cv2.FlannBasedMatcher(index_params, search_params)
                self.matching_method = self.matcher.knnMatch
            elif self._matcher.lower() == "bruteforce":
                self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
                self.matching_method = self.matcher.match
            else:
                self.matcher = None
                print "matcher not supported or no matcher"
        else:
            self.matcher = None
            print "matcher not required"

        # # read matching method
        # if self.matcher is not None and self._matching_method is not None:
        #     if self._matching_method.lower() == "knn":
        #         self.matching_method = self.matcher.knnMatch
        #     else:  # default is one match
        #         self.matching_method = self.matcher.match
        # else:
        #     self.matching_method = None
        #     print "matching method not required"

    def connect_device(self):
        self.cap = cv2.VideoCapture(self.dev)

    def disconnect_device(self):
        cv2.destroyAllWindows()
        self.cap.release()

    def streaming(self, dev, *args):
        """ stream the video from dev (e.g. 0),
            apply masking_method: color, canny, binary, etc.
            detect the shape from the mask.
            *args is for additional args of the masking_method
        """
        if self.detector is not None:
            kp_r, des_r = self.detector.detectAndCompute(self.target, None)
            target_kp = cv2.drawKeypoints(self.target, kp_r, color=(0,0,255))
            cv2.imshow("target", target_kp)

        # connect video
        try:
            self.dev = dev
        except ValueError, IOError:
            print "device is not found"

        # connect device
        self.connect_device()

        # deal for each frame
        while True:
            # read image
            ret, frame = self.cap.read()
            # blur
            frame = cv2.GaussianBlur(frame, (5, 5), 0)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)

            # create mask
            if self.masker is not None:
                if self._masker.lower() == "color":
                    mask = self.masker(frame, *args)
                else:  # other kind of masking, use gray
                    mask = self.masker(gray, *args)
                # find boundingboxes of the mask
                boundingrect = self.find_contours(mask)  # , candidate_shape)
            else:  # no mask then full frame as bounding
                height, width, channels = frame.shape
                boundingrect = [0, 0, width, height]

            # draw boundingbox
            for br in boundingrect:
                print "has br"
                x, y, w, h = br
                # problem: trimmed
                obj = gray[y:y+h, x:x+w]
                kp_o, des_o = self.detector.detectAndCompute(obj, None)
                obj_kp = cv2.drawKeypoints(obj, kp_o, color=(0,255,0))
                cv2.imshow("frame_kp", obj_kp)
                # no keypoints detected, go to next frame
                if len(kp_o) == 0 or des_o == None:
                    continue

                # match descriptors
                # matching on the boundingrect
                if self._matcher.lower() == "flann":  # need feature matching
                    # print len(kp_r), len(kp_o)
                    try:
                        if len(kp_r) > self.k and len(kp_o) > self.k:
                            matches = self.matching_method(des_r, des_o, k=self.K)
                    except:
                        matches = list()
                        pass

                    # store all the good matches as per Lowe's ratio test.
                    good = []
                    for m, n in matches:
                        if m.distance < 0.75*n.distance:
                            good.append(m)

                    #print len(good) > self.MIN_MATCH_COUNT

                    frame_idx = list()
                    for mat in good:
                        frame_idx.append(mat.trainIdx)

                    if frame_idx == []:  # empty
                        # print "no match"
                        matched_kp2 = []
                        break
                    else:
                        matched_kp2 = [kp_o[i] for i in frame_idx]
                        matched_kp2_array = np.float32([p.pt for p in matched_kp2]).reshape(-1, 1, 2)

                    x, y, w, h = cv2.boundingRect(matched_kp2_array)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                else:
                    matches = self.matching_method(des_r, des_o)
                    # draw object on frame, if threshold met
                    if(len(matches) >= self.MIN_MATCH_COUNT):
                        cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0), 2)



                # cv2.rectangle(frame, (x-5, y-5), (x+w+5, y+h+5), (0, 255, 0), 3)

            cv2.imshow('frame', frame)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break

        # disconnect and destroy
        self.disconnect_device()

    def morphological(self, mask):
        """ tune the mask """
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
            return None
            raise ValueError

        return self.morphological(mask)

    def binary_mask(self, gray, is_adaptive=False):
        if not is_adaptive:
            ret, mask = cv2.threshold(gray, 100, 200, cv2.THRESH_BINARY_INV)

        else:
            mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY_INV, 11, 2)
        # return self.morphological(mask)
        return mask

    def canny_edge_mask(self, gray, threshold1=100, threshold2=200):
        mask = cv2.Canny(gray, threshold1, threshold2, L2gradient=True)
        return mask

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
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # find all contours
        # area = list()
        boundingrect = list()
        candidate_shape = list()
        isconvex = list()
        for i, cnt in enumerate(contours):
            area.append(cv2.contourArea(cnt))
            # approx for each contour
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            print len(approx)
            boundingrect.append(cv2.boundingRect(approx))

            try:
                # find convex hull
                hull = cv2.convexHull(cnt, returnPoints=False)
                defects = cv2.convexityDefects(cnt, hull)

                #customized isconvex based on empirical
                defect_distance=list()
                for j in range(defects.shape[0]):
                    s,e,f,d = defects[j,0]
                    defect_distance.append(d)

                if np.amax(defect_distance) < 1000:  # 1000 is empirical
                    isconvex.append(True)
                else:
                    isconvex.append(False)

                candiate_shape.append(len(approx))
                # # shape determine
                # if len(approx) > 10 and isconvex is True:
                #     candidate_shape = "circle"
                # elif 7 >= len(approx) >= 3:
                #     candidate_shape.append("triangle")
                # elif len(approx) > 8 and isconvex is False:
                #     candidate_shape.append("cross")
                # else:
                #     candidate_shape.append("unkown")

                # if candidate_shape == candidate_shape.lower():  # shape name matches
                #     boundingrect.append(cv2.boundingRect(approx))
            except:
                break

        return [boundingrect, candidate_shape]


if __name__ == "__main__":
    # cd = ColorDetection(dev=0, col_under_det="red")
    # cd = ColorDetection(dev=0, col_under_det="green")
    cd = Masking(color="red", shape="circle", masker="color", detector="surf", matcher="flann", matching_method="knn")
    # cd.hsv_calibartion()
    # cd.streaming("binary_mask", True)
    cd.streaming(0, "red")
    # cd.streaming("canny_edge_mask")
