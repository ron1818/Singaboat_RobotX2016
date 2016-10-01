import numpy as np
import cv2

class Flag_match(object):
    """ matches flag shape: cross, triangle, circle """
    MIN_MATCH_COUNT = 10
    K = 2
    circle_template = 'image/template-circle-new.jpg'
    triangle_template = 'image/template-triangle-new.jpg'
    cross_template = 'image/template-cross-new.jpg'

    def __init__(self, shape, scene, detector, matcher):
        # read template
        if shape == "circle":
            self.img1 = cv2.imread(self.circle_template,0)
        elif shape == "cross":
            self.img1 = cv2.imread(self.cross_template,0)
        elif shape == "triangle":
            self.img1 = cv2.imread(self.triangle_template,0)
        else:
            self.img1 = None
            print "shape not supported"
            raise ValueError

        # read scene
        self.img2 = cv2.imread(scene, 0)
        # cv2.imshow("template", self.img1)
        # cv2.imshow("scene", self.img2)
        # cv2.waitKey(0)

        # load detector
        if detector.lower() == "sift":
            self.detector = cv2.SIFT()
        elif detector.lower() == "surf":
            self.detector = cv2.SURF()
        elif detector.lower() == "orb":
            self.detector = cv2.ORB()
        elif detector.lower() == "fast":
            self.detector = cv2.FastFeatureDetector()
        else:
            self.detector = None
            print "detector not supported"
            raise ValueError

        # load matcher
        if matcher.lower() == "flann":
            FLANN_INDEX_KDTREE = 0
            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks = 50)
            matcher = cv2.FlannBasedMatcher(index_params, search_params)
        elif matcher.lower() == "bruteforce":
            matcher = cv2.BFMatcher()
        else:
            self.matcher = None
            print "matcher not supported"
            raise ValueError

        if detector.lower() == "orb":
            kp1 = self.detector.detect(self.img1, None)
            kp1, des1 = self.detector.compute(self.img1, kp1)
            kp2 = self.detector.detect(self.img2, None)
            kp2, des2 = self.detector.compute(self.img2, kp1)
        else:
            # find the keypoints and descriptors with SIFT
            kp1, des1 = self.detector.detectAndCompute(self.img1, None)
            kp2, des2 = self.detector.detectAndCompute(self.img2, None)

        # img1_kp = cv2.drawKeypoints(self.img1, kp1, color=(0,0,255))
        # img2_kp = cv2.drawKeypoints(self.img2, kp2, color=(0,0,255))

        matches = matcher.knnMatch(des1, des2, k=self.K)
        print des1
        print des2

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.85*n.distance:
                good.append(m)

        print len(good) > self.MIN_MATCH_COUNT

        img2_idx = list()
        for mat in good:
            # img1_idx.append(mat.queryIdx)
            img2_idx.append(mat.trainIdx)

        matched_kp2 = [kp2[i] for i in img2_idx]
        matched_kp2_array = np.float32([p.pt for p in matched_kp2]).reshape(-1, 1, 2)

        img = cv2.drawKeypoints(self.img2, kp2, color=(0,255,0), flags=0)
        trackbox = cv2.boundingRect(matched_kp2_array)
        # cv2.imshow("kpts", img)

        x, y, w, h = trackbox
        cv2.rectangle(self.img2, (x, y), (x+w, y+h), (0, 255, 0), 3)
        cv2.imshow("image2", self.img2)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        circle = Flag_match(shape="circle", scene="image/template-2.jpg", detector="SIFT", matcher="FLANN")
        # triangle = Flag_match(shape="triangle", scene="image/template-4.jpg", detector="SURF", matcher="FLANN")
        # cross = Flag_match(shape="cross", scene="image/template-3.jpg", detector="ORB", matcher="FLANN")
    except:
        pass









# img1 = cv2.imread('image/template-cross-new.jpg',0)          # queryImage
# img2 = cv2.imread('image/template-4.jpg',0) # trainImage
# # img1 = cv2.imread('image/buoy_marker.png',0)          # queryImage
# # img2 = cv2.imread('image/sea.png',0) # trainImage
# # img1 = cv2.imread('image/cameraman.png',0)          # queryImage
# # img2 = cv2.imread('image/cameraman_rot55.png',0) # trainImage
#
# # Initiate SIFT detector
# sift = cv2.SIFT()
#
# # find the keypoints and descriptors with SIFT
# kp1, des1 = sift.detectAndCompute(img1,None)
# kp2, des2 = sift.detectAndCompute(img2,None)
#
# img1_kp = cv2.drawKeypoints(img1, kp1, color=(0,0,255))
# cv2.imshow("img1 kp", img1_kp)
#
# # # FLANN parameters
# # FLANN_INDEX_KDTREE = 0
# # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
# # search_params = dict(checks=50)   # or pass empty dictionary
# #
# # flann = cv2.FlannBasedMatcher(index_params,search_params)
# #
# # # single matcher
# # matches = flann.match(des1,des2)
# #
# # img1_idx = list()
# # img2_idx = list()
# # for mat in matches:
# #     img1_idx.append(mat.queryIdx)
# #     img2_idx.append(mat.trainIdx)
# #
# # # for i in img2_idx:
# # #     good_kp2 = kp2[i].pt
# # #     cv2.circle(img2, (int(good_kp2[0]), int(good_kp2[1])), 1, (255,0,0), 1)
# # matched_kp2 = [kp2[i] for i in img2_idx]
# # print matched_kp2
# # print [p.pt for p in matched_kp2]
# # matched_kp2_array = np.float32([p.pt for p in matched_kp2]).reshape(-1, 1, 2)
# #
# # img = cv2.drawKeypoints(img2, matched_kp2, color=(0,255,0), flags=0)
# # trackbox = cv2.boundingRect(matched_kp2_array)
# # cv2.imshow("kpts", img)
# #
# # x, y, w, h = trackbox
# # cv2.rectangle(img2, (x, y), (x+w, y+h), (0, 255, 0), 3)
# #
# # cv2.imshow('image', img2)
# # cv2.waitKey(0)
#
# # knn based matcher
# FLANN_INDEX_KDTREE = 0
# index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
# search_params = dict(checks = 50)
#
# flann = cv2.FlannBasedMatcher(index_params, search_params)
#
# matches = flann.knnMatch(des1, des2, k=2)
#
# # store all the good matches as per Lowe's ratio test.
# good = []
# for m,n in matches:
#     if m.distance < 0.85*n.distance:
#         good.append(m)
#
# print good
#
# img2_idx = list()
# for mat in good:
#     # img1_idx.append(mat.queryIdx)
#     img2_idx.append(mat.trainIdx)
#
# # for i in img2_idx:
# #     good_kp2 = kp2[i].pt
# #     cv2.circle(img2, (int(good_kp2[0]), int(good_kp2[1])), 1, (255,0,0), 1)
# matched_kp2 = [kp2[i] for i in img2_idx]
# print matched_kp2
# print [p.pt for p in matched_kp2]
# matched_kp2_array = np.float32([p.pt for p in matched_kp2]).reshape(-1, 1, 2)
#
# img = cv2.drawKeypoints(img2, kp2, color=(0,255,0), flags=0)
# trackbox = cv2.boundingRect(matched_kp2_array)
# cv2.imshow("kpts", img)
#
# x, y, w, h = trackbox
# cv2.rectangle(img2, (x, y), (x+w, y+h), (0, 255, 0), 3)
#
# cv2.imshow('image', img2)
# cv2.waitKey(0)
#
#
# # if len(good)>1: # MIN_MATCH_COUNT:
# #     src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
# #     dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
# #
# #     M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
# #     matchesMask = mask.ravel().tolist()
# #
# #     h,w = img1.shape
# #     pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
# #     dst = cv2.perspectiveTransform(pts,M)
# #
# #     img3 = cv2.polylines(img2,[np.int32(dst)],True,255)
# #
# # else:
# #     print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
# #     matchesMask = None
# #
# # print matchesMask
#
# cv2.destroyAllWindows()
