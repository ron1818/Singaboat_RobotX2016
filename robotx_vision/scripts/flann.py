import numpy as np
import cv2
from matplotlib import pyplot as plt

img1 = cv2.imread('image/buoy_marker.png',0)          # queryImage
img2 = cv2.imread('image/sea.png',0) # trainImage

# Initiate SIFT detector
sift = cv2.SIFT()

# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

# FLANN parameters
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)   # or pass empty dictionary

flann = cv2.FlannBasedMatcher(index_params,search_params)

# matches is a 2-d d-match,
matches = flann.match(des1,des2)

img1_idx = list()
img2_idx = list()
for mat in matches:
    img1_idx.append(mat.queryIdx)
    img2_idx.append(mat.trainIdx)

# for i in img2_idx:
#     good_kp2 = kp2[i].pt
#     cv2.circle(img2, (int(good_kp2[0]), int(good_kp2[1])), 1, (255,0,0), 1)
good_kp2 = [kp2[i] for i in img2_idx]

img = cv2.drawKeypoints(img2, good_kp2, color=(0,255,0), flags=0)
cv2.imshow("kpts", img)


x, y, w, h = cv2.boundingRect(good_kp2)
cv2.rectangle(img2, (x, y), (x+w, y+h), (0, 255, 0), 3)




# cv2.rectangle(img2, (x, y), (x+w, y+h), (0, 255, 0), 3)


cv2.imshow('image', img2)
cv2.waitKey(0)
cv2.destroyAllWindows()
