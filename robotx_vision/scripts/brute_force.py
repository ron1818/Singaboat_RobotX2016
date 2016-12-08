import numpy as np
import cv2
from matplotlib import pyplot as plt

img1 = cv2.imread('image/buoy_marker.png',0)          # queryImage
img2 = cv2.imread('image/sea.png',0) # trainImage

# Initiate SIFT detector
orb = cv2.ORB()
sift = cv2.SIFT()

# find the keypoints and descriptors with SIFT
# kp1, des1 = orb.detectAndCompute(img1,None)
# kp2, des2 = orb.detectAndCompute(img2,None)

kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

# create BFMatcher object
bf = cv2.BFMatcher()

# Match descriptors.
# matches = bf.match(des1,des2)
knnmatches = bf.knnMatch(des1, des2, k=2)

# Apply ratio test
good = []
for m,n in knnmatches:
    if m.distance < 0.75*n.distance:
        good.append([m])

print good

# Sort them in the order of their distance.
# matches = sorted(matches, key = lambda x:x.distance)
