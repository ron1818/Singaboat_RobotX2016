# brute force with orb

import numpy as np
import cv2
from matplotlib import pyplot as plt

def drawMatches(img1, kp1, img2, kp2, matches):
    """
    My own implementation of cv2.drawMatches as OpenCV 2.4.9
    does not have this function available but it's supported in
    OpenCV 3.0.0

    This function takes in two images with their associated
    keypoints, as well as a list of DMatch data structure (matches)
    that contains which keypoints matched in which images.

    An image will be produced where a montage is shown with
    the first image followed by the second image beside it.

    Keypoints are delineated with circles, while lines are connected
    between matching keypoints.

    img1,img2 - Grayscale images
    kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint
              detection algorithms
    matches - A list of matches of corresponding keypoints through any
              OpenCV keypoint matching algorithm
    """

    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')
    # Place the first image to the left
    out[:rows1,:cols1] = np.dstack([img1, img1, img1])

    # Place the next image to the right of it
    out[:rows2,cols1:] = np.dstack([img2, img2, img2])

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for mat in matches:

        # Get the matching keypoints for each of the images
        img1_idx = mat.queryIdx
        img2_idx = mat.trainIdx

        # x - columns
        # y - rows
        (x1,y1) = kp1[img1_idx].pt
        (x2,y2) = kp2[img2_idx].pt

        # Draw a small circle at both co-ordinates
        # radius 4
        # colour blue
        # thickness = 1
        cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)
        cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

        # Draw a line in between the two points
        # thickness = 1
        # colour blue
        cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 1)


    # Show the image
    cv2.imshow('Matched Features', out)
    cv2.waitKey(0)
    cv2.destroyWindow('Matched Features')

    # Also return the image if you'd like a copy
    return out


# main program
# img1 = cv2.imread('image/cameraman.png')  # queryImage
img1 = cv2.imread('image/buoy_marker.png')  # queryImage
gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
gray1_np = np.float32(gray1)
# _, img1 = cv2.threshold(img1, 127, 255, cv2.THRESH_BINARY)
# img2 = cv2.imread('image/cameraman_rot55.png')  # trainImage
img2 = cv2.imread('image/sea.png')  # trainImage
gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
gray2_np = np.float32(gray2)
# _, img2 = cv2.threshold(img2, 127, 255, cv2.THRESH_BINARY)

dst = cv2.cornerHarris(gray1_np, 2, 3, 0.04)
dst = cv2.dilate(dst,None)

img1[dst>0.01*dst.max()]=[0,0,255]

cv2.imshow('dst',img1)
if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()

########### ORB #########
# Initiate orb detector
orb = cv2.ORB()

# find the keypoints and descriptors with SIFT
kp1, des1 = orb.detectAndCompute(gray1, None)
kp2, des2 = orb.detectAndCompute(gray2, None)

img = cv2.drawKeypoints(img1, kp1, color=(0,255,0), flags=0)
cv2.imshow("img", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# # create BFMatcher object
# bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#
# # Match descriptors.
# matches = bf.match(des1, des2)
#
# # Sort them in the order of their distance.
# matches = sorted(matches, key = lambda x:x.distance)

# FLANN parameters
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)   # or pass empty dictionary

flann = cv2.FlannBasedMatcher(index_params,search_params)

matches = flann.knnMatch(des1,des2,k=2)

# Need to draw only good matches, so create a mask
matchesMask = [[0,0] for i in xrange(len(matches))]

# ratio test as per Lowe's paper
for i,(m,n) in enumerate(matches):
    if m.distance < 0.7*n.distance:
        matchesMask[i]=[1,0]

draw_params = dict(matchColor = (0,255,0),
                   singlePointColor = (255,0,0),
                   matchesMask = matchesMask,
                   flags = 0)

# img3 = cv2.drawMatchesKnn(gray1,kp1,gray2,kp2,matches,None,**draw_params)

# Draw first 10 matches.
img3 = drawMatches(gray1,kp1,gray2,kp2,matches[:10])

########### SIFT ###########
# # Initiate SIFT detector
# surf = cv2.Feature2D_create("SIFT")
#
# # find the keypoints and descriptors with SIFT
# kp1, des1 = surf.detectAndCompute(img1, None)
# kp2, des2 = surf.detectAndCompute(img2, None)
#
# img = cv2.drawKeypoints(img1, kp1)
#
# cv2.imshow("img", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
# # BFMatcher with default params
# bf = cv2.BFMatcher()
# matches = bf.knnMatch(des1,des2, k=2)
#
# # Apply ratio test
# good = []
# for m,n in matches:
#     if m.distance < 0.75*n.distance:
#         good.append([m])
#
# # cv2.drawMatchesKnn expects list of lists as matches.
# img3 = drawMatchesKnn(img1,kp1,img2,kp2,good,flags=2)

