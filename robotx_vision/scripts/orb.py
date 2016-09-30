import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('image/docking_side.jpg',0)
# ret, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
img = cv2.GaussianBlur(img, (5, 5), 0)
# kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
# img = cv2.filter2D(img, -1, kernel)

# Initiate STAR detector
orb = cv2.ORB()

# find the keypoints with ORB
kp, des = orb.detectAndCompute(img, None)

# draw only keypoints location,not size and orientation
img2 = cv2.drawKeypoints(img,kp,color=(0,255,0), flags=0)
plt.imshow(img2),plt.show()
plt.imshow(bw),plt.show()
