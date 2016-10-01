import numpy as np
import cv2

img = cv2.imread('image/circle.jpg',0)
# img = cv2.pyrDown(img)
img = cv2.GaussianBlur(img, (5, 5), 0)
img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

# Initiate orb detector
# detector = cv2.SIFT()
detector = cv2.ORB(edgeThreshold=5)
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
matcher = cv2.FlannBasedMatcher(index_params, search_params)

# find the keypoints with ORB
kp1, des1 = detector.detectAndCompute(img, None)
img_kp1 = cv2.drawKeypoints(img, kp1, color=(255,0,0))
# cv2.imshow("image", img_kp1)
# cv2.waitKey(0)

# load video
video = cv2.VideoCapture(0)

while video.isOpened():
    ret, frame = video.read()
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray = cv2.adaptiveThreshold(frame_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    # frame_gray = cv2.GaussianBlur(frame_gray, (5, 5), 0)

    # blob_detector = cv2.SimpleBlobDetector()
    # keypoints = blob_detector.detect(frame_gray)
    # frame_kp = cv2.drawKeypoints(frame, keypoints, color=(255,0,0))
    # cv2.imshow("keypoints", frame_kp)

    kp2, des2 = detector.detectAndCompute(frame_gray, None)
    # # print kp2
    # # frame_kp = cv2.drawKeypoints(frame, kp2, color=(255,0,0))


    try:
        matches = matcher.knnMatch(np.float32(des1), np.float32(des2), k=2)
        # # print matches

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.75*n.distance:
                good.append(m)

        print len(good) > 10

        img2_idx = list()
        for mat in good:
            # img1_idx.append(mat.queryIdx)
            img2_idx.append(mat.trainIdx)

        matched_kp2 = [kp2[i] for i in img2_idx]
        matched_kp2_array = np.float32([p.pt for p in matched_kp2]).reshape(-1, 1, 2)
        # print matched_kp2_array

        trackbox = cv2.boundingRect(matched_kp2_array)
        print trackbox

        x, y, w, h = trackbox
        cv2.rectangle(frame_gray, (x, y), (x+w, y+h), (0, 255, 0), 3)
        cv2.imshow("video", frame_gray)
    except:
        print "match fails"
        pass
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

video.release()
cv2.destroyAllWindows()


