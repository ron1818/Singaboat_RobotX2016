import cv2
img = cv2.imread('docking_side.jpg', cv2.CV_LOAD_IMAGE_GRAYSCALE);
mser = cv2.MSER()
mser_areas = mser.detect(img)
for mser_region in mser_areas:
    ellipse = cv2.fitEllipse(mser_region)
    cv2.ellipse(img, ellipse, (0, 255, 0), 2)

cv2.imshow("img", img)
cv2.waitKey(0)

