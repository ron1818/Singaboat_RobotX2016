import cv2
import numpy as np

img = cv2.imread('image/circle.jpg')
img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
img_gray = cv2.GaussianBlur(img_gray, (5, 5), 0)
ret, thresh = cv2.threshold(img_gray, 127, 255,cv2.THRESH_BINARY_INV)
res = cv2.bitwise_and(img_gray, img_gray, mask=thresh)
contours,hierarchy = cv2.findContours(res,2,1)
cnt = contours[0]

hull = cv2.convexHull(cnt,returnPoints = False)
defects = cv2.convexityDefects(cnt,hull)
print cv2.isContourConvex(cnt)

for i in range(defects.shape[0]):
    s,e,f,d = defects[i,0]
    print d
    start = tuple(cnt[s][0])
    end = tuple(cnt[e][0])
    far = tuple(cnt[f][0])
    cv2.line(img,start,end,[0,255,0],2)
    cv2.circle(img,far,5,[0,0,255],-1)

cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
