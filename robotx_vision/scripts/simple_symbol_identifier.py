#!/usr/bin/env python

'''
MSER detector demo
==================

Usage:
------
    mser.py [<video source>]

Keys:
-----
    ESC   - exit

'''

import numpy as np
import cv2
# import video
def morphologial(mask):
    # morphological closing (remove small objects from the foreground)
    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # morphological closing (fill small objects from the foreground)
    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

if __name__ == '__main__':
    cam = cv2.VideoCapture(0)
    mser = cv2.MSER()
    while True:
        ret, img = cam.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        vis = img.copy()

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # hsv range
        red_color_mask = cv2.inRange(hsv, np.array([0,90,90]), np.array([25,255,255]))\
            + cv2.inRange(hsv, np.array([175,90,90]), np.array([255,255,255]))
        blue_color_mask = cv2.inRange(hsv, np.array([105,90,90]), np.array([135,255,255]))
        green_color_mask = cv2.inRange(hsv, np.array([45,90,90]), np.array([75,255,255]))
        rgb_color_mask = red_color_mask + blue_color_mask + green_color_mask

        # cv2.imshow("color mask morp", color_mask)
        rgb_res = cv2.bitwise_and(gray, gray, mask=rgb_color_mask)
        cv2.imshow('mask', rgb_color_mask)

        regions = mser.detect(rgb_res)

        # for p in regions:
        #     area.append(cv2.contourArea(np.array(p).reshape(-1,1,2)))
        # sort_index = np.argsort(area)
        # area_sorted = np.sort(area)
        # sort_index_desc = sort_index[::-1]
        # area_sorted_desc = area_sorted[::-1]

        contours, hierarchy = cv2.findContours(rgb_res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        area = list()
        boundingrect = list()
        approx_list = list()
        approx_lens = list()
        for i, cnt in enumerate(contours):
            area.append(cv2.contourArea(cnt))
            # approx for each contour
            approx = cv2.approxPolyDP(cnt, 0.03 * cv2.arcLength(cnt, True), True)
            approx_list.append(approx)
            approx_lens.append(len(approx))
            boundingrect.append(cv2.boundingRect(approx))
        sort_index = np.argsort(area)
        area_sorted = np.sort(area)
        sort_index_desc = sort_index[::-1]
        area_sorted_desc = area_sorted[::-1]

        if sort_index_desc.size >=3:
            for i in sort_index_desc[0:3]:
                cnt = contours[i]
                # cv2.drawContours(vis, [cnt], 0, (0,255,0), 3)
                # print approx_lens[i]
                x, y, w, h = boundingrect[i]
                hsv_roi = hsv[x:x+w, y:y+h]
                color_text = ""
                if 0 < np.median(hsv_roi[:,:,0]) < 25 or 175 < np.median(hsv_roi[:,:,0]) < 255:
                    color_text = "red"
                elif 105 < np.median(hsv_roi[:,:,0]) < 135:
                    color_text = "blue"
                elif 45 < np.median(hsv_roi[:,:,0]) < 75:
                    color_text = "green"


                mask_roi = rgb_color_mask[x:x+w, y:y+h]
                hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
                cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
                hist = hist.reshape(-1)
                # print hist


                cv2.rectangle(vis, (x, y), (x+w, y+h), (0, 255, 0), 3)
                if 6 < approx_lens[i] < 10:
                    cv2.putText(vis, color_text + "circle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)
                elif 1 < approx_lens[i] < 4:
                    cv2.putText(vis, color_text + "triangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)
                elif 10 < approx_lens[i] < 14:
                    cv2.putText(vis, color_text + "cruciform", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)
                else:
                    cv2.putText(vis, "unknown", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)




        # # find maximum
        # if sort_index_desc.size > 0:
        #     max_index = sort_index_desc[0]
        #     hulls = [cv2.convexHull(np.array(regions[max_index]).reshape(-1, 1, 2))]
        #     # hulls = [cv2.convexHull(np.array(p).reshape(-1, 1, 2)) for p in regions[max_index]]
        #     # cv2.polylines(gray, hulls, 1, (255, 255, 0))
        #     x, y, w, h = cv2.boundingRect(hulls[0].reshape(-1, 1, 2))
        #     cv2.rectangle(vis, (x, y), (x+w, y+h), (0, 255, 0), 3)

        # # find max 3
        # if sort_index_desc.size >= 3:
        #     max_index = sort_index_desc[0:3]
        #     hulls = list()
        #     for i in max_index:
        #         hulls.append(cv2.convexHull(np.array(regions[i]).reshape(-1, 1, 2)))
        #     # cv2.polylines(gray, hulls, 1, (255, 255, 0))
        #     for i in range(len(hulls)):
        #         x, y, w, h = cv2.boundingRect(hulls[i].reshape(-1, 1, 2))
        #         cv2.rectangle(vis, (x, y), (x+w, y+h), (0, 255, 0), 3)

        cv2.imshow('img', vis)
        if 0xFF & cv2.waitKey(5) == 27:
            break
    cv2.destroyAllWindows()
