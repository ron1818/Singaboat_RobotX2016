#!/usr/bin/env python

'''
mser to detect large connected blob
color threshold to get the mask
'''

import numpy as np
import cv2

def morphologial(mask):
    # morphological closing (remove small objects from the foreground)
    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # morphological closing (fill small objects from the foreground)
    kernel = np.ones((2, 2), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def poly_identify(mask_roi, hsv_roi):
    try:
        contours, hierarchy = cv2.findContours(mask_roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    except:
        return ["", ""]

    # cv2.drawContours(roi, contours, -1, (0,255,0),3)
    # cv2.imshow("roi", roi)
    area = list()
    boundingrect = list()
    approx_list = list()
    approx_lens = list()
    for i, cnt in enumerate(contours):
        area.append(cv2.contourArea(cnt))
        # approx for each contour
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        approx_list.append(approx)
        approx_lens.append(len(approx))
        boundingrect.append(cv2.boundingRect(approx))
    sort_index = np.argsort(area)
    area_sorted = np.sort(area)
    sort_index_desc = sort_index[::-1]
    area_sorted_desc = area_sorted[::-1]

    if sort_index_desc.size > 0: # find max
        i = sort_index_desc[0]
        cnt = contours[i]
        x, y, w, h = boundingrect[i]
        hsv_roi = hsv_roi[x:x+w, y:y+h]
        if 0 < np.median(hsv_roi[:,:,0]) < 25 or 175 < np.median(hsv_roi[:,:,0]) < 255:
            color_text = "red"
        elif 105 < np.median(hsv_roi[:,:,0]) < 135:
            color_text = "blue"
        elif 45 < np.median(hsv_roi[:,:,0]) < 75:
            color_text = "green"
        else:
            color_text = ""

        if 6 < approx_lens[i] < 10:
            shape_text = "circle"
        elif 1 < approx_lens[i] < 4:
            shape_text = "triangle"
        elif 10 < approx_lens[i] < 14:
            shape_text = "cruciform"
        else:
            shape_text = "unknown"
        return [color_text, shape_text]
    else:
        return ["", ""]

def region_post_process(regions):
    # sort the regions
    area = list()
    center = list()
    radius = list()
    for p in regions:
        cnt = np.array(p).reshape(-1,1,2)
        # cnt_moment = cv2.moments(cnt)
        # area
        area.append(cv2.contourArea(cnt))
        # centroid and radius
        (x, y), r = cv2.minEnclosingCircle(cnt)
        center.append((int(x), int(y)))
        radius.append(int(r))


    sort_index = np.argsort(area)
    # area_sorted = np.sort(area)
    sort_index_desc = sort_index[::-1]
    # area_sorted_desc = area_sorted[::-1]
    # check if one area intersect another area

    # eliminate index with intersect
    # select the top 10 index
    top_indices = sort_index_desc[0:10]
    center_vector = np.array([center[i] for i in top_indices])
    radius_vector = np.array([radius[i] for i in top_indices])
    ratio_mat = np.empty([10,10])
    for i in range(10):
        for j in range(10):
            ratio_mat[i, j] = np.sqrt((center_vector[i,0]-center_vector[j,0]) ** 2 +
                                      (center_vector[i,1]-center_vector[j,1]) ** 2) / \
                                  (radius_vector[i] + radius_vector[j])

    # find those ratio < 0.5



    return [sort_index_desc, area, center, radius]




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

        # rgb_color_mask = morphological(rgb_color_mask)
        # cv2.imshow("color mask morp", color_mask)

        # gray after color mask
        rgb_res = cv2.bitwise_and(gray, gray, mask=rgb_color_mask)
        # cv2.imshow('mask', rgb_color_mask)

        # mser detect
        regions = mser.detect(rgb_res)
        region_post_process(regions)



        # # find maximum
        # if sort_index_desc.size > 0:
        #     max_index = sort_index_desc[0]
        #     hulls = [cv2.convexHull(np.array(regions[max_index]).reshape(-1, 1, 2))]
        #     # hulls = [cv2.convexHull(np.array(p).reshape(-1, 1, 2)) for p in regions[max_index]]
        #     # cv2.polylines(gray, hulls, 1, (255, 255, 0))
        #     x, y, w, h = cv2.boundingRect(hulls[0].reshape(-1, 1, 2))
        #     cv2.rectangle(vis, (x, y), (x+w, y+h), (0, 255, 0), 3)

        # find max 3
        if sort_index_desc.size >= 3:
            max_index = sort_index_desc[0:3]
            hulls = list()
            # identify the largest hulls
            for i in max_index:
                cnt = np.array(regions[i].reshape(-1, 1, 2))
                hulls.append(cv2.convexHull(cnt))
            # cv2.polylines(vis, hulls, 1, (255, 255, 0))

            # for each hulls, get bounding box and roi
            for i in range(len(hulls)):
                x, y, w, h = cv2.boundingRect(hulls[i].reshape(-1, 1, 2))
                # enlarge the boundingbox for shape identify
                delta = 10
                ex_x0, ex_x1 = np.max([0, x-delta]), np.min([vis.shape[1], x+w+delta])
                ex_y0, ex_y1 = np.max([0, y-delta]), np.min([vis.shape[0], y+h+delta])
                mask_roi = rgb_color_mask[ex_y0:ex_y1, ex_x0:ex_x1]
                hsv_roi = hsv[ex_y0:ex_y1, ex_x0:ex_x1]
                # identify based on roi
                [color_text, shape_text] = poly_identify(mask_roi, hsv_roi)
                # draw the boundingbox and text
                cv2.rectangle(vis, (x, y), (x+w, y+h), (0, 255, 0), 3)
                cv2.putText(vis, color_text + shape_text, (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255,255,255), 2)

        cv2.imshow('img', vis)
        if 0xFF & cv2.waitKey(5) == 27:
            break
    cv2.destroyAllWindows()
