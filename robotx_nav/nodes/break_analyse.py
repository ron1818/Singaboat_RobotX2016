#! /use/bin/env python

from sklearn.cluster import KMeans, DBSCAN
from sklearn.preprocessing import normalize
import numpy as np
import math

def find_yellow(data_list):
    """ find the yellow pieces """
    hist_id = 2
    k_means = KMeans(n_clusters=2)
    yellow_list = data_list[data_list[:,2]==hist_id]
    k_means.fit(yellow_list)
    center = k_means.cluster_centers_
    return center

def find_break(data_list):
    """ find the breaks """
    center = list()
    hist_id = 2
    dbscan = DBSCAN(eps=0.5, min_samples=5)
    orange_data = data_list[data_list[:,1]<hist_id]
    n_data = normalize(orange_data, axis=0)
    dbscan.fit(n_data)
    labels = dbscan.labels_
    for i in range(0, np.max(labels) + 1):
        center.append(np.mean(orange_data[labels == i]))
    return center[:, 0:1]

def cosine_theory(yellow_center, orange_center):
    """ filter out orange center that is outside yellow center """
    within_center = 0
    C = distance(yellow_center[0], yellow_center[1])
    for c in orange_center:
        xo, yo = c
        A = distance(yellow_center[0], c)
        B = distance(yellow_center[1], c)
        theta = math.acos(A ** 2 + B ** 2 - C ** 2) / (2 * A * B)
        if theta <= 0:  # inside
            within_center += 1

    return within_center

def distance(a, b):
    """ eucledian distance"""
    xa, ya = a
    xb, yb = b
    return math.sqrt((xa- xb) ** 2 + (ya - yb) ** 2)


if __name__ == "__main__":
    data_list = np.empty((100, 5))
    yellow_center = find_yellow(data_list)
    orange_center = find_break(data_list)
    break_number = cosine_theory(yellow_center, orange_center)

    print break_number
