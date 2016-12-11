#!usr/bin/env python
'''This example is aiming to generate several Gaussian function samples in one picture '''

from sklearn.cluster import DBSCAN
from sklearn.preprocessing import normalize, StandardScaler
import numpy as np
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt

# mean = [[0,0],[10,10],[20,20],[30,30],[40,40]]
#
# cov1,cov2  = [[5,0],[0,20]],[[15,0],[0,5]]
#
# x1,y1 = np.random.multivariate_normal(mean[0],cov1,300).T
# plt.plot(x1,y1,'ro')
#
# for i in range(3):
# 	x,y = np.random.multivariate_normal(mean[i+1],cov2,300).T
# 	plt.plot(x,y,'mo')
#
# x5,y5 = np.random.multivariate_normal(mean[4],cov1,300).T
# plt.plot(x5,y5,'ro')
#
# plt.ylabel("Y axis")
# plt.xlabel("X axis")
# plt.axis([-20, 60, -20, 60 ])
#
# plt.show()

# doing clustering
t = 0
data = np.array([[0,0,0,0,0]])
while True:
    if t < 100:
        x = t / 10 + np.random.randn() * 0.5
        y = np.random.randn() * 0.5
        if t < 10 or t > 90:
            x = t / 10 + np.random.randn() * 0.5
            y = np.random.randn() * 0.1
            h = np.random.randint(0, 10)
            blob = np.random.randint(0, 3)
            area = np.random.randn()*2 + 0.05
        elif 15 < t < 25 or 35 < t < 45 or 55 < t< 65 or 65 < t < 75:
            x = t / 10 + np.random.randn() * 0.1
            y = np.random.randn() * 0.5
            h = np.random.randint(100, 120)
            blob = np.random.randint(0, 2)
            area = np.random.randn()*2 + 0.5
        else:
            x = t / 10 + np.random.randn() * 0.1
            y = np.random.randn() * 0.5
            h = np.random.randint(100, 120)
            blob = np.random.randint(2, 4)
            area = np.random.randn()*2 + 0.25

        data = np.append(data, [[x, y, h, blob, area]], axis=0)
        t += 0.5
    else:
        break

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# ax.scatter(data[:,0], data[:,1], data[:,2])

# plt.plot(data[:,0], data[:,1])

dbscan = DBSCAN(eps= 0.05, min_samples=5)
X1 = normalize(data, axis=0)
X2 = StandardScaler().fit_transform(data)
dbscan.fit(X1)
print dbscan.labels_
# plt.show()

""" subscribe to 'break' with std_msgs.msg Float64MultiArray,
msg.data[0] x
msg.data[1] y
msg.data[2] h
msg.data[3] # blob
msg.data[4] % area
"""
