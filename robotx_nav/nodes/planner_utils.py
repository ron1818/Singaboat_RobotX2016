#! /usr/bin/env python

import numpy as np
import math
import random
import time


def within_map(map_corners, point):

	a=math.sqrt((x0-x1)**2+(y0-y1)**2)
	c=math.sqrt((x2-x1)**2+(y2-y1)**2)
	b=math.sqrt((x0-x2)**2+(y0-y2)**2)
	s1=(a+b+c)/2
	area1=math.sqrt(s1*(s1-a)*(s1-b)*(s1-c))
	d=math.sqrt((x1-x3)**2+(y1-y3)**2)
	e=math.sqrt((x3-x2)**2+(y3-y2)**2)
	s2=(c+d+e)/2
	area2=math.sqrt(s2*(s2-c)*(s2-e)*(s2-d))
	map_area=area1+area2


	#trig 1
	f=math.sqrt((x0-x1)**2+(y0-y1)**2)
	g=math.sqrt((x0-xt)**2+(y0-yt)**2)
	h=math.sqrt((xt-x1)**2+(yt-y1)**2)
	s3=(f+g+h)/2
	area3=math.sqrt(s3*(s3-f)*(s3-g)*(s3-h))

	#trig 2
	i=math.sqrt((x2-x1)**2+(y2-y1)**2)
	j=math.sqrt((x2-xt)**2+(y2-yt)**2)
	k=math.sqrt((xt-x1)**2+(yt-y1)**2)
	s4=(i+j+k)/2
	area4=math.sqrt(s4*(s4-i)*(s4-j)*(s4-k))

	#trig 3
	l=math.sqrt((x2-x3)**2+(y2-y3)**2)
	m=math.sqrt((x2-xt)**2+(y2-yt)**2)
	n=math.sqrt((xt-x3)**2+(yt-y3)**2)
	s5=(l+m+n)/2
	area5=math.sqrt(s5*(s5-l)*(s5-m)*(s5-n))

	#trig 4
	o=math.sqrt((x0-x3)**2+(y0-y3)**2)
	p=math.sqrt((x0-xt)**2+(y0-yt)**2)
	q=math.sqrt((xt-x3)**2+(yt-y3)**2)
	s6=(o+p+q)/2
	area6=math.sqrt(s6*(s6-o)*(s6-p)*(s6-q))

	cal_area=area3+area4+area5+area6

	if cal_area==map_area:
		return True
	elif cal_area>map_area:
		return False



def distance(a,b):

	dis=math.sqrt((xa-xb)**2+(ya-yb)**2)
	return dis
    
    #  a = np.array[[xa,ya]], b = np.array[[xb,yb]]



if __name__ == "__main__":
    map_corners = np.array([[2,1], [1,5], [8,3], 6, 12])
    point1 = np.array([4,4])
    point2 = np.array([1,1])

    """ assignment, find whether point1 and point2 is in the map area"""

