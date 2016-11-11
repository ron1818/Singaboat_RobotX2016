#!/usr/bin/python

#Le Duyanh 11/11/2016 -> FA day 

import rospy
import actionlib
from actionlib_msgs.msg import *
from Tkinter import *
from PIL import ImageTk, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TwistStamped, Vector3
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, pi, sin, cos, atan2, floor, ceil, sqrt
class StatusDisplay(object):
	# initialize boat pose param
    x0, y0, z0, roll0, pitch0, yaw0, speed, lon, lat, rota_speed = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    status=""
    

    def __init__(self):
    
        self.root = Tk()
        self.canvas = Canvas(self.root, width=400, height=400)
        self.canvas.pack()
        # Store canvas in root and in canvas itself for callbacks
        self.root.canvas = self.canvas.canvas = self.canvas
        # Set up canvas data and call init
        
        rospy.Subscriber("/odom",Odometry, self.odom_callback, queue_size = 50)
	rospy.Subscriber("/chatter", Vector3, self.chatter_callback, queue_size = 50)
    	rospy.Subscriber("/navsat/fix", NavSatFix, self.navsat_callback, queue_size = 50)
	self.canvas.data = { }
        self.scancode()
	while not rospy.is_shutdown():
	    self.showWidget()
	    self.root.update_idletasks()
    	    self.root.update()
	    rospy.sleep(1)
          # This call BLOCKS (so your program waits until you close the window!)

    def scancode(self):
    	self.canvas.delete(ALL)
	self.canvas.create_text(75,20, font=("Purisa", 15), text="Boat Status:")
	self.canvas.create_text(75,60, font=("Purisa", 13), text="Position X(m):")
	self.canvas.create_text(75,100, font=("Purisa", 13), text="Position Y(m):")
	self.canvas.create_text(75,140, font=("Purisa", 13), text="Longitude::")
	self.canvas.create_text(75,180, font=("Purisa", 13), text="Latitude:")
	self.canvas.create_text(75,220, font=("Purisa", 13), text="Speed(m/s):")
	self.canvas.create_text(75,260, font=("Purisa", 13), text="Rotational speed:")
	self.canvas.create_text(75,300, font=("Purisa", 13), text="Orientation:")
	
    def showWidget(self):
	self.canvas.create_rectangle(150, 0, 350 ,40, fill="white", )
	self.canvas.create_rectangle(150, 40, 350 ,80, fill="white", )
	self.canvas.create_rectangle(150, 80, 350, 120, fill="white", )
	self.canvas.create_rectangle(150, 120, 350, 160, fill="white", )
	self.canvas.create_rectangle(150, 160, 350, 200, fill="white", )
	self.canvas.create_rectangle(150, 200, 350, 240, fill="white", )
	self.canvas.create_rectangle(150, 240, 350, 280, fill="white", )
	self.canvas.create_rectangle(150, 280, 350, 320, fill="white", )
	self.canvas.create_text(255,20, font=("Purisa", 13), text=self.status)
	self.canvas.create_text(255,60, font=("Purisa", 13), text=self.x0)
	self.canvas.create_text(255,100, font=("Purisa", 13), text=self.y0)
	self.canvas.create_text(255,140, font=("Purisa", 13), text=self.lon)
	self.canvas.create_text(255,180, font=("Purisa", 13), text=self.lat)
	self.canvas.create_text(255,220, font=("Purisa", 13), text=self.speed)
	self.canvas.create_text(255,260, font=("Purisa", 13), text=self.rota_speed)
	self.canvas.create_text(255,300, font=("Purisa", 13), text=self.yaw0)
	
    def odom_callback(self, msg):
	
        """ call back to subscribe, get odometry data:
        pose and orientation of the current boat,
        suffix 0 is for origin """
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.roll0, self.pitch0, self.yaw0 = euler_from_quaternion((x, y, z, w))
	self.rota_speed = msg.twist.twist.angular.z
	self.speed = msg.twist.twist.linear.x
	self.odom_received = True

    def navsat_callback(self, msg):
	self.lon = msg.longitude
	self.lat = msg.latitude

    def chatter_callback(self,msg):
	if msg.z == 2 :
	    self.status = "automatic"
	elif msg.z == 3:
	    self.status = "manual" 
			
########### copy-paste below here ###########

	
if __name__ == '__main__':
    try:
        StatusDisplay()
    except rospy.ROSInterruptException:
        rospy.loginfo("GUI of boat status shown")
