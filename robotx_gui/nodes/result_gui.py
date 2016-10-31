#!/usr/bin/python

#Le Duyanh 31/10/16->halloween day

import rospy
import actionlib
from actionlib_msgs.msg import *
from Tkinter import *
from Image import *
from PIL import ImageTk, Image

class Result_GUI(object):


    def __init__(self):
    
        self.root = Tk()
        self.canvas = Canvas(self.root, width=400, height=600)
        self.canvas.pack()
        # Store canvas in root and in canvas itself for callbacks
        self.root.canvas = self.canvas.canvas = self.canvas
        # Set up canvas data and call init
        self.canvas.data = { }
        self.scancode()
        #change variable in here

    
	while not rospy.is_shutdown():	

	    color1=rospy.get_param("/gui/color1")
	    color2=rospy.get_param("/gui/color2")
	    color3=rospy.get_param("/gui/color3")

	    shape1=rospy.get_param("/gui/shape1")
	    shape2=rospy.get_param("/gui/shape2")

	    break_num=rospy.get_param("/gui/break_number", "5")
		

            self.setScanTheColor(color1, color2, color3)
            self.setScanTheShape(shape1, shape2)
            self.SetNumberOfSegment(break_num)
	    self.root.update_idletasks()
    	    self.root.update()
	    rospy.sleep(1)

          # This call BLOCKS (so your program waits until you close the window!)

    def scancode(self):
    	self.canvas.delete(ALL)
	#draw the background
	self.canvas.create_rectangle(12.5, 12.5, 387.5 ,225, fill="white")
	self.canvas.create_rectangle(12.5, 250, 387.5 ,450, fill="white")
	self.canvas.create_rectangle(12.5, 475, 387.5 ,575, fill="white")
	# draw semi-transparent rectangles in the scan the code box
	self.canvas.create_rectangle(25, 60, 125,90, fill="white",)
    	self.canvas.create_rectangle(150, 60, 250, 90, fill="white", )
    	self.canvas.create_rectangle(275, 60, 375, 90, fill="white", )
	# draw semi-transparent rectangles in the mscan the Coral Survey
	self.canvas.create_rectangle(50, 297.5, 150, 327.5, fill="white", )
	self.canvas.create_rectangle(225, 297.5, 325, 327.5, fill="white", )
	self.canvas.create_rectangle(50, 340, 150, 440, fill="black", )
	self.canvas.create_rectangle(225, 340, 325, 440, fill="black", )
	# draw semi-transparent rectangles in the mscan the find the break
	self.canvas.create_rectangle(75, 500, 325, 540, fill="white", )
	#write some word in font purisa and text size 20
	
	self.canvas.create_text(200,25, font=("Purisa", 17), text="Scan the code")
	self.canvas.create_text(200,262.5, font=("Purisa", 17), text="Coral Survey")
	self.canvas.create_text(200,487.5, font=("Purisa", 17), text="Find the Break")
	
    def setScanTheColor(self, color1, color2, color3):
	self.canvas.create_rectangle(25, 100, 125, 200, fill=color1,)
    	self.canvas.create_rectangle(150, 100, 250, 200, fill=color2, )
    	self.canvas.create_rectangle(275, 100, 375, 200, fill=color3, )
	self.canvas.create_rectangle(25, 60, 125,90, fill="white",)
    	self.canvas.create_rectangle(150, 60, 250, 90, fill="white", )
    	self.canvas.create_rectangle(275, 60, 375, 90, fill="white", )
	
	self.canvas.create_text(75,75, font=("Purisa", 15), text=color1)
	self.canvas.create_text(200,75, font=("Purisa", 15), text=color2)
	self.canvas.create_text(325,75, font=("Purisa", 15), text=color3)
	if (color1=="white"):
   	    self.canvas.create_rectangle(25, 60, 125,90, fill="white",)
	if (color2=="white"):
	    self.canvas.create_rectangle(150, 60, 250, 90, fill="white", )
	if (color3=="white"):
	    self.canvas.create_rectangle(275, 60, 375, 90, fill="white", )


    def setScanTheShape(self, pic1, pic2):
	self.canvas.create_rectangle(50, 297.5, 150, 327.5, fill="white", )
	self.canvas.create_rectangle(225, 297.5, 325, 327.5, fill="white", )
	if pic1 == "CIR":
	    im = Image.open('/home/rm/catkin_ws/src/Singaboat_RobotX2016/robotx_gui/picture/CIR.jpg')#Must change the location of the image in here
	    self.canvas.create_text(100,312.5,font=("Purisa", 12), text="CIRCLE")	 
	elif pic1 == "TRI":
	    im = Image.open('/home/rm/catkin_ws/src/Singaboat_RobotX2016/robotx_gui/picture/TRI.jpg')
	    self.canvas.create_text(100,312.5,font=("Purisa", 12), text="TRIANGLE")
	elif pic1 == "CRU":
	    im = Image.open('/home/rm/catkin_ws/src/Singaboat_RobotX2016/robotx_gui/picture/CRU.jpg')
	    self.canvas.create_text(100,312.5,font=("Purisa", 12), text="CRUCIFORM")	
	else:
	    self.canvas.create_rectangle(50, 297.5, 150, 327.5, fill="white", )
	    rospy.loginfo("no picture")
	    #im = Image.open('/home/duy/NONE.jpg')

	if pic2 == "CIR":
	    im2 = Image.open('/home/rm/catkin_ws/src/Singaboat_RobotX2016/robotx_gui/picture/CIR.jpg')
	    #Must change the location of the image in here
	    self.canvas.create_text(275,312.5,font=("Purisa", 15), text="CIRCLE")	 
	elif pic2 == "TRI":
	    im2 = Image.open('/home/rm/catkin_ws/src/Singaboat_RobotX2016/robotx_gui/picture/TRI.jpg')
	    self.canvas.create_text(275,312.5,font=("Purisa", 12), text="TRIANGLE")
	elif pic2 == "CRU":
	    im2 = Image.open('/home/rm/catkin_ws/src/Singaboat_RobotX2016/robotx_gui/picture/CRU.jpg')
	    self.canvas.create_text(275,312.5,font=("Purisa", 12), text="CRUCIFORM")
	else:
	    self.canvas.create_rectangle(225, 297.5, 325, 327.5, fill="white", )
	    rospy.loginfo("no picture")
	    #im = Image.open('/home/duy/NONE.jpg')

	self.canvas.image = ImageTk.PhotoImage(im)
	self.canvas.create_image(50, 340, image=self.canvas.image, anchor='nw')
	self.canvas.image2 = ImageTk.PhotoImage(im2)
	self.canvas.create_image(225, 340, image=self.canvas.image2, anchor='nw')
	#canvas.create_text(100,312.5, font=("Purisa", 15), text=pic1)
	#canvas.create_text(275,312.5, font=("Purisa", 15), text=pic2)

    def SetNumberOfSegment(self, num):
	self.canvas.create_rectangle(75, 500, 325, 540, fill="white", )
	self.canvas.create_text(200,520, font=("Purisa", 15), text=num)
	if (num==0):
	    self.canvas.create_rectangle(75, 500, 325, 540, fill="white", )

########### copy-paste below here ###########

	
if __name__ == '__main__':
    try:
        Result_GUI()
    except rospy.ROSInterruptException:
        rospy.loginfo("GUI of results shown")
