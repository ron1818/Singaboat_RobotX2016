#!/usr/bin/python
#chmod +x ....location of file


import rospy
import actionlib
from actionlib_msgs.msg import *
from Tkinter import *
from Image import *
from PIL import ImageTk, Image

# Here, we are creating our class, Window, and inheriting from the Frame
# class. Frame is a class from the tkinter module. (see Lib/tkinter/__init__)
class Window(Frame):

    # Define settings upon initialization. Here you can specify
    def __init__(self, master=None):
        
        # parameters that you want to send through the Frame class. 
        Frame.__init__(self, master)   

        #reference to the master widget, which is the tk window                 
        self.master = master

        #with that, we want to then run init_window, which doesn't yet exist
        self.init_window()
	self.create_widgets()
    #Creation of init_window
    def init_window(self):

        # changing the title of our master widget      
        self.master.title("GUI")

        # allowing the widget to take the full space of the root window
        self.pack(fill=BOTH, expand=1)

        # creating a button instance
        StopButton = Button(self, text="Stop",command=self.client_stop)
	InitButton = Button(self, text="Set initial position",command=self.client_init_position)
	ManualButton = Button(self, text="Manual",command=self.client_Manual)
	Miss1Button = Button(self, text="Mission1",command=self.Mission1)
	Miss2Button = Button(self, text="Mission2",command=self.Mission2)
	Miss3Button = Button(self, text="Mission3",command=self.Mission3)
	Miss4Button = Button(self, text="Mission4",command=self.Mission4)
	Miss5Button = Button(self, text="Mission5",command=self.Mission5)
	Miss6Button = Button(self, text="Mission6",command=self.Mission6)
	Miss7Button = Button(self, text="Mission7",command=self.Mission7)
	Miss8Button = Button(self, text="Mission8",command=self.Mission8)
        # placing the button on my window
        StopButton.grid(row=1,column=1,sticky = E)
	InitButton.grid(row=1,column=2,sticky = E)
       	ManualButton.grid(row=1,column=4,sticky = E)
	Miss1Button.grid(row=2,column=1,sticky = E)
	Miss2Button.grid(row=2,column=2,sticky = E)
	Miss3Button.grid(row=2,column=4,sticky = W)
	Miss4Button.grid(row=3,column=1,sticky = E)
	Miss5Button.grid(row=3,column=2,sticky = E)
	Miss6Button.grid(row=3,column=4,sticky = W)
	Miss7Button.grid(row=4,column=1,sticky = E)
	Miss8Button.grid(row=4,column=2,sticky = E)
	#create widgets
    def	create_widgets(self):
	#create button+text+widgets
	self.intro = Label(self,text="Go to latitude")
	self.intro.grid(row=5,column=1,sticky = W)
	self.text1 = Text(self,width=25,height =1, wrap=WORD)
	self.text1.grid (row=5, column=2,columnspan=2, sticky=W)
	self.intro = Label(self,text="Go to longitude")
	self.intro.grid(row=6,column=1,sticky = W)
	self.text2 = Text(self,width=25,height =1, wrap=WORD)
	self.text2.grid (row=6, column=2,columnspan=2, sticky=W)
	GoButton11 = Button(self, text="Go",command=self.Go)
	GoButton11.grid(row=7,column=2,sticky = E)
	#change the command in here	
    def client_stop(self):
        exit()
    def client_init_position(self):
	print "1"
    def client_Manual(self):
	print "2"
    def Mission1(self):
	print "3"
    def Mission2(self):
	print "4"
    def Mission3(self):
	print "5"
    def Mission4(self):
	print "6"
    def Mission5(self):
	print "7"
    def Mission6(self):
	print "8"
    def Mission7(self):
	print "9"
    def Mission8(self):
	print "10"
    def Go(self):
	content1= self.text1.get("1.0", "end-1c")
	content2= self.text2.get("1.0","end-1c")
	print (content1)
	print (content2)
# root window created. Here, that would be the only window, but
# you can later have windows within windows.
root = Tk()

root.geometry("400x400")

#creation of an instance
app = Window(root)

#mainloop5
root.update()  # This call BLOCKS (so your program waits until you close the window!)
root.mainloop()
