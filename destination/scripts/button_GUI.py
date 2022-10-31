#!/usr/bin/env python
# license removed for brevity
import rospy, sys
from std_msgs.msg import String
from Tkinter import *


# Function that signals main node to 'start'
def start():
    rospy.loginfo("Yes Selected")
    pub.publish("start")


# Function that signals main node to 'cancel'
def cont():
    rospy.loginfo("Continue Selected")         
    pub.publish("continue")


# Function that signals main node to 'stop assistance'
def stop_assistance():
    rospy.loginfo("Stop Assistance Selected")
    pub.publish("stop assistance")
 

# Function that signals nodes to 'emergency stop'
def stop_emergency():
    rospy.loginfo("Emergency Stop Selected")
    pub.publish("stop emergency")
    rospy.signal_shutdown("Emergency Stop Selected")
    sys.exit(0)


# Function to close node when finished
def close(msg):
    rospy.signal_shutdown("Finished")
    sys.exit(0)


# Setup node and publisher
rospy.init_node('button', anonymous=True, disable_signals=True)
pub = rospy.Publisher('button', String, queue_size=10)
rospy.Subscriber('close', String, close)

# Tkinter Loop
top = Tk()
top.geometry("650x300")

#Create 2x2 grid of window dynamic buttons
Grid.rowconfigure(top, 0, weight=1)
Grid.columnconfigure(top, 0, weight=1)

frame=Frame(top)
frame.grid(row=0, column=0, sticky=N+S+E+W)

btn_names = ['Continue','Emergency Stop']
btn_commands = [lambda : cont(),lambda : stop_emergency()]
index = 0

for row in range(1):
    Grid.rowconfigure(frame, row, weight=1)
    for col in range(2):
        Grid.columnconfigure(frame, col, weight=1)
        btn = Button(frame, background='white', height = 20, width = 100, text = btn_names[index], command = btn_commands[index])
        btn.grid(row=row, column=col, sticky=N+S+E+W)
        index += 1

top.mainloop()
    
