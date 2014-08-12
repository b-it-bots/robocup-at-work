#!/usr/bin/env python
import rospy
import sys

import select
import tty
import termios

from geometry_msgs.msg import WrenchStamped


from Tkinter import *
import Tkinter

from thread import start_new_thread


w = WrenchStamped()
var = 0.0

def createWindow():
    master = Tkinter.Tk()
    
    label = Label( master, text="Wrench torque / force")
    label.pack(side=TOP)

    scaleX = Scale( master, command = torqueX ,from_=-10, to=10, resolution=0.1, label="tX")
    scaleX.pack(side=LEFT)
    scaleY = Scale( master, command = torqueY ,from_=-10, to=10, resolution=0.1, label="tY")
    scaleY.pack(side=LEFT)
    scaleZ = Scale( master, command = torqueZ ,from_=-10, to=10, resolution=0.1, label="tZ")
    scaleZ.pack(side=LEFT)

    scaleX = Scale( master, command = forceX ,from_=-10, to=10, resolution=0.1, label="fX")
    scaleX.pack(side=LEFT)
    scaleY = Scale( master, command = forceY ,from_=-10, to=10, resolution=0.1, label="fY")
    scaleY.pack(side=LEFT)
    scaleZ = Scale( master, command = forceZ ,from_=-25, to=25, resolution=0.1, label="fZ")
    scaleZ.pack(side=LEFT)


    master.title("Wrench Mockup")

    master.mainloop()

    rospy.signal_shutdown("GUI closed")


def torqueX(slider):
    w.wrench.torque.x = float(slider)

def torqueY(slider):
    w.wrench.torque.y = float(slider)

def torqueZ(slider):
    w.wrench.torque.z = float(slider)

def forceX(slider):
    w.wrench.force.x = float(slider)

def forceY(slider):
    w.wrench.force.y = float(slider)

def forceZ(slider):
    w.wrench.force.z = float(slider)


def wrenchPublisher():
    pub = rospy.Publisher('~wrench', WrenchStamped)

    w.header.frame_id = rospy.get_param("~link", "/arm_7_link")



    while not rospy.is_shutdown():
        w.header.stamp = rospy.Time.now()
        pub.publish(w)
        rospy.sleep(0.1)


def main():
    rospy.init_node('wrench_mockup')

    import thread
    try:
        thread.start_new_thread(createWindow,tuple())

        wrenchPublisher()
    except rospy.ROSInterruptException:
        pass
