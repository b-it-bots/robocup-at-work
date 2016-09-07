#!/usr/bin/env python
"""
This module contains a component that publishes a mock-up
pose error.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import mcr_manipulation_msgs.msg
import Tkinter

RESOLUTION = 0.005  # in meters
MAX_ERROR = 0.1     # in meters
MIN_ERROR = -0.1     # in meters
CYCLE_TIME = 0.1     # in seconds

pose_error = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()


def create_window():
    master = Tkinter.Tk()
    
    label = Tkinter.Label(master, text="Pose Error")
    label.pack(side=Tkinter.TOP)

    scale_x = Tkinter.Scale(
        master, command=error_x, from_=MIN_ERROR, to=MAX_ERROR,
        resolution=RESOLUTION, label="error_x"
    )
    scale_x.pack(side=Tkinter.LEFT)
    scale_y = Tkinter.Scale(
        master, command=error_y, from_=MIN_ERROR, to=MAX_ERROR,
        resolution=RESOLUTION, label="error_y"
    )
    scale_y.pack(side=Tkinter.LEFT)
    scale_z = Tkinter.Scale(
        master, command=error_z, from_=MIN_ERROR, to=MAX_ERROR,
        resolution=RESOLUTION, label="error_z"
    )
    scale_z.pack(side=Tkinter.LEFT)

    master.title("Pose error mock-up")
    master.mainloop()
    rospy.signal_shutdown("GUI closed")


def error_x(slider):
    pose_error.linear.x = float(slider)


def error_y(slider):
    pose_error.linear.y = float(slider)


def error_z(slider):
    pose_error.linear.z = float(slider)


def publish_pose_error():
    # node cycle rate (in hz)
    loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

    pub = rospy.Publisher(
        '~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference
    )

    while not rospy.is_shutdown():
        pub.publish(pose_error)
        loop_rate.sleep()


def main():
    rospy.init_node('pose_error_mock_up')

    import thread
    try:
        thread.start_new_thread(create_window, tuple())

        publish_pose_error()
    except rospy.ROSInterruptException:
        pass
