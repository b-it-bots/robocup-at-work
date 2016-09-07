#!/usr/bin/env python
"""
This module contains a component that publishes a simulated
twist.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import geometry_msgs.msg
import Tkinter

RESOLUTION = 0.005  # in meters
MAX_VELOCITY = 0.2     # in meters/second
MIN_VELOCITY = -0.2    # in meters/second

twist = geometry_msgs.msg.TwistStamped()


def create_window():
    master = Tkinter.Tk()
    
    label = Tkinter.Label(master, text="Twist")
    label.pack(side=Tkinter.TOP)

    scale_x = Tkinter.Scale(
        master, command=linear_velocity_x, from_=MIN_VELOCITY, to=MAX_VELOCITY,
        resolution=RESOLUTION, label="linear_velocity_x"
    )
    scale_x.pack(side=Tkinter.LEFT)
    scale_y = Tkinter.Scale(
        master, command=linear_velocity_y, from_=MIN_VELOCITY, to=MAX_VELOCITY,
        resolution=RESOLUTION, label="linear_velocity_y"
    )
    scale_y.pack(side=Tkinter.LEFT)
    scale_z = Tkinter.Scale(
        master, command=linear_velocity_z, from_=MIN_VELOCITY, to=MAX_VELOCITY,
        resolution=RESOLUTION, label="linear_velocity_z"
    )
    scale_z.pack(side=Tkinter.LEFT)

    master.title("Simulated twist GUI")
    master.mainloop()
    rospy.signal_shutdown("GUI closed")


def linear_velocity_x(slider):
    twist.twist.linear.x = float(slider)


def linear_velocity_y(slider):
    twist.twist.linear.y = float(slider)


def linear_velocity_z(slider):
    twist.twist.linear.z = float(slider)


def publish_topics():
    # node cycle rate (in hz)
    loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

    # publishers
    pub_twist = rospy.Publisher('~twist', geometry_msgs.msg.TwistStamped)

    twist.header.stamp = rospy.Time.now()
    twist.header.frame_id = 'base_link'

    while not rospy.is_shutdown():
        pub_twist.publish(twist)
        loop_rate.sleep()


def main():
    rospy.init_node('simulated_twist')

    import thread
    try:
        thread.start_new_thread(create_window, tuple())

        publish_topics()
    except rospy.ROSInterruptException:
        pass
