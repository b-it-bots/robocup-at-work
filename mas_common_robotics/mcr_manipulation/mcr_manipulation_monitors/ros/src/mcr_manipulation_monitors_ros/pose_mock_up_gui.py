#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This module contains a component that publishes an artificial object pose.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
import Tkinter

__author__ = 'jsanch'

RESOLUTION = 0.005      # in meters
MAX_POSITION_X = 0.6    # in meters
MIN_POSITION_X = 0.0    # in meters
MAX_POSITION_Y = 0.3    # in meters
MIN_POSITION_Y = -0.3   # in meters
MAX_POSITION_Z = 0.0    # in meters
MIN_POSITION_Z = 0.6    # in meters
LOOP_RATE = 0.1         # in seconds

TRANSPARENCY = 0.5

target_pose = geometry_msgs.msg.PoseStamped()
tolerance = visualization_msgs.msg.Marker()


def create_window():
    master = Tkinter.Tk()

    label = Tkinter.Label(master, text="Target Pose")
    label.pack(side=Tkinter.TOP)

    scale_x = Tkinter.Scale(
        master, command=position_x, from_=MIN_POSITION_X, to=MAX_POSITION_X,
        resolution=RESOLUTION, label="position_x"
    )
    scale_x.pack(side=Tkinter.LEFT)
    scale_y = Tkinter.Scale(
        master, command=position_y, from_=MIN_POSITION_Y, to=MAX_POSITION_Y,
        resolution=RESOLUTION, label="position_y"
    )
    scale_y.pack(side=Tkinter.LEFT)
    scale_z = Tkinter.Scale(
        master, command=position_z, from_=MIN_POSITION_Z, to=MAX_POSITION_Z,
        resolution=RESOLUTION, label="position_z"
    )
    scale_z.pack(side=Tkinter.LEFT)

    master.title("Pose error mock-up")
    master.mainloop()
    rospy.signal_shutdown("GUI closed")


def position_x(slider):
    target_pose.pose.position.x = float(slider)
    tolerance.pose.position.x = float(slider)


def position_y(slider):
    target_pose.pose.position.y = float(slider)
    tolerance.pose.position.y = float(slider)


def position_z(slider):
    target_pose.pose.position.z = float(slider)
    tolerance.pose.position.z = float(slider)


def publish_topics():
    # node cycle rate (in hz)
    loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))
    # the minimum Euclidean distance to activate the monitor (in meters)
    specified_tolerance = rospy.get_param('~tolerance')

    # publishers
    start_converter = rospy.Publisher('~start_converter', std_msgs.msg.String)
    pub_target_pose = rospy.Publisher('~target_pose', geometry_msgs.msg.PoseStamped)
    pub_tolerance = rospy.Publisher('~visual_tolerance', visualization_msgs.msg.Marker)

    target_pose.header.stamp = rospy.Time.now()
    target_pose.header.frame_id = 'base_link'

    tolerance.header.stamp = rospy.Time.now()
    tolerance.header.frame_id = 'base_link'

    # create a blue cube with the size of the tolerance
    tolerance.type = 1
    tolerance.scale.x = specified_tolerance
    tolerance.scale.y = specified_tolerance
    tolerance.scale.z = specified_tolerance
    tolerance.color.r = 0.0
    tolerance.color.g = 0.0
    tolerance.color.b = 1.0
    tolerance.color.a = TRANSPARENCY

    while not rospy.is_shutdown():
        start_converter.publish('e_start')
        pub_target_pose.publish(target_pose)
        pub_tolerance.publish(tolerance)
        loop_rate.sleep()


def main():
    rospy.init_node('object_pose_mock_up')

    import thread
    try:
        thread.start_new_thread(create_window, tuple())

        publish_topics()
    except rospy.ROSInterruptException:
        pass
