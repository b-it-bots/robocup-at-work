#!/usr/bin/env python
"""
This module contains a GUI that allows to vary different spherical
sampler parameters on a topics.

"""
#-*- encoding: utf-8 -*-

import math
import threading
import Tkinter
import rospy
import mcr_manipulation_msgs.msg

LINEAR_RESOLUTION = 0.005               # in meters
MIN_DISTANCE = 0.0                      # in meters
MAX_DISTANCE = 0.3                      # in meters
ANGULAR_RESOLUTION = math.pi / 180.     # in radians
MIN_ORIENTATION = 0                     # in radians
MAX_ORIENTATION = 2 * math.pi           # in degrees

sampling_parameter = mcr_manipulation_msgs.msg.SphericalSamplerParameters()

global lock
lock = threading.Lock()


def create_window():
    """
    Creates a GUI window to publish a pose.

    """
    master = Tkinter.Tk()
    height_min_box = Tkinter.Scale(
        master, command=height_min, from_=MIN_DISTANCE, to=MAX_DISTANCE,
        resolution=LINEAR_RESOLUTION, label="Height (Minimum)"
    )
    height_min_box.grid(row=0, column=0)
    height_max_box = Tkinter.Scale(
        master, command=height_max, from_=MIN_DISTANCE, to=MAX_DISTANCE,
        resolution=LINEAR_RESOLUTION, label="Height (Maximum)"
    )
    height_max_box.grid(row=1, column=0)

    zenith_min_box = Tkinter.Scale(
        master, command=zenith_min, from_=MIN_ORIENTATION, to=MAX_ORIENTATION,
        resolution=ANGULAR_RESOLUTION, label="Zenith (Minimum)"
    )
    zenith_min_box.grid(row=0, column=1)
    height_max_box = Tkinter.Scale(
        master, command=zenith_max, from_=MIN_ORIENTATION, to=MAX_ORIENTATION,
        resolution=ANGULAR_RESOLUTION, label="Zenith (Maximum)"
    )
    height_max_box.grid(row=1, column=1)

    azimuth_min_box = Tkinter.Scale(
        master, command=azimuth_min, from_=MIN_ORIENTATION, to=MAX_ORIENTATION,
        resolution=ANGULAR_RESOLUTION, label="Azimuth (Minimum)"
    )
    azimuth_min_box.grid(row=0, column=2)
    azimuth_max_box = Tkinter.Scale(
        master, command=azimuth_max, from_=MIN_ORIENTATION, to=MAX_ORIENTATION,
        resolution=ANGULAR_RESOLUTION, label="Azimuth (Maximum)"
    )
    azimuth_max_box.grid(row=1, column=2)

    yaw_min_box = Tkinter.Scale(
        master, command=yaw_min, from_=MIN_ORIENTATION, to=MAX_ORIENTATION,
        resolution=ANGULAR_RESOLUTION, label="Yaw (Minimum)"
    )
    yaw_min_box.grid(row=0, column=3)
    yaw_max_box = Tkinter.Scale(
        master, command=yaw_max, from_=MIN_ORIENTATION, to=MAX_ORIENTATION,
        resolution=ANGULAR_RESOLUTION, label="Yaw (Maximum)"
    )
    yaw_max_box.grid(row=1, column=3)

    radial_min_box = Tkinter.Scale(
        master, command=radial_min, from_=MIN_DISTANCE, to=MAX_DISTANCE,
        resolution=LINEAR_RESOLUTION, label="Radial distance (Minimum)"
    )
    radial_min_box.grid(row=0, column=4)
    radial_max_box = Tkinter.Scale(
        master, command=radial_max, from_=MIN_DISTANCE, to=MAX_DISTANCE,
        resolution=LINEAR_RESOLUTION, label="Radial distance (Maximum)"
    )
    radial_max_box.grid(row=1, column=4)

    master.title("Sampling parameters GUI")
    master.mainloop()
    rospy.signal_shutdown("GUI closed")


def height_max(value):
    """
    Sets value as the maximum height of the sampling parameters.

    """
    global lock
    lock.acquire()
    sampling_parameter.height.maximum = float(value)
    lock.release()


def height_min(value):
    """
    Sets value as the minimum height of the sampling parameters.

    """
    global lock
    lock.acquire()
    sampling_parameter.height.minimum = float(value)
    lock.release()


def zenith_max(value):
    """
    Sets value as the maximum zenith of the sampling parameters.

    """
    global lock
    lock.acquire()
    sampling_parameter.zenith.maximum = float(value)
    lock.release()


def zenith_min(value):
    """
    Sets value as the minimum zenith of the sampling parameters.

    """
    global lock
    lock.acquire()
    sampling_parameter.zenith.minimum = float(value)
    lock.release()


def azimuth_max(value):
    """
    Sets value as the maximum azimuth of the sampling parameters.

    """
    global lock
    lock.acquire()
    sampling_parameter.azimuth.maximum = float(value)
    lock.release()


def azimuth_min(value):
    """
    Sets value as the minimum azimuth of the sampling parameters.

    """
    global lock
    lock.acquire()
    sampling_parameter.azimuth.minimum = float(value)
    lock.release()


def yaw_max(value):
    """
    Sets value as the maximum yaw of the sampling parameters.

    """
    global lock
    lock.acquire()
    sampling_parameter.yaw.maximum = float(value)
    lock.release()


def yaw_min(value):
    """
    Sets value as the minimum yaw of the sampling parameters.

    """
    global lock
    lock.acquire()
    sampling_parameter.yaw.minimum = float(value)
    lock.release()


def radial_max(value):
    """
    Sets value as the maximum radial distance of the sampling parameters.

    """
    global lock
    lock.acquire()
    sampling_parameter.radial_distance.maximum = float(value)
    lock.release()


def radial_min(value):
    """
    Sets value as the minimum radial distance of the sampling parameters.

    """
    global lock
    lock.acquire()
    sampling_parameter.radial_distance.minimum = float(value)
    lock.release()


def publish_pose():
    """
    Publishes the target pose.

    """
    # node cycle rate (in hz)
    loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

    # publishers
    pub_pose = rospy.Publisher(
        '~sampling_parameters', mcr_manipulation_msgs.msg.SphericalSamplerParameters,
        queue_size=1
    )

    while not rospy.is_shutdown():
        pub_pose.publish(sampling_parameter)
        loop_rate.sleep()


def main():
    rospy.init_node('target_pose_mock_up')

    import thread
    try:
        thread.start_new_thread(create_window, tuple())

        publish_pose()
    except rospy.ROSInterruptException:
        pass
