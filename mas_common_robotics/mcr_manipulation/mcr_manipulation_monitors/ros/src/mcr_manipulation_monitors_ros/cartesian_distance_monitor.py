#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This module contains a component that monitors if the
distance between two poses is within a specified tolerance.

"""

import math
import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf

__author__ = 'jsanch'


class CartesianDistanceMonitor(object):
    """
    Monitors if two poses are within a specified tolerance.

    """
    def __init__(self):
        # params
        self.event = None
        self.pose_1 = None
        self.pose_2 = None
        self.transformed_pose = None

        self.listener = tf.TransformListener()

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))
        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)
        # the minimum Euclidean distance to activate the monitor (in meters)
        self.tolerance = rospy.get_param('~tolerance', 0.03)

        # publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~pose_1', geometry_msgs.msg.PoseStamped, self.pose_1_cb)
        rospy.Subscriber('~pose_2', geometry_msgs.msg.PoseStamped, self.pose_2_cb)

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        """
        self.event = msg.data

    def pose_1_cb(self, msg):
        """
        Obtains the first pose.

        """
        self.pose_1 = msg

    def pose_2_cb(self, msg):
        """
        Obtains the second pose.

        """
        self.pose_2 = msg

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.pose_1 and self.pose_2:
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'RUNNING'
        elif self.event == 'e_stop':
            self.transformed_pose = None
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.transformed_pose = None
            return 'INIT'
        else:
            self.transformed_pose = self.transform_pose(self.pose_1, self.pose_2)

            if self.transformed_pose:
                self.monitor_euclidean_distance()

            return 'RUNNING'

    def transform_pose(self, reference_pose, target_pose):
        """
        Transforms the target pose into the frame of the reference pose.

        :param reference_pose: The reference pose.
        :type reference_pose: geometry_msgs.msg.PoseStamped

        :param target_pose: The current pose.
        :type target_pose: geometry_msgs.msg.PoseStamped

        :return: The target pose transformed to the frame of the reference pose.
        :rtype: geometry_msgs.msg.PoseStamped or None

        """
        try:
            self.listener.waitForTransform(
                target_pose.header.frame_id, reference_pose.header.frame_id,
                rospy.Time(0), rospy.Duration(self.wait_for_transform)
            )

            transformed_pose = self.listener.transformPose(
                reference_pose.header.frame_id, target_pose,
            )

            return transformed_pose
        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None

    def monitor_euclidean_distance(self):
        """
        Publishes an 'e_done' event if the Euclidean distance between
        two poses is lesser or equal than the specified threshold.

        """
        distance = calculate_euclidean_distance(
            self.pose_1.pose.position.x, self.pose_1.pose.position.y,
            self.pose_1.pose.position.z, self.transformed_pose.pose.position.x,
            self.transformed_pose.pose.position.y, self.transformed_pose.pose.position.z
        )

        if distance <= self.tolerance:
            self.event_out.publish('e_done')


def calculate_euclidean_distance(x_1, y_1, z_1, x_2, y_2, z_2):
    """
    Calculates the Euclidean distance between two points.

    :param x_1: The 'x' component of the first point.
    :type x_1: float

    :param y_1: The 'y' component of the first point.
    :type y_1: float

    :param z_1: The 'z' component of the first point.
    :type z_1: float

    :param x_2: The 'x' component of the second point.
    :type x_2: float

    :param y_2: The 'y' component of the second point.
    :type y_2: float

    :param z_2: The 'z' component of the second point.
    :type z_2: float

    :return: The Euclidean distance between two points.
    :rtype: float

    """
    distance = math.sqrt((x_2 - x_1)**2 + (y_2 - y_1)**2 + (z_2 - z_1)**2)

    return distance


def main():
    rospy.init_node('cartesian_distance_monitor', anonymous=True)
    cartesian_distance_monitor = CartesianDistanceMonitor()
    cartesian_distance_monitor.start()
