#!/usr/bin/env python
"""
This module contains a component that converts the
ROS-TF transform between two frames into a ROS pose message.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf


class TransformToPoseConverter(object):
    """
    Converts a transform, between a target frame with
    respect to a specified frame, into a pose.

    """
    def __init__(self):
        # params
        self.event = None
        self.reference_frame = rospy.get_param('~reference_frame', None)
        assert self.reference_frame is not None, "Reference frame must be defined."
        self.target_frame = rospy.get_param('~target_frame', None)
        assert self.target_frame is not None, "Target frame must be defined."

        self.listener = tf.TransformListener()

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # publishers
        self.converted_pose = rospy.Publisher(
            '~converted_pose', geometry_msgs.msg.PoseStamped, queue_size=1
        )

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)

    def start_transform_to_pose_converter(self):
        """
        Starts the transform to pose converter.

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
        Obtains an event for the transform to pose converter.

        """
        self.event = msg.data

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event = None
            return 'INIT'
        elif self.target_frame and self.reference_frame:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event = None
            return 'INIT'
        else:
            self.publish_converted_pose()
            self.event = None
            return 'IDLE'

    def publish_converted_pose(self):
        """
        Publishes the converted pose based on the transform
        between a target frame and the reference frame.

        """
        converted_pose = geometry_msgs.msg.PoseStamped()
        converted_pose.header.frame_id = self.reference_frame

        try:
            self.listener.waitForTransform(
                self.reference_frame, self.target_frame,
                rospy.Time(0), rospy.Duration(self.wait_for_transform)
            )

            (translation, rotation) = self.listener.lookupTransform(
                self.reference_frame, self.target_frame, rospy.Time(0)
            )

            converted_pose.pose.position.x = translation[0]
            converted_pose.pose.position.y = translation[1]
            converted_pose.pose.position.z = translation[2]
            converted_pose.pose.orientation.x = rotation[0]
            converted_pose.pose.orientation.y = rotation[1]
            converted_pose.pose.orientation.z = rotation[2]
            converted_pose.pose.orientation.w = rotation[3]

            converted_pose.header.stamp = self.listener.getLatestCommonTime(
                self.reference_frame, self.target_frame
            )

            self.converted_pose.publish(converted_pose)
        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))


def main():
    rospy.init_node('transform_to_pose_converter', anonymous=True)
    transform_to_pose_converter = TransformToPoseConverter()
    transform_to_pose_converter.start_transform_to_pose_converter()
