#!/usr/bin/env python
"""
This module contains a component that gets the
ROS-TF transform between two frames and publishes
as a static transform.
"""
#-*- encoding: utf-8 -*-
__author__ = 'shehzad'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf


class StaticTransformPublisher(object):
    """
    Publishes transform between a target frame with
    respect to a specified reference frame as a static transform.

    """
    def __init__(self):
        self.event = None
        self.static_transform_pose = None

        self.reference_frame = rospy.get_param('~reference_frame', None)
        assert self.reference_frame is not None, "Reference frame must be defined."

        self.target_frame = rospy.get_param('~target_frame', None)
        assert self.target_frame is not None, "Target frame must be defined."

        self.static_transform_frame = rospy.get_param('~static_transform_frame', None)
        assert self.static_transform_frame is not None, "Static transform frame must be defined."

        self.trigger_mode = rospy.get_param('~trigger_mode', False)

        self.listener = tf.TransformListener()
        self.tf_broadcast = tf.TransformBroadcaster()

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        
        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)

    def run(self):
        """
        Starts the transform to static transform pubslisher.

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
        Obtains an event for the static transform pubslisher.

        """
        self.event = msg.data

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            self.event = None
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
            self.static_transform_pose = self.compute_static_transform_pose()
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
            self.static_transform_pose = None
            return 'INIT'
        elif self.event == 'e_start':
            self.event = None
            self.static_transform_pose = None
            return 'IDLE'
        else:
            if self.static_transform_pose is not None:
                self.publish_static_transform()

            if self.trigger_mode:
               return 'INIT'

        return 'RUNNING'

    def compute_static_transform_pose(self):
        """
        Computes the converted pose based on the transform
        between a target frame and the reference frame.

        """
        transform_pose = geometry_msgs.msg.PoseStamped()
        transform_pose.header.frame_id = self.reference_frame

        try:
            self.listener.waitForTransform(
                self.reference_frame, self.target_frame,
                rospy.Time(0), rospy.Duration(self.wait_for_transform)
            )

            (translation, rotation) = self.listener.lookupTransform(
                self.reference_frame, self.target_frame, rospy.Time(0)
            )

            transform_pose.pose.position.x = translation[0]
            transform_pose.pose.position.y = translation[1]
            transform_pose.pose.position.z = translation[2]
            transform_pose.pose.orientation.x = rotation[0]
            transform_pose.pose.orientation.y = rotation[1]
            transform_pose.pose.orientation.z = rotation[2]
            transform_pose.pose.orientation.w = rotation[3]

            transform_pose.header.stamp = self.listener.getLatestCommonTime(
                self.reference_frame, self.target_frame
            )

        except tf.Exception, error:
            self.static_transform_pose = None
            rospy.logwarn("Exception occurred: {0}".format(error))

        return transform_pose

    def publish_static_transform(self):
        """
        Broadcast the transform in the reference frame 
        based on the static tranform pose.

        """

        pos = (
                self.static_transform_pose.pose.position.x,
                self.static_transform_pose.pose.position.y,
                self.static_transform_pose.pose.position.z
                )

        quat = (
                self.static_transform_pose.pose.orientation.x,
                self.static_transform_pose.pose.orientation.y,
                self.static_transform_pose.pose.orientation.z,
                self.static_transform_pose.pose.orientation.w
                )

        self.tf_broadcast.sendTransform(pos, quat,
                                        rospy.Time.now(),
                                        self.static_transform_frame,
                                        self.reference_frame)

def main():
    rospy.init_node('static_transform_publisher_node', anonymous=True)
    static_transform_publisher = StaticTransformPublisher()
    static_transform_publisher.run()
