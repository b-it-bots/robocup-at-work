#!/usr/bin/env python
"""
This module contains a component that calculates
the component-wise error between two poses.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import mcr_manipulation_msgs.msg
import tf


class ComponentWisePoseErrorCalculator(object):
    """
    Calculates the error between two poses in three
    linear components and three angular components.

    """
    def __init__(self):
        # params
        self.monitor_event = None
        self.pose_1 = None
        self.pose_2 = None
        self.listener = tf.TransformListener()

        # node cycle rate (in seconds)
        self.loop_rate = rospy.get_param('~loop_rate')
        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # publishers
        self.pose_error = rospy.Publisher(
            '~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference
        )

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~pose_1', geometry_msgs.msg.PoseStamped, self.pose_1_cb)
        rospy.Subscriber('~pose_2', geometry_msgs.msg.PoseStamped, self.pose_2_cb)

    def start(self):
        """
        Starts the component-wise pose error calculator.

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
            rospy.sleep(self.loop_rate)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component-wise pose error calculator.

        """
        self.monitor_event = msg.data

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
        if self.pose_2 and self.pose_1:
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_start':
            return 'RUNNING'
        elif self.monitor_event == 'e_stop':
            self.pose_1 = None
            self.pose_2 = None
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_stop':
            self.pose_1 = None
            self.pose_2 = None
            return 'INIT'
        else:
            transformed_pose = self.transform_pose(self.pose_1, self.pose_2)

            if transformed_pose:
                pose_error = calculate_component_wise_pose_error(
                    self.pose_1, transformed_pose
                )

                self.pose_error.publish(pose_error)

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


def calculate_component_wise_pose_error(current_pose, target_pose):
    """
    Calculates the component-wise error between two 'PoseStamped' objects.
    It assumes that both poses are specified with respect of the same
    reference frame.

    :param current_pose: The current pose.
    :type current_pose: geometry_msgs.msg.PoseStamped

    :param target_pose: The target pose.
    :type target_pose: geometry_msgs.msg.PoseStamped

    :return: The difference in the six components
    (three linear and three angular).
    :rtype: mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference

    """
    current_quaternion = []
    target_quaternion = []
    error = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
    error.header.frame_id = current_pose.header.frame_id

    point_dimensions = ['x', 'y', 'z']
    quaternion_dimensions = ['x', 'y', 'z', 'w']

    # calculate linear distances
    for _, dim in enumerate(point_dimensions):
        difference = getattr(target_pose.pose.position, dim) - \
            getattr(current_pose.pose.position, dim)
        setattr(error.linear, dim, difference)

    # convert quaternions into roll, pitch, yaw angles
    for _, dim in enumerate(quaternion_dimensions):
        current_quaternion.append(getattr(current_pose.pose.orientation, dim))
        target_quaternion.append(getattr(target_pose.pose.orientation, dim))

    current_angles = tf.transformations.euler_from_quaternion(current_quaternion)
    target_angles = tf.transformations.euler_from_quaternion(target_quaternion)

    # calculate angular distances
    for i, (target, current) in enumerate(
            zip(target_angles, current_angles)
    ):
        difference = target - current
        setattr(error.angular, point_dimensions[i], difference)

    return error


def main():
    rospy.init_node('component_wise_pose_error_calculator_node', anonymous=True)
    component_wise_pose_error_calculator = ComponentWisePoseErrorCalculator()
    component_wise_pose_error_calculator.start()
