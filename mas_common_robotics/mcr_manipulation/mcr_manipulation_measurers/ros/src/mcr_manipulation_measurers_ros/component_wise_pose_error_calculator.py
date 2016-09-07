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

        # linear offset applied to the result (a three-element list)
        self.linear_offset = rospy.get_param('~linear_offset', None)
        if self.linear_offset is not None:
            assert (
                isinstance(self.linear_offset, list) and len(self.linear_offset) == 3
            ), "If linear offset is specified, it must be a three-dimensional array."

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # how long to wait for transform (in seconds)
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # publishers
        self.pose_error = rospy.Publisher(
            '~pose_error', mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference, queue_size=5
        )
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=5)

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
            self.loop_rate.sleep()

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
        if self.monitor_event == 'e_start':
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        elif self.pose_1 and self.pose_2:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.monitor_event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            transformed_pose = self.transform_pose(self.pose_1, self.pose_2)

            if transformed_pose:
                pose_error = calculate_component_wise_pose_error(
                    self.pose_1, transformed_pose, self.linear_offset
                )

                self.pose_error.publish(pose_error)
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

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
            target_pose.header.stamp = self.listener.getLatestCommonTime(
                target_pose.header.frame_id, reference_pose.header.frame_id
            )

            self.listener.waitForTransform(
                target_pose.header.frame_id, reference_pose.header.frame_id,
                target_pose.header.stamp, rospy.Duration(self.wait_for_transform)
            )

            transformed_pose = self.listener.transformPose(
                reference_pose.header.frame_id, target_pose,
            )

            return transformed_pose

        except tf.Exception, error:
            rospy.logwarn("Exception occurred: {0}".format(error))
            return None

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.monitor_event = None
        self.pose_1 = None
        self.pose_2 = None


def calculate_component_wise_pose_error(current_pose, target_pose, offset=None):
    """
    Calculates the component-wise error between two 'PoseStamped' objects.
    It assumes that both poses are specified with respect of the same
    reference frame.

    :param current_pose: The current pose.
    :type current_pose: geometry_msgs.msg.PoseStamped

    :param target_pose: The target pose.
    :type target_pose: geometry_msgs.msg.PoseStamped

    :param offset: A linear offset in X, Y, Z.
    :type offset: list

    :return: The difference in the six components
    (three linear and three angular).
    :rtype: mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference

    """
    error = mcr_manipulation_msgs.msg.ComponentWiseCartesianDifference()
    error.header.frame_id = current_pose.header.frame_id

    # calculate linear distances
    error.linear.x = target_pose.pose.position.x - current_pose.pose.position.x
    error.linear.y = target_pose.pose.position.y - current_pose.pose.position.y
    error.linear.z = target_pose.pose.position.z - current_pose.pose.position.z

    current_quaternion = [
        current_pose.pose.orientation.x, current_pose.pose.orientation.y,
        current_pose.pose.orientation.z, current_pose.pose.orientation.w
    ]
    target_quaternion = [
        target_pose.pose.orientation.x, target_pose.pose.orientation.y,
        target_pose.pose.orientation.z, target_pose.pose.orientation.w
    ]

    # convert quaternions into roll, pitch, yaw angles
    current_angles = tf.transformations.euler_from_quaternion(current_quaternion)
    target_angles = tf.transformations.euler_from_quaternion(target_quaternion)

    # calculate angular distances
    error.angular.x = target_angles[0] - current_angles[0]
    error.angular.y = target_angles[1] - current_angles[1]
    error.angular.z = target_angles[2] - current_angles[2]

    if offset is not None:
        offset = tuple(offset)
        error.linear.x += offset[0]
        error.linear.y += offset[1]
        error.linear.z += offset[2]

    return error


def main():
    rospy.init_node('component_wise_pose_error_calculator_node', anonymous=True)
    component_wise_pose_error_calculator = ComponentWisePoseErrorCalculator()
    component_wise_pose_error_calculator.start()
