#!/usr/bin/env python
"""
This module contains components that monitor
the joint position of a manipulator.

"""
#-*- encoding: utf-8 -*-
__author__ = 'jsanch'

import rospy
import std_msgs.msg
import sensor_msgs.msg


class JointPositionMonitor(object):
    """
    Monitors the joint positions of a manipulator.

    """
    def __init__(self):
        # params
        self.monitor_event = None
        self.desired_joint_positions = None
        self.current_joint_positions = None
        self.mapping_table = {}

        # node cycle rate (in seconds)
        self.loop_rate = rospy.get_param('~loop_rate')
        # tolerance for the joint positions
        self.epsilon = rospy.get_param('~epsilon')
        # target joint names
        self.target_joint_names = rospy.get_param('~target_joint_names')

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~configuration", sensor_msgs.msg.JointState,
                         self.configure_joint_monitor_cb)
        rospy.Subscriber("~joint_states", sensor_msgs.msg.JointState,
                         self.read_joint_positions_cb)

        # create a mapping table of the target joint names and their indices
        self.current_joint_positions = [None for _ in self.target_joint_names]
        for index, joint in enumerate(self.target_joint_names):
            self.mapping_table[joint] = index

    def start_joint_position_monitor(self):
        """
        Starts the joint position monitor.

        """
        rospy.loginfo("Joint position monitor ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                if self.current_joint_positions and \
                   self.desired_joint_positions:
                    state = 'IDLE'
                    rospy.logdebug("Joint position monitor: state=" + str(state))
            elif state == 'IDLE':
                if self.monitor_event == 'e_start':
                    state = 'RUNNING'
                    rospy.logdebug("Joint position monitor: event e_start, state=" + str(state))
            elif state == 'RUNNING':
                if check_joint_positions(self.current_joint_positions,
                                         self.desired_joint_positions.position,
                                         self.epsilon):
                    rospy.logdebug("Joint position monitor finished: state=" + str(state))
                    self.event_out.publish('e_done')

                if self.monitor_event == 'e_stop':
                    self.current_joint_positions = None
                    self.desired_joint_positions = None
                    state = 'INIT'
                    rospy.logdebug("Joint position monitor: event e_done, state=" + str(state))

            rospy.sleep(self.loop_rate)

    def event_in_cb(self, msg):
        """
        Obtains an event for the joint position monitor.

        """
        self.monitor_event = msg.data

    def read_joint_positions_cb(self, msg):
        """
        Obtains the current joint positions values of the desired joints.

        """
        for desired in self.target_joint_names:
            for i, current in enumerate(msg.name):
                if current == desired:
                    index = self.mapping_table[desired]
                    self.current_joint_positions[index] = msg.position[i]

    def configure_joint_monitor_cb(self, msg):
        """
        Obtains the desired joint positions.

        """
        self.desired_joint_positions = msg


def check_joint_positions(actual, reference, tolerance):
    """
    Detects if all joints have reached their desired
    joint position within a tolerance.

    :param actual: The current joint positions.
    :type actual: []Float

    :param reference: The reference joint positions.
    :type reference: []Float

    :param tolerance: The tolerance allowed between
                      reference and actual joint positions.
    :type tolerance: Float

    :return: The status of all joints in their desired positions.
    :rtype: Boolean

    """
    try:
        for i, desired in enumerate(reference):
            below_limit = (actual[i] >= desired + tolerance)
            above_limit = (actual[i] <= desired - tolerance)

        if below_limit or above_limit:
            return False

    except IndexError:
        rospy.logwarn(
            "The length of the current and reference joint positions do not match."
        )
        return False

    return True


def main():
    rospy.init_node("joint_position_monitors", anonymous=True)
    joint_position_monitor = JointPositionMonitor()
    joint_position_monitor.start_joint_position_monitor()
