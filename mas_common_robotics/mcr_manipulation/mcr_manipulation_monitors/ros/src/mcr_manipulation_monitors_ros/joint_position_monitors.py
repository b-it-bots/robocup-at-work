#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This module contains components that monitor
the joint position of a manipulator.
"""

import numpy
import rospy
import std_msgs.msg
import sensor_msgs.msg

__author__ = 'jsanch'


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

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))
        # tolerance for the joint positions
        self.epsilon = rospy.get_param('~epsilon')
        # target joint names
        self.target_joint_names = rospy.get_param('~target_joint_names')

        # create a mapping table of the target joint names and their indices
        self.current_joint_positions = [None for _ in self.target_joint_names]
        for index, joint in enumerate(self.target_joint_names):
            self.mapping_table[joint] = index

        # publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String)

        # subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~configuration", sensor_msgs.msg.JointState,
                         self.configure_joint_monitor_cb)
        rospy.Subscriber("~joint_states", sensor_msgs.msg.JointState,
                         self.read_joint_positions_cb)

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

    def start(self):
        """
        Starts the joint position monitor.

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
            self.event_out.publish('e_stopped')
            self.reset_component_data()
            return 'INIT'

        if numpy.all(self.current_joint_positions) is not None and \
                self.desired_joint_positions is not None:
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
            self.event_out.publish('e_stopped')
            self.reset_component_data()
            return 'INIT'

        if self.joint_positions_reached():
            self.event_out.publish('e_done')
            self.reset_component_data()
            return 'INIT'
        else:
            self.event_out.publish('e_configuration_not_reached')
            self.reset_component_data()

        return 'IDLE'

    def joint_positions_reached(self):
        """
        Checks if all the desired joint positions have been reached
        (within a tolerance).

        :return: True, if all the desired joint positions have reached
            their target values; otherwise False is returned.
        :rtype: bool

        """
        current_joint_positions = sensor_msgs.msg.JointState()
        current_joint_positions.name = [joint for joint in self.target_joint_names]
        current_joint_positions.position = [
            self.current_joint_positions[self.mapping_table[joint]]
            for joint in self.target_joint_names
        ]

        sorted_joint_positions = sort_joint_values(
            current_joint_positions, self.desired_joint_positions
        )

        return check_joint_positions(
            self.current_joint_positions, sorted_joint_positions, self.epsilon
        )

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.current_joint_positions = [None for _ in self.target_joint_names]
        self.desired_joint_positions = None
        self.monitor_event = None


def sort_joint_values(actual, reference):
    """
    Sorts a set of joint values matching the order of the 'actual'
    joint values.

    :param actual: The current joint positions.
    :type actual: sensor_msgs.msg.JointState

    :param reference: The reference joint positions.
    :type reference: sensor_msgs.msg.JointState

    :return: The joint positions values sorted with the order of the 'actual'
        joint values.
    :rtype: []Float

    """
    assert len(actual.name) == len(reference.name), \
        "The length of the current and reference joint must be the same."

    return [
        reference.position[ii] for a_name in actual.name
        for ii, r_name in enumerate(reference.name) if a_name == r_name
    ]


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
    for i, desired in enumerate(reference):
        below_limit = (actual[i] >= desired + tolerance)
        above_limit = (actual[i] <= desired - tolerance)
        if below_limit or above_limit:
            return False

    return True


def main():
    rospy.init_node("joint_position_monitors", anonymous=True)
    joint_position_monitor = JointPositionMonitor()
    joint_position_monitor.start()
