#!/usr/bin/env python
"""
This module contains a component that measures the
distance between a list of current joint positions
and a list of target joint positions.

"""
#-*- encoding: utf-8 -*-

import numpy
import rospy
import std_msgs.msg
import sensor_msgs.msg
import mcr_manipulation_msgs.msg


class JointDistanceMeasurer(object):
    """
    Measures the distance between a list of current joint positions
    and a list of target joint positions.

    """
    def __init__(self):
        # params
        self.monitor_event = None
        self.target_joint_positions = None
        self.mapping_table = {}

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))
        # target joint names
        self.target_joint_names = rospy.get_param('~target_joint_names')

        # publishers
        self.joint_distances = rospy.Publisher(
            '~joint_distances', mcr_manipulation_msgs.msg.JointDistance
        )

        # subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~current_joint_values', sensor_msgs.msg.JointState,
                         self.current_joint_values_cb)
        rospy.Subscriber('~target_joint_values', sensor_msgs.msg.JointState,
                         self.target_joint_values_cb)

        # create a mapping table of the target joint names and their indices
        self.current_joint_positions = [None for _ in self.target_joint_names]
        for index, joint in enumerate(self.target_joint_names):
            self.mapping_table[joint] = index

    def event_in_cb(self, msg):
        """
        Obtains an event for the joint distance measurer.

        """
        self.monitor_event = msg.data

    def target_joint_values_cb(self, msg):
        """
        Obtains the target joint positions values of each joint.

        """
        self.target_joint_positions = msg

    def current_joint_values_cb(self, msg):
        """
        Obtains the current joint positions values of the desired joints.

        """
        for desired in self.target_joint_names:
            for i, current in enumerate(msg.name):
                if current == desired:
                    index = self.mapping_table[desired]
                    self.current_joint_positions[index] = msg.position[i]

    def start(self):
        """
        Starts the joint distance measurer.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'VALIDATION':
                state = self.validation_state()
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
        if self.target_joint_positions and not \
                numpy.equal(self.current_joint_positions, None).any():
            return 'VALIDATION'
        else:
            return 'INIT'

    def validation_state(self):
        """
        Executes the VALIDATION state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.target_joint_positions.name == self.target_joint_names:
            return 'IDLE'
        else:
            self.current_joint_positions = None
            self.target_joint_positions = None
            rospy.logwarn(
                "Target joint position names do not match the target joint names."
            )
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
            self.current_joint_positions = None
            self.target_joint_positions = None
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
            self.current_joint_positions = None
            self.target_joint_positions = None
            return 'INIT'
        else:
            self.publish_joint_distances()
            return 'RUNNING'

    def publish_joint_distances(self):
        """
        Publishes the joint distances between the current and
        target joint position values.

        """
        distances = measure_joint_distances(
            self.current_joint_positions,
            self.target_joint_positions.position
        )

        if distances:
            joint_distances = mcr_manipulation_msgs.msg.JointDistance()
            joint_distances.name = self.target_joint_positions.name
            joint_distances.distance = distances
            self.joint_distances.publish(joint_distances)


def measure_joint_distances(current, target):
    """
    Measures the difference between the current and target joint
    position values in a list of joints.

    :param current: The current joint position values.
    :type current: []Float

    :param target: The target joint position values.
    :type target: []Float

    :return: The distances between the current and target joint values.
    :rtype: []Float

    """
    current_positions = numpy.array(current)
    target_positions = numpy.array(target)

    distances = target_positions - current_positions
    joint_distances = distances.tolist()

    return joint_distances


def main():
    rospy.init_node('joint_distance_measurer', anonymous=True)
    joint_distance_measurer = JointDistanceMeasurer()
    joint_distance_measurer.start()

